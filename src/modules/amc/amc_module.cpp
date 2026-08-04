// ── AMC(자동 변조분류) 모듈 등록 + 워커 슬롯 + 로그/아카이브 ────────────────
//
// 다른 해독 모듈과 다른 점이 하나 있다: **운용자가 채널마다 켜 주지 않는다.**
// 채널을 detect 필터로 만들면 그 순간 자동으로 붙는다. 그 자동무장을 host_poll
// (module_api.hpp) 에서 하고, 실제 시작·정지는 기존 bewe_mod_set_target 경로를
// 그대로 태운다 — host_start / host_mask / ring 공급 / JOIN [RUN] 표시가 전부
// 공짜로 따라온다.
#include "fft_viewer.hpp"
#include "amc_module.hpp"
#include "amc_ai.hpp"
#include "module_api.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <sys/stat.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace amc_mod {

std::mutex             mtx;
std::vector<AmcRecord> log;
char                   filter[64] = {};

// ── 워커 슬롯 ──────────────────────────────────────────────────────────────
struct ChWork {
    std::atomic<bool>   on{false};
    std::atomic<bool>   stop{false};
    std::atomic<size_t> rp{0};
    std::atomic<bool>   manual{false};   // 'a' 키 요청 (워커가 소비)
    std::thread         thr;
};
static ChWork g_w[MAX_CHANNELS];
static std::mutex g_mgmt;

std::atomic<size_t>& worker_rp(int ch){ return g_w[ch].rp; }
bool worker_stop_req(int ch){ return g_w[ch].stop.load(std::memory_order_relaxed); }
bool worker_take_manual(int ch){ return g_w[ch].manual.exchange(false, std::memory_order_acq_rel); }
void worker_post_manual(int ch){
    if(ch>=0 && ch<MAX_CHANNELS) g_w[ch].manual.store(true, std::memory_order_release);
}
void worker_natural_exit(FFTViewer& v, int ch){
    g_w[ch].on.store(false);
    bewe_mod_host_mask_clear(v, "amc", ch);
}

static bool host_start(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return false;
    std::lock_guard<std::mutex> lk(g_mgmt);
    ChWork& w = g_w[ch];
    if(!w.on.load() && w.thr.joinable()) w.thr.join();
    if(w.on.load()) return true;
    if(!v.channels[ch].filter_active) return false;   // 복조 모드 무관 — 워커가 IQ ring 직접 탭
    // 데몬 기동은 반드시 여기서 (g_mgmt 락 안 = 단일스레드 시점). 워커 스레드에서
    // fork 하면 다른 스레드가 쥔 락이 자식에 복제돼 자식이 그대로 굳는다.
    amc_ai_ensure_daemon();
    w.stop.store(false);
    w.on.store(true);
    w.thr = std::thread(worker, std::ref(v), ch);
    return true;
}
static void host_stop(FFTViewer& v, int ch){
    (void)v;
    if(ch<0 || ch>=MAX_CHANNELS) return;
    std::lock_guard<std::mutex> lk(g_mgmt);
    ChWork& w = g_w[ch];
    if(w.on.load()){
        w.stop.store(true);
        if(w.thr.joinable()) w.thr.join();
        w.on.store(false);
    } else if(w.thr.joinable()) w.thr.join();
}
static void on_ch_stop(FFTViewer& v, int ch){
    if((bewe_mod_host_mask("amc")>>ch)&1){
        host_stop(v, ch);
        bewe_mod_host_mask_clear(v, "amc", ch);
    }
}

// ── HOST 주기 훅 (~1Hz) ────────────────────────────────────────────────────
// 무장은 운용자가 채널 행의 AMC 버튼으로 한다 (기본 꺼짐). 여기서는 이 기지가
// 실제로 분류를 할 수 있는지(venv/모델 설치 여부)만 알린다 — JOIN 이 그걸 받아
// 버튼을 파랑(가능)/빨강(불가)으로 그린다. 안 그러면 눌러도 아무 일이 없는데
// 이유를 알 수 없다.
static void host_poll(FFTViewer& v){
    (void)v;
    bewe_mod_set_avail("amc", amc_ai_enabled());   // 변화 있을 때만 방송된다
}

// ── 수동 실행 ('a' 키) ─────────────────────────────────────────────────────
// JOIN 이면 HOST 로 요청을 보내고, HOST/LOCAL 이면 워커 플래그를 직접 세운다.
// 어느 쪽이든 여기서 추론하지 않는다 — UI/net 스레드다.
static void on_manual(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    if(!v.channels[ch].filter_active) return;
    // 무장 안 된 채널(=detect 아님)에서 눌러도 재 볼 수 있게 한다. 운용자가 굳이
    // 지목한 것이므로 detect 여부로 막지 않는다 — 다만 워커가 없으면 의미가 없다.
    if(v.remote_mode){
        bewe_mod_rec_request("amc", bewe_mod_my_station(), ch, AMC_MANUAL_MAGIC);
        return;
    }
    worker_post_manual(ch);
}
// HOST: JOIN 이 보낸 수동 요청 수신 (net 스레드 — 플래그만).
static void on_rec_req(FFTViewer& v, int ch, uint64_t rec_id){
    (void)v;
    if(rec_id != AMC_MANUAL_MAGIC) return;
    worker_post_manual(ch);
}

// station_id("DGS-2_DGS-2") → 표시명("DGS-2"). 빈 문자열이면 LOCAL.
static void station_disp(const char* sid, char* out, size_t cap){
    size_t o=0;
    for(const char* p=sid; *p && *p!='_' && o+1<cap; ++p) out[o++]=*p;
    if(o==0 && cap>5){ strncpy(out,"LOCAL",cap-1); out[cap-1]=0; return; }
    out[o]=0;
}

// ── 표시 로그 append ───────────────────────────────────────────────────────
void append_log(const AmcRecord& m){
#ifdef BEWE_HEADLESS
    (void)m; return;  // CLI: 뷰 없음 — 표시 로그 RAM 미적재
#else
    std::lock_guard<std::mutex> lk(mtx);
    int n=(int)log.size();
    // 히스토리/라이브 경계 중복 방어 (같은 시각·채널·클래스면 같은 건으로 본다)
    for(int i=n-1;i>=0 && i>=n-64;i--)
        if(log[i].t_ms==m.t_ms && log[i].ch==m.ch && log[i].cls==m.cls) return;
    if(n>=LOG_MAX) log.erase(log.begin());
    log.push_back(m);
#endif
}

// ── 일단위 JSONL 아카이브 (HOST) ───────────────────────────────────────────
static void json_escape(const char* s, char* out, size_t cap){
    size_t o=0;
    for(const char* p=s; *p && o+6<cap; ++p){
        unsigned char c=(unsigned char)*p;
        if(c=='"'||c=='\\'){ out[o++]='\\'; out[o++]=(char)c; }
        else if(c<0x20){ o+=snprintf(out+o,cap-o,"\\u%04x",c); }
        else out[o++]=(char)c;
    }
    out[o]=0;
}
static std::string store_dir(){ return BEWEPaths::data_dir()+"/modules/amc"; }
static std::string store_path(int64_t t_ms){
    struct tm tmv; KST::to_tm((time_t)(t_ms/1000), tmv);
    char d[16]; strftime(d,sizeof(d),"%Y%m%d",&tmv);
    return store_dir()+"/amc_"+d+".jsonl";
}
void store_append(const AmcRecord& m){
    mkdir((BEWEPaths::data_dir()+"/modules").c_str(),0755);
    mkdir(store_dir().c_str(),0755);
    FILE* f=fopen(store_path(m.t_ms).c_str(),"ab"); if(!f) return;
    char mv[24]; json_escape(m.model,mv,sizeof(mv));
    fprintf(f,"{\"t\":%lld,\"ch\":%d,\"f\":%.4f,\"bw\":%.2f,\"cls\":%d,\"cf\":%.4f,"
              "\"cls2\":%d,\"cf2\":%.4f,\"snr\":%.1f,\"tg\":%d,\"mv\":\"%s\"}\n",
        (long long)m.t_ms,m.ch,m.freq,m.bw_khz,m.cls,m.conf,
        m.cls2,m.conf2,m.snr_db,(int)m.trig,mv);
    fclose(f);
}
bool store_read_today(std::string& out){
    int64_t now=(int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    FILE* f=fopen(store_path(now).c_str(),"rb"); if(!f) return false;
    char buf[8192]; size_t n;
    while((n=fread(buf,1,sizeof(buf),f))>0) out.append(buf,n);
    fclose(f); return true;
}
static bool jstr(const char* l,const char* k,char* o,size_t cap){
    const char* p=strstr(l,k); if(!p) return false; p+=strlen(k);
    size_t i=0; while(*p&&*p!='"'&&i+1<cap){ if(*p=='\\'){ ++p; if(*p) o[i++]=*p++; continue; } o[i++]=*p++; } o[i]=0; return true;
}
static long long jll(const char* l,const char* k){ const char* p=strstr(l,k); return p?atoll(p+strlen(k)):0; }
static double    jf (const char* l,const char* k){ const char* p=strstr(l,k); return p?atof (p+strlen(k)):0.0; }
void store_parse_jsonl(const char* data, size_t n, std::vector<AmcRecord>& out){
    size_t i=0; std::string line;
    while(i<n){
        size_t e=i; while(e<n && data[e]!='\n') e++;
        line.assign(data+i,e-i); i=e+1;
        if(line.size()<8) continue;
        const char* l=line.c_str();
        AmcRecord m{};
        m.t_ms  = jll(l,"\"t\":");
        m.ch    = (int)jll(l,"\"ch\":");
        m.freq  = (float)jf(l,"\"f\":");
        m.bw_khz= (float)jf(l,"\"bw\":");
        m.cls   = (int)jll(l,"\"cls\":");
        m.conf  = (float)jf(l,"\"cf\":");
        m.cls2  = (int)jll(l,"\"cls2\":");
        m.conf2 = (float)jf(l,"\"cf2\":");
        m.snr_db= (float)jf(l,"\"snr\":");
        m.trig  = (uint8_t)jll(l,"\"tg\":");
        jstr(l,"\"mv\":\"",m.model,sizeof(m.model));
        if(m.t_ms) out.push_back(m);
    }
}

// ── 워커 → 방출 ───────────────────────────────────────────────────────────
void host_emit(FFTViewer& v, AmcRecord m){
    if(!m.t_ms)
        m.t_ms=(int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    store_append(m);
    AmcWire w; amc_to_wire(m, w);
    bewe_mod_emit(v, "amc", &w, sizeof(w));   // Central 전송 + 로컬 on_data
}

// ── 수신 (JOIN/로컬) ───────────────────────────────────────────────────────
static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v;
    if(n < sizeof(AmcWire)) return;
    AmcRecord m; amc_from_wire(*reinterpret_cast<const AmcWire*>(d), m);
    bewe_mod_stat_bump("amc", station, m.ch, m.t_ms);
    station_disp(station, m.station, sizeof(m.station));
    append_log(m);
}

// ── 과거조회(Hist) ─────────────────────────────────────────────────────────
static std::vector<AmcRecord> g_stash;
static void log_stash(){
    std::lock_guard<std::mutex> lk(mtx);
    g_stash = std::move(log); log.clear();
}
static void log_restore(){
    std::lock_guard<std::mutex> lk(mtx);
    log = std::move(g_stash); g_stash.clear();
}
static void on_hist_file(const char* station, const char* data, size_t n){
    std::vector<AmcRecord> recs;
    store_parse_jsonl(data, n, recs);
    std::lock_guard<std::mutex> lk(mtx);
    for(auto& m : recs){
        station_disp(station, m.station, sizeof(m.station));
        log.push_back(m);
    }
}

// ── 등록 ──────────────────────────────────────────────────────────────────
static bool s_reg = [](){
    BeweModule m{};
    m.id    = "amc";
    m.label = "AMC";
    m.planned = false;
    // detect 채널은 unlock 상태에서 DM_NONE, lock 되면 AM/FM 이 된다 — 셋 다 받아야
    // 그 전이 동안 계속 붙어 있다. (target_modes=0 으로 두면 안 된다: 프레임워크의
    // 아카이브 push 와 reconcile 이 !target_modes 를 걸러 버린다.)
    m.target_modes = (uint8_t)((1u<<Channel::DM_NONE)|(1u<<Channel::DM_AM)|(1u<<Channel::DM_FM));
    // 0 고정. 비0이면 코어가 detect 폭을 이 규격폭으로 고정해 버려서, 우리가 재려는
    // "검출된 실제 폭"(AmcRecord.bw_khz)이 의미를 잃는다.
    m.spec_bw_hz   = 0.0f;
#ifndef BEWE_HEADLESS
    m.draw_content = &draw_content;
    m.ch_btn       = "AMC";      // 채널 행 DET 오른쪽 버튼
    m.panel        = "Automatic Modulation Classification";
    m.draw_panel   = &draw_panel;
#endif
    m.host_poll  = &host_poll;
    m.host_start = &host_start;
    m.host_stop  = &host_stop;
    m.on_ch_stop = &on_ch_stop;
    m.on_manual  = &on_manual;
    m.on_rec_req = &on_rec_req;
    m.on_data    = &on_data;
    m.log_stash    = &log_stash;
    m.log_restore  = &log_restore;
    m.on_hist_file = &on_hist_file;
    bewe_register_module(m);
    return true;
}();

} // namespace amc_mod
