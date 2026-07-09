// ── GUARD 모듈 본체: 등록, ais_guard 데몬 확보, 경보 tail → framework 방출 ────
// 채널 타깃형 아님(target_modes=0) — AIS 디코드가 켜진 HOST 에서 ais_module 이
// guard_mod::host_ensure() 를 불러 활성화. Python(ais_guard 데몬)이 판단·경보를
// alerts_live.jsonl 에 append → 여기서 250ms 폴링 tail → GuardWireMsg 로 emit.
// JOIN 은 on_data 에서 aid 기준 upsert (CLEAR 는 active=false 마킹, 기록 보존).
#include "guard_meta.hpp"
#include "module_api.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

namespace guard_mod {

static std::mutex            g_mtx;
static std::vector<AlertRow> g_log;      // aid 별 최신상태 (upsert; 시간순 append)
constexpr int LOG_MAX = 20000;

// ── 경로 (계약 고정) ───────────────────────────────────────────────────────
static std::string guard_data_dir(){ return BEWEPaths::data_dir()+"/BEAE/ais/data/guard"; }
static std::string alerts_path(){ return guard_data_dir()+"/alerts_live.jsonl"; }
static std::string venv_python(){ return BEWEPaths::data_dir()+"/BEAE/ais/.venv/bin/python"; }
static std::string ais_pkg_dir(){ return BEWEPaths::data_dir()+"/BEAE/ais"; }
static std::string pid_path(){ return guard_data_dir()+"/guard.pid"; }   // Python 데몬이 단일 소유(기록/회수) — C++은 읽기만

// ── JSON 미니 파서 (ais_module.cpp jll/jf/jstr 스타일; 외부 라이브러리 금지) ──
static bool jstr(const char* l,const char* k,char* o,size_t cap){
    const char* p=strstr(l,k); if(!p) return false; p+=strlen(k);
    size_t i=0; while(*p&&*p!='"'&&i+1<cap){ if(*p=='\\'){ ++p; if(*p=='n'){ o[i++]='\n'; ++p; continue; } if(*p) o[i++]=*p++; continue; } o[i++]=*p++; } o[i]=0; return true;
}
static long long jll(const char* l,const char* k){ const char* p=strstr(l,k); return p?atoll(p+strlen(k)):0; }
static double    jf (const char* l,const char* k){ const char* p=strstr(l,k); return p?atof (p+strlen(k)):0.0; }

// alerts_live.jsonl 한 줄 → GuardAlert (파이프 계약 v1 스키마)
static bool parse_alert_line(const char* l, GuardAlert& a){
    a = GuardAlert{};
    a.t_ms =jll(l,"\"t\":");
    a.aid  =(uint32_t)jll(l,"\"aid\":");
    a.typ  =(uint8_t)jll(l,"\"typ\":");
    a.sev  =(uint8_t)jll(l,"\"sev\":");
    a.state=(uint8_t)jll(l,"\"st\":");
    a.mmsi =(uint32_t)jll(l,"\"mmsi\":");
    a.mmsi2=(uint32_t)jll(l,"\"mmsi2\":");
    a.lat=(float)jf(l,"\"lat\":"); a.lon=(float)jf(l,"\"lon\":");
    a.score=(float)jf(l,"\"score\":");
    if(strstr(l,"\"cpa\":"))  a.cpa_m =(float)jf(l,"\"cpa\":");    // 없으면 -1(n/a) 유지
    if(strstr(l,"\"tcpa\":")) a.tcpa_s=(float)jf(l,"\"tcpa\":");
    jstr(l,"\"msg\":\"",a.msg,sizeof(a.msg));
    jstr(l,"\"reco\":\"",a.reco,sizeof(a.reco));
    return a.t_ms>0 && a.aid!=0 && a.typ>=1 && a.typ<=7   // 6=위험구역 7=데모항적 (보조)
        && a.sev>=1 && a.sev<=3 && a.state>=1 && a.state<=3;
}

// station_id ("DGS-2_DGS-2") → 표시명 ("DGS-2") — ais_module 과 동일
static void station_disp(const char* sid, char* out, size_t cap){
    size_t o=0;
    for(const char* p=sid; *p && *p!='_' && o+1<cap; ++p) out[o++]=*p;
    if(o==0 && cap>5){ strncpy(out,"LOCAL",cap-1); out[cap-1]=0; return; }
    out[o]=0;
}

// ── 일 단위 JSONL 아카이브 (Central 자동push 프레임워크 대상 경로) ───────────
static std::string store_dir(){ return BEWEPaths::data_dir() + "/modules/guard"; }
static std::string store_path(int64_t t_ms){
    struct tm tmv; KST::to_tm((time_t)(t_ms/1000), tmv);
    char d[16]; strftime(d,sizeof(d),"%Y%m%d",&tmv);
    return store_dir() + "/guard_" + d + ".jsonl";
}
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
static void store_append(const GuardAlert& a){
    mkdir((BEWEPaths::data_dir()+"/modules").c_str(),0755);
    mkdir(store_dir().c_str(),0755);
    FILE* f=fopen(store_path(a.t_ms).c_str(),"ab"); if(!f) return;
    char ms[200], rc[260]; json_escape(a.msg,ms,sizeof(ms)); json_escape(a.reco,rc,sizeof(rc));
    fprintf(f,"{\"v\":1,\"t\":%lld,\"aid\":%u,\"typ\":%u,\"sev\":%u,\"st\":%u,"
              "\"mmsi\":%u,\"mmsi2\":%u,\"lat\":%.6f,\"lon\":%.6f,\"score\":%.1f,"
              "\"cpa\":%.1f,\"tcpa\":%.1f,\"msg\":\"%s\",\"reco\":\"%s\"}\n",
        (long long)a.t_ms,a.aid,(unsigned)a.typ,(unsigned)a.sev,(unsigned)a.state,
        a.mmsi,a.mmsi2,a.lat,a.lon,a.score,a.cpa_m,a.tcpa_s,ms,rc);
    fclose(f);
}

// ── 보조 오버레이 조립 (typ=6 폴리곤 / typ=7 항적) ───────────────────────────
// 꼭짓점은 reco 에 "lat,lon;lat,lon;..." 텍스트 패킹. 긴 경로는 청크 분할:
// mmsi=청크 idx, mmsi2=총 청크 수, 이름(msg)은 청크 0 에만. state=3(CLEAR)=제거.
static std::map<uint32_t, guard_mod::OverlayPath>            g_overlays;   // 조립 완료 (aid 키)
static std::map<uint32_t, std::map<uint32_t,std::string>>   g_ovchunks;   // aid → idx → 꼭짓점 텍스트
static std::map<uint32_t, guard_mod::OverlayPath>            g_ovmeta;     // aid → 메타(typ/kind/name)

static void overlay_ingest(const GuardAlert& a){
    std::lock_guard<std::mutex> lk(g_mtx);
    if(a.state==3){ g_overlays.erase(a.aid); g_ovchunks.erase(a.aid); g_ovmeta.erase(a.aid); return; }
    uint32_t idx=a.mmsi, total=a.mmsi2? a.mmsi2 : 1;
    auto& meta = g_ovmeta[a.aid];
    meta.aid=a.aid; meta.typ=a.typ; meta.kind=a.sev;
    if(idx==0 && a.msg[0]){ strncpy(meta.name, a.msg, sizeof(meta.name)-1); meta.name[sizeof(meta.name)-1]=0; }
    g_ovchunks[a.aid][idx] = a.reco;
    if(g_ovchunks[a.aid].size() < total) return;          // 청크 미완 — 대기
    // 전 청크 도착 → idx 순 이어붙여 파싱
    guard_mod::OverlayPath p = meta;
    for(uint32_t i=0;i<total;i++){
        auto it=g_ovchunks[a.aid].find(i); if(it==g_ovchunks[a.aid].end()) return;
        const char* s=it->second.c_str();
        if(*s=='M'){                                          // "M<mmsi>;" — 항적 소유 MMSI (경보 연결)
            p.mmsi=(uint32_t)strtoul(s+1,nullptr,10);
            const char* sc=strchr(s,';'); s = sc? sc+1 : s+strlen(s);
        }
        while(*s){
            char* e=nullptr;
            float la=strtof(s,&e); if(e==s||*e!=',') break;
            s=e+1; float lo=strtof(s,&e); if(e==s) break;
            p.ll.push_back(la); p.ll.push_back(lo);
            s = (*e==';')? e+1 : e;
        }
    }
    if(p.ll.size()>=4) g_overlays[a.aid]=std::move(p);    // 최소 2점
    g_ovchunks.erase(a.aid);
}

// ── 내부 log upsert (aid 기준; CLEAR 는 active=false 마킹, 기록 보존) ─────────
static void upsert(const GuardAlert& a){
    std::lock_guard<std::mutex> lk(g_mtx);
    for(int i=(int)g_log.size()-1; i>=0; --i){
        if(g_log[i].a.aid != a.aid) continue;
        g_log[i].a = a;                             // 같은 사건 갱신 (UPDATE/CLEAR)
        g_log[i].active = (a.state != 3);
        return;
    }
    if((int)g_log.size() >= LOG_MAX) g_log.erase(g_log.begin());
    g_log.push_back(AlertRow{a, a.state != 3});
}

// ── 뷰 접근자 (ais_view 오버레이 / guard_view 가 사용) ───────────────────────
std::vector<AlertRow> snapshot(){
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_log;
}
int active_count(){
    std::lock_guard<std::mutex> lk(g_mtx);
    int n=0;
    for(const auto& r : g_log)
        if(r.active && r.a.typ<5) n++;              // 보고(5)/보조(6,7)는 경보 아님
    return n;
}
bool vessel_alert(uint32_t mmsi, uint8_t& typ, uint8_t& sev){
    if(mmsi==0) return false;
    std::lock_guard<std::mutex> lk(g_mtx);
    bool found=false; uint8_t bt=0, bs=0;
    for(const auto& r : g_log){
        if(!r.active || r.a.typ>=5) continue;
        if(r.a.mmsi!=mmsi && r.a.mmsi2!=mmsi) continue;
        if(!found || r.a.sev>bs){ bt=r.a.typ; bs=r.a.sev; found=true; }
    }
    if(found){ typ=bt; sev=bs; }
    return found;
}
std::vector<OverlayPath> overlays(){
    std::lock_guard<std::mutex> lk(g_mtx);
    std::vector<OverlayPath> out; out.reserve(g_overlays.size());
    for(const auto& kv : g_overlays) out.push_back(kv.second);
    return out;
}

// ── JOIN/뷰어: 데이터 수신 → upsert ─────────────────────────────────────────
static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v;
    if(n < sizeof(GuardWireMsg)) return;
    GuardWireMsg w; memcpy(&w, d, sizeof(w));
    GuardAlert a; guard_wire_to_msg(w, a);
    bewe_mod_stat_bump("guard", station, 0, a.t_ms);   // 채널 없음 → ch 0 고정
    station_disp(station, a.station, sizeof(a.station));
    if(a.typ>=6) overlay_ingest(a);                    // 위험구역/데모항적 → 오버레이 조립
    else         upsert(a);                            // 경보/보고 → log
}

// ── HOST: ais_guard 데몬 확보 (ais_ai.cpp ai_ensure_daemon 패턴) ─────────────
// venv python 존재 시에만 spawn. pidfile 로 이중기동 방지(systemd 등 외부 데몬 공존).
static pid_t g_daemon_pid = -1;

static bool pidfile_alive(){                 // pidfile 의 pid 가 살아있나 (외부 데몬 감지)
    FILE* f=fopen(pid_path().c_str(),"rb"); if(!f) return false;
    long pid=0; int ok=fscanf(f,"%ld",&pid); fclose(f);
    return ok==1 && pid>1 && kill((pid_t)pid,0)==0;
}
static void stop_daemon(){                   // BEWE 종료 시 자식 데몬 정리 (자기 spawn 분만)
    if(g_daemon_pid>0){
        kill(g_daemon_pid, SIGTERM);
        waitpid(g_daemon_pid, nullptr, 0);
        g_daemon_pid=-1;
    }
}
static void ensure_daemon(){
    if(g_daemon_pid>0){                      // 이미 spawn 한 자식 — 살아있나 확인
        if(waitpid(g_daemon_pid, nullptr, WNOHANG)==0) return;   // 살아있음
        g_daemon_pid=-1;                     // 죽었음 → 수확 완료, 재spawn 진행
    }
    if(pidfile_alive()) return;              // 외부(systemd) 데몬이 이미 동작 → spawn 불필요
    std::string py=venv_python(), cwd=ais_pkg_dir();
    if(access(py.c_str(), X_OK)!=0) return;  // venv 없음(개발PC 등) → 완전 no-op
    static bool s_atexit=false;              // 최초 spawn 시 1회 등록
    if(!s_atexit){ atexit(stop_daemon); s_atexit=true; }
    pid_t pid=fork();
    if(pid<0) return;
    if(pid==0){                              // 자식: cwd=BEAE/ais 로 -m ais_guard daemon
        setsid();                            // 세션 분리 (BEWE 종료 시그널 전파 격리)
        if(chdir(cwd.c_str())!=0) _exit(127);
        int nul=open("/dev/null",O_RDONLY); if(nul>=0){ dup2(nul,0); if(nul>0) close(nul); }
        execl(py.c_str(), py.c_str(), "-m", "ais_guard", "daemon", (char*)nullptr);
        _exit(127);
    }
    g_daemon_pid=pid;                        // 부모: pid 기억 (종료 시 kill, 좀비 수확용)
    // pidfile 은 Python 데몬(_write_pidfile)이 단일 소유자로 기록·회수 — 여기서 쓰지 않음
    // (이중기동 방지는 자식 self-check + 위 pidfile_alive 읽기로 충분)
}

// ── HOST: alerts_live.jsonl 폴링 tail → store + emit ─────────────────────────
// inotify 없이 250ms 폴링. 오프셋 유지, 로테이션/truncate(size<off) 안전.
// 최초 발견 시 파일 끝부터 tail — BEWE 재기동 시 과거 경보 재방출(중복 store) 방지.
static FFTViewer*        g_v = nullptr;
static std::atomic<bool> g_tail_on{false};

static void tail_loop(){
    const std::string path = alerts_path();
    long off = -1;                           // -1 = 아직 파일 못 봄 (첫 발견 시 끝부터)
    std::string acc;                         // 줄 경계 걸친 부분줄 보관
    while(g_tail_on.load(std::memory_order_relaxed)){
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        struct stat st{};
        if(stat(path.c_str(), &st)!=0){ off=-1; acc.clear(); continue; }   // 파일 없음/로테이션
        if(off<0) off=(long)st.st_size;
        if((long)st.st_size < off){ off=0; acc.clear(); }   // truncate/재생성 → 처음부터
        if((long)st.st_size == off) continue;
        FILE* f=fopen(path.c_str(),"rb"); if(!f) continue;
        if(fseek(f, off, SEEK_SET)!=0){ fclose(f); continue; }
        char buf[8192]; size_t r;
        while((r=fread(buf,1,sizeof(buf),f))>0) acc.append(buf,r);
        off=(long)ftell(f);
        fclose(f);
        size_t nl;
        while((nl=acc.find('\n'))!=std::string::npos){
            std::string line=acc.substr(0,nl); acc.erase(0,nl+1);
            if(line.size()<8) continue;
            GuardAlert a;
            if(!parse_alert_line(line.c_str(), a)) continue;
            if(a.typ<=5) store_append(a);    // 경보/보고만 일 아카이브 (보조 6/7 은 재발행성 — 제외)
            GuardWireMsg w; guard_msg_to_wire(a, w);
            if(g_v) bewe_mod_emit(*g_v, "guard", &w, sizeof(w));   // 로컬 뷰는 on_data 로 반영
        }
    }
}

// ais_module 의 host_start(g_mgmt 락 내부, 단일스레드 시점)가 호출 — idempotent.
void host_ensure(FFTViewer& v){
    static std::mutex s_mtx;
    std::lock_guard<std::mutex> lk(s_mtx);
    g_v = &v;
    mkdir((BEWEPaths::data_dir()+"/BEAE").c_str(),0755);
    mkdir((BEWEPaths::data_dir()+"/BEAE/ais").c_str(),0755);
    mkdir((BEWEPaths::data_dir()+"/BEAE/ais/data").c_str(),0755);
    mkdir(guard_data_dir().c_str(),0755);
    ensure_daemon();
    if(!g_tail_on.load()){
        g_tail_on.store(true);
        std::thread(tail_loop).detach();     // 프로세스 수명 데몬 스레드 (250ms sleep 폴링)
    }
}

// ── 모듈 등록 (static-init) ────────────────────────────────────────────────
static bool s_registered = [](){
    BeweModule m{};
    m.id    = "guard";
    m.label = "GUARD";
    m.planned = false;
    m.target_modes = 0;                      // 채널 타깃형 아님 (AIS 디코드에 편승)
    // draw_content 없음 — 별도 탭 대신 모든 경보/구역/항적을 AIS 지도에 오버레이
    m.on_data = &on_data;
    bewe_register_module(m);
    return true;
}();

} // namespace guard_mod
