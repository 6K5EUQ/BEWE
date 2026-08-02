#include "host_state.hpp"
#include "fft_viewer.hpp"
#include "channel.hpp"
#include "bewe_paths.hpp"
#include "json_scan.hpp"
#include "module_api.hpp"   // 디코드 모듈 on/off 상태 영속화 (bewe_modules / host_mask)
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <errno.h>

namespace HostState {

std::string file_path(const std::string& station){
    std::string s = station.empty() ? "default" : station;
    for(char& c : s) if(c=='/' || c=='\\') c='_';  // 경로 구분자 방지
    return BEWEPaths::data_dir() + "/host_state_" + s + ".json";
}

static void json_escape(std::string& out, const char* s){
    for(; *s; s++){
        char c = *s;
        if(c=='"' || c=='\\'){ out += '\\'; out += c; }
        else if((unsigned char)c < 0x20) {}
        else out += c;
    }
}

// Holding(범위 밖) 중엔 보존된 mode 가 진짜 의도된 mode.
static int effective_mode(const Channel& ch){
    if(ch.dem_paused.load() && ch.dem_paused_mode != Channel::DM_NONE)
        return (int)ch.dem_paused_mode;
    return (int)ch.mode;
}

void save(const FFTViewer& v, const std::string& station){
    char buf[512];
    float cf = (float)(v.header.center_frequency / 1e6);
    float sr = (float)(v.header.sample_rate / 1e6);
    // DF 설정. Kraken 백엔드가 아니어도 저장한다 — 설정을 잃으면 다음에
    // --sdr kraken 으로 뜰 때 배열 반경부터 다시 넣어야 한다.
    PktDfConfig dfc{}; v.df_get_cfg(dfc);
    std::string out;
    snprintf(buf, sizeof(buf),
        "{\n  \"cf_mhz\":%.6f,\n  \"sr_msps\":%.6f,\n  \"gain_db\":%.2f,\n"
        "  \"df_radius_m\":%.6f,\n  \"df_heading_deg\":%.2f,\n  \"df_elements\":%d,\n"
        "  \"df_algo\":%d,\n  \"df_sense\":%d,\n  \"df_avg_frames\":%d,\n  \"df_signal_dim\":%d,\n"
        "  \"df_snr_thr_db\":%.2f,\n  \"df_dc_guard_hz\":%.1f,\n  \"df_c_papr\":%.2f,\n"
        "  \"df_target_looks\":%d,\n  \"df_fft_size\":%d,\n  \"df_max_frames\":%d,\n"
        "  \"df_enable_control\":%d,\n  \"df_array_type\":%d,\n",
        cf, sr, v.gain_db,
        (double)dfc.radius_m, (double)dfc.heading_deg, (int)dfc.elements, (int)dfc.algo,
        (int)dfc.sense, (int)dfc.avg_frames, (int)dfc.signal_dim,
        (double)dfc.snr_thr_db, (double)dfc.dc_guard_hz, (double)dfc.c_papr,
        (int)dfc.target_looks, (int)dfc.fft_size, (int)dfc.max_frames,
        (int)dfc.enable_control, (int)dfc.array_type);
    out += buf;

    // 소자 좌표. 파서가 배열을 안 읽으므로 소자마다 키를 하나씩 쓴다 (최대 8개라
    // 장황해도 감당된다). 구버전 파일엔 이 키가 없고, 그러면 좌표가 0 으로 남아
    // df::Config::geom() 이 프리셋에서 되만든다.
    for(int m = 0; m < 8; m++){
        snprintf(buf, sizeof(buf), "  \"df_ex%d\":%.6f,\n  \"df_ey%d\":%.6f,\n",
                 m, (double)dfc.elem_x[m], m, (double)dfc.elem_y[m]);
        out += buf;
    }
    out += "  \"channels\": [\n";

    bool first = true;
    for(int i=0;i<MAX_CHANNELS;i++){
        const Channel& ch = v.channels[i];
        if(!ch.filter_active) continue;
        if(!first) out += ",\n";
        first = false;
        char own[33]={}; strncpy(own, ch.owner, 31);
        std::string oesc; json_escape(oesc, own);
        bool det_on = ch.det_on.load();
        // det_on 이면 mode 는 항상 NONE 으로 저장한다 — lock 중이었다면 ch.mode 가
        // AM/FM 으로 바뀌어 있는데(신호 잡았을 때 뭘로 복조할지 결정한 값일 뿐),
        // 그걸 그대로 저장하면 재시작 시 "디텍션이었다"는 사실이 사라지고 고정
        // AM/FM 채널로 굳어버린다 (이 함수가 고치는 버그). det_s/det_e 는 탐색 대역
        // 원본 — lock 중엔 ch.s/e 가 좁아져 있으므로 별도로 보존해야 한다.
        snprintf(buf, sizeof(buf),
            "    {\"s\":%.6f,\"e\":%.6f,\"mode\":%d,\"audio_mask\":%u,\"pan\":%d,\"sq\":%.2f,"
            "\"sq_manual\":%d,\"det_on\":%d,\"det_s\":%.6f,\"det_e\":%.6f,\"owner\":\"",
            det_on ? ch.det_s : ch.s, det_on ? ch.det_e : ch.e,
            det_on ? (int)Channel::DM_NONE : effective_mode(ch),
            (unsigned)ch.audio_mask.load(), ch.pan, ch.sq_threshold.load(),
            ch.sq_manual.load() ? 1 : 0,
            det_on ? 1 : 0, ch.det_s, ch.det_e);
        out += buf;
        out += oesc;
        // 이 채널에 켜진 디코드 모듈 id (host_mask 비트) — 재시작 시 디코드 재개용
        std::string dmods;
        for(const auto& m : bewe_modules()){
            if(!m.target_modes) continue;
            if(bewe_mod_host_mask(m.id) & (1ull<<i)){
                if(!dmods.empty()) dmods += ',';
                dmods += m.id;
            }
        }
        out += "\",\"decode_mods\":\"";
        out += dmods;          // 모듈 id 는 안전 문자 (escaping 불필요)
        out += "\"}";
    }
    out += "\n  ],\n";

    // 노치 — 순수 대역 목록. EMA 상태(lo_lvl 등)는 렌더가 매 프레임 다시 잡으므로
    // 저장하지 않는다.
    out += "  \"notches\": [\n";
    {
        std::lock_guard<std::mutex> lk(const_cast<FFTViewer&>(v).notches_mtx);
        bool nfirst = true;
        int written = 0;
        for(const auto& n : v.notches){
            if(written >= MAX_NOTCHES) break;
            if(!nfirst) out += ",\n";
            nfirst = false;
            snprintf(buf, sizeof(buf), "    {\"lo\":%.6f,\"hi\":%.6f}",
                     n.freq_lo_mhz, n.freq_hi_mhz);
            out += buf;
            written++;
        }
    }
    out += "\n  ]\n}\n";

    std::string dir = BEWEPaths::data_dir();
    mkdir(dir.c_str(), 0755);
    std::string path = file_path(station);
    std::string tmp  = path + ".tmp";
    FILE* fp = fopen(tmp.c_str(), "w");
    if(!fp){ fprintf(stderr,"[HostState] save: cannot open %s errno=%d\n", tmp.c_str(), errno); return; }
    size_t wrote = fwrite(out.data(), 1, out.size(), fp);
    bool ok = (wrote == out.size());
    if(fflush(fp) != 0) ok = false;
    if(fclose(fp) != 0) ok = false;
    if(!ok){  // 디스크 풀 등 부분 기록 — 기존 good 파일 보존, 빈 파일로 덮지 않음
        fprintf(stderr,"[HostState] save: write incomplete, keeping old %s\n", path.c_str());
        remove(tmp.c_str());
        return;
    }
    rename(tmp.c_str(), path.c_str());  // 원자적 교체 — 부분 기록 파일 복원 방지
}

Snapshot load(const std::string& station){
    Snapshot st;
    std::string path = file_path(station);
    FILE* fp = fopen(path.c_str(), "r");
    if(!fp) return st;  // 파일 없음 → ok=false
    fseek(fp,0,SEEK_END); long sz = ftell(fp); fseek(fp,0,SEEK_SET);
    if(sz <= 0){ fclose(fp); return st; }
    std::string body(sz, 0);
    if(fread(&body[0], 1, sz, fp) != (size_t)sz){ fclose(fp); return st; }
    fclose(fp);

    JScan js{body.data(), body.data()+body.size()};
    if(!js.consume('{')) return st;
    std::string key;
    while(js.read_key(key)){
        if(key=="cf_mhz"){ double d=0; js.read_number(d); st.cf_mhz=(float)d; }
        else if(key=="sr_msps"){ double d=0; js.read_number(d); st.sr_msps=(float)d; }
        else if(key=="gain_db"){ double d=0; js.read_number(d); st.gain_db=(float)d; st.has_gain=true; }
        else if(key=="df_radius_m"){    double d=0; js.read_number(d); st.df.radius_m=(float)d;        st.has_df=true; }
        else if(key=="df_heading_deg"){ double d=0; js.read_number(d); st.df.heading_deg=(float)d;     st.has_df=true; }
        else if(key=="df_elements"){    double d=0; js.read_number(d); st.df.elements=(uint8_t)d;   st.has_df=true; }
        else if(key=="df_algo"){        double d=0; js.read_number(d); st.df.algo=(uint8_t)d;       st.has_df=true; }
        else if(key=="df_sense"){       double d=0; js.read_number(d); st.df.sense=(uint8_t)d;      st.has_df=true; }
        else if(key=="df_avg_frames"){  double d=0; js.read_number(d); st.df.avg_frames=(uint8_t)d; st.has_df=true; }
        else if(key=="df_signal_dim"){  double d=0; js.read_number(d); st.df.signal_dim=(uint8_t)d; st.has_df=true; }
        else if(key=="df_snr_thr_db"){  double d=0; js.read_number(d); st.df.snr_thr_db=(float)d;      st.has_df=true; }
        else if(key=="df_dc_guard_hz"){ double d=0; js.read_number(d); st.df.dc_guard_hz=(float)d;     st.has_df=true; }
        else if(key=="df_c_papr"){      double d=0; js.read_number(d); st.df.c_papr=(float)d;          st.has_df=true; }
        else if(key=="df_target_looks"){double d=0; js.read_number(d); st.df.target_looks=(uint16_t)d; st.has_df=true; }
        else if(key=="df_fft_size"){    double d=0; js.read_number(d); st.df.fft_size=(uint16_t)d;     st.has_df=true; }
        else if(key=="df_max_frames"){  double d=0; js.read_number(d); st.df.max_frames=(uint8_t)d;    st.has_df=true; }
        else if(key=="df_enable_control"){double d=0;js.read_number(d);st.df.enable_control=(uint8_t)d;st.has_df=true; }
        else if(key=="df_array_type"){  double d=0; js.read_number(d); st.df.array_type=(uint8_t)d;    st.has_df=true; }
        else if(key.size()==6 && key.compare(0,5,"df_ex")==0 && key[5]>='0' && key[5]<='7'){
            double d=0; js.read_number(d); st.df.elem_x[key[5]-'0']=(float)d;                          st.has_df=true; }
        else if(key.size()==6 && key.compare(0,5,"df_ey")==0 && key[5]>='0' && key[5]<='7'){
            double d=0; js.read_number(d); st.df.elem_y[key[5]-'0']=(float)d;                          st.has_df=true; }
        else if(key=="notches"){
            if(!js.consume('[')) break;
            while(!js.peek(']')){
                if(!js.consume('{')) break;
                NotchSnap n;
                std::string k;
                while(js.read_key(k)){
                    if(k=="lo"){ double d=0; js.read_number(d); n.lo=(float)d; }
                    else if(k=="hi"){ double d=0; js.read_number(d); n.hi=(float)d; }
                    else { double d; js.read_number(d); }
                    if(js.peek('}')) break;
                }
                js.consume('}');
                if(st.n_notches < MAX_NOTCHES && n.hi > n.lo)
                    st.notches[st.n_notches++] = n;
            }
            js.consume(']');
        }
        else if(key=="channels"){
            if(!js.consume('[')) break;
            while(!js.peek(']')){
                if(!js.consume('{')) break;
                ChanSnap c;
                std::string k;
                while(js.read_key(k)){
                    if(k=="s"){ double d=0; js.read_number(d); c.s=(float)d; }
                    else if(k=="e"){ double d=0; js.read_number(d); c.e=(float)d; }
                    else if(k=="mode"){ double d=0; js.read_number(d); c.mode=(int)d; }
                    else if(k=="audio_mask"){ double d=0; js.read_number(d); c.audio_mask=(uint32_t)d; }
                    else if(k=="pan"){ double d=0; js.read_number(d); c.pan=(int)d; }
                    else if(k=="sq"){ double d=0; js.read_number(d); c.sq=(float)d; }
                    else if(k=="sq_manual"){ double d=0; js.read_number(d); c.sq_manual=(d!=0); }
                    else if(k=="owner"){ std::string s; js.read_string(s); strncpy(c.owner, s.c_str(), 31); }
                    else if(k=="decode_mods"){ std::string s; js.read_string(s); strncpy(c.decode_mods, s.c_str(), sizeof(c.decode_mods)-1); }
                    else if(k=="det_on"){ double d=0; js.read_number(d); c.det_on=(d!=0); }
                    else if(k=="det_s"){ double d=0; js.read_number(d); c.det_s=(float)d; }
                    else if(k=="det_e"){ double d=0; js.read_number(d); c.det_e=(float)d; }
                    else {  // 미지 키 — 문자열/숫자/배열/객체 모두 스킵
                        if(js.peek('"')){ std::string t; js.read_string(t); }
                        else if(js.consume('[') || js.consume('{')){
                            int depth=1;
                            while(js.p<js.end && depth>0){
                                if(*js.p=='['||*js.p=='{') depth++;
                                else if(*js.p==']'||*js.p=='}') depth--;
                                js.p++;
                            }
                        } else { double d; js.read_number(d); }
                    }
                    if(js.peek('}')) break;
                }
                js.consume('}');
                if(st.n_chans < MAX_CHANNELS && c.e > c.s)
                    st.chans[st.n_chans++] = c;
            }
            js.consume(']');
        } else {
            // 미지 키 스킵
            if(js.peek('"')){ std::string t; js.read_string(t); }
            else if(js.consume('[') || js.consume('{')){
                int depth=1;
                while(js.p<js.end && depth>0){
                    if(*js.p=='['||*js.p=='{') depth++;
                    else if(*js.p==']'||*js.p=='}') depth--;
                    js.p++;
                }
            } else { double d; js.read_number(d); }
        }
    }
    st.ok = true;
    return st;
}

static inline uint64_t mix(uint64_t h, uint64_t x){
    h ^= x + 0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2);
    return h;
}
static inline uint64_t f2u(float f){ uint32_t u; memcpy(&u,&f,4); return u; }

uint64_t fingerprint(const FFTViewer& v){
    uint64_t h = 1469598103934665603ULL;
    h = mix(h, (uint64_t)v.header.center_frequency);
    h = mix(h, (uint64_t)v.header.sample_rate);
    h = mix(h, f2u(v.gain_db));
    {   // DF 설정이 바뀌면 저장되게 fingerprint 에 넣는다. (packed 32B → u64 4회 mix)
        PktDfConfig d{}; v.df_get_cfg(d);
        static_assert(sizeof(d) % 8 == 0, "PktDfConfig mix assumes 8-byte multiple");
        uint64_t w[sizeof(d)/8]; memcpy(w, &d, sizeof(d));
        for(size_t i = 0; i < sizeof(d)/8; i++) h = mix(h, w[i]);
    }
    for(int i=0;i<MAX_CHANNELS;i++){
        const Channel& ch = v.channels[i];
        if(!ch.filter_active) continue;
        h = mix(h, (uint64_t)i);
        h = mix(h, f2u(ch.s));
        h = mix(h, f2u(ch.e));
        h = mix(h, (uint64_t)effective_mode(ch));
        h = mix(h, (uint64_t)ch.audio_mask.load());
        h = mix(h, (uint64_t)(uint32_t)ch.pan);
        h = mix(h, f2u(ch.sq_threshold.load()));
        for(int k=0;k<32 && ch.owner[k]; k++) h = mix(h, (uint64_t)(unsigned char)ch.owner[k]);
    }
    {   // 노치 추가/삭제도 저장 트리거
        std::lock_guard<std::mutex> lk(const_cast<FFTViewer&>(v).notches_mtx);
        for(const auto& n : v.notches){ h = mix(h, f2u(n.freq_lo_mhz)); h = mix(h, f2u(n.freq_hi_mhz)); }
    }
    // 디코드 모듈 on/off 변경도 감지 → 토글 시 저장 트리거
    for(const auto& m : bewe_modules()){
        if(!m.target_modes) continue;
        h = mix(h, (uint64_t)bewe_mod_host_mask(m.id));
    }
    return h;
}

void apply_channels(FFTViewer& v, const Snapshot& st){
    for(int i=0;i<st.n_chans && i<MAX_CHANNELS;i++){
        const ChanSnap& c = st.chans[i];
        int slot = i;
        v.stop_dem(slot);
        v.channels[slot].reset_slot();
        v.channels[slot].s = c.s;
        v.channels[slot].e = c.e;
        v.channels[slot].filter_active = true;
        strncpy(v.channels[slot].owner, c.owner, 31);
        v.channels[slot].audio_mask.store(c.audio_mask);
        v.channels[slot].pan = c.pan;
        v.channels[slot].sq_threshold.store(c.sq);
        // 수동 조정값이면 그대로 확정(재캘리브 제외). 자동값이면 sq_calibrated=false 로 두어
        // 다음 autoscale/캘리브에서 현재 노이즈플로어로 다시 잡히게 한다.
        v.channels[slot].sq_manual.store(c.sq_manual);
        v.channels[slot].sq_calibrated.store(c.sq_manual);
        v.local_ch_out[slot] = 3;
        if(c.det_on && c.det_e > c.det_s){
            // 디텍션 필터로 무장 상태 복원 — arm_detect(slot,true) 와 동일 절차.
            // baseline 은 재시작 시점 대역 상태로 새로 잡는다 (이전 baseline 은 무의미).
            Channel& ch = v.channels[slot];
            ch.s = c.det_s; ch.e = c.det_e;      // lock 중 저장이어도 탐색 대역 전체로 복귀
            ch.det_s = c.det_s; ch.det_e = c.det_e;
            ch.det_hold = 0; ch.det_ext_cnt = 0;
            ch.det_locked.store(false);
            ch.det_on.store(true);
            ch.det_base_reset();
            ch.mode = Channel::DM_NONE;
            float t = c.sq;
            if(!(t >= DET_MARGIN_MIN_DB && t <= DET_MARGIN_MAX_DB)) t = DET_MARGIN_DEF_DB;
            ch.sq_threshold.store(t);
            ch.sq_calibrated.store(true);
            ch.sq_calib_cnt = 0;
        } else {
            Channel::DemodMode dm = (c.mode>=0 && c.mode<=2) ? (Channel::DemodMode)c.mode
                                                             : Channel::DM_NONE;
            v.channels[slot].mode = dm;
            if(dm != Channel::DM_NONE) v.start_dem(slot, dm);
        }
        // 저장된 디코드 모듈 재개 (필터처럼 복원 — 재시작 후에도 디코드 유지).
        // LOCAL 경로 → host_apply_set → 워커 재기동 + host_mask 복구 + STATE 브로드캐스트.
        if(c.decode_mods[0]){
            char tmp[64]; strncpy(tmp, c.decode_mods, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0;
            for(char* tok=strtok(tmp,","); tok; tok=strtok(nullptr,","))
                bewe_mod_set_target(v, tok, bewe_mod_my_station(), slot, true);
        }
    }
    // 범위 밖 채널은 Holding 으로 (복조 중이면 mode 보존 + stop)
    v.update_dem_by_freq(v.header.center_frequency/1e6f);
}

} // namespace HostState

namespace HostState {
void apply_df(FFTViewer& v, const Snapshot& st){
    if(!st.ok || !st.has_df) return;   // 구버전 파일 — 기본값을 그대로 둔다
    // 구 파일엔 없던 필드는 0 으로 남는다. 0 이 유효하지 않은 항목만 되살린다.
    PktDfConfig d = st.df;
    if(d.elements == 0)     d.elements = 5;
    if(d.avg_frames == 0)   d.avg_frames = 3;
    if(d.max_frames < d.avg_frames) d.max_frames = 12;
    if(d.signal_dim == 0)   d.signal_dim = 1;
    if(d.target_looks == 0) d.target_looks = 2048;
    if(d.fft_size == 0)     d.fft_size = 8192;
    if(d.dc_guard_hz <= 0)  d.dc_guard_hz = 2000.0f;
    if(d.c_papr <= 0)       d.c_papr = 30.0f;
    if(d.radius_m <= 0)     d.radius_m = 0.175f;
    // DAQ 제어는 항상 켠다 (v15.4.2 부터 끄는 수단이 없다). 저장돼 있던 0 도
    // 되돌린다 — 0 이면 주파수축 재튠도 게인 변경도 조용히 무시돼 원인을 못 찾는다.
    d.enable_control = 1;
    const_cast<Snapshot&>(st).df = d;
    v.df_set_cfg(st.df);
}

void apply_notches(FFTViewer& v, const Snapshot& st){
    if(!st.ok || st.n_notches <= 0) return;
    std::lock_guard<std::mutex> lk(v.notches_mtx);
    v.notches.clear();
    for(int i=0;i<st.n_notches;i++){
        FFTViewer::NotchFilter n;
        n.freq_lo_mhz = st.notches[i].lo;
        n.freq_hi_mhz = st.notches[i].hi;
        v.notches.push_back(n);   // EMA 상태는 기본값 — 첫 프레임에 스스로 잡는다
    }
    bewe_log_push(0,"[HostState] restored %d notch(es)\n", st.n_notches);
}
} // namespace HostState
