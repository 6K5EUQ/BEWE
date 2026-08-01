// ── DF API 의 JOIN arm (GUI 전용) ─────────────────────────────────────────
// GUI 빌드는 JOIN 전용이라 kraken_io.cpp 와 src/df/* 를 링크하지 않는다. 그런데
// fft_viewer.hpp:986-1035 가 선언한 DF API 는 df_view.cpp 와 ui.cpp 가 그대로
// 호출한다 — DF 설정 패널·DF LED·숫자키 측정 요청은 JOIN 의 정상 기능이기
// 때문이다. 그 심볼들을 여기서 JOIN 관점으로 채운다.
//
// 원본은 kraken_io.cpp 의 `if(net_cli)` arm 이다. 로컬 DAQ 가 없는 쪽만 남겼으므로
// 엔진에 묻던 것(ready/measuring/live/pump)은 전부 "없음" 으로 답한다.
//
// 설계의 핵심은 kraken_io.cpp:670-673 주석과 같다 — **JOIN 은 자기 DF 설정을
// 갖지 않는다.** 읽을 때는 HOST 가 방송한 정본을, 쓸 때는 HOST 로 요청만 보낸다.
// 그래야 여러 JOIN 이 동시에 만져도 한 값으로 수렴하고, 측정을 실제로 하는
// 쪽(HOST)의 설정과 화면이 어긋나지 않는다.
//
// 이 파일은 df/ 를 include 하지 않는다 (그러면 GUI 가 다시 DF 엔진에 컴파일
// 의존한다). PktDfConfig 같은 plain 타입만 오간다.

#include "fft_viewer.hpp"
#include "net_client.hpp"
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstring>

// ── 로컬 DF 엔진이 없다는 사실을 그대로 답하는 것들 ──────────────────────
// JOIN 에서 초록/측정중을 흉내내면 운용자가 자기 PC 가 재는 줄 안다. HOST 의
// 상태는 df_link_state() 가 하트비트로 따로 알려준다.
bool FFTViewer::df_engine_ready() const { return false; }
bool FFTViewer::df_measuring()    const { return false; }
void FFTViewer::df_stop_engine()        {}

bool FFTViewer::df_submit(double, double, int, int, bool){ return false; }

// HOST 가 방송한 DF_RESULT 를 소비한다. 세대 카운터를 보고 변화가 있을 때만
// 옮기므로 패널이 닫혀 있어도 쌓이지 않는다 (한 칸 슬롯 = 항상 최신).
// 적용은 fft_viewer.cpp 의 df_apply_result 한 곳이 한다 — HOST arm 과 같은 함수라
// 두 경로가 갈라질 수 없다.
void FFTViewer::df_pump(){
    if(!net_cli) return;
    const uint32_t s = net_cli->df_res_seq.load(std::memory_order_acquire);
    if(s == df_res_seen) return;
    df_res_seen = s;
    PktDfResult r;
    uint8_t q[360];
    {
        std::lock_guard<std::mutex> lk(net_cli->df_res_mtx);
        r = net_cli->df_res;
        memcpy(q, net_cli->df_res_spec, sizeof q);
    }
    df_apply_result(r, q);
}

// JOIN 은 HOST 의 DF 를 원격으로 쓴다. HOST 가 하트비트로 실제 가용도를 보내주므로
// (0=불가 1=가능 2=준비중) 이 함수의 반환 규약(0=down 1=calibrating 2=streaming)
// 으로 옮겨 담는다. 원본: kraken_io.cpp:443-455.
int FFTViewer::df_link_state() const {
    if(!net_cli) return 0;
    if(!net_cli->is_connected()) return 0;
    const uint8_t d = net_cli->remote_df_state.load();
    return (d == 1) ? 2 : (d == 2 ? 1 : 0);
}

// HOST 의 heimdall 판독을 DF_STATUS(0x61) 로 받아 그대로 옮긴다. 예전엔 여기서
// 빈 구조체를 돌려줬고, 그래서 DF 패널의 좌측 열·λ/모호비 줄·측정 진행바·소자수
// 불일치 경고가 전부 JOIN 에서 죽어 있었다 — 거절 이유를 설명하는 값들이 정작
// 조작하는 사람 화면에 없었다.
//
// 6초보다 낡으면 통째로 0 을 돌려준다. 죽은 HOST 의 마지막 상태가 계속 초록으로
// 남아 있으면 안 된다 (안전 측 실패).
void FFTViewer::df_get_live(FFTViewer::DFLive& o) const {
    o = FFTViewer::DFLive{};
    if(!net_cli) return;
    const int64_t t = net_cli->df_stat_ms.load(std::memory_order_acquire);
    if(t == 0) return;
    const int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();
    if(now - t > 6000) return;

    PktDfStatus s;
    { std::lock_guard<std::mutex> lk(net_cli->df_stat_mtx); s = net_cli->df_stat; }
    o.link = s.link;  o.usable = (s.usable != 0);
    o.sync_state = s.sync_state; o.delay_sync = s.delay_sync;
    o.iq_sync = s.iq_sync;       o.noise_src = s.noise_src;
    o.channels = s.channels;     o.overdrive = s.overdrive;
    o.daq_cf_mhz = s.daq_cf_mhz; o.daq_fs_msps = s.daq_fs_msps;
    o.frame_rate_hz = s.frame_rate_hz; o.recv_mbps = s.recv_mbps;
    o.frames_ok = s.frames_ok;   o.frames_cal = s.frames_cal;
    o.frames_bad = s.frames_bad; o.gaps = s.gaps; o.reconnects = s.reconnects;
    for(int i = 0; i < 8; i++) o.gain_tenths[i] = s.gain_tenths[i];
    memcpy(o.hw_id, s.hw_id, sizeof s.hw_id);
    o.hw_id[sizeof o.hw_id - 1] = 0;
    snprintf(o.last_error, sizeof o.last_error, "%s", s.last_error);
    o.lambda_m = s.lambda_m; o.ambiguity = s.ambiguity;
    o.measuring = (s.measuring != 0);
    o.progress  = s.progress_pct / 100.f;
}

// ── 순수 함수 (원본 kraken_io.cpp:488-530 그대로) ────────────────────────
// 문구가 HOST 와 한 글자라도 다르면 같은 사건이 기지별로 다르게 보인다.
const char* FFTViewer::df_short_reason(const char* detail){
    if(!detail || !*detail) return "Not Available";
    if(strstr(detail, "already running")) return "Already running";
    if(strstr(detail, "no signal"))       return "No signal";
    return "Not Available";
}

void FFTViewer::df_format_line(char* out, size_t n, bool detailed) const {
    const auto& r = pending_df_result;
    if(r.ok){
        snprintf(out, n, "CH%d: %.1f deg (SNR : %.1f dB)%s",
                 r.dnum, r.bearing, r.snr,
                 r.overdrive ? " [OVERDRIVE]" : "");
        return;
    }
    const char* sh = df_short_reason(r.err);
    bool same = true;
    for(const char *a = sh, *b = r.err; ; a++, b++){
        if(tolower((unsigned char)*a) != tolower((unsigned char)*b)){ same = false; break; }
        if(!*a) break;
    }
    if(r.dnum > 0){
        if(detailed && !same) snprintf(out, n, "CH%d : %s (%s)", r.dnum, sh, r.err);
        else                  snprintf(out, n, "CH%d : %s", r.dnum, sh);
    } else {
        if(detailed && !same) snprintf(out, n, "%s (%s)", sh, r.err);
        else                  snprintf(out, n, "%s", sh);
    }
}

// JOIN 에서도 거절은 일어난다 (링크 없음·HOST 가 Kraken 아님·필터 없음). HOST 와
// 같은 슬롯에 실어야 ui.cpp 의 드레인 한 곳이 채팅/로그로 내보낸다.
void FFTViewer::df_post_refusal(int dnum, const char* msg){
    auto& p = pending_df_result;
    p.dnum = dnum; p.arr_idx = -1;
    p.cf_mhz = p.bw_khz = 0.f;
    p.bearing = p.bearing_rel = p.conf = p.snr = p.pwr = 0.f;
    p.frames_used = p.frames_discarded = 0;
    p.ok = false; p.overdrive = false;
    p.algo_papr = p.eff_bw_khz = p.n_eff = p.ambiguity = p.diag_spread_db = 0.f;
    p.alt_deg[0]=p.alt_deg[1]=p.alt_db[0]=p.alt_db[1]=0.f;
    p.alt_n = p.elements = p.algo = 0;
    p.imbalance = false; p.has_spec = false;
    p.t_end_ms = 0; p.dur_ms = 0;
    p.from_auto = df_req_auto;
    snprintf(p.err, sizeof p.err, "%s", msg);
    p.pending.store(true, std::memory_order_release);
}

// 로컬에 SDR 이 없으니 HOST 에 대신 시킨다. 채널 배열은 CHANNEL_SYNC 로 동기화되어
// 표시번호가 HOST 와 같으므로 번호를 그대로 넘기면 된다. 결과는 HOST 가
// DF_RESULT(0x60) 로 돌려주고 df_pump 가 받는다.
// 원본: kraken_io.cpp:532-560.
bool FFTViewer::df_request_by_display_num(int dnum, bool from_auto){
    df_req_auto = from_auto;   // 로컬 거절에도 origin 이 붙어야 한다
    if(!net_cli || !net_cli->is_connected()){
        df_post_refusal(dnum, "no link"); return false;
    }
    if(net_cli->remote_hw.load() != 3){   // 3 = KrakenSDR
        df_post_refusal(dnum, "Not Available"); return false;
    }
    int found = -1;
    for(int i = 0; i < MAX_CHANNELS; i++){
        if(!channels[i].filter_active) continue;
        if(freq_sorted_display_num(i) == dnum){ found = i; break; }
    }
    if(found < 0){
        df_post_refusal(dnum, "no such filter"); return false;
    }
    // 채팅이 아니라 전용 커맨드로 보낸다. 채팅으로 보내면 "DGS-x: /df 1" 이
    // 대화 로그에 남아 시끄럽고, 명령이 사람 입력인 척 섞인다.
    net_cli->cmd_df_measure(dnum, from_auto);
    return true;
}

// ── 설정 ─────────────────────────────────────────────────────────────────
// HOST 방송을 아직 못 받았을 때 보여줄 기본값. df::Config 의 기본값과 같은 값을
// 손으로 옮겨 적은 것이다 — GUI 는 df/df_config.hpp 를 include 하지 않기 때문.
// 이 값은 "첫 방송 전 빈 화면" 을 막는 용도뿐이고, HOST 가 한 번 방송하면 즉시
// 덮인다. df::Config 기본값을 바꾸면 여기도 같이 고칠 것.
static void df_default_pkt(PktDfConfig& p){
    p = PktDfConfig{};
    p.radius_m       = 0.175f;
    p.heading_deg    = 0.0f;
    p.snr_thr_db     = 10.0f;
    p.dc_guard_hz    = 2000.0f;
    p.c_papr         = 30.0f;
    p.target_looks   = 2048;
    p.fft_size       = 8192;
    p.elements       = 5;
    p.algo           = 2;   // MUSIC
    p.sense          = 0;   // CW
    p.avg_frames     = 3;
    p.max_frames     = 12;
    p.signal_dim     = 1;
    p.enable_control = 1;
}

void FFTViewer::df_get_cfg(PktDfConfig& out) const {
    if(net_cli && net_cli->df_cfg_valid.load()){
        std::lock_guard<std::mutex> lk(net_cli->df_cfg_mtx);
        out = net_cli->df_cfg;
        return;
    }
    df_default_pkt(out);
}

// 요청만 보낸다. 정본은 HOST 가 적용한 뒤 방송하는 값이다 — 여기서 로컬에 미리
// 반영하면 HOST 가 거절(validate 실패)했을 때 화면만 바뀐 채로 남는다.
void FFTViewer::df_set_cfg(const PktDfConfig& in){
    if(net_cli) net_cli->send_df_config(in);
}

// 방송은 HOST 만 한다.
void FFTViewer::df_broadcast_cfg() const {}

double FFTViewer::df_snr_threshold() const {
    PktDfConfig p{}; df_get_cfg(p);
    return p.snr_thr_db;
}
