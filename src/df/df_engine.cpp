#include "df_engine.hpp"
#include "df_estimator.hpp"
#include "df_manifold.hpp"
#include "df_xspec.hpp"
#include "heimdall_client.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>

namespace df {

using std::chrono::steady_clock;
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

namespace {
int64_t now_ms(){
    return (int64_t)duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
} // namespace

const char* status_text(Status s){
    switch(s){
        case Status::Ok:             return "ok";
        case Status::NoSignal:       return "no signal";
        case Status::LinkDown:       return "no link";
        case Status::NotCalibrated:  return "calibrating";
        case Status::BandOutOfSpan:  return "out of band";
        case Status::DcOverlap:      return "on LO leakage";
        case Status::TooFewChannels: return "too few channels";
        case Status::ArrayMismatch:  return "array mismatch";
        case Status::Timeout:        return "timeout";
        case Status::BadRequest:     return "bad request";
        case Status::Cancelled:      return "cancelled";
        case Status::Overdrive:      return "overdrive: reduce DAQ gain";
    }
    return "?";
}

// ── Impl ──────────────────────────────────────────────────────────────────
struct Engine::Impl {
    std::thread       thr;
    std::atomic<bool> run{false};

    mutable std::mutex cfg_mtx;
    Config             cfg;

    mutable std::mutex st_mtx;
    DaqStatus          st;

    // 요청/결과: 한 칸짜리 슬롯. 측정이 직렬화되므로 큐가 필요 없다.
    std::atomic<bool> req_pending{false};
    std::atomic<bool> res_ready{false};
    std::atomic<bool> cancel_req{false};
    std::atomic<bool> is_armed{false};
    std::atomic<int>  frames_done{0}, frames_want{0};
    std::mutex        slot_mtx;
    Request           req{};
    Result            res{};

    std::mutex ch0_mtx;
    Ch0Sink    ch0;

    HeimdallClient client;
    Manifold       manifold;
    XSpec          xspec;

    void loop();
    void publish(const Result& r){
        { std::lock_guard<std::mutex> lk(slot_mtx); res = r; }
        res_ready.store(true, std::memory_order_release);
        is_armed.store(false, std::memory_order_release);
    }
    void fail(const Request& q, Status s, const char* note = nullptr){
        Result r{};
        r.seq = q.seq; r.ui_tag = q.ui_tag; r.ui_dnum = q.ui_dnum;
        r.status = s; r.algo = q.algo;
        r.center_hz = q.center_hz; r.bandwidth_hz = q.bandwidth_hz;
        r.t_start_ms = r.t_end_ms = now_ms();
        snprintf(r.note, sizeof r.note, "%s", note ? note : status_text(s));
        publish(r);
    }
};

Engine::Engine() : d_(new Impl) {}
Engine::~Engine(){ stop(); }

bool Engine::running() const { return d_->run.load(); }
bool Engine::armed()   const { return d_->is_armed.load(); }

float Engine::progress() const {
    const int w = d_->frames_want.load();
    if(w <= 0) return 0.f;
    return std::min(1.f, (float)d_->frames_done.load() / (float)w);
}

Config Engine::config() const {
    std::lock_guard<std::mutex> lk(d_->cfg_mtx);
    return d_->cfg;
}

void Engine::apply_config(const Config& c){
    std::lock_guard<std::mutex> lk(d_->cfg_mtx);
    // host/port 는 재접속이 필요하므로 스레드가 다음 재연결 때 집는다.
    d_->cfg = c;
}

DaqStatus Engine::status() const {
    std::lock_guard<std::mutex> lk(d_->st_mtx);
    return d_->st;
}

void Engine::set_ch0_sink(Ch0Sink s){
    std::lock_guard<std::mutex> lk(d_->ch0_mtx);
    d_->ch0 = std::move(s);
}

bool Engine::start(const Config& cfg){
    if(d_->run.load()) return false;
    char e[128];
    if(!cfg.validate(e, sizeof e)) return false;
    { std::lock_guard<std::mutex> lk(d_->cfg_mtx); d_->cfg = cfg; }
    d_->run.store(true);
    d_->thr = std::thread([this]{ d_->loop(); });
    return true;
}

void Engine::stop(){
    if(!d_->run.exchange(false)) return;
    if(d_->thr.joinable()) d_->thr.join();
    d_->client.disconnect();
}

bool Engine::submit(const Request& r){
    if(!d_->run.load()) return false;
    if(d_->is_armed.load() || d_->req_pending.load()) return false;
    { std::lock_guard<std::mutex> lk(d_->slot_mtx); d_->req = r; }
    d_->cancel_req.store(false);
    d_->req_pending.store(true, std::memory_order_release);
    return true;
}

bool Engine::poll(Result& out){
    if(!d_->res_ready.exchange(false, std::memory_order_acquire)) return false;
    std::lock_guard<std::mutex> lk(d_->slot_mtx);
    out = d_->res;
    return true;
}

void Engine::cancel(){ d_->cancel_req.store(true); }

bool Engine::set_center_freq(uint64_t hz, char* err, size_t errn){
    Config c = config();
    if(!c.enable_control){ snprintf(err, errn, "DAQ control is disabled in DF settings"); return false; }
    return heimdall_set_freq(c.host, c.ctrl_port, hz, 20000, err, errn);
}

bool Engine::set_gain_tenths(const uint32_t* per_ch, int n, char* err, size_t errn){
    Config c = config();
    if(!c.enable_control){ snprintf(err, errn, "DAQ control is disabled in DF settings"); return false; }
    return heimdall_set_gain_tenths(c.host, c.ctrl_port, per_ch, n, 20000, err, errn);
}

// ── DAQ 스레드 ────────────────────────────────────────────────────────────
void Engine::Impl::loop(){
    char err[128] = {};
    auto set_link = [&](LinkState s){
        std::lock_guard<std::mutex> lk(st_mtx);
        st.link = s;
    };

    // 측정 진행 상태 (스레드 지역)
    bool     measuring = false;
    Request  cur{};
    int      got = 0, discarded = 0, budget = 0;
    uint32_t od_mask = 0;
    int64_t  t_start = 0;

    steady_clock::time_point last_rate = steady_clock::now();
    uint64_t last_frames = 0, last_bytes = 0;

    while(run.load()){
        if(!client.connected()){
            if(measuring){ fail(cur, Status::LinkDown); measuring = false; is_armed.store(false); }
            set_link(LinkState::Connecting);
            Config c = [&]{ std::lock_guard<std::mutex> lk(cfg_mtx); return cfg; }();
            if(!client.connect(c.host, c.data_port, 8000, err, sizeof err)){
                { std::lock_guard<std::mutex> lk(st_mtx);
                  st.link = LinkState::Down;
                  snprintf(st.last_error, sizeof st.last_error, "%s", err); }
                // 재시도 간격. iq_server 는 클라이언트가 끊기면 리스닝 소켓을
                // 다시 만드는 동안 잠깐 아무도 안 듣는 구간이 있다.
                for(int i = 0; i < 20 && run.load(); i++)
                    std::this_thread::sleep_for(milliseconds(100));
                continue;
            }
            { std::lock_guard<std::mutex> lk(st_mtx); st.reconnects++; st.last_error[0] = 0; }
        }

        HeimdallClient::Frame f;
        if(!client.next_frame(f, err, sizeof err)){
            { std::lock_guard<std::mutex> lk(st_mtx);
              snprintf(st.last_error, sizeof st.last_error, "%s", err); }
            // 소켓은 즉시 반납한다. 남겨두면 daq_start_sm.sh 의 포트 게이트가
            // 클라이언트 소켓까지 "사용 중"으로 보고 영원히 안 끝난다.
            client.disconnect();
            continue;
        }
        const IqHeader& h = *f.hdr;

        // ── 상태 갱신 ────────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(st_mtx);
            st.sync_state = h.sync_state;
            st.delay_sync_flag = h.delay_sync_flag;
            st.iq_sync_flag = h.iq_sync_flag;
            st.noise_source_state = h.noise_source_state;
            st.adc_overdrive_flags = h.adc_overdrive_flags;
            st.active_ant_chs = h.active_ant_chs;
            st.cpi_length = h.cpi_length;
            st.rf_center_hz = h.rf_center_freq;
            st.sampling_hz = h.sampling_freq;
            for(int i = 0; i < 8 && i < (int)h.active_ant_chs; i++)
                st.if_gain_tenths[i] = h.if_gains[i];
            memcpy(st.hardware_id, h.hardware_id, 16); st.hardware_id[16] = 0;
            st.usable = usable_for_df(h) && !f.stale;
            st.cpi_gaps = client.gaps();
            st.last_frame_wall_ms = now_ms();
            if(h.frame_type == FRAME_CAL)        st.frames_cal++;
            else if(h.frame_type == FRAME_DUMMY) st.frames_dummy++;
            else if(st.usable)                   st.frames_ok++;
            else                                 st.frames_bad++;
            st.link = st.usable ? LinkState::Streaming : LinkState::Calibrating;

            const auto now = steady_clock::now();
            const double el = std::chrono::duration<double>(now - last_rate).count();
            if(el >= 1.0){
                st.frame_rate_hz = (client.frames_read() - last_frames) / el;
                st.recv_mbps = (client.bytes_read() - last_bytes) / el / 1e6;
                last_frames = client.frames_read();
                last_bytes = client.bytes_read();
                last_rate = now;
            }
        }

        // ── ch0 탭 ───────────────────────────────────────────────────────
        if(f.iq && !f.stale && h.frame_type == FRAME_DATA){
            std::lock_guard<std::mutex> lk(ch0_mtx);
            if(ch0) ch0(f.channel(0), f.samples_per_ch, h.rf_center_freq,
                        h.sampling_freq, h.adc_overdrive_flags, (int64_t)h.time_stamp);
        }

        // ── 새 요청 집기 ─────────────────────────────────────────────────
        if(!measuring && req_pending.exchange(false, std::memory_order_acquire)){
            { std::lock_guard<std::mutex> lk(slot_mtx); cur = req; }
            Config c = [&]{ std::lock_guard<std::mutex> lk(cfg_mtx); return cfg; }();

            if(cur.bandwidth_hz <= 0.0 || cur.center_hz <= 0.0){ fail(cur, Status::BadRequest); continue; }
            if(h.active_ant_chs < 3){ fail(cur, Status::TooFewChannels); continue; }
            if((int)h.active_ant_chs != c.elements){
                char n[96]; snprintf(n, sizeof n, "DAQ reports %u channels, DF is configured for %d",
                                     h.active_ant_chs, c.elements);
                fail(cur, Status::ArrayMismatch, n); continue;
            }

            XSpec::Params xp;
            xp.ch_center_hz  = cur.center_hz;
            xp.ch_bw_hz      = cur.bandwidth_hz;
            xp.daq_center_hz = (double)h.rf_center_freq;
            xp.daq_fs_hz     = (double)h.sampling_freq;
            xp.elements      = (int)h.active_ant_chs;
            xp.dc_guard_hz   = c.dc_guard_hz;
            xp.target_looks  = c.target_looks;
            xp.fft_size      = c.fft_size;
            Status why = Status::Ok;
            if(!xspec.prepare(xp, why)){ fail(cur, why); continue; }
            xspec.reset();

            manifold.ensure(cur.center_hz, c.radius_m, (int)h.active_ant_chs, c.sense);
            if(!manifold.valid()){ fail(cur, Status::BadRequest, "bad array geometry"); continue; }

            measuring  = true;
            got        = 0;
            discarded  = 0;
            od_mask    = 0;
            t_start    = now_ms();
            budget     = std::max(c.max_frames, cur.frames) + 8;   // CAL 버스트 여유
            frames_done.store(0);
            frames_want.store(cur.frames);
            is_armed.store(true, std::memory_order_release);
        }

        // ── 측정 진행 ────────────────────────────────────────────────────
        if(measuring){
            if(cancel_req.exchange(false)){
                fail(cur, Status::Cancelled);
                measuring = false;
                continue;
            }
            budget--;
            if(!usable_for_df(h) || f.stale || !f.iq){
                // CAL 버스트·DUMMY·미캘리브레이션 프레임은 버리고 예산만 쓴다.
                discarded++;
                if(budget <= 0){
                    Status s = (h.delay_sync_flag && h.iq_sync_flag) ? Status::Timeout
                                                                     : Status::NotCalibrated;
                    fail(cur, s);
                    measuring = false;
                }
                continue;
            }
            // ── 과입력 프레임은 누적하지 않고 버린다 ──────────────────────
            // ADC 클리핑은 측정 대상 그 자체인 채널간 위상을 파괴하는 경성
            // 비선형이다. 그런데 클리핑된 프레임도 강한 지배 고유값과 높은
            // PAPR 을 만들어 수락 규칙을 그대로 통과한다 — 즉 결과가 "자신만만
            // 하게 틀린 방위"가 되고 출력만 봐서는 구분할 방법이 없다.
            // 예전엔 전량 누적하고 플래그만 od_mask 에 OR 해 표시용으로 썼다.
            if(h.adc_overdrive_flags){
                discarded++;
                od_mask |= h.adc_overdrive_flags;
            } else if(xspec.add_frame(f.iq, f.samples_per_ch)){
                got++;
                frames_done.store(got);
            }
            if(got < cur.frames && budget > 0) continue;
            if(got == 0){
                // 쓸 만한 프레임이 하나도 없었다. 원인이 과입력이면 그렇게
                // 말해준다 — enable_control 이 켜져 있으면 운용자가 DAQ gain 을
                // 바로 내릴 수 있으니 실행 가능한 안내다.
                fail(cur, od_mask ? Status::Overdrive : Status::Timeout);
                measuring = false; continue;
            }
            // got > 0 이면 예산이 클리핑 프레임에서 끝났더라도 모아둔 클린
            // 프레임으로 푼다. 여기서 버리면 "3장 중 2장은 깨끗했는데 마지막이
            // 클리핑" 이 통째로 실패가 된다 — 부분 평균이 실패보다 낫다.

            // ── 풀이 ─────────────────────────────────────────────────────
            Config c = [&]{ std::lock_guard<std::mutex> lk(cfg_mtx); return cfg; }();
            std::complex<double> R[kMaxElements * kMaxElements];
            if(!xspec.snapshot_R(R)){ fail(cur, Status::Timeout); measuring = false; continue; }

            const int M = xspec.elements();
            Estimate e = estimate_doa(R, M, manifold, cur.algo, cur.signal_dim,
                                      xspec.n_eff(), c.c_papr, c.snr_threshold_db);

            Result r{};
            r.seq = cur.seq; r.ui_tag = cur.ui_tag; r.ui_dnum = cur.ui_dnum;
            r.status = e.ok ? Status::Ok : Status::NoSignal;
            r.bearing_rel_deg = e.bearing_deg;
            r.bearing_deg = std::fmod(e.bearing_deg + c.heading_deg + 360.0, 360.0);
            r.confidence_db = e.confidence_db;
            r.eig_snr_db = e.eig_snr_db;
            r.power_dbfs = e.power_dbfs;
            r.center_hz = cur.center_hz;
            r.bandwidth_hz = cur.bandwidth_hz;
            r.effective_bw_hz = xspec.effective_bw_hz();
            r.daq_center_hz = h.rf_center_freq;
            r.daq_fs_hz = h.sampling_freq;
            r.frames_used = got;
            r.frames_discarded = discarded;
            r.n_eff_looks = xspec.n_eff();
            r.elements = M;
            r.algo = cur.algo;
            r.overdrive_mask = od_mask;
            r.ambiguity_ratio = manifold.ambiguity_ratio();
            r.algo_papr_db = e.algo_papr_db;
            r.diag_spread_db = e.diag_spread_db;
            r.imbalance = e.imbalance;
            for(int i = 0; i < M && i < kMaxElements; i++) r.eval[i] = e.eval[i];
            r.alt_n = e.alt_n;
            for(int i = 0; i < e.alt_n; i++){
                r.alt_deg[i] = std::fmod(e.alt_deg[i] + c.heading_deg + 360.0, 360.0);
                r.alt_db[i]  = e.alt_db[i];
            }
            memcpy(r.spectrum_db, e.spectrum_db, sizeof r.spectrum_db);
            r.t_start_ms = t_start;
            r.t_end_ms = now_ms();
            // note 는 채팅 한 줄에 그대로 들어간다. 짧게 유지할 것 —
            // 자세한 수치는 DF 설정 패널이 보여준다.
            //
            // 실패 경로의 문구는 건드리지 말 것: df_short_reason 이 이 문자열을
            // 패턴매칭해 3개 결과로 압축하는 계약이 있다. 대안 방위는 성공
            // 경로에만 붙인다.
            // 순서 = 심각도. 위쪽일수록 결과 자체를 못 믿는다는 뜻이고,
            // alt 는 마지막이다 — 정보성 안내가 하드 경고를 가리면 안 된다.
            if(!e.ok)
                snprintf(r.note, sizeof r.note, "no signal");
            else if(e.imbalance)
                snprintf(r.note, sizeof r.note, "channel imbalance %.0f dB", e.diag_spread_db);
            else if(od_mask)
                snprintf(r.note, sizeof r.note, "overdrive");
            else if(r.ambiguity_ratio > 1.0)
                snprintf(r.note, sizeof r.note, "grating lobes");
            else if(e.alt_n > 0)
                snprintf(r.note, sizeof r.note, "alt %.0f deg %.0f dB",
                         r.alt_deg[0], e.alt_db[0]);

            publish(r);
            measuring = false;
        }
    }

    client.disconnect();
    set_link(LinkState::Down);
}

} // namespace df
