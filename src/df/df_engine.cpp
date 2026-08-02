#include <deque>
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

    // ── 매니폴드 캘리브레이션 ────────────────────────────────────────────
    // last_ok 는 poll() 로 소비된 뒤에도 남는 "마지막으로 수락된 측정" 이다.
    // 캘리브는 운용자가 결과를 보고 "이건 90 도가 맞다" 고 판단한 뒤에 누르므로,
    // 그 시점에는 res 슬롯이 이미 비어 있다.
    std::mutex cal_mtx;
    Calib      cal;
    Result     last_ok{};
    bool       has_last_ok = false;
    // manifold 는 엔진 스레드 소유라 UI 에서 직접 읽으면 race 다. 보정이 실제로
    // 걸렸는지만 원자적으로 내보낸다.
    std::atomic<bool> cal_active{false};

    void loop();
    void publish(const Result& r){
        { std::lock_guard<std::mutex> lk(slot_mtx); res = r; }
        // 수락된 측정만 캘리브 후보로 남긴다 — 거부된 것(잡음뿐)의 주 고유벡터는
        // 방위와 무관한 잡음 방향이라 보정에 넣으면 매니폴드를 망친다.
        if(r.status == Status::Ok){
            std::lock_guard<std::mutex> lk(cal_mtx);
            last_ok = r; has_last_ok = true;
        }
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

// ── 매니폴드 캘리브레이션 ────────────────────────────────────────────────
bool Engine::cal_add(double bearing_deg, char* err, size_t errn){
    auto fail = [&](const char* m){
        if(err && errn){ snprintf(err, errn, "%s", m); }
        return false;
    };
    Config c = config();

    std::lock_guard<std::mutex> lk(d_->cal_mtx);
    if(!d_->has_last_ok)             return fail("no accepted measurement yet");
    const Result& r = d_->last_ok;
    if(r.elements != c.elements)     return fail("element count changed since that measurement");
    if(r.center_hz <= 0.0)           return fail("measurement has no frequency");

    // 캘리브 세트는 한 주파수·한 배열에 묶인다. 그것과 다른 측정이 들어오면
    // 이전 점들은 의미가 없으므로 비우고 새로 시작한다 (조용히 섞으면 보정이
    // 엉뚱해지고 원인을 찾기 어렵다).
    // 같은 세트인지만 본다. usable_at 은 "적용 가능한가" 라 점이 1 개면 거짓이고,
    // 그걸 여기서 쓰면 두 번째 점을 넣을 때마다 첫 점을 지워 영원히 1 점이 된다.
    if(d_->cal.count() > 0 && !d_->cal.same_context(r.center_hz, r.elements))
        d_->cal.clear();
    if(d_->cal.count() == 0) d_->cal.set_context(r.center_hz, r.elements);

    // 그 방위의 **이론** 조향벡터. 보정 없는 순수 기하로 만들어야 한다 —
    // 이미 보정된 매니폴드로 나누면 보정이 두 번 곱해진다.
    Manifold plain;
    plain.ensure(r.center_hz, c.geom(), nullptr);
    if(!plain.valid())               return fail("bad array geometry");
    std::complex<double> a[kMaxElements];
    plain.steer(bearing_deg - c.heading_deg, a);   // 보고값은 offset 이 더해진 값이다

    // 이론 벡터도 소자 0 위상 0 으로 맞춘다 (실측이 그렇게 정규화돼 있다).
    if(std::abs(a[0]) > 1e-12){
        const std::complex<double> rot = std::conj(a[0]) / std::abs(a[0]);
        for(int m = 0; m < r.elements; m++) a[m] *= rot;
    }

    d_->cal.add(bearing_deg - c.heading_deg, r.bearing_rel_deg, r.eig_snr_db,
                r.principal, a, r.elements);
    return true;
}

void Engine::cal_clear(){
    std::lock_guard<std::mutex> lk(d_->cal_mtx);
    d_->cal.clear();
}

void Engine::cal_remove(int idx){
    std::lock_guard<std::mutex> lk(d_->cal_mtx);
    d_->cal.remove_at(idx);
}

bool Engine::cal_save(const char* path) const {
    std::lock_guard<std::mutex> lk(d_->cal_mtx);
    return d_->cal.save(path);
}

bool Engine::cal_load(const char* path){
    std::lock_guard<std::mutex> lk(d_->cal_mtx);
    return d_->cal.load(path);
}

Engine::CalInfo Engine::cal_info() const {
    CalInfo o;
    std::lock_guard<std::mutex> lk(d_->cal_mtx);
    o.n            = d_->cal.count();
    o.freq_hz      = d_->cal.freq_hz();
    o.elements     = d_->cal.elements();
    o.worst_dev_db = d_->cal.worst_dev_db();
    o.active       = d_->cal_active.load(std::memory_order_relaxed);
    for(int i = 0; i < o.n && i < kMaxCalPoints; i++){
        o.bearing[i]  = d_->cal.at(i).bearing_deg;
        o.measured[i] = d_->cal.at(i).measured_deg;
        o.snr_db[i]   = d_->cal.at(i).snr_db;
    }
    return o;
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

    // ── 최근 프레임 보관 (버스트 소급 측정용) ────────────────────────────
    // 버스트 신호는 요청이 도착했을 때 이미 끝나 있다. 그래서 직전 프레임을
    // 몇 개 들고 있다가, use_backlog 요청이 오면 그것부터 적분한다.
    //
    // 5채널 원본을 그대로 복사해 둔다 — DF 는 채널간 위상차가 전부라, ch0 만
    // 담는 ring/롤링 IQ 로는 절대 대신할 수 없다.
    // 프레임 하나 = samples_per_ch * channels * 8 B (complex<float>).
    // 2.4 MSPS/5ch 기준 약 21 MB 이므로 2 개까지만 둔다 (~42 MB).
    struct HeldFrame {
        std::vector<std::complex<float>> iq;   // channel-major 사본
        uint32_t channels = 0, samples_per_ch = 0;
        uint32_t overdrive = 0;
        uint64_t rf_center_hz = 0, sampling_hz = 0;
        bool     usable = false;
    };
    constexpr size_t kHeldMax = 2;
    std::deque<HeldFrame> held;
    // 보관 자체가 비용(21 MB memcpy/프레임 = 2.4 MSPS 에서 초당 약 48 MB)이라
    // 항상 켜두지 않는다. 버스트 요청이 한 번 오면 켜고, 한동안 안 오면 끈다.
    bool     hold_on = false;
    int64_t  hold_last_use_ms = 0;
    constexpr int64_t kHoldIdleMs = 60000;   // 마지막 버스트 요청 후 1분

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

        // ── 최근 프레임 보관 (버스트 요청이 최근에 있었을 때만) ──────────
        // client 의 Frame 은 내부 수신 버퍼를 가리키는 뷰라 다음 next_frame 에서
        // 덮인다. 보관하려면 사본이 있어야 한다.
        if(hold_on && now_ms() - hold_last_use_ms > kHoldIdleMs){
            hold_on = false;
            held.clear();
            held.shrink_to_fit();          // 42 MB 를 실제로 돌려준다
        }
        if(hold_on && f.iq && f.channels > 0 && f.samples_per_ch > 0){
            HeldFrame hf;
            const size_t n = (size_t)f.channels * f.samples_per_ch;
            if(held.size() >= kHeldMax){    // 가장 오래된 버퍼를 재활용
                hf = std::move(held.front());
                held.pop_front();
            }
            hf.iq.resize(n);
            memcpy(hf.iq.data(), f.iq, n * sizeof(std::complex<float>));
            hf.channels       = f.channels;
            hf.samples_per_ch = f.samples_per_ch;
            hf.overdrive      = h.adc_overdrive_flags;
            hf.rf_center_hz   = h.rf_center_freq;
            hf.sampling_hz    = h.sampling_freq;
            hf.usable         = usable_for_df(h) && !f.stale && h.frame_type == FRAME_DATA;
            held.push_back(std::move(hf));
        }

        // ── 새 요청 집기 ─────────────────────────────────────────────────
        if(!measuring && req_pending.exchange(false, std::memory_order_acquire)){
            { std::lock_guard<std::mutex> lk(slot_mtx); cur = req; }
            Config c = [&]{ std::lock_guard<std::mutex> lk(cfg_mtx); return cfg; }();

            if(cur.bandwidth_hz <= 0.0 || cur.center_hz <= 0.0){ fail(cur, Status::BadRequest); continue; }
            if(h.active_ant_chs < 3){ fail(cur, Status::TooFewChannels); continue; }
            // 소자 수는 DAQ 가 정한다. 예전엔 설정과 다르면 ArrayMismatch 로
            // 거부했는데, 그 설정을 운용자가 맞출 방법이 슬라이더 하나뿐이라
            // 틀리게 두면 측정이 통째로 막혔다 (Kraken 은 늘 5채널이므로 5 말고는
            // 전부 오답이다). 이제 DAQ 값으로 맞추고 좌표도 따라 다시 만든다.
            if((int)h.active_ant_chs != c.elements){
                c.elements = (int)h.active_ant_chs;
                // Custom 좌표는 소자 수가 바뀌면 남거나 모자란다 — 손으로 찍은
                // 배치를 반쪽만 쓰느니 프리셋으로 되돌리는 편이 낫다.
                if(c.array_type == ArrayType::Custom) c.array_type = ArrayType::Uca;
                c.rebuild_geom();
                std::lock_guard<std::mutex> lk(cfg_mtx);
                cfg.elements   = c.elements;
                cfg.array_type = c.array_type;
                for(int m = 0; m < kMaxElements; m++){
                    cfg.elem_x[m] = c.elem_x[m];
                    cfg.elem_y[m] = c.elem_y[m];
                }
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

            // 소자 수는 바로 위에서 active_ant_chs 와 일치함이 확인됐다.
            // 캘리브가 이 주파수에 유효하면 보정된 매니폴드가 만들어진다.
            { std::lock_guard<std::mutex> lk(cal_mtx);
              manifold.ensure(cur.center_hz, c.geom(), &cal);
              cal_active.store(manifold.calibrated(), std::memory_order_relaxed); }
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

            // ── 버스트 소급: 보관해 둔 직전 프레임부터 적분 ──────────────
            // 다음에 올 프레임을 기다리면 26 ms 버스트는 이미 지나간 뒤다.
            if(cur.use_backlog){
                hold_last_use_ms = now_ms();
                if(!hold_on){
                    // 이번 요청은 보관본이 없어 소급이 안 된다. 다음 요청부터
                    // 쓸 수 있게 켜만 두고, 이번 건은 기존대로 앞을 보고 잰다.
                    hold_on = true;
                } else {
                    for(const HeldFrame& hf : held){
                        if(got >= cur.frames) break;
                        if(!hf.usable){ discarded++; continue; }
                        // 보관 시점과 지금의 튜닝/샘플레이트가 다르면 그 프레임은
                        // 이 요청의 채널과 무관하다 (재튠 직후). 섞으면 안 된다.
                        if(hf.rf_center_hz != h.rf_center_freq ||
                           hf.sampling_hz  != h.sampling_freq  ||
                           hf.channels     != h.active_ant_chs){ discarded++; continue; }
                        // 실시간 경로와 같은 이유로 버리지 않는다 (아래 긴 주석 참조):
                        // heimdall 의 플래그는 1 샘플 기준이라 포화 판정으로 못 쓴다.
                        if(hf.overdrive) od_mask |= hf.overdrive;
                        if(xspec.add_frame(hf.iq.data(), hf.samples_per_ch)){
                            got++;
                            frames_done.store(got);
                        }
                    }
                }
            }
        }

        // ── 측정 진행 ────────────────────────────────────────────────────
        if(measuring){
            if(cancel_req.exchange(false)){
                fail(cur, Status::Cancelled);
                measuring = false;
                continue;
            }
            // 소급(use_backlog)으로 이미 목표 프레임을 채웠으면 지금 프레임은
            // 더하지 않고 바로 푼다. 그러지 않으면 버스트가 끝난 뒤의 잡음
            // 프레임이 섞여 애써 모은 버스트 구간을 희석한다.
            const bool backlog_filled = (got >= cur.frames);
            if(!backlog_filled){
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
                // ── 과입력 프레임은 누적하지 않고 버린다 ──────────────────
                // ADC 클리핑은 채널간 위상을 파괴하는 경성 비선형이라 한동안
                // 그 프레임을 통째로 버렸다. 그런데 heimdall 의 판정이 그 대응을
                // 감당할 만큼 정밀하지 않다 (rtl_daq.c):
                //
                //   for(n=0; n<buffer_size; n++) if(buf[n] == 255) flags |= 1<<i;
                //
                // 임계도 비율도 없다 — CPI 한 장(262144 샘플, I/Q 52만 바이트)에
                // 255 가 **하나만** 있어도 그 채널이 클리핑으로 찍히고, rebuffer 가
                // 그걸 CPI 전체에 OR 로 누적한다. 8 비트 ADC 라 잡음의 가우시안
                // 꼬리만으로도 이따금 걸린다. 즉 이 플래그는 "포화했다" 가 아니라
                // "최댓값 샘플이 한 번 있었다" 는 뜻이다.
                //
                // 그 상태에서 프레임을 버리면 멀쩡한 신호에서도 쓸 프레임이 안 모여
                // 진행바가 0% 에 멈춘 채 Overdrive 로 실패한다 (2026-08-02 DGS-X:
                // 게인을 1.4 dB 까지 내려도 계속 떴다). 52 만 샘플 중 몇 개가 잘린
                // 것은 공분산 추정에 사실상 영향이 없으므로 누적하고, 플래그는
                // 표시용으로만 남긴다.
                //
                // 진짜로 심하게 포화하면 파형이 뭉개져 eig_snr 과 PAPR 이 같이
                // 떨어지므로 수락 규칙이 거른다 — 그쪽이 원래 그 일을 하는 관문이다.
                if(h.adc_overdrive_flags) od_mask |= h.adc_overdrive_flags;
                if(xspec.add_frame(f.iq, f.samples_per_ch)){
                    got++;
                    frames_done.store(got);
                }
                if(got < cur.frames && budget > 0) continue;
            }
            if(got == 0){
                // 쓸 만한 프레임이 하나도 없었다. 클리핑은 더 이상 프레임을
                // 버리지 않으므로(위 주석) 여기까지 왔다면 원인은 usable 아님 /
                // 재튠으로 인한 불일치 / 프레임 자체가 안 온 것이다.
                fail(cur, Status::Timeout);
                measuring = false; continue;
            }

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
            for(int i = 0; i < M && i < kMaxElements; i++){
                r.eval[i]      = e.eval[i];
                r.principal[i] = e.principal[i];   // 캘리브레이션이 쓰는 실측 조향벡터
            }
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
