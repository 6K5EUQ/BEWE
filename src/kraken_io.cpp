// ── KrakenSDR / heimdall DAQ 백엔드 ───────────────────────────────────────
//
// 다른 백엔드와 달리 BEWE 가 USB 장치를 직접 열지 않는다. heimdall DAQ 체인
// (rtl_daq.out -> rebuffer -> decimate -> delay_sync)이 동글 5개를 소유하고
// 위상보정된 5채널 IQ 를 만들며, iq_server.out 이 그걸 TCP :5000 으로 내준다.
// BEWE 는 그 스트림의 소비자다:
//
//   ch0        -> 기존 스펙트럼/워터폴/ring/복조 경로 (다른 백엔드와 동일)
//   ch0 .. ch4 -> DF(방탐) 엔진
//
// 스레드 구성
//   df::Engine 스레드 : TCP 소비 + DF 측정. ch0 을 Ch0Sink 로 넘긴다.
//   캡처 스레드        : Ch0Sink 가 채워둔 int16 스테이징을 받아 FFT/ring/TM.
// 두 개로 나눈 이유는 DF 측정(수백 ms)이 스펙트럼 갱신을 멈추면 안 되기 때문이고,
// 또 캡처 스레드는 기존 코드가 존재를 전제하기 때문이다 (cap.join 등).
//
// 진입 조건: --sdr kraken 명시 지정일 때만. 자동 감지 대상이 아니다 —
// heimdall 이 다른 소비자를 위해 떠 있을 수도 있고, 기동마다 :5000 을 두드리면
// 그만큼 느려진다.

#include "fft_viewer.hpp"
#include "net_server.hpp"
#include "long_waterfall.hpp"
#include "df/df_engine.hpp"
#include "df/df_manifold.hpp"
#include "df/heimdall_header.hpp"

#include <volk/volk.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cctype>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace {

// ── DF 엔진 + ch0 스테이징 ───────────────────────────────────────────────
// 엔진은 프로세스 전역 1개다. FFTViewer 가 헤더를 안 건드리게 하려고 여기 둔다
// (kraken_io.cpp 만 df/ 를 알면 된다).
struct KrakenState {
    std::unique_ptr<df::Engine> engine;

    // DF 설정. 헤더가 df/ 를 include 하지 않게 하려고 여기에 둔다. 설정 패널은
    // plain 스칼라 setter (FFTViewer::df_set_*) 로만 건드린다.
    std::mutex  cfg_mtx;
    df::Config  cfg;

    // ch0 스테이징: 엔진 스레드가 채우고 캡처 스레드가 소비한다. 42 MB 짜리
    // 프레임 전체가 아니라 ch0 만, 그것도 int16 로 줄여 담는다 (1048576 샘플
    // -> 4 MB).
    //
    // 예전엔 한 칸짜리 슬롯이었다. 스펙트럼만 보면 "밀리면 최신 것만 그린다"
    // 가 맞지만, 이 스테이징은 ring 을 거쳐 복조·IQ 녹음·TM 으로도 흘러간다.
    // 프레임을 버리면 오디오에 437 ms 짜리 구멍이 뚫리고(뚝뚝 끊김), 녹음
    // 파일에도 같은 크기의 누락이 남는다. heimdall 은 437 ms 마다 1 M 샘플을
    // 버스트로 주므로, 캡처가 행 페이싱으로 잠깐 자는 동안 도착한 프레임이
    // 통째로 사라지는 구조였다.
    //
    // 유한 큐(drop-oldest)로 바꾼다. 가득 차면 **가장 오래된 것**을 버린다 —
    // sink 는 DF 엔진 스레드에서 불리므로 절대 블로킹하면 안 된다 (기다리면
    // DF 측정 자체가 멈춘다). 큐 길이 2 = 최대 ~0.9 초 지연, +4 MB.
    static constexpr size_t CH0_QUEUE_MAX = 2;
    struct Ch0Frame {
        std::vector<int16_t> iq;        // interleaved I/Q
        size_t   n = 0;
        uint64_t cf_hz = 0, fs_hz = 0;
        uint32_t overdrive = 0;
    };
    std::mutex              mtx;
    std::condition_variable cv;
    std::deque<Ch0Frame>    q;          // mtx 보호
    std::deque<std::vector<int16_t>> freelist;  // 버퍼 재활용 (프레임당 4 MB 재할당 방지)
    uint64_t                dropped = 0;   // 캡처가 못 따라가 버린 프레임 수
};

KrakenState& kst(){ static KrakenState s; return s; }

// heimdall 은 공칭 +-1.0 의 complex float32 를 준다. ring/TM/demod/IQ녹음은
// 전부 int16 interleaved 를 전제하므로 변환이 필요하다. BladeRF/Pluto 와 같은
// SC16_Q11 스케일(2048)로 맞춰야 dB 눈금 / 스퀠치 임계 / 녹음 파일이 백엔드마다
// 달라지지 않는다. +-1.0 -> +-2048 은 int16 포화까지 24 dB 여유.
// (변환 자체는 ch0 sink 의 volk_32f_s32f_convert_16i — 동일 스케일/포화/라운딩.)

constexpr int64_t kKrakenRecalGraceMs = 8000;   // FREQ 후 DF 를 못 믿는 구간
constexpr int64_t kFreqCoalesceMs     = 1000;   // 주파수축 드래그 합치기

} // namespace

// ── 초기화 ────────────────────────────────────────────────────────────────
bool FFTViewer::initialize_kraken(float cf_mhz){
    auto& K = kst();
    (void)cf_mhz;   // 아래 참조: DAQ 가 이미 튜닝된 주파수를 채택한다

    if(!K.engine) K.engine.reset(new df::Engine());

    df::Config cfg;
    { std::lock_guard<std::mutex> lk(K.cfg_mtx); cfg = K.cfg; }
    char verr[128] = {};
    if(!cfg.validate(verr, sizeof verr)){
        bewe_log_push(2,"[Kraken] bad DF config: %s\n", verr);
        return false;
    }
    if(!K.engine->running() && !K.engine->start(cfg)){
        bewe_log_push(2,"[Kraken] failed to start DF engine\n");
        return false;
    }
    K.engine->apply_config(cfg);

    // 링크가 설 때까지 기다린다. heimdall 이 캘리브레이션 중이면 Streaming 까지
    // 몇 초 걸릴 수 있어 Calibrating 도 성공으로 친다 — 헤더는 이미 유효하다.
    df::DaqStatus st{};
    bool up = false;
    for(int i = 0; i < 60; i++){
        st = K.engine->status();
        if(st.link == df::LinkState::Streaming || st.link == df::LinkState::Calibrating){ up = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if(!up || st.sampling_hz == 0 || st.active_ant_chs == 0){
        bewe_log_push(2,"[Kraken] no DAQ on %s:%u (%s)\n",
                      cfg.host, (unsigned)cfg.data_port,
                      st.last_error[0] ? st.last_error : "timeout");
        bewe_log_push(2,"[Kraken] start it with: /home/ku/krakensdr_doa/bewe_df_start.sh\n");
        K.engine->stop();
        return false;
    }

    // ── DAQ 가 이미 맞춰둔 주파수를 그대로 채택한다 ──────────────────────
    // 여기서 FREQ 를 쏘면 heimdall 이 STATE_INIT 으로 돌아가 매 기동마다
    // 수 초씩 DF 를 못 쓴다. 운용자가 원할 때만 명시적으로 재튠한다.
    const uint32_t actual_sr = (uint32_t)st.sampling_hz;
    const uint64_t adopted   = st.rf_center_hz;

    hw = make_kraken_config(actual_sr);
    if(gain_db <= 0.f) gain_db = hw.gain_default;

    bewe_log_push(0,"[Kraken] %s  %u ch  %.4f MHz  %.3f MSPS  (adopted from DAQ)\n",
                  st.hardware_id, st.active_ant_chs, adopted / 1e6, actual_sr / 1e6);
    if(st.active_ant_chs != (uint32_t)cfg.elements)
        bewe_log_push(2,"[Kraken] DAQ has %u channels but DF is configured for %d - "
                        "fix it in the DF panel\n", st.active_ant_chs, cfg.elements);

    // ── 이하는 다른 백엔드와 같은 계약 ──────────────────────────────────
    // 이 블록이 드로잉/워터폴/TM/네트워크 경로를 손 안 대고 살리는 지점이다.
    std::memcpy(header.magic,"FFTD",4);
    fft_input_size = fft_size / FFT_PAD_FACTOR;
    header.version=1; header.fft_size=fft_size; header.sample_rate=actual_sr;
    header.center_frequency=adopted;
    live_cf_hz.store(adopted, std::memory_order_release);
    time_average=hw.compute_time_average(fft_input_size);
    header.time_average=time_average; header.power_min=-100; header.power_max=0; header.num_ffts=0;
    fft_data.resize((size_t)FFT_HISTORY_ROWS*fft_size);
    current_spectrum.resize(fft_size,-100.0f);

    char title[256]; snprintf(title,256,"BEWE (" BEWE_VERSION ")");
    window_title=title; display_power_min=-100; display_power_max=0;
    if(fft_in)  { fftwf_free(fft_in);  fft_in=nullptr; }
    if(fft_out) { fftwf_free(fft_out); fft_out=nullptr; }
    fft_in =fftwf_alloc_complex(fft_size);
    fft_out=fftwf_alloc_complex(fft_size);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex));
    fft_plan=bewe_fft_plan(fft_size,fft_in,fft_out,FFTW_FORWARD,/*learn=*/true);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex));
    if(win_buf) free(win_buf);
    win_buf=(float*)volk_malloc(fft_input_size*sizeof(float), volk_get_alignment());
    fill_nuttall_window(win_buf, fft_input_size);
    if(mag_sq_buf) volk_free(mag_sq_buf);
    mag_sq_buf=(float*)volk_malloc(fft_size*sizeof(float), volk_get_alignment());
    ring.resize(IQ_RING_CAPACITY*2,0);
    autoscale_req.store(true, std::memory_order_relaxed);

    // ── ch0 탭 등록 ─────────────────────────────────────────────────────
    // 엔진 스레드에서 불린다. 블로킹 금지 — 변환해서 스테이징에 넣고 즉시 반환.
    // 블로킹 금지 — 여기서 기다리면 DF 측정 루프가 통째로 멈춘다. 락도 try_to_lock
    // 으로만 잡고, 못 잡으면 그 프레임만 포기한다 (캡처가 큐를 만지는 그 짧은 순간).
    K.engine->set_ch0_sink([](const std::complex<float>* ch0, size_t n,
                              uint64_t cf, uint64_t fs, uint32_t od, int64_t){
        auto& S = kst();
        std::unique_lock<std::mutex> lk(S.mtx, std::try_to_lock);
        if(!lk.owns_lock()){ S.dropped++; return; }
        // 재활용 버퍼가 있으면 꺼내 쓴다 (프레임당 4 MB 재할당 방지).
        KrakenState::Ch0Frame fr;
        if(!S.freelist.empty()){ fr.iq = std::move(S.freelist.back()); S.freelist.pop_back(); }
        if(fr.iq.size() < n*2) fr.iq.resize(n*2);
        // std::complex<float> 는 interleaved float 연속 배열 — VOLK 커널이 kr_f2i 와
        // 동일한 스케일·포화·nearest-even 라운딩으로 한 번에 변환 (스칼라 루프 대체)
        volk_32f_s32f_convert_16i(fr.iq.data(),
                                  reinterpret_cast<const float*>(ch0),
                                  2048.0f, (unsigned)(n*2));
        fr.n = n; fr.cf_hz = cf; fr.fs_hz = fs; fr.overdrive = od;
        // drop-oldest: 캡처가 계속 못 따라오면 가장 오래된 것부터 버린다.
        while(S.q.size() >= KrakenState::CH0_QUEUE_MAX){
            if(S.freelist.size() < KrakenState::CH0_QUEUE_MAX + 1)
                S.freelist.push_back(std::move(S.q.front().iq));
            S.q.pop_front();
            S.dropped++;
        }
        S.q.push_back(std::move(fr));
        lk.unlock();
        S.cv.notify_one();
    });

    return true;
}

// ── 캡처 루프 ─────────────────────────────────────────────────────────────
// 구조는 capture_and_process_pluto 와 같다 (큰 블록 수신 -> iq16 -> ring/TM ->
// fft_input_size 슬라이스). RX 구간만 "엔진이 채워둔 스테이징 대기" 로 바뀐다.
void FFTViewer::capture_and_process_kraken(){
    CapLifeGuard cap_life(&cap_exited);
    auto& K = kst();
    if(!K.engine){ sdr_stream_error.store(true); return; }

    std::vector<int16_t> iq16;
    std::vector<float>   pacc(fft_size, 0.0f);
    int   fcnt = 0;
    static constexpr int WARMUP_FFTS = 30;
    int   warmup_cnt = 0;
    static constexpr int MAX_ROW_FFTS = 32;
    int   win_skip = 0;
    const float iq_scale = hw.iq_scale;

    int rx_pos = 0, rx_avail = 0;
    int64_t last_freq_ms = 0;
    // ── 워터폴 행 페이싱 ─────────────────────────────────────────────────
    // heimdall 은 437 ms 마다 1,048,576 샘플을 덩어리로 준다. 그대로 처리하면
    // 행 8개가 순식간에 쏟아지고 400 ms 를 쉬어서 화면이 뚝뚝 끊긴다
    // (RTL/Pluto 는 작은 버퍼가 연속으로 와 저절로 고르게 나온다).
    // 한 행이 담는 시간 = fft_input_size * time_average / sample_rate 이므로
    // 그 간격으로 절대 데드라인 페이싱한다. 누적 드리프트를 막으려고 상대
    // sleep 이 아니라 다음 행의 목표 시각을 들고 간다.
    auto row_due = std::chrono::steady_clock::now();
    bool row_due_init = false;
    auto now_ms = []{ return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch()).count(); };

    // ch0 프레임 유실 진단 (1분 주기). 이 카운터가 오르면 그만큼 오디오/녹음에
    // 구멍이 난 것이다 — 예전엔 증가만 하고 아무 데서도 안 읽혀 증상만 보였다.
    int64_t  drop_log_ms = now_ms();
    uint64_t drop_seen   = 0;

    while(is_running && !sdr_stream_error.load(std::memory_order_relaxed)){
        if(capture_pause.load(std::memory_order_relaxed)){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            rx_pos=0; rx_avail=0;
            continue;
        }

        {   // 유실 보고 — 늘었을 때만 찍는다 (정상 운용 로그를 더럽히지 않게).
            const int64_t t = now_ms();
            if(t - drop_log_ms >= 60000){
                uint64_t d;
                { std::lock_guard<std::mutex> lk(K.mtx); d = K.dropped; }
                if(d != drop_seen){
                    bewe_log_push(2,"[Kraken] ch0 frames dropped: %llu (+%llu in last min) "
                                    "- audio/recording gaps\n",
                                  (unsigned long long)d,
                                  (unsigned long long)(d - drop_seen));
                    drop_seen = d;
                }
                drop_log_ms = t;
            }
        }

        // ── 샘플레이트 변경 요청은 거부한다 ─────────────────────────────
        // 샘플레이트는 heimdall 의 daq_chain_config.ini 소유고 :5001 로 바꿀
        // 수단이 없다. 플래그만 내리고 알린다.
        if(sr_change_req){
            sr_change_req = false;
            bewe_log_push(2,"[Kraken] sample rate is fixed by heimdall "
                            "daq_chain_config.ini - request ignored\n");
        }

        // ── 재튠 ────────────────────────────────────────────────────────
        if(freq_req.load(std::memory_order_acquire)){
            // 주파수축 드래그는 매 프레임 freq_req 를 세운다. Kraken 에선 FREQ
            // 한 번이 수 초의 재캘리브레이션을 부르므로, 직전 요청으로부터
            // 1초가 안 지났으면 요청을 남겨둔 채 미룬다 (드래그 = 재튠 1회).
            const int64_t t = now_ms();
            if(t - last_freq_ms >= kFreqCoalesceMs){
                const float cf = pending_cf.load(std::memory_order_relaxed);
                char err[128] = {};
                if(K.engine->set_center_freq((uint64_t)(cf*1e6), err, sizeof err)){
                    last_freq_ms = t;
                    rx_pos=0; rx_avail=0;
                    { std::lock_guard<std::mutex> lk(data_mtx);
                      header.center_frequency=(uint64_t)(cf*1e6); }
                    live_cf_hz.store((uint64_t)(cf*1e6), std::memory_order_release);
                    LongWaterfall::request_rotate();
                    autoscale_req.store(true, std::memory_order_relaxed);
                    sq_recalib_req.store(true, std::memory_order_relaxed);
                    warmup_cnt=0;
                    update_dem_by_freq(cf);
                    bewe_log_push(0,"Freq > %.4f MHz (KrakenSDR: DAQ recalibrating, "
                                    "DF unavailable ~%.0f s)\n", cf, kKrakenRecalGraceMs/1000.0);
                } else {
                    bewe_log_push(2,"[Kraken] retune failed: %s\n", err);
                }
                freq_req.store(false, std::memory_order_relaxed);
            }
            // 코얼레싱 중이면 freq_req 를 그대로 두고 다음 사이클에 다시 본다.
        }

        // ── RX: 엔진이 채워둔 ch0 스테이징을 받는다 ─────────────────────
        if(rx_avail <= 0){
            size_t n = 0;
            {
                std::unique_lock<std::mutex> lk(K.mtx);
                if(!K.cv.wait_for(lk, std::chrono::milliseconds(1500),
                                  [&]{ return !K.q.empty() || !is_running; }))
                    continue;                      // 타임아웃 — 루프 조건 다시 검사
                if(K.q.empty()) continue;
                // 큐 맨 앞(가장 오래된 것)부터 순서대로 — 버려야 할 때는 sink 가
                // 이미 버렸다. 여기서 최신만 집으면 그 사이 프레임이 또 사라진다.
                n = K.q.front().n;
                // 4MB memcpy 대신 버퍼 소유권 교환. 직전에 쓰던 버퍼는 sink 가
                // 다시 쓸 수 있게 freelist 로 돌려준다.
                std::vector<int16_t> prev = std::move(iq16);
                iq16 = std::move(K.q.front().iq);
                K.q.pop_front();
                if(K.freelist.size() < KrakenState::CH0_QUEUE_MAX + 1)
                    K.freelist.push_back(std::move(prev));
                if(iq16.size() < n*2) iq16.resize(n*2);
            }
            if(n == 0) continue;

            // IQ Ring + TM IQ 기록 (pluto/rtl 과 동일)
            bool need_ring = rec_on.load(std::memory_order_relaxed);
            if(!need_ring) for(int i=0;i<MAX_CHANNELS;i++) if(channels[i].dem_run.load()){need_ring=true;break;}
            if(!need_ring) need_ring = mod_wants_ring.load(std::memory_order_relaxed);
            bool need_tm = tm_iq_on.load(std::memory_order_relaxed) && (warmup_cnt>=WARMUP_FFTS);
            if(need_ring || need_tm){
                size_t wp=ring_wp.load(std::memory_order_relaxed);
                size_t nn=n, cap=IQ_RING_CAPACITY;
                if(nn > cap) nn = cap;
                if(wp+nn<=cap) memcpy(&ring[wp*2],iq16.data(),nn*2*sizeof(int16_t));
                else{
                    size_t p1=cap-wp, p2=nn-p1;
                    memcpy(&ring[wp*2],iq16.data(),p1*2*sizeof(int16_t));
                    memcpy(&ring[0],iq16.data()+p1*2,p2*2*sizeof(int16_t));
                }
                ring_wp.store((wp+nn)&IQ_RING_MASK, std::memory_order_release);
                if(need_tm) tm_iq_write(iq16.data(), (int)n);
            }
            rx_pos=0; rx_avail=(int)n;
            // 프레임 경계에서 페이싱 기준을 다시 잡는다. 오래 밀렸으면
            // 따라잡으려 몰아치지 말고 지금부터 균등하게 낸다.
            const auto now_p = std::chrono::steady_clock::now();
            if(!row_due_init || row_due < now_p){ row_due = now_p; row_due_init = true; }
        }

        if(rx_avail < fft_input_size){ rx_avail=0; rx_pos=0; continue; }

        if(!render_visible.load(std::memory_order_relaxed)){
            rx_pos+=fft_input_size; rx_avail-=fft_input_size;
            std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            continue;
        }
        if(!spectrum_pause.load(std::memory_order_relaxed)){
            const int fft_stride = time_average>MAX_ROW_FFTS
                                 ? (time_average+MAX_ROW_FFTS-1)/MAX_ROW_FFTS : 1;
            if(++win_skip < fft_stride){
                rx_pos+=fft_input_size; rx_avail-=fft_input_size;
                continue;
            }
            win_skip=0;
            const int16_t* rp = iq16.data() + (size_t)rx_pos*2;
            const float inv_scale = 1.0f / iq_scale;
            for(int i=0;i<fft_input_size;i++){
                fft_in[i][0] = (float)rp[i*2+0] * inv_scale;
                fft_in[i][1] = (float)rp[i*2+1] * inv_scale;
            }
            volk_32fc_32f_multiply_32fc((lv_32fc_t*)fft_in, (lv_32fc_t*)fft_in,
                                        win_buf, fft_input_size);
            fftwf_execute(fft_plan);
            {
                volk_32fc_magnitude_squared_32f(mag_sq_buf, (lv_32fc_t*)fft_out, fft_size);
                const float scale=NUTTALL_WINDOW_CORRECTION/((float)fft_input_size*(float)fft_input_size);
                for(int i=0;i<fft_size;i++) pacc[i] += mag_sq_buf[i]*scale + 1e-10f;
            }
            pacc[0]=(pacc[1]+pacc[fft_size-1])*0.5f; fcnt++;
            if(fcnt>=(time_average+fft_stride-1)/fft_stride){
                if(warmup_cnt < WARMUP_FFTS){
                    warmup_cnt++;
                    std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
                    rx_pos+=fft_input_size; rx_avail-=fft_input_size;
                    continue;
                }
                commit_fft_row(pacc, fcnt);   // 공용 커밋 (bladerf_io.cpp — 4백엔드 단일 정본)
                std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;

                // 행 하나가 담는 실제 시간만큼 기다렸다 다음 행을 낸다.
                // 정지/비표시 상태에선 페이싱하지 않는다 (아래 조건 밖).
                if(hw.sample_rate > 0){
                    const double row_s = (double)fft_input_size * (double)time_average
                                       / (double)hw.sample_rate;
                    row_due += std::chrono::microseconds((long long)(row_s * 1e6));
                    const auto now_r = std::chrono::steady_clock::now();
                    // 다음 프레임이 이미 큐에 쌓여 있으면 자지 않는다. 자는 동안
                    // 큐가 더 밀리면 sink 가 drop-oldest 로 버리기 시작하고, 그건
                    // 곧 오디오·녹음의 구멍이다. 화면 부드러움보다 IQ 연속성이
                    // 우선이다 — 밀린 상태에선 몰아서 내고 페이싱은 다음 프레임에
                    // 다시 잡는다.
                    bool backlog;
                    { std::lock_guard<std::mutex> lk(K.mtx); backlog = !K.q.empty(); }
                    if(row_due > now_r && !backlog){
                        // 프레임 주기보다 오래 자면 다음 프레임을 놓친다. 상한을 둔다.
                        auto wait = row_due - now_r;
                        const auto cap = std::chrono::milliseconds(200);
                        if(wait > cap){ wait = cap; row_due = now_r + cap; }
                        std::this_thread::sleep_for(wait);
                    } else {
                        row_due = now_r;   // 밀렸으면 기준을 현재로 당긴다
                    }
                }
            }
        }
        rx_pos+=fft_input_size; rx_avail-=fft_input_size;
    }

    // 캡처가 멈춰도 엔진은 살려둔다 — /rx stop 후 /rx start 로 다시 붙는다.
    // 엔진 종료는 initialize 실패 경로와 프로세스 종료에서만 한다.
}

// ── DF 배선 (UI 계층이 쓰는 얇은 래퍼) ────────────────────────────────────
// FFTViewer 헤더가 df/ 를 include 하지 않도록 여기서만 타입을 안다.

bool FFTViewer::df_engine_ready() const {
    auto& K = kst();
    return K.engine && K.engine->running();
}

int FFTViewer::df_link_state() const {
    auto& K = kst();
    // (JOIN arm 제거 — 이 TU 는 CLI 전용이라 net_cli 는 항상 null. JOIN 쪽 정본은 df_join.cpp)
    if(!K.engine || !K.engine->running()) return 0;
    switch(K.engine->status().link){
        case df::LinkState::Streaming:   return 2;
        case df::LinkState::Calibrating: return 1;
        default:                         return 0;
    }
}

bool FFTViewer::df_submit(double center_hz, double bw_hz, int arr_idx, int dnum,
                          bool use_backlog){
    auto& K = kst();
    if(!K.engine || !K.engine->running()) return false;
    df::Config c = K.engine->config();
    df::Request q;
    q.center_hz    = center_hz;
    q.bandwidth_hz = bw_hz;
    q.frames       = c.avg_frames;
    q.algo         = c.algo;
    q.signal_dim   = c.signal_dim;
    q.ui_tag       = arr_idx;
    q.ui_dnum      = dnum;
    q.seq          = ++df_seq;
    q.use_backlog  = use_backlog;
    return K.engine->submit(q);
}

// 거절 사유도 결과와 같은 슬롯으로 보낸다. 드레인이 한 곳이면 채팅/로그/브로드
// 캐스트 경로가 갈라지지 않는다. UI 스레드에서만 불린다.
// 운용자에게 보이는 실패 사유는 딱 셋이다. 왜 셋뿐인가:
//   "Already running" — 다시 누르면 된다
//   "No signal"       — 신호가 임계에 못 미친다. 임계는 DF 탭에서 조정
//   "Not Available"   — 지금은 잴 수 없다 (대역 밖·캘리브레이션·필터 없음 등)
// 세 번째를 세분하면 운용자가 조치할 수 있는 게 늘지 않으면서 채팅만 시끄러워진다.
// 자세한 사유는 로그(detailed=true)와 DF 설정 패널이 그대로 보여준다.
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
    // 로그에는 원인을 남긴다 — 짧은 문구만으로는 사후 분석이 안 된다.
    // 다만 원인이 압축 문구와 같은 말이면 괄호를 붙이지 않는다
    // ("No signal (no signal)" 같은 중복 방지). 대소문자만 다른 것도 같게 본다.
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

bool FFTViewer::df_request_by_display_num(int dnum, bool from_auto){
    auto& K = kst();
    char msg[160];

    // 결과(또는 거절)가 나올 때까지 살아 있어야 한다 — df_pump 와 df_post_refusal
    // 둘 다 이걸 읽어 origin 을 채운다.
    df_req_auto = from_auto;

    if(hw.type != HWType::KRAKEN){
        snprintf(msg, sizeof msg, "Not Available");
        (void)hw.name;
        df_post_refusal(dnum, msg); return false;
    }
    if(!K.engine || !K.engine->running()){
        df_post_refusal(dnum, "Not Available"); return false;
    }

    // 표시번호는 freq_sorted_display_num 이 매 프레임 계산하는 주파수 정렬 순위지
    // 배열 인덱스가 아니다. 절대 인덱스로 폴백하지 말 것 — 필터를 주파수 순서대로
    // 만들지 않은 순간 엉뚱한 채널을 재게 된다.
    int arr = -1;
    for(int i = 0; i < MAX_CHANNELS; i++){
        if(!channels[i].filter_active) continue;
        if(freq_sorted_display_num(i) == dnum){ arr = i; break; }
    }
    if(arr < 0){
        df_post_refusal(dnum, "no such filter"); return false;
    }

    const double cf_mhz = (channels[arr].s + channels[arr].e) * 0.5;
    const double bw_hz  = std::fabs(channels[arr].e - channels[arr].s) * 1e6;
    if(bw_hz <= 0.0){
        df_post_refusal(dnum, "zero bandwidth"); return false;
    }

    // DAQ 스팬 밖이면 거절한다. update_dem_by_freq 가 Holding 으로 넘기는 것과
    // 같은 기준을 쓴다 — 오디오가 안 나오는 채널은 DF 도 안 된다.
    const double daq_cf = live_cf_hz.load(std::memory_order_acquire) / 1e6;
    const double half   = hw.sample_rate_mhz * 0.5;
    if(cf_mhz < daq_cf - half || cf_mhz > daq_cf + half){
        snprintf(msg, sizeof msg, "out of band (%.3f..%.3f MHz)",
                 daq_cf - half, daq_cf + half);
        df_post_refusal(dnum, msg); return false;
    }
    if(K.engine->armed()){
        df_post_refusal(dnum, "already running"); return false;
    }
    const df::DaqStatus st = K.engine->status();
    // 캘리브레이션 판정은 delay_sync_flag / iq_sync_flag 로만 한다 (link ==
    // Streaming 이 그 둘을 담고 있다). sync_state 는 추가로 보지 않는다.
    //
    // 왜: sync_state 5(TRACK_LOCK)는 보정이 이미 측정되고 적용된 상태이고,
    // 6(TRACK)은 그 뒤의 정상 추적 상태일 뿐이다. 5 에서 이미 두 플래그가 1 이
    // 되므로 6 을 요구하면 램프(플래그 기준)는 초록인데 요청은 거부되는
    // 한 프레임짜리 불일치가 생긴다 — 실측 확인: 초록 2.84s, 허용 2.95s.
    // heimdall 자신의 레퍼런스 클라이언트(iq_eth_sink.py)도 플래그만 본다.
    //
    // 재튠 직후 sync_state 가 이전 값 6 으로 잠깐 남아 있는 구간이 있는데
    // (실측 t=0.76s: DATA sst=6 ds=0 iq=0), 플래그 기준이라 그것도 올바르게
    // 거부된다 — sync_state 를 봤다면 오히려 통과시킬 뻔했다.
    if(st.link != df::LinkState::Streaming){
        snprintf(msg, sizeof msg, "calibrating (%u/6)", st.sync_state);
        df_post_refusal(dnum, msg); return false;
    }

    // AUTO 요청은 스컬치가 막 열린 것을 보고 나온다. 버스트 신호라면 그 사이
    // 송신이 끝났을 수 있으므로 엔진이 들고 있는 직전 프레임부터 적분하게 한다.
    if(!df_submit(cf_mhz * 1e6, bw_hz, arr, dnum, /*use_backlog=*/from_auto)){
        df_post_refusal(dnum, "busy"); return false;
    }
    return true;
}

void FFTViewer::df_pump(){
    auto& K = kst();
    if(!K.engine || !K.engine->running()) return;
    if(pending_df_result.pending.load(std::memory_order_acquire)) return;  // 아직 안 가져감

    df::Result r;
    if(!K.engine->poll(r)) return;

    auto& p = pending_df_result;
    p.dnum = r.ui_dnum; p.arr_idx = r.ui_tag;
    p.cf_mhz = (float)(r.center_hz / 1e6);
    p.bw_khz = (float)(r.bandwidth_hz / 1e3);
    p.bearing = (float)r.bearing_deg;
    p.bearing_rel = (float)r.bearing_rel_deg;
    p.conf = (float)r.confidence_db;
    p.snr = (float)r.eig_snr_db;
    p.pwr = (float)r.power_dbfs;
    p.frames_used = r.frames_used;
    p.frames_discarded = r.frames_discarded;
    p.overdrive = (r.overdrive_mask != 0);
    p.ok = (r.status == df::Status::Ok);
    snprintf(p.err, sizeof p.err, "%s",
             r.note[0] ? r.note : df::status_text(r.status));

    // ── 와이어로 나갈 나머지 ──────────────────────────────────────────────
    // 예전엔 여기서 전부 버렸다 (HOST 밖으로 나갈 길이 없었으니까). 이제
    // DF_RESULT 로 JOIN 에 실어보내므로 df::Result 를 온전히 옮겨 담는다.
    p.algo_papr      = (float)r.algo_papr_db;
    p.eff_bw_khz     = (float)(r.effective_bw_hz / 1e3);
    p.n_eff          = (float)r.n_eff_looks;
    p.ambiguity      = (float)r.ambiguity_ratio;
    p.diag_spread_db = (float)r.diag_spread_db;
    p.imbalance      = r.imbalance;
    p.elements       = r.elements;
    p.algo           = (int)r.algo;
    p.alt_n          = r.alt_n;
    for(int i = 0; i < 2; i++){
        p.alt_deg[i] = (float)r.alt_deg[i];
        p.alt_db[i]  = (float)r.alt_db[i];
    }
    p.t_end_ms = r.t_end_ms;
    p.dur_ms   = (int)std::min<int64_t>(r.t_end_ms - r.t_start_ms, 65535);
    p.from_auto = df_req_auto;

    // 의사스펙트럼 양자화. spectrum_db 는 이미 최대 정규화(전부 <=0)라
    // q = round(-2*dB) 로 0.5 dB/LSB, 하한 -127.5 dB.
    if(p.ok){
        for(int i = 0; i < 360; i++){
            long q = std::lround(-2.0 * (double)r.spectrum_db[i]);
            if(q < 0) q = 0; else if(q > 255) q = 255;
            p.spec_q[i] = (uint8_t)q;
        }
        p.has_spec = true;
    } else {
        p.has_spec = false;
    }

    p.pending.store(true, std::memory_order_release);
}

// ── DAQ 실시간 판독 (HOST) ───────────────────────────────────────────────
// 예전엔 HOST 의 DF 설정 패널이 이걸 그렸다. GUI 가 JOIN 전용이 되면서 호출자가
// 사라져 dead code 로 정리됐는데, 이제 DF_STATUS(0x61) 방송의 소스로 되살아난다 —
// 판독을 볼 수 있는 쪽(HOST)과 조작하는 쪽(JOIN)이 갈라졌기 때문에 실어보내야 한다.
void FFTViewer::df_get_live(FFTViewer::DFLive& o) const {
    auto& K = kst();
    o = FFTViewer::DFLive{};
    if(!K.engine || !K.engine->running()) return;
    const df::DaqStatus s = K.engine->status();
    switch(s.link){
        case df::LinkState::Streaming:   o.link = 2; break;
        case df::LinkState::Calibrating: o.link = 1; break;
        default:                         o.link = 0; break;
    }
    o.usable      = s.usable;
    o.sync_state  = s.sync_state;
    o.delay_sync  = s.delay_sync_flag;
    o.iq_sync     = s.iq_sync_flag;
    o.noise_src   = s.noise_source_state;
    o.channels    = s.active_ant_chs;
    o.overdrive   = s.adc_overdrive_flags;
    o.daq_cf_mhz  = s.rf_center_hz / 1e6;
    o.daq_fs_msps = s.sampling_hz  / 1e6;
    o.frame_rate_hz = s.frame_rate_hz;
    o.recv_mbps   = s.recv_mbps;
    o.frames_ok   = s.frames_ok;
    o.frames_cal  = s.frames_cal;
    o.frames_bad  = s.frames_bad;
    o.gaps        = s.cpi_gaps;
    o.reconnects  = s.reconnects;
    for(int i=0;i<8;i++) o.gain_tenths[i] = s.if_gain_tenths[i];
    snprintf(o.hw_id, sizeof o.hw_id, "%s", s.hardware_id);
    snprintf(o.last_error, sizeof o.last_error, "%s", s.last_error);
    o.measuring = K.engine->armed();
    o.progress  = K.engine->progress();

    // 격자엽 지표는 현재 DAQ 중심주파수 기준으로 보여준다 (측정은 채널 주파수를 쓴다).
    df::Config c = K.engine->config();
    if(s.rf_center_hz > 0){
        df::Manifold mf;
        mf.ensure((double)s.rf_center_hz, c.radius_m, c.elements, c.sense);
        o.lambda_m  = mf.lambda_m();
        o.ambiguity = mf.ambiguity_ratio();
    }
}

bool FFTViewer::df_measuring() const {
    auto& K = kst();
    return K.engine && K.engine->armed();
}

void FFTViewer::df_stop_engine(){
    auto& K = kst();
    if(K.engine) K.engine->stop();
}

// ── DF 설정 (HOST 소유, 전원 공유) ───────────────────────────────────────
// JOIN 은 자기 값을 갖지 않는다. 읽을 때는 HOST 가 방송한 정본을, 쓸 때는
// HOST 로 요청만 보낸다. 그래야 여러 JOIN 이 동시에 만져도 한 값으로 수렴하고,
// 무엇보다 측정을 실제로 하는 쪽(HOST)의 설정과 화면이 어긋나지 않는다.
static void cfg_to_pkt(const df::Config& c, PktDfConfig& p){
    p = PktDfConfig{};
    p.radius_m       = (float)c.radius_m;
    p.heading_deg    = (float)c.heading_deg;
    p.snr_thr_db     = (float)c.snr_threshold_db;
    p.dc_guard_hz    = (float)c.dc_guard_hz;
    p.c_papr         = (float)c.c_papr;
    p.target_looks   = (uint16_t)std::min(c.target_looks, 65535);
    p.fft_size       = (uint16_t)std::min(c.fft_size, 65535);
    p.elements       = (uint8_t)c.elements;
    p.algo           = (uint8_t)c.algo;
    p.sense          = (uint8_t)c.sense;
    p.avg_frames     = (uint8_t)c.avg_frames;
    p.max_frames     = (uint8_t)c.max_frames;
    p.signal_dim     = (uint8_t)c.signal_dim;
    p.enable_control = c.enable_control ? 1 : 0;
    p.gain_idx       = 255;   // 정본 송신 시엔 아래 df_get_cfg 가 실제값으로 채운다
}

static void pkt_to_cfg(const PktDfConfig& p, df::Config& c){
    c.radius_m         = p.radius_m;
    c.heading_deg      = p.heading_deg;
    c.snr_threshold_db = p.snr_thr_db;
    c.dc_guard_hz      = p.dc_guard_hz;
    c.c_papr           = p.c_papr;
    c.target_looks     = p.target_looks;
    c.fft_size         = p.fft_size;
    c.elements         = p.elements;
    c.algo             = (df::Algo)std::min<int>(p.algo, 2);
    c.sense            = (df::Sense)std::min<int>(p.sense, 1);
    c.avg_frames       = p.avg_frames;
    c.max_frames       = p.max_frames;
    c.signal_dim       = p.signal_dim;
    c.enable_control   = (p.enable_control != 0);
}

void FFTViewer::df_get_cfg(PktDfConfig& out) const {
    auto& K = kst();
    { std::lock_guard<std::mutex> lk(K.cfg_mtx); cfg_to_pkt(K.cfg, out); }
    // 게인은 df::Config 가 아니라 DAQ 가 들고 있다 (heimdall 소유). 상태에서
    // 실제 값을 읽어 인덱스로 바꿔 실어 보낸다 — JOIN 슬라이더가 현재값에서
    // 출발하려면 이 정본이 필요하다. ch0 기준 (5채널은 항상 같은 게인).
    if(K.engine && K.engine->running()){
        const df::DaqStatus st = K.engine->status();
        out.gain_idx = (uint8_t)HWConfig::rtl_gain_index((int)st.if_gain_tenths[0]);
    }
}

// 게인 적용. heimdall 은 5채널을 한 번에 받는다 — 채널마다 다르면 위상 기준이
// 깨져 방위가 통째로 틀어지므로 전 소자에 같은 값을 쓴다. DAQ 는 이 명령을 받으면
// 노이즈소스로 재캘리브레이션을 돌리므로 수 초간 DF 가 멈춘다 (Streaming -> Calibrating).
bool FFTViewer::df_set_gain_index(int idx, char* err, size_t errn){
    auto& K = kst();
    if(!K.engine || !K.engine->running()){
        if(err) snprintf(err, errn, "DF engine is not running");
        return false;
    }
    const int n = std::max(1, std::min((int)K.engine->status().active_ant_chs, 8));
    const int tenths = HWConfig::rtl_gain_tenths_at(idx);
    uint32_t per_ch[8];
    for(int i = 0; i < n; i++) per_ch[i] = (uint32_t)tenths;
    if(!K.engine->set_gain_tenths(per_ch, n, err, errn)) return false;
    gain_db = (float)tenths / 10.0f;   // 표시용 — 실제 정본은 다음 DAQ 상태에서 온다
    bewe_log_push(0, "[Kraken] gain -> %.1f dB (all %d ch) - DAQ recalibrating\n",
                  gain_db, n);
    return true;
}

void FFTViewer::df_set_cfg(const PktDfConfig& in){
    auto& K = kst();
    df::Config c;
    bool need_echo = false;
    {
        std::lock_guard<std::mutex> lk(K.cfg_mtx);
        c = K.cfg;
        pkt_to_cfg(in, c);
        char err[128] = {};
        if(!c.validate(err, sizeof err)){
            bewe_log_push(2,"[DF] config rejected: %s\n", err);
            need_echo = true;   // 정본을 되쏴 요청자 위젯을 진실로 되돌린다
        }
        if(!need_echo) K.cfg = c;
    }
    // 거절이면 K.cfg 는 그대로다 — 그 값을 다시 방송해 JOIN 이 유령값을 붙들지 않게 한다.
    if(need_echo){ df_broadcast_cfg(); return; }
    if(K.engine) K.engine->apply_config(c);
    df_broadcast_cfg();
}

void FFTViewer::df_broadcast_cfg() const {
    if(!net_srv) return;
    PktDfConfig p{};
    df_get_cfg(p);
    net_srv->broadcast_df_config(p);
}

double FFTViewer::df_snr_threshold() const {
    PktDfConfig p{}; df_get_cfg(p);
    return p.snr_thr_db;
}


