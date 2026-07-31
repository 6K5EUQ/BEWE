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

    // ch0 스테이징: 엔진 스레드가 채우고 캡처 스레드가 소비하는 한 칸.
    // 42 MB 짜리 프레임 전체가 아니라 ch0 만, 그것도 int16 로 줄여 담는다
    // (1048576 샘플 -> 4 MB). 큐를 두지 않는 이유는 스펙트럼이 최신 프레임만
    // 필요하기 때문 — 밀리면 오래된 걸 그리느니 버리는 게 맞다.
    std::mutex              mtx;
    std::condition_variable cv;
    std::vector<int16_t>    stage;      // interleaved I/Q
    size_t                  stage_n = 0;
    bool                    ready   = false;
    uint64_t                cf_hz = 0, fs_hz = 0;
    uint32_t                overdrive = 0;
    uint64_t                dropped = 0;   // 캡처가 못 따라가 버린 프레임 수
};

KrakenState& kst(){ static KrakenState s; return s; }

// heimdall 은 공칭 +-1.0 의 complex float32 를 준다. ring/TM/demod/IQ녹음은
// 전부 int16 interleaved 를 전제하므로 변환이 필요하다. BladeRF/Pluto 와 같은
// SC16_Q11 스케일(2048)로 맞춰야 dB 눈금 / 스퀠치 임계 / 녹음 파일이 백엔드마다
// 달라지지 않는다. +-1.0 -> +-2048 은 int16 포화까지 24 dB 여유.
inline int16_t kr_f2i(float f){
    const float s = f * 2048.0f;
    return (int16_t)(s > 32767.f ? 32767 : (s < -32768.f ? -32768 : (int)lrintf(s)));
}

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
    K.engine->set_ch0_sink([](const std::complex<float>* ch0, size_t n,
                              uint64_t cf, uint64_t fs, uint32_t od, int64_t){
        auto& S = kst();
        std::unique_lock<std::mutex> lk(S.mtx, std::try_to_lock);
        if(!lk.owns_lock()){ S.dropped++; return; }   // 캡처가 붙잡고 있으면 이번 프레임은 버린다
        if(S.ready){ S.dropped++; }                   // 아직 안 가져갔으면 최신 것으로 덮는다
        if(S.stage.size() < n*2) S.stage.resize(n*2);
        for(size_t i = 0; i < n; i++){
            S.stage[i*2+0] = kr_f2i(ch0[i].real());
            S.stage[i*2+1] = kr_f2i(ch0[i].imag());
        }
        S.stage_n = n; S.cf_hz = cf; S.fs_hz = fs; S.overdrive = od;
        S.ready = true;
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

    while(is_running && !sdr_stream_error.load(std::memory_order_relaxed)){
        if(capture_pause.load(std::memory_order_relaxed)){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            rx_pos=0; rx_avail=0;
            continue;
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
                                  [&]{ return K.ready || !is_running; }))
                    continue;                      // 타임아웃 — 루프 조건 다시 검사
                if(!K.ready) continue;
                n = K.stage_n;
                if(iq16.size() < n*2) iq16.resize(n*2);
                memcpy(iq16.data(), K.stage.data(), n*2*sizeof(int16_t));
                K.ready = false;
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
                int fi=total_ffts%FFT_HISTORY_ROWS;
                float* rowp=fft_data.data()+fi*fft_size;
                {std::lock_guard<std::mutex> lk(data_mtx);
                 volk_32f_log2_32f(rowp, pacc.data(), (unsigned int)fft_size);
                 volk_32f_s32f_multiply_32f(rowp, rowp, 3.01029996f, (unsigned int)fft_size);
                 const float row_off = 10.0f*log10f((float)fcnt);
                 for(int i=0;i<fft_size;i++) rowp[i] -= row_off;

                 if(autoscale_req.exchange(false)){
                     autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
                     autoscale_wp=0; autoscale_buf_full=false;
                     sq_recalib_req.store(true, std::memory_order_relaxed);
                 }
                 if(autoscale_active){
                     auto now_as=std::chrono::steady_clock::now();
                     if(autoscale_start==std::chrono::steady_clock::time_point{})
                         autoscale_start=now_as;
                     if(!autoscale_init){
                         size_t cap=(size_t)fft_size*100;
                         if(autoscale_accum.size()!=cap) autoscale_accum.assign(cap,0.0f);
                         autoscale_wp=0; autoscale_buf_full=false;
                         autoscale_last=now_as;
                         autoscale_init=true;
                     }
                     size_t cap=autoscale_accum.size();
                     for(int i=1;i<fft_size;i++){
                         autoscale_accum[autoscale_wp]=rowp[i];
                         if(++autoscale_wp>=cap){ autoscale_wp=0; autoscale_buf_full=true; }
                     }
                     float el=std::chrono::duration<float>(now_as-autoscale_last).count();
                     float el_total=std::chrono::duration<float>(now_as-autoscale_start).count();
                     bool  deadline=el_total>=AUTOSCALE_DEADLINE_S;
                     if((el>=1.0f||deadline)&&(autoscale_buf_full||autoscale_wp>0)){
                         size_t nn=autoscale_buf_full?cap:autoscale_wp;
                         std::vector<float> tmp(autoscale_accum.begin(),
                                                autoscale_accum.begin()+(ptrdiff_t)nn);
                         size_t idx_lo=(size_t)(nn*0.15f);
                         std::nth_element(tmp.begin(),tmp.begin()+(ptrdiff_t)idx_lo,tmp.end());
                         float noise=tmp[idx_lo];
                         float peak=*std::max_element(tmp.begin(),tmp.end());
                         display_power_min=noise-5.0f;
                         display_power_max=peak+20.0f;
                         if(display_power_max-display_power_min<20.f)
                             display_power_max=display_power_min+20.f;
                         header.power_min=display_power_min;
                         header.power_max=display_power_max;
                         bewe_log_push(0,"[autoscale]%s noise=%.1f peak=%.1f > pmin=%.1f pmax=%.1f\n",
                             deadline?" (deadline)":"", noise, peak, display_power_min, display_power_max);
                         autoscale_active=false; autoscale_init=false;
                         autoscale_wp=0; autoscale_buf_full=false;
                         autoscale_start=std::chrono::steady_clock::time_point{};
                         cached_sp_idx=-1;
                     }
                 }
                 total_ffts++; current_fft_idx=total_ffts-1;
                 header.num_ffts=std::min(total_ffts,FFT_HISTORY_ROWS);
                 row_write_pos[current_fft_idx%MAX_FFTS_MEMORY]=tm_iq_write_sample;
                 row_wall_ms[current_fft_idx%MAX_FFTS_MEMORY]=(int64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                 if(tm_iq_on.load(std::memory_order_relaxed))
                     tm_mark_rows(current_fft_idx%MAX_FFTS_MEMORY);
                 else
                     iq_row_avail[current_fft_idx%MAX_FFTS_MEMORY]=false;
                 tm_add_time_tag(current_fft_idx);
                 net_bcast_seq.fetch_add(1, std::memory_order_release);
                 net_bcast_cv.notify_one();
                }
                std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;

                // 행 하나가 담는 실제 시간만큼 기다렸다 다음 행을 낸다.
                // 정지/비표시 상태에선 페이싱하지 않는다 (아래 조건 밖).
                if(hw.sample_rate > 0){
                    const double row_s = (double)fft_input_size * (double)time_average
                                       / (double)hw.sample_rate;
                    row_due += std::chrono::microseconds((long long)(row_s * 1e6));
                    const auto now_r = std::chrono::steady_clock::now();
                    if(row_due > now_r){
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
    // JOIN 은 HOST 의 DF 를 원격으로 쓴다. HOST 의 백엔드 종류(remote_hw==3)까지는
    // STATUS 패킷으로 알 수 있지만 캘리브레이션 상태까지는 모른다 — 그래서 초록이
    // 아니라 노랑으로 둔다. 거짓 초록보다 정직한 노랑이 낫다.
    if(net_cli){
        if(!net_cli->is_connected()) return 0;
        // HOST 가 하트비트로 실제 가용도를 보내준다 (0=불가 1=가능 2=준비중).
        // 여기 값 체계는 이 함수의 반환 규약(0=down 1=calibrating 2=streaming)과
        // 다르므로 옮겨 담는다.
        const uint8_t d = net_cli->remote_df_state.load();
        return (d == 1) ? 2 : (d == 2 ? 1 : 0);
    }
    if(!K.engine || !K.engine->running()) return 0;
    switch(K.engine->status().link){
        case df::LinkState::Streaming:   return 2;
        case df::LinkState::Calibrating: return 1;
        default:                         return 0;
    }
}

bool FFTViewer::df_submit(double center_hz, double bw_hz, int arr_idx, int dnum){
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
        snprintf(out, n, "CH%d: %.1f\u00B0 (SNR : %.1fdB)%s",
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
    snprintf(p.err, sizeof p.err, "%s", msg);
    p.pending.store(true, std::memory_order_release);
}

bool FFTViewer::df_request_by_display_num(int dnum){
    auto& K = kst();
    char msg[160];

    // ── JOIN: 로컬에 SDR 이 없으니 HOST 에 대신 시킨다 ──────────────────
    // 채널 배열은 CHANNEL_SYNC 로 동기화되어 표시번호가 HOST 와 같으므로,
    // 번호를 그대로 넘기면 된다. 결과는 HOST 가 broadcast_chat("DF", ...) 로
    // 모두에게 돌려주므로 여기서 따로 받을 게 없다.
    if(net_cli){
        if(!net_cli->is_connected()){
            df_post_refusal(dnum, "no link"); return false;
        }
        if(net_cli->remote_hw.load() != 3){
            df_post_refusal(dnum, "Not Available");
            return false;
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
        net_cli->cmd_df_measure(dnum);
        return true;
    }

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

    if(!df_submit(cf_mhz * 1e6, bw_hz, arr, dnum)){
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

    if(p.ok){
        df_last_valid = true;
        df_last_dnum = r.ui_dnum;
        df_last_cf_mhz = p.cf_mhz; df_last_bw_khz = p.bw_khz;
        df_last_bearing = p.bearing; df_last_bearing_rel = p.bearing_rel;
        df_last_conf = p.conf; df_last_snr = p.snr; df_last_pwr = p.pwr;
        memcpy(df_last_spectrum, r.spectrum_db, sizeof df_last_spectrum);
    }
    p.pending.store(true, std::memory_order_release);
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
    if(net_cli){
        if(net_cli->df_cfg_valid.load()){
            std::lock_guard<std::mutex> lk(net_cli->df_cfg_mtx);
            out = net_cli->df_cfg;
            return;
        }
        // 아직 HOST 방송을 못 받았으면 기본값을 보여준다 (빈 화면보다 낫다).
        cfg_to_pkt(df::Config{}, out);
        return;
    }
    auto& K = kst();
    std::lock_guard<std::mutex> lk(K.cfg_mtx);
    cfg_to_pkt(K.cfg, out);
}

void FFTViewer::df_set_cfg(const PktDfConfig& in){
    if(net_cli){ net_cli->send_df_config(in); return; }   // 요청만. 정본은 HOST 가 돌려준다
    auto& K = kst();
    df::Config c;
    {
        std::lock_guard<std::mutex> lk(K.cfg_mtx);
        c = K.cfg;
        pkt_to_cfg(in, c);
        char err[128] = {};
        if(!c.validate(err, sizeof err)){
            bewe_log_push(2,"[DF] config rejected: %s\n", err);
            return;
        }
        K.cfg = c;
    }
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

