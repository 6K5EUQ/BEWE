// ── AMC HOST 워커: IQ ring 독립 read-ptr 탭 → 채널 DDC → 버스트 캡처 → 추론 ──
//
// 다른 모듈 워커와 달리 **상시 복조하지 않는다.** 평소엔 ring 을 따라가며 버리기만
// 하고, 스퀄치가 열리는 순간(=detect lock)에만 고정 길이 버스트를 모아 한 번 잰다.
// 변조 판별은 짧은 표본 하나면 충분하고, 계속 재 봐야 CPU 만 먹는다.
#include "fft_viewer.hpp"
#include "amc_module.hpp"
#include "amc_ai.hpp"
#include "module_api.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

namespace amc_mod {

static int64_t now_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// 모델에 보낼 복소 표본 수. 고정이다 — 모델의 nsamp(현재 2048)와 일부러 다르게 크게
// 잡고, 크롭/리샘플은 Python 이 한다. 그래야 재학습으로 nsamp 이 바뀌어도 C++ 을 다시
// 컴파일할 필요가 없다. 4096*8B + 28B 헤더 = 32796 B (프레임 상한 64KB 안).
static constexpr int AMC_CAP = 4096;
// 같은 채널에서 이 간격 안에는 다시 재지 않는다. detect 는 hold 창(360ms) 경계에서
// lock↔release 가 튈 수 있어, 없으면 한 교신에 추론이 수십 번 돈다.
static constexpr int64_t AMC_MIN_GAP_MS = 500;
// 스퀄치가 계속 열려 있는 연속 신호(방송 등)에서 다시 재는 주기. 버스트는 상승엣지가
// 매번 트리거하므로 이 주기와 무관하다. 1초로 잡은 이유: 추론 1회가 DDC ~4096 표본 +
// UDS 왕복 10 ms 라 채널당 1 Hz 면 코어 1개의 1 % 수준이고, 그 대신 판정이 한 번의
// 2.4 ms 스냅샷에 걸리지 않고 여러 표본으로 평균된다 (분산 감소).
static constexpr int64_t AMC_PERIOD_MS = 1000;
// 데시메이션 하한. 출력 48 kHz 이상을 보장해 4096 표본이 항상 ≤85 ms 안에 차게 한다 —
// detect hold 창(360 ms)보다 넉넉히 짧아야 lock 이 풀리기 전에 캡처가 끝난다.
static constexpr double AMC_MIN_OUT_SR = 48000.0;
// 대역내 SNR 하한. 이보다 낮으면 스퀄치가 열려 있어도 재지 않는다 — 분류기가
// 잡음을 넣으면 FM 이라고 답하기 때문이다(잡음의 순시주파수 분산이 광대역 FM 과
// 닮았다). 6 dB 는 잡음바닥 추정 자체의 흔들림(하위 25% 분위수, ~1 dB)보다 충분히
// 크면서, 약한 실신호를 버리지 않는 선이다.
static constexpr float AMC_MIN_SNR_DB = 6.0f;

void worker(FFTViewer& v, int ch_idx){
    Channel& ch = v.channels[ch_idx];
    uint32_t msr = v.header.sample_rate;
    const float inv_scale = 1.0f/v.hw.iq_scale;
    uint64_t init_cf = v.live_cf_hz.load(std::memory_order_acquire);
    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(init_cf/1e6f)) * 1e6f;
    float bw_hz  = fabsf(ch.e-ch.s) * 1e6f;
    if(bw_hz < 1.0f) bw_hz = 1.0f;

    // ── DDC: 채널폭의 4배 근처로 데시메이트 (변조 판별에 충분한 오버샘플) ──
    auto calc_decim = [&](float bwh)->uint32_t{
        double want = 4.0 * (double)bwh;
        if(want < AMC_MIN_OUT_SR) want = AMC_MIN_OUT_SR;
        uint32_t d = (uint32_t)std::max(1.0, std::floor((double)msr / want));
        if(d < 1) d = 1;
        return d;
    };
    uint32_t decim  = calc_decim(bw_hz);
    double   fs_out = (double)msr / decim;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;
    float    prev_center = (ch.s+ch.e)/2.0f;
    float    prev_bw = bw_hz;

    IIR1 lpi[4], lpq[4];
    auto set_lpf = [&](float bwh, double fso){
        float cut = std::min(bwh*0.5f, (float)fso*0.45f);
        float cn = cut/(float)msr; if(cn>0.45f)cn=0.45f; if(cn<0.005f)cn=0.005f;
        for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); }
    };
    set_lpf(bw_hz, fs_out);
    double dec_i=0, dec_q=0; uint32_t dec_cnt=0;

    // 캡처 버퍼 (interleaved I/Q float)
    std::vector<float> cap; cap.resize((size_t)AMC_CAP*2);
    int   cap_have = 0;
    bool  cap_arm  = false;
    uint8_t cap_trig = AMC_TRIG_SQUELCH;
    int64_t cap_t_ms = 0;
    float   cap_snr  = 0.f;
    int64_t last_infer_ms = 0;
    int64_t last_diag_ms  = 0;
    size_t  last_diag_wp  = 0;

    const size_t MAX_LAG = ring_max_lag(msr, v.hw.burst_samples);
    const size_t BATCH   = std::max<size_t>(4096, msr/50);
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());

    bewe_log_push(0,"AMC[%d] start: %.4f MHz  BW=%.1f kHz  decim=%u fs_out=%.1f kHz%s\n",
        ch_idx,(ch.s+ch.e)/2.0f, bw_hz/1000.f, decim, fs_out/1e3,
        amc_ai_enabled() ? "" : "  (AI off - capture only)");

    auto reset_dsp = [&](){
        for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
        dec_i=dec_q=0; dec_cnt=0; cap_have=0; cap_arm=false;
    };

    bool gate_prev = false, hold_prev = false;
    while(!worker_stop_req(ch_idx) && !v.sdr_stream_error.load() && ch.filter_active){
        // Holding(가시대역 밖) → DDC 정지. 안 하면 신호도 없는 대역을 풀레이트로 계속
        // 돌려 CLI 기지 CPU/배터리를 태운다.
        bool hold = ch.dem_paused.load(std::memory_order_relaxed);
        if(hold!=hold_prev){
            bewe_mod_host_ch_hold(ch_idx, hold);
            if(hold) reset_dsp();
            hold_prev=hold;
        }
        if(hold){
            my_rp.store(v.ring_wp.load(std::memory_order_acquire), std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            gate_prev = false;
            continue;
        }

        // 재튜닝/폭변경 추종
        { uint64_t cur=v.live_cf_hz.load(std::memory_order_acquire);
          float cc=(ch.s+ch.e)/2.0f, cb=fabsf(ch.e-ch.s)*1e6f; if(cb<1.f) cb=1.f;
          if(cur!=prev_cf || cc!=prev_center){
              off_hz=(cc-(float)(cur/1e6))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur; prev_center=cc;
              cap_have=0; cap_arm=false;            // 진행 중 캡처는 버린다 (주파수가 바뀜)
          }
          if(fabsf(cb-prev_bw) > prev_bw*0.02f){    // 폭이 2% 넘게 바뀌면 DDC 재설정
              bw_hz=cb; prev_bw=cb;
              decim=calc_decim(bw_hz); fs_out=(double)msr/decim;
              set_lpf(bw_hz, fs_out); reset_dsp();
          }
        }

        // ── 트리거 ──────────────────────────────────────────────────────────
        // detect 채널에서 sq_gate 는 det_locked 일 때만 열린다 = 버스트 시작 시점.
        // Channel::sq_gate_prev 는 GUI 페이드용이라 건드리면 안 된다 — 로컬 변수로 엣지를 만든다.
        bool gate = ch.sq_gate.load(std::memory_order_relaxed);
        int64_t tnow = now_ms();
        // 트리거는 오직 스퀄치다 (detect 여부와 무관).
        //   버스트   : 스퀄치를 넘는 순간마다 1회 — 짧아도 놓치지 않는다.
        //   연속신호 : 넘고 있는 동안 AMC_PERIOD_MS(1초) 주기로 계속.
        // 신호가 없으면(게이트 닫힘) 아무것도 안 한다 — 잡음을 분류해 봐야 표만 더럽다.
        // **캡처 중이면 절대 재무장하지 않는다.** 이걸 빠뜨리면 periodic 조건이 매
        // 루프마다 참이 되어(완료 전이라 last_infer_ms 가 안 갱신됨) 매 반복이 캡처를
        // 리셋한다 — 영원히 have=0 이고 한 건도 못 잰다 (2026-08-04 실측).
        const bool rising  = gate && !gate_prev;
        const bool periodic= gate && (tnow - last_infer_ms >= AMC_PERIOD_MS);
        // 잡음 배제 — 스퀄치가 열려 있어도 대역내 SNR 이 낮으면 재지 않는다.
        // AIS 처럼 슬롯이 26.7 ms 인 버스트 채널은 대부분의 시간이 빈 채널인데
        // 스퀄치는 계속 열려 있어, 그대로 두면 표본의 87% 가 잡음이 된다 (2026-08-04
        // 실측: ch4 345건 중 301건이 잡음권, 그 82% 가 FM 으로 분류됐다). 잡음을
        // 평균에 넣으면 진짜 버스트가 묻혀 평균이 오히려 나빠진다.
        const float snr_now = ch.sq_sig.load(std::memory_order_relaxed)
                            - ch.sq_nf.load(std::memory_order_relaxed);
        const bool  snr_ok  = snr_now >= AMC_MIN_SNR_DB;
        if(!cap_arm && snr_ok && (rising || periodic) && tnow - last_infer_ms > AMC_MIN_GAP_MS){
            cap_have = 0; cap_arm = true;
            cap_trig = AMC_TRIG_SQUELCH;
            cap_t_ms = tnow;
            cap_snr  = snr_now;
            // ring 이 조용하던 동안 rp 가 뒤처져 있다. 여기서 안 당기면 첫 캡처가
            // 수 초 전 노이즈가 된다.
            my_rp.store(v.ring_wp.load(std::memory_order_acquire), std::memory_order_release);
            reset_dsp();
            cap_arm = true;   // reset_dsp 가 껐으므로 다시 켠다
        }
        gate_prev = gate;

        // 왜 안 재는지 알 수 없으면 운용자가 안테나·게인·모델을 차례로 의심하며
        // 시간을 버린다. 30초 넘게 한 건도 못 쟀을 때만 그 이유를 한 줄 남긴다.
        if(tnow - last_diag_ms >= 30000){
            last_diag_ms = tnow;
            if(last_infer_ms == 0 || tnow - last_infer_ms > 30000){
                size_t wp_d=v.ring_wp.load(std::memory_order_acquire);
                size_t adv=(wp_d-last_diag_wp)&IQ_RING_MASK;   // 30초 동안 ring 이 얼마나 돌았나
                bewe_log_push(2,"AMC[%d] idle: gate=%d sig=%.1f nf=%.1f SNR=%.1f(min %.0f) thr=%.1f dB  ring_adv=%zu  armed=%d have=%d/%d%s\n",
                    ch_idx, gate?1:0,
                    ch.sq_sig.load(std::memory_order_relaxed),
                    ch.sq_nf.load(std::memory_order_relaxed),
                    snr_now, (double)AMC_MIN_SNR_DB,
                    ch.sq_threshold.load(std::memory_order_relaxed),
                    adv, cap_arm?1:0, cap_have, AMC_CAP,
                    adv==0 ? "  (IQ ring not being filled)"
                           : (gate ? "" : "  (squelch closed)"));
                last_diag_wp = wp_d;
            }
        }

        size_t wp=v.ring_wp.load(std::memory_order_acquire);
        size_t rp=my_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        if(lag>MAX_LAG){                                    // 과부하 → 경계 점프
            size_t keep = ring_keep_after_lag(msr, v.hw.burst_samples);
            rp=(wp-keep)&IQ_RING_MASK; my_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            dec_i=dec_q=0; dec_cnt=0;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }

        // 무장 안 됐으면 DDC 를 아예 돌리지 않는다 — rp 만 밀어 버린다.
        // 상시 복조가 목적이 아니므로 이게 절감의 핵심이다.
        if(!cap_arm){
            my_rp.store((rp+lag)&IQ_RING_MASK, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail && cap_arm;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]*inv_scale, sq=v.ring[pos*2+1]*inv_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            mi=lpi[0].p(mi); mi=lpi[1].p(mi); mi=lpi[2].p(mi); mi=lpi[3].p(mi);
            mq=lpq[0].p(mq); mq=lpq[1].p(mq); mq=lpq[2].p(mq); mq=lpq[3].p(mq);
            dec_i+=mi; dec_q+=mq;
            if(++dec_cnt>=decim){
                float oi=(float)(dec_i/decim), oq=(float)(dec_q/decim);
                dec_i=dec_q=0; dec_cnt=0;
                cap[(size_t)cap_have*2  ]=oi;
                cap[(size_t)cap_have*2+1]=oq;
                if(++cap_have>=AMC_CAP) cap_arm=false;      // 다 찼다
            }
        }
        my_rp.store((rp+avail)&IQ_RING_MASK, std::memory_order_release);

        if(!cap_arm && cap_have>=AMC_CAP){
            cap_have = 0;
            last_infer_ms = now_ms();
            AmcRecord m{};
            m.t_ms   = cap_t_ms;
            m.ch     = ch_idx;
            m.freq   = (ch.s+ch.e)/2.0f;
            m.bw_khz = bw_hz/1000.f;
            m.snr_db = cap_snr;
            m.trig   = cap_trig;
            int cls=-1, cls2=-1; float conf=0.f, conf2=0.f;
            if(amc_ai_infer(ch_idx, m.t_ms, (uint32_t)llround(fs_out), (uint32_t)llround(bw_hz),
                            cap_trig, cap.data(), AMC_CAP,
                            cls, conf, cls2, conf2, m.model, sizeof(m.model),
                            m.p, AMC_NCLASS)){
                m.cls=cls; m.conf=conf; m.cls2=cls2; m.conf2=conf2;
                host_emit(v, m);
            }
            // 판정 실패(데몬 없음/타임아웃)면 레코드를 만들지 않는다 — 빈 줄이 쌓이면
            // 표가 쓰레기로 찬다. 실패 원인은 데몬 로그에 남는다.
        }
    }
    worker_natural_exit(v, ch_idx);
}

} // namespace amc_mod
