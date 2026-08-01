// ── ACARS HOST 워커: IQ ring 독립 read-ptr 탭 → AM 포락선 → 스트리밍 디코더 ──
// demod_worker / 오디오 ring 과 완전 분리. 디코드 결과는 host_emit() 으로
// 로그 + 일 단위 저장 + 전 JOIN 브로드캐스트.
#include "fft_viewer.hpp"
#include "acars_module.hpp"
#include "acars_decode.hpp"
#include "module_api.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <thread>

namespace acars_mod {

// acars_module.cpp 의 워커 슬롯 접근자
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

// 무신호 DSP 스킵 킬스위치 (BEWE_ACARS_SQGATE=0 이면 항상 풀레이트 — 구동작).
// 1회 평가 캐시. 약신호 유실이 의심되면 이걸로 즉시 되돌린다.
static bool sqgate_enabled(){
    static int c=-1;
    if(c<0){ const char* e=getenv("BEWE_ACARS_SQGATE"); c=(e && e[0]=='0') ? 0 : 1; }
    return c==1;
}

void worker(FFTViewer& v, int ch_idx){
    Channel& ch=v.channels[ch_idx];
    uint32_t msr=v.header.sample_rate;
    const float inv_scale=1.0f/v.hw.iq_scale;  // ÷ → ×
    uint64_t init_cf=v.live_cf_hz.load(std::memory_order_acquire);
    float off_hz=(((ch.s+ch.e)/2.0f)-(float)(init_cf/1e6f))*1e6f;
    float bw_hz=fabsf(ch.e-ch.s)*1e6f;

    uint32_t inter_sr,audio_decim,cap_decim;
    demod_rates(msr,bw_hz,inter_sr,audio_decim,cap_decim);
    uint32_t actual_inter=msr/cap_decim;
    uint32_t actual_ad=std::max(1u,(uint32_t)round((double)actual_inter/AUDIO_SR));
    uint32_t actual_asr=actual_inter/actual_ad;

    Oscillator osc; osc.set_freq((double)off_hz,(double)msr);
    uint64_t prev_cf=init_cf;
    double cap_i=0,cap_q=0; int cap_cnt=0;
    IIR1 lpi[4],lpq[4];
    { float cn=(bw_hz*0.5f)/(float)msr; if(cn>0.45f)cn=0.45f;
      for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); } }
    float am_dc=0;
    float am_dc_alpha=1.0f-expf(-2.0f*M_PI*30.0f/(float)actual_inter);
    IIR1 alf; alf.set(std::min(3000.0f, bw_hz*0.5f) / (float)actual_inter); // ~3 kHz audio LPF (ACARS tones <=2400 Hz)
    double aac=0; int acnt=0;

    AcarsDecoder dec;
    dec.on_record=[&v](const AcarsMsg& m){ host_emit(v, m); };
    dec.reset((float)actual_asr, ch_idx);
    bewe_log_push(0,"ACARS[%d] start: %.4f MHz  BW=%.1fkHz  asr=%u\n",
        ch_idx,(ch.s+ch.e)/2.0f,bw_hz/1000.f,actual_asr);

    const size_t MAX_LAG = ring_max_lag(msr, v.hw.burst_samples);
    const size_t BATCH  =(size_t)cap_decim*actual_asr/50;
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());

    // 무신호 스킵용 pre-roll: 게이트 열림 시 되감을 샘플 수.
    // 링 용량(IQ_RING_CAPACITY)의 1/4 로 클램프 — 고SR(bladeRF 61.44M 등)에서는 링이
    // 짧아(1<<22 샘플 = 68ms) 300ms 되감기가 랩어라운드로 미래(스테일) 샘플을 읽게 됨.
    const size_t PREROLL = std::min((size_t)((double)msr * (DEC_GATE_PREROLL_MS/1000.0)),
                                    (size_t)(IQ_RING_CAPACITY/4));
    const bool   USE_GATE = sqgate_enabled();
    bool idle_skip=false;      // 무신호 스킵 상태
    bool catching_up=false;    // 되감기 직후 백로그 소화 중 (MAX_LAG 리미터 우회)
    bool ring_live=false;      // 스킵 중 ring 이 실제로 채워지고 있는가 (되감기 안전조건)
    size_t wp_idle_last=0;
    auto   catchup_start=std::chrono::steady_clock::now();

    bool hold_prev=false;
    while(!worker_stop_req(ch_idx) && !v.sdr_stream_error.load() && ch.filter_active){
        // 가시대역 밖(Holding) → 복조 불가 → DDC 정지(연산/배터리 절약). 진입 edge 에서 상태 리셋
        // (디코더 framing/sync 잔류 → 복귀 시 false frame 방지) + runtime 누적 freeze.
        bool hold = ch.dem_paused.load(std::memory_order_relaxed);
        if(hold!=hold_prev){
            bewe_mod_host_ch_hold(ch_idx, hold);
            if(hold){ for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; } alf.s=0; am_dc=0;
                      cap_i=cap_q=0; cap_cnt=0; aac=0; acnt=0; dec.reset((float)actual_asr, ch_idx); }
            hold_prev=hold;
        }
        if(hold){
            my_rp.store(v.ring_wp.load(std::memory_order_acquire), std::memory_order_release);
            // Holding 중엔 dem_run 이 내려가 ring 공급이 끊길 수 있음 → 복귀 시 되감기 금지
            // (되감으면 링에 남은 스테일 샘플을 읽게 됨). 스킵 상태도 여기서 청산.
            idle_skip=false; catching_up=false;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        { uint64_t cur=v.live_cf_hz.load(std::memory_order_acquire);
          if(cur!=prev_cf){
              off_hz=(((ch.s+ch.e)/2.0f)-(float)(cur/1e6))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur;
          }
        }

        // ── 무신호(dec_gate 닫힘) → 풀레이트 DDC(믹서+8단 IIR+박스카) 스킵 ──────
        //    dec_gate 는 FFT 스레드가 raw 행 peak 로 독립 계산(thr-6dB, 2초 hold)하므로
        //    DSP 를 멈춰도 신호 복귀 감지에 영향 없음. demod.cpp 의 스컬치 스킵과 동형.
        //    단 ACARS 는 버스트 선두가 곧 프리앰블이라, 열림 시 ring 을 PREROLL 만큼
        //    되감아 FFT 검출지연(행주기+EMA, ~50-100ms)을 흡수한다 → 버스트 무손실.
        if(USE_GATE && !ch.dec_gate.load(std::memory_order_relaxed)){
            size_t wp_now=v.ring_wp.load(std::memory_order_acquire);
            my_rp.store(wp_now, std::memory_order_release);
            if(!idle_skip){   // 진입 edge: DSP/디코더 상태 리셋 (잔류 → false frame 방지)
                for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
                alf.s=0; am_dc=0; cap_i=cap_q=0; cap_cnt=0; aac=0; acnt=0;
                dec.reset((float)actual_asr, ch_idx);
                idle_skip=true; ring_live=false; wp_idle_last=wp_now;
            } else if(wp_now != wp_idle_last){
                ring_live=true;              // 캡처가 ring 을 계속 채우는 중 → 되감기 안전
                wp_idle_last=wp_now;
            }
            catching_up=false;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        if(idle_skip){
            // 닫힘→열림 edge: PREROLL 만큼 되감아 버스트 선두(프리앰블) 복원.
            // 단 ring 이 실제로 채워지고 있을 때만 — need_ring 이 꺼져 wp 가 얼어 있으면
            // 되감기가 과거가 아닌 스테일 샘플을 읽어 가짜 디코드를 낳는다.
            if(ring_live){
                size_t wp0=v.ring_wp.load(std::memory_order_acquire);
                my_rp.store((wp0-PREROLL)&IQ_RING_MASK, std::memory_order_release);
                catching_up=true;   // 되감은 백로그(>MAX_LAG)를 리미터가 도로 버리지 않도록
                catchup_start=std::chrono::steady_clock::now();
            }
            idle_skip=false;
        }

        size_t wp=v.ring_wp.load(std::memory_order_acquire);
        size_t rp=my_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        // catch-up 중에는 리미터 상한을 넓힘 (되감기 백로그 소화 목적).
        // 그래도 못 따라잡으면(워커가 실시간 미달) 상한 초과 시 정상 클램프로 안전 복귀.
        size_t lag_limit = catching_up ? std::min((size_t)(IQ_RING_CAPACITY/2),
                                                  PREROLL + MAX_LAG) : MAX_LAG;
        if(lag>lag_limit){
            size_t keep = ring_keep_after_lag(msr, v.hw.burst_samples);
            rp=(wp-keep)&IQ_RING_MASK;
            my_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            am_dc=0; cap_i=cap_q=0; cap_cnt=0;
            lag=(wp-rp)&IQ_RING_MASK;
            catching_up=false;
        }
        // 백로그 소화 완료, 또는 시간제한(1s) 초과 — 워커가 실시간에 못 미치면
        // 넓힌 리미터에 영구히 머물며 오래된 IQ 를 계속 디코드하게 되므로 강제 해제.
        if(catching_up && (lag<=MAX_LAG
            || std::chrono::steady_clock::now()-catchup_start > std::chrono::seconds(1)))
            catching_up=false;
        if(lag==0){ std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]*inv_scale, sq=v.ring[pos*2+1]*inv_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            float mi_aa=mi, mq_aa=mq;
            mi_aa=lpi[0].p(mi_aa); mi_aa=lpi[1].p(mi_aa); mi_aa=lpi[2].p(mi_aa); mi_aa=lpi[3].p(mi_aa);
            mq_aa=lpq[0].p(mq_aa); mq_aa=lpq[1].p(mq_aa); mq_aa=lpq[2].p(mq_aa); mq_aa=lpq[3].p(mq_aa);
            cap_i+=mi_aa; cap_q+=mq_aa; cap_cnt++;
            if(cap_cnt<(int)cap_decim) continue;
            float fi=(float)(cap_i/cap_cnt), fq=(float)(cap_q/cap_cnt);
            cap_i=cap_q=0; cap_cnt=0;
            // AM envelope (amplitude-independent decode -> no AGC needed)
            float env=sqrtf(fi*fi+fq*fq);
            am_dc+=am_dc_alpha*(env-am_dc);
            float audio=alf.p(env-am_dc);
            aac+=audio; acnt++;
            if(acnt>=(int)actual_ad){
                float a=(float)(aac/acnt); aac=0; acnt=0;
                dec.feed(a);
            }
        }
        my_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    // 정지 요청 없이 끝났으면 (채널 삭제/스트림 에러) 상태 정리 + 브로드캐스트
    if(!worker_stop_req(ch_idx)) worker_natural_exit(v, ch_idx);
    bewe_log_push(0,"ACARS[%d] stop\n",ch_idx);
}

} // namespace acars_mod
