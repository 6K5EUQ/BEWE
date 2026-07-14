#pragma once
// ── Detect: per-bin 기준선(baseline) 관리 ───────────────────────────────────
// ui.cpp / cli_host.cpp 의 update_channel_squelch() 가 공유한다 (로직 이중화 방지).
//
// Detect 는 "간헐 버스트"를 잡는 기능이다. 연속적으로 늘 나와 있는 신호는 사용자가
// 수동으로 채널 필터를 걸어 쓴다 — detect 가 그런 신호에 붙잡히면 안 된다.
//
// 기존 검출기는 채널당 스칼라 임계(sq_threshold) 하나로 탐색 대역 전체를 판정했다.
// 그 값은 "대역 peak 의 하위 퍼센타일 + 10dB" 라서, 탐색 대역이 넓고 그 안에 상시 강한
// 신호(인접 방송국 / 스퍼 / DC 스파이크)가 하나라도 있으면 임계가 그놈을 따라 올라가
// 실제 노이즈플로어보다 수십 dB 위에서 굳는다 → 약한 협대역 버스트는 영영 못 본다.
// 탐색 대역을 넓힐수록 감도가 떨어지는 구조였다.
//
// 대신 arm 시점에 탐색 대역의 스펙트럼을 bin 별로 1초간 평균내어 기준선으로 굳힌다.
// 상시 존재하는 것은 전부 기준선 안에 흡수되고(캐리어든 스퍼든 대역별 노이즈 기울기든),
// 판정은 bin 마다 "기준선 대비 +DET_MARGIN_DB" 로 한다. 기준선을 넘는 것 = 새로 나타난
// 것 = 버스트. 정확히 원하는 동작이다.
//
// 기준선은 SDR 재시작/autoscale(노이즈플로어 자체가 이동) 이나 탐색 대역/FFT 설정
// 변경 시 무효가 된다 — det_base_valid() 가 그걸 판정하고, 아니면 다시 1초를 쌓는다.

#include "config.hpp"
#include "channel.hpp"
#include <algorithm>
#include <chrono>

// 채널의 sq_threshold 에 값을 적용한다. detect 채널이면 그 필드는 절대 dB 가 아니라
// 기준선 대비 마진이므로(config.hpp) 마진 범위로, 아니면 절대 dB 범위로 클램프한다.
// JOIN 이 보내는 SET_SQ_THRESH 와 HOST 로컬 슬라이더가 같은 규칙을 타게 하는 단일 지점.
inline void det_apply_sq_thresh(Channel& ch, float v){
    if(ch.det_on.load(std::memory_order_relaxed)){
        v = std::max(DET_MARGIN_MIN_DB, std::min(DET_MARGIN_MAX_DB, v));
    } else {
        v = std::max(-100.0f, std::min(0.0f, v));
    }
    ch.sq_threshold.store(v, std::memory_order_relaxed);
    ch.sq_manual.store(true, std::memory_order_relaxed);
    ch.sq_calibrated.store(true, std::memory_order_relaxed);  // 자동 캘리브가 덮어쓰지 못하게
}

// 지금 들고 있는 기준선이 현재 캡처 설정/탐색 대역에 대해 유효한가.
inline bool det_base_valid(const Channel& ch, float scan_s, float scan_e,
                           uint64_t cf, uint32_t sr, int fft_size, int n_bins)
{
    return ch.det_base_ready
        && ch.det_base_s == scan_s && ch.det_base_e == scan_e
        && ch.det_base_cf == cf && ch.det_base_sr == sr
        && ch.det_base_fft == fft_size
        && (int)ch.det_base.size() == n_bins;
}

// 새 FFT 행 하나를 기준선 수집에 먹인다. DET_BASE_MS 가 지나면 평균을 확정하고
// det_base_ready 를 세운다. 호출자는 ready 가 될 때까지 lock 을 시도하지 않는다.
//
// rowp    : 최신 FFT 행 (dB, 길이 fft_size)
// bin_s   : 탐색 대역 시작 bin (DC 랩 가능)
// n_bins  : 탐색 대역 bin 수
// now_ms  : steady clock ms
// 반환값: 이번 호출로 기준선이 확정됐으면 true (로그용)
inline bool det_base_accumulate(Channel& ch, const float* rowp,
                                int bin_s, int n_bins, int fft_size,
                                float scan_s, float scan_e,
                                uint64_t cf, uint32_t sr, int64_t now_ms)
{
    if(n_bins <= 0) return false;
    // 이미 확정된 기준선을 실수로 덮어쓰지 않는다 — 재수집은 det_base_reset() 을 거친다.
    // (호출자는 det_base_valid() 가 false 일 때만 부르지만, ready 인데 설정이 달라진
    //  경우가 있으므로 여기서도 막는다.)
    if(ch.det_base_ready) return false;

    // 수집 시작 또는 설정이 바뀌어 재수집 — 버퍼를 잡고 시계를 건다.
    if((int)ch.det_base_acc.size() != n_bins || ch.det_base_rows == 0){
        ch.det_base_acc.assign((size_t)n_bins, 0.0);
        ch.det_base_rows  = 0;
        ch.det_base_t0_ms = now_ms;
        ch.det_base_ready = false;
    }

    for(int k = 0; k < n_bins; k++){
        int b = bin_s + k; if(b >= fft_size) b -= fft_size;
        ch.det_base_acc[(size_t)k] += (double)rowp[b];
    }
    ch.det_base_rows++;

    // 최소 1행은 봤고 1초가 지났으면 확정. FFT 행 갱신이 느린 설정(~1.5Hz)에서는
    // 행 수가 적지만, 기준선은 "상시 신호를 흡수" 하는 게 목적이라 몇 행이면 충분하다.
    if(now_ms - ch.det_base_t0_ms < DET_BASE_MS) return false;

    ch.det_base.resize((size_t)n_bins);
    double inv = 1.0 / (double)ch.det_base_rows;
    for(int k = 0; k < n_bins; k++)
        ch.det_base[(size_t)k] = (float)(ch.det_base_acc[(size_t)k] * inv);

    ch.det_base_ready = true;
    ch.det_base_s = scan_s; ch.det_base_e = scan_e;
    ch.det_base_cf = cf;    ch.det_base_sr = sr;
    ch.det_base_fft = fft_size;
    ch.det_base_acc.clear();   // 수집 버퍼 반납 (재수집은 det_base_reset() 이 트리거)
    return true;
}
