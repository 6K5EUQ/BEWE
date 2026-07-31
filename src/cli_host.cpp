// ── BEWE CLI HOST Mode ──────────────────────────────────────────────────
// 라즈베리파이5 등 디스플레이 없는 환경에서 HOST 모드 전용 실행
// GLFW/OpenGL/ImGui 의존성 없음

#include "fft_viewer.hpp"
#include "detect_base.hpp"
#include "module_api.hpp"
#include "sigmf.hpp"
#include "iq_filename.hpp"
#include "login.hpp"
#include "bewe_paths.hpp"
#include "central_client.hpp"
#include "mission_push.hpp"
#include "hist_check.hpp"
#include "net_protocol.hpp"
#include "host_band_plan.hpp"
#include <zstd.h>   // Central 릴레이 CHANNEL_SYNC 해제 (v13)
#include "host_band_categories.hpp"
#include "host_state.hpp"
#include "long_waterfall.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <csignal>
#include <ctime>
#include <sys/statvfs.h>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <poll.h>
#include <unistd.h>
#include <termios.h>
#include <arpa/inet.h>
#include <dirent.h>
#include <sys/stat.h>

// ── bewe_log (ui.cpp 대체) ───────────────────────────────────────────────
void bewe_log(const char* fmt, ...){
    char buf[256];
    va_list ap; va_start(ap,fmt);
    vsnprintf(buf,sizeof(buf),fmt,ap);
    va_end(ap);
    bewe_log_push(0, "%s", buf);
}

// ── bladerf_usb_reset 선언 (hw_detect.cpp) ───────────────────────────────
bool bladerf_usb_reset();

// ── 채널 스컬치 (ui.cpp에서 추출 - GUI 의존성 없음) ──────────────────────
void FFTViewer::update_channel_squelch(){
    if(total_ffts < 1 || fft_size < 1) return;

    // SDR (재)시작/autoscale → 노이즈플로어가 달라졌으므로 자동 캘리브 채널만 다시 잡는다.
    // 사용자가 손댄 채널(sq_manual)은 그 값을 유지.
    if(sq_recalib_req.exchange(false, std::memory_order_relaxed)){
        for(int c = 0; c < MAX_CHANNELS; c++){
            Channel& ch = channels[c];
            if(!ch.filter_active) continue;
            // detect 기준선은 절대 dB 스냅샷이라 노이즈플로어가 이동하면 무조건 무효다.
            // sq_manual(사용자가 스컬치 임계를 직접 잡음)과는 무관 — 별개 축.
            ch.det_base_reset();
            if(ch.sq_manual.load(std::memory_order_relaxed)) continue;
            ch.sq_calibrated.store(false, std::memory_order_relaxed);
            ch.sq_calib_cnt = 0;
        }
    }

    // 호출 간격 기반 실 delta 시간 (시간 카운터용)
    static auto sq_last_tick = std::chrono::steady_clock::now();
    auto sq_now = std::chrono::steady_clock::now();
    float real_dt = std::chrono::duration<float>(sq_now - sq_last_tick).count();
    if(real_dt > 0.5f) real_dt = 0.02f; // 첫 호출/정지 이후 복귀 보정
    sq_last_tick = sq_now;

    // Detect 모드 채널의 s/e 변경 요청 (락 밖에서 적용 — start/stop_dem 은 스레드 join)
    struct DetApply { int ch; float s, e; bool lock; };
    std::vector<DetApply> det_pending;

    // 노치 구간 스냅샷 — detect 는 이 안의 bin 을 신호로 치지 않는다 (스퍼/간섭 배제).
    // data_mtx 를 잡기 전에 떠서 락 순서를 고정한다 (notches_mtx → data_mtx 로 잡는 곳 없음).
    std::vector<std::pair<float,float>> notch_bands;   // (lo_mhz, hi_mhz)
    {
        std::lock_guard<std::mutex> nlk(notches_mtx);
        notch_bands.reserve(notches.size());
        for(const auto& n : notches)
            notch_bands.emplace_back(std::min(n.freq_lo_mhz, n.freq_hi_mhz),
                                     std::max(n.freq_lo_mhz, n.freq_hi_mhz));
    }
    {
    std::lock_guard<std::mutex> lk(data_mtx);
    float cf_mhz = (float)(header.center_frequency / 1e6);
    float nyq_mhz = header.sample_rate / 2e6f;
    if(nyq_mhz < 0.001f) return;
    int hf = fft_size / 2;
    int fi = (total_ffts > 0 ? total_ffts - 1 : 0) % FFT_HISTORY_ROWS;
    const float* rowp = fft_data.data() + fi * fft_size;
    // 같은 FFT 행이면 채널별 peak 재스캔 생략 (행 갱신은 ~1.5-37Hz, 호출은 ~50Hz)
    bool same_row = (total_ffts == sq_last_total_ffts);
    int64_t now_ms_row = std::chrono::duration_cast<std::chrono::milliseconds>(
                             sq_now.time_since_epoch()).count();
    if(!same_row || sq_row_change_ms == 0) sq_row_change_ms = now_ms_row;
    sq_last_total_ffts = total_ffts;
    auto freq_to_bin = [&](float rel_mhz) -> int {
        int bin = (rel_mhz >= 0)
            ? (int)((rel_mhz / nyq_mhz) * hf)
            : fft_size + (int)((rel_mhz / nyq_mhz) * hf);
        return std::max(0, std::min(fft_size - 1, bin));
    };
    auto bin_to_freq = [&](int bin) -> float {
        float rel = (bin <= hf) ? ((float)bin / (float)hf) * nyq_mhz
                                : ((float)(bin - fft_size) / (float)hf) * nyq_mhz;
        return cf_mhz + rel;
    };
    for(int c = 0; c < MAX_CHANNELS; c++){
        Channel& ch = channels[c];
        if(!ch.filter_active) continue;
        // Detect 채널은 좁아진 s/e 가 아니라 원래 탐색 대역 기준으로 스컬치를 잡는다
        // (좁은 폭으로 재계산하면 임계값이 자기 신호를 물어 올라간다)
        bool det = ch.det_on.load(std::memory_order_relaxed);
        float scan_s = det ? ch.det_s : ch.s;
        float scan_e = det ? ch.det_e : ch.e;
        float s_mhz = std::min(scan_s, scan_e) - cf_mhz;
        float e_mhz = std::max(scan_s, scan_e) - cf_mhz;
        int bin_s = freq_to_bin(s_mhz);
        int bin_e = freq_to_bin(e_mhz);
        // 행·대역 둘 다 그대로면 캐시 재사용 (계산결과 동일 → 게이트/시간 동작 불변)
        float peak_db;
        if(same_row && ch.sq_calibrated.load(std::memory_order_relaxed)
           && ch.sq_scan_s == scan_s && ch.sq_scan_e == scan_e){
            peak_db = ch.sq_cached_peak;
        } else {
            peak_db = -120.0f;
            if(bin_s <= bin_e){
                for(int b = bin_s; b <= bin_e; b++)
                    if(rowp[b] > peak_db) peak_db = rowp[b];
            } else {
                for(int b = bin_s; b < fft_size; b++)
                    if(rowp[b] > peak_db) peak_db = rowp[b];
                for(int b = 0; b <= bin_e; b++)
                    if(rowp[b] > peak_db) peak_db = rowp[b];
            }
            ch.sq_cached_peak = peak_db;
            ch.sq_scan_s = scan_s; ch.sq_scan_e = scan_e;
        }
        float prev = ch.sq_sig.load(std::memory_order_relaxed);
        float sig = 0.3f * peak_db + 0.7f * prev;
        ch.sq_sig.store(sig, std::memory_order_relaxed);
        // detect 채널은 캘리브 제외 — sq_threshold 가 절대 dB 가 아니라 기준선 대비
        // 마진이라(config.hpp) 절대값으로 덮어쓰면 마진 설정이 날아간다.
        if(!det && !ch.sq_calibrated.load(std::memory_order_relaxed)){
            if(ch.sq_calib_cnt < 60)
                ch.sq_calib_buf[ch.sq_calib_cnt++] = peak_db;
            if(ch.sq_calib_cnt >= 60){
                float tmp[60];
                memcpy(tmp, ch.sq_calib_buf, sizeof(tmp));
                std::nth_element(tmp, tmp + 12, tmp + 60); // 20th percentile
                ch.sq_threshold.store(tmp[12] + 10.0f, std::memory_order_relaxed);
                ch.sq_calibrated.store(true, std::memory_order_relaxed);
                ch.sq_calib_cnt = 0;
            }
        }
        float thr = ch.sq_threshold.load(std::memory_order_relaxed);
        bool gate = ch.sq_gate.load(std::memory_order_relaxed);
        const float HYS = 3.0f;
        // ui.cpp update_channel_squelch() 와 동일 정책 — 양쪽 같이 유지할 것.
        const int HOLD_FRAMES = 4;  // ~70ms @ 18ms 틱
        if(ch.sq_calibrated.load(std::memory_order_relaxed)){
            if(!gate && sig >= thr){
                gate = true;
                ch.sq_gate_hold = HOLD_FRAMES;
            }
            if(gate){
                if(sig >= thr - HYS)
                    ch.sq_gate_hold = HOLD_FRAMES;
                else if(--ch.sq_gate_hold <= 0)
                    gate = false;
            }
        }
        // ── 에너지 디텍션 (Detect 모드) ───────────────────────────────────
        // 간헐 버스트 전용 (연속 신호는 사용자가 수동 필터). 판정은 스칼라 thr 가 아니라
        // arm 시점에 굳힌 bin 별 기준선 대비 마진 (detect_base.hpp 주석 참조).
        // 상시 존재하는 신호는 기준선에 흡수돼 무시되고, 새로 뜬 것만 잡힌다.
        // 기준선 대비 SNR 이 최대인 연속 구간(run)으로 채널 폭을 좁히고, 끊기면 원래 폭 복귀.
        // 이 채널의 스컬치 게이트는 검출기가 대신한다.
        // 새 FFT 행에서만 스캔 — 같은 행 재스캔은 낭비
        if(det && !same_row){
            int n_bins = (bin_s <= bin_e) ? (bin_e-bin_s+1)
                                          : (fft_size-bin_s) + (bin_e+1);
            auto bin_at = [&](int k){ int b = bin_s + k; return (b >= fft_size) ? b-fft_size : b; };
            // 기준선이 없거나(arm 직후) 캡처 설정/탐색 대역이 바뀌었으면 다시 1초를 쌓는다.
            // sq_recalib_req(SDR 재시작/autoscale)는 위에서 sq_calibrated 를 내렸고,
            // set_channel_detect/노브 조작이 det_base_reset() 을 호출한다.
            bool base_ok = det_base_valid(ch, scan_s, scan_e, header.center_frequency,
                                          header.sample_rate, fft_size, n_bins);
            if(!base_ok){
                // ready 인데 valid 가 아니다 = 대역/캡처 설정이 바뀌었다 → 기준선 폐기 후 재수집
                if(ch.det_base_ready) ch.det_base_reset();
                if(det_base_accumulate(ch, rowp, bin_s, n_bins, fft_size, scan_s, scan_e,
                                       header.center_frequency, header.sample_rate, now_ms_row))
                    bewe_log_push(0,"[DETECT] CH%d baseline ready (%d bins)\n", c, n_bins);
            } else if(!same_row){
                // 새 행에서만 기준선을 끌어당긴다 (같은 행 재추적은 dt 만 낭비).
                // 이동하는 스퍼를 몇 초에 걸쳐 흡수 — lock/hold 중엔 함수 내부에서 스킵.
                det_base_track(ch, rowp, bin_s, n_bins, fft_size, now_ms_row);
            }
            float best_lo=0, best_hi=0, best_snr=-999.f;
            const bool locked = ch.det_locked.load(std::memory_order_relaxed);
            if(base_ok){
                int run_start=-1, gap=0;
                float run_snr=-999.f;
                // 짧은 갭은 이어붙임 (사이드밴드 딥에서 run 쪼개짐 방지). 폭은 bin 수가 아니라
                // 주파수로 정한다 — bin 폭이 설정마다 달라 나란한 두 교신이 묶이는 것을 막는다.
                float bin_hz = (float)header.sample_rate / (float)std::max(1, fft_size);
                const int GAP_BINS = std::max(1, (int)(DET_GAP_KHZ * 1000.0f / std::max(1.0f, bin_hz)));
                // 노치를 스캔 인덱스(k) 구간으로 미리 변환한다. bin 마다 주파수를 만들어
                // 노치 리스트를 훑으면 그 검사만으로 스캔 본체보다 4배 비싸진다 (측정치).
                // k 는 scan_s..scan_e 안에서 선형이라 경계만 계산하면 루프는 정수 비교로 끝난다.
                struct KRange { int lo, hi; };
                static thread_local std::vector<KRange> notch_k;
                notch_k.clear();
                if(!notch_bands.empty() && n_bins > 1){
                    float lo_mhz = std::min(scan_s, scan_e);
                    float hi_mhz = std::max(scan_s, scan_e);
                    float span   = hi_mhz - lo_mhz;
                    if(span > 0.f){
                        float k_per_mhz = (float)(n_bins - 1) / span;
                        for(const auto& nb : notch_bands){
                            if(nb.second < lo_mhz || nb.first > hi_mhz) continue;  // 대역 밖
                            int k0 = (int)std::floor((nb.first  - lo_mhz) * k_per_mhz);
                            int k1 = (int)std::ceil ((nb.second - lo_mhz) * k_per_mhz);
                            k0 = std::max(0, k0);
                            k1 = std::min(n_bins - 1, k1);
                            if(k0 <= k1) notch_k.push_back({k0, k1});
                        }
                        // 정렬 후 스캔에서 커서 하나로 훑는다 → bin 당 비교 1회 (노치 수 무관)
                        std::sort(notch_k.begin(), notch_k.end(),
                                  [](const KRange& a, const KRange& b){ return a.lo < b.lo; });
                    }
                }
                size_t nk_i = 0;   // 현재 k 이후의 첫 노치 (스캔이 전진하면 같이 전진)
                // lock 중에는 "지금 듣고 있는 그 신호"만 따라간다. 대역 어딘가에서 더 센 신호가
                // 떠도 그건 별개 교신이므로 무시 — 안 그러면 그 run 이 best 가 되고, 아래 확장
                // 분기가 min/max 로 두 신호를 다 덮어 필터가 통째로 벌어진다 (두 신호가 동시에
                // 들림). 잠긴 대역에 겹치거나 가드밴드만큼 인접한 run 만 후보로 인정한다 —
                // 같은 교신의 사이드밴드는 붙어 있고, 다른 교신은 떨어져 있다.
                // 신호가 끝나 release 되면 다음 프레임부터 다시 대역 전체를 본다.
                const float NEAR_MHZ = DET_GAP_KHZ * 0.001f;
                auto close_run = [&](int k_end){
                    if(run_start < 0) return;
                    if(k_end - run_start + 1 >= DET_MIN_RUN_BINS && run_snr > best_snr){
                        float lo = bin_to_freq(bin_at(run_start));
                        float hi = bin_to_freq(bin_at(k_end));
                        bool adjacent = !locked ||
                                        (hi >= ch.s - NEAR_MHZ && lo <= ch.e + NEAR_MHZ);
                        if(adjacent){
                            best_snr = run_snr;
                            best_lo = lo;
                            best_hi = hi;
                        }
                    }
                    run_start=-1; run_snr=-999.f;
                };
                // detect 채널의 sq_threshold 는 절대 dB 가 아니라 기준선 대비 마진이다
                // (config.hpp 주석 참조). 슬라이더로 조절되며 CH_SYNC/host_state 를 그대로 탄다.
                float margin = std::max(DET_MARGIN_MIN_DB, std::min(DET_MARGIN_MAX_DB, thr));
                for(int k=0; k<n_bins; k++){
                    // 노치 구간은 신호로 치지 않는다 (Ctrl+우클릭으로 친 스퍼/간섭 대역).
                    // 진행 중인 run 은 여기서 끊는다 — gap 으로 세면 노치를 건너뛰어 양옆
                    // 신호가 한 run 으로 이어져 필터가 노치를 통째로 삼킨다.
                    // 커서(nk_i)는 k 와 함께 전진하므로 bin 당 비교는 1회다.
                    while(nk_i < notch_k.size() && notch_k[nk_i].hi < k) nk_i++;
                    if(nk_i < notch_k.size() && k >= notch_k[nk_i].lo){
                        if(run_start >= 0) close_run(k - 1);
                        gap = 0;
                        k = notch_k[nk_i].hi;   // 노치 끝까지 건너뛴다 (루프의 k++ 가 다음 bin)
                        continue;
                    }
                    float snr = rowp[bin_at(k)] - ch.det_base[(size_t)k];
                    if(snr >= margin){
                        if(run_start < 0){ run_start = k; run_snr = -999.f; }
                        if(snr > run_snr) run_snr = snr;
                        gap = 0;
                    } else if(run_start >= 0){
                        if(++gap > GAP_BINS) close_run(k - gap);
                    }
                }
                if(run_start >= 0) close_run(n_bins-1);
            }
            bool have = (best_snr > -999.f) && (best_hi > best_lo);
            const int   DET_HOLD_FRAMES   = 18;
            const float DET_GUARD_MHZ     = 0.002f;  // 2 kHz 가드밴드
            const int   DET_EXPAND_FRAMES = 3;       // 확장은 연속 관측 시에만 (스파이크 방어)

            // lock 중 폭은 넓어지기만 한다 — 한 교신에서 관측된 최대 폭 유지.
            // (FM 편이가 출렁일 때 따라 좁히면 필터가 요동친다)
            if(have){
                ch.det_hold = DET_HOLD_FRAMES;
                if(!locked){
                    det_pending.push_back({c, best_lo-DET_GUARD_MHZ, best_hi+DET_GUARD_MHZ, true});
                    ch.det_ext_cnt = 0;
                } else if(bewe_mod_ch_spec_bw(c) > 0.f){
                    // 폭이 디코더 규격으로 고정된 채널 — 확장하지 않는다. 확장해 봐야 lock
                    // 적용부가 다시 규격 폭으로 되돌리므로, 매번 stop_dem/start_dem 만 돌아
                    // 복조가 끊긴다. 중심 이동은 release→재lock 으로 따라간다.
                    ch.det_ext_cnt = 0;
                } else if(best_lo < ch.s || best_hi > ch.e){
                    // 대역 밖으로 삐져나감 → 확장 후보. 연속 관측될 때만 반영 (최대치 누적)
                    if(ch.det_ext_cnt == 0){
                        ch.det_ext_s = best_lo; ch.det_ext_e = best_hi;
                    } else {
                        ch.det_ext_s = std::min(ch.det_ext_s, best_lo);
                        ch.det_ext_e = std::max(ch.det_ext_e, best_hi);
                    }
                    if(++ch.det_ext_cnt >= DET_EXPAND_FRAMES){
                        det_pending.push_back({c,
                            std::min(ch.s, ch.det_ext_s - DET_GUARD_MHZ),
                            std::max(ch.e, ch.det_ext_e + DET_GUARD_MHZ), true});
                        ch.det_ext_cnt = 0;
                    }
                } else {
                    ch.det_ext_cnt = 0;   // 대역 안 — 좁히지 않는다 (최대 폭 유지)
                }
            } else if(locked && --ch.det_hold <= 0){
                det_pending.push_back({c, ch.det_s, ch.det_e, false});
            }
        }
        // 스컬치 우회 — 잡고 있는 동안은 매 프레임 열어 둔다 (스캔은 새 행에서만)
        if(det && ch.det_locked.load(std::memory_order_relaxed)) gate = true;

        ch.sq_gate.store(gate, std::memory_order_relaxed);

        // ── 디코더 전용 게이트 (관대) ─────────────────────────────────────
        // raw 행 peak 이 thr-6dB 만 넘으면 열고 2초 hold. 캘리브레이션 전엔 항상 열림.
        // (오디오 게이트는 EMA sig >= thr — 그보다 훨씬 빨리/오래 열려 버스트 유실 방지)
        {
            int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 sq_now.time_since_epoch()).count();
            // fail-open: FFT 행이 멎으면(spectrum_pause / rx stop / SDR 에러 / render off)
            // peak 가 갱신되지 않아 게이트가 영구히 닫힌 채 굳는다 → 디코더가 조용히 정지.
            // 행이 stale 하면 무조건 열어 둔다 (구동작 = 풀레이트 복조로 안전 복귀).
            bool rows_stale = (now_ms - sq_row_change_ms) > DEC_GATE_HOLD_MS;
            if(rows_stale || !ch.sq_calibrated.load(std::memory_order_relaxed)){
                ch.dec_gate.store(true, std::memory_order_relaxed);
                ch.dec_gate_until_ms.store(0, std::memory_order_relaxed);
            } else {
                int64_t until = ch.dec_gate_until_ms.load(std::memory_order_relaxed);
                if(peak_db >= thr - DEC_GATE_MARGIN_DB){
                    until = now_ms + DEC_GATE_HOLD_MS;
                    ch.dec_gate_until_ms.store(until, std::memory_order_relaxed);
                }
                ch.dec_gate.store(now_ms < until, std::memory_order_relaxed);
            }
        }

        // 스컬치 누적 시간 — 실벽시계 delta 사용 (Holding 중에는 정지)
        if(ch.filter_active && !ch.dem_paused.load()){
            if(!sdr_stream_error.load()){
                ch.sq_total_time += real_dt;
                if(gate) ch.sq_active_time += real_dt;
            }
        } else if(!ch.filter_active){
            ch.sq_active_time = 0;
            ch.sq_total_time = 0;
        }
    }
    }  // data_mtx 해제

    // ── Detect 적용: 채널 폭 조정 + 복조 시작/정지 (락 밖) ────────────────
    for(auto d : det_pending){
        Channel& ch = channels[d.ch];
        if(!ch.det_on.load(std::memory_order_relaxed)) continue;
        if(d.lock){
            // 가드밴드가 탐색 대역을 넘지 않도록 클램프
            d.s = std::max(d.s, ch.det_s);
            d.e = std::min(d.e, ch.det_e);
            if(d.e <= d.s) continue;
            float mid = (d.s + d.e) * 0.5f;
            // 이 채널에 디코더가 켜져 있으면 폭은 그 신호 규격이 정한다 — 검출된 폭은
            // 버스트마다 흔들리고(페이딩/마진), 그대로 필터로 쓰면 디코더 통과대역이
            // 같이 흔들린다. 중심주파수만 검출값을 쓰고 폭은 규격값으로 고정한다.
            // (규격 미지정 모듈이거나 디코더가 없으면 0 → 검출 폭 그대로)
            float spec_bw = bewe_mod_ch_spec_bw(d.ch);
            if(spec_bw > 0.f){
                float half = spec_bw * 0.5e-6f;              // Hz → MHz, 반폭
                d.s = mid - half; d.e = mid + half;
                // 탐색 대역을 넘지 않게 클램프 (좁은 탐색대역에 넓은 규격이 걸린 경우)
                d.s = std::max(d.s, ch.det_s);
                d.e = std::min(d.e, ch.det_e);
                if(d.e <= d.s) continue;
            }
            Channel::DemodMode md = (mid >= 118.0f && mid <= 137.0f)
                                        ? Channel::DM_AM : Channel::DM_FM;
            bool was = ch.det_locked.load(std::memory_order_relaxed);
            stop_dem(d.ch, false);          // 재튜닝 — 디코더 보존
            ch.s = d.s; ch.e = d.e;
            if(!was){
                ch.audio_mask.store(0xFFFFFFFFu);
                ch.det_locked.store(true, std::memory_order_relaxed);
                bewe_log_push(0,"[DETECT] CH%d locked %.4f-%.4f MHz (%s)\n",
                              d.ch, d.s, d.e, md==Channel::DM_AM?"AM":"FM");
            }
            start_dem(d.ch, md);
        } else {
            // 신호 종료 → 오디오 복조만 정지하고 **디코더는 보존**한다 (stop_decoders=false).
            // detect 채널에 decode 를 걸어두는 것이 주 용도다 — 예: AIS 두 주파수를 한 detect
            // 필터로 덮고 decode=ais 를 걸어두면, 버스트가 뜰 때마다 그 주파수로 폭이 좁혀지고
            // 디코더가 그대로 따라간다. 여기서 디코더를 죽이면 교신이 끝날 때마다 워커가
            // 재시작돼 다음 버스트 앞부분을 놓친다.
            stop_dem(d.ch, false);
            ch.s = d.s; ch.e = d.e;
            ch.mode = Channel::DM_NONE;
            ch.det_locked.store(false, std::memory_order_relaxed);
            ch.sq_gate.store(false, std::memory_order_relaxed);
            ch.det_hold = 0; ch.det_ext_cnt = 0;   // 다음 교신은 다시 처음부터 최대 폭을 쌓는다
            bewe_log_push(0,"[DETECT] CH%d released → %.4f-%.4f MHz\n", d.ch, d.s, d.e);
        }
    }
    if(!det_pending.empty() && net_srv)
        net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
}

// ── Detect 모드 토글 (CLI HOST) ──────────────────────────────────────────
void FFTViewer::set_channel_detect(int ch_idx, bool on){
    if(ch_idx < 0 || ch_idx >= MAX_CHANNELS) return;
    Channel& ch = channels[ch_idx];
    if(!ch.filter_active) return;
    if(on){
        if(ch.det_on.load()) return;
        ch.det_s = ch.s; ch.det_e = ch.e;
        ch.det_hold = 0; ch.det_ext_cnt = 0;
        ch.det_locked.store(false);
        ch.det_on.store(true);
        ch.det_base_reset();   // 무장할 때마다 기준선을 새로 잡는다 (arm 시점의 대역 상태 = 기준)
        // detect 채널의 sq_threshold 는 절대 dB 가 아니라 기준선 대비 마진으로 재해석된다
        // (config.hpp). 절대 dB 가 들어 있던 값을 기본 마진으로 바꾼다.
        {
            float t = ch.sq_threshold.load();
            if(!(t >= DET_MARGIN_MIN_DB && t <= DET_MARGIN_MAX_DB))
                ch.sq_threshold.store(DET_MARGIN_DEF_DB);
        }
        ch.sq_calibrated.store(true);   // detect 중엔 자동 캘리브를 돌리지 않는다
        ch.sq_calib_cnt = 0;
        bewe_log_push(0,"[DETECT] CH%d armed %.4f-%.4f MHz — baseline 수집 중 (margin %.0fdB)\n",
                      ch_idx, ch.det_s, ch.det_e, ch.sq_threshold.load());
    } else {
        if(!ch.det_on.load()) return;
        bool was = ch.det_locked.load();
        ch.det_on.store(false);
        ch.det_locked.store(false);
        ch.det_hold = 0; ch.det_ext_cnt = 0;
        ch.det_base_reset();
        if(was){
            stop_dem(ch_idx);
            ch.s = ch.det_s; ch.e = ch.det_e;
            ch.mode = Channel::DM_NONE;
            ch.sq_gate.store(false);
        }
        // sq_threshold 에는 마진값(0~40)이 들어 있다 — 절대 dB 로 해석되면 게이트가 영영
        // 안 열린다. 재캘리브로 절대 dB 를 다시 잡게 한다.
        ch.sq_manual.store(false);
        ch.sq_calibrated.store(false); ch.sq_calib_cnt = 0;
        bewe_log_push(0,"[DETECT] CH%d disarmed\n", ch_idx);
    }
    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
}

// ── Signal handler ───────────────────────────────────────────────────────
static std::atomic<bool> g_shutdown{false};
static void sig_handler(int){ g_shutdown.store(true); }
// /powercycle full — 정상 종료(상태 저장 포함)를 마친 뒤 머신을 재부팅한다.
// 종료 경로 한가운데서 재부팅하면 host_state 가 안 써진 채 날아갈 수 있어,
// 모든 정리가 끝난 main() 맨 끝에서만 실행한다.
static std::atomic<bool> g_reboot_on_exit{false};
// /powercycle partial — 종료 후 systemd 가 다시 띄우게 한다. 유닛이
// Restart=on-failure 라 정상 종료(0)로는 재기동이 안 걸린다.
static std::atomic<bool> g_restart_on_exit{false};

// ── System monitor helpers (from ui.cpp) ─────────────────────────────────
static void read_cpu(long long& idle, long long& total){
    FILE* f=fopen("/proc/stat","r"); if(!f){idle=total=0;return;}
    long long u,n,s,i,iow,irq,sirq;
    if(fscanf(f,"cpu %lld %lld %lld %lld %lld %lld %lld",&u,&n,&s,&i,&iow,&irq,&sirq)!=7)
        { idle=total=0; fclose(f); return; }
    fclose(f); idle=i+iow; total=u+n+s+i+iow+irq+sirq;
}
static float read_ram(){
    FILE* f=fopen("/proc/meminfo","r"); if(!f) return 0.0f;
    long long total=0,avail=0; char key[64]; long long val;
    for(int i=0;i<10;i++){
        if(fscanf(f,"%63s %lld kB",key,&val)!=2) break;
        if(!strcmp(key,"MemTotal:"))      total=val;
        else if(!strcmp(key,"MemAvailable:")) avail=val;
    }
    fclose(f);
    return (total>0)?(float)(total-avail)/total*100.0f:0.0f;
}
static float read_ghz(){
    double sum=0; int cnt=0;
    for(int c=0;c<256;c++){
        char path[128];
        snprintf(path,sizeof(path),"/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq",c);
        FILE* f=fopen(path,"r"); if(!f) break;
        long long khz=0; if(fscanf(f,"%lld",&khz)){} fclose(f);
        sum+=khz; cnt++;
    }
    return cnt>0?(float)(sum/cnt/1e6):0.0f;
}
static int read_cpu_temp_c(){
    // 1) hwmon coretemp (Intel/AMD)
    for(int i=0;i<32;i++){
        char np[80]; snprintf(np,sizeof(np),"/sys/class/hwmon/hwmon%d/name",i);
        FILE* fn=fopen(np,"r"); if(!fn) continue;
        char name[32]={}; fgets(name,sizeof(name),fn); fclose(fn);
        if(strncmp(name,"coretemp",8)==0 || strncmp(name,"k10temp",7)==0
           || strncmp(name,"cpu_thermal",11)==0){
            char tp[80]; snprintf(tp,sizeof(tp),"/sys/class/hwmon/hwmon%d/temp1_input",i);
            FILE* ft=fopen(tp,"r"); if(!ft) continue;
            int milli=0; if(fscanf(ft,"%d",&milli)==1){ fclose(ft); return milli/1000; }
            fclose(ft);
        }
    }
    // 2) thermal_zone: x86_pkg_temp / TCPU / cpu-thermal (RPi5 등 ARM)
    for(int i=0;i<16;i++){
        char tt[80]; snprintf(tt,sizeof(tt),"/sys/class/thermal/thermal_zone%d/type",i);
        FILE* ft=fopen(tt,"r"); if(!ft) continue;
        char zt[32]={}; fgets(zt,sizeof(zt),ft); fclose(ft);
        if(strncmp(zt,"x86_pkg_temp",12)==0 || strncmp(zt,"TCPU",4)==0
           || strncmp(zt,"cpu-thermal",11)==0 || strncmp(zt,"cpu_thermal",11)==0){
            char tp[80]; snprintf(tp,sizeof(tp),"/sys/class/thermal/thermal_zone%d/temp",i);
            FILE* fv=fopen(tp,"r"); if(!fv) continue;
            int milli=0; if(fscanf(fv,"%d",&milli)==1){ fclose(fv); return milli/1000; }
            fclose(fv);
        }
    }
    // 3) fallback: thermal_zone0
    FILE* fv=fopen("/sys/class/thermal/thermal_zone0/temp","r");
    if(fv){ int milli=0; if(fscanf(fv,"%d",&milli)==1 && milli>0){ fclose(fv); return milli/1000; } fclose(fv); }
    return 0;
}
// sysfs 에 안 뜨는 I2C 연료게이지(Pi + X1200 UPS 등) 폴백: ups-log 데몬이 남기는
// ~/ups_history.csv 마지막 줄. 형식 = timestamp,volt,soc,ac,status
// 파일이 없거나 5분 이상 갱신이 없으면 UPS 가 없는 것으로 본다(255).
static uint8_t read_bat_pct_csv(uint8_t* ac_out){
    const char* home = getenv("HOME"); if(!home) return 255;
    char p[256]; snprintf(p,sizeof(p),"%s/ups_history.csv",home);
    struct stat sb;
    if(stat(p,&sb)!=0) return 255;
    if(time(nullptr) - sb.st_mtime > 300) return 255;
    FILE* f=fopen(p,"rb"); if(!f) return 255;
    char tail[512]; long n=(long)sizeof(tail)-1;
    fseek(f,0,SEEK_END); long sz=ftell(f);
    if(sz<n) n=sz;
    fseek(f,-n,SEEK_END);
    size_t rd=fread(tail,1,(size_t)n,f); fclose(f);
    tail[rd]='\0';
    char* end=tail+rd;
    while(end>tail && (end[-1]=='\n'||end[-1]=='\r')) *--end='\0';
    char* line=strrchr(tail,'\n'); line = line ? line+1 : tail;
    const char* c1=strchr(line,',');   if(!c1) return 255;
    const char* c2=strchr(c1+1,',');   if(!c2) return 255;
    const char* c3=strchr(c2+1,',');   if(!c3) return 255;
    *ac_out = (c3[1]=='1') ? 1 : (c3[1]=='0' ? 0 : 2);
    // 여기까지 왔으면 로거가 살아 있다 = UPS 는 붙어 있다. soc 가 NA/범위밖이면
    // 게이지(i2c 0x36)가 답을 안 하는 것이므로 "없음"이 아니라 "고장"으로 구분한다.
    double soc=atof(c2+1);
    if(soc<=0.0 || soc>100.0) return 254;
    return (uint8_t)std::min(100,(int)(soc+0.5));
}
// 배터리 % 와 AC 연결 상태를 함께 읽는다.
//   반환   = 배터리 % (0-100) / 254 = UPS 있으나 게이지 무응답 / 255 = 배터리 없음
//   ac_out = 0 방전 중, 1 AC 연결, 2 알 수 없음
static uint8_t read_bat_pct(uint8_t* ac_out){
    *ac_out = 2;
    for(int i=0; i<4; i++){
        char p[80]; snprintf(p,sizeof(p),"/sys/class/power_supply/BAT%d/capacity",i);
        FILE* f=fopen(p,"r"); if(!f) continue;
        int cap=0; bool ok=(fscanf(f,"%d",&cap)==1); fclose(f);
        if(!ok) continue;
        snprintf(p,sizeof(p),"/sys/class/power_supply/BAT%d/status",i);
        FILE* fs=fopen(p,"r");
        if(fs){ char st[32]={}; if(fgets(st,sizeof(st),fs)) *ac_out=(strncmp(st,"Discharging",11)==0)?0:1; fclose(fs); }
        return (uint8_t)std::min(100,std::max(0,cap));
    }
    return read_bat_pct_csv(ac_out); // sysfs 없음 → UPS CSV 폴백
}
static long long read_io_ms(){
    FILE* f=fopen("/proc/diskstats","r"); if(!f) return 0;
    long long sum=0; char dev[32]; unsigned int maj,min_;
    long long f1,f2,f3,f4,f5,f6,f7,f8,f9,io_ticks;
    while(fscanf(f,"%u %u %31s %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %*[^\n]",
                 &maj,&min_,dev,&f1,&f2,&f3,&f4,&f5,&f6,&f7,&f8,&f9,&io_ticks)==13){
        if(dev[0]=='s'&&dev[2]>='a'&&dev[2]<='z'&&dev[3]=='\0') sum+=io_ticks;
        else if(dev[0]=='n'&&dev[1]=='v'&&strstr(dev,"p")==nullptr) sum+=io_ticks;
        else if(dev[0]=='v'&&dev[1]=='d'&&dev[3]=='\0') sum+=io_ticks;
    }
    fclose(f); return sum;
}

// ── Prompt helper (with default value) ───────────────────────────────────
static std::string prompt_input(const char* label, const char* def=nullptr){
    if(def && def[0])
        bewe_log_push(0,"%s [%s]: ", label, def);
    else
        bewe_log_push(0,"%s: ", label);
    fflush(stdout);
    char buf[128]={};
    if(!fgets(buf,sizeof(buf),stdin)){
        // Ctrl+C 또는 EOF (fgets가 EINTR로 중단되거나 stdin 닫힘)
        if(g_shutdown.load()) std::exit(0);
        return def ? def : "";
    }
    buf[strcspn(buf,"\r\n")]=0;
    if(buf[0]=='\0' && def) return def;
    return buf;
}

// ── stdin readline (non-blocking) ────────────────────────────────────────
// stdin EOF 플래그: printf 파이프 기동(fleet 표준) 시 로그인 입력 후 파이프가 닫혀
// EOF 가 된다. EOF 파이프에 poll(STDIN, timeout) 은 즉시 POLLHUP 리턴이라 메인 루프
// sleep 이 전혀 안 걸려 코어 하나를 통째로 스핀 — EOF 감지 후엔 plain sleep 사용.
static bool g_stdin_eof = false;
static bool read_line_nb(std::string& out){
    if(g_stdin_eof) return false;
    struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
    if(poll(&pfd,1,0)<=0) return false;
    char buf[512];
    if(!fgets(buf,sizeof(buf),stdin)){
        if(feof(stdin)) g_stdin_eof = true;   // 파이프 닫힘 — 이후 키보드 입력 없음
        return false;
    }
    size_t len=strlen(buf);
    while(len>0 && (buf[len-1]=='\n'||buf[len-1]=='\r')) buf[--len]=0;
    out=buf;
    return len>0;
}

// /mission [start|end|status] 공용 처리 — CLI stdin 과 채팅(JOIN/HOST) 양쪽에서 쓴다.
// who = 미션 시작자로 기록될 이름. reply = 결과 한 줄을 돌려줄 곳(채팅이면 방송, CLI면 로그).
// mission_start/end 가 내부에서 mission_broadcast_sync() 를 호출하므로 Central·JOIN 전파는 자동.
static void handle_mission_cmd(FFTViewer& v, const std::string& sub, const char* who,
                               const std::function<void(const char*)>& reply){
    char buf[256];
    if(sub.empty() || sub == "status"){
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        if(v.mission_state == Mission::State::ACTIVE){
            long elapsed = (long)(time(nullptr) - v.mission_start_utc);
            snprintf(buf, sizeof(buf), "Mission: ACTIVE %04d/%s by '%s' elapsed=%lds",
                     v.mission_year, v.mission_code, v.mission_started_by, elapsed);
        } else {
            snprintf(buf, sizeof(buf), "Mission: IDLE");
        }
        reply(buf);
    } else if(sub.rfind("start", 0) == 0){
        bool ok = v.mission_start(who ? who : "cli", /*op_index=*/0, /*rollover=*/false);
        if(ok) snprintf(buf, sizeof(buf), "Mission started: %04d/%s", v.mission_year, v.mission_code);
        else   snprintf(buf, sizeof(buf), "Mission already ACTIVE: %04d/%s", v.mission_year, v.mission_code);
        reply(buf);
    } else if(sub == "end"){
        bool ok = v.mission_end();
        snprintf(buf, sizeof(buf), ok ? "Mission ended." : "Mission end failed (none ACTIVE)");
        reply(buf);
    } else {
        reply("Usage: /mission [start|end|status]");
    }
}

// ══════════════════════════════════════════════════════════════════════════
void run_cli_host(){
    // SA_RESTART 없이 등록 → fgets 등 blocking syscall이 Ctrl+C에 EINTR로 중단됨
    struct sigaction sa{};
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;  // SA_RESTART 미설정
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // ── Interactive prompts ──────────────────────────────────────────────
    bewe_log_push(0,"\n=== WELCOME TO BEWE HOST CLI ====\n\n");

    std::string id_str = prompt_input("ID ");
    if(id_str.empty()){ bewe_log_push(0,"Aborted.\n"); return; }

    // 패스워드 에코 숨기기
    bewe_log_push(0,"PW : "); fflush(stdout);
    struct termios old_t, new_t;
    tcgetattr(STDIN_FILENO,&old_t); new_t=old_t;
    new_t.c_lflag &= ~(ECHO);
    tcsetattr(STDIN_FILENO,TCSANOW,&new_t);
    char pw_buf[64]={};
    if(!fgets(pw_buf,sizeof(pw_buf),stdin)){
        tcsetattr(STDIN_FILENO,TCSANOW,&old_t);
        if(g_shutdown.load()){ bewe_log_push(0,"\n"); std::exit(0); }
        bewe_log_push(0,"\nAborted.\n");
        return;
    }
    tcsetattr(STDIN_FILENO,TCSANOW,&old_t);
    pw_buf[strcspn(pw_buf,"\r\n")]=0;
    bewe_log_push(0,"\n");

    int tier = atoi(prompt_input("Tier ").c_str());
    if(tier<1||tier>2){ bewe_log_push(0,"CLI HOST requires tier 1 or 2.\n"); return; }

    std::string server_str = CENTRAL_DEFAULT_HOST;

    // ── Station selection ────────────────────────────────────────────────
    struct StationPreset { const char* name; float lat; float lon_e; };
    static const StationPreset presets[] = {
        { "DGS-1", 35.1786f, 128.5553f },
        { "DGS-2", 35.2054f, 128.7076f },
        { "DGS-3", 35.8685f, 128.6046f },
        { "DGS-X", 38.7000f, 125.3833f },   // 드론 탑재 Pi5 (raspb2). 38 42'N 125 23'E
    };
    const int n_preset = (int)(sizeof(presets)/sizeof(presets[0]));
    const int etc_choice = n_preset + 1;   // ETC 는 항상 프리셋 다음 번호
    bewe_log_push(0,"\n=== SELECT HOSTING LOCATION ===\n");
    for(int i=0;i<n_preset;i++)
        bewe_log_push(0,"%d. %s\n", i+1, presets[i].name);
    bewe_log_push(0,"%d. ETC\n\n", etc_choice);

    int loc_choice = 0;
    while(loc_choice < 1 || loc_choice > etc_choice){
        std::string s = prompt_input("> ");
        loc_choice = atoi(s.c_str());
        if(loc_choice < 1 || loc_choice > etc_choice)
            bewe_log_push(0,"Please enter 1-%d.\n", etc_choice);
    }

    float lat, lon;
    std::string station_str;
    if(loc_choice >= 1 && loc_choice <= n_preset){
        const StationPreset& p = presets[loc_choice - 1];
        station_str = p.name;
        lat = p.lat;
        lon = -p.lon_e;  // 동경(E) → 내부 규약(서경=양수이므로 부호 반전)
        bewe_log_push(0,"Station: %s  (%.4f N, %.4f E)\n", station_str.c_str(), lat, p.lon_e);
    } else {
        lat = atof(prompt_input("Lat ").c_str());
        lon = -atof(prompt_input("Lon ").c_str());
        station_str = prompt_input("Station ");
        if(station_str.empty()){ bewe_log_push(0,"Aborted.\n"); return; }
    }
    float cf  = 100.0f;

    bewe_log_push(0,"\n");

    // ── Login ────────────────────────────────────────────────────────────
    cli_login(id_str.c_str(), pw_buf, tier, server_str.c_str());
    bewe_log_push(0,"[BEWE CLI] Login: %s (Tier %d)\n", login_get_id(), login_get_tier());

    // ── FFTViewer init ───────────────────────────────────────────────────
    FFTViewer v;
    extern FFTViewer* g_log_viewer;
    g_log_viewer = &v;
    v.station_name = station_str;
    v.station_lat  = lat;
    v.station_lon  = lon;
    v.station_location_set = true;
    strncpy(v.host_name, login_get_id(), 31);

    // ── Restore saved host state — 재시작 시 직전 상태 그대로 (cf/sr 먼저) ──
    HostState::Snapshot saved_state = HostState::load(station_str);
    float init_sr = 0.f;
    // DF 설정은 initialize() 보다 먼저 넣어야 한다 — Kraken 백엔드가 기동할 때
    // 이 설정으로 엔진을 띄우기 때문. 채널 복원(apply_channels)보다 이르다.
    HostState::apply_df(v, saved_state);
    // 노치도 캡처 시작 전에 넣는다 — 첫 프레임부터 스컬치 계산이 제외 대역을 알아야
    // 한다 (안 그러면 상시 스퍼가 첫 캘리브레이션에 섞여 임계가 올라간다).
    HostState::apply_notches(v, saved_state);
    if(saved_state.ok){
        if(saved_state.cf_mhz >= 0.1f && saved_state.cf_mhz <= 6000.f) cf = saved_state.cf_mhz;
        if(saved_state.sr_msps >= 0.1f && saved_state.sr_msps <= 61.44f) init_sr = saved_state.sr_msps;
        bewe_log_push(0,"[BEWE CLI] restoring state for %s: cf=%.4f MHz sr=%.4f MSPS, %d channel(s)\n",
                      station_str.c_str(), cf, init_sr, saved_state.n_chans);
    }

    // ── SDR init ─────────────────────────────────────────────────────────
    // 부팅 직후 USB 재열거/느린 부팅과 겹칠 수 있어 5회(2초 간격)까지 재시도 후 포기.
    std::thread cap;
    bool sdr_ok = v.initialize(cf, init_sr);
    for(int retry = 2; retry <= 5 && !sdr_ok; retry++){
        bewe_log_push(0,"[BEWE CLI] SDR init failed, retry %d/5 in 2s ...\n", retry);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        sdr_ok = v.initialize(cf, init_sr);
    }
    if(!sdr_ok){
        bewe_log_push(0,"[BEWE CLI] SDR init failed after 5 attempts - running without hardware\n");
        v.sdr_stream_error.store(true);
        // SDR 없음 → 파일 분석 모드로 대기. 자동 재시도는 안 함 — 꽂히면 /rx start 로 수동 기동
        // (사용자 방향: JOIN 에서 SDR 상태등 보고 직접 /rx start).
        v.rx_stopped.store(true);
        v.fft_size = DEFAULT_FFT_SIZE * FFT_PAD_FACTOR;
        v.fft_input_size = DEFAULT_FFT_SIZE;
        v.header.fft_size  = DEFAULT_FFT_SIZE;
        v.header.power_min = -100.f;
        v.header.power_max = 0.f;
        v.display_power_min = -80.f;
        v.display_power_max = 0.f;
        v.fft_data.assign((size_t)FFT_HISTORY_ROWS * DEFAULT_FFT_SIZE * FFT_PAD_FACTOR, 0);
        v.current_spectrum.assign(DEFAULT_FFT_SIZE * FFT_PAD_FACTOR, -80.f);
        v.autoscale_active = false;
        v.create_waterfall_texture();
    } else {
        v.sdr_hw_present.store(true);
        bewe_log_push(0,"[BEWE CLI] SDR: %s detected\n",
               v.hw.type==HWType::BLADERF ? "BladeRF" :
               v.hw.type==HWType::PLUTO   ? "ADALM-Pluto" : "RTL-SDR");
        bewe_spawn_capture(v, cap);
        if(saved_state.ok && saved_state.has_gain){
            v.gain_db = saved_state.gain_db;
            v.set_gain(saved_state.gain_db);
        }
    }
    v.mix_stop.store(false);
    v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);

    // ── Long Waterfall worker (post-FFT image accumulator) ──────────────
    LongWaterfall::start_worker(&v);

    // ── SIGINT Mission: load history + start UTC0 rollover worker ───────
    v.mission_load_history();
    v.mission_save_meta_to_disk(); // persist any stale-entry closures
    v.mission_migrate_old_layout();   // v3.20.0 — legacy paths → station-keyed
    Mission::start_utc0_worker(&v);

    // ── NetServer ────────────────────────────────────────────────────────
    NetServer* srv = new NetServer();
    int host_port = 0;

    // Static state shared with callbacks
    std::vector<std::string> rec_iq_files;
    std::atomic<bool> pending_chassis1_reset{false};
    std::atomic<bool> pending_chassis2_reset{false};
    std::atomic<bool> pending_rx_stop{false};
    std::atomic<bool> pending_rx_start{false};
    // 복구 명령 3단계. 상위는 하위가 고치는 문제를 전부 포함한다.
    //   /chassis 1 reset    USB 재열거 + SDR 재초기화        (프로세스 유지)
    //   /powercycle partial 위 + BEWE 프로세스 재시작        (머신 유지)
    //   /powercycle full    위 + 머신 재부팅
    // 재열거를 상위 단계에서 빼면 포함관계가 깨진다 — 프로세스/머신만 새로 떠도
    // 장치가 굳어 있으면 그대로 다시 만난다.
    std::atomic<bool> pending_powercycle_partial{false};
    std::atomic<bool> pending_powercycle_full{false};
    bool usb_reset_pending = false;
    std::atomic<bool> ch_sync_dirty_flag{false};

    // Central client
    CentralClient central_cli;
    char central_host[128] = {};
    strncpy(central_host, login_get_server(), 127);
    const char* env_central = getenv("BEWE_CENTRAL");
    if(env_central && env_central[0]){
        strncpy(central_host, env_central, 127);
        central_host[127] = '\0';
        bewe_log_push(0,"[BEWE CLI] Central overridden: BEWE_CENTRAL=%s\n", central_host);
    }
    constexpr int central_port = CENTRAL_PORT;

    // ── Server callbacks (from ui.cpp 2243-2822) ─────────────────────────
    srv->cb.on_auth = [&,srv](const char* id, const char* pw,
                               uint8_t tier, uint8_t& idx) -> bool {
        static uint8_t next=1;
        idx = next++;
        if(next>MAX_OPERATORS) next=1;
        std::thread([&v](){
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            v.mission_broadcast_sync(); // AUTH_ACK 이후 전송 — pre-auth skip 방지
        }).detach();
        bewe_log_push(0,"[CLI] Client authenticated: idx=%d\n", idx);
        return true;
    };

    srv->cb.on_set_freq   = [&](const char* who, float cf){
        bewe_log_push(0, "[CMD:%s] Freq > %.3f MHz\n", who, cf);
        v.set_frequency(cf);
    };
    srv->cb.on_set_gain   = [&](const char* who, float db){
        bewe_log_push(0, "[CMD:%s] Gain > %.1f dB\n", who, db);
        v.gain_db=db; v.set_gain(db);
    };
    srv->cb.on_create_ch  = [&](int idx, float s, float e, const char* creator){
        if(idx<0||idx>=MAX_CHANNELS) return;
        bewe_log_push(0, "[CMD:%s] CH%d create s=%.4f e=%.4f bw=%.4f\n",
                      creator?creator:"?", idx, s, e, fabsf(e-s));
        v.stop_dem(idx);
        v.channels[idx].reset_slot();
        v.channels[idx].s=s; v.channels[idx].e=e;
        v.channels[idx].filter_active=true;
        strncpy(v.channels[idx].owner, creator?creator:"", 31);
        v.channels[idx].audio_mask.store(0xFFFFFFFFu & ~0x1u);
        v.local_ch_out[idx] = 3;
        // 생성 직후 범위 판정 (범위 밖이면 Holding으로) + CH_SYNC 브로드캐스트
        v.update_dem_by_freq(v.header.center_frequency/1e6f);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_module_pipe = [&](const uint8_t* pl, uint32_t len){
        bewe_mod_route(v, true, pl, len);
    };
    bewe_mod_set_broadcast([&v](const void* pl, uint32_t len){
        if(!v.net_srv) return false; v.net_srv->broadcast_module_pipe(pl, len); return true; });
    srv->cb.on_delete_ch  = [&](const char* who, int idx){
        if(idx<0||idx>=MAX_CHANNELS) return;
        bewe_log_push(0, "[CMD:%s] CH%d deleted\n", who, idx);
        if(v.channels[idx].audio_rec_on.load())
            v.stop_audio_rec(idx);
        v.stop_dem(idx);
        v.channels[idx].reset_slot();
        v.local_ch_out[idx] = 1;
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_ch_mode= [&](const char* who, int idx, int mode){
        if(idx<0||idx>=MAX_CHANNELS) return;
        if(mode<0||mode>2) return;   // NONE/AM/FM 외 거부 (구버전 JOIN 보호)
        static const char* mn[]={"NONE","AM","FM"};
        bewe_log_push(0, "[CMD:%s] CH%d mode > %s\n", who, idx, mn[mode]);
        v.stop_dem(idx,false);   // 오디오 모드만 변경 — IQ-탭 디코더 보존 (decode 는 모드 무관)
        auto dm=(Channel::DemodMode)mode;
        v.channels[idx].mode=dm;
        if(dm!=Channel::DM_NONE && v.channels[idx].filter_active)
            v.start_dem(idx,dm);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_ch_audio=[&](int idx, uint32_t mask){
        if(idx<0||idx>=MAX_CHANNELS) return;
        v.channels[idx].audio_mask.store(mask);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_ch_pan =[&](int idx, int pan){
        if(idx<0||idx>=MAX_CHANNELS) return;
        v.channels[idx].pan=pan;
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_sq_thresh = [&](int idx2, float thr){
        if(idx2<0||idx2>=MAX_CHANNELS) return;
        det_apply_sq_thresh(v.channels[idx2], thr);   // detect 채널이면 마진으로 클램프
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_autoscale = [&](){
        v.autoscale_req.store(true, std::memory_order_relaxed);  // 캡처 스레드가 처리 (레이스 방지)
        v.sq_recalib_req.store(true, std::memory_order_relaxed);
    };
    // JOIN 이 숫자키를 눌렀을 때. JOIN 에는 SDR 이 없으므로 HOST 가 대신 잰다.
    // 표시번호는 CHANNEL_SYNC 로 동기화된 채널 배열에서 나온 값이라 양쪽이 같다.
    // 결과·거절 사유는 df_pump 드레인이 broadcast_chat 으로 모두에게 돌려준다.
    // SNR 임계는 HOST 소유다. JOIN 이 DF 탭에서 바꾸면 이 명령으로 들어오고,
    // 적용 결과는 하트비트로 전원에게 되돌아간다.
    srv->cb.on_df_set_config = [&](const PktDfConfig& c){
        v.df_set_cfg(c);          // 적용 + 정본 재방송
    };
    srv->cb.on_df_set_snr = [&](int snr_db){
        PktDfConfig c{}; v.df_get_cfg(c);
        c.snr_thr_db = (float)snr_db;
        v.df_set_cfg(c);
    };
    srv->cb.on_df_measure = [&](int dnum){
        v.df_request_by_display_num(dnum);
    };
    srv->cb.on_set_ch_detect = [&](int idx, bool on){
        v.set_channel_detect(idx, on);
    };
    srv->cb.on_toggle_tm_iq = [&](){
        bool cur=v.tm_iq_on.load();
        if(cur){
            v.tm_iq_on.store(false); v.tm_add_event_tag(2); v.tm_iq_was_stopped=true;
            srv->broadcast_wf_event(0,(int64_t)time(nullptr),2,"IQ Stop");
        } else {
            if(v.tm_iq_was_stopped){ v.tm_iq_close(); v.tm_iq_was_stopped=false; }
            v.tm_iq_open();
            if(v.tm_iq_file_ready){
                v.tm_iq_on.store(true); v.tm_add_event_tag(1);
                srv->broadcast_wf_event(0,(int64_t)time(nullptr),1,"IQ Start");
            }
        }
    };
    srv->cb.on_set_capture_pause = [&](bool pause){
        v.capture_pause.store(pause);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_spectrum_pause = [&](bool pause){
        v.spectrum_pause.store(pause);
    };

    // JOIN per-channel IQ recording (HOST에서 대행)
    srv->cb.on_start_iq_rec = [&](uint8_t op_idx, const char* who, uint8_t ch_idx){
        if(ch_idx >= MAX_CHANNELS) return;
        if(!v.channels[ch_idx].filter_active || !v.channels[ch_idx].dem_run.load()) return;
        float bw = fabsf(v.channels[ch_idx].e - v.channels[ch_idx].s) * 1e3f; // kHz
        if(bw > 100.f){ bewe_log_push(0,"[CMD:%s] IQ REC ch%d denied: BW=%.0fkHz>100kHz\n", who, ch_idx, bw); return; }
        if(v.channels[ch_idx].iq_rec_on.load()){ bewe_log_push(0,"[CMD:%s] IQ REC ch%d already on\n", who, ch_idx); return; }
        v.start_iq_rec(ch_idx);
        bewe_log_push(0,"[CMD:%s] IQ REC start ch%d (%.0fkHz)\n", who, ch_idx, bw);
    };
    srv->cb.on_stop_iq_rec = [&](uint8_t op_idx, const char* who, uint8_t ch_idx){
        if(ch_idx >= MAX_CHANNELS) return;
        if(!v.channels[ch_idx].iq_rec_on.load()) return;
        v.stop_iq_rec(ch_idx);
        bewe_log_push(0,"[CMD:%s] IQ REC stop ch%d\n", who, ch_idx);
        // 녹음 파일을 요청한 JOIN에게 전송
        std::string path = v.channels[ch_idx].iq_rec_path;
        if(!path.empty()){
            static std::atomic<uint32_t> g_iq_req{2000};
            uint32_t req_id = g_iq_req.fetch_add(1);
            auto* central_ptr = &central_cli;
            std::thread([path, srv, req_id, op_idx, central_ptr](){
                FILE* fp = fopen(path.c_str(), "rb");
                if(!fp) return;
                fseek(fp, 0, SEEK_END); uint64_t fsz = (uint64_t)ftell(fp); fseek(fp, 0, SEEK_SET);
                const char* fn = strrchr(path.c_str(), '/');
                fn = fn ? fn+1 : path.c_str();
                uint32_t sr=0; { SigMF::Meta m; if(SigMF::read_meta(path,m)) sr=m.sample_rate; }
                // START
                { PktIqChunkHdr ch{}; ch.req_id=req_id; ch.seq=0;
                  strncpy(ch.filename, fn, 127); ch.filesize=fsz; ch.data_len=0; ch.sample_rate=sr;
                  auto bewe=make_packet(PacketType::IQ_CHUNK, &ch, sizeof(ch));
                  if(srv->cb.on_relay_broadcast) srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), true); }
                // DATA
                const size_t CHUNK=64*1024;
                std::vector<uint8_t> buf(sizeof(PktIqChunkHdr)+CHUNK);
                uint64_t sent=0; uint32_t seq=1;
                while(true){
                    size_t n=fread(buf.data()+sizeof(PktIqChunkHdr),1,CHUNK,fp);
                    if(n==0) break;
                    auto* ch=reinterpret_cast<PktIqChunkHdr*>(buf.data());
                    ch->req_id=req_id; ch->seq=seq++; strncpy(ch->filename,fn,127);
                    ch->filesize=fsz; ch->data_len=(uint32_t)n;
                    auto bewe=make_packet(PacketType::IQ_CHUNK,buf.data(),(uint32_t)(sizeof(PktIqChunkHdr)+n));
                    if(srv->cb.on_relay_broadcast) srv->cb.on_relay_broadcast(bewe.data(),bewe.size(),true);
                    sent+=n;
                    while(central_ptr->queue_bytes()>2*1024*1024)
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                fclose(fp);
                // END
                { PktIqChunkHdr ch{}; ch.req_id=req_id; ch.seq=0xFFFFFFFF;
                  strncpy(ch.filename,fn,127); ch.filesize=fsz; ch.data_len=0;
                  auto bewe=make_packet(PacketType::IQ_CHUNK,&ch,sizeof(ch));
                  if(srv->cb.on_relay_broadcast) srv->cb.on_relay_broadcast(bewe.data(),bewe.size(),true); }
                // 전송 완료 후 HOST에서 삭제 (data + .sigmf-meta sidecar)
                remove(path.c_str());
                remove(SigMF::sidecar_path(path).c_str());
                bewe_log_push(0,"[CLI] IQ REC transferred and deleted: %s\n", path.c_str());
            }).detach();
        }
    };


    // Region IQ request from JOIN
    srv->cb.on_request_region = [&](uint8_t op_idx, const char* op_name,
                                     int32_t fft_top, int32_t fft_bot,
                                     float freq_lo, float freq_hi,
                                     int64_t time_start_ms, int64_t time_end_ms,
                                     int64_t samp_start, int64_t samp_end){
        std::string fname;
        {
            std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
            FFTViewer::RecEntry e{};
            time_t t=time(nullptr); struct tm tm2; KST::to_tm(t,tm2);
            char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
            float cf_mhz = (freq_lo+freq_hi)/2.0f;
            char fn[128]; snprintf(fn,sizeof(fn),"IQ_%.3fMHz_%s.wav",cf_mhz,dts);
            e.filename = fn;
            e.is_region = true;
            e.req_state = FFTViewer::RecEntry::REQ_CONFIRMED;
            e.req_op_idx = op_idx;
            strncpy(e.req_op_name, op_name?op_name:"?", 31);
            e.req_fft_top=fft_top; e.req_fft_bot=fft_bot;
            e.req_freq_lo=freq_lo; e.req_freq_hi=freq_hi;
            e.req_time_start=time_start_ms/1000; e.req_time_end=time_end_ms/1000;
            e.t_start=std::chrono::steady_clock::now();
            v.rec_entries.push_back(e);
            fname = fn;
        }
        // JOIN 요청: time_start/time_end는 절대 wall_time
        // HOST에서 FFT 인덱스 변환 없이 time 기반으로 직접 샘플 위치 계산
        float fl=freq_lo, fh=freq_hi;
        uint8_t oidx=op_idx;
        std::string sid = v.station_name + "_" + std::string(login_get_id());
        static std::atomic<uint32_t> g_req_id{1000};
        uint32_t req_id_val = g_req_id.fetch_add(1);
        std::thread([&v,srv,fl,fh,time_start_ms,time_end_ms,samp_start,samp_end,oidx,fname,sid,&central_cli,req_id_val](){
          try {
            uint32_t req_id = req_id_val;
            for(int w=0;w<200&&v.rec_busy_flag.load();w++)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            v.region.fft_top=0; v.region.fft_bot=0; // 사용 안 함 (samp/time 기반)
            v.region.freq_lo=fl; v.region.freq_hi=fh;
            v.region.time_start_ms=time_start_ms;
            v.region.time_end_ms=time_end_ms;
            v.region.samp_start=samp_start;
            v.region.samp_end=samp_end;
            v.region.active=true;
            v.rec_busy_flag.store(true);
            v.rec_state = FFTViewer::REC_BUSY;
            v.rec_anim_timer = 0.0f;
            v.region.active = false;
            if(srv){
                PktIqProgress prog{};
                prog.req_id = req_id;
                strncpy(prog.filename, fname.c_str(), 127);
                prog.done=0; prog.total=0; prog.phase=0;
                srv->broadcast_iq_progress(prog);
            }
            bewe_log_push(0,"[CLI] region_save: tm_on=%d tm_write=%lld t_ms=%lld~%lld\n",
                (int)v.tm_iq_on.load(), (long long)v.tm_iq_write_sample,
                (long long)time_start_ms, (long long)time_end_ms);
            std::string path = v.do_region_save_work();
            v.rec_state = FFTViewer::REC_SUCCESS;
            v.rec_success_timer = 3.0f;
            v.rec_busy_flag.store(false);
            bewe_log_push(0,"[CLI] region_save done: path='%s'\n", path.c_str());
            if(path.empty()){
                if(srv) srv->send_region_response((int)oidx, false);
                return;
            }
            // 같은 PC 에서 HOST+JOIN 동시 운용 시, JOIN 의 저장 경로(no-mission: record/iq)와
            // HOST 전송용 사본이 동일 파일이 되어 HOST 의 전송후-삭제가 JOIN 파일을 지움.
            // 전송용 사본을 전용 temp(.region_tx)로 옮긴 뒤 거기서 송신·삭제 → JOIN 사본 보존.
            {
                std::string txdir = BEWEPaths::recordings_dir() + "/.region_tx";
                mkdir(txdir.c_str(), 0755);
                const char* bn = strrchr(path.c_str(), '/');
                std::string newpath = txdir + "/" + std::string(bn ? bn+1 : path.c_str());
                if(rename(path.c_str(), newpath.c_str()) == 0){
                    auto swap_ext = [](std::string p){
                        size_t pos = p.rfind(".sigmf-data");
                        if(pos != std::string::npos) p.replace(pos, 11, ".sigmf-meta");
                        return p;
                    };
                    rename(swap_ext(path).c_str(), swap_ext(newpath).c_str()); // meta sidecar best-effort
                    path = newpath;
                }
            }
            uint64_t fsz=0;
            {FILE* f=fopen(path.c_str(),"rb");if(f){fseek(f,0,SEEK_END);fsz=(uint64_t)ftell(f);fclose(f);}}
            // IQ chunk transfer via central relay
            if(srv && srv->cb.on_relay_broadcast){
                const char* fn_only2 = strrchr(path.c_str(), '/');
                fn_only2 = fn_only2 ? fn_only2+1 : path.c_str();
                bewe_log_push(0,"[CLI] IQ_CHUNK transfer start: req_id=%u file='%s' size=%.1fMB\n",
                       req_id, fn_only2, fsz/1048576.0);
                uint32_t rsr=0; { SigMF::Meta m; if(SigMF::read_meta(path,m)) rsr=m.sample_rate; }
                {
                    PktIqChunkHdr ch{};
                    ch.req_id = req_id; ch.seq = 0;
                    strncpy(ch.filename, fn_only2, 127);
                    ch.filesize = fsz; ch.data_len = 0; ch.sample_rate = rsr;
                    auto bewe = make_packet(PacketType::IQ_CHUNK, &ch, sizeof(ch));
                    srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), true);
                }
                auto* central_ptr = &central_cli;
                std::thread([&v, fname, path, fsz, srv, req_id,
                             fn2 = std::string(fn_only2), central_ptr](){
                    FILE* fp = fopen(path.c_str(), "rb");
                    if(!fp) return;
                    const size_t CHUNK = 64 * 1024;
                    std::vector<uint8_t> buf(sizeof(PktIqChunkHdr) + CHUNK);
                    uint64_t sent = 0; uint32_t seq = 1;
                    while(true){
                        size_t n = fread(buf.data() + sizeof(PktIqChunkHdr), 1, CHUNK, fp);
                        if(n == 0) break;
                        auto* ch = reinterpret_cast<PktIqChunkHdr*>(buf.data());
                        ch->req_id = req_id; ch->seq = seq++;
                        strncpy(ch->filename, fn2.c_str(), 127);
                        ch->filesize = fsz; ch->data_len = (uint32_t)n;
                        auto bewe = make_packet(PacketType::IQ_CHUNK, buf.data(), (uint32_t)(sizeof(PktIqChunkHdr)+n));
                        if(srv->cb.on_relay_broadcast)
                            srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), true);
                        sent += n;
                        // pacing: 큐가 2MB 넘으면 sender가 따라잡을 때까지 대기
                        while(central_ptr->queue_bytes() > 2*1024*1024)
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    fclose(fp);
                    {
                        PktIqChunkHdr ch{};
                        ch.req_id = req_id; ch.seq = 0xFFFFFFFF;
                        strncpy(ch.filename, fn2.c_str(), 127);
                        ch.filesize = fsz; ch.data_len = 0;
                        auto bewe = make_packet(PacketType::IQ_CHUNK, &ch, sizeof(ch));
                        if(srv->cb.on_relay_broadcast)
                            srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), true);
                    }
                    {
                        PktIqProgress prog{};
                        prog.req_id = req_id;
                        strncpy(prog.filename, fname.c_str(), 127);
                        prog.done = sent; prog.total = fsz; prog.phase = 2;
                        srv->broadcast_iq_progress(prog);
                    }
                    // 전송 완료 후 HOST temp 사본 삭제 (data + meta)
                    if(remove(path.c_str()) == 0)
                        bewe_log_push(0,"[CLI] region IQ transferred and deleted: %s\n", path.c_str());
                    {
                        std::string mp = path; size_t pp = mp.rfind(".sigmf-data");
                        if(pp != std::string::npos){ mp.replace(pp, 11, ".sigmf-meta"); remove(mp.c_str()); }
                    }
                }).detach();
            } else {
                // Direct TCP send
                srv->send_file_to((int)oidx, path.c_str(), 0);
            }
          } catch(const std::exception& e){
            bewe_log_push(0,"[CLI] region thread exception: %s\n", e.what());
            v.rec_busy_flag.store(false);
          } catch(...){
            bewe_log_push(0,"[CLI] region thread unknown exception\n");
            v.rec_busy_flag.store(false);
          }
        }).detach();
    };

    srv->cb.on_toggle_recv = [&](int ch_idx, uint8_t op_idx, bool enable){
        if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
        uint32_t bit = 1u << op_idx;
        uint32_t old_mask = v.channels[ch_idx].audio_mask.load();
        uint32_t new_mask;
        do {
            new_mask = enable ? (old_mask | bit) : (old_mask & ~bit);
        } while(!v.channels[ch_idx].audio_mask.compare_exchange_weak(old_mask, new_mask));
    };
    srv->cb.on_update_ch_range = [&](int idx, float s, float e){
        if(idx<0||idx>=MAX_CHANNELS) return;
        Channel& c = v.channels[idx];
        // detect 채널을 lock 아닌 상태에서 사용자가 넓히면(= 탐색 대역을 다시 그린 것)
        // det_s/det_e 도 따라가야 한다. arm 시점 값만 붙들고 있으면, 신호가 끝나 release
        // 될 때 ch.s/e 를 옛 폭으로 되돌려 사용자가 늘린 게 사라진다.
        // lock 중이라면 ch.s/e 는 잡은 신호의 폭이므로 탐색 대역으로 옮기지 않는다.
        if(c.det_on.load(std::memory_order_relaxed) &&
           !c.det_locked.load(std::memory_order_relaxed)){
            c.det_s = s; c.det_e = e;
            c.det_base_reset();   // 대역이 바뀌었으니 기준선은 무효 — 새로 쌓는다
        }
        c.s = s;
        c.e = e;
        if(v.channels[idx].dem_run.load()){
            Channel::DemodMode md = v.channels[idx].mode;
            v.stop_dem(idx,false); v.start_dem(idx, md);   // 재튜닝 — 디코더 보존
        }
        bewe_mod_ch_retune(v, idx);   // 디코더 새 band(주파수/대역폭) 로 재시작
        // 리사이즈로 범위 밖/안 전환될 수 있음 → 재평가
        v.update_dem_by_freq(v.header.center_frequency/1e6f);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_start_rec  = [&](int){ v.start_rec(); };
    srv->cb.on_stop_rec   = [&](){ v.stop_rec(); };
    // 채팅 수신 처리 — 두 경로에서 같이 쓴다.
    //  ① srv->cb.on_chat        : HOST 에 직접 붙은 JOIN
    //  ② set_on_central_chat    : Central 릴레이를 거쳐 온 JOIN (평소 운용은 전부 이쪽)
    // ②를 빼먹으면 Central 경유 채팅이 통째로 버려져 명령이 먹지 않는다.
    auto chat_handler = [&](const char* from, const char* msg){
        // 같은 한 줄이 ①②로 두 번 들어온다 — Central 이 소스 룸 HOST 에 포워드하면서
        // 전역 방송에도 실어 보내기 때문이다. 여기서 걸러내지 않으면 명령이 두 번
        // 실행되고(/hist check 가 2회 대조) 응답도 두 번 방송된다.
        // 같은 (from,msg) 가 1초 안에 또 오면 릴레이 중복으로 본다. 사람이 같은 줄을
        // 1초 안에 두 번 치는 경우는 실질적으로 없고, 있어도 잃는 건 채팅 한 줄이다.
        {
            static std::mutex dup_mtx;
            static std::string last_key;
            static std::chrono::steady_clock::time_point last_t{};
            std::string key = std::string(from ? from : "") + "\x01" + (msg ? msg : "");
            auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lk(dup_mtx);
            if(key == last_key && now - last_t < std::chrono::seconds(1)) return;
            last_key = key; last_t = now;
        }
        bewe_log_push(0,"[CHAT] %s: %s\n", from, msg);
        // 채팅으로 들어온 /mission 명령 처리 — JOIN 이든 HOST UI 든 동일 경로.
        // 결과는 SYSTEM 이름으로 방송해 모든 참가자가 보게 한다.
        if(strncmp(msg, "/mission", 8) == 0 && (msg[8] == 0 || msg[8] == ' ')){
            std::string sub(msg + 8);
            while(!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
            handle_mission_cmd(v, sub, from, [&](const char* r){
                bewe_log_push(0,"[CMD:%s] %s\n", from, r);
                if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", r);
            });
        }
        // "/hist check" — JOIN 이나 HOST UI 에서도 칠 수 있게 한다. 대상은 언제나
        // 이 기지(명령을 받은 HOST)의 로컬 HIST 뿐이다. 남의 기지 것은 애초에 여기
        // 없고 Central 도 요청한 룸의 기지 것만 답한다.
        else if(strncmp(msg, "/hist", 5) == 0 && (msg[5] == 0 || msg[5] == ' ')){
            std::string sub(msg + 5);
            while(!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
            if(sub.rfind("check", 0) == 0){
                bewe_log_push(0,"[CMD:%s] /hist check\n", from);
                // 결과는 대조가 끝난 뒤 HistCheck 워커가 직접 방송한다 (Fix/Del 줄).
                // 여기서 "started" 를 또 쏘면 채팅이 두 배로 시끄럽다.
                HistCheck::run_command(sub.c_str() + 5);
            } else if(v.net_srv){
                v.net_srv->broadcast_chat("SYSTEM", "Usage: /hist check");
            }
        }
        // "/powercycle partial|full" — /chassis 1 reset 이 안 먹을 때의 상위 복구.
        // JOIN 이든 HOST UI 든 여기로 모인다. 실제 동작은 메인 루프가 한다
        // (SDR 재초기화가 캡처 스레드 수명을 건드리므로 네트워크 스레드에선 안 된다).
        //
        // 인자는 필수다. full 은 머신을 재부팅하므로 오타나 습관적 입력으로 실행되면
        // 복구까지 1~2분이 날아간다 — 무엇을 하는지 명시하게 강제한다.
        else if(strncmp(msg, "/powercycle", 11) == 0 && (msg[11] == 0 || msg[11] == ' ')){
            const char* arg = msg + 11;
            while(*arg == ' ') arg++;
            if(strcmp(arg, "partial") == 0){
                bewe_log_push(0,"[CMD:%s] /powercycle partial\n", from);
                pending_powercycle_partial.store(true);
            } else if(strcmp(arg, "full") == 0){
                bewe_log_push(0,"[CMD:%s] /powercycle full\n", from);
                pending_powercycle_full.store(true);
            } else {
                bewe_log_push(2,"[CMD:%s] /powercycle needs an argument\n", from);
                if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM",
                    "Usage: /powercycle partial (restart BEWE) | /powercycle full (reboot machine)");
            }
        }
    };
    srv->cb.on_chat = chat_handler;
    central_cli.set_on_central_chat(chat_handler);

    srv->cb.on_set_fft_size = [&](const char* who, uint32_t size){
        bewe_log_push(0, "[CMD:%s] FFT size > %u\n", who, size);
        static const int valid[]={512,1024,2048,4096,8192,16384};
        for(int vs : valid)
            if((uint32_t)vs==size){ v.pending_fft_size=size; v.fft_size_change_req=true; break; }
    };
    srv->cb.on_set_sr = [&](const char* who, float msps){
        bewe_log_push(0, "[CMD:%s] SR > %.2f MSPS\n", who, msps);
        v.pending_sr_msps=msps; v.sr_change_req=true;
    };
    srv->cb.on_set_antenna = [&](const char* who, const char* antenna){
        bewe_log_push(0, "[CMD:%s] Antenna > '%s'\n", who, antenna?antenna:"");
        strncpy(v.host_antenna, antenna?antenna:"", sizeof(v.host_antenna)-1);
        v.host_antenna[sizeof(v.host_antenna)-1] = '\0';
    };
    srv->cb.on_set_hw = [&](const char* who, const char* sdr_name){
        bewe_log_push(0, "[CMD:%s] SDR switch > '%s'\n", who, sdr_name?sdr_name:"");
        std::string nm = sdr_name ? sdr_name : "";
        if(nm != "bladerf" && nm != "pluto" && nm != "rtlsdr") return;
        { std::lock_guard<std::mutex> lk(v.pending_sdr_mtx); v.pending_sdr_name = nm; }
        v.pending_sdr_switch.store(true);
    };

    // ── 예약 녹음 (JOIN → HOST) ───────────────────────────────────────────
    srv->cb.on_add_sched = [&](uint8_t op_idx, const char* op_name,
                                int64_t start_time, float duration_sec,
                                float freq_mhz, float bw_khz,
                                const char* target){
        if(duration_sec <= 0 || freq_mhz <= 0 || bw_khz <= 0){
            bewe_log_push(0,"[CMD:%s] SCHED add denied: invalid params\n", op_name);
            return;
        }
        time_t now = time(nullptr);
        if((time_t)start_time + (time_t)duration_sec < now){
            bewe_log_push(0,"[CMD:%s] SCHED add denied: past time\n", op_name);
            return;
        }
        {
            std::lock_guard<std::mutex> lk(v.sched_mtx);
            if(v.sched_has_overlap((time_t)start_time, duration_sec)){
                bewe_log_push(0,"[CMD:%s] SCHED add denied: overlap\n", op_name);
                return;
            }
            if((int)v.sched_entries.size() >= MAX_SCHED_ENTRIES){
                bewe_log_push(0,"[CMD:%s] SCHED add denied: list full\n", op_name);
                return;
            }
            FFTViewer::SchedEntry e;
            e.start_time   = (time_t)start_time;
            e.duration_sec = duration_sec;
            e.freq_mhz     = freq_mhz;
            e.bw_khz       = bw_khz;
            e.status       = FFTViewer::SchedEntry::WAITING;
            e.op_index     = op_idx;
            strncpy(e.operator_name, op_name?op_name:"", sizeof(e.operator_name)-1);
            strncpy(e.target,        target ?target :"", sizeof(e.target)-1);
            // Stamp with HOST's current active mission (empty if no active mission).
            {
                std::lock_guard<std::mutex> mlk(v.mission_mtx);
                if(v.mission_state == Mission::State::ACTIVE && v.mission_code[0]){
                    e.mission_year = v.mission_year;
                    memcpy(e.mission_code, v.mission_code, sizeof(e.mission_code));
                }
            }
            v.sched_entries.push_back(e);
            bewe_log_push(0,"[CMD:%s] SCHED added: %.3fMHz %.0fkHz dur=%.0fs target='%s' at %lld\n",
                          op_name, freq_mhz, bw_khz, duration_sec, e.target, (long long)start_time);
        }
        v.broadcast_sched_list();
    };
    srv->cb.on_remove_sched = [&](uint8_t op_idx, const char* op_name,
                                   int64_t start_time, float freq_mhz){
        bool removed = false;
        {
            std::lock_guard<std::mutex> lk(v.sched_mtx);
            for(auto it = v.sched_entries.begin(); it != v.sched_entries.end(); ++it){
                if((time_t)start_time != it->start_time) continue;
                if(fabsf(freq_mhz - it->freq_mhz) > 0.0001f) continue;
                // RECORDING/ARMED는 불가 (권한 검사는 누구나 가능하도록 제거)
                if(it->status == FFTViewer::SchedEntry::RECORDING ||
                   it->status == FFTViewer::SchedEntry::ARMED){
                    bewe_log_push(0,"[CMD:%s] SCHED remove denied: in progress\n", op_name);
                    return;
                }
                (void)op_idx;
                v.sched_entries.erase(it);
                removed = true;
                break;
            }
        }
        if(removed){
            bewe_log_push(0,"[CMD:%s] SCHED removed\n", op_name);
            v.broadcast_sched_list();
        }
    };

    // 예약 녹음 완료 시 자동 DB 업로드: HOST가 로컬 파일을 central DB로 전송
    // on_relay_broadcast가 central_cli에 연결되어 있으면 그 경로로, 아니면 로컬 DB 폴더에 복사
    v.sched_db_upload_fn = [&, srv](const std::string& path, const std::string& op, const std::string& info){
        // 동시에 여러 sched가 끝나면 Central의 room->db_fp 단일 슬롯이 META 도착 시
        // 덮어써져 직전 업로드가 truncate됨. 같은 HOST의 업로드를 직렬화해 보호.
        std::lock_guard<std::mutex> _ul(v.sched_db_upload_mtx);
        FILE* fp = fopen(path.c_str(), "rb");
        if(!fp){ bewe_log_push(0,"[SCHED-DB] open failed: %s\n", path.c_str()); return; }
        fseek(fp, 0, SEEK_END); long total = ftell(fp); fseek(fp, 0, SEEK_SET);
        if(total <= 0){ fclose(fp); return; }

        const char* slash = strrchr(path.c_str(), '/');
        const char* base = slash ? slash+1 : path.c_str();

        bool relay_ok = (srv && srv->cb.on_relay_broadcast) ? true : false;

        if(relay_ok){
            PktDbSaveMeta meta{};
            strncpy(meta.filename, base, sizeof(meta.filename)-1);
            meta.total_bytes = (uint64_t)total;
            static std::atomic<uint32_t> g_tid{1};
            meta.transfer_id = (uint8_t)(g_tid.fetch_add(1) & 0xFF);
            strncpy(meta.operator_name, op.c_str(), sizeof(meta.operator_name)-1);
            strncpy(meta.info_data, info.c_str(), sizeof(meta.info_data)-1);

            auto meta_pkt = make_packet(PacketType::DB_SAVE_META, &meta, sizeof(meta));
            srv->cb.on_relay_broadcast(meta_pkt.data(), meta_pkt.size(), true);

            constexpr size_t CHUNK = 64*1024;
            std::vector<uint8_t> buf(sizeof(PktDbSaveData) + CHUNK);
            long sent = 0;
            while(sent < total){
                size_t n = (size_t)std::min<long>(CHUNK, total - sent);
                auto* d = reinterpret_cast<PktDbSaveData*>(buf.data());
                d->transfer_id = meta.transfer_id;
                d->is_last     = (sent + (long)n >= total) ? 1 : 0;
                d->chunk_bytes = (uint32_t)n;
                if(fread(buf.data() + sizeof(PktDbSaveData), 1, n, fp) != n) break;
                auto data_pkt = make_packet(PacketType::DB_SAVE_DATA, buf.data(), sizeof(PktDbSaveData)+n);
                srv->cb.on_relay_broadcast(data_pkt.data(), data_pkt.size(), true);
                // backpressure: 큐 2MB 초과 시 대기 (파일 전체 RAM 상주 방지)
                while(central_cli.uplink_backlogged())
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                sent += (long)n;
            }
            bewe_log_push(0,"[SCHED-DB] uploaded %s (%ld bytes) by %s\n", base, total, op.c_str());
        } else {
            // 로컬 DB 폴더에 복사 (central 미연결 시 폴백)
            std::string db_path = BEWEPaths::database_dir() + "/" + base;
            FILE* out = fopen(db_path.c_str(), "wb");
            if(out){
                constexpr size_t BUF = 64*1024;
                std::vector<uint8_t> tmp(BUF);
                size_t n;
                while((n = fread(tmp.data(), 1, BUF, fp)) > 0) fwrite(tmp.data(), 1, n, out);
                fclose(out);
                std::string info_path = SigMF::sidecar_path(db_path);
                FILE* fi = fopen(info_path.c_str(), "w");
                if(fi){ fputs(info.c_str(), fi); fclose(fi); }
                bewe_log_push(0,"[SCHED-DB] saved locally: %s\n", db_path.c_str());
            }
        }
        fclose(fp);
    };
    srv->cb.on_chassis_reset = [&](const char* who){ bewe_log_push(0,"[CMD:%s] /chassis 1 reset\n",who); pending_chassis1_reset.store(true); };
    srv->cb.on_net_reset     = [&](const char* who){ bewe_log_push(0,"[CMD:%s] /chassis 2 reset\n",who); pending_chassis2_reset.store(true); };
    srv->cb.on_rx_stop       = [&](const char* who){ bewe_log_push(0,"[CMD:%s] /rx stop\n",who); pending_rx_stop.store(true); };
    srv->cb.on_rx_start      = [&](const char* who){ bewe_log_push(0,"[CMD:%s] /rx start\n",who); pending_rx_start.store(true); };

    // ── DB Save: JOIN이 파일을 Central DB에 저장 → HOST가 대행 ──────
    static struct { FILE* fp=nullptr; std::string path; uint8_t tid=0; } db_recv;
    srv->cb.on_db_save = [&](uint8_t op_idx, const char* op_name,
                              const PktDbSaveMeta* meta, const uint8_t* data, uint32_t len){
        if(meta){
            // META: 파일 열기 (flat — operator는 .info 의 Operator: 필드로 보존)
            mkdir(BEWEPaths::database_dir().c_str(), 0755);
            std::string dst = BEWEPaths::database_dir() + "/" + std::string(meta->filename);
            bewe_log_push(0,"[DB] Save '%s' by %s (%.1fMB)\n",
                meta->filename, meta->operator_name, meta->total_bytes/1048576.0);
            if(db_recv.fp) fclose(db_recv.fp);
            db_recv.fp = fopen(dst.c_str(), "wb");
            db_recv.path = dst;
            db_recv.tid = meta->transfer_id;
            // .info 저장
            if(meta->info_data[0]){
                std::string info_dst = SigMF::sidecar_path(dst);
                FILE* fi = fopen(info_dst.c_str(), "w");
                if(fi){ fwrite(meta->info_data, 1, strnlen(meta->info_data, sizeof(meta->info_data)-1), fi); fclose(fi); }
            }
        } else if(data && len >= sizeof(PktDbSaveData)){
            // DATA: 파일에 쓰기
            auto* d = reinterpret_cast<const PktDbSaveData*>(data);
            if(db_recv.fp && d->chunk_bytes > 0){
                fwrite(data + sizeof(PktDbSaveData), 1, d->chunk_bytes, db_recv.fp);
            }
            if(d->is_last && db_recv.fp){
                fclose(db_recv.fp);
                bewe_log_push(0,"[DB] Save complete: %s\n", db_recv.path.c_str());
                db_recv.fp = nullptr;
                db_recv.path.clear();
            }
        }
    };

    srv->cb.on_db_delete = [&](const char* who, const char* filename, const char* operator_name){
        // Central 연결 시 → Central로 포워드
        if(srv->cb.on_relay_broadcast){
            PktDbDeleteReq req{};
            strncpy(req.filename, filename, 127);
            strncpy(req.operator_name, operator_name, 31);
            auto pkt = make_packet(PacketType::DB_DELETE_REQ, &req, sizeof(req));
            srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
            bewe_log_push(0,"[CMD:%s] DB_DELETE '%s' by '%s' → Central\n", who, filename, operator_name);
        } else {
            // Central 없음 → 로컬 삭제 (flat — operator_name 무시)
            std::string fpath = BEWEPaths::database_dir() + "/" + filename;
            int r = remove(fpath.c_str());
            remove(SigMF::sidecar_path(fpath).c_str());
            bewe_log_push(0,"[CMD:%s] DB_DELETE '%s': %s\n", who, filename,
                          r==0 ? "OK" : strerror(errno));
        }
    };

    srv->cb.on_db_download_req = [&](uint8_t op_idx, const char* who, const char* filename, const char* operator_name){
        // Central 연결 시 → Central로 포워드
        if(srv->cb.on_relay_broadcast){
            PktDbDownloadReq req{};
            strncpy(req.filename, filename, 127);
            strncpy(req.operator_name, operator_name, 31);
            auto pkt = make_packet(PacketType::DB_DOWNLOAD_REQ, &req, sizeof(req));
            srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
            bewe_log_push(0,"[CMD:%s] DB_DOWNLOAD '%s' by '%s' → Central\n", who, filename, operator_name);
        } else {
            bewe_log_push(0,"[CMD:%s] DB_DOWNLOAD '%s': no Central, not supported\n", who, filename);
        }
    };

    // ── Start server ─────────────────────────────────────────────────────
    if(!srv->start(0)){
        bewe_log_push(0,"[BEWE CLI] Server start failed\n");
        delete srv; srv=nullptr;
        g_shutdown.store(true);
    } else {
        host_port = srv->listen_port();
        v.net_srv = srv;
        srv->set_host_info(login_get_id(), (uint8_t)login_get_tier());
        bewe_log_push(0,"[BEWE CLI] Server started on port %d\n", host_port);

        // Central MUX adapter
        if(central_host[0] != '\0'){
            std::string sid = v.station_name + "_" + std::string(login_get_id());
            // 부팅 직후 tailscaled 가 아직 tailnet 로그인을 못 마쳤을 때 open_room 이
            // 실패하는 경쟁조건이 있다 (2026-07-26 DGS-X 재부팅 실사고 — SDR/서버는
            // 정상 기동됐는데 Central 만 영구 고립). SDR init 과 같은 스타일로 짧게
            // 재시도한다 (2초x4회, 총 8초). 그래도 안 되면 콜백 등록은 그대로 진행하고,
            // 아래에서 reconnect_fn 을 즉시 1회 발동해 백그라운드 무한 재시도로 넘긴다.
            int rfd = central_cli.open_room(
                central_host, central_port, sid, v.station_name,
                v.station_lat, v.station_lon,
                (uint8_t)login_get_tier());
            for(int retry = 2; retry <= 5 && rfd < 0; retry++){
                bewe_log_push(0,"[CLI] Central open_room failed, retry %d/5 in 2s ...\n", retry);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                rfd = central_cli.open_room(
                    central_host, central_port, sid, v.station_name,
                    v.station_lat, v.station_lon,
                    (uint8_t)login_get_tier());
            }
            {
                bewe_mod_set_my_station(sid.c_str());
                central_cli.set_on_central_module_pipe([&v](const uint8_t* pkt, size_t len){
                    if(len > 9) bewe_mod_route(v, true, pkt+9, len-9);   // BEWE 헤더 스킵
                });
                // Relay CHANNEL_SYNC callback
                central_cli.set_on_central_ch_sync([&v](const uint8_t* pkt, size_t len){
                    if(len < 9) return;
                    size_t entry_sz = sizeof(ChSyncEntry); // 88 bytes
                    const uint8_t* payload = pkt + 9;      // BEWE 헤더(9) 스킵
                    size_t body_len = len - 9;
                    // body_len == entry_sz*MAX_CHANNELS → raw, 아니면 zstd 압축본 (v13)
                    static thread_local std::vector<uint8_t> dec;
                    if(body_len != entry_sz*MAX_CHANNELS){
                        dec.resize(entry_sz*MAX_CHANNELS);
                        size_t d = ZSTD_decompress(dec.data(), dec.size(), payload, body_len);
                        if(ZSTD_isError(d) || d != dec.size()) return;
                        payload = dec.data();
                    }
                    for(int i=0; i<MAX_CHANNELS; i++){
                        uint32_t old_mask = v.channels[i].audio_mask.load();
                        uint32_t mask;
                        memcpy(&mask, payload + i*entry_sz + 12, sizeof(mask));
                        // 진단: active 채널의 audio_mask 가 JOIN 비트 보유 → HOST 비트만 으로 collapse
                        // 되는 순간 로깅. v4.1.2 audio=0 stuck 추적용.
                        if(v.channels[i].filter_active
                           && (old_mask & ~0x1u) != 0
                           && (mask & ~0x1u) == 0){
                            bewe_log_push(0, "[CH%d] audio_mask collapsed 0x%x -> 0x%x\n",
                                          i, old_mask, mask);
                        }
                        v.channels[i].audio_mask.store(mask);
                    }
                });
                // Central DB 목록 수신
                extern std::vector<DbFileEntry> g_db_list;
                extern std::mutex g_db_list_mtx;
                central_cli.set_on_central_db_list([&](const uint8_t* pkt, size_t len){
                    extern std::vector<DbFileEntry> g_db_list;
                    extern std::mutex g_db_list_mtx;
                    if(len < 9 + sizeof(PktDbList)) return;
                    const uint8_t* payload = pkt + 9;
                    auto* hdr2 = reinterpret_cast<const PktDbList*>(payload);
                    uint16_t cnt2 = hdr2->count;
                    size_t expected = sizeof(PktDbList) + cnt2 * sizeof(DbFileEntry);
                    if(len - 9 < expected) return;
                    const DbFileEntry* ent = reinterpret_cast<const DbFileEntry*>(payload + sizeof(PktDbList));
                    std::vector<DbFileEntry> entries(ent, ent + cnt2);
                    { std::lock_guard<std::mutex> lk(g_db_list_mtx);
                      g_db_list = entries; }
                    // 직접 접속 JOIN에도 DB_LIST 전달
                    if(srv) srv->broadcast_db_list(entries);
                    bewe_log_push(0,"[Central] DB_LIST: %u files\n", cnt2);
                });

                // DB 다운로드 .info 수신 (Central → HOST) — .wav 보다 먼저 도착
                central_cli.set_on_central_db_dl_info([](const uint8_t* pkt, size_t len){
                    if(len < 9 + sizeof(PktDbDownloadInfo)) return;
                    const auto* di = reinterpret_cast<const PktDbDownloadInfo*>(pkt + 9);
                    char fn[129]={}; strncpy(fn, di->filename, 128);
                    bool is_iq = (is_iq_filename(fn));
                    std::string dir = is_iq ? BEWEPaths::record_iq_dir() : BEWEPaths::record_audio_dir();
                    mkdir(dir.c_str(), 0755);
                    std::string ipath = SigMF::sidecar_path(dir + "/" + fn);
                    FILE* fi = fopen(ipath.c_str(), "w");
                    if(fi){
                        size_t n = strnlen(di->info_data, sizeof(di->info_data));
                        if(n > 0) fwrite(di->info_data, 1, n, fi);
                        fclose(fi);
                        bewe_log_push(0,"[DB] Download meta saved: %s\n", ipath.c_str());
                    }
                });

                // DB 다운로드 데이터 수신 (Central → HOST)
                static FILE* host_db_dl_fp = nullptr;
                static std::string host_db_dl_path;
                central_cli.set_on_central_db_dl_data([&v](const uint8_t* pkt, size_t len){
                    if(len < 9 + sizeof(PktDbDownloadData)) return;
                    const auto* d = reinterpret_cast<const PktDbDownloadData*>(pkt + 9);
                    const uint8_t* data = pkt + 9 + sizeof(PktDbDownloadData);
                    uint32_t data_len = d->chunk_bytes;
                    if(d->is_first){
                        bool is_iq = (is_iq_filename(d->filename));
                        std::string dir = is_iq ? BEWEPaths::record_iq_dir() : BEWEPaths::record_audio_dir();
                        mkdir(dir.c_str(), 0755);
                        host_db_dl_path = dir + "/" + d->filename;
                        if(host_db_dl_fp) fclose(host_db_dl_fp);
                        host_db_dl_fp = fopen(host_db_dl_path.c_str(), "wb");
                        bewe_log_push(0,"[DB] Download start: %s (%.1fMB)\n", d->filename, d->total_bytes/1048576.0);
                    }
                    if(host_db_dl_fp && data_len > 0)
                        fwrite(data, 1, data_len, host_db_dl_fp);
                    if(d->is_last && host_db_dl_fp){
                        fclose(host_db_dl_fp);
                        host_db_dl_fp = nullptr;
                        bewe_log_push(0,"[DB] Download done: %s\n", host_db_dl_path.c_str());
                        host_db_dl_path.clear();
                    }
                });

                // Central에 저장된 예약 리스트를 HOST가 받아 v.sched_entries 복원
                central_cli.set_on_central_sched_sync([&v](const uint8_t* pkt, size_t len){
                    if(len < 9 + sizeof(PktSchedSync)) return;
                    auto* ss = reinterpret_cast<const PktSchedSync*>(pkt + 9);
                    int n = std::min<int>(ss->count, MAX_SCHED_ENTRIES);
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    // 현재 활성(ARM/REC) 엔트리가 있으면 그 식별자를 기억
                    int64_t active_st   = -1;
                    float   active_freq = 0.f;
                    if(v.sched_active_idx >= 0 && v.sched_active_idx < (int)v.sched_entries.size()){
                        active_st   = (int64_t)v.sched_entries[v.sched_active_idx].start_time;
                        active_freq = v.sched_entries[v.sched_active_idx].freq_mhz;
                    }
                    std::vector<FFTViewer::SchedEntry> next;
                    next.reserve(n);
                    int new_active = -1;
                    for(int i=0; i<n; i++){
                        const auto& se = ss->entries[i];
                        if(!se.valid) continue;
                        FFTViewer::SchedEntry ne;
                        ne.start_time   = (time_t)se.start_time;
                        ne.duration_sec = se.duration_sec;
                        ne.freq_mhz     = se.freq_mhz;
                        ne.bw_khz       = se.bw_khz;
                        ne.op_index     = se.op_index;
                        strncpy(ne.operator_name, se.operator_name, sizeof(ne.operator_name)-1);
                        strncpy(ne.target,        se.target,        sizeof(ne.target)-1);
                        // 활성 엔트리는 로컬 상태/타임스탬프/채널 유지
                        if(active_st == se.start_time && fabsf(active_freq - se.freq_mhz) < 1e-4f){
                            ne.status      = v.sched_entries[v.sched_active_idx].status;
                            ne.temp_ch_idx = v.sched_entries[v.sched_active_idx].temp_ch_idx;
                            ne.rec_started = v.sched_entries[v.sched_active_idx].rec_started;
                            new_active     = (int)next.size();
                        } else {
                            ne.status = (FFTViewer::SchedEntry::Status)se.status;
                        }
                        next.push_back(ne);
                    }
                    v.sched_entries = std::move(next);
                    v.sched_active_idx = new_active;
                    bewe_log_push(0, "[Central] restored %d scheduled entries\n", (int)v.sched_entries.size());
                });

                // ── Host-owned band plan (~/BEWE/band_plan.json) ────────
                // Load from disk on host start, mirror into v.band_segments,
                // accept JOIN/HOST edits, persist + rebroadcast.
                HostBandPlan::load_from_file();
                HostBandPlan::rebuild_cache();
                auto mirror_into_v = [&v](){
                    PktBandPlan bp{};
                    HostBandPlan::snapshot_pkt(bp);
                    std::lock_guard<std::mutex> lk(v.band_mtx);
                    v.band_segments.clear();
                    int n = std::min<int>((int)bp.count, MAX_BAND_SEGMENTS);
                    for(int i=0;i<n;i++){
                        const auto& be = bp.entries[i];
                        if(!be.valid) continue;
                        FFTViewer::BandSegment s;
                        s.freq_lo_mhz = be.freq_lo_mhz;
                        s.freq_hi_mhz = be.freq_hi_mhz;
                        s.category    = be.category;
                        strncpy(s.label,       be.label,       sizeof(s.label)-1);
                        strncpy(s.description, be.description, sizeof(s.description)-1);
                        v.band_segments.push_back(s);
                    }
                };
                mirror_into_v();
                auto rebroadcast_band_plan = [&v, mirror_into_v](){
                    HostBandPlan::save_to_file();
                    HostBandPlan::rebuild_cache();
                    PktBandPlan bp{};
                    HostBandPlan::snapshot_pkt(bp);
                    if(v.net_srv) v.net_srv->broadcast_band_plan(bp);
                    mirror_into_v();
                };
                srv->cb.on_band_add = [rebroadcast_band_plan](const PktBandEntry& e){
                    if(HostBandPlan::apply_add(e)) rebroadcast_band_plan();
                };
                srv->cb.on_band_update = [rebroadcast_band_plan](const PktBandEntry& e){
                    if(HostBandPlan::apply_update(e)) rebroadcast_band_plan();
                };
                srv->cb.on_band_remove = [rebroadcast_band_plan](const PktBandRemove& r){
                    if(HostBandPlan::apply_remove(r)) rebroadcast_band_plan();
                };

                // ── Host-owned band categories (~/BEWE/band_categories.json) ─
                HostBandCategories::load_from_file();
                HostBandCategories::rebuild_cache();
                auto rebroadcast_band_cat = [&v](){
                    HostBandCategories::save_to_file();
                    HostBandCategories::rebuild_cache();
                    PktBandCatSync cs{};
                    HostBandCategories::snapshot_pkt(cs);
                    if(v.net_srv) v.net_srv->broadcast_band_categories(cs);
                };
                srv->cb.on_band_cat_upsert = [rebroadcast_band_cat](const PktBandCategory& c){
                    if(HostBandCategories::apply_upsert(c)) rebroadcast_band_cat();
                };
                srv->cb.on_band_cat_delete = [rebroadcast_band_cat](uint8_t id){
                    if(HostBandCategories::apply_delete(id)) rebroadcast_band_cat();
                };

                // ── Long Waterfall: serve list + file download to JOINs ─
                srv->cb.on_lwf_list_req = [&v](int op_index, const char* who){
                    PktLwfList list{};
                    LongWaterfall::scan_dir_into_list(list);
                    if(v.net_srv) v.net_srv->send_lwf_list_to_op(op_index, list);
                    bewe_log_push(0, "[LWF] LIST_REQ from op=%d '%s' → %u files\n",
                                  op_index, who?who:"?", (unsigned)list.count);
                };
                srv->cb.on_lwf_dl_req = [&v](int op_index, const char* who, const char* fn){
                    if(!fn || !fn[0]) return;
                    if(strchr(fn, '/')) return;
                    std::string full = BEWEPaths::hist_host_dir() + "/" + fn;
                    std::string who_s = who ? who : "?";
                    static std::atomic<uint8_t> tid_ctr{1};
                    uint8_t tid = tid_ctr.fetch_add(1);
                    if(tid == 0) tid = tid_ctr.fetch_add(1);  // 0 reserved for non-HIST
                    bewe_log_push(0, "[LWF] DL_REQ from op=%d '%s' file=%s tid=%u\n",
                                  op_index, who_s.c_str(), fn, (unsigned)tid);
                    std::thread([&v, op_index, full, tid](){
                        if(v.net_srv) v.net_srv->send_file_to(op_index, full.c_str(), tid);
                    }).detach();
                };
                // STREAM opt-in (v4.6.0 제거): LWF_LIVE_REQ 폐기. JOIN은 미션창 archive 다운로드만.
                // Remote delete: JOIN이 host의 HIST 파일 삭제 요청. 현재 LIVE 파일은 보호.
                srv->cb.on_lwf_delete_req = [&v](int op_index, const char* who, const char* fn){
                    if(!fn || !fn[0] || strchr(fn, '/')) return;
                    PktLwfLiveStart ls{};
                    if(LongWaterfall::snapshot_live_start(ls) && std::string(ls.filename) == fn){
                        bewe_log_push(1, "[LWF] DEL_REQ refused (active LIVE): '%s' from op=%d\n",
                                      fn, op_index);
                        return;
                    }
                    std::string full = BEWEPaths::hist_host_dir() + "/" + fn;
                    if(unlink(full.c_str()) != 0){
                        bewe_log_push(1, "[LWF] DEL_REQ unlink failed: '%s' errno=%d from op=%d\n",
                                      fn, errno, op_index);
                        return;
                    }
                    bewe_log_push(0, "[LWF] DEL_REQ '%s' deleted by op=%d '%s'\n",
                                  fn, op_index, who?who:"?");
                    PktLwfList list{};
                    LongWaterfall::scan_dir_into_list(list);
                    if(v.net_srv) v.net_srv->send_lwf_list_to_op(op_index, list);
                };

                // ── SIGINT Mission System ──────────────────────────────
                // JOIN이 보낸 미션 명령을 HOST가 실행 (op_name으로 started_by 채움).
                srv->cb.on_mission_start = [&v](int op_index, const char* who){
                    bool ok = v.mission_start(who ? who : "join",
                                              (uint8_t)op_index, /*rollover=*/false);
                    bewe_log_push(0, "[CLI-HOST] on_mission_start op=%d who='%s' → ok=%d state=%d\n",
                                  op_index, who ? who : "", (int)ok, (int)v.mission_state);
                    // ok=false이면 mission_start 내부 broadcast가 안 일어남.
                    // 그 경우라도 현재 상태를 JOIN에 알려 stale UI를 갱신.
                    if(!ok) v.mission_broadcast_sync();
                };
                srv->cb.on_mission_end = [&v](int op_index, const char* who){
                    (void)op_index; (void)who;
                    v.mission_end();
                };
                srv->cb.on_mission_list_req = [&v](int op_index, const char* who){
                    (void)op_index; (void)who;
                    v.mission_broadcast_sync();
                };
                srv->cb.on_mission_delete = [&v](int op_index, const char* who,
                                                  const PktMissionDelete& d){
                    (void)op_index; (void)who;
                    char code[9] = {}; memcpy(code, d.code, 8);
                    v.mission_delete((int)d.year, code);
                };
                // MISSION_UPDATE는 자동 캡처 모델에서 의미 없음 — 콜백 등록 안 함.

                // 새 JOIN이 Central을 통해 들어오면 cached band plan + category 즉시 푸시
                central_cli.set_on_central_conn_open([&v, &central_cli](uint16_t cid){
                    std::vector<uint8_t> bp_pkt;
                    {
                        std::lock_guard<std::mutex> lk(HostBandPlan::g_mtx);
                        bp_pkt = HostBandPlan::g_cached_pkt;
                    }
                    std::vector<uint8_t> bc_pkt;
                    {
                        std::lock_guard<std::mutex> lk(HostBandCategories::g_mtx);
                        bc_pkt = HostBandCategories::g_cached_pkt;
                    }
                    bewe_log_push(0, "[HostBand] CONN_OPEN cid=%u → push plan %zu + cats %zu bytes\n",
                                  cid, bp_pkt.size(), bc_pkt.size());
                    if(!bc_pkt.empty())
                        central_cli.enqueue_relay_broadcast(bc_pkt.data(), bc_pkt.size(), true);
                    if(!bp_pkt.empty())
                        central_cli.enqueue_relay_broadcast(bp_pkt.data(), bp_pkt.size(), true);
                    // 설치된 모듈 상태 push (예: ACARS 채널 디코드 on/off)
                    bewe_mod_host_announce(v);
                    // 채널 sync 재push — Central cached_ch_sync 를 채워 타 기지 demod 통합목록(CH_LIST)에
                    // 이 기지 채널이 뜨게 함. boot 시 broadcast_channel_sync(line ~1489)는 relay 연결 전이라
                    // Central 캐시에 안 들어갈 수 있음. conn_open 시점(연결 확립)에 다시 보내야 빈 기지가 안 생김.
                    if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
                    // LIVE_START는 JOIN이 STREAM 버튼으로 명시 요청(LWF_LIVE_REQ)할 때만 unicast.
                    // mission_sync는 on_auth 200ms 스레드에서 AUTH_ACK 이후에 전송 (pre-auth 전송 시 JOIN이 skip함)
                });

                // 업링크 드롭 카운터를 LWF 에 물린다 — 파일 수명 동안 이 값이 늘면
                // Central 아카이브에 행이 빠진 것이므로 로컬본을 지우면 안 된다.
                LongWaterfall::set_drop_counter(
                    [&central_cli](){ return central_cli.drop_count(); });

                // Worker → NetServer LIVE broadcast 연결.
                LongWaterfall::LiveCallbacks lcb;
                lcb.on_start = [&v](const PktLwfLiveStart& s){
                    if(v.net_srv) v.net_srv->broadcast_lwf_live_start(s);
                };
                lcb.on_row   = [&v](const PktLwfLiveRowHdr& hdr,
                                    const uint8_t* row, uint32_t row_bytes){
                    if(v.net_srv) v.net_srv->broadcast_lwf_live_row(hdr, row, row_bytes);
                };
                lcb.on_stop  = [&v](const PktLwfLiveStop& s){
                    if(v.net_srv) v.net_srv->broadcast_lwf_live_stop(s);
                };
                LongWaterfall::set_live_callbacks(lcb);

                srv->cb.on_relay_broadcast = [&central_cli](const uint8_t* pkt, size_t len, bool no_drop){
                    central_cli.enqueue_relay_broadcast(pkt, len, no_drop);
                };
                // 부팅 시 mission_load_history → broadcast_sync는 net_srv/on_relay_broadcast가
                // 없을 때 호출돼서 Central 캐시가 비어있다. 여기서 한 번 더 broadcast해서
                // 신규 JOIN이 ACTIVE 상태를 받게 함.
                // 단 Central 로 나가는 몫은 여기서 보내면 버려진다 — enqueue_central 이
                // central_sender_running_ 를 보고 조용히 drop 하는데, 그 플래그는 아래
                // start_mux_adapter() 안에서야 true 가 된다. 그래서 실제 Central 전파는
                // start_mux_adapter() 직후에 다시 한다 (아래 참조).
                // Auto-reconnect function
                auto reconnect_fn = std::make_shared<std::function<void()>>();
                *reconnect_fn = [&v, &central_cli,
                                 rh = std::string(central_host), rp = central_port,
                                 reconnect_fn](){
                    // Central 연결 끊김 — 현재 HIST 파일을 dirty 표시.
                    // 끊긴 동안의 LIVE row 가 Central 에 도달 못 했을 수 있으므로
                    // finalize 시 통파일 push 로 보완.
                    LongWaterfall::mark_dirty();
                    bewe_log_push(0,"[CLI] Central disconnected — mark HIST file dirty\n");
                    std::thread([&v, &central_cli, rh, rp, reconnect_fn](){
                        for(int attempt=1; ; attempt++){
                            // 5초 대기를 0.1초 단위로 쪼개어 shutdown 즉시 반응
                            for(int i=0;i<50;i++){
                                if(g_shutdown.load()) return;
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            }
                            if(g_shutdown.load()) return;
                            if(!v.net_srv) return;
                            bewe_log_push(0,"[CLI] Central auto-reconnect attempt %d\n", attempt);
                            std::string sid2 = v.station_name + "_" + std::string(login_get_id());
                            int rfd2 = central_cli.open_room(
                                rh, rp, sid2, v.station_name,
                                v.station_lat, v.station_lon,
                                (uint8_t)login_get_tier());
                            if(rfd2 >= 0){
                                central_cli.start_mux_adapter(rfd2,
                                    [&v](int fd2){ if(v.net_srv) v.net_srv->inject_fd(fd2); },
                                    [&v](){ return v.net_srv ? (uint8_t)v.net_srv->client_count() : (uint8_t)0; },
                                    *reconnect_fn);
                                bewe_log_push(0,"[CLI] Central auto-reconnected\n");
                                // 재연결 직후 모듈 디코드 상태 재방송 (Central 의 새 빈 mod_mask 즉시 복구
                                // → JOIN 들이 stale decode_on=0 안 보게). JOIN CONN_OPEN 만 의존하지 않음.
                                bewe_mod_host_announce(v);
                                // 채널 sync 도 재방송 — Central 의 새 cached_ch_sync 를 채워 타 기지
                                // demod 통합목록(CH_LIST)에서 이 기지가 사라지지 않게.
                                if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
                                // 재연결 직후 미션이 ACTIVE면 Central에 상태 재동기화 + HIST 재개
                                if(v.mission_state == Mission::State::ACTIVE){
                                    bewe_log_push(0,"[CLI] re-broadcasting mission_sync after auto-reconnect\n");
                                    v.mission_broadcast_sync();
                                    LongWaterfall::request_rotate();
                                }
                                return;
                            }
                        }
                    }).detach();
                };
                if(rfd >= 0){
                    central_cli.start_mux_adapter(rfd,
                        [&v](int local_fd){ if(v.net_srv) v.net_srv->inject_fd(local_fd); },
                        [&v](){ return v.net_srv ? (uint8_t)v.net_srv->client_count() : (uint8_t)0; },
                        *reconnect_fn);
                    // 최초 연결도 재연결과 똑같이 처리해야 한다. start_mux_adapter 가
                    // central_sender_running_ 를 켠 "뒤"라야 mission_sync 가 Central 에
                    // 실제로 도달한다. 이게 없으면 Central 의 active_mission_valid 가
                    // false 로 남아 archive_hist_on_live_start 가 전부 반려되고 —
                    // 정각 rotate 로 나가는 이후 LIVE_START 까지 계속 반려된다 —
                    // 그 스테이션 HIST 가 아카이브에 통째로 안 남는다. 그런데 HOST 는
                    // 끊긴 적이 없어 finalize 를 CLEAN 으로 보고 로컬본까지 지운다.
                    // (실제 사고: 2026-07-27 DGS-2 19~21시 3시간분 영구 소실.)
                    if(v.mission_state == Mission::State::ACTIVE){
                        bewe_log_push(0,"[CLI-HOST] mission_sync + HIST rotate after relay connect\n");
                        v.mission_broadcast_sync();
                        LongWaterfall::request_rotate();
                    }
                } else {
                    // 짧은 재시도(위)도 실패 — reconnect_fn 을 그대로 발동시켜 무한
                    // 백그라운드 재시도로 넘긴다. mark_dirty() 는 여기선 무해(어차피
                    // 아직 연결된 적이 없으니 HIST 는 처음부터 dirty 로 시작하는 셈).
                    (*reconnect_fn)();
                }

                // Mission File Push worker (Phase 2, v3.8.0):
                // 미션 dir 안 닫힌 IQ/audio/hist 파일을 Central archive로 업로드,
                // ACK 받으면 로컬 unlink. central_cli mux가 동작해야 ACK 수신 가능.
                MissionPush::start(&v, &central_cli);

                // HIST 정합성 대조 워커. 정각 finalize 로 보존된 파일이 생기면
                // 그 미션 dir 을 대조한다 (하루치 일괄 스캔 아님).
                HistCheck::start(&v, &central_cli);
                central_cli.set_on_central_hist_stat(
                    [](const uint8_t* p, size_t n){ HistCheck::on_hist_stat(p, n); });
                LongWaterfall::set_on_retained(
                    [](const std::string& path){ HistCheck::notify_finalized(path); });
                // 지난 실행에서 ACK 못 받고 남은 파일 재투입 (큐가 메모리에만 있어
                // 재시작 때마다 고아가 누적됐다).
                MissionPush::scan_orphans_enqueue();
                central_cli.set_state_fn([&v](CentralHostStateFull& st){
                    const char* lid = login_get_id();
                    if(lid) strncpy(st.operator_login, lid, sizeof(st.operator_login)-1);
                    st.center_freq_hz = v.live_cf_hz.load();
                    st.sample_rate_hz = v.header.sample_rate;
                    PktLwfLiveStart lst{};
                    st.hist_recording = LongWaterfall::snapshot_live_start(lst) ? 1 : 0;
                    int cnt = 0;
                    for(int i=0; i<MAX_CHANNELS && cnt<CENTRAL_HSTATE_MAX_CHANNELS; i++){
                        const auto& c = v.channels[i];
                        if(!c.filter_active) continue;
                        auto& d = st.channels[cnt++];
                        d.active = 1;
                        d.mode = (uint8_t)c.mode;
                        d.iq_rec_on = c.iq_rec_on.load() ? 1 : 0;
                        d.audio_rec_on = c.audio_rec_on.load() ? 1 : 0;
                        d.dem_run = c.dem_run.load() ? 1 : 0;
                        d.s_mhz = c.s; d.e_mhz = c.e;
                        memcpy(d.owner, c.owner, sizeof(d.owner));
                    }
                    st.channel_count = (uint8_t)cnt;
                    st.bat_pct = v.sysmon_bat.load();
                });
                // Central 상태페이지의 live-HIST 정보. 예전엔 GUI-HOST 만 이걸 걸어서,
                // 무인 기지(cli_host)는 상태페이지에 HIST 가 비어 보였다.
                central_cli.set_hist_state_fn([](CentralHostHistInfo& hi) -> bool {
                    PktLwfLiveStart lst{};
                    if(!LongWaterfall::snapshot_live_start(lst)) return false;
                    memcpy(hi.filename, lst.filename, sizeof(hi.filename));
                    hi.start_utc_unix = lst.start_utc_unix;
                    hi.center_freq_hz = lst.center_freq_hz;
                    hi.sample_rate_hz = (uint32_t)lst.sample_rate_hz;
                    hi.fft_size       = lst.fft_size;
                    hi.row_rate_hz    = lst.row_rate_hz;
                    return true;
                });
                if(rfd >= 0)
                    bewe_log_push(0,"[BEWE CLI] Central relay connected\n");
                else
                    bewe_log_push(0,"[BEWE CLI] Central relay unavailable - retrying in background\n");
            }
        }

        // Broadcast thread
        v.net_bcast_stop.store(false);
        v.net_bcast_thr = std::thread(&FFTViewer::net_bcast_worker, &v);
    }

    // TM IQ(롤링 IQ 녹음) 기본 OFF — 원격 토글(on_toggle_tm_iq) 시 lazy open
    bewe_log_push(0,"[BEWE CLI] IQ rolling disabled (default off)\n");

    // ── System monitor state ─────────────────────────────────────────────
    long long cpu_last_idle=0, cpu_last_total=0, io_last_ms=0;
    read_cpu(cpu_last_idle, cpu_last_total);
    io_last_ms = read_io_ms();

    using clk = std::chrono::steady_clock;
    auto sysmon_last   = clk::now();
    auto sq_sync_last  = clk::now();
    auto status_last   = clk::now();
    auto heartbeat_last= clk::now();
    auto status_print_last = clk::now();
    // v4.5.2 — SDR 재연결 후 autoscale 지연 트리거 (RTL settling time 확보).
    // time_point{} 이면 대기 없음, 그 외 값이면 그 시각에 autoscale_active=true 적용.
    clk::time_point pending_autoscale_at{};
    auto loop_last     = clk::now();
    auto next_tick     = clk::now();  // 절대 데드라인 pacing 기준점

    // SDR reconnect state
    bool     bg_join_started = false;
    std::atomic<bool> cap_joined{false};
    bool     usb_reset_done = false;
    std::atomic<bool> usb_reset_in_progress{false};
    float    sdr_retry_timer = 0.f;
    float    chassis_unpause_timer = -1.f;

    bewe_log_push(0,"[BEWE CLI] Ready. Type /help for commands.\n");
    fflush(stdout);

    // ── Restore saved channel filters (net_srv 기동 후 — JOIN 에 sync) ────
    if(saved_state.ok && saved_state.n_chans > 0){
        HostState::apply_channels(v, saved_state);
        if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
        bewe_log_push(0,"[BEWE CLI] restored %d channel filter(s)\n", saved_state.n_chans);
        fflush(stdout);
    }
    uint64_t last_state_fp = HostState::fingerprint(v);

    // ══════════════════════════════════════════════════════════════════════
    //  Main loop
    // ══════════════════════════════════════════════════════════════════════
    while(!g_shutdown.load()){
        // ~50Hz loop (20ms sleep) — squelch 캘리브레이션이 1.2초 내 완료되도록
        // 다른 주기적 작업들은 자체 interval check 있어서 부하 무관
        auto now = clk::now();
        float dt = std::chrono::duration<float>(now - loop_last).count();
        loop_last = now;

        // ── DF 측정 결과 -> 로그 + 방송 ──────────────────────────────────
        // GUI 는 채팅 패널에도 넣지만 헤드리스는 로그/방송이 전부다.
        v.df_pump();
        if(v.pending_df_result.pending.exchange(false)){
            auto& r = v.pending_df_result;
            char line[256], logline[320];
            v.df_format_line(line,    sizeof line,    /*detailed=*/false);
            v.df_format_line(logline, sizeof logline, /*detailed=*/true);
            bewe_log_push(r.ok?0:2, "[DF] %s\n", logline);
            if(v.net_srv) v.net_srv->broadcast_chat("DF", line);
        }
        // 절대 데드라인 pacing — 기존 dt 기반 계산은 dt 에 이전 iteration 의 poll
        // sleep 이 포함되어 sleep/no-sleep 교대 발생, 실 루프가 ~95Hz 였음.
        next_tick += std::chrono::milliseconds(20);
        if(next_tick < now) next_tick = now; // 장시간 블록(rx stop join 등) 후 burst 방지
        int sleep_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(next_tick - now).count();
        if(sleep_ms > 0){
            if(g_stdin_eof){
                // 파이프 기동: EOF stdin 에 poll 하면 즉시 POLLHUP 리턴 → 스핀. plain sleep.
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            } else {
                struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
                int pr = poll(&pfd, 1, sleep_ms);
                // 데이터 없이 HUP/ERR 만 = 파이프 닫힘 확정
                if(pr > 0 && (pfd.revents & (POLLHUP|POLLERR)) && !(pfd.revents & POLLIN))
                    g_stdin_eof = true;
            }
        }

        // ── System monitor (1s) ──────────────────────────────────────────
        {
            float el = std::chrono::duration<float>(clk::now()-sysmon_last).count();
            if(el >= 1.0f){
                sysmon_last = clk::now();
                long long idle,total; read_cpu(idle,total);
                long long d_idle=idle-cpu_last_idle, d_total=total-cpu_last_total;
                v.sysmon_cpu=(d_total>0)?(1.0f-(float)d_idle/d_total)*100.0f:0.0f;
                cpu_last_idle=idle; cpu_last_total=total;
                v.sysmon_ghz=read_ghz();
                v.sysmon_ram=read_ram();
                v.sysmon_cpu_temp_c.store(read_cpu_temp_c());
                { uint8_t bac=2; v.sysmon_bat.store(read_bat_pct(&bac)); v.sysmon_bat_ac.store(bac); }
                long long io_now=read_io_ms();
                v.sysmon_io=std::min(100.0f,(float)(io_now-io_last_ms)/10.0f);
                io_last_ms=io_now;
                // Central 업로드 레이트 (1초 창) — heartbeat 로 JOIN STATUS 패널에 전달
                static ByteRateMeter up_meter;
                v.net_up_kbps.store(
                    (float)up_meter.sample(central_cli.stat_tx_total_bytes.load(std::memory_order_relaxed)));
            }
        }

        // ── 복조 의도 재조정 (1s): 필터 깜빡임/SDR 끊김으로 죽은 복조 자동 재시작 ──
        {
            static auto rec_last = clk::now();
            if(std::chrono::duration<float>(clk::now()-rec_last).count() >= 1.0f){
                rec_last = clk::now();
                bewe_mod_reconcile(v);
            }
        }

        // ── Persist host state on change (재시작 복원용 — 변경 즉시 저장) ──
        {
            uint64_t fp = HostState::fingerprint(v);
            if(fp != last_state_fp){
                HostState::save(v, station_str);
                last_state_fp = fp;
            }
        }

        // ── Periodic status print (30s) ──────────────────────────────────
        {
            float el = std::chrono::duration<float>(clk::now()-status_print_last).count();
            if(el >= 30.0f){
                status_print_last = clk::now();
                time_t t=time(nullptr); struct tm tm2; KST::to_tm(t,tm2);
                char ts[16]; strftime(ts,sizeof(ts),"%H:%M:%S",&tm2);
                int clients = v.net_srv ? v.net_srv->client_count() : 0;
                uint64_t net_tx=0, net_drops=0;
                if(v.net_srv){ auto ns=v.net_srv->collect_stats(); net_tx=ns.tx_bytes; net_drops=ns.drops; }
                bewe_log_push(0,"[%s] CF=%.1fMHz SR=%.2fM Clients=%d CPU=%.0f%% RAM=%.0f%% SDR=%s IQ=%s TX=%.1fMB Drops=%llu\n",
                       ts,
                       v.header.center_frequency/1e6,
                       v.header.sample_rate/1e6,
                       clients,
                       v.sysmon_cpu, v.sysmon_ram,
                       v.sdr_stream_error.load()?"ERR":(v.rx_stopped.load()?"STOP":"OK"),
                       v.tm_iq_on.load()?"ON":"OFF",
                       (double)net_tx/(1024*1024),
                       (unsigned long long)net_drops);
                fflush(stdout);
            }
        }

        // ── FFT_META (입력 크기) 1초 주기 + 신규 JOIN auth 시 즉시 ──────────
        // PktFftFrame 은 구 JOIN 호환 때문에 크기 동결 → 메타는 별도 패킷으로.
        if(v.net_srv && (v.net_srv->client_count() > 0 || v.net_srv->has_relay())){
            static auto fm_last = clk::now() - std::chrono::seconds(2);
            bool force = v.net_srv->fftmeta_force_.exchange(false, std::memory_order_relaxed);
            if(force || clk::now() - fm_last >= std::chrono::seconds(1)){
                fm_last = clk::now();
                v.net_srv->broadcast_fft_meta(v.fft_size, v.fft_input_size);
            }
        }

        // ── Squelch update (20ms, GUI 프레임레이트와 유사) + sync broadcast (100ms 유지) ─
        // 빠른 update는 스퀄치 캘리브레이션(60 샘플)이 ~1.2초 내 완료되도록 함
        if(v.net_srv){
            static auto sq_update_last = clk::now();
            float el_up = std::chrono::duration<float>(clk::now()-sq_update_last).count();
            // 0.018f: 20ms 격자와의 beat 방지 — 경계 지터로 격회 스킵(25Hz화) 막음
            if(el_up >= 0.018f){
                sq_update_last = clk::now();
                v.update_channel_squelch();
            }
        }
        if(v.net_srv && v.net_srv->client_count()>0){
            float el = std::chrono::duration<float>(clk::now()-sq_sync_last).count();
            if(el >= 0.2f){   // 5Hz (구 10Hz/0.1f) — chsync 대역 절반. 통계/스컬치 UI 5Hz로 충분
                sq_sync_last = clk::now();
                v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS, /*periodic=*/true);
            }
        }

        // ── Scheduled recording tick (1초마다) ───────────────────────────
        {
            static auto sched_last = clk::now();
            float el = std::chrono::duration<float>(clk::now()-sched_last).count();
            if(el >= 1.0f){
                sched_last = clk::now();
                // 상태 변화 감지용 스냅샷
                uint32_t before_hash = 0;
                {
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    for(auto& e : v.sched_entries)
                        before_hash = before_hash*131 + (uint32_t)e.status;
                }
                v.sched_tick();
                uint32_t after_hash = 0;
                {
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    for(auto& e : v.sched_entries)
                        after_hash = after_hash*131 + (uint32_t)e.status;
                }
                // 상태 전이 발생 시 JOIN들에게 즉시 통지
                if(before_hash != after_hash)
                    v.broadcast_sched_list();
            }
        }

        // ── Time tag + wf_event broadcast (5초마다) ─────────────────────
        {
            static int cli_last_tagged_sec = -1;
            time_t now_tt = time(nullptr);
            struct tm tt; KST::to_tm(now_tt, tt);
            int cur5 = tt.tm_hour*720 + tt.tm_min*12 + tt.tm_sec/5;
            if(cur5 != cli_last_tagged_sec){
                cli_last_tagged_sec = cur5;
                v.tm_add_time_tag(v.current_fft_idx);
                if(v.net_srv){
                    char lbl[32]; strftime(lbl, sizeof(lbl), "%H:%M:%S", &tt);
                    v.net_srv->broadcast_wf_event(0, (int64_t)now_tt, 0, lbl);
                }
            }
        }

        // ── STATUS broadcast (1s) ────────────────────────────────────────
        if(v.net_srv && v.net_srv->client_count()>0){
            float el = std::chrono::duration<float>(clk::now()-status_last).count();
            if(el >= 1.0f){
                status_last = clk::now();
                uint8_t hwt = (v.hw.type==HWType::RTLSDR) ? 1 :
                              (v.hw.type==HWType::PLUTO)  ? 2 :
                              (v.hw.type==HWType::KRAKEN) ? 3 : 0;
                v.net_srv->broadcast_status(
                    (float)(v.header.center_frequency/1e6),
                    v.gain_db, v.header.sample_rate, hwt);
            }
        }

        // ── Heartbeat (1s) ───────────────────────────────────────────────
        // ── /rx stop 대기 중 저빈도 presence-only 스캔 (3초 간격) ────────────
        // 자동 재시작은 안 함 — 표시(빨강/노랑)용. /rx start 는 사용자가 직접.
        if(v.rx_stopped.load()){
            static auto sdr_scan_last = clk::now() - std::chrono::seconds(3);
            if(std::chrono::duration<float>(clk::now()-sdr_scan_last).count() >= 3.0f){
                sdr_scan_last = clk::now();
                v.sdr_hw_present.store(scan_sdr_present_quiet());
            }
        }

        if(v.net_srv){
            float el = std::chrono::duration<float>(clk::now()-heartbeat_last).count();
            bool cur_sdr_err = v.sdr_stream_error.load();
            if(el >= 1.0f){
                heartbeat_last = clk::now();
                uint8_t sdr_t_hb = 0;
                if(v.dev_blade){
                    float _t = 0.f;
                    if(bladerf_get_rfic_temperature(v.dev_blade, &_t) == 0)
                        sdr_t_hb = (uint8_t)std::min(255.f, std::max(0.f, _t));
                } else if(v.pluto_ctx){
                    float _t = v.pluto_get_temp_c();
                    if(_t > 0.f) sdr_t_hb = (uint8_t)std::min(255.f, _t);
                }
                uint8_t hst = v.spectrum_pause.load() ? 2 : 0;
                // sdr_st: 0=OK(스트리밍 정상) 1=중단·SDR 없음(빨강) 2=중단·SDR 감지됨(노랑, /rx start 대기)
                //         3=스트림 에러(빨강, 뽑힘/초기화실패 아닌 런타임 오류) — 구 JOIN 은 !=0 이면 전부 빨강 취급
                uint8_t sdr_st = cur_sdr_err ? 3
                               : v.rx_stopped.load() ? (v.sdr_hw_present.load() ? 2 : 1)
                               : 0;
                uint8_t iq_st = v.tm_iq_on.load() ? 1 : 0;
                uint8_t cpu_pct  = (uint8_t)std::min(255.f, std::max(0.f, v.sysmon_cpu));
                uint8_t ram_pct  = (uint8_t)std::min(255.f, std::max(0.f, v.sysmon_ram));
                uint8_t cpu_temp = (uint8_t)std::min(255, std::max(0, v.sysmon_cpu_temp_c.load()));
                const char* sk = v.dev_blade ? "BladeRF" : v.pluto_ctx ? "Pluto" : v.dev_rtl ? "RTL-SDR" : "Unknown";
                uint32_t up_x100 = kbps_to_x100(v.net_up_kbps.load());
                // DF 가용도: 0=불가 1=가능(캘리 완료) 2=준비중/측정중.
                // JOIN 이 이걸 받아 HOST 와 같은 색으로 DF 램프를 그린다.
                const int dls = v.df_link_state();
                const uint8_t df_st = (v.hw.type != HWType::KRAKEN) ? 0
                                    : v.df_measuring()              ? 2
                                    : (dls == 2)                    ? 1
                                    : (dls == 1)                    ? 2 : 0;
                v.net_srv->broadcast_heartbeat(hst, sdr_t_hb, sdr_st, iq_st,
                                               cpu_pct, ram_pct, cpu_temp, v.host_antenna, sk,
                                               v.sysmon_bat.load(), up_x100, v.sysmon_bat_ac.load(),
                                               df_st, (int8_t)lrint(v.df_snr_threshold()));
            }
        }

        // ── Disk stat (5s): HOST recordings 디스크 여유공간 broadcast ────
        if(v.net_srv){
            static auto disk_stat_last = clk::now();
            float el = std::chrono::duration<float>(clk::now()-disk_stat_last).count();
            if(el >= 5.0f){
                disk_stat_last = clk::now();
                struct statvfs vfs{};
                std::string rec = BEWEPaths::recordings_dir();
                if(statvfs(rec.c_str(), &vfs) == 0){
                    uint64_t free_b  = (uint64_t)vfs.f_bavail * vfs.f_frsize;
                    uint64_t total_b = (uint64_t)vfs.f_blocks * vfs.f_frsize;
                    v.net_srv->broadcast_disk_stat(free_b, total_b, v.station_name.c_str());
                }
            }
        }

        // ── SDR 런타임 교체 ──────────────────────────────────────────────
        if(v.pending_sdr_switch.load()){
            v.pending_sdr_switch.store(false);
            std::string new_sdr;
            { std::lock_guard<std::mutex> lk(v.pending_sdr_mtx); new_sdr = v.pending_sdr_name; }
            bewe_log_push(0, "[CLI][SDR] switching to %s ...\n", new_sdr.c_str());
            float cur_cf = (float)(v.header.center_frequency / 1e6);
            for(int ci=0; ci<MAX_CHANNELS; ci++){ v.stop_dem(ci); }
            v.is_running = false;
            v.sdr_stream_error.store(true);
            if(cap.joinable()) cap.join();
            v.dev_blade = nullptr; v.dev_rtl = nullptr;
            v.pluto_ctx=nullptr; v.pluto_phy_dev=nullptr; v.pluto_rx_dev=nullptr;
            v.pluto_rx_i_ch=nullptr; v.pluto_rx_q_ch=nullptr; v.pluto_rx_buf=nullptr;
            g_sdr_force = new_sdr;
            v.is_running = true;
            if(v.initialize(cur_cf, 0.f)){
                v.set_gain(v.gain_db);
                v.sdr_stream_error.store(false);
                bewe_spawn_capture(v, cap);
                bewe_log_push(0,"[CLI][SDR] switched to %s\n", new_sdr.c_str());
            } else {
                bewe_log_push(2,"[CLI][SDR] switch to %s FAILED\n", new_sdr.c_str());
            }
        }

        // ── 복구 3단계 공통: SDR 정지 + USB 재열거 ────────────────────────
        // 세 단계(/chassis 1 reset, /powercycle partial, /powercycle full)가 전부
        // 이 루틴을 거친다. 상위 단계가 재열거를 건너뛰면 "상위는 하위를 포함한다"가
        // 깨진다 — 프로세스나 머신을 새로 띄워도 굳은 장치는 그대로 다시 만난다.
        //
        // 재초기화 자체는 하지 않고 sdr_stream_error 로 아래 reconnect 로직에
        // 위임한다. 그쪽이 이미 fft plan 재생성·dem_worker 재기동·autoscale
        // 재트리거를 다 하고 있어서, 여기서 중복 구현하면 어긋난다.
        auto stop_sdr_and_reenumerate = [&](const char* tag){
            uint16_t vid = 0, pid = 0; const char* pc_label = "SDR";
            sdr_usb_ids(v.hw.type, &vid, &pid, &pc_label);

            // 캡처 스레드를 먼저 내린다. 재열거 도중 살아 있으면 사라진 장치에
            // 계속 read 를 걸어 libusb 가 에러 폭주한다.
            v.is_running = false;
            v.sdr_stream_error.store(true);
            v.tm_iq_on.store(false);
            v.spectrum_pause.store(true);
            if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);

            // 여기서 그냥 join 하면 캡처가 블로킹 read 안에 갇혀 있을 때 메인 루프가
            // 통째로 멈춘다 — 이 명령들은 바로 그 상황을 풀려고 치는 것인데, 정작
            // 재열거 코드에 도달조차 못 하는 자기모순이 된다.
            // (2026-07-29 DGS-2: 무한 iio_buffer_refill 에 갇혀 명령 무반응)
            // 그래서 USB reset 으로 블로킹 read 를 먼저 깨워 스레드가 빠져나오게 한다.
            // 그래도 안 빠지면 detach 해서 버린다 — 계속 기다리느니 재열거를 진행하는
            // 편이 낫다. 재열거가 끝나면 그 스레드의 read 는 어차피 에러로 리턴하고
            // 루프가 종료된다(is_running=false).
            if(cap.joinable()){
                for(int attempt = 0; attempt < 5 && !v.cap_exited.load(); attempt++){
                    if(vid) usb_reset_vidpid(vid, pid, pc_label);   // 블로킹 read 깨우기
                    std::this_thread::sleep_for(std::chrono::milliseconds(400));
                }
                if(v.cap_exited.load()){
                    cap.join();
                } else {
                    bewe_log_push(2,"[CLI] %s: capture thread stuck - "
                                    "detaching and proceeding with re-enumeration\n", tag);
                    cap.detach();   // joinable 인 채로 재대입하면 std::terminate
                }
            }

            // 핸들을 닫아야 커널이 deauthorize 할 때 걸리지 않는다.
            if(v.dev_blade){
                bladerf_close(v.dev_blade); v.dev_blade = nullptr;
            }
            if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl = nullptr; }
            // Pluto: 정상 종료면 캡처 루프가 이미 정리했지만, 캡처가 뜨기 전에
            // 명령이 들어온 경우엔 컨텍스트가 살아 있다. 남으면 libiio 의 USB 클레임
            // 때문에 deauthorize 가 깨끗하게 안 된다.
            v.pluto_release();

            return std::make_tuple(vid, pid, pc_label);
        };

        // ── /chassis 1 reset — USB 재열거 + SDR 재초기화 ──────────────────
        // 예전엔 USBDEVFS_RESET 을 장치 핸들로 쏘는 약한 리셋이었다. 펌웨어 링크가
        // 죽어 있으면(BladeRF NIOS II timeout, Pluto USB wedge) 그 리셋조차 장치에
        // 안 닿아 몇 번을 쳐도 안 살아났다. 이제는 커널에게 unbind/재열거를 시켜
        // (authorized 0>1) 케이블을 뽑았다 꽂은 것과 같은 상태로 만든다.
        if(v.net_srv && pending_chassis1_reset.load()){
            pending_chassis1_reset.store(false);
            bewe_log_push(0,"[CLI] Chassis 1 reset: deep USB re-enumeration ...\n");
            v.net_srv->broadcast_chat("SYSTEM", "Chassis 1 reset ...");
            v.net_srv->broadcast_heartbeat(1);

            auto [vid, pid, pc_label] = stop_sdr_and_reenumerate("Chassis 1 reset");

            // 블로킹 구간(최소 2초 off + 재열거 대기)이라 메인 루프를 세우지 않도록
            // 별도 스레드에서 돌린다. 끝나면 reconnect 로직이 이어받는다.
            std::atomic<bool>* pc_busy = &usb_reset_in_progress;
            NetServer* pc_srv = v.net_srv;
            pc_busy->store(true);
            std::thread([vid, pid, pc_label, pc_busy, pc_srv](){
                int rc = 1;
                if(vid) rc = usb_deep_powercycle(vid, pid, 2000);
                if(rc == 2){
                    // udev rule 미배포 — 이 기지에선 딥 리셋이 불가능하다. 조용히
                    // 실패하면 운용자가 "쳤는데 왜 안 되지" 로 시간을 버리므로 명시한다.
                    bewe_log_push(2,"[CLI] Chassis 1 reset: no permission - falling back to "
                                    "USB reset (deploy 99-bewe-usb-powercycle.rules)\n");
                    if(pc_srv) pc_srv->broadcast_chat("SYSTEM",
                        "Chassis 1 reset: no permission - udev rule missing, using plain USB reset");
                    usb_reset_vidpid(vid, pid, pc_label);
                } else if(rc != 0){
                    bewe_log_push(2,"[CLI] Chassis 1 reset FAILED (rc=%d)\n", rc);
                    if(pc_srv) pc_srv->broadcast_chat("SYSTEM", "Chassis 1 reset FAILED - check station log");
                } else {
                    // 재열거가 끝나고 udev 가 권한을 다시 붙일 때까지 기다린다.
                    // 너무 빨리 bladerf_open 하면 장치는 보이는데 권한이 없어 실패한다.
                    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                    bewe_log_push(0,"[CLI] Chassis 1 reset done - reconnecting\n");
                    if(pc_srv) pc_srv->broadcast_chat("SYSTEM", "Chassis 1 reset done - reconnecting ...");
                }
                pc_busy->store(false);
            }).detach();

            // 캡처 스레드 회수는 위에서 끝냈다 (stuck 이면 detach 된 채로 둔다).
            // reconnect 로직이 다시 join 을 시도하지 않도록 상태를 정리해 둔다.
            bg_join_started = false;
            cap_joined.store(true);
            usb_reset_pending = false;
            usb_reset_done    = true;
        }

        // ── /powercycle partial|full ─────────────────────────────────────
        // chassis 1 reset 이 못 고치는 것 = 프로세스 자신의 상태다 (핸들 누수, 갇힌
        // 스레드, 메모리). 그건 프로세스를 새로 띄워야만 청소된다 — OS 가 fd 를
        // 강제 회수하기 때문이다. full 은 거기에 머신 재부팅까지 얹는다 (커널/드라이버
        // 레벨까지 초기화).
        //
        // 어느 쪽이든 먼저 USB 재열거를 돌린다. 안 그러면 새로 뜬 프로세스가 굳은
        // 장치를 그대로 다시 만난다 — 상위가 하위를 포함해야 한다는 요구가 깨진다.
        //
        // 재기동은 systemd 가 한다 (Restart=always). BEWE 는 그냥 종료하면 되고,
        // full 은 종료 전에 재부팅을 예약한다. host_state_<STATION>.json 이
        // cf/sr/gain/채널필터/디코드모듈을 이미 들고 있어 복귀 시 직전 상태로 돌아온다.
        if(v.net_srv && (pending_powercycle_partial.load() || pending_powercycle_full.load())){
            const bool full = pending_powercycle_full.load();
            pending_powercycle_partial.store(false);
            pending_powercycle_full.store(false);
            const char* tag = full ? "Power cycle full" : "Power cycle partial";

            bewe_log_push(0,"[CLI] %s: re-enumerating USB before restart ...\n", tag);
            v.net_srv->broadcast_chat("SYSTEM",
                full ? "Power cycle full: rebooting station ..."
                     : "Power cycle partial: restarting BEWE ...");
            v.net_srv->broadcast_heartbeat(1);

            auto [vid, pid, pc_label] = stop_sdr_and_reenumerate(tag);
            if(vid){
                int rc = usb_deep_powercycle(vid, pid, 2000);
                if(rc != 0){
                    bewe_log_push(2,"[CLI] %s: re-enumeration rc=%d - falling back to USB reset\n",
                                  tag, rc);
                    usb_reset_vidpid(vid, pid, pc_label);
                }
            }

            if(full) g_reboot_on_exit.store(true);
            else     g_restart_on_exit.store(true);
            // 종료 경로는 SIGINT 와 동일하다 — 녹음 flush, host_state 저장,
            // Central 정리까지 전부 그쪽이 한다. 여기서 따로 하면 이중 정리가 된다.
            g_shutdown.store(true);
        }

        if(v.net_srv && pending_chassis2_reset.load()){
            pending_chassis2_reset.store(false);
            bewe_log_push(0,"[CLI] Chassis 2 reset ...\n");
            if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "Chassis 2 reset ...");
            if(v.net_srv) v.net_srv->broadcast_heartbeat(2);
            v.net_bcast_pause.store(true, std::memory_order_relaxed);
            v.net_srv->pause_broadcast();
            v.net_srv->flush_clients();
            if(central_cli.is_central_connected())
                central_cli.send_net_reset(0);
            NetServer* srv_ptr = v.net_srv;
            std::atomic<bool>* bcast_pause_ptr = &v.net_bcast_pause;
            CentralClient* central_ptr = &central_cli;
            FFTViewer* vp = &v;
            std::string rh = central_host;
            int rp = central_port;
            std::thread([srv_ptr, bcast_pause_ptr, central_ptr, vp, rh, rp](){
                std::this_thread::sleep_for(std::chrono::seconds(1));
                srv_ptr->flush_clients();
                srv_ptr->resume_broadcast();
                bcast_pause_ptr->store(false, std::memory_order_relaxed);
                srv_ptr->broadcast_heartbeat(0);
                srv_ptr->broadcast_chat("SYSTEM", "Chassis 2 stable ...");
                if(!central_ptr->is_central_connected() && !rh.empty()){
                    central_ptr->stop_mux_adapter();
                    std::string sid = vp->station_name + "_" + std::string(login_get_id());
                    int rfd = central_ptr->open_room(
                        rh, rp, sid, vp->station_name,
                        vp->station_lat, vp->station_lon,
                        (uint8_t)login_get_tier());
                    if(rfd >= 0){
                        central_ptr->start_mux_adapter(rfd,
                            [vp](int fd2){ if(vp->net_srv) vp->net_srv->inject_fd(fd2); },
                            [vp](){ return vp->net_srv ? (uint8_t)vp->net_srv->client_count() : (uint8_t)0; });
                        bewe_log_push(0,"[CLI] Central reconnected after chassis 2 reset\n");
                    }
                } else if(central_ptr->is_central_connected()){
                    central_ptr->send_net_reset(1);
                }
                bewe_log_push(0,"[CLI] Chassis 2 stable\n");
            }).detach();
        }

        // RX stop/start from network
        if(v.net_srv && pending_rx_stop.load()){
            pending_rx_stop.store(false);
            if(!v.rx_stopped.load() && (v.is_running || cap.joinable())){
                bewe_log_push(0,"[CLI] RX stop (remote)\n");
                v.net_srv->broadcast_chat("SYSTEM", "RX stop");
                if(v.rec_on.load()) v.stop_rec();
                if(v.tm_iq_on.load()){ v.tm_iq_on.store(false); v.tm_iq_close(); }
                v.stop_all_dem();
                v.is_running = false;
                if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
                v.mix_stop.store(true);
                if(v.mix_thr.joinable()) v.mix_thr.join();
                if(cap.joinable()) cap.join();
                Mission::stop_utc0_worker();
                LongWaterfall::stop_worker();
                if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                if(v.dev_blade){
                    bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
                    bladerf_close(v.dev_blade); v.dev_blade=nullptr;
                }
                if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
                v.rx_stopped.store(true);
                v.sdr_stream_error.store(false);
                v.spectrum_pause.store(false);
                bewe_log_push(0,"[CLI] RX stopped\n");
            }
        }
        if(v.net_srv && pending_rx_start.load()){
            pending_rx_start.store(false);
            if(v.rx_stopped.load()){
                bewe_log_push(0,"[CLI] RX start (remote)\n");
                v.rx_stopped.store(false);
                float cur_cf = (float)(v.header.center_frequency / 1e6);
                if(cur_cf < 0.1f) cur_cf = 100.f;
                float cur_sr = v.header.sample_rate / 1e6f;
                if(cur_sr < 0.1f) cur_sr = 61.44f;
                v.is_running = true;
                if(v.initialize(cur_cf, cur_sr)){
                    v.set_gain(v.gain_db);
                    bewe_spawn_capture(v, cap);
                    v.mix_stop.store(false);
                    v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);
                    // RX stop 이 stop_worker() 로 HIST worker 를 죽였으므로 재개한다.
                    // start_worker 는 idempotent (이미 돌면 뷰어 rebind 만). initialize() 가
                    // SDR 을 새로 감지하므로 RTL->Pluto 교체 등 fft_size/sr 변경도 반영된다.
                    // rotate 로 현재 파일을 닫아 새 헤더(sr/fft)로 재생성시킨다.
                    LongWaterfall::start_worker(&v);
                    LongWaterfall::request_rotate();
                    // RX stop 이 죽인 미션 자정 rollover worker 도 재개 (idempotent).
                    Mission::start_utc0_worker(&v);
                    v.net_srv->broadcast_chat("SYSTEM", "RX start");
                    bewe_log_push(0,"[CLI] RX started\n");
                } else {
                    v.is_running = false;
                    v.rx_stopped.store(true);
                    bewe_log_push(0,"[CLI] RX start failed - SDR not found\n");
                }
            }
        }

        // ── Chassis 1 unpause timer ──────────────────────────────────────
        if(chassis_unpause_timer > 0.f){
            chassis_unpause_timer -= dt;
            if(chassis_unpause_timer <= 0.f){
                chassis_unpause_timer = -1.f;
                v.spectrum_pause.store(false);
                if(v.net_srv) v.net_srv->broadcast_heartbeat(0, 0, 0);
                bewe_log_push(0,"[CLI] chassis 1 reset: spectrum_pause released\n");
            }
        }

        // ── SR 변경 후 demod 재시작 (BladeRF/RTL-SDR/Pluto 공통) ────────
        if(v.dem_restart_needed.load()){
            v.dem_restart_needed.store(false);
            for(int di=0; di<MAX_CHANNELS; di++){
                if(v.channels[di].dem_run.load()){
                    auto dm = v.channels[di].mode;
                    v.stop_dem(di,false);   // SR 재시작 — 디코더 보존
                    v.start_dem(di, dm);
                }
                bewe_mod_ch_retune(v, di);   // SR 변경 → 디코더 decim/filter 재계산 위해 재시작
            }
            // SR 변경 → HIST 헤더의 sample_rate 가 파일당 고정이므로 현재 파일을 닫고
            // 새 sr 로 재생성한다. worker 는 g_fp==null 이면 g_v->header.sample_rate 를
            // 다시 읽어 새 파일을 연다 (long_waterfall.cpp worker_loop).
            LongWaterfall::request_rotate();
        }

        // ── 지연 autoscale 트리거 (SDR 재연결 settling 끝난 후 발동) ──────
        // autoscale_active/init/accum 은 캡처 스레드 소유다 (락 없이 hot loop 에서 읽고 쓴다).
        // 여기서 직접 건드리면 캡처 스레드가 변경을 못 보거나(평범한 bool — 캐싱 가능),
        // accum 을 동시에 만져 레이스가 난다. atomic 인 autoscale_req 만 세우고, 실제 리셋은
        // 캡처 스레드가 data_mtx 안에서 exchange 해 처리한다 (/rx autoscale 명령과 같은 경로).
        if(pending_autoscale_at != clk::time_point{} && clk::now() >= pending_autoscale_at){
            pending_autoscale_at = clk::time_point{};
            v.autoscale_req.store(true, std::memory_order_relaxed);
            bewe_log_push(0, "[autoscale] post-reconnect trigger (2s settling done)\n");
        }

        // ── FFT-stall watchdog ───────────────────────────────────────────
        // rtlsdr_read_sync 는 BULK_TIMEOUT=0 (무한) 이라 USB wedge 시 에러 없이
        // 영구 hang → sdr_stream_error 안 set → 아래 reconnect 영원히 안 탐 (silent
        // death, ~22h). total_ffts 정체를 직접 감지해 강제 복구. RTL 은 reopen 만으론
        // 안 풀려 USB reset 으로 hung read_sync 를 깨운 뒤 reconnect 에 위임. (v4.6.1)
        // USB reset 은 SDR 종류를 가리지 않는다 — 예전엔 RTL 만 했는데, Pluto 도
        // 같은 wedge 가 나므로 재초기화만 반복하며 안 풀렸다 (2026-07-29 DGS-2).
        {
            static int  wd_last_ffts = -1;
            static auto wd_last_change = clk::now();
            bool steady = !v.remote_mode && !v.rx_stopped.load() && v.is_running
                          && !v.sdr_stream_error.load() && !v.spectrum_pause.load();
            if(steady){
                int cur = v.total_ffts;
                if(cur != wd_last_ffts){
                    wd_last_ffts = cur;
                    wd_last_change = clk::now();
                } else if(std::chrono::duration<float>(clk::now()-wd_last_change).count() >= 10.0f){
                    bewe_log_push(0,"[CLI] SDR STALL: total_ffts frozen >=10s (read_sync hang) "
                                    "- forcing recovery\n");
                    // 깨워야 블로킹 read (read_sync / iio_buffer_refill) 가 에러 반환
                    uint16_t wvid=0, wpid=0; const char* wlabel=nullptr;
                    if(sdr_usb_ids(v.hw.type, &wvid, &wpid, &wlabel))
                        usb_reset_vidpid(wvid, wpid, wlabel);
                    v.sdr_stream_error.store(true);  // 아래 reconnect 트리거
                    wd_last_change = clk::now();      // 복구 중 재발화 방지
                }
            } else {
                wd_last_change = clk::now();  // 정상 capture 아니면 타이머 리셋
            }
        }

        // ── SDR reconnect logic ──────────────────────────────────────────
        if(!v.remote_mode && v.sdr_stream_error.load() && !v.rx_stopped.load()){
            if(!bg_join_started && v.hw.type == HWType::BLADERF)
                usb_reset_pending = true;
            if(!bg_join_started && cap.joinable()){
                bg_join_started = true;
                cap_joined.store(false);
                usb_reset_done = false;
                std::thread([&cap, &cap_joined](){
                    if(cap.joinable()) cap.join();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                    cap_joined.store(true);
                }).detach();
            } else if(!cap.joinable()){
                cap_joined.store(true);
            }

            // SDR 종류와 무관하게 USB 리셋을 건다. 예전엔 BladeRF 만 리셋하고
            // "Pluto/RTL-SDR: USB reset 불필요" 로 넘겼는데, 그러면 /chassis 1 reset
            // 이 하는 일이 재초기화뿐이라 stall watchdog 이 이미 10초마다 하던 것과
            // 같아진다 — USB 가 wedge 된 상태에선 몇 번을 쳐도 안 살아난다.
            // (2026-07-29 DGS-2 Pluto: STALL/reconnect 33회 무한루프)
            if(cap_joined.load() && usb_reset_pending && !usb_reset_done){
                usb_reset_done = true;
                usb_reset_pending = false;
                uint16_t rvid=0, rpid=0; const char* rlabel=nullptr;
                if(sdr_usb_ids(v.hw.type, &rvid, &rpid, &rlabel)){
                    usb_reset_in_progress.store(true);
                    // 핸들이 열려 있으면 리셋 후 stale fd 가 남는다. 종류별로 닫는다.
                    if(v.dev_blade){ bladerf_close(v.dev_blade); v.dev_blade=nullptr; }
                    if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
                    v.pluto_release();
                    std::thread([&usb_reset_in_progress, rvid, rpid, rlabel](){
                        bewe_log_push(0,"[CLI] chassis 1 reset: USB reset %s...\n", rlabel);
                        usb_reset_vidpid(rvid, rpid, rlabel);
                        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        usb_reset_in_progress.store(false);
                    }).detach();
                }
            }

            sdr_retry_timer -= dt;
            if(usb_reset_in_progress.load()) sdr_retry_timer = 1.f;
            if(sdr_retry_timer <= 0.f && cap_joined.load() && !usb_reset_in_progress.load()){
                sdr_retry_timer = 2.f;
                if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                float cur_cf = (float)(v.header.center_frequency / 1e6);
                if(cur_cf < 0.1f) cur_cf = 100.f;
                float cur_sr2 = v.header.sample_rate / 1e6f;
                if(cur_sr2 < 0.1f) cur_sr2 = 61.44f;
                v.is_running = true;
                if(v.initialize(cur_cf, cur_sr2)){
                    bewe_log_push(0,"[CLI] SDR reconnected - resuming at %.2f MHz\n", cur_cf);
                    v.sdr_stream_error.store(false);
                    bg_join_started = false;
                    cap_joined.store(false);
                    usb_reset_in_progress.store(false);
                    // v4.4.4 — SDR 재연결 후 dem_worker 들이 죽은 상태로 stuck 되어 있음.
                    // sdr_stream_error 가 true 였을 때 dem_worker loop 가 exit 했지만
                    // dem_run 은 true 그대로 → start_dem 도 무시. 강제 stop+start 사이클 트리거.
                    v.dem_restart_needed.store(true);
                    // v4.5.2 — autoscale 즉시 트리거하지 않고 2초 지연. 첫 1초 의 FFT 에는
                    // RTL-SDR 의 DC offset 정리 + USB 버퍼 transient 가 섞여 noise floor
                    // 추정이 +10 dB 가량 biased 됨 → pmin 잘못 잡혀 워터폴 contrast 어색.
                    // 2초 settling 후 깨끗한 신호로 캘리브.
                    pending_autoscale_at = clk::now() + std::chrono::seconds(2);
                    v.set_gain(v.gain_db);
                    bewe_spawn_capture(v, cap);
                    if(v.spectrum_pause.load())
                        chassis_unpause_timer = 1.f;
                    if(v.net_srv){
                        uint8_t hst = v.spectrum_pause.load() ? 2 : 0;
                        v.net_srv->broadcast_heartbeat(hst, 0, 0);
                    }
                } else {
                    v.is_running = false;
                }
            }
        }

        // ── stdin command processing ─────────────────────────────────────
        std::string line;
        while(read_line_nb(line)){
            if(line == "/shutdown"){
                g_shutdown.store(true);
            } else if(line == "/status"){
                int clients = v.net_srv ? v.net_srv->client_count() : 0;
                bewe_log_push(0,"  CF=%.3f MHz  SR=%.2f MSPS  Gain=%.1f dB\n",
                       v.header.center_frequency/1e6,
                       v.header.sample_rate/1e6,
                       v.gain_db);
                bewe_log_push(0,"  Clients=%d  SDR=%s  IQ=%s\n",
                       clients,
                       v.sdr_stream_error.load()?"ERROR":(v.rx_stopped.load()?"STOPPED":"OK"),
                       v.tm_iq_on.load()?"ON":"OFF");
                bewe_log_push(0,"  CPU=%.0f%%  RAM=%.0f%%  IO=%.0f%%  GHz=%.2f\n",
                       v.sysmon_cpu, v.sysmon_ram, v.sysmon_io, v.sysmon_ghz);
                if(v.net_srv){
                    auto ns = v.net_srv->collect_stats();
                    auto fb = [](uint64_t b) -> std::string {
                        char buf[32];
                        if(b < 1024)               snprintf(buf,sizeof(buf),"%llu B",(unsigned long long)b);
                        else if(b < 1024*1024)     snprintf(buf,sizeof(buf),"%.1f KB",(double)b/1024);
                        else if(b < 1024ULL*1024*1024) snprintf(buf,sizeof(buf),"%.1f MB",(double)b/(1024*1024));
                        else                       snprintf(buf,sizeof(buf),"%.2f GB",(double)b/(1024ULL*1024*1024));
                        return buf;
                    };
                    bewe_log_push(0,"  NET: TX=%s  RX=%s  Drops=%llu  Q(fft=%zu audio=%zu)\n",
                           fb(ns.tx_bytes).c_str(), fb(ns.rx_bytes).c_str(),
                           (unsigned long long)ns.drops, ns.q_fft, ns.q_audio);
                }
                fflush(stdout);
            } else if(line == "/clients"){
                if(v.net_srv){
                    auto ops = v.net_srv->get_operators();
                    bewe_log_push(0,"  Connected operators (%d):\n", (int)ops.size());
                    for(auto& op : ops)
                        bewe_log_push(0,"    [%d] %s (tier %d)\n", op.index, op.name, op.tier);
                } else {
                    bewe_log_push(0,"  No server running.\n");
                }
                fflush(stdout);
            } else if(line == "/chassis 1 reset"){
                bewe_log_push(0,"[CMD:CLI] /chassis 1 reset\n");
                if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "Chassis 1 reset ...");
                if(v.net_srv) v.net_srv->broadcast_heartbeat(1);
                if(v.is_running || cap.joinable()){
                    v.is_running = false;
                    v.sdr_stream_error.store(true);
                    v.tm_iq_on.store(false);
                    v.spectrum_pause.store(true);
                    usb_reset_pending = true;
                } else {
                    bewe_log_push(0,"[CLI] No SDR connected - skip HW reset\n");
                }
            } else if(line == "/chassis 2 reset"){
                bewe_log_push(0,"[CMD:CLI] /chassis 2 reset\n");
                pending_chassis2_reset.store(true);
            } else if(line == "/rx stop"){
                bewe_log_push(0,"[CMD:CLI] /rx stop\n");
                if(v.rx_stopped.load()){
                    bewe_log_push(0,"[CLI] RX already stopped.\n");
                } else if(!v.is_running && !cap.joinable()){
                    bewe_log_push(0,"[CLI] No SDR running.\n");
                } else {
                    bewe_log_push(0,"[CLI] RX stop\n");
                    if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "RX stop");
                    if(v.rec_on.load()) v.stop_rec();
                    if(v.tm_iq_on.load()){ v.tm_iq_on.store(false); v.tm_iq_close(); }
                    v.stop_all_dem();
                    v.is_running = false;
                    if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
                    v.mix_stop.store(true);
                    if(v.mix_thr.joinable()) v.mix_thr.join();
                    if(cap.joinable()) cap.join();
                    if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                    if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                    if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                    if(v.dev_blade){
                        bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
                        bladerf_close(v.dev_blade); v.dev_blade=nullptr;
                    }
                    if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
                    v.rx_stopped.store(true);
                    v.sdr_stream_error.store(false);
                    v.spectrum_pause.store(false);
                    bewe_log_push(0,"[CLI] RX stopped.\n");
                }
            } else if(line == "/rx start"){
                bewe_log_push(0,"[CMD:CLI] /rx start\n");
                if(!v.rx_stopped.load()){
                    bewe_log_push(0,"[CLI] RX already running.\n");
                } else {
                    bewe_log_push(0,"[CLI] RX start - initializing SDR ...\n");
                    v.rx_stopped.store(false);
                    float cur_cf = (float)(v.header.center_frequency / 1e6);
                    if(cur_cf < 0.1f) cur_cf = cf;
                    float cur_sr3 = v.header.sample_rate / 1e6f;
                    if(cur_sr3 < 0.1f) cur_sr3 = 61.44f;
                    v.is_running = true;
                    if(v.initialize(cur_cf, cur_sr3)){
                        v.set_gain(v.gain_db);
                        bewe_spawn_capture(v, cap);
                        v.mix_stop.store(false);
                        v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);
                        if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "RX start");
                        bewe_log_push(0,"[CLI] RX started. SDR online.\n");
                    } else {
                        v.is_running = false;
                        v.rx_stopped.store(true);
                        bewe_log_push(0,"[CLI] RX start failed - SDR not found.\n");
                    }
                }
            } else if(line.rfind("/freq", 0) == 0){
                // /freq <MHz> — 중심 주파수 변경 (HOST 자기 SDR + JOIN sync)
                const char* arg = line.c_str() + 5;
                while(*arg == ' ') arg++;
                if(*arg == 0){
                    bewe_log_push(0,"  Current CF=%.4f MHz. Usage: /freq <MHz>\n",
                                  v.header.center_frequency/1e6);
                } else {
                    float cf = (float)atof(arg);
                    if(cf < 0.1f || cf > 6000.f){
                        bewe_log_push(0,"  Invalid freq (0.1~6000 MHz): %s\n", arg);
                    } else {
                        bewe_log_push(0,"[CMD:CLI] /freq → %.4f MHz\n", cf);
                        v.set_frequency(cf);
                        if(v.net_srv){
                            uint8_t hwt = (v.hw.type==HWType::RTLSDR) ? 1 :
                                          (v.hw.type==HWType::PLUTO)  ? 2 :
                              (v.hw.type==HWType::KRAKEN) ? 3 : 0;
                            v.net_srv->broadcast_status(cf, v.gain_db,
                                                        v.header.sample_rate, hwt);
                        }
                    }
                }
                fflush(stdout);
            } else if(line.rfind("/sr", 0) == 0){
                // /sr <MSPS> — 샘플레이트 변경 (capture 루프가 sr_change_req 소비)
                const char* arg = line.c_str() + 3;
                while(*arg == ' ') arg++;
                if(*arg == 0){
                    bewe_log_push(0,"  Current SR=%.2f MSPS. Usage: /sr <MSPS>\n",
                                  v.header.sample_rate/1e6);
                } else {
                    float msps = (float)atof(arg);
                    if(msps < 0.1f || msps > 61.44f){
                        bewe_log_push(0,"  Invalid SR (0.1~61.44 MSPS): %s\n", arg);
                    } else {
                        bewe_log_push(0,"[CMD:CLI] /sr -> %.2f MSPS\n", msps);
                        v.pending_sr_msps = msps;
                        v.sr_change_req   = true;
                    }
                }
                fflush(stdout);
            } else if(line == "/powercycle" || line.rfind("/powercycle ", 0) == 0){
                // /chassis 1 reset 이 안 먹는(= 프로세스 자신이 문제인) 상태의 상위 복구.
                // 인자 필수 — full 은 머신을 재부팅하므로 습관적 입력으로 실행되면 안 된다.
                std::string arg = line.size() > 11 ? line.substr(11) : "";
                while(!arg.empty() && arg.front() == ' ') arg.erase(arg.begin());
                if(arg == "partial"){
                    bewe_log_push(0,"[CMD:CLI] /powercycle partial\n");
                    pending_powercycle_partial.store(true);
                } else if(arg == "full"){
                    bewe_log_push(0,"[CMD:CLI] /powercycle full\n");
                    pending_powercycle_full.store(true);
                } else {
                    bewe_log_push(2,"[CMD:CLI] Usage: /powercycle partial (restart BEWE) | "
                                    "/powercycle full (reboot machine)\n");
                }
                fflush(stdout);
            } else if(line == "/hist" || line.rfind("/hist ", 0) == 0){
                // /hist check — 보존된 로컬 .bewehist 를 Central 아카이브와 대조해
                // 빠진 구간만 올리고, 다 있으면 로컬본을 지운다.
                std::string sub = line.size() > 5 ? line.substr(5) : "";
                while(!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
                if(sub.rfind("check", 0) == 0){
                    HistCheck::run_command(sub.c_str() + 5);
                } else {
                    printf("  Usage: /hist check\n");
                }
                fflush(stdout);
            } else if(line == "/df" || line.rfind("/df ", 0) == 0){
                // /df <n> — 표시번호 n 의 채널 필터를 방탐 측정. GUI 숫자키와 같은 경로.
                // 결과·거절 사유는 메인 루프의 df_pump 드레인이 로그/방송으로 낸다.
                std::string sub = line.size() > 3 ? line.substr(3) : "";
                while(!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
                if(!sub.empty() && sub[0] >= '0' && sub[0] <= '9')
                    v.df_request_by_display_num(atoi(sub.c_str()));
                else
                    bewe_log_push(0,"  Usage: /df <filter number>   (the number drawn on the filter)\n");
                fflush(stdout);
            } else if(line == "/ch" || line.rfind("/ch ", 0) == 0){
                // /ch add <CF_MHz> <BW_kHz> [none|am|fm]  /  /ch list  /  /ch del <n>
                // 채널 필터 생성/목록/삭제 — 네트워크 CREATE_CH(+SET_CH_MODE)/DELETE_CH 경로와 동일 로직
                static const char* mn[] = {"NONE","AM","FM"};
                std::string sub = line.size() > 3 ? line.substr(3) : "";
                while(!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
                if(sub.empty() || sub == "help"){
                    bewe_log_push(0,"  Usage: /ch add <CF_MHz> <BW_kHz> [none|am|fm]\n");
                    bewe_log_push(0,"         /ch list\n");
                    bewe_log_push(0,"         /ch del <n>\n");
                } else if(sub.rfind("add", 0) == 0){
                    float cf2=0, bw_khz=0; char modebuf[16]={0};
                    int n = sscanf(sub.c_str()+3, "%f %f %15s", &cf2, &bw_khz, modebuf);
                    Channel::DemodMode dm = Channel::DM_NONE;
                    bool mode_ok = true;
                    if(n >= 3){
                        for(char* p=modebuf; *p; ++p) if(*p>='A'&&*p<='Z') *p+=32;
                        if(!strcmp(modebuf,"am"))        dm=Channel::DM_AM;
                        else if(!strcmp(modebuf,"fm"))   dm=Channel::DM_FM;
                        else if(!strcmp(modebuf,"none")) dm=Channel::DM_NONE;
                        else mode_ok=false;
                    }
                    if(n < 2){
                        bewe_log_push(0,"  Usage: /ch add <CF_MHz> <BW_kHz> [none|am|fm]\n");
                    } else if(!mode_ok){
                        bewe_log_push(0,"  Invalid mode '%s' (none|am|fm)\n", modebuf);
                    } else if(cf2 < 0.1f || cf2 > 6000.f){
                        bewe_log_push(0,"  Invalid CF (0.1~6000 MHz): %.4f\n", cf2);
                    } else if(bw_khz <= 0.f || bw_khz > 61440.f){
                        bewe_log_push(0,"  Invalid BW (0~61440 kHz): %.2f\n", bw_khz);
                    } else {
                        int slot=-1;
                        for(int i=0;i<MAX_CHANNELS;i++) if(!v.channels[i].filter_active){ slot=i; break; }
                        if(slot < 0){
                            bewe_log_push(0,"  No free channel slot (max %d)\n", MAX_CHANNELS);
                        } else {
                            float half = (bw_khz * 1e-3f) * 0.5f;
                            float s = cf2 - half, e = cf2 + half;
                            // create (mirror on_create_ch)
                            v.stop_dem(slot);
                            v.channels[slot].reset_slot();
                            v.channels[slot].s=s; v.channels[slot].e=e;
                            v.channels[slot].filter_active=true;
                            strncpy(v.channels[slot].owner, login_get_id(), 31);
                            v.channels[slot].audio_mask.store(0xFFFFFFFFu & ~0x1u);
                            v.local_ch_out[slot] = 3;
                            v.update_dem_by_freq(v.header.center_frequency/1e6f);
                            // set mode (mirror on_set_ch_mode)
                            if(dm != Channel::DM_NONE){
                                v.stop_dem(slot);
                                v.channels[slot].mode = dm;
                                if(v.channels[slot].filter_active) v.start_dem(slot, dm);
                            }
                            if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
                            bewe_log_push(0,"[CMD:CLI] CH%d create cf=%.4f bw=%.2fkHz mode=%s%s\n",
                                          slot, cf2, bw_khz, mn[dm],
                                          v.channels[slot].dem_paused.load()?" (Holding)":"");
                        }
                    }
                } else if(sub == "list"){
                    int cnt=0;
                    for(int i=0;i<MAX_CHANNELS;i++){
                        Channel& ch=v.channels[i];
                        if(!ch.filter_active) continue;
                        float cfm=(ch.s+ch.e)*0.5f, bwk=fabsf(ch.e-ch.s)*1e3f;
                        bewe_log_push(0,"  CH%d  cf=%.4f MHz  bw=%.2f kHz  mode=%s%s  owner=%s\n",
                                      i, cfm, bwk, mn[ch.mode],
                                      ch.dem_paused.load()?" (Holding)":"", ch.owner);
                        cnt++;
                    }
                    if(!cnt) bewe_log_push(0,"  No active channels.\n");
                } else if(sub.rfind("del", 0) == 0){
                    int idx=-1;
                    if(sscanf(sub.c_str()+3, "%d", &idx) != 1 || idx<0 || idx>=MAX_CHANNELS){
                        bewe_log_push(0,"  Usage: /ch del <n>  (0~%d)\n", MAX_CHANNELS-1);
                    } else if(!v.channels[idx].filter_active){
                        bewe_log_push(0,"  CH%d not active\n", idx);
                    } else {
                        // mirror on_delete_ch
                        if(v.channels[idx].audio_rec_on.load()) v.stop_audio_rec(idx);
                        v.stop_dem(idx);
                        v.channels[idx].reset_slot();
                        v.local_ch_out[idx] = 1;
                        if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
                        bewe_log_push(0,"[CMD:CLI] CH%d deleted\n", idx);
                    }
                } else {
                    bewe_log_push(0,"  Unknown /ch subcommand. Try: /ch help\n");
                }
                fflush(stdout);
            } else if(line == "/notch" || line.rfind("/notch ", 0) == 0){
                // /notch add <lo_MHz> <hi_MHz>  /  /notch list  /  /notch del <n>
                // 노치는 순수 로컬 표시/스컬치 제외 대역이다 — 와이어에 없다(JOIN 은
                // 자기 화면에 자기 노치를 건다). HostState 에 저장돼 재시작에도 남는다.
                std::string sub = line.size() > 6 ? line.substr(6) : "";
                while(!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
                if(sub.empty() || sub == "help"){
                    bewe_log_push(0,"  Usage: /notch add <lo_MHz> <hi_MHz>\n");
                    bewe_log_push(0,"         /notch list\n");
                    bewe_log_push(0,"         /notch del <n>\n");
                } else if(sub.rfind("add", 0) == 0){
                    float lo=0, hi=0;
                    if(sscanf(sub.c_str()+3, "%f %f", &lo, &hi) != 2){
                        bewe_log_push(0,"  Usage: /notch add <lo_MHz> <hi_MHz>\n");
                    } else {
                        if(lo > hi){ float t=lo; lo=hi; hi=t; }
                        if(lo < 0.f || hi > 6000.f || hi - lo < 1e-6f){
                            bewe_log_push(0,"  Invalid range: %.6f ~ %.6f MHz\n", lo, hi);
                        } else {
                            std::lock_guard<std::mutex> lk(v.notches_mtx);
                            if((int)v.notches.size() >= HostState::MAX_NOTCHES){
                                bewe_log_push(0,"  Notch limit reached (max %d)\n", HostState::MAX_NOTCHES);
                            } else {
                                FFTViewer::NotchFilter n;
                                n.freq_lo_mhz = lo; n.freq_hi_mhz = hi;
                                v.notches.push_back(n);
                                bewe_log_push(0,"[CMD:CLI] notch%d add %.6f ~ %.6f MHz (%.2f kHz)\n",
                                              (int)v.notches.size()-1, lo, hi, (hi-lo)*1e3f);
                            }
                        }
                    }
                } else if(sub == "list"){
                    std::lock_guard<std::mutex> lk(v.notches_mtx);
                    if(v.notches.empty()) bewe_log_push(0,"  No notches.\n");
                    for(size_t i=0;i<v.notches.size();i++)
                        bewe_log_push(0,"  notch%d  %.6f ~ %.6f MHz  (%.2f kHz)\n",
                                      (int)i, v.notches[i].freq_lo_mhz, v.notches[i].freq_hi_mhz,
                                      (v.notches[i].freq_hi_mhz - v.notches[i].freq_lo_mhz)*1e3f);
                } else if(sub.rfind("del", 0) == 0){
                    int idx=-1;
                    std::lock_guard<std::mutex> lk(v.notches_mtx);
                    if(sscanf(sub.c_str()+3, "%d", &idx) != 1 || idx < 0 || idx >= (int)v.notches.size()){
                        bewe_log_push(0,"  Usage: /notch del <n>  (0~%d)\n", (int)v.notches.size()-1);
                    } else {
                        v.notches.erase(v.notches.begin() + idx);
                        bewe_log_push(0,"[CMD:CLI] notch%d deleted\n", idx);
                    }
                } else {
                    bewe_log_push(0,"  Unknown /notch subcommand. Try: /notch help\n");
                }
                fflush(stdout);
            } else if(line == "/tm" || line.rfind("/tm ", 0) == 0){
                // /tm save <ch> [sec_ago] — TM 롤링 IQ 버퍼에서 과거 구간을 잘라 녹음.
                // GUI 는 스페이스바로 뷰를 얼리고 R 을 누르지만 CLI 엔 뷰가 없다.
                // 그 뷰 상태(freeze idx / offset / 선택 채널)를 인자로 대신 세운다.
                std::string sub = line.size() > 3 ? line.substr(3) : "";
                while(!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
                if(sub.empty() || sub == "help" || sub.rfind("save", 0) != 0){
                    bewe_log_push(0,"  Usage: /tm save <ch> [sec_ago]   (ch = /ch list index, default 0s = live)\n");
                } else {
                    int ch = -1; float sec_ago = 0.f;
                    int na = sscanf(sub.c_str()+4, "%d %f", &ch, &sec_ago);
                    if(na < 1 || ch < 0 || ch >= MAX_CHANNELS){
                        bewe_log_push(0,"  Usage: /tm save <ch> [sec_ago]  (ch 0~%d)\n", MAX_CHANNELS-1);
                    } else if(!v.channels[ch].filter_active){
                        bewe_log_push(0,"  CH%d not active\n", ch);
                    } else if(!v.tm_iq_on.load() || !v.tm_iq_file_ready){
                        bewe_log_push(0,"  TM IQ rolling is off - nothing buffered (JOIN can toggle it)\n");
                    } else if(v.mission_state != Mission::State::ACTIVE){
                        bewe_log_push(0,"  No ACTIVE mission - recordings have nowhere to go\n");
                    } else if(v.rec_on.load()){
                        bewe_log_push(0,"  Already recording (CH%d) - stop it first\n", v.rec_ch);
                    } else {
                        if(sec_ago < 0.f) sec_ago = 0.f;
                        // GUI 의 스페이스바 진입과 같은 상태를 만든다: 지금을 freeze 로
                        // 잡고 tm_update_display 가 offset->display_idx 를 계산하게 한다.
                        v.tm_freeze_idx = v.current_fft_idx;
                        v.tm_offset     = sec_ago;
                        v.tm_update_display();
                        int prev_sel = v.selected_ch;
                        v.selected_ch = ch;
                        bool ok = v.tm_rec_start();
                        if(!ok){
                            v.selected_ch = prev_sel;
                            bewe_log_push(2,"  TM save refused (requested %.1fs ago, available %.1fs)\n",
                                          sec_ago, v.tm_max_sec);
                        } else {
                            bewe_log_push(0,"[CMD:CLI] TM save CH%d %.1fs ago (window %.1fs)\n",
                                          ch, v.tm_offset, v.tm_max_sec);
                        }
                    }
                }
                fflush(stdout);
            } else if(line.rfind("/mission", 0) == 0){
                // /mission start [comment]  /  /mission end  /  /mission status
                // 채팅 경로와 동일한 헬퍼를 쓴다 (동작 어긋남 방지).
                std::string sub = line.substr(8);
                while(!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
                const char* who = login_get_id();
                handle_mission_cmd(v, sub, who ? who : "cli", [&](const char* r){
                    bewe_log_push(0,"  %s\n", r);
                    if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", r);
                });
                fflush(stdout);
            } else if(line == "/help"){
                bewe_log_push(0,"Commands:\n");
                bewe_log_push(0,"  /status          - Show system status\n");
                bewe_log_push(0,"  /clients         - List connected operators\n");
                bewe_log_push(0,"  /freq <MHz>      - Change center frequency\n");
                bewe_log_push(0,"  /sr <MSPS>       - Change sample rate\n");
                bewe_log_push(0,"  /hist check             - Verify local HIST against Central, upload missing rows\n");
                bewe_log_push(0,"  /ch add <CF> <BW> [mode] - Create channel filter (CF MHz, BW kHz, mode none|am|fm)\n");
                bewe_log_push(0,"  /ch list         - List active channel filters\n");
                bewe_log_push(0,"  /ch del <n>      - Delete channel filter n\n");
                bewe_log_push(0,"  /notch add <lo> <hi> - Mask a band (MHz) from display + squelch\n");
                bewe_log_push(0,"  /notch list      - List notches\n");
                bewe_log_push(0,"  /notch del <n>   - Delete notch n\n");
                bewe_log_push(0,"  /tm save <ch> [sec_ago] - Save TM rolling IQ for a channel\n");
                bewe_log_push(0,"  /mission start   - Begin a new mission\n");
                bewe_log_push(0,"  /mission end     - End active mission\n");
                bewe_log_push(0,"  /mission status  - Show current mission state\n");
                // 복구 3티어 — 낮은 것부터. 위로 갈수록 잃는 범위가 커진다.
                bewe_log_push(0,"  /chassis 1 reset    - USB re-enumerate + SDR reinit (keeps process)\n");
                bewe_log_push(0,"  /chassis 2 reset    - Network broadcast reset\n");
                bewe_log_push(0,"  /powercycle partial - above + restart BEWE (keeps machine)\n");
                bewe_log_push(0,"  /powercycle full    - above + reboot machine\n");
                bewe_log_push(0,"  /rx stop         - Stop SDR capture\n");
                bewe_log_push(0,"  /rx start        - Restart SDR capture\n");
                bewe_log_push(0,"  /shutdown        - Clean exit\n");
                bewe_log_push(0,"  /help            - Show this help\n");
                bewe_log_push(0,"  <text>           - Broadcast as chat message\n");
                fflush(stdout);
            } else if(!line.empty()){
                // Chat message
                if(v.net_srv)
                    v.net_srv->broadcast_chat(login_get_id(), line.c_str());
                bewe_log_push(0,"[CHAT] %s: %s\n", login_get_id(), line.c_str());
            }
        }
    }

    // 종료 직전 마지막 상태 저장 (graceful) — 재시작 시 그대로 복원
    HostState::save(v, station_str);

    // ══════════════════════════════════════════════════════════════════════
    //  Cleanup
    // ══════════════════════════════════════════════════════════════════════
    bewe_log_push(0,"[BEWE CLI] Shutting down...\n");

    // 0) 활성 미션 있으면 안전하게 종료 — HIST/IQ/Audio finalize + Central archive push +
    //    mission_history 저장 + IDLE 전이 + JOIN들에게 sync broadcast.
    //    이건 worker stop보다 먼저 (LongWaterfall worker가 살아있어야 HIST rename됨).
    if(v.mission_state == Mission::State::ACTIVE){
        bewe_log_push(0,"[BEWE CLI] auto-ending active mission before shutdown\n");
        v.mission_end();
        // mission_end가 LongWaterfall::request_rotate를 큐잉만 하므로
        // worker가 rotate를 처리할 시간을 짧게 준다 (~600ms).
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
    }

    // 1) Background workers 즉시 중단 (sleep_for 안에 있어도 1초 내 깨어남)
    Mission::stop_utc0_worker();
    LongWaterfall::stop_worker();
    MissionPush::stop();
    // HistCheck 워커도 여기서 join. 빠뜨리면 살아있는 스레드가 정적 소멸 단계에서
    // condvar/mutex 전역과 엉켜 futex 에 고착 → systemd 가 90초 타임아웃 후
    // SIGKILL 할 때까지 프로세스가 안 죽는다 (2026-07-28 DGS-X 사례).
    HistCheck::stop();

    // 2) Central 쪽을 먼저 완전히 끊어서 auto-reconnect 스레드가 더 이상
    //    mux_adapter를 살리지 못하게 함 (g_shutdown 체크로 reconnect도 자가 종료)
    central_cli.stop_mux_adapter();
    central_cli.stop_polling();

    v.is_running = false;
    if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
    v.stop_all_dem();
    if(v.rec_on.load()) v.stop_rec();
    if(v.tm_iq_file_ready){
        v.tm_iq_on.store(false);
        v.tm_iq_close();
    }
    v.mix_stop.store(true); if(v.mix_thr.joinable()) v.mix_thr.join();
    v.net_bcast_stop.store(true);
    v.net_bcast_cv.notify_all();
    if(v.net_bcast_thr.joinable()) v.net_bcast_thr.join();
    if(v.net_srv){ v.net_srv->stop(); delete v.net_srv; v.net_srv=nullptr; }
    if(!v.remote_mode && cap.joinable()) cap.join();
    if(v.dev_blade){
        bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(v.dev_blade); v.dev_blade=nullptr;
    }
    if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
    v.sa_cleanup();
    v.eid_cleanup();

    // 종료 시 record/ 녹음을 보존 (이전엔 private/ 로 이동했으나 제거).

    bewe_log_push(0,"[BEWE CLI] Stopped.\n");

    // /powercycle full — 여기까지 왔으면 host_state 저장·미션 finalize·HIST rename 이
    // 전부 끝났다. 이제서야 재부팅한다. polkit 이 systemctl reboot 을 허용하므로
    // sudo 는 불필요하다 (전 기지 확인함). 부팅 후엔 systemd 유닛이 BEWE 를 다시
    // 띄우고, host_state 가 직전 상태로 복원한다.
    if(g_reboot_on_exit.load()){
        bewe_log_push(0,"[BEWE CLI] Power cycle full: rebooting station now.\n");
        std::fflush(stdout); std::fflush(stderr);
        // polkit 이 막으면 ("Interactive authentication required") 재부팅이 안 된다.
        // 그때 그냥 리턴하면 종료 코드 0 이라 Restart=on-failure 가 안 걸려 기지가
        // 통째로 죽은 채 남는다 (2026-07-30 DGS-2 실측). 재부팅에 실패하면 최소한
        // 프로세스는 살려야 하므로 partial 과 같은 경로로 떨어뜨린다.
        if(std::system("systemctl reboot") == 0)
            return;   // 재부팅이 시작됐다 — systemd 가 재기동할 필요 없다
        bewe_log_push(2,"[BEWE CLI] reboot command failed - falling back to process restart. "
                        "Grant reboot rights: scripts/install-station-unit.sh (polkit rule).\n");
        g_restart_on_exit.store(true);
    }

    // /powercycle partial 은 systemd 가 다시 띄워 줘야 완성된다. 유닛이
    // Restart=on-failure 이므로(정상 종료로 죽었다 살아나는 혼란을 막으려는 설정)
    // 그냥 리턴하면 종료 코드 0 이라 재기동이 안 걸린다 — 기지가 그대로 죽는다.
    // 그래서 "실패" 로 종료해 재기동을 유도한다. systemctl stop / SIGTERM 은
    // 이 플래그가 안 서므로 정상 리턴해 조용히 멈춘다.
    if(g_restart_on_exit.load()){
        bewe_log_push(0,"[BEWE CLI] Power cycle partial: exiting for systemd restart.\n");
        std::fflush(stdout); std::fflush(stderr);
        std::exit(42);
    }
}
