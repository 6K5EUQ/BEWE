#include "doppler_extract.hpp"
#include "../hist_reader.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace Doppler {

uint32_t auto_decimate(double bin_hz, double row_rate_hz, double cf_hz){
    if(cf_hz <= 0 || bin_hz <= 0 || row_rate_hz <= 0) return 1;
    // 0.7 bin 번짐 / (f*(v/c)/66s) → 상수 1.82e6 = 0.7*66/2.535e-5
    double L = 1.82e6 * bin_hz * row_rate_hz / cf_hz;
    if(!(L > 1.0)) return 1;
    if(L > 512.0) L = 512.0;
    return (uint32_t)(L + 0.5);
}

// 저장행 → 선형 순서로 언시프트해 dst 에 담는다.
// 저장 bin 0 = DC 이므로 [N/2,N) 이 앞쪽(저주파), [0,N/2) 가 뒤쪽(고주파)이 된다.
static inline void unshift_row(const uint8_t* src, uint8_t* dst, uint32_t N){
    const uint32_t h = N/2;
    memcpy(dst,     src + h, h);   // 저장 상반 → 선형 하반
    memcpy(dst + h, src,     h);   // 저장 하반 → 선형 상반
}

bool calibrate(HistReader& R, const ExtractParams& P, Calib& out,
               const ProgressFn& prog, const CancelFn& cancel){
    out = Calib{};
    const uint32_t N = R.fft_size();
    const uint32_t nrows = R.num_rows();
    if(N == 0 || nrows == 0) return false;

    // bin별 256레벨 히스토그램. uint16 이라 표본 행수를 6만으로 캡해야 오버플로가 없다.
    // N=32768 에서 16 MB — uint32 로 하면 33 MB 라 굳이.
    static constexpr uint32_t MAX_SAMPLE_ROWS = 60000;
    std::vector<uint16_t> hist((size_t)N * 256, 0);

    // v4 는 블록 단위로 풀리므로 블록 안에서 건너뛰어봐야 이득이 없다 — 블록 경계로
    // 청크를 잡고 청크를 건너뛴다 (scan_file_db_range 가 쓰는 것과 같은 요령).
    const uint32_t chunk = R.block_rows() ? R.block_rows() : 1;
    uint32_t stride = 8;
    {   // 표본 행수가 캡을 넘지 않게 stride 를 키운다
        const uint64_t est = (uint64_t)nrows / stride;
        if(est > MAX_SAMPLE_ROWS) stride = (uint32_t)((uint64_t)nrows / MAX_SAMPLE_ROWS + 1);
    }
    std::vector<uint8_t> lin(N);
    uint64_t sampled = 0;
    const uint32_t cstride = std::max<uint32_t>(1, stride / std::max<uint32_t>(1, chunk));
    for(uint64_t c = 0; c * chunk < nrows; c += cstride){
        if(cancel && cancel()) return false;
        const uint32_t r0 = (uint32_t)(c * chunk);
        const uint32_t r1 = (uint32_t)std::min<uint64_t>((uint64_t)r0 + chunk, nrows);
        const uint32_t rstep = (chunk == 1) ? stride : std::max<uint32_t>(1, stride / 1);
        for(uint32_t r = r0; r < r1; r += (chunk == 1 ? 1 : rstep)){
            const uint8_t* row = R.get_row(r);
            if(!row) continue;
            unshift_row(row, lin.data(), N);
            for(uint32_t i = 0; i < N; i++) {
                uint16_t& h = hist[(size_t)i*256 + lin[i]];
                if(h < 65535) h++;
            }
            if(++sampled >= MAX_SAMPLE_ROWS) break;
        }
        if(sampled >= MAX_SAMPLE_ROWS) break;
        if(prog && (c % 64) == 0)
            prog((float)(c*chunk)/(float)nrows, "calibrate");
    }
    if(sampled == 0) return false;
    out.rows_sampled = sampled;

    const float dmin = R.hdr().db_min, dmax = R.hdr().db_max;
    const float lsb  = (dmax - dmin) / 255.0f;

    // ── bin별 잡음바닥 = 25 퍼센타일 ────────────────────────────────────
    // 15% 는 표시용 autoscale 에는 맞지만 검출용으론 분산이 크다. 실측상 대역의
    // 1.3% 는 상시 캐리어(스퍼)라 중앙값은 신호에 물린다. 25% 가 절충.
    out.nf_db.assign(N, 0.0f);
    out.mask.assign(N, 0);
    std::vector<uint8_t> nf_byte(N, 0);
    for(uint32_t i = 0; i < N; i++){
        const uint16_t* h = &hist[(size_t)i*256];
        uint64_t tot = 0; for(int b = 0; b < 256; b++) tot += h[b];
        if(tot == 0){ out.mask[i] = 1; continue; }
        const uint64_t want = tot / 4;      // 하위 25%
        uint64_t acc = 0; int q = 0;
        for(int b = 0; b < 256; b++){ acc += h[b]; if(acc >= want){ q = b; break; } }
        nf_byte[i] = (uint8_t)q;
        out.nf_db[i] = LongWaterfall::byte_to_db((uint8_t)q, dmin, dmax);
    }

    // ── 임계: bin별 p99.9 초과분의 **bin 간 중앙값** + 1 dB ──────────────
    // 시그마 배수를 추측하지 않는 이유 — max-hold 로 만들어진 꼬리라 가우시안이
    // 아니고 SDR/설정마다 다르다. 데이터에서 직접 뽑는다.
    //
    // 전 bin·전 행을 한 덩어리로 모아 분위수를 내면 안 된다 (실측 실패): 실제 신호가
    // 실린 소수 bin 이 0.01% 꼬리를 통째로 차지해, 24개 파일 중 10개가 상한 12 dB 에
    // 박혔다. bin 마다 따로 분위수를 내고 그 값들의 중앙값을 취하면 신호 실린 bin 이
    // 소수인 한 잡음 bin 이 답을 정한다.
    {
        std::vector<float> per_bin;
        per_bin.reserve(N);
        std::vector<float> med_pool;
        for(uint32_t i = 0; i < N; i++){
            if(out.mask[i]) continue;
            const uint16_t* h = &hist[(size_t)i*256];
            const int nb = nf_byte[i];
            uint64_t tot = 0;
            for(int b = 0; b < 256; b++) tot += h[b];
            if(tot < 64) continue;                 // 표본 부족한 bin 은 제외
            const uint64_t w999 = (uint64_t)(tot * 0.999);
            const uint64_t w50  = tot / 2;
            uint64_t acc = 0; int q999 = nb, q50 = nb;
            bool got50 = false;
            for(int b = 0; b < 256; b++){
                acc += h[b];
                if(!got50 && acc >= w50){ q50 = b; got50 = true; }
                if(acc >= w999){ q999 = b; break; }
            }
            per_bin.push_back((float)std::max(0, q999 - nb) * lsb);
            med_pool.push_back((float)std::max(0, q50  - nb) * lsb);
        }
        float thr = 3.0f;
        if(!per_bin.empty()){
            std::nth_element(per_bin.begin(), per_bin.begin()+per_bin.size()/2, per_bin.end());
            thr = per_bin[per_bin.size()/2] + 1.0f;
        }
        if(thr < 2.5f)  thr = 2.5f;
        if(thr > 12.0f) thr = 12.0f;
        // 완화 후에도 바닥 위 1.5 dB 는 남긴다. 느슨한 민감도(-2 dB)에서 임계가
        // 잡음바닥에 닿으면 모든 bin 이 피크가 되어 트랙 추출이 폭주한다.
        out.thr_db = std::max(1.5f, thr + P.thr_relax_db);
        if(!med_pool.empty()){
            std::nth_element(med_pool.begin(), med_pool.begin()+med_pool.size()/2, med_pool.end());
            out.sigma_db = med_pool[med_pool.size()/2] * 1.4826f;   // MAD → sigma
        }
    }

    // ── 마스크: DC 험프 ±N bin, 대역 양끝, 상시 캐리어 ───────────────────
    const uint32_t dc = N/2;                       // 선형 순서에서 DC 는 한가운데
    for(uint32_t d = 0; d <= P.dc_mask_bins; d++){
        if(dc >= d)     out.mask[dc - d] = 1;
        if(dc + d < N)  out.mask[dc + d] = 1;
    }
    const uint32_t edge = (uint32_t)(N * P.edge_frac);
    for(uint32_t i = 0; i < edge && i < N; i++){ out.mask[i] = 1; out.mask[N-1-i] = 1; }
    // 표본의 50% 초과가 임계 위에 있으면 상시 켜진 캐리어 — 도플러가 아니다.
    {
        const int thr_byte = (int)std::lround(out.thr_db / lsb);
        for(uint32_t i = 0; i < N; i++){
            if(out.mask[i]) continue;
            const uint16_t* h = &hist[(size_t)i*256];
            const int lim = std::min(255, (int)nf_byte[i] + thr_byte);
            uint64_t above = 0, tot = 0;
            for(int b = 0; b < 256; b++){ tot += h[b]; if(b > lim) above += h[b]; }
            if(tot && (double)above/(double)tot > 0.5) out.mask[i] = 1;
        }
    }
    for(uint32_t i = 0; i < N; i++) if(out.mask[i]) out.n_masked++;

    // ── 메인로브 폭 측정 ─────────────────────────────────────────────────
    // 상위 피크들의 -3dB 폭 중앙값. 이 한 수가 피크 분리폭·sigma_f·폴드 판정을 다 정한다.
    // 기대값은 1.976*osr (Nuttall). 폴드가 있으면 0.75bin max 창만큼 넓어진다.
    {
        // 표본 프레임 몇 개만 다시 훑어 피크를 찾는다 (히스토그램으론 폭을 못 잰다).
        std::vector<float> widths;
        const uint32_t probe_stride = std::max<uint32_t>(1, nrows / 64);
        std::vector<float> e(N);
        for(uint32_t r = 0; r < nrows && widths.size() < 200; r += probe_stride){
            const uint8_t* row = R.get_row(r);
            if(!row) continue;
            unshift_row(row, lin.data(), N);
            for(uint32_t i = 0; i < N; i++)
                e[i] = out.mask[i] ? -999.0f
                                   : LongWaterfall::byte_to_db(lin[i], dmin, dmax) - out.nf_db[i];
            // 고립된 톤만 센다. 광대역 신호 위를 -3dB 탐색이 타고 흘러가면 폭이 60 bin
            // 까지 튀고(실측), 그 한 수가 피크 분리폭·폴드 판정을 전부 망친다.
            // 그래서 ① 탐색 범위를 기대폭의 4배로 제한하고 ② 그 안에서 못 내려오면
            // 버리며 ③ 좌우 비대칭이 3배 넘으면 버린다.
            const float expect_w = 1.976f * (float)R.osr();
            const int   wlim = std::max(4, (int)std::lround(expect_w * 4.0f));
            for(uint32_t i = 2; i + 2 < N; i++){
                if(e[i] < out.thr_db + 3.0f) continue;      // 폭을 재려면 -3dB 여유 필요
                if(!(e[i] >= e[i-1] && e[i] >= e[i+1])) continue;
                const float half = e[i] - 3.0f;
                int l = (int)i, steps = 0;
                while(l > 0 && e[l] > half && steps < wlim){ l--; steps++; }
                if(steps >= wlim || e[l] > half) continue;   // 안 내려옴 = 광대역
                int rr = (int)i; steps = 0;
                while(rr < (int)N-1 && e[rr] > half && steps < wlim){ rr++; steps++; }
                if(steps >= wlim || e[rr] > half) continue;
                const float wl = (float)((int)i - l), wrr = (float)(rr - (int)i);
                const float lo = std::min(wl, wrr), hi = std::max(wl, wrr);
                if(hi > 3.0f * std::max(0.5f, lo)) continue; // 비대칭 = 톤 아님
                const float w = wl + wrr;
                if(w > 0.5f && w < 64.0f) widths.push_back(w);
                if(widths.size() >= 200) break;
            }
        }
        // **중앙값이 아니라 하위 10 퍼센타일**을 쓴다. 메인로브 폭은 계측기 속성
        // (FFT 창 x 제로패딩)이지 신호 속성이 아닌데, 측정은 파일에 실제로 들어 있는
        // 신호에서 할 수밖에 없다. 어떤 신호도 창 메인로브보다 좁을 수 없으므로
        // **가장 좁은 피크가 계측기 응답에 수렴**한다. 중앙값을 쓰면 그 시간대에 마침
        // 광대역 신호가 많았는지에 따라 답이 흔들린다 (실측: 같은 설정 파일이 1.98 vs
        // 8.00 으로 갈렸다).
        const float expect = 1.976f * (float)R.osr();
        if(widths.size() >= 8){
            const size_t k = widths.size()/10;
            std::nth_element(widths.begin(), widths.begin()+k, widths.end());
            out.mainlobe_bins = widths[k];
        } else {
            out.mainlobe_bins = expect;                    // 표본 부족 — 이론값
        }
        // 이론 하한 아래로는 내려갈 수 없다 (내려갔다면 잡음 첨두를 잡은 것).
        if(out.mainlobe_bins < expect) out.mainlobe_bins = expect;

        // ── 폴드 보정은 자동 적용하지 않는다 (의도된 결정) ───────────────────
        // max-hold 폴드가 있으면 원시 피크가 0.44 bin 낮게 편향되고 +0.375 로 보정하면
        // 잔차가 0.07 bin 으로 준다. 문제는 **판정이 신뢰할 만하지 않다**는 것: 폭은
        // 계측기 속성인데 파일에 든 신호에서 추정할 수밖에 없어 광대역 신호가 폴드처럼
        // 보인다 (실측 24개 중 1개가 그렇게 오탐했다).
        //
        // 오탐 비용이 미탐 비용보다 크다. 3750 Hz bin 인 파일에서 잘못 건 0.375 bin
        // 보정은 f_center 를 1400 Hz 틀어 놓는다.
        //
        // 그리고 결정적으로, **매칭 결과는 어느 쪽이든 거의 안 바뀐다**: 매처는 정지
        // 주파수 f0 를 자유 파라미터로 두고 해석적으로 소거하므로(doppler_match),
        // f_center 는 f0_est 초기값으로만 쓰이고 f_obs=f0(1-rdot/c) 의 둘째 항에만
        // 들어간다. 1400 Hz 오차 -> 모델 오차 1400*2.5e-5 = 0.035 Hz. 무시할 수준.
        // 즉 f_center 는 사람이 읽는 값이지 매칭 정확도를 좌우하는 값이 아니다.
        //
        // 그래서 편향을 걸지 않고 **불확실도로 넘긴다**. 의심되면 그 사실만 기록한다.
        out.fold_detected = (out.mainlobe_bins > 1.30f*expect);   // 진단용 플래그
        out.fold_bias     = 0.0f;
    }

    out.valid = true;
    return true;
}


// ── 트랙 추출 ────────────────────────────────────────────────────────────────

// 물리 상한: 최저 LEO 의 TCA 도플러율. |df/dt|max = f*(v/c)/tau_min.
static constexpr double V_OVER_C   = 2.535e-5;   // 7.6 km/s
static constexpr double TAU_MIN_S  = 66.0;       // 500km / 7.6km/s

namespace {

struct LiveTrack {
    std::vector<TrackPoint> pts;
    double last_t = 0, last_f = 0;
    double slope  = 0;          // Hz/s, 최근 8점 가중적합
    int    last_frame = -1;
    int    seen = 0;            // 매칭된 프레임 수
    int    span = 0;            // 처음~마지막 프레임 수
    bool   alive = true;
};

// 최근 8점 가중 선형적합 → Hz/s. 점이 부족하면 0.
double fit_slope(const std::vector<TrackPoint>& p){
    const size_t n = p.size();
    if(n < 3) return 0.0;
    const size_t k = std::min<size_t>(8, n);
    const double t0 = p[n-k].t_utc;
    double sw=0, sx=0, sy=0, sxx=0, sxy=0;
    for(size_t i = n-k; i < n; i++){
        const double w = 1.0 / std::max(1.0, (double)p[i].sigma_hz * p[i].sigma_hz);
        const double x = p[i].t_utc - t0, y = p[i].f_hz;
        sw += w; sx += w*x; sy += w*y; sxx += w*x*x; sxy += w*x*y;
    }
    const double den = sw*sxx - sx*sx;
    if(std::fabs(den) < 1e-12) return 0.0;
    return (sw*sxy - sx*sy) / den;
}

struct Peak { double lin; float snr; float w_bins; uint8_t flags; };

// 점별 주파수 불확실도. 좁은 톤이면 SNR 이 지배하고(계측기 폭 / sqrt(2 SNR)),
// 넓은 신호면 신호 폭이 지배한다. 둘을 제곱합해 자연스럽게 넘어가게 한다.
inline double peak_sigma_hz(float snr_db, float w_bins, float mainlobe_bins, double bin_hz){
    const double snr_lin = std::pow(10.0, std::max(0.1f, snr_db)/10.0);
    const double s_snr = ((double)mainlobe_bins*0.5)*bin_hz / std::sqrt(2.0*snr_lin);
    // 폭이 계측기 한계보다 넓은 만큼만 추가 불확실도로 센다.
    const double excess = std::max(0.0, (double)w_bins - (double)mainlobe_bins);
    const double s_w = 0.25 * excess * bin_hz;
    double s = std::sqrt(s_snr*s_snr + s_w*s_w);
    if(s < 0.05*bin_hz) s = 0.05*bin_hz;
    return s;
}

} // anon

// 프레임 하나에서 피크를 뽑는다. 반환 false = 간섭 프레임 (전부 버림).
static bool pick_peaks(const std::vector<float>& e, const Calib& C, const ExtractParams& P,
                       uint32_t N, float w_sep, const std::vector<uint8_t>* sat,
                       std::vector<Peak>& out){
    out.clear();
    uint32_t over = 0, usable = 0;
    for(uint32_t i = 0; i < N; i++){
        if(C.mask[i]) continue;
        usable++;
        if(e[i] >= C.thr_db) over++;
    }
    // 대역의 5% 넘게 임계 위 = 레이더 스윕/광대역 버스트. 이 프레임은 통째로 미스 처리.
    if(usable == 0 || (double)over / (double)usable > 0.05) return false;

    // 후보 로컬 최대 수집
    std::vector<Peak> cand;
    for(uint32_t i = 1; i + 1 < N; i++){
        if(C.mask[i] || C.mask[i-1] || C.mask[i+1]) continue;
        if(e[i] < C.thr_db) continue;
        if(!(e[i] >= e[i-1] && e[i] >= e[i+1])) continue;
        double sub = 0.0;
        float  wbin = 0.0f;
        uint8_t fl = TP_INTERP;
        const bool saturated = sat && ((*sat)[i] || (*sat)[i-1] || (*sat)[i+1]);
        if(saturated){
            // 포화면 포물선이 무의미하다 (평평한 꼭대기). 피크 -3dB 이내 연속 구간의
            // 파워 중심으로 대체.
            const float lim = e[i] - 3.0f;
            int l = (int)i; while(l > 0 && e[l] >= lim) l--;
            int r = (int)i; while(r < (int)N-1 && e[r] >= lim) r++;
            double num = 0, den = 0;
            for(int b = l; b <= r; b++){
                const double w = std::pow(10.0, e[b]/10.0);
                num += w * (double)b; den += w;
            }
            sub = (den > 0) ? (num/den - (double)i) : 0.0;
            wbin = (float)(r - l);
            fl = TP_SATURATED;
        } else {
            // dB 도메인 포물선. 폴드 보정(C.fold_bias)은 기본 0 — doppler_extract.cpp
            // 의 판정 주석 참조 (매칭에 영향이 없어 편향 대신 불확실도로 넘긴다).
            const double A = e[i-1], B = e[i], Cc = e[i+1];
            const double den = (A - 2.0*B + Cc);
            if(std::fabs(den) > 1e-9) sub = 0.5*(A - Cc)/den;
            if(sub >  1.0) sub =  1.0;
            if(sub < -1.0) sub = -1.0;
            sub += (double)C.fold_bias;
            // 이 피크의 -3dB 폭. **계측기 메인로브가 아니라 신호 자체의 폭**이다.
            // 광대역 신호(실측 32.8 kHz = 130 bin)에서 중심주파수는 원리적으로 그
            // 폭보다 정밀하게 정의되지 않으므로, 점별 불확실도를 여기서 끌어와야
            // rms 게이트가 신호 성질에 맞게 자동으로 늘어난다.
            const float half = e[i] - 3.0f;
            int lw = (int)i, rw = (int)i, steps = 0;
            while(lw > 0        && e[lw] > half && steps < 512){ lw--; steps++; }
            steps = 0;
            while(rw < (int)N-1 && e[rw] > half && steps < 512){ rw++; steps++; }
            wbin = (float)(rw - lw);
        }
        cand.push_back({ (double)i + sub, e[i], wbin, fl });
    }
    if(cand.empty()) return true;

    // 강한 것부터 탐욕 분리 — 메인로브 폭에서 유도한 w_sep 안엔 하나만 남긴다.
    std::sort(cand.begin(), cand.end(), [](const Peak&a, const Peak&b){ return a.snr > b.snr; });
    for(const Peak& p : cand){
        bool ok = true;
        for(const Peak& q : out) if(std::fabs(p.lin - q.lin) < w_sep){ ok = false; break; }
        if(ok) out.push_back(p);
        if((int)out.size() >= P.max_peaks_frame) break;
    }
    return true;
}

// 공통 코어: [row_lo,row_hi) x [lin_lo,lin_hi) 범위에서 트랙을 뽑는다.
static bool extract_range(HistReader& R, const ExtractParams& P, const Calib& C,
                          uint32_t row_lo, uint32_t row_hi,
                          uint32_t lin_lo, uint32_t lin_hi,
                          bool single_track,
                          std::vector<Candidate>& out, ExtractStats& st,
                          const ProgressFn& prog, const CancelFn& cancel){
    const uint32_t N = R.fft_size();
    if(N == 0 || row_hi <= row_lo) return false;
    const double bin_hz  = R.bin_hz();
    const double cf_hz   = (double)R.hdr().center_freq_hz;
    const double rr_hz   = (R.hdr().row_rate_hz > 0.0f) ? (double)R.hdr().row_rate_hz : 1.0;

    const uint32_t L = P.force_L ? P.force_L
                                 : auto_decimate(bin_hz, rr_hz, cf_hz);
    st.L = L;
    st.frame_dt_s = (double)L / rr_hz;
    const double dt = st.frame_dt_s;
    const float  w_sep = std::max(1.0f, std::ceil(1.5f * C.mainlobe_bins));
    const double slope_lim = cf_hz * V_OVER_C / TAU_MIN_S;      // Hz/s 물리 상한
    // 갭 한도. 버스트에서만 프레임과 연동한다 — dt 가 파일마다 0.96~7.34s (7.6배) 라
    // 고정 초로 두면 허용 miss 가 8~62 프레임으로 들쭉날쭉하다.
    const double gap_lim = P.burst
        ? std::max(P.max_gap_s, P.min_gap_frames * dt)
        : P.max_gap_s;

    std::vector<double> acc(N);
    std::vector<float>  e(N);
    std::vector<uint8_t> lin(N);
    std::vector<uint8_t> sat(N);
    std::vector<Peak>   peaks;
    std::vector<LiveTrack> live;
    std::vector<LiveTrack> done;

    // dB→선형 파워 LUT. 행은 이미 max-hold 라 2차 max-hold 를 하면 바닥만 올라간다 —
    // 선형 파워 평균이라야 sqrt(L) 만큼 잡음이 준다.
    double lut[256];
    for(int b = 0; b < 256; b++)
        lut[b] = std::pow(10.0, R.db((uint8_t)b)/10.0);

    const uint32_t nframes = (row_hi - row_lo) / std::max<uint32_t>(1, L);
    for(uint32_t fi = 0; fi < nframes; fi++){
        if(cancel && cancel()) return false;
        const uint32_t r0 = row_lo + fi*L;
        std::fill(acc.begin(), acc.end(), 0.0);
        std::fill(sat.begin(), sat.end(), 0);
        uint32_t got = 0;
        for(uint32_t k = 0; k < L; k++){
            const uint8_t* row = R.get_row(r0 + k);
            if(!row) continue;
            unshift_row(row, lin.data(), N);
            for(uint32_t i = 0; i < N; i++){
                acc[i] += lut[lin[i]];
                if(lin[i] == 255) sat[i] = 1;
            }
            got++;
        }
        if(got == 0) continue;
        const double inv = 1.0/(double)got;
        for(uint32_t i = 0; i < N; i++)
            e[i] = (float)(10.0*std::log10(std::max(1e-30, acc[i]*inv))) - C.nf_db[i];
        // 관심 범위 밖은 마스크와 같은 효과로 눌러 둔다 (정밀분석용).
        if(lin_lo > 0 || lin_hi < N){
            for(uint32_t i = 0; i < N; i++)
                if(i < lin_lo || i >= lin_hi) e[i] = -999.0f;
        }
        st.frames++;

        if(!pick_peaks(e, C, P, N, w_sep, &sat, peaks)){
            st.interference_frames++;
            peaks.clear();                    // 전 트랙에 대해 미스로 처리
        }
        st.peaks += (uint32_t)peaks.size();

        const double t_now = R.utc_of_row((double)r0 + 0.5*L);

        // ── 연관: 예측 f 기준 게이트 안에서 가장 가까운 것부터 1:1 배정 ──────
        std::vector<char> used(peaks.size(), 0);
        for(LiveTrack& T : live){
            if(!T.alive) continue;
            const double dtl = t_now - T.last_t;
            const double f_pred = T.last_f + T.slope*dtl;
            // sigma_pred: 최근 점 시그마 + 기울기 불확실
            const double sig = T.pts.empty() ? bin_hz : std::max((double)T.pts.back().sigma_hz, 0.1*bin_hz);
            double gate = std::max({ 2.0*bin_hz, 3.0*sig, 0.5*std::fabs(T.slope)*dtl });
            // 버스트: 갭에 비례하는 불확실도를 더한다. 위 항들 중 dtl 에 반응하는 건
            // 0.5*|slope|*dtl 뿐인데 slope 는 점 3개 미만이면 0 이라(fit_slope) 침묵 뒤
            // 재등장에 게이트가 전혀 안 열렸다. slope 를 모를 때도 물리 상한만큼은 연다.
            if(P.burst)
                gate = std::max(gate, (double)P.slope_unc_frac * slope_lim * dtl);
            int best = -1; double bestd = 1e300;
            for(size_t pi = 0; pi < peaks.size(); pi++){
                if(used[pi]) continue;
                const double f = R.freq_hz_of_linear(peaks[pi].lin);
                const double d = std::fabs(f - f_pred);
                if(d < gate && d < bestd){ bestd = d; best = (int)pi; }
            }
            T.span++;
            if(best < 0){
                if(dtl > gap_lim) T.alive = false;        // 갭 한도 초과 → 종료
                continue;
            }
            used[best] = 1;
            const Peak& pk = peaks[best];
            TrackPoint tp;
            tp.t_utc = t_now;
            tp.row   = r0;
            tp.f_hz  = R.freq_hz_of_linear(pk.lin);
            tp.snr_db= pk.snr;
            // sigma_f ~ (mainlobe/2)*bin / sqrt(2*SNR_linear), 0.05 bin 하한.
            tp.sigma_hz = (float)peak_sigma_hz(pk.snr, pk.w_bins, C.mainlobe_bins, bin_hz);
            tp.flags = pk.flags;
            T.pts.push_back(tp);
            T.last_t = t_now; T.last_f = tp.f_hz;
            T.slope = fit_slope(T.pts);
            if(T.slope >  slope_lim) T.slope =  slope_lim;
            if(T.slope < -slope_lim) T.slope = -slope_lim;
            T.seen++;
            T.last_frame = (int)fi;
        }

        // 남은 피크 → 새 트랙 씨앗
        for(size_t pi = 0; pi < peaks.size(); pi++){
            if(used[pi]) continue;
            // 단일트랙 모드는 "동시에 하나" 지 "평생 하나" 가 아니다. 살아있는 트랙이
            // 없으면 다시 씨를 뿌려야 한다 — 안 그러면 첫 트랙이 페이드로 한 번
            // 죽는 순간 그 뒤 구간을 통째로 못 본다 (실측: refine 이 통째로 실패했다).
            if(single_track && !live.empty()) break;
            if((int)live.size() >= P.max_tracks) break;
            LiveTrack T;
            TrackPoint tp;
            tp.t_utc = t_now; tp.row = r0;
            tp.f_hz = R.freq_hz_of_linear(peaks[pi].lin);
            tp.snr_db = peaks[pi].snr;
            tp.sigma_hz = (float)peak_sigma_hz(peaks[pi].snr, peaks[pi].w_bins,
                                               C.mainlobe_bins, bin_hz);
            tp.flags = peaks[pi].flags;
            T.pts.push_back(tp);
            T.last_t = t_now; T.last_f = tp.f_hz; T.seen = 1; T.span = 1;
            T.last_frame = (int)fi;
            live.push_back(std::move(T));
            st.tracks_seeded++;
        }

        // 죽은 트랙 회수
        for(size_t i = 0; i < live.size();){
            if(!live[i].alive){ done.push_back(std::move(live[i])); live.erase(live.begin()+i); }
            else i++;
        }
        if(prog && (fi % 32) == 0) prog((float)fi/(float)std::max(1u,nframes), "extract");
    }
    for(LiveTrack& T : live) done.push_back(std::move(T));

    // ── 출력 조건 ────────────────────────────────────────────────────────
    const double min_dur = single_track ? 30.0 : P.min_dur_s;
    const float  min_occ = single_track ? 0.15f : P.min_occupancy;
    // 버스트는 duty 대신 절대 점 개수로 거른다. 12점이 문턱인 이유는 mono_frac 이
    // n>=12 에서 Spearman 순위상관을 쓰기 때문이다 — 그 경로는 점 사이 간격에 완전히
    // 불변이라 버스트에 유리하고, n<12 인접차분 경로는 tot=n-1 이 작아 실패 한 번에
    // 0.90 컷을 놓친다.
    const size_t min_pts = P.burst ? P.min_points : 4;
    for(LiveTrack& T : done){
        if(T.pts.size() < min_pts) continue;
        const double dur = T.pts.back().t_utc - T.pts.front().t_utc;
        const float  occ = (T.span > 0) ? (float)T.seen/(float)T.span : 0.0f;
        if(dur < min_dur || occ < min_occ) continue;
        Candidate c;
        c.id = (uint32_t)out.size();
        c.is_burst  = P.burst;
        c.file_path = R.path();
        c.station   = std::string(R.hdr().station_name,
                                  strnlen(R.hdr().station_name, sizeof(R.hdr().station_name)));
        c.station_lat_deg      = R.station_lat();
        c.station_lon_east_deg = R.station_lon_east();
        c.center_freq_hz = R.hdr().center_freq_hz;
        c.sample_rate_hz = R.hdr().sample_rate_hz;
        c.fft_size       = R.fft_size();
        c.fft_input_size = R.fft_input_size();
        c.bin_hz         = bin_hz;
        c.row_rate_hz    = rr_hz;
        c.t_start_utc    = T.pts.front().t_utc;
        c.t_end_utc      = T.pts.back().t_utc;
        c.occupancy      = occ;
        double fmin = 1e300, fmax = -1e300, smax = -1e300;
        std::vector<float> snrs; snrs.reserve(T.pts.size());
        for(const TrackPoint& p : T.pts){
            fmin = std::min(fmin, p.f_hz); fmax = std::max(fmax, p.f_hz);
            smax = std::max(smax, (double)p.snr_db);
            snrs.push_back(p.snr_db);
        }
        std::nth_element(snrs.begin(), snrs.begin()+snrs.size()/2, snrs.end());
        c.snr_med_db = snrs[snrs.size()/2];
        c.snr_max_db = (float)smax;
        c.f_min_hz = fmin; c.f_max_hz = fmax;
        c.pts = std::move(T.pts);
        out.push_back(std::move(c));
        st.tracks_kept++;
    }
    return true;
}

bool extract_tracks(HistReader& R, const ExtractParams& P, const Calib& C,
                    std::vector<Candidate>& out, ExtractStats& st,
                    const ProgressFn& prog, const CancelFn& cancel){
    if(!C.valid) return false;
    out.clear(); st = ExtractStats{};
    return extract_range(R, P, C, 0, R.num_rows(), 0, R.fft_size(), false, out, st, prog, cancel);
}

bool refine_in_box(HistReader& R, uint32_t row_lo, uint32_t row_hi,
                   uint32_t lin_lo, uint32_t lin_hi,
                   const ExtractParams& P, const Calib& C, Candidate& out,
                   const ProgressFn& prog, const CancelFn& cancel){
    if(!C.valid) return false;
    // 박스 안에서는 다중비교 모집단이 ~1e4 배 작아지므로 임계를 낮추고, 페이드를 더
    // 오래 견디며, 비콘만 뜨는 방사체도 살린다. 트랙은 하나만.
    ExtractParams Q = P;
    // **L 은 1 로 두면 안 된다.** L 은 "프레임 안 도플러 번짐 < 0.7 bin" 의 상한이지
    // 목표가 아니다. 1 로 강제하면 sqrt(L) 만큼의 SNR 을 그냥 버린다 — 실측에서
    // L=1 은 프레임당 피크가 64개(상한 포화)로 터지고 트랙 occupancy 가 0.15 밑으로
    // 떨어져 **정상 통과 신호에서 refine 이 통째로 실패했다** (DGS-2 G29, L=18 전수
    // 스캔은 같은 신호를 mono 1.00 / rms 0.28 bin 으로 잡는데).
    Q.force_L      = 0;                 // 전수 스캔과 같은 자동 결정
    Q.max_gap_s    = 60.0;
    Q.min_occupancy= 0.15f;
    Q.max_tracks   = 1;
    Calib D = C;
    D.thr_db = std::max(2.0f, C.thr_db - 2.0f);
    std::vector<Candidate> v;
    ExtractStats st;
    if(!extract_range(R, Q, D, row_lo, row_hi, lin_lo, lin_hi, true, v, st, prog, cancel))
        return false;
    if(getenv("BEWE_DOP_DEBUG"))
        fprintf(stderr, "[refine] L=%u frames=%u peaks=%u seeded=%u kept=%u interf=%u\n",
                st.L, st.frames, st.peaks, st.tracks_seeded, st.tracks_kept,
                st.interference_frames);
    if(v.empty()) return false;
    // 가장 점이 많은 것
    size_t best = 0;
    for(size_t i = 1; i < v.size(); i++) if(v[i].pts.size() > v[best].pts.size()) best = i;
    out = std::move(v[best]);
    return true;
}

} // namespace Doppler
