#include "sat_trim.hpp"
#include "sigmf.hpp"
#include <fftw3.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

namespace SatTrim {

static float median_of(std::vector<float> v){
    if(v.empty()) return 0.f;
    size_t m = v.size()/2;
    std::nth_element(v.begin(), v.begin()+m, v.end());
    return v[m];
}

Result trim(const std::string& path, const std::function<double(double)>& dop_fn){
    Result R;
    SigMF::Meta meta;
    if(!SigMF::read_meta(path, meta) || meta.sample_rate == 0){ R.why = "no meta"; return R; }
    const uint32_t sr = meta.sample_rate;

    int fd = open(path.c_str(), O_RDONLY);
    if(fd < 0){ R.why = "open failed"; return R; }
    struct stat st{};
    fstat(fd, &st);
    const size_t n = (size_t)st.st_size / 4;          // ci16 쌍
    if(n < (size_t)sr * 10){ close(fd); R.why = "shorter than 10 s"; return R; }
    void* map = mmap(nullptr, (size_t)st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    if(map == MAP_FAILED){ R.why = "mmap failed"; return R; }
    const int16_t* x = (const int16_t*)map;

    // ── 1) 1초 행 PSD — bin ~250 Hz, 그 1초의 샘플을 전부 평균 ─────────────
    // bin 을 30 Hz 로 잘게 쪼개면 약한 패스에서 신호 덩어리 안에 잡음 골이 생겨
    // 점유 구간이 반토막 난다 (DGS-2 22:40 실측 2.75 kHz vs 실제 5.4 kHz).
    int N = 256;
    while(N < 16384 && (double)sr / N > 250.0) N <<= 1;
    const int AVG = std::max(1, (int)(sr / N));
    const size_t rows = n / sr;
    std::vector<float> win(N);
    for(int i = 0; i < N; i++) win[i] = 0.5f - 0.5f*cosf(2.f*(float)M_PI*i/(N-1));
    fftwf_complex* in  = fftwf_alloc_complex(N);
    fftwf_complex* out = fftwf_alloc_complex(N);
    fftwf_plan plan = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    std::vector<float> P(rows * (size_t)N, 0.f);       // fftshift 순서, 행별 정규화
    for(size_t r = 0; r < rows; r++){
        float* row = &P[r*(size_t)N];
        int got = 0;
        for(int a = 0; a < AVG; a++){
            size_t s0 = r*(size_t)sr + (size_t)a*N;
            if(s0 + N > n || s0 + N > (r+1)*(size_t)sr) break;
            for(int i = 0; i < N; i++){
                in[i][0] = x[2*(s0+i)]   * win[i];
                in[i][1] = x[2*(s0+i)+1] * win[i];
            }
            fftwf_execute(plan);
            for(int i = 0; i < N; i++){
                int k = (i + N/2) % N;
                row[i] += out[k][0]*out[k][0] + out[k][1]*out[k][1];
            }
            got++;
        }
    }
    fftwf_destroy_plan(plan); fftwf_free(in); fftwf_free(out);

    // 1차 파일은 채널 LPF 로 양끝이 깎여 있다. 행 중앙값으로만 나누면 그 모양이
    // 모든 행을 "신호"로 만든다 (DGS-2 실측 681/681). 먼저 bin 별 시간축 10번째
    // 백분위로 나눠 모양을 지운다 — 신호가 패스의 90% 미만에만 있으면 바닥이 남는다.
    {
        std::vector<float> col(rows);
        for(int i = 0; i < N; i++){
            for(size_t r = 0; r < rows; r++) col[r] = P[r*(size_t)N + i];
            size_t q = rows / 10;
            std::nth_element(col.begin(), col.begin()+q, col.end());
            float f = col[q] > 0 ? col[q] : 1e-30f;
            for(size_t r = 0; r < rows; r++) P[r*(size_t)N + i] /= f;
        }
        // 행별 이득 변화는 안쪽 절반 중앙값으로 맞춘다
        std::vector<float> inr(N/2);
        for(size_t r = 0; r < rows; r++){
            float* row = &P[r*(size_t)N];
            for(int i = 0; i < N/2; i++) inr[i] = row[N/4 + i];
            float med = median_of(inr);
            if(med <= 0) med = 1e-30f;
            for(int i = 0; i < N; i++) row[i] /= med;
        }
    }

    // ── 2) 위성 대역 찾기: bin 별 "켜져 있던 행 수"(지속도) ─────────────────
    // 도플러를 따라가는 프레임에서 위성은 같은 bin 에 수십 초 머물고, 지상 반송파는
    // 대역을 가로질러 지나가 bin 마다 몇 행만 머문다. 그래서 행 단위로 먼저 고르면
    // (예전 방식) 약한 패스에서 지나가는 반송파 행이 평균을 지배해 위성이 탈락했다
    // (DGS-1 10/2 22:19: 34행 중 15행만 위성). 지속도로 대역부터 찾는다.
    // 켜짐 = 9bin(~2 kHz) 평활이 +1 dB 초과 (1초 전부 평균이라 잡음편차 ~0.1 dB → ~10σ).
    // 위성은 지정 정지주파수 근처 — 중심 ±N/10 (캡처폭의 ±1/8, 200 kHz 면 ±25 kHz) 만 본다.
    const int pk_lo = N/2 - N/10, pk_hi = N/2 + N/10;
    const float ON = 1.2589f;                         // 10^(1/10)
    std::vector<int> cnt(N, 0);
    for(size_t r = 0; r < rows; r++){
        const float* row = &P[r*(size_t)N];
        for(int i = pk_lo + 4; i < pk_hi - 4; i++){
            float s9 = 0;
            for(int j = -4; j <= 4; j++) s9 += row[i+j];
            if(s9 * (1.0f/9.0f) > ON) cnt[i]++;
        }
    }
    int k0 = pk_lo + 4;
    for(int i = pk_lo + 4; i < pk_hi - 4; i++) if(cnt[i] > cnt[k0]) k0 = i;
    if(cnt[k0] < 10){ munmap(map, (size_t)st.st_size); R.why = "no signal"; return R; }
    int ba = k0, bb = k0;                             // 지속도 절반 이상인 연속 구간
    const int half_cnt = std::max(5, cnt[k0] / 2);
    while(ba > pk_lo + 4 && cnt[ba-1] >= half_cnt) ba--;
    while(bb < pk_hi - 5 && cnt[bb+1] >= half_cnt) bb++;

    // 신호 행 = 그 구간 평균이 +1 dB 를 넘는 행
    std::vector<double> mean(N, 0.0);
    std::vector<size_t> row_idx;
    for(size_t r = 0; r < rows; r++){
        const float* row = &P[r*(size_t)N];
        double m = 0;
        for(int i = ba; i <= bb; i++) m += row[i];
        if(m / (bb - ba + 1) <= ON) continue;
        for(int i = 0; i < N; i++) mean[i] += row[i];
        row_idx.push_back(r);
    }
    R.signal_rows = (int)row_idx.size();
    if(R.signal_rows < 10){ munmap(map, (size_t)st.st_size); R.why = "no signal"; return R; }

    // ── 3) 점유 대역: 신호 행 평균 PSD, 안쪽 절반에서 바닥·편차 ───────────────
    std::vector<float> db(N);
    for(int i = 0; i < N; i++) db[i] = 10.f*log10f((float)(mean[i]/R.signal_rows) + 1e-30f);
    const int lo_i = N/4, hi_i = 3*N/4;
    std::vector<float> inner(db.begin()+lo_i, db.begin()+hi_i);
    const float floor_db = median_of(inner);
    std::vector<float> dev(inner.size());
    for(size_t i = 0; i < inner.size(); i++) dev[i] = fabsf(inner[i] - floor_db);
    const float sigma = 1.4826f * median_of(dev);
    int kp = ba;                                      // 지속 구간 안의 최대에서 키운다
    for(int i = ba; i <= bb; i++) if(db[i] > db[kp]) kp = i;
    // 임계 = 바닥 + max(6σ, 피크 높이의 절반). 고정 1 dB 로 두면 약한 패스(바닥 위 ~1 dB 의
    // 평평한 5 kHz 덩어리)에서 양끝 뿔만 넘어 1.6 kHz 로 잘렸다 (DGS-1 10/2 22:19).
    const float thr = floor_db + std::max(6.0f*sigma, 0.5f*(db[kp] - floor_db));
    if(db[kp] - floor_db <= 6.0f*sigma){ munmap(map, (size_t)st.st_size); R.why = "peak below threshold"; return R; }
    int a = kp, b = kp;
    while(a > lo_i){
        if(db[a-1] > thr) a--;
        else if(a-2 >= lo_i && db[a-2] > thr) a -= 2;
        else break;
    }
    while(b < hi_i-1){
        if(db[b+1] > thr) b++;
        else if(b+2 < hi_i && db[b+2] > thr) b += 2;
        else break;
    }
    const double bin_hz = (double)sr / N;
    R.occ_bw_hz     = (b - a + 1) * bin_hz;
    R.center_off_hz = ((a + b) * 0.5 - N/2) * bin_hz;
    if(b - a + 1 < 3){ munmap(map, (size_t)st.st_size); R.why = "occupied band too narrow"; return R; }

    if(dop_fn){
        // 신호가 보인 동안 도플러가 거의 안 변했으면(패스 끝 저고도) 고정 지상 반송파도
        // 추적 프레임에서 한 자리에 머문다 — 구별할 수 없으니 자르지 않는다.
        double dmin = 1e30, dmax = -1e30;
        for(size_t r : row_idx){
            double d = dop_fn((double)meta.start_unix + r + 0.5);
            dmin = std::min(dmin, d); dmax = std::max(dmax, d);
        }
        if(dmax - dmin < 500.0){
            munmap(map, (size_t)st.st_size);
            char w[96]; snprintf(w, sizeof w, "Doppler barely changed while visible (%.0f Hz)", dmax - dmin);
            R.why = w; return R;
        }
    }

    // ── 4) 재절단: 중심을 0 으로 옮기고 BW x1.25 로 LPF + 정수 decim ─────────
    const double bw_rec    = R.occ_bw_hz * 1.25;
    const double target_sr = std::max(bw_rec * 1.25, 2000.0);
    const int    decim     = std::max(1, (int)floor(sr / target_sr));
    if(decim < 2){ munmap(map, (size_t)st.st_size); R.why = "already narrow"; return R; }
    R.sr_out = sr / decim;
    const int    L  = 8*decim + 1;                    // Blackman 윈도 sinc
    const double fc = (bw_rec * 0.5) / sr;
    std::vector<float> h(L);
    double hs = 0;
    for(int i = 0; i < L; i++){
        int m = i - L/2;
        double sinc = (m == 0) ? 2*fc : sin(2*M_PI*fc*m) / (M_PI*m);
        double w = 0.42 - 0.5*cos(2*M_PI*i/(L-1)) + 0.08*cos(4*M_PI*i/(L-1));
        h[i] = (float)(sinc * w); hs += h[i];
    }
    for(auto& v : h) v = (float)(v / hs);

    std::string tmp_path = path + ".trim";
    FILE* fo = fopen(tmp_path.c_str(), "wb");
    if(!fo){ munmap(map, (size_t)st.st_size); R.why = "cannot write"; return R; }
    // 입력을 한 번씩만 믹싱(복소 회전자, 주기적 정규화)하고 최근 L 개를 링에 둔다.
    // decim 번째마다 그 링과 탭을 내적해 출력 한 개.
    const double w0 = -2.0*M_PI*R.center_off_hz / sr;
    std::complex<double> rot(1, 0), step(cos(w0), sin(w0));
    std::vector<std::complex<float>> ring(L);
    std::vector<int16_t> obuf; obuf.reserve(1<<16);
    int rp = 0, phase = 0;
    for(size_t s = 0; s < n; s++){
        std::complex<float> v((float)x[2*s], (float)x[2*s+1]);
        ring[rp] = v * std::complex<float>((float)rot.real(), (float)rot.imag());
        rot *= step;
        if((s & 0xFFF) == 0) rot /= std::abs(rot);
        rp = (rp + 1 == L) ? 0 : rp + 1;
        if(s + 1 < (size_t)L) continue;
        if(++phase < decim) continue;
        phase = 0;
        float ai = 0, aq = 0;
        int k = rp;                                   // 가장 오래된 샘플부터
        for(int j = 0; j < L; j++){
            ai += h[j] * ring[k].real();
            aq += h[j] * ring[k].imag();
            k = (k + 1 == L) ? 0 : k + 1;
        }
        obuf.push_back((int16_t)std::max(-32767.f, std::min(32767.f, ai)));
        obuf.push_back((int16_t)std::max(-32767.f, std::min(32767.f, aq)));
        if(obuf.size() >= (1u<<16)){ fwrite(obuf.data(), 2, obuf.size(), fo); obuf.clear(); }
    }
    if(!obuf.empty()) fwrite(obuf.data(), 2, obuf.size(), fo);
    fclose(fo);
    munmap(map, (size_t)st.st_size);

    if(rename(tmp_path.c_str(), path.c_str()) != 0){ remove(tmp_path.c_str()); R.why = "rename failed"; return R; }
    meta.sample_rate    = R.sr_out;
    meta.center_freq_hz = (uint64_t)llround((double)meta.center_freq_hz + R.center_off_hz);
    meta.bandwidth_hz   = bw_rec;
    char note[200];
    snprintf(note, sizeof note, "%sDoppler-corrected; occupied %.2f kHz from %d signal s",
             meta.notes.empty() ? "" : "; ", R.occ_bw_hz/1e3, R.signal_rows);
    meta.notes += note;
    SigMF::write_meta(path, meta);
    R.trimmed = true;
    return R;
}

} // namespace SatTrim
