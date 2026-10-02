#include "sat_demod.hpp"
#include "sigmf.hpp"
#include "bewe_paths.hpp"
#include <fftw3.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>

namespace SatDemod {

static constexpr double FS       = 50000.0;   // 재샘플 레이트 (5000 bd 의 10배)
static constexpr int    SPS      = 10;
static constexpr double RATE_LO  = 4700.0, RATE_HI = 5100.0;

double Frame::stable_frac() const {
    if(conf.empty()) return 0;
    size_t n = 0;
    for(uint8_t c : conf) if(c >= 5) n++;
    return (double)n / conf.size();
}

// 복소 신호를 FFT 영역에서 길이 m 으로 늘린다 (대역제한 보간 — Python 검증과 동일)
static std::vector<std::complex<float>> resample(const std::complex<float>* z, size_t n, size_t m){
    std::vector<std::complex<float>> X(n), Y(m, 0.f), out(m);
    fftwf_plan p1 = fftwf_plan_dft_1d((int)n, (fftwf_complex*)const_cast<std::complex<float>*>(z),
                                      (fftwf_complex*)X.data(), FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_execute(p1); fftwf_destroy_plan(p1);
    const size_t h = (n + 1) / 2;                // 양의 주파수 [0,h), 음의 [h,n)
    for(size_t i = 0; i < h; i++)        Y[i] = X[i];
    for(size_t i = h; i < n; i++)        Y[m - (n - i)] = X[i];
    fftwf_plan p2 = fftwf_plan_dft_1d((int)m, (fftwf_complex*)Y.data(),
                                      (fftwf_complex*)out.data(), FFTW_BACKWARD, FFTW_ESTIMATE);
    fftwf_execute(p2); fftwf_destroy_plan(p2);
    const float s = 1.0f / n;
    for(auto& v : out) v *= s;
    return out;
}

// 실수열 a,b(±1) 원형상관 c[k] = sum a[i] b[i-k] → b 를 k 만큼 굴리면 a 와 맞는다
static std::vector<double> circ_corr(const std::vector<double>& a, const std::vector<double>& b){
    const int n = (int)a.size();
    std::vector<std::complex<float>> A(n/2+1), B(n/2+1);
    std::vector<float> ta(a.begin(), a.end()), tb(b.begin(), b.end()), c(n);
    fftwf_plan pa = fftwf_plan_dft_r2c_1d(n, ta.data(), (fftwf_complex*)A.data(), FFTW_ESTIMATE);
    fftwf_plan pb = fftwf_plan_dft_r2c_1d(n, tb.data(), (fftwf_complex*)B.data(), FFTW_ESTIMATE);
    fftwf_execute(pa); fftwf_execute(pb);
    for(int i = 0; i <= n/2; i++) A[i] *= std::conj(B[i]);
    fftwf_plan pc = fftwf_plan_dft_c2r_1d(n, (fftwf_complex*)A.data(), c.data(), FFTW_ESTIMATE);
    fftwf_execute(pc);
    fftwf_destroy_plan(pa); fftwf_destroy_plan(pb); fftwf_destroy_plan(pc);
    return std::vector<double>(c.begin(), c.end());   // 크기 비교만 하므로 1/n 생략
}

static int best_shift(const std::vector<double>& c, int& pol){
    int k = 0;
    for(int i = 1; i < (int)c.size(); i++) if(fabs(c[i]) > fabs(c[k])) k = i;
    pol = c[k] >= 0 ? 1 : -1;
    return k;
}

Result run(const std::string& path){
    Result R;
    SigMF::Meta meta;
    if(!SigMF::read_meta(path, meta) || meta.sample_rate == 0){ R.why = "no meta"; return R; }
    const uint32_t sr = meta.sample_rate;
    FILE* f = fopen(path.c_str(), "rb");
    if(!f){ R.why = "open failed"; return R; }
    std::vector<int16_t> raw;
    { int16_t buf[1<<14]; size_t g; while((g = fread(buf, 2, 1<<14, f)) > 0) raw.insert(raw.end(), buf, buf+g); }
    fclose(f);
    const size_t n = raw.size() / 2;
    if(n < (size_t)sr * 10){ R.why = "too short"; return R; }

    // ── 1) 신호 구간: 1초 전력 > 1.5x 중앙값, 3초 이하 틈은 이어 붙인 최장 구간 ──
    const size_t secs = n / sr;
    std::vector<double> pw(secs, 0.0);
    for(size_t s = 0; s < secs; s++)
        for(size_t i = s*sr; i < (s+1)*sr; i++){ double a = raw[2*i], b = raw[2*i+1]; pw[s] += a*a + b*b; }
    std::vector<double> tmp(pw);
    std::nth_element(tmp.begin(), tmp.begin()+tmp.size()/2, tmp.end());
    const double thr = 1.5 * tmp[tmp.size()/2];
    size_t best_a = 0, best_b = 0, cur_a = 0, last = 0; bool in = false;
    for(size_t s = 0; s < secs; s++){
        if(pw[s] <= thr) continue;
        if(!in || s - last > 4){ cur_a = s; in = true; }
        last = s;
        if(last - cur_a > best_b - best_a || best_b == 0){ best_a = cur_a; best_b = last; }
    }
    if(!in || best_b - best_a + 1 < 10){ R.why = "signal shorter than 10 s"; return R; }
    const size_t s0 = best_a * sr, ns = (best_b - best_a + 1) * sr;

    // ── 2) 50 kHz 로 재샘플 → FM 판별 → 1심볼 이동평균 ──────────────────────
    std::vector<std::complex<float>> z(ns);
    for(size_t i = 0; i < ns; i++) z[i] = { (float)raw[2*(s0+i)], (float)raw[2*(s0+i)+1] };
    raw.clear(); raw.shrink_to_fit();
    const size_t m = (size_t)llround((double)ns * FS / sr);
    std::vector<std::complex<float>> zz = resample(z.data(), ns, m);
    z.clear(); z.shrink_to_fit();
    std::vector<float> fi(m - 1);
    for(size_t i = 1; i < m; i++) fi[i-1] = std::arg(zz[i] * std::conj(zz[i-1]));
    zz.clear(); zz.shrink_to_fit();
    { std::vector<float> t(fi); std::nth_element(t.begin(), t.begin()+t.size()/2, t.end());
      const float med = t[t.size()/2]; for(auto& v : fi) v -= med; }   // 잔여 주파수 오프셋
    std::vector<float> y(fi.size(), 0.f);
    { double acc = 0; for(size_t i = 0; i < fi.size(); i++){
        acc += fi[i]; if(i >= (size_t)SPS) acc -= fi[i-SPS];
        y[i] = (float)(acc / SPS); } }
    fi.clear(); fi.shrink_to_fit();

    // ── 3) 심볼율: (dy)^2 스펙트럼의 4700~5100 Hz 최대선 ──────────────────────
    size_t N = 1; while(N < y.size()) N <<= 1;
    std::vector<float> d(N, 0.f);
    double mean = 0;
    for(size_t i = 1; i < y.size(); i++){ float v = y[i]-y[i-1]; d[i] = v*v; mean += d[i]; }
    mean /= (y.size() - 1);
    for(size_t i = 1; i < y.size(); i++) d[i] -= (float)mean;
    std::vector<std::complex<float>> D(N/2+1);
    fftwf_plan pd = fftwf_plan_dft_r2c_1d((int)N, d.data(), (fftwf_complex*)D.data(), FFTW_ESTIMATE);
    fftwf_execute(pd); fftwf_destroy_plan(pd);
    d.clear(); d.shrink_to_fit();
    size_t k_lo = (size_t)(RATE_LO * N / FS), k_hi = (size_t)(RATE_HI * N / FS), kb = k_lo;
    for(size_t k = k_lo; k <= k_hi; k++) if(std::norm(D[k]) > std::norm(D[kb])) kb = k;
    const double rb = kb * FS / N;
    R.frame.rate_bd = rb;

    // ── 4) 1초 블록 판정: 10개 위상 중 |v| 평균 최대 ──────────────────────────
    const int L = (int)llround(rb);                // 프레임 길이 = 1초 심볼 수
    auto at = [&](double t)->float{                 // y 를 시각 t(초) 에서 선형 보간
        double x = t * FS; size_t i = (size_t)x; if(i + 1 >= y.size()) return 0.f;
        double fr = x - i; return (float)(y[i]*(1-fr) + y[i+1]*fr); };
    const double dur = (double)y.size() / FS;
    std::vector<std::vector<uint8_t>> frames;
    for(double t0 = 0; t0 + 1.0 + 1.0/rb < dur; t0 += (double)L / rb){
        int bp = 0; double bm = -1;
        for(int ph = 0; ph < SPS; ph++){
            double s = 0;
            for(int k = 0; k < L; k++) s += fabs(at(t0 + (k + ph/(double)SPS) / rb));
            if(s > bm){ bm = s; bp = ph; }
        }
        std::vector<uint8_t> b(L);
        for(int k = 0; k < L; k++) b[k] = at(t0 + (k + bp/(double)SPS) / rb) > 0 ? 1 : 0;
        frames.push_back(std::move(b));
    }
    if(frames.size() < 5){ R.why = "fewer than 5 frames"; return R; }

    // ── 5) 대표 프레임: 원형상관으로 정렬해 다수결 ───────────────────────────────
    std::vector<double> acc(L, 0.0), ref(L);
    for(int i = 0; i < L; i++) ref[i] = frames[0][i] ? 1.0 : -1.0;
    for(auto& fr : frames){
        std::vector<double> x(L);
        for(int i = 0; i < L; i++) x[i] = fr[i] ? 1.0 : -1.0;
        int pol; int s = best_shift(circ_corr(ref, x), pol);
        for(int i = 0; i < L; i++) acc[i] += pol * x[((i - s) % L + L) % L];
        for(int i = 0; i < L; i++) ref[i] = acc[i] >= 0 ? 1.0 : -1.0;
    }
    R.frame.n_frames = (int)frames.size();
    R.frame.bits.resize(L); R.frame.conf.resize(L);
    for(int i = 0; i < L; i++){
        R.frame.bits[i] = acc[i] > 0 ? 1 : 0;
        R.frame.conf[i] = (uint8_t)std::min(9.0, floor(fabs(acc[i]) / frames.size() * 10.0));
    }
    R.ok = true;
    return R;
}

bool save(const std::string& path, const Frame& f, const std::string& header){
    FILE* o = fopen(path.c_str(), "w");
    if(!o) return false;
    fprintf(o, "%s\n", header.c_str());
    for(uint8_t b : f.bits) fputc('0' + b, o);
    fputc('\n', o);
    for(uint8_t c : f.conf) fputc('0' + c, o);
    fputc('\n', o);
    fclose(o);
    return true;
}

bool load(const std::string& path, Frame& f, std::string* header){
    FILE* in = fopen(path.c_str(), "r");
    if(!in) return false;
    std::string lines[3];
    for(int i = 0; i < 3; i++){
        int c;
        while((c = fgetc(in)) != EOF && c != '\n') lines[i] += (char)c;
    }
    fclose(in);
    if(lines[1].empty() || lines[1].size() != lines[2].size()) return false;
    if(header) *header = lines[0];
    f.bits.clear(); f.conf.clear();
    for(char c : lines[1]) f.bits.push_back(c == '1');
    for(char c : lines[2]) f.conf.push_back((uint8_t)(c - '0'));
    return true;
}

std::string frame_dir(){ return BEWEPaths::data_dir() + "/sat_frames"; }

std::string latest_name(int norad){
    char b[48]; snprintf(b, sizeof b, "SATFRAME_%d.txt", norad);
    return b;
}

Compare compare(const Frame& a, const Frame& b){
    Compare C;
    if(a.bits.empty() || a.bits.size() != b.bits.size()) return C;
    const int L = (int)a.bits.size();
    std::vector<double> x(L), y(L);
    for(int i = 0; i < L; i++){ x[i] = a.bits[i] ? 1 : -1; y[i] = b.bits[i] ? 1 : -1; }
    C.shift = best_shift(circ_corr(x, y), C.polarity);
    for(int i = 0; i < L; i++){
        int j = ((i - C.shift) % L + L) % L;
        if(a.conf[i] < 5 || b.conf[j] < 5) continue;
        C.n_conf++;
        int bb = C.polarity > 0 ? b.bits[j] : 1 - b.bits[j];
        if(bb != a.bits[i]) C.n_diff++;
    }
    C.ok = C.n_conf > 0;
    C.agree = C.ok ? 1.0 - (double)C.n_diff / C.n_conf : 0;
    return C;
}

} // namespace SatDemod
