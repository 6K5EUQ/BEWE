#include "doppler_fit.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace Doppler {

static constexpr double V_LEO_KM_S = 7.6;
static constexpr double C_KM_S     = 299792.458;
static constexpr double V_OVER_C   = V_LEO_KM_S / C_KM_S;   // 2.535e-5

namespace {

struct Pt { double t, y, w; };   // t=상대초, y=상대Hz, w=1/sigma^2

// (t_tca, tau) 고정 시 (A,B) 폐형해 + 가중 RMS 잔차.
// f = A - B*g,  g = x/sqrt(1+x^2)  →  2x2 정규방정식.
double solve_AB(const std::vector<Pt>& p, double t_tca, double tau,
                double& A, double& B){
    double sw=0, sg=0, sgg=0, sy=0, sgy=0;
    for(const Pt& q : p){
        const double x = (q.t - t_tca)/tau;
        const double g = x/std::sqrt(1.0 + x*x);
        sw += q.w; sg += q.w*g; sgg += q.w*g*g; sy += q.w*q.y; sgy += q.w*g*q.y;
    }
    const double den = sw*sgg - sg*sg;
    if(std::fabs(den) < 1e-12){ A = 0; B = 0; return 1e300; }
    // [sw  -sg ][A]   [sy ]
    // [-sg  sgg][B] = [-sgy]
    A = ( sgg*sy - sg*sgy) / den;
    B = ( sg*sy  - sw*sgy) / den;
    double ss = 0, sww = 0;
    for(const Pt& q : p){
        const double x = (q.t - t_tca)/tau;
        const double g = x/std::sqrt(1.0 + x*x);
        const double r = q.y - (A - B*g);
        ss += q.w*r*r; sww += q.w;
    }
    return (sww > 0) ? std::sqrt(ss/sww) : 1e300;
}

// 가중 다항 적합(차수 1 또는 2)의 RMS 잔차 — S곡선과 비교할 기준선.
double poly_rms(const std::vector<Pt>& p, int deg){
    const int n = deg + 1;
    double M[3][4] = {{0}};
    for(const Pt& q : p){
        double pw[3] = {1.0, q.t, q.t*q.t};
        for(int i = 0; i < n; i++){
            for(int j = 0; j < n; j++) M[i][j] += q.w*pw[i]*pw[j];
            M[i][n] += q.w*pw[i]*q.y;
        }
    }
    for(int i = 0; i < n; i++){                     // 가우스 소거
        int piv = i;
        for(int r = i+1; r < n; r++) if(std::fabs(M[r][i]) > std::fabs(M[piv][i])) piv = r;
        if(std::fabs(M[piv][i]) < 1e-12) return 1e300;
        if(piv != i) for(int c = 0; c <= n; c++) std::swap(M[i][c], M[piv][c]);
        for(int r = 0; r < n; r++){
            if(r == i) continue;
            const double f = M[r][i]/M[i][i];
            for(int c = i; c <= n; c++) M[r][c] -= f*M[i][c];
        }
    }
    double a[3] = {0,0,0};
    for(int i = 0; i < n; i++) a[i] = M[i][n]/M[i][i];
    double ss = 0, sw = 0;
    for(const Pt& q : p){
        const double v = a[0] + a[1]*q.t + a[2]*q.t*q.t;
        const double r = q.y - v;
        ss += q.w*r*r; sw += q.w;
    }
    return (sw > 0) ? std::sqrt(ss/sw) : 1e300;
}

} // anon

bool fit_scurve(const std::vector<TrackPoint>& pts, double cf_hz, double bin_hz,
                double row_rate_hz, SCurveFit& out){
    out = SCurveFit{};
    if(pts.size() < 6) return false;

    // 상대 좌표로 옮긴다 — 절대 UTC 초(1.8e9)와 절대 Hz(4e8)를 그대로 쓰면
    // 정규방정식이 수치적으로 무너진다.
    const double t0 = pts.front().t_utc;
    const double f0ref = cf_hz;
    std::vector<Pt> p; p.reserve(pts.size());
    for(const TrackPoint& q : pts){
        const double s = std::max(1.0, (double)q.sigma_hz);
        p.push_back({ q.t_utc - t0, q.f_hz - f0ref, 1.0/(s*s) });
    }
    const double dur = p.back().t - p.front().t;
    if(dur < 20.0) return false;

    // ── 바깥 2D 격자: tau 로그 24점 [30,600]s, t_tca 는 트랙 스팬 ±50% 를 5s 로 ──
    double best_rms = 1e300, best_tca = 0, best_tau = 0, best_A = 0, best_B = 0;
    const double tca_lo = p.front().t - 0.5*dur, tca_hi = p.back().t + 0.5*dur;
    for(int i = 0; i < 24; i++){
        const double tau = 30.0*std::pow(600.0/30.0, i/23.0);
        for(double tc = tca_lo; tc <= tca_hi; tc += 5.0){
            double A, B;
            const double r = solve_AB(p, tc, tau, A, B);
            if(r < best_rms){ best_rms = r; best_tca = tc; best_tau = tau; best_A = A; best_B = B; }
        }
    }
    if(best_rms >= 1e299) return false;

    // ── 황금분할 정밀화 (t_tca, tau 각각 3회) ────────────────────────────
    auto eval = [&](double tc, double tau){ double A,B; return solve_AB(p, tc, tau, A, B); };
    for(int it = 0; it < 3; it++){
        {   double lo = best_tca - 5.0, hi = best_tca + 5.0;
            const double gr = 0.6180339887;
            double c = hi - gr*(hi-lo), d = lo + gr*(hi-lo);
            for(int k = 0; k < 24; k++){
                if(eval(c, best_tau) < eval(d, best_tau)) hi = d; else lo = c;
                c = hi - gr*(hi-lo); d = lo + gr*(hi-lo);
            }
            best_tca = 0.5*(lo+hi);
        }
        {   double lo = best_tau/1.6, hi = best_tau*1.6;
            if(lo < 10.0) lo = 10.0;
            if(hi > 1200.0) hi = 1200.0;
            const double gr = 0.6180339887;
            double c = hi - gr*(hi-lo), d = lo + gr*(hi-lo);
            for(int k = 0; k < 24; k++){
                if(eval(best_tca, c) < eval(best_tca, d)) hi = d; else lo = c;
                c = hi - gr*(hi-lo); d = lo + gr*(hi-lo);
            }
            best_tau = 0.5*(lo+hi);
        }
    }
    best_rms = solve_AB(p, best_tca, best_tau, best_A, best_B);

    // B<0 이면 상승 곡선 — 부호를 뒤집어 tau 대칭성으로 표준형(감소)으로 맞추지 않고
    // 그대로 둔다. 단조 게이트가 거른다.
    out.t_tca_utc   = t0 + best_tca;
    out.f_center_hz = f0ref + best_A;
    out.half_swing_hz = best_B;
    out.tau_s       = best_tau;
    out.max_slope_hz_s = (best_tau > 0) ? std::fabs(best_B)/best_tau : 0.0;
    out.slant_km    = best_tau * V_LEO_KM_S;
    out.rms_resid_hz   = best_rms;
    out.rms_resid_bins = (bin_hz > 0) ? best_rms/bin_hz : 0.0;

    // f_center 불확실도: 적합 통계 + 폴드 미보정 잔여편향(0.44 bin, 상수).
    // 매칭에는 거의 영향이 없다 (f0 는 매처가 해석적으로 소거한다) — 사람이 읽는 값의
    // 정직한 오차막대다.
    {
        double sw = 0; for(const Pt& q : p) sw += q.w;
        const double stat = (sw > 0) ? best_rms/std::sqrt(sw*best_rms*best_rms/std::max(1.0,(double)p.size())) : 0.0;
        out.f_center_sigma_hz = std::sqrt(stat*stat + (0.44*bin_hz)*(0.44*bin_hz));
    }
    // t_tca 정확도는 적합이 아니라 row_rate 오차가 지배한다 (1초 창 평균 + 업링크
    // 드롭 구멍). 600s 트랙에서 1% 면 6초 — 적합 시그마보다 훨씬 크다.
    out.time_sigma_s = std::max(2.0, 0.01*dur);
    (void)row_rate_hz;

    // ── 판별 지표 ────────────────────────────────────────────────────────
    {
        // **인접 표본이 아니라 구간 중앙값으로 잰다.** 물리적 질문은 "매 표본이
        // 감소하나" 가 아니라 "추세가 감소하나" 다. 인접 차분의 잡음은 sqrt(2)*sigma
        // 인데 곡선은 양끝이 거의 평평하므로, 표본별로 재면 진짜 S곡선도 15~20% 가
        // 위로 튄다 (실측: 완벽한 합성 S곡선이 0.84 로 떨어져 거부됐다).
        // 구간 중앙값은 잡음을 sqrt(k) 로 줄이면서 상승 처프·요동을 그대로 잡는다.
        // **Spearman 순위상관**을 쓴다. 구간 중앙값 방식도 잡음에 흔들려, 육안으로
        // 명백한 실제 S곡선이 0.60~0.67 로 떨어져 거부됐다 (실측, DGS-2 G29).
        // 순위상관은 척도무관·이상치강건이고 "추세가 감소하는가" 를 정확히 그대로
        // 묻는다. rho=-1(완전 감소) → 1.0, rho=0(무관) → 0.5, rho=+1(상승) → 0.0
        // 으로 옮겨 기존 임계(0.90 = rho -0.8)를 그대로 쓴다.
        const size_t n = pts.size();
        if(n >= 12){
            std::vector<size_t> ord(n);
            for(size_t i = 0; i < n; i++) ord[i] = i;
            std::sort(ord.begin(), ord.end(),
                      [&](size_t a, size_t b){ return pts[a].f_hz < pts[b].f_hz; });
            std::vector<double> rank(n);
            for(size_t k = 0; k < n; ){        // 동점은 평균순위
                size_t j = k;
                while(j+1 < n && pts[ord[j+1]].f_hz == pts[ord[k]].f_hz) j++;
                const double r = 0.5*((double)k + (double)j) + 1.0;
                for(size_t m = k; m <= j; m++) rank[ord[m]] = r;
                k = j+1;
            }
            // 시간 순위는 1..n (트랙은 이미 시간순)
            const double mt = 0.5*(n+1.0);
            double num = 0, dt2 = 0, df2 = 0;
            for(size_t i = 0; i < n; i++){
                const double a = (double)(i+1) - mt, b = rank[i] - mt;
                num += a*b; dt2 += a*a; df2 += b*b;
            }
            const double rho = (dt2 > 0 && df2 > 0) ? num/std::sqrt(dt2*df2) : 0.0;
            out.mono_frac = (float)((1.0 - rho)*0.5);
        } else {
            int mono = 0, tot = 0;
            for(size_t i = 1; i < n; i++){
                const double d = pts[i].f_hz - pts[i-1].f_hz;
                const double s = std::max(1.0, (double)pts[i].sigma_hz);
                if(d <= 2.0*s) mono++;      // 차분 잡음 sqrt(2)*sigma 를 감안
                tot++;
            }
            out.mono_frac = tot ? (float)mono/(float)tot : 0.0f;
        }
    }
    out.swing_ratio = (std::fabs(out.f_center_hz) > 1.0)
        ? (float)(std::fabs(best_B) / (std::fabs(out.f_center_hz)*V_OVER_C)) : 0.0f;
    {
        const double rl = poly_rms(p, 1), rq = poly_rms(p, 2);
        out.lin_ratio  = (best_rms > 1e-9) ? (float)(rl/best_rms) : 0.0f;
        out.quad_ratio = (best_rms > 1e-9) ? (float)(rq/best_rms) : 0.0f;
    }
    {   // 잔차 반대칭성 — TCA 대칭 위치의 잔차가 서로 부호 반대면 S 가 안 맞은 것
        double num = 0, d1 = 0, d2 = 0;
        int npairs = 0;
        for(size_t i = 0; i < p.size(); i++){
            const double u = p[i].t - best_tca;
            if(u <= 0) continue;
            // 대칭점 -u 에 가장 가까운 표본
            size_t j = 0; double bd = 1e300;
            for(size_t k = 0; k < p.size(); k++){
                const double dd = std::fabs((p[k].t - best_tca) + u);
                if(dd < bd){ bd = dd; j = k; }
            }
            if(bd > 5.0) continue;
            auto resid = [&](size_t idx){
                const double x = (p[idx].t - best_tca)/best_tau;
                const double g = x/std::sqrt(1.0+x*x);
                return p[idx].y - (best_A - best_B*g);
            };
            const double a = resid(i), b = -resid(j);
            num += a*b; d1 += a*a; d2 += b*b;
            npairs++;
        }
        out.antisym = (d1 > 0 && d2 > 0) ? (float)(num/std::sqrt(d1*d2)) : 0.0f;
        // 짝이 몇 개로 잰 값인지 남긴다. 0 이면 antisym 은 "대칭적"이 아니라
        // "측정 못 함"이고, 그 둘은 같은 0.0 으로 보인다 — 채점이 구분해야 한다.
        out.antisym_pairs = npairs;
    }
    {   double ss = 0, sy = 0, sw = 0, mean = 0;
        for(const Pt& q : p){ mean += q.w*q.y; sw += q.w; }
        mean = sw ? mean/sw : 0;
        for(const Pt& q : p){
            const double x = (q.t - best_tca)/best_tau;
            const double g = x/std::sqrt(1.0+x*x);
            const double r = q.y - (best_A - best_B*g);
            ss += q.w*r*r; sy += q.w*(q.y-mean)*(q.y-mean);
        }
        out.r2 = (sy > 0) ? (float)(1.0 - ss/sy) : 0.0f;
    }
    out.tca_inside = (best_tca > p.front().t + 20.0 && best_tca < p.back().t - 20.0);
    out.truncation = 0;
    if(best_tca <= p.front().t + 20.0) out.truncation |= TRUNC_HEAD;
    if(best_tca >= p.back().t  - 20.0) out.truncation |= TRUNC_TAIL;
    out.valid = true;
    return true;
}

// 소프트 시그모이드 — 문턱에서 뚝 끊지 않고 점수로 녹인다.
static float soft(double v, double lo, double hi){
    if(hi <= lo) return 0.0f;
    const double t = (v - lo)/(hi - lo);
    return (float)std::max(0.0, std::min(1.0, t));
}

float score_candidate(Candidate& c, std::string& reason){
    reason.clear();
    const SCurveFit& F = c.fit;
    if(!F.valid){ reason = "fit failed"; return c.score = 0.0f; }
    char buf[128];
    const double dur = c.t_end_utc - c.t_start_utc;
    const double swing_bins = (c.bin_hz > 0) ? (c.f_max_hz - c.f_min_hz)/c.bin_hz : 0.0;

    // 싸고 선택적인 것부터. 2번(swing_ratio)이 주력 — 400 MHz 에서 지상 30m/s=40Hz,
    // 여객기 250m/s=333Hz, LEO=20kHz 라 이 하나가 지상 이동체를 전부 거른다.
    if(F.mono_frac < 0.90f){
        snprintf(buf,sizeof buf,"%s","frequency does not fall steadily"); reason=buf; return c.score=0.0f; }
    if(F.swing_ratio < 0.5f || F.swing_ratio > 1.5f){
        snprintf(buf,sizeof buf,"frequency shift too %s for a satellite", F.swing_ratio < 1.0 ? "small" : "large"); reason=buf; return c.score=0.0f; }
    if(F.tau_s < 30.0 || F.tau_s > 600.0){
        snprintf(buf,sizeof buf,"pass too %s", F.tau_s < 30.0 ? "brief" : "long"); reason=buf; return c.score=0.0f; }
    if(F.lin_ratio < 3.0f){
        snprintf(buf,sizeof buf,"%s","looks like a slow drift, not a pass"); reason=buf; return c.score=0.0f; }
    if(F.quad_ratio < 1.5f){
        snprintf(buf,sizeof buf,"%s","curve shape does not match a pass"); reason=buf; return c.score=0.0f; }
    if(F.antisym > 0.5f){
        snprintf(buf,sizeof buf,"%s","curve is lopsided"); reason=buf; return c.score=0.0f; }
    {
        double med_sig = 0;
        if(!c.pts.empty()){
            std::vector<float> s; s.reserve(c.pts.size());
            for(const auto& p : c.pts) s.push_back(p.sigma_hz);
            std::nth_element(s.begin(), s.begin()+s.size()/2, s.end());
            med_sig = s[s.size()/2];
        }
        const double lim = std::max(1.5*c.bin_hz, 2.0*med_sig);
        if(F.rms_resid_hz > lim){
            snprintf(buf,sizeof buf,"%s","does not follow a satellite curve");
            reason=buf; return c.score=0.0f; }
    }
    if(dur < 60.0 || dur > 1500.0){
        snprintf(buf,sizeof buf,"signal lasted %.0f min - too %s", dur/60.0, dur < 60.0 ? "short" : "long"); reason=buf; return c.score=0.0f; }
    // 8번(스윙 bin 수)은 **거부가 아니라 강등** — 30.72 Msps 같은 저해상도 설정에서
    // 전체 스윙이 12 bin 뿐이라 형상 판별력이 약할 뿐 신호는 진짜일 수 있다.
    // 10번(TCA 내부)도 거부 아님 — HIST 는 매시 회전해 실제 패스가 두 파일로 쪼개진다.

    float s = 1.0f;
    s *= soft(F.mono_frac,   0.88, 0.97);
    s *= soft(F.lin_ratio,   3.0,  10.0);
    s *= soft(F.quad_ratio,  1.5,  4.0);
    // 버스트는 시간축이 비대칭이라 antisym 짝이 안 생길 수 있다. 그때 antisym 은 0 이고
    // soft(1-0) = 만점이 되는데, 재지 못한 것이 최고점을 받으면 안 된다. 짝이 없으면
    // 중립 감점으로 대신한다. (연속 트랙은 짝이 늘 충분해 이 분기를 안 탄다.)
    if(c.is_burst && F.antisym_pairs < 3) s *= 0.7f;
    else                                  s *= soft(1.0-F.antisym, 0.5, 0.9);
    // occupancy 는 duty cycle 이다 — 신호 특성이지 품질이 아니다. 버스트(실측 0.08)를
    // 여기서 재면 soft 램프 하한 0.4 에 걸려 score 가 통째로 0 이 된다. 곡선을 그리는
    // 데 필요한 건 비율이 아니라 점 개수이고, 그건 추출의 min_points 가 보증한다.
    if(!c.is_burst) s *= soft(c.occupancy, 0.4, 0.8);
    s *= soft(swing_bins,    4.0,  20.0);          // 저해상도는 감점만
    if(F.truncation) s *= 0.85f;
    c.score = std::max(0.0f, std::min(1.0f, s));
    return c.score;
}

} // namespace Doppler
