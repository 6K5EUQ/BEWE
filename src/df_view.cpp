// ── DF(방탐) 전체영역 오버레이 ────────────────────────────────────────────
// 하단 상태바의 DF 를 누르면 열린다. 다른 오버레이와 상호배타이고, 열려 있는
// 동안 메인페이지 키보드/마우스는 막힌다 (상태바 32 px 는 남겨 다시 눌러 닫는다).
//
// 화면 구성이 v14.2 에서 뒤집혔다. 예전에는 설정 위젯이 화면의 90% 를 차지하고
// 결과는 우하단 200 px 원 하나였다 — 운용자가 계속 보는 것과 한 번 만지고 마는
// 것의 비중이 정반대였다. 지금은:
//
//   [좌] 방위 이력표      — 측정이 안정적인지 표류하는지, 왜 거절됐는지
//   [우] 지도 + LOB 광선  — 방위가 실제로 어디를 가리키는지, 교차점은 어디인지
//   [설정] 우측 접이식    — 기본 닫힘. 열면 예전 위젯이 그대로 들어 있다
//
// 설정은 전부 HOST 소유다. JOIN 은 HOST 가 방송한 정본을 보여주고, 바꾸면 HOST 로
// 요청만 보낸다 — 측정을 실제로 하는 쪽과 화면이 어긋나지 않게.
//
// 이 파일에는 설명 문구를 넣지 않는다 (BEWE.md "UI 에 설명글 임의 추가 금지").
// 값을 보여주는 툴팁·라벨은 정보라 예외다.

#include "fft_viewer.hpp"
#include "net_client.hpp"
#include "kst_time.hpp"
#include "modules/modview.hpp"
#include "modules/common/modview_map.hpp"

#include "imgui.h"
#include <imgui_internal.h>   // BringWindowToDisplayFront / DC.CurrLineTextBaseOffset
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr float PI_F = 3.14159265358979f;
constexpr double D2R = 3.14159265358979 / 180.0;
constexpr double KM_PER_DEG_LAT = 111.32;

int64_t now_ms_local(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void hms(int64_t ms, char* o, size_t n){
    if(ms <= 0){ snprintf(o, n, "--:--:--"); return; }
    time_t t = (time_t)(ms / 1000); struct tm tv; KST::to_tm(t, tv);
    strftime(o, n, "%H:%M:%S", &tv);
}

const char* link_text(int l){
    return l == 2 ? "STREAMING" : (l == 1 ? "CALIBRATING" : "DOWN");
}

const char* algo_name(int a){
    return a == 0 ? "Bartlett" : (a == 1 ? "Capon" : "MUSIC");
}

// 엔진의 Config::validate() 와 같은 교차필드 제약. 위젯은 슬라이더 "범위"만 좁히고
// 저장값은 그대로 두기 때문에, elements 를 내리면 signal_dim 이 범위 밖에 남는다.
// 그 struct 는 HOST 가 거절하는데 거절 통보 경로가 없어 위젯이 유령값을 붙들고 있었다.
void clamp_cross_fields(PktDfConfig& c){
    if(c.elements < 3) c.elements = 3;
    if(c.elements > 8) c.elements = 8;
    if(c.signal_dim < 1) c.signal_dim = 1;
    if(c.signal_dim >= c.elements) c.signal_dim = (uint8_t)(c.elements - 1);
    if(c.max_frames < c.avg_frames) c.max_frames = c.avg_frames;
}

// ── 나침반 ────────────────────────────────────────────────────────────────
// 예전 draw_array_diagram + draw_polar_spectrum 을 하나로 합쳤다. 예전 플롯에는
// "0" 눈금 하나뿐이라 방위를 눈으로 읽을 수가 없었고, dB 바닥이 -40 에 박혀 있어
// MUSIC 의 뾰족한 봉우리가 바늘로만 보였다.
//
// spec_q: dB_below_peak = -0.5 * q (와이어 양자화 그대로). null 이면 곡선 생략.
// hover_az: 마우스가 원 안에 있으면 그 방위(도)를 돌려준다 (없으면 <0).
void draw_compass(ImDrawList* dl, ImVec2 c, float R,
                  const uint8_t* spec_q, bool have,
                  double bearing_deg, const double* alt_deg, int alt_n,
                  int elements, int sense, float floor_db, float* hover_az){
    const ImU32 col_ring  = IM_COL32(90,90,90,255);
    const ImU32 col_tick  = IM_COL32(120,120,120,255);
    const ImU32 col_txt   = IM_COL32(150,150,150,255);
    const ImU32 col_curve = IM_COL32(80,180,255,200);
    const ImU32 col_brg   = IM_COL32(80,220,80,255);

    auto pt = [&](double deg, float r){
        const float a = (float)(deg * D2R);
        return ImVec2(c.x + r * std::sin(a), c.y - r * std::cos(a));
    };

    // dB 링 3개 (바닥 / 절반 / 0)
    for(int i = 1; i <= 3; i++){
        const float rr = R * (float)i / 3.f;
        dl->AddCircle(c, rr, i == 3 ? col_ring : IM_COL32(60,60,60,255), 64, i == 3 ? 1.5f : 1.0f);
    }
    {
        char lb[16];
        for(int i = 1; i <= 3; i++){
            snprintf(lb, sizeof lb, "%.0f", floor_db * (float)(3 - i) / 3.f);
            dl->AddText(ImVec2(c.x + 3, c.y - R * (float)i / 3.f - 2), IM_COL32(95,95,95,255), lb);
        }
    }

    // 30도 눈금 + 사방위 + 도 라벨
    static const char* card[12] = { "N","30","60","E","120","150","S","210","240","W","300","330" };
    for(int i = 0; i < 12; i++){
        const double d = i * 30.0;
        const bool major = (i % 3) == 0;
        dl->AddLine(pt(d, R), pt(d, R - (major ? 10.f : 5.f)), col_tick, major ? 1.6f : 1.0f);
        const ImVec2 tp = pt(d, R + 13.f);
        const ImVec2 ts = ImGui::CalcTextSize(card[i]);
        dl->AddText(ImVec2(tp.x - ts.x * 0.5f, tp.y - ts.y * 0.5f),
                    major ? IM_COL32(190,190,190,255) : col_txt, card[i]);
    }

    // 의사스펙트럼 곡선
    if(have && spec_q && floor_db < -1.f){
        ImVec2 prev;
        for(int i = 0; i <= 360; i++){
            const int k = i % 360;
            const float db = -0.5f * (float)spec_q[k];
            // db=0 -> 반지름 1, db=floor -> 0. floor 는 음수다.
            float r = 1.0f - (db / floor_db);
            if(r < 0.f) r = 0.f; else if(r > 1.f) r = 1.f;
            const ImVec2 p = pt((double)k, R * r);
            if(i > 0) dl->AddLine(prev, p, col_curve, 1.2f);
            prev = p;
        }
    }

    // 배열 다이어그램 (중앙) — 배선 sense 실수가 보이는 유일한 곳이라 존치
    {
        const float ar = R * 0.30f;
        dl->AddCircle(c, ar, IM_COL32(70,70,70,255), 32, 1.0f);
        const float dir = (sense == 0) ? 1.0f : -1.0f;
        const int m = (elements < 1) ? 1 : elements;
        for(int i = 0; i < m; i++){
            const float phi = dir * 2.0f * PI_F * (float)i / (float)m;
            const ImVec2 p(c.x + ar * std::sin(phi), c.y - ar * std::cos(phi));
            dl->AddCircleFilled(p, 3.5f, i == 0 ? IM_COL32(255,200,0,255) : IM_COL32(150,150,150,255));
        }
    }

    // 대안 방위 (모호집합) — 주 방위보다 흐리게
    for(int i = 0; i < alt_n && i < 2; i++){
        dl->AddLine(c, pt(alt_deg[i], R), IM_COL32(255,170,60,120), 1.4f);
        dl->AddCircleFilled(pt(alt_deg[i], R), 3.0f, IM_COL32(255,170,60,160));
    }

    // 주 방위
    if(have){
        const ImVec2 tip = pt(bearing_deg, R + 8.f);
        dl->AddLine(c, tip, col_brg, 2.5f);
        dl->AddCircleFilled(tip, 4.0f, col_brg);
        char lb[24]; snprintf(lb, sizeof lb, "%.1f", bearing_deg);
        const ImVec2 lp = pt(bearing_deg, R + 30.f);
        const ImVec2 ls = ImGui::CalcTextSize(lb);
        dl->AddText(ImVec2(lp.x - ls.x * 0.5f, lp.y - ls.y * 0.5f), col_brg, lb);
    }

    // 호버 판독 — 값이지 설명이 아니므로 허용
    *hover_az = -1.f;
    const ImVec2 mp = ImGui::GetIO().MousePos;
    const float dx = mp.x - c.x, dy = mp.y - c.y;
    const float rr = std::sqrt(dx*dx + dy*dy);
    if(rr <= R + 6.f){
        float az = std::atan2(dx, -dy) * 180.f / PI_F;
        if(az < 0.f) az += 360.f;
        *hover_az = az;
        dl->AddLine(c, pt(az, R), IM_COL32(200,200,200,90), 1.0f);
        char lb[48];
        if(have && spec_q){
            const int k = ((int)(az + 0.5f)) % 360;
            snprintf(lb, sizeof lb, "%.0f deg   %.1f dB", az, -0.5f * (float)spec_q[k]);
        } else {
            snprintf(lb, sizeof lb, "%.0f deg", az);
        }
        dl->AddText(ImVec2(c.x - R, c.y + R + 18.f), IM_COL32(210,210,210,220), lb);
    }
}

// ── LOB 교차 fix ──────────────────────────────────────────────────────────
// 선택된 방위선들의 최소자승 교점 + 1σ 오차타원.
// ENU 근사 (Korea bbox 안에서 대권/평면 차이는 200 km 에서 1 px 미만).
//
// ui.cpp 의 accum_fix 를 부르지 않는다 — 그건 방출체 위치를 입력으로 요구하고
// 로그인 데모를 유기적으로 보이게 하려고 결정적 1도 의사난수 오차를 더한다.
// 그 항이 실제 DF 표시에 들어가면 안 된다. 누산 관용구만 가져왔다.
struct FixSolution {
    bool   ok = false;
    double lat = 0, lon = 0;      // lon: 동경 양수
    double maj_km = 0, min_km = 0, orient_deg = 0;
    double sig_deg_used = 0;      // 이 fix 에 실제로 쓴 평균 방위 불확도 (표시용)
    bool   crossing = false;      // 실제 교차인가 (false = 종방향은 추정 못 함)
};

// ── 측정 하나의 방위 불확도 (도) ──────────────────────────────────────────
// 예전엔 전 LOB 에 1도를 고정으로 썼다. 그러면 오차타원이 측정 품질과 무관해져
// SNR 38 dB 로 깨끗하게 잡은 방위와 임계에 겨우 걸친 방위가 같은 크기로 그려진다.
//
// 하한은 UCA CRB 다 (df/df_manifold.hpp 유도와 같은 식):
//   var(beta) = 1 / (N * SNR_el * (kr)^2 * M)      [rad^2]
// 다만 CRB 만 쓰면 안 된다. 실측(DGS-1, 700 MHz, SNR 38 dB, n_eff 6290)에서 CRB 는
// 0.0016도인데 같은 신호를 3회 잰 산포는 0.1도였다 — 실제 오차는 통계가 아니라
// 미보정 소자 위상오차(0.246 deg/deg)와 멀티패스가 지배한다. 그래서 CRB 를 계산하되
// 시스템 바닥으로 잘라, 신호가 아무리 좋아도 그 아래로는 안 내려가게 한다.
double df_bearing_sigma_deg(const FFTViewer::DFFix& f){
    const double kFloorDeg = 0.15;   // 아무리 좋아도 이 아래로는 안 믿는다
    const double kMaxDeg   = 8.0;    // 이보다 나쁘면 교점이 사실상 무의미
    if(f.manual_lob) return 2.0;     // 사람이 불러준 값 — 측정 통계가 없다
    if(f.cf_mhz <= 0.0f || f.elements < 3) return 1.0;

    // CRB 형태(1/(N*SNR*(kr)^2*M))를 쓰되 N 은 실측으로 앵커한다. 이론 n_eff(~6000)
    // 를 그대로 넣으면 700 MHz/SNR 38 dB 에서 0.0016도가 나오는데, 같은 신호를 3회
    // 잰 실제 산포는 0.1도였다 — 실오차는 통계가 아니라 미보정 소자 위상오차
    // (0.246 deg/deg)와 멀티패스가 지배한다. 그 지점이 0.1도가 되도록 N=1.58 로
    // 잡으면 SNR·주파수 의존성(kr 이 클수록, 신호가 셀수록 정확)은 CRB 대로 살면서
    // 절대값은 현실에 맞는다. 배열을 캘리브레이션하면 이 상수를 올려야 한다.
    const double kAnchorN = 1.58;
    const double kr  = 2.0 * 3.14159265358979 * ((double)f.cf_mhz * 1e6) / 299792458.0 * 0.175;
    const double snr = std::pow(10.0, (double)f.snr_db / 10.0);
    const double var = 1.0 / std::max(kAnchorN * snr * kr * kr * (double)f.elements, 1e-12);
    const double sig = std::sqrt(var) / D2R;
    return std::min(std::max(sig, kFloorDeg), kMaxDeg);
}

// lob_km: 광선 표시 길이. LOB 이 하나뿐일 때 **종방향 사전분포**로 쓴다 (아래 참조).
FixSolution df_solve_fix(const FFTViewer::DFFix* const* sel, int n, double lob_km){
    FixSolution out;
    if(n < 1) return out;
    if(n > FFTViewer::DF_HIST_MAX) n = FFTViewer::DF_HIST_MAX;

    // 좌표가 없는 fix 는 통째로 제외한다. 지도의 광선/마커도 같은 조건으로
    // 건너뛰므로, 여기만 (0,0) 을 남기면 적도-그리니치의 유령 기지가 해에
    // 끼어들어 보이지도 않는 선으로 교점을 끌어당긴다.
    double slat[FFTViewer::DF_HIST_MAX], slon[FFTViewer::DF_HIST_MAX], sbrg[FFTViewer::DF_HIST_MAX];
    double ssig[FFTViewer::DF_HIST_MAX];   // 측정별 방위 불확도 (rad)
    int m = 0;
    for(int i = 0; i < n; i++){
        float la = sel[i]->station_lat, lo = sel[i]->station_lon;
        if(la == 0.f && lo == 0.f) continue;
        if(lo < 0.f) lo = -lo;      // 지도 경로와 같은 정규화 (서경 저장 방어)
        if(la < 0.f) la = -la;
        slat[m] = la; slon[m] = lo; sbrg[m] = sel[i]->bearing_deg;
        ssig[m] = df_bearing_sigma_deg(*sel[i]) * D2R;
        m++;
    }
    if(m < 1) return out;

    // 기지가 전부 같은 자리인가 (= 교차가 아예 없는가). 예전엔 여기서 포기했는데,
    // 그러면 LOB 이 하나이거나 한 기지에서 반복 측정한 경우 아무것도 안 그렸다.
    // 실제로는 **횡방향은 이미 제약돼 있다** — 모르는 건 선을 따라 얼마나 먼가뿐이다.
    // 그 사실을 아주 길쭉한 타원으로 보여주는 게 맞다 (아래 종방향 사전분포).
    bool distinct = false;
    for(int i = 1 ; i < m && !distinct; i++)
        for(int j = 0; j < i && !distinct; j++)
            if(std::abs(slat[i]-slat[j]) > 1e-4 || std::abs(slon[i]-slon[j]) > 1e-4)
                distinct = true;

    double lat0 = 0, lon0 = 0;
    for(int i = 0; i < m; i++){ lat0 += slat[i]; lon0 += slon[i]; }
    lat0 /= m; lon0 /= m;
    const double kmlon = KM_PER_DEG_LAT * std::cos(lat0 * D2R);
    if(kmlon < 1e-6) return out;

    // s_i = 기지 ENU, p_i = 방위선에 수직인 단위벡터, d_i = 방위 방향.
    // 제약: p_i . x = p_i . s_i  (x 가 방위선 위에 있다는 뜻)
    double sx[FFTViewer::DF_HIST_MAX], sy[FFTViewer::DF_HIST_MAX];
    double px[FFTViewer::DF_HIST_MAX], py[FFTViewer::DF_HIST_MAX];
    double dx_[FFTViewer::DF_HIST_MAX], dy_[FFTViewer::DF_HIST_MAX];
    for(int i = 0; i < m; i++){
        sx[i] = (slon[i] - lon0) * kmlon;
        sy[i] = (slat[i] - lat0) * KM_PER_DEG_LAT;
        const double b = sbrg[i] * D2R;
        dx_[i] =  std::sin(b);      // 방위 방향 (East, North)
        dy_[i] =  std::cos(b);
        px[i] =  std::cos(b);       // 그 수직
        py[i] = -std::sin(b);
    }
    n = m;

    // 실제 교차가 있는가 = 서로 다른 위치 + 방위가 유의하게 갈림. 없으면 종방향은
    // 사전분포만으로 정해지므로, 중심을 광선 중점에 두어 타원이 선 위에 얹히게 한다.
    bool crossing = distinct;
    if(crossing){
        double mx_ang = 0.0;
        for(int i = 1; i < n; i++){
            double da = std::fabs(sbrg[i] - sbrg[0]);
            while(da > 180.0) da = 360.0 - da;
            mx_ang = std::max(mx_ang, da);
        }
        if(mx_ang < 2.0) crossing = false;   // 사실상 평행 — 교점을 못 믿는다
    }

    double ex = 0, ey = 0;
    double A[4] = {0,0,0,0};
    for(int pass = 0; pass < 3; pass++){
        double a11=0, a12=0, a22=0, b1=0, b2=0;
        for(int i = 0; i < n; i++){
            double w = 1.0;
            if(pass > 0){
                // 먼 LOB 일수록 각도 오차가 큰 횡오차로 번진다: sigma_perp = R * sigma_ang.
                // sigma_ang 은 측정마다 다르다 (df_bearing_sigma_deg) — 깨끗하게 잡은
                // 방위가 임계에 걸친 방위보다 교점을 더 강하게 끌어야 한다.
                const double dxk = ex - sx[i], dyk = ey - sy[i];
                const double R = std::sqrt(dxk*dxk + dyk*dyk);
                const double sperp = std::max(R * ssig[i], 0.05);   // 하한 50 m
                w = 1.0 / (sperp * sperp);
            }
            const double c = px[i]*sx[i] + py[i]*sy[i];
            a11 += w*px[i]*px[i]; a12 += w*px[i]*py[i]; a22 += w*py[i]*py[i];
            b1  += w*px[i]*c;     b2  += w*py[i]*c;
        }

        // ── 종방향 사전분포 ────────────────────────────────────────────────
        // LOB 하나로 만든 A = w·ppᵀ 는 rank 1 이라 역행렬이 없다 (횡방향만 제약,
        // 종방향은 완전 무제약). 예전엔 여기서 det≈0 을 보고 포기했는데, 그건
        // "아무것도 모른다" 가 아니라 "선을 따라 어디인지만 모른다" 다 — 횡방향은
        // 이미 방위 정밀도만큼 좁혀져 있다.
        //
        // 그래서 각 광선의 진행방향에 유한한 사전분포(표준편차 = 표시 길이의 절반)를
        // 넣는다. 운용자가 정한 LOB 길이가 곧 "이 안 어딘가"라는 뜻이므로 자연스럽고,
        // 결과는 횡으로 좁고 종으로 그 길이만큼 긴 타원이 된다.
        // 교차가 실제로 있으면 그쪽 정보가 훨씬 강해 이 항은 무시할 만큼 묻힌다.
        {
            const double Lp = std::max(lob_km * 0.5, 1.0);
            const double wp = 1.0 / (Lp * Lp);
            for(int i = 0; i < n; i++){
                const double cc = dx_[i]*sx[i] + dy_[i]*sy[i] + (crossing ? 0.0 : lob_km * 0.5);
                a11 += wp*dx_[i]*dx_[i]; a12 += wp*dx_[i]*dy_[i]; a22 += wp*dy_[i]*dy_[i];
                b1  += wp*dx_[i]*cc;     b2  += wp*dy_[i]*cc;
            }
        }

        const double det = a11*a22 - a12*a12;
        if(std::abs(det) < 1e-12) return out;
        ex = ( a22*b1 - a12*b2) / det;
        ey = (-a12*b1 + a11*b2) / det;
        A[0]=a11; A[1]=a12; A[2]=a12; A[3]=a22;
    }

    // LOB 은 직선이 아니라 **광선**이다. 최소자승은 직선으로 풀기 때문에 해가
    // 안테나 뒤쪽에 떨어질 수 있는데, 그건 방위가 가리키는 방향의 정반대다.
    // 그런 fix 를 지도에 그리면 운용자가 표적을 정반대로 읽는다.
    for(int i = 0; i < n; i++){
        const double t = (ex - sx[i])*dx_[i] + (ey - sy[i])*dy_[i];
        if(t <= 0.0) return out;   // 이 LOB 의 뒤쪽 — 교점 아님
    }
    out.crossing = crossing;

    // 공분산 = A^-1. 2x2 대칭 고유분해로 1σ 반축.
    const double det = A[0]*A[3] - A[1]*A[2];
    if(std::abs(det) < 1e-12) return out;
    const double c11 =  A[3]/det, c12 = -A[1]/det, c22 = A[0]/det;
    const double tr = c11 + c22, dt = c11*c22 - c12*c12;
    const double disc = std::max(tr*tr*0.25 - dt, 0.0);
    const double l1 = tr*0.5 + std::sqrt(disc);
    const double l2 = std::max(tr*0.5 - std::sqrt(disc), 0.0);
    out.maj_km = std::sqrt(std::max(l1, 0.0));
    out.min_km = std::sqrt(l2);
    // 장축 방향. atan2(2*c12, c11-c22)/2 를 쓴다 — 지구본 데모(ui.cpp draw_err_ellipse
    // 호출부)의 atan2(c12, l1-c22) 는 c12==0 이고 c11<c22 인 축정렬 공분산에서
    // atan2(0, 음수)=pi 가 되어 장단축이 90도 뒤바뀐다. 수치로 확인한 차이다.
    out.orient_deg = 0.5 * std::atan2(2.0*c12, c11 - c22) / D2R;

    double sg = 0.0;
    for(int i = 0; i < n; i++) sg += ssig[i];
    out.sig_deg_used = (sg / n) / D2R;

    out.lat = lat0 + ey / KM_PER_DEG_LAT;
    out.lon = lon0 + ex / kmlon;
    out.ok  = true;
    return out;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
void df_draw_panel(FFTViewer& v, bool just_opened){
    if(!v.df_panel_open) return;
    ImGuiIO& io = ImGui::GetIO();
    const float kBottomBarH = 32.0f;

    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, io.DisplaySize.y - kBottomBarH));
    ImGui::SetNextWindowBgAlpha(0.97f);
    if(just_opened) ImGui::SetNextWindowFocus();
    ImGui::Begin("##df_overlay", &v.df_panel_open,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    // 텍스트 입력 중에는 ESC/Ctrl+C 를 가로채지 않는다 — 필터를 치다가 ESC 를
    // 누르면 편집 취소가 아니라 패널 전체가 닫혀 버린다.
    const bool typing    = ImGui::GetIO().WantTextInput;
    const bool win_focus = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    if(win_focus && !typing && ImGui::IsKeyPressed(ImGuiKey_Escape, false))
        v.df_panel_open = false;

    // ── 숫자키 측정 (메인페이지와 동일 동작) ──────────────────────────────
    // 메인 핸들러(ui.cpp)는 `main_kbd_active` 안에 있어 DF 패널이 열리면 죽는다
    // (오버레이 격리 규칙). 그런데 DF 는 자기 기능이라 자기 창에서 막힐 이유가
    // 없다 — 같은 핸들러를 여기 둬서 메인/DF 두 곳 모두에서 눌린다.
    // 0 = 표시번호 10. 표시번호는 freq_sorted_display_num 이 매 프레임 계산하는
    // 주파수정렬 순위지 배열 인덱스가 아니다.
    //
    // **win_focus 로 게이트하지 않는다.** 채팅창은 별도 루트 윈도우라 그쪽이
    // 포커스를 쥐면 win_focus 가 false 가 되고, 그러면 "채팅 보면서 DF 숫자키"
    // 라는 요구가 그대로 깨진다. 대신 글자 입력 중인지(typing)와 위젯이 활성인지
    // 만 본다 — DF 패널이 열려 있다는 사실 자체가 이 코드가 도는 조건이다.
    if(!typing && !ImGui::IsAnyItemActive()){
        for(int k = 0; k < 10; k++){
            if(!ImGui::IsKeyPressed((ImGuiKey)(ImGuiKey_0 + k), false)) continue;
            const int want = (k == 0) ? 10 : k;
            v.df_request_by_display_num(want, /*from_auto=*/false);
            break;
        }
    }

    FFTViewer::DFLive L{};
    v.df_get_live(L);
    const bool is_join = (v.net_cli != nullptr);

    // HOST 정본 + 편집 버퍼. 정본을 매 프레임 위젯 변수에 덮어쓰면 드래그 중
    // 프레임마다 DF_CONFIG 가 나가고(HOST 가 validate+apply+broadcast+JSON write 를
    // 60 Hz 로 돈다) 손잡이가 뒤로 튄다. 위젯이 활성인 동안(그리고 송신 직후 hold
    // 창 동안) 재동기를 멈춘다. hold 가 만료되면 ui = canon 으로 되돌아가므로
    // HOST 가 거절한 값은 자동으로 진실에 스냅백한다.
    PktDfConfig canon{};
    v.df_get_cfg(canon);
    static PktDfConfig ui{};
    static double      hold_until = 0.0;
    const double now = ImGui::GetTime();
    if(just_opened){ ui = canon; hold_until = 0.0; }
    if(!(ImGui::IsAnyItemActive() || now < hold_until)) ui = canon;
    PktDfConfig& c = ui;

    // ── 영속 UI 상태 ──────────────────────────────────────────────────────
    static modview_map::MapView mv;
    static modview::Selection   sel;
    static float  split_tw   = 460.f;
    static bool   setup_open = false;
    static int    sort_col   = -1;
    static bool   sort_asc   = true;
    static bool   at_bottom  = true;
    static char   filter[64] = {};
    static float  lob_km     = 150.f;
    static float  polar_floor_db = -40.f;
    static int    meas_ch    = 0;          // 0 = 자동(선택 채널)
    static uint32_t last_seq  = 0;    // 마지막으로 본 df_hist_seq (개수 아님)
    static int      last_vis_n = 0;   // 직전 프레임 가시행 수 (tail-follow 용)
    // 수동 LOB 입력 (다른 기지의 방위를 손으로 넣어 교차 fix 를 만든다).
    // 기지는 이름으로 기억한다 — 목록은 unordered_map 순회로 매 프레임 다시
    // 만들어지므로 인덱스를 붙들면 다른 기지를 가리키게 된다.
    static bool        lob_popup = false;
    static std::string lob_stn;
    static float       lob_brg = 0.f;

    if(just_opened){ mv.big = false; }

    // 새 결과 도착 감지 (tail-follow 판정용). 개수가 아니라 push 시퀀스로 본다 —
    // df_hist_n 은 64 에서 포화하므로 링이 한 번 차면 개수 비교가 영원히 거짓이 된다.
    if(v.df_hist_seq != last_seq) last_seq = v.df_hist_seq;

    // ══════════════════ 헤더 스트립 ══════════════════
    const float W  = ImGui::GetContentRegionAvail().x;
    const float Hh = ImGui::GetContentRegionAvail().y;
    const float HDR = 30.f;

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f,0.12f,0.16f,1.f));
    ImGui::BeginChild("##df_hdr", ImVec2(W, HDR), false,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    {
        const float VCEN = 16.5f, GAP = 8.f;
        // ── 이 줄의 모든 프레임 위젯은 스스로 세로중심을 다시 잡아야 한다 ──
        // ImGui 의 SameLine 은 CursorPos.y 를 CursorPosPrevLine.y 로 되돌리는데,
        // ItemSize 가 그 값을 **줄의 첫 아이템** 기준으로 고정해 둔다 (line_y1).
        // 중간 아이템에 SetCursorPosY 를 걸어도 line_y1 은 안 움직인다. 즉 이 줄의
        // 원점은 맨 앞 제목 텍스트의 Y(텍스트 중심)에 박혀 있고, 버튼·콤보·입력창
        // (프레임 높이가 더 큼)은 그대로 두면 전부 3px 아래로 처진다.
        // 그래서 위젯마다 이 헬퍼를 부른다. 새 위젯을 추가할 때도 빠뜨리지 말 것.
        auto ROWY = [&]{ ImGui::SetCursorPosY(VCEN - ImGui::GetFrameHeight()*0.5f); };
        // 텍스트는 한 겹 더 있다: TextEx 가 그릴 때 CurrLineTextBaseOffset 을 더하는데,
        // 앞선 프레임 위젯들이 그 값을 FramePadding.y 로 올려놓는다. 그래서 목표 Y 에서
        // 그만큼 미리 빼야 실제로 줄 중앙에 놓인다.
        auto ROWT = [&]{
            const float off = ImGui::GetCurrentWindow()->DC.CurrLineTextBaseOffset;
            ImGui::SetCursorPosY(VCEN - ImGui::GetTextLineHeight()*0.5f - off);
        };

        ImGui::SetCursorPos(ImVec2(GAP, 0.f)); ROWT();
        ImGui::TextColored(ImVec4(0.9f,0.9f,0.9f,1.f), "DIRECTION FINDING");

        const bool df_ok = v.df_link_state() != 0;
        ImGui::SameLine(0, GAP*2);
        ROWY();
        ImGui::BeginDisabled(!df_ok);
        if(ImGui::Button("MEASURE")){
            int dn = meas_ch;
            if(dn <= 0 && v.selected_ch >= 0) dn = v.freq_sorted_display_num(v.selected_ch);
            if(dn > 0) v.df_request_by_display_num(dn, /*from_auto=*/false);
        }
        ImGui::EndDisabled();

        // 채널 선택 — 표시번호는 주파수 정렬 순위다 (배열 인덱스가 아님)
        ImGui::SameLine(0, 4);
        ROWY();
        ImGui::SetNextItemWidth(150);
        {
            char cur[48];
            if(meas_ch <= 0) snprintf(cur, sizeof cur, "CH (selected)");
            else             snprintf(cur, sizeof cur, "CH%d", meas_ch);
            if(ImGui::BeginCombo("##df_ch", cur)){
                if(ImGui::Selectable("CH (selected)", meas_ch <= 0)) meas_ch = 0;
                for(int i = 0; i < MAX_CHANNELS; i++){
                    if(!v.channels[i].filter_active) continue;
                    const int dn = v.freq_sorted_display_num(i);
                    if(dn <= 0) continue;
                    char it[64];
                    snprintf(it, sizeof it, "CH%d  %.4f MHz", dn,
                             (v.channels[i].s + v.channels[i].e) * 0.5);
                    if(ImGui::Selectable(it, meas_ch == dn)) meas_ch = dn;
                }
                ImGui::EndCombo();
            }
        }

        // AUTO DF 현황 — 여기서 만드는 게 아니라 채널 패널의 상태를 비추기만 한다.
        // 다만 일괄 해제는 둔다: 자동 측정이 폭주할 때 채널 패널을 열지 않고
        // 멈출 수 있어야 한다.
        int auto_n = 0;
        for(int i = 0; i < MAX_CHANNELS; i++)
            if(v.channels[i].filter_active && v.auto_df_on[i]) auto_n++;
        ImGui::SameLine(0, GAP);
        ROWY();
        if(auto_n > 0){
            char lb[32]; snprintf(lb, sizeof lb, "AUTO %d OFF", auto_n);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f,0.55f,0.1f,1.f));
            if(ImGui::Button(lb))
                for(int i = 0; i < MAX_CHANNELS; i++) v.auto_df_on[i] = false;
            ImGui::PopStyleColor();
        } else {
            ImGui::BeginDisabled(true); ImGui::Button("AUTO 0"); ImGui::EndDisabled();
        }

        ImGui::SameLine(0, GAP);
        ROWY();
        if(ImGui::Button("+ LOB")) lob_popup = true;

        ImGui::SameLine(0, GAP);
        ROWY();
        ImGui::SetNextItemWidth(180);
        ImGui::InputText("##df_filter", filter, sizeof filter);

        ImGui::SameLine(0, GAP);
        char cnt[32]; snprintf(cnt, sizeof cnt, "%d fix", v.df_hist_n);
        ROWT();
        ImGui::TextDisabled("%s", cnt);

        // 우측: CLEAR + SETUP. 창이 좁으면 왼쪽 위젯 위로 겹치므로 현재 커서
        // 오른쪽으로만 민다 (겹치느니 잘리는 편이 낫다).
        const float RM = 12.f;
        const float rx = W - RM - 70.f - 4.f - 60.f;
        ImGui::SameLine();
        ImGui::SetCursorPosX(std::max(rx, ImGui::GetCursorPosX() + 8.f));
        ROWY();
        if(ImGui::Button("CLEAR", ImVec2(60,0))){
            v.df_hist_n = 0; v.df_hist_head = 0; sel.clear();
            v.df_last_valid = false;
            last_seq = v.df_hist_seq; last_vis_n = 0;
        }
        ImGui::SameLine(0, 4);
        ROWY();
        // 클릭 전 상태를 스냅샷해서 push/pop 가드를 고정한다. `if(setup_open)` 을
        // 양쪽에 쓰면 버튼이 그 조건 자체를 뒤집어 컬러 스택이 매 토글마다 깨진다.
        const bool setup_hi = setup_open;
        if(setup_hi) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f,0.45f,0.6f,1.f));
        if(ImGui::Button("SETUP", ImVec2(70,0))) setup_open = !setup_open;
        if(setup_hi) ImGui::PopStyleColor();
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // Kraken 이 아닌 로컬 HOST 면 여기서 끝. (원인 + 조치라 정보다)
    if(!is_join && v.hw.type != HWType::KRAKEN){
        lob_popup = false;   // 여기서 소비 안 하면 다음에 열 때 팝업이 튀어나온다
        ImGui::TextColored(ImVec4(1.f,0.4f,0.4f,1.f),
            "DF requires the KrakenSDR backend. Start BEWE with  --sdr kraken");
        ImGui::End();
        return;
    }

    // 헤더/배너는 child 라 각각 뒤에 ItemSpacing 이 붙는다. 그만큼 빼지 않으면
    // 배너가 켜질 때 본문이 창 밖으로 밀려 아래가 잘린다.
    const float SPY = ImGui::GetStyle().ItemSpacing.y;
    float body_h = Hh - HDR - SPY;
    if(body_h < 120.f) body_h = 120.f;

    // ══════════════════ 가시 목록 (필터 + 정렬) ══════════════════
    static std::vector<int> vis;
    vis.clear();
    for(int i = 0; i < v.df_hist_n; i++){
        const FFTViewer::DFFix& f = v.df_hist_at(i);
        if(filter[0]){
            char buf[160];
            snprintf(buf, sizeof buf, "CH%u %.4f %.1f %s", (unsigned)f.dnum,
                     f.cf_mhz, f.bearing_deg, f.note);
            if(!modview::ci_find(buf, filter)) continue;
        }
        vis.push_back(i);
    }
    // 0 Time 1 CH 2 Freq 3 Brg 4 SNR 5 Conf 6 Frm 7 Note
    modview::sort_vis(vis, sort_col, sort_asc, [&](int col, int a, int b)->int{
        const FFTViewer::DFFix& x = v.df_hist_at(a);
        const FFTViewer::DFFix& y = v.df_hist_at(b);
        auto cf = [](double p, double q){ return p < q ? -1 : (p > q ? 1 : 0); };
        switch(col){
            case 0: return cf((double)x.t_end_ms, (double)y.t_end_ms);
            case 1: return cf(x.dnum, y.dnum);
            case 2: return cf(x.cf_mhz, y.cf_mhz);
            case 3: return cf(x.bearing_deg, y.bearing_deg);
            case 4: return cf(x.snr_db, y.snr_db);
            case 5: return cf(x.conf_db, y.conf_db);
            case 6: return cf(x.frames_used, y.frames_used);
            default: return strcmp(x.note, y.note);
        }
    });
    // 행 키는 push 시퀀스다. t_end_ms 를 쓰면 측정 전 거절(df_post_refusal)이
    // 전부 0 이라 같은 키가 되고, 거절 하나를 클릭하면 거절 행 전체가 선택된다.
    auto key_of = [&](int hist_i){
        char k[24]; snprintf(k, sizeof k, "%u", (unsigned)v.df_hist_at(hist_i).seq);
        return std::string(k);
    };
    auto key_at_vis = [&](int p)->std::string{
        return (p >= 0 && p < (int)vis.size()) ? key_of(vis[p]) : std::string();
    };

    // ══════════════════ 좌: 이력표 | 스플리터 | 우: 지도 ══════════════════
    const float setup_w = setup_open ? 340.f : 0.f;
    float tw, mapw;
    if(mv.big){ tw = 0.f; mapw = W - setup_w - (setup_w > 0 ? 4.f : 0.f); }
    else {
        tw = split_tw;
        const float maxtw = W - setup_w - 60.f;
        if(tw > maxtw) tw = maxtw;
        if(tw < 200.f) tw = 200.f;
        mapw = W - tw - 6.f - setup_w - (setup_w > 0 ? 4.f : 0.f);
        if(mapw < 10.f) mapw = 10.f;
    }

    if(!mv.big){
        ImGuiTableFlags tf = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg |
                             ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable;
        if(ImGui::BeginTable("##df_tbl", 8, tf, ImVec2(tw, body_h))){
            ImGui::TableSetupScrollFreeze(2, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 74);
            ImGui::TableSetupColumn("CH",   ImGuiTableColumnFlags_WidthFixed, 36);
            ImGui::TableSetupColumn("Freq", ImGuiTableColumnFlags_WidthFixed, 84);
            ImGui::TableSetupColumn("Brg",  ImGuiTableColumnFlags_WidthFixed, 54);
            ImGui::TableSetupColumn("SNR",  ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("Conf", ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("Frm",  ImGuiTableColumnFlags_WidthFixed, 52);
            ImGui::TableSetupColumn("Note", ImGuiTableColumnFlags_WidthStretch);
            modview::sortable_headers(8, sort_col, sort_asc, /*text_col=*/7);

            for(int p = 0; p < (int)vis.size(); p++){
                const int hi = vis[p];
                const FFTViewer::DFFix& f = v.df_hist_at(hi);
                const std::string k = key_of(hi);
                ImGui::TableNextRow();

                char ts[16]; hms(f.t_end_ms, ts, sizeof ts);
                if(modview::row_col0((int)(f.t_end_ms & 0x7fffffff) ^ hi, sel.selected(k), ts))
                    sel.click(k, p, key_at_vis, (int)vis.size(), io.KeyCtrl, io.KeyShift);

                char b[32];
                ImGui::TableSetColumnIndex(1);
                if(f.dnum > 0){ snprintf(b, sizeof b, "%u", (unsigned)f.dnum); modview::cell(b); }
                else modview::cell("-");
                ImGui::TableSetColumnIndex(2);
                if(f.cf_mhz > 0.f){ snprintf(b, sizeof b, "%.4f", f.cf_mhz); modview::cell(b); }
                else modview::cell("-");
                ImGui::TableSetColumnIndex(3);
                if(f.kind == 0){
                    snprintf(b, sizeof b, "%.1f", f.bearing_deg);
                    modview::cell(b, f.manual_lob ? ImVec4(0.6f,0.8f,1.f,1.f)
                                                  : ImVec4(0.3f,0.9f,0.3f,1.f));
                } else modview::cell("-");
                // 수동 LOB 은 측정이 아니다 — 0 을 SNR/신뢰도/프레임수로 찍으면
                // 실제로 잰 값처럼 보인다.
                const bool measured = (f.kind == 0) && !f.manual_lob;
                ImGui::TableSetColumnIndex(4);
                if(measured){ snprintf(b, sizeof b, "%.1f", f.snr_db); modview::cell(b); }
                else modview::cell("-");
                ImGui::TableSetColumnIndex(5);
                if(measured){ snprintf(b, sizeof b, "%.2f", f.conf_db); modview::cell(b); }
                else modview::cell("-");
                ImGui::TableSetColumnIndex(6);
                if(measured){
                    snprintf(b, sizeof b, "%u/%u", (unsigned)f.frames_used,
                             (unsigned)f.frames_discarded); modview::cell(b);
                } else modview::cell("-");
                ImGui::TableSetColumnIndex(7);
                {
                    const bool bad = (f.kind != 0) || f.imbalance || f.overdrive_mask;
                    ImVec4 nc = bad ? ImVec4(1.f,0.6f,0.35f,1.f) : ImVec4(0.65f,0.65f,0.65f,1.f);
                    // 성공했는데 특별히 알릴 게 없으면 엔진이 note 에 "ok" 를 채운다
                    // (status_text(Status::Ok)). 그건 방위·SNR 이 이미 말해주는 사실이라
                    // 행마다 반복하면 정작 봐야 할 진단 문구가 묻힌다.
                    const bool plain_ok = (f.kind == 0) && (strcmp(f.note, "ok") == 0);
                    char nb[96];
                    snprintf(nb, sizeof nb, "%s%s", f.origin ? "[A] " : "",
                             plain_ok ? "" : f.note);
                    modview::cell_left(nb, &nc);
                }
            }
            // grew 는 **가시행 수가 늘었을 때**만 참이어야 한다. 전체 개수와
            // 비교하면 필터가 켜진 동안 매 프레임 참이 되어 스크롤이 바닥에
            // 못박히고, 필터가 없으면 영영 거짓이라 tail-follow 자체가 죽는다.
            modview::tail_follow(at_bottom, (int)vis.size() > last_vis_n && sort_col < 0);
            ImGui::EndTable();
        }

        // 스플리터
        ImGui::SameLine(0, 0);
        ImGui::InvisibleButton("##df_split", ImVec2(6.f, body_h));
        if(ImGui::IsItemActive()){
            // 클램프한 값(tw)에서 이어 받는다. 원시 누적값을 계속 키우면 한계
            // 밖으로 나간 만큼이 그대로 남아, 되돌릴 때 그만큼 먹통 구간이 생긴다.
            split_tw = tw + io.MouseDelta.x;
            const float maxtw = W - setup_w - 60.f;
            if(split_tw > maxtw) split_tw = maxtw;
            if(split_tw < 200.f) split_tw = 200.f;
        }
        if(ImGui::IsItemHovered() || ImGui::IsItemActive()){
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            const ImVec2 rmin = ImGui::GetItemRectMin(), rmax = ImGui::GetItemRectMax();
            ImGui::GetWindowDrawList()->AddLine(ImVec2((rmin.x+rmax.x)*0.5f, rmin.y),
                                                ImVec2((rmin.x+rmax.x)*0.5f, rmax.y),
                                                IM_COL32(120,160,200,180), 1.5f);
        }
        ImGui::SameLine(0, 0);
    }

    // ── 선택된 fix 들 ────────────────────────────────────────────────────
    // **가시(필터 통과) 행만** 모은다. 필터로 숨긴 행이 계속 교점을 끌어당기면
    // 지도에 그려지지 않은 선이 fix 를 움직이는 셈이라 설명이 불가능해진다.
    static std::vector<const FFTViewer::DFFix*> selected;
    selected.clear();
    const FFTViewer::DFFix* focus = nullptr;
    for(int p = 0; p < (int)vis.size(); p++){
        const FFTViewer::DFFix& f = v.df_hist_at(vis[p]);
        if(f.kind != 0) continue;
        const std::string k = key_of(vis[p]);
        if(sel.selected(k)){
            selected.push_back(&f);
            if(sel.focus == k) focus = &f;
        }
    }
    if(!focus && !selected.empty()) focus = selected.back();
    if(!focus && v.df_hist_n > 0){
        for(int i = v.df_hist_n - 1; i >= 0; i--)
            if(v.df_hist_at(i).kind == 0){ focus = &v.df_hist_at(i); break; }
    }

    // ── 지도 ─────────────────────────────────────────────────────────────
    static std::vector<modview_map::MapPoint>  pts;
    static std::vector<modview_map::MapStation> stns;
    static std::vector<std::string>            stn_names;
    pts.clear(); stns.clear(); stn_names.clear();

    // 한국 운용 전제 좌표 정규화 — 서경으로 저장된 값 방어 (cli_host 는 서경 양수)
    auto norm = [](float& la, float& lo){ if(lo < 0.f) lo = -lo; if(la < 0.f) la = -la; };

    {
        std::lock_guard<std::mutex> ck(v.station_geo_cache_mtx);
        for(const auto& kv : v.station_geo_cache){
            float la = kv.second.first, lo = kv.second.second;
            if(la == 0.f && lo == 0.f) continue;
            norm(la, lo);
            stn_names.push_back(kv.first);
            modview_map::MapStation ms; ms.lat = la; ms.lon = lo;
            stns.push_back(ms);
        }
        if(stns.empty() && (v.station_lat != 0.f || v.station_lon != 0.f)){
            float la = v.station_lat, lo = v.station_lon; norm(la, lo);
            stn_names.push_back(v.station_name.empty() ? std::string("HOST") : v.station_name);
            modview_map::MapStation ms; ms.lat = la; ms.lon = lo;
            stns.push_back(ms);
        }
    }
    for(size_t i = 0; i < stns.size(); i++) stns[i].name = stn_names[i].c_str();

    // LOB 위에 마커를 찍지 않는다 (pts 는 계속 빈 채로 draw_map 에 넘긴다).
    // 지도의 점은 "거기에 뭐가 있다" 로 읽히는데, 방위선 중점은 표적 위치가 아니라
    // 그냥 선의 절반 지점이다 — 표적은 그 선 **어딘가**에 있고, 어디인지는 다른
    // 선과 만나야 알 수 있다. 클릭 선택은 광선 자체를 히트테스트해 처리한다.

    // 이력이 비면 draw_map 은 auto-fit 을 안 한다 — 기지 중심으로 시드해 준다.
    if(just_opened && pts.empty() && !stns.empty() && !mv.initialized){
        mv.lat0 = stns[0].lat - 0.75; mv.lat1 = stns[0].lat + 0.75;
        mv.lon0 = stns[0].lon - 0.75; mv.lon1 = stns[0].lon + 0.75;
        mv.initialized = true;
    }

    const ImVec2 map_p0 = ImGui::GetCursorScreenPos();
    (void)modview_map::draw_map("##df_map", mv, pts, ImVec2(mapw, body_h),
                              just_opened, &stns, nullptr);

    // 지도 클릭 -> 표와 **동일한** 선택 경로 (선택 모델이 하나라야 ctrl/shift 의미가
    // 같다). 클릭 대상은 마커가 아니라 **광선 자체**다 — 아래 LOB 렌더 블록에서
    // 커서와 각 선분의 거리를 재 가장 가까운 것을 고른다.
    int    lob_hit_p   = -1;
    double lob_hit_d2  = 12.0 * 12.0;   // 12 px 안쪽만 후보

    // ── LOB 광선 + 교차 fix (draw_map 위에 덧그린다) ────────────────────
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->PushClipRect(map_p0, ImVec2(map_p0.x + mapw, map_p0.y + body_h), true);
        const ImVec2 mp_pos = io.MousePos;
        auto P = [&](double lat, double lon){
            return ImVec2((float)(map_p0.x + (lon - mv.lon0) / (mv.lon1 - mv.lon0) * mapw),
                          (float)(map_p0.y + (mv.lat1 - lat) / (mv.lat1 - mv.lat0) * body_h));
        };
        for(int p = 0; p < (int)vis.size(); p++){
            const FFTViewer::DFFix& f = v.df_hist_at(vis[p]);
            if(f.kind != 0) continue;
            float sla = f.station_lat, slo = f.station_lon;
            if(sla == 0.f && slo == 0.f) continue;
            norm(sla, slo);
            const bool is_sel = sel.selected(key_of(vis[p]));
            const float amul  = is_sel ? 1.0f : 0.35f;
            const double clat = std::cos(sla * D2R);
            const double e_lat = sla + (lob_km * std::cos(f.bearing_deg * D2R)) / KM_PER_DEG_LAT;
            const double e_lon = slo + (lob_km * std::sin(f.bearing_deg * D2R))
                                       / (KM_PER_DEG_LAT * (clat > 0.05 ? clat : 0.05));
            ImVec2 prev = P(sla, slo);
            for(int s = 1; s <= 16; s++){
                const double t = (double)s / 16.0;
                const ImVec2 cur = P(sla + (e_lat - sla)*t, slo + (e_lon - slo)*t);
                const float fade = (float)(1.0 - 0.65*t) * amul;
                dl->AddLine(prev, cur, IM_COL32(255,210,60,(int)(55*fade)),  5.0f);
                dl->AddLine(prev, cur, IM_COL32(255,236,150,(int)(205*fade)), 1.6f);
                // 커서-선분 거리 (클릭 히트테스트용)
                {
                    const float vx = cur.x-prev.x, vy = cur.y-prev.y;
                    const float wx = mp_pos.x-prev.x, wy = mp_pos.y-prev.y;
                    const float L2 = vx*vx + vy*vy;
                    float tt = (L2 > 1e-6f) ? (wx*vx + wy*vy)/L2 : 0.f;
                    tt = tt < 0.f ? 0.f : (tt > 1.f ? 1.f : tt);
                    const float ddx = wx - vx*tt, ddy = wy - vy*tt;
                    const double d2 = (double)(ddx*ddx + ddy*ddy);
                    if(d2 < lob_hit_d2){ lob_hit_d2 = d2; lob_hit_p = p; }
                }
                prev = cur;
            }
        }
        // 교차 fix
        if(!selected.empty()){
            FixSolution fx = df_solve_fix(selected.data(), (int)selected.size(), lob_km);
            if(fx.ok){
                const ImVec2 cp = P(fx.lat, fx.lon);
                const double clat = std::cos(fx.lat * D2R);
                const double kmlon = KM_PER_DEG_LAT * (clat > 0.05 ? clat : 0.05);
                // 95% 신뢰타원. 1σ 를 그대로 그리면 2D 정규분포에서 39% 밖에
                // 안 담고, 무엇보다 실제 축척에서 안 보인다 — 두 기지가 다 들어오는
                // 줌(0.7~7 px/km)에서 깨끗한 fix 의 1σ 는 지름 0.2~2 px 다.
                // r = sqrt(-2 ln(1-p)) 이므로 95% 는 2.4477σ.
                const double kConf = 2.4477;
                const double maj_km = fx.maj_km * kConf, min_km = fx.min_km * kConf;

                ImVec2 poly[48];
                const double th = fx.orient_deg * D2R;
                for(int i = 0; i < 48; i++){
                    const double a = 2.0*3.14159265358979*i/48.0;
                    const double ex = maj_km*std::cos(a), ey = min_km*std::sin(a);
                    const double rx = ex*std::cos(th) - ey*std::sin(th);
                    const double ry = ex*std::sin(th) + ey*std::cos(th);
                    poly[i] = P(fx.lat + ry/KM_PER_DEG_LAT, fx.lon + rx/kmlon);
                }

                // 화면에서 얼마나 큰지 재서, 너무 작으면 최소 크기를 보장한다.
                // 다만 그때는 **점선**으로 그려 "이건 실제 크기가 아니라 하한을
                // 그린 것" 임을 구분한다 — 실선으로 키우면 오차를 과장해 보고하는
                // 셈이고, 그건 지운 글로우와 같은 잘못이다.
                float mnx = poly[0].x, mxx = poly[0].x, mny = poly[0].y, mxy = poly[0].y;
                for(int i = 1; i < 48; i++){
                    mnx = std::min(mnx, poly[i].x); mxx = std::max(mxx, poly[i].x);
                    mny = std::min(mny, poly[i].y); mxy = std::max(mxy, poly[i].y);
                }
                const float span_px = std::max(mxx-mnx, mxy-mny);
                // 14px = 눈에 형태가 잡히는 하한. 더 키우면 실제로는 보이는 크기인
                // 타원까지 점선으로 밀려난다 (전국 축척의 weak fix 가 21px 다).
                const float kMinPx  = 14.f;
                const bool  tiny    = (span_px < kMinPx);
                if(tiny){
                    const float k = kMinPx / std::max(span_px, 0.01f);
                    for(int i = 0; i < 48; i++){
                        poly[i].x = cp.x + (poly[i].x - cp.x) * k;
                        poly[i].y = cp.y + (poly[i].y - cp.y) * k;
                    }
                }

                if(!tiny){
                    dl->AddConvexPolyFilled(poly, 48, IM_COL32(255,205,70,30));
                    for(int i = 0; i < 48; i++)
                        dl->AddLine(poly[i], poly[(i+1)%48], IM_COL32(255,232,130,150), 1.2f);
                } else {
                    // 점선 = 실제 타원이 이 원보다 작다는 뜻
                    for(int i = 0; i < 48; i += 2)
                        dl->AddLine(poly[i], poly[(i+1)%48], IM_COL32(255,232,130,130), 1.2f);
                }
                // 중심은 십자만. 지구본 데모처럼 반지름 6px 글로우를 얹으면,
                // 기본 줌에서 타원 자체가 1px 미만이라 **그 글로우가 곧 "동그라미"로
                // 보인다** — 실제 오차 크기와 무관한 고정 크기 원이 오차타원 행세를
                // 한다. 타원이 안 보일 만큼 작으면 작다는 사실이 보여야 맞다.
                if(fx.crossing){
                    dl->AddLine(ImVec2(cp.x-7,cp.y), ImVec2(cp.x+7,cp.y), IM_COL32(255,240,160,230), 1.6f);
                    dl->AddLine(ImVec2(cp.x,cp.y-7), ImVec2(cp.x,cp.y+7), IM_COL32(255,240,160,230), 1.6f);
                }
                char lb[96];
                // 라벨은 언제나 실제 95% 크기를 적는다 (화면에서 키웠든 아니든).
                // 교차가 없으면 "FIX" 라고 쓰면 안 된다 — 위치가 정해진 게 아니라
                // 선을 따라 어디인지만 모르는 상태다. 종방향 크기는 LOB 길이에서
                // 온 가정값이므로 그렇게 읽히게 라벨을 나눈다.
                if(fx.crossing)
                    snprintf(lb, sizeof lb, "FIX %.4fN %.4fE  95%% %.2f x %.2f km  (%.1f deg)%s",
                             fx.lat, fx.lon, maj_km, min_km, fx.sig_deg_used,
                             tiny ? "  [<]" : "");
                else
                    snprintf(lb, sizeof lb, "LOB only  cross-track 95%% %.2f km  (%.1f deg)",
                             min_km, fx.sig_deg_used);
                dl->AddText(ImVec2(cp.x + 10, cp.y - 16), IM_COL32(255,240,160,240), lb);
            }
        }
        dl->PopClipRect();

        // 광선 클릭 -> 표와 같은 선택 경로. map_view 와 같은 규칙으로 드래그를
        // 걸러낸다 (놓는 순간 + 이동량 4px 미만) — 안 그러면 지도를 팬할 때마다
        // 지나간 선이 선택된다. 호버 시 굵기를 키워 클릭 가능함을 알린다.
        if(lob_hit_p >= 0){
            const bool inside = mp_pos.x >= map_p0.x && mp_pos.x <= map_p0.x + mapw &&
                                mp_pos.y >= map_p0.y && mp_pos.y <= map_p0.y + body_h;
            const ImVec2 dg = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
            const bool dragged = (dg.x*dg.x + dg.y*dg.y) > 16.f;
            if(inside) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            if(inside && !dragged && ImGui::IsMouseReleased(ImGuiMouseButton_Left)){
                sel.click(key_of(vis[lob_hit_p]), lob_hit_p, key_at_vis,
                          (int)vis.size(), io.KeyCtrl, io.KeyShift);
            }
        }
    }

    // ── 나침반 오버레이 (지도 좌하단 — 우하단은 축척바, 좌상단은 토글열) ──
    {
        // 좌하단이되 map_view 의 커서 lat/lon 판독(맨 아래 줄)은 피해 올려둔다.
        const float CR = 92.f;
        const ImVec2 cc(map_p0.x + CR + 34.f, map_p0.y + body_h - CR - 58.f);
        if(mapw > 2*CR + 80.f && body_h > 2*CR + 80.f){
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->PushClipRect(map_p0, ImVec2(map_p0.x + mapw, map_p0.y + body_h), true);
            dl->AddCircleFilled(cc, CR + 34.f, IM_COL32(12,14,18,190), 48);
            float hover_az = -1.f;
            const bool have = (focus != nullptr);
            double altd[2] = {0,0};
            int altn = 0;
            if(have){ altd[0]=focus->alt_deg[0]; altd[1]=focus->alt_deg[1]; altn=focus->alt_n; }
            draw_compass(dl, cc, CR,
                         have && focus->has_spec ? focus->spec_q : nullptr,
                         have, have ? focus->bearing_deg : 0.0, altd, altn,
                         c.elements, c.sense, polar_floor_db, &hover_az);
            dl->PopClipRect();
        }
    }

    // ── 세부 판독 (지도 우상단) ─────────────────────────────────────────
    if(focus){
        ImDrawList* dl = ImGui::GetWindowDrawList();
        char l1[128], l2[128], l3[128], l4[128];
        char ts[16]; hms(focus->t_end_ms, ts, sizeof ts);
        snprintf(l1, sizeof l1, "CH%u  %.4f MHz  BW %.1f kHz  %s",
                 (unsigned)focus->dnum, focus->cf_mhz, focus->bw_khz, ts);
        snprintf(l2, sizeof l2, "bearing %.1f deg   (ant0 %.1f)",
                 focus->bearing_deg, focus->bearing_rel_deg);
        snprintf(l3, sizeof l3, "SNR %.1f dB   conf %.2f dB   pwr %.1f dBFS   %s",
                 focus->snr_db, focus->conf_db, focus->power_dbfs, algo_name(focus->algo));
        if(focus->alt_n > 0)
            snprintf(l4, sizeof l4, "alt %.0f deg %.0f dB", focus->alt_deg[0], focus->alt_db[0]);
        else l4[0] = 0;
        const float bw = 380.f;
        const ImVec2 p0(map_p0.x + mapw - bw - 10.f, map_p0.y + 10.f);
        const float bh = l4[0] ? 78.f : 62.f;
        dl->PushClipRect(map_p0, ImVec2(map_p0.x + mapw, map_p0.y + body_h), true);
        dl->AddRectFilled(p0, ImVec2(p0.x + bw, p0.y + bh), IM_COL32(12,14,18,205), 4.f);
        dl->AddRect(p0, ImVec2(p0.x + bw, p0.y + bh), IM_COL32(70,80,95,220), 4.f);
        dl->AddText(ImVec2(p0.x+8, p0.y+6),  IM_COL32(200,200,200,255), l1);
        dl->AddText(ImVec2(p0.x+8, p0.y+22), IM_COL32(90,230,90,255),   l2);
        dl->AddText(ImVec2(p0.x+8, p0.y+38), IM_COL32(180,180,180,255), l3);
        if(l4[0]) dl->AddText(ImVec2(p0.x+8, p0.y+54), IM_COL32(255,180,80,255), l4);
        dl->PopClipRect();
    }

    // ══════════════════ 설정 창 (기본 닫힘) ══════════════════
    if(setup_open){
        ImGui::SameLine(0, 4);
        // JOIN 이 HOST 정본을 아직 못 받았으면 못 만지게 막는다. 안 막으면
        // df_default_pkt 하드코딩 기본값이 첫 조작에서 통째로 HOST 를 덮어쓴다
        // (host_state 에 영속화까지 된다).
        const bool cfg_ready = !is_join || v.net_cli->df_cfg_valid.load();
        ImGui::BeginChild("##df_setup", ImVec2(setup_w, body_h), true);
        ImGui::BeginDisabled(!cfg_ready);

        ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "DAQ");
        {
            const ImVec4 cl = (L.link==2) ? ImVec4(0.3f,0.9f,0.3f,1.f)
                            : (L.link==1) ? ImVec4(1.f,0.8f,0.f,1.f)
                                          : ImVec4(0.9f,0.3f,0.3f,1.f);
            ImGui::TextColored(cl, "[%s]", link_text(L.link));
            ImGui::SameLine();
            ImGui::Text("sync %u/6  ch %u", L.sync_state, L.channels);
            if(L.link > 0){
                ImGui::Text("%s  %.4f MHz  %.3f MSPS", L.hw_id, L.daq_cf_mhz, L.daq_fs_msps);
                ImGui::Text("%.2f fps  %.1f MB/s", L.frame_rate_hz, L.recv_mbps);
                ImGui::Text("ok %llu  cal %llu  bad %llu  gap %llu  rec %llu",
                            L.frames_ok, L.frames_cal, L.frames_bad, L.gaps, L.reconnects);
                ImGui::Text("gain:");
                for(unsigned i = 0; i < L.channels && i < 8; i++){
                    ImGui::SameLine(); ImGui::Text("%.1f", L.gain_tenths[i]/10.0);
                }
            }
            if(L.last_error[0]) ImGui::TextColored(ImVec4(1.f,0.5f,0.5f,1.f), "%s", L.last_error);
            if(L.overdrive) ImGui::TextColored(ImVec4(1.f,0.5f,0.f,1.f),
                                               "ADC OVERDRIVE 0x%x", L.overdrive);
            if(L.channels > 0 && (unsigned)c.elements != L.channels)
                ImGui::TextColored(ImVec4(1.f,0.5f,0.5f,1.f), "DAQ ch=%u", L.channels);
        }
        if(L.measuring){
            ImGui::TextColored(ImVec4(1.f,0.8f,0.f,1.f), "measuring...");
            ImGui::ProgressBar(L.progress, ImVec2(-1, 0));
        }

        ImGui::Dummy(ImVec2(0,6)); ImGui::Separator();
        ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "DAQ CONTROL");
        { bool ctl = (c.enable_control != 0);
          if(ImGui::Checkbox("retune DAQ", &ctl)) c.enable_control = ctl?1:0; }

        ImGui::Dummy(ImVec2(0,6)); ImGui::Separator();
        ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "ARRAY GEOMETRY");
        ImGui::SetNextItemWidth(150);
        ImGui::InputFloat("radius (m)", &c.radius_m, 0.005f, 0.05f, "%.4f");
        { int el = c.elements; ImGui::SetNextItemWidth(150);
          if(ImGui::SliderInt("elements", &el, 3, 8)) c.elements = (uint8_t)el; }
        { int sn = c.sense; ImGui::SetNextItemWidth(150);
          const char* it[] = { "CW", "CCW" };
          if(ImGui::Combo("numbering", &sn, it, 2)) c.sense = (uint8_t)sn; }
        ImGui::SetNextItemWidth(150);
        ImGui::InputFloat("heading offset (deg)", &c.heading_deg, 1.0f, 10.0f, "%.1f");
        if(L.lambda_m > 0.0){
            ImGui::Text("lambda %.3f m   ambiguity %.3f", L.lambda_m, L.ambiguity);
            if(L.ambiguity > 1.0)
                ImGui::TextColored(ImVec4(1.f,0.6f,0.f,1.f), "grating lobes");
        }

        ImGui::Dummy(ImVec2(0,6)); ImGui::Separator();
        ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "ESTIMATION");
        { int al = c.algo; ImGui::SetNextItemWidth(150);
          const char* it[] = { "Bartlett", "Capon (MVDR)", "MUSIC" };
          if(ImGui::Combo("algorithm", &al, it, 3)) c.algo = (uint8_t)al; }
        { int sd = c.signal_dim; ImGui::SetNextItemWidth(150);
          if(ImGui::SliderInt("sources (MUSIC)", &sd, 1, c.elements > 1 ? c.elements-1 : 1))
              c.signal_dim = (uint8_t)sd; }
        { int af = c.avg_frames; ImGui::SetNextItemWidth(150);
          if(ImGui::SliderInt("frames to average", &af, 1, 30)) c.avg_frames = (uint8_t)af;
          ImGui::SameLine(); ImGui::TextDisabled("~%.1f s", af * 0.437); }
        { int mf = c.max_frames; ImGui::SetNextItemWidth(150);
          if(ImGui::SliderInt("frame budget", &mf, c.avg_frames, 60)) c.max_frames = (uint8_t)mf; }

        ImGui::Dummy(ImVec2(0,6)); ImGui::Separator();
        ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "ACCEPTANCE");
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("SNR threshold (dB)", &c.snr_thr_db, -10.0f, 40.0f, "%.0f");

        ImGui::Dummy(ImVec2(0,6)); ImGui::Separator();
        ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "SIGNAL EXTRACTION");
        ImGui::SetNextItemWidth(150);
        ImGui::InputFloat("DC guard (Hz)", &c.dc_guard_hz, 100.f, 1000.f, "%.0f");
        { int tl = c.target_looks; ImGui::SetNextItemWidth(150);
          if(ImGui::SliderInt("target looks", &tl, 128, 16384)) c.target_looks = (uint16_t)tl; }
        { int kfs = c.fft_size; ImGui::SetNextItemWidth(150);
          const char* it[] = { "1024","2048","4096","8192","16384","32768" };
          int ki = 3; for(int i=0;i<6;i++) if(kfs == (1024<<i)) ki = i;
          if(ImGui::Combo("segment FFT", &ki, it, 6)) c.fft_size = (uint16_t)(1024<<ki); }

        ImGui::Dummy(ImVec2(0,4));
        if(ImGui::TreeNode("Advanced")){
            ImGui::SetNextItemWidth(150);
            ImGui::InputFloat("c_papr", &c.c_papr, 1.0f, 10.0f, "%.1f");
            ImGui::TreePop();
        }

        ImGui::EndDisabled();

        // 표시 전용 (와이어에 안 올린다 — HOST 소유 설정이 아니라 화면 취향이다)
        ImGui::Dummy(ImVec2(0,6)); ImGui::Separator();
        ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "DISPLAY");
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("polar floor (dB)", &polar_floor_db, -80.f, -10.f, "%.0f");
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("LOB length (km)", &lob_km, 10.f, 500.f, "%.0f");

        ImGui::EndChild();

        // 손을 뗀 프레임에만 한 번 보낸다. HOST 가 거절하면 에코가 안 오고,
        // hold 만료 시 ui = canon 으로 진실에 스냅백한다.
        if(cfg_ready && !ImGui::IsAnyItemActive()){
            clamp_cross_fields(ui);
            if(memcmp(&canon, &ui, sizeof ui) != 0){
                v.df_set_cfg(ui);
                hold_until = now + 1.0;
            }
        }
    }

    // ── 수동 LOB 입력 팝업 ───────────────────────────────────────────────
    // JOIN 은 Central 룸 하나에만 있어 한 기지의 DF 결과만 받는다. 다른 기지의
    // 방위를 손으로 넣으면 와이어 변경 없이 기지간 교차 fix 를 만들 수 있다.
    if(lob_popup){ ImGui::OpenPopup("##df_lob"); lob_popup = false; }
    if(ImGui::BeginPopup("##df_lob")){
        ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "MANUAL LOB");
        ImGui::SetNextItemWidth(200);
        int lob_idx = -1;
        for(int i = 0; i < (int)stn_names.size(); i++)
            if(stn_names[i] == lob_stn){ lob_idx = i; break; }
        if(lob_idx < 0 && !stn_names.empty()){ lob_idx = 0; lob_stn = stn_names[0]; }
        {
            const char* cur = (lob_idx >= 0) ? stn_names[lob_idx].c_str() : "(no station)";
            if(ImGui::BeginCombo("station", cur)){
                for(int i = 0; i < (int)stn_names.size(); i++)
                    if(ImGui::Selectable(stn_names[i].c_str(), lob_idx == i)){
                        lob_idx = i; lob_stn = stn_names[i];
                    }
                ImGui::EndCombo();
            }
        }
        ImGui::SetNextItemWidth(200);
        ImGui::InputFloat("bearing (deg)", &lob_brg, 1.f, 10.f, "%.1f");
        ImGui::BeginDisabled(lob_idx < 0);
        if(ImGui::Button("ADD") && lob_idx >= 0 && lob_idx < (int)stns.size()){
            FFTViewer::DFFix f{};
            f.t_end_ms    = now_ms_local();
            f.bearing_deg = std::fmod(lob_brg + 360.f, 360.f);
            f.station_lat = (float)stns[lob_idx].lat;
            f.station_lon = (float)stns[lob_idx].lon;
            f.kind = 0; f.manual_lob = 1;
            snprintf(f.note, sizeof f.note, "MANUAL %s", stn_names[lob_idx].c_str());
            v.df_push_fix(f);
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if(ImGui::Button("CANCEL")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // Ctrl+C: 선택 행을 탭 구분으로 복사
    if(win_focus && !typing && io.KeyCtrl &&
       ImGui::IsKeyPressed(ImGuiKey_C, false) && !sel.keys.empty()){
        std::string out;
        for(int p = 0; p < (int)vis.size(); p++){
            const int hi = vis[p];
            if(!sel.selected(key_of(hi))) continue;
            const FFTViewer::DFFix& f = v.df_hist_at(hi);
            char ts[16]; hms(f.t_end_ms, ts, sizeof ts);
            char line[256];
            snprintf(line, sizeof line, "%s\tCH%u\t%.4f\t%.1f\t%.1f\t%.2f\t%u/%u\t%s\n",
                     ts, (unsigned)f.dnum, f.cf_mhz, f.bearing_deg, f.snr_db, f.conf_db,
                     (unsigned)f.frames_used, (unsigned)f.frames_discarded, f.note);
            out += line;
        }
        if(!out.empty()) ImGui::SetClipboardText(out.c_str());
    }

    last_vis_n = (int)vis.size();
    ImGui::End();
}
