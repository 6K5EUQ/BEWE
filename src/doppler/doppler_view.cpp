// ── HIST 뷰어 도플러 패널 (GUI 전용) ─────────────────────────────────────────
//
// 우측 자식 패널 + 스펙트로그램 오버레이. 하단이 아니라 우측인 이유: 이미지 세로축이
// 주파수이고 도플러 곡선이 ±100~200 bin 을 지나가므로 세로 해상도를 잃으면 안 된다.
//
// **이미지 위 마우스 바인딩을 하나도 추가하지 않는다.** 좌드래그=줌박스, 우클릭=줌백,
// Ctrl+우드래그=측정영역, 휠=시간줌, Ctrl+휠=주파수줌, Home/Del/Ctrl+Z 가 이미 다
// 잡혀 있다. 조작은 전부 패널에서 하고 이미지 변경은 읽기전용 렌더뿐이라
// "드래그했더니 트랙이 아니라 측정영역이 생겼다" 류 버그가 통째로 사라진다.
#include "doppler_view.hpp"
#include "doppler_scan.hpp"
#include "../hist_reader.hpp"
#include "../modules/modview.hpp"
#include "../bewe_paths.hpp"
#include "../kst_time.hpp"
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <ctime>

namespace DopplerView {

namespace {

bool  g_open = false;
bool  g_show_rejected = false;   // 위성이 아니라고 판정된 트랙까지 표에 낼까
int   g_sel_track = -1;
int   g_sort_col  = -1;
bool  g_sort_asc  = true;
modview::Selection g_sel;
DopplerMatch::Params g_mp;      // 검색 옵션 (Starlink 포함 여부 등)
std::string g_want_tle;         // Central 에 요청해 놓은 파일명 (비면 없음)
double g_want_since = 0.0;      // 그 요청을 건 시각 (타임아웃 판정용)
bool   g_pending_scan = false;  // 열렸으니 스캔해야 한다 — 실행은 draw_panel 이 한다
bool   g_auto_selected = false; // 이 결과에서 자동선택을 이미 했나 (해제 유지용)
// 방향키로 옮긴 선택을 표가 따라 스크롤하게 한다 (표는 6행만 보인다).
bool   g_scroll_to_sel = false;
// 탐지 민감도. 파일을 바꿔도 유지한다 — 운용자가 고른 작업 방식이지 파일 속성이 아니다.
Doppler::Sensitivity g_sens = Doppler::Sensitivity::Normal;

// 분석 결과가 어느 녹화 것인지. 파일을 열 때마다 비우므로 캐시가 아니라 소유 표시다.
std::string g_owner_path;
uint64_t    g_owner_rows = 0;

std::vector<Doppler::Candidate> g_tracks;
DopplerMatch::Result            g_match;
bool                            g_have_match = false;
uint32_t                        g_match_of = 0xFFFFFFFFu;

// 오버레이 라벨용 트랙별 1위 후보명. 매 프레임 match_of() 로 Result 전체(후보 20개)를
// 복사하면 낭비라, 워커가 새 스냅샷을 낼 때만 이름을 받아 둔다. 스캔 중에는 매칭이
// 트랙보다 늦게 채워지므로 빈 칸이 남을 수 있고, 다음 스냅샷에서 메워진다.
std::vector<std::string> g_top_name;    // g_tracks 와 같은 길이

std::string tle_dir(){ return BEWEPaths::assets_dir() + "/tle"; }

std::string hhmmss(double t){
    time_t s = (time_t)t; struct tm tv{}; KST::to_tm(s, tv);
    char b[16]; snprintf(b, sizeof b, "%02d:%02d:%02d", tv.tm_hour, tv.tm_min, tv.tm_sec);
    return b;
}

// 좌측정렬 셀. 열 폭을 넘치면 잘라 쓰고 hover 로 전체 이름을 준다 (이름은 데이터다).
// modview::cell_left 를 안 고치는 이유: acars/ais/wifi 가 같이 쓰는 헬퍼다.
void cell_name(const char* s){
    const float av = ImGui::GetContentRegionAvail().x;
    if(ImGui::CalcTextSize(s).x <= av){ ImGui::TextUnformatted(s); return; }
    // 말줄임 없이 (non-ASCII 금지) 폭에 맞게 자른다.
    const int n = (int)strlen(s);
    int keep = n;
    while(keep > 1 && ImGui::CalcTextSize(s, s+keep).x > av) keep--;
    ImGui::TextUnformatted(s, s+keep);
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("%s", s);
}

void refresh_from_worker(){
    std::vector<Doppler::Candidate> t;
    if(DopplerScan::results(t)){
        g_tracks = std::move(t);
        // 오버레이 라벨 캐시 갱신. 위성으로 판정된 트랙만 이름을 붙인다.
        g_top_name.assign(g_tracks.size(), std::string());
        for(size_t i = 0; i < g_tracks.size(); i++)
            if(g_tracks[i].score > 0.0f)
                DopplerScan::top_name_of(g_tracks[i].id, g_top_name[i]);
    }
    if(g_sel_track >= 0 && g_sel_track < (int)g_tracks.size()){
        const uint32_t id = g_tracks[g_sel_track].id;
        if(id != g_match_of){
            g_have_match = DopplerScan::match_of(id, g_match);
            g_match_of = id;
        }
    }
}

// 첫 위성 트랙을 자동 선택. 위성이 하나도 없으면 아무것도 고르지 않는다 —
// 표가 위성만 싣는데 거부 트랙을 골라 두면 표에 없는 행이 선택된 채로 아래
// 요약이 그려진다 ("Show non-satellites" 를 켜면 그때 손으로 고를 수 있다).
//
// **결과 한 벌에 딱 한 번만 고른다.** 매 프레임 돌면 사용자가 행을 다시 눌러
// 선택을 푼 순간 곧바로 되살아나 해제가 불가능해진다.
void auto_select(){
    if(g_auto_selected || g_sel_track >= 0 || g_tracks.empty()) return;
    g_auto_selected = true;
    for(size_t i = 0; i < g_tracks.size(); i++)
        if(g_tracks[i].score > 0.0f){ g_sel_track = (int)i; return; }
    if(g_show_rejected) g_sel_track = 0;
}

} // anon

// 이 녹화가 필요로 하는 원소 파일명. 이미 로컬에 있으면 빈 문자열.
std::string needed_tle_name(const HistReader& R){
    if(!R.is_open()) return "";
    const time_t t = (time_t)R.hdr().start_utc_unix;
    struct tm g{}; gmtime_r(&t, &g);
    // Starlink 를 포함하려면 all_ 계열이 필요하다 (leo_ 는 저장 시점에 걸러져 있다).
    const char* pre = g_mp.include_starlink ? "all" : "leo";
    char nm[32];
    snprintf(nm, sizeof nm, "%s_%04d%02d%02d.txt", pre, g.tm_year+1900, g.tm_mon+1, g.tm_mday);
    const std::string p = tle_dir() + "/archive/" + nm;
    struct stat st{};
    if(stat(p.c_str(), &st) == 0 && st.st_size > 0) return "";
    return nm;
}

// Central 이 그 날짜를 안 가진 경우 **아무 응답도 오지 않는다** (central_server.cpp 는
// file not found 를 자기 로그에만 찍는다). 그래서 도착 통보만 기다리면 영원히 멈춘다.
// 시간이 지나면 포기하고 로컬에 있는 가장 가까운 날짜로 돌린다 — 나이는 바에 뜬다.
static constexpr double kTleWaitSec = 20.0;
static bool tle_wait_expired(){
    return !g_want_tle.empty() && g_want_since > 0.0
        && (ImGui::GetTime() - g_want_since) > kTleWaitSec;
}
bool tle_pending(){ return !g_want_tle.empty(); }
void note_tle_arrived(const std::string& filename){
    if(!g_want_tle.empty() && filename == g_want_tle){ g_want_tle.clear(); g_want_since = 0.0; }
}

bool panel_open(){ return g_open; }

// 상하 방향키를 이 패널이 가져갈 상황인가 — 뷰어의 주파수 팬과 겹치므로 뷰어가
// 물어본다. 트랙을 고른 상태에서만 가져간다 (안 골랐으면 팬이 정상 동작).
bool wants_updown(){
    return g_open && g_sel_track >= 0 && !g_tracks.empty();
}

bool toolbar_button(const HistReader& R){
    // 좌표계 없는 파일(v2 헤더/미설정 기지)은 look-angle 을 못 구한다.
    const bool ok = R.is_open() && R.has_station_pos();

    // 라벨은 **항상 "SAT SCAN"** 이다. 진행 상태를 여기 싣지 않는다 — 버튼이
    // 스캔 단계마다 이름이 바뀌면(CALIB/MATCH…) 딴 버튼처럼 보이고 폭도 흔들린다.
    // 진행률은 패널 안 진행 바가 이미 전담한다.
    const char* label = "SAT SCAN";

    if(!ok) ImGui::BeginDisabled();
    if(g_open) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f,0.42f,0.22f,1.0f));
    const bool hit = ImGui::Button(label);
    if(g_open) ImGui::PopStyleColor();
    if(!ok) ImGui::EndDisabled();
    return hit && ok;
}

// Central 요청 훅 — ui.cpp 가 NetClient 를 물려 준다 (doppler 는 net 을 모른다).
static std::function<bool(const std::string&)> g_req_tle;
void set_tle_requester(std::function<bool(const std::string&)> fn){ g_req_tle = std::move(fn); }

// 필요한 원소가 로컬에 없으면 Central 에 한 번 요청한다. true = 요청함(대기).
static bool ensure_tle(const HistReader& R){
    const std::string nm = needed_tle_name(R);
    if(nm.empty()){ g_want_tle.clear(); g_want_since = 0.0; return false; }
    if(g_want_tle == nm) return true;              // 이미 대기 중
    if(g_req_tle && g_req_tle(nm)){
        g_want_tle = nm; g_want_since = ImGui::GetTime();
        return true;
    }
    return false;
}

// 결과가 어느 녹화 것인지 기록해 둔다 (오버레이가 남의 트랙을 그리지 않게).
static void claim(const HistReader& R){
    g_owner_path = R.is_open() ? R.path() : std::string();
    g_owner_rows = R.is_open() ? R.num_rows() : 0;
}
static void wipe(){
    g_sel_track = -1; g_sel.clear();
    g_have_match = false; g_match_of = 0xFFFFFFFFu;
    g_tracks.clear(); g_top_name.clear();
    g_want_tle.clear(); g_want_since = 0.0;
    g_pending_scan = false;
    g_auto_selected = false;
}

// HIST 파일을 열면 **언제나** 이전 분석을 버린다. 같은 파일을 다시 열어도 마찬가지다 —
// 남겨 두면 그 결과가 어느 패스·어느 민감도로 나온 것인지 알 수 없어, SAT SCAN 을
// 눌렀는데 옛 표가 뜨고 스캔은 안 도는 상태로 되돌아간다.
void note_file_changed(const HistReader& R){
    (void)R;
    DopplerScan::shutdown();     // 옛 리더를 문 워커부터 세운다
    wipe();
    g_owner_path.clear(); g_owner_rows = 0;
}

void toggle(const HistReader& R, const Doppler::ExtractParams& P){
    if(g_open){ g_open = false; return; }
    g_open = true;
    if(!R.is_open() || !R.has_station_pos()) return;
    // **캐시 재사용 없음 — 열면 언제나 새로 스캔한다.**
    // 결과를 아껴 두는 최적화는 "버튼을 눌렀는데 아무 일도 안 일어난다" 로 계속
    // 되돌아왔다 (남아 있던 결과가 어느 패스 것인지에 따라 표가 반쪽이 되거나
    // 통째로 옛것이었다). 스캔이 2~8초라 아껴서 얻는 것보다 잃는 신뢰가 크다.
    wipe();
    claim(R);
    // LIVE 는 계속 자라서 선해제도 안 되고 행수가 스캔 중에 바뀐다 — 자동 스캔 대상 아님.
    if(R.is_live()) return;
    // 원소부터 확보한다. 요청을 보냈으면 도착을 기다렸다 돌고(낡은 원소로 돌리면
    // 순위가 틀린다 — 실측 38일 낡음에서 6건 중 5건 오답 1위), 보낼 수단이 없으면
    // 있는 원소로 바로 돈다. 어느 쪽이든 **스캔은 draw_panel 이 반드시 건다.**
    ensure_tle(R);
    g_pending_scan = (g_want_tle.empty());
}

void start_refine(const HistReader& R, uint32_t row_lo, uint32_t row_hi,
                  uint32_t lin_lo, uint32_t lin_hi, const Doppler::ExtractParams& P){
    if(!R.is_open() || !R.has_station_pos()) return;
    g_open = true;
    wipe();
    claim(R);
    DopplerScan::start_refine(R, row_lo, row_hi, lin_lo, lin_hi, P, g_mp, tle_dir());
}

float draw_panel(const HistReader& R, float h){
    if(!g_open) return 0.0f;
    const float W = 460.0f;
    refresh_from_worker();
    auto_select();

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.07f,0.08f,0.10f,1.0f));
    ImGui::BeginChild("##dop_panel", ImVec2(W, h), true);
    ImGui::PopStyleColor();

    const DopplerScan::Status st = DopplerScan::status();

    // 원소가 도착했으면(파일이 생겼으면) 그때 스캔을 건다. Central 이 그 날짜를
    // 안 가진 경우엔 응답이 아예 없으므로, 기다리다 지치면 로컬 최근사로 돌린다 —
    // 나이는 아래 바에 뜬다.
    if(!g_want_tle.empty() && (needed_tle_name(R).empty() || tle_wait_expired())){
        g_want_tle.clear(); g_want_since = 0.0;
        g_pending_scan = true;
    }
    // 열자마자 한 번은 반드시 돈다. 예전에는 toggle() 이 스캔을 걸지 **못한** 경우
    // (요청을 못 보냈다 = g_want_tle 이 빈 채로 남았다) 위 블록의 조건에도 안 걸려
    // 아무도 스캔을 시작하지 않았고, 사용자가 RESCAN 을 손으로 눌러야 했다.
    // 시작 책임을 여기 한 곳으로 모은다 — toggle 은 의사만 세우고 실행은 패널이 한다.
    if(g_pending_scan && !DopplerScan::busy() && !R.is_live()){
        g_pending_scan = false;
        g_auto_selected = false;
        Doppler::ExtractParams EP; EP.apply(g_sens);
        DopplerScan::start_full(R, EP, g_mp, tle_dir());
    }

    // ── 헤더 (modview::header_bar 의 겉모습만 재현) ──────────────────────
    // header_bar 자체는 못 쓴다 — 시그니처가 모듈 프레임워크(bewe_mod_recv/hist_mode)에
    // 묶여 있어 여기서 의미 없는 RECV/DB/PLAYBACK 버튼을 그린다.
    {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f,0.12f,0.16f,1.0f));
        ImGui::BeginChild("##dop_hdr", ImVec2(0, 30), false);
        ImGui::SetCursorPos(ImVec2(12, 6));
        // 세는 대상은 위성이다. 잡힌 트랙 수를 내면 지상 신호까지 성과처럼 보인다.
        { int n_sat = 0;
          for(const auto& t : g_tracks) if(t.score > 0.0f) n_sat++;
          ImGui::Text(n_sat==1 ? "%d satellite" : "%d satellites", n_sat); }

        const bool running = (st.st == DopplerScan::State::Running);
        const char* rl = running ? "CANCEL" : "RESCAN";
        const float pad = ImGui::GetStyle().FramePadding.x*2;
        const float bw  = ImGui::CalcTextSize(rl).x + pad;
        // 우측 정렬 기준은 **자식 창의 실제 폭**이다. 바깥 W(460) 로 잡으면 자식의
        // 좌우 패딩(WindowPadding*2)만큼 넘쳐 마지막 버튼이 잘린다 — 실제로 RESCAN
        // 오른쪽이 잘려 있었다.
        const float availW = ImGui::GetWindowWidth();
        const float rmargin = 8.0f;
        if(running){
            ImGui::SameLine(availW - bw - rmargin);
            if(ImGui::Button(rl)) DopplerScan::cancel();
        } else {
            // 버스트(간헐 송신) 2차 패스는 명시 실행이다. 자동으로 돌리면 스캔이 두 배
            // 걸리는데, 대부분의 녹화에는 버스트 위성이 없다.
            const float bb = ImGui::CalcTextSize("BURST").x + pad;
            // 민감도 — 약한 연속신호가 기본값에서 안 잡히는 경우가 있다. 고르면
            // 그 자리에서 다시 스캔한다 (골라 놓고 RESCAN 을 또 누르게 하지 않는다).
            const float sw = 74.0f;
            ImGui::SameLine(availW - bw - bb - sw - rmargin - 16.0f);
            ImGui::SetNextItemWidth(sw);
            static const char* SENS[3] = {"LOOSE", "NORMAL", "STRICT"};
            if(ImGui::BeginCombo("##dop_sens", SENS[(int)g_sens])){
                for(int k = 0; k < 3; k++)
                    if(ImGui::Selectable(SENS[k], (int)g_sens == k)){
                        g_sens = (Doppler::Sensitivity)k;
                        g_auto_selected = false;
                        Doppler::ExtractParams P; P.apply(g_sens);
                        DopplerScan::start_full(R, P, g_mp, tle_dir());
                    }
                ImGui::EndCombo();
            }
            ImGui::SameLine(availW - bw - bb - rmargin - 8.0f);
            // 새 결과가 나오면 자동선택을 한 번 다시 허용한다.
            if(ImGui::Button("BURST")){
                g_auto_selected = false;
                DopplerScan::start_burst(R, g_mp, tle_dir(), g_sens);
            }
            ImGui::SameLine(availW - bw - rmargin);
            if(ImGui::Button(rl)){
                g_auto_selected = false;
                Doppler::ExtractParams P; P.apply(g_sens);
                DopplerScan::start_full(R, P, g_mp, tle_dir());
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();      // BeginChild 가 ChildBg 를 소비하므로 여기서 해제
    }

    // ── 진행 바 ─────────────────────────────────────────────────────────
    // 원소 수신과 스캔을 한 줄로 잇는다. 사용자에게는 "위성 찾는 중" 하나의 일이라
    // 단계가 바뀌었다고 바가 사라졌다 다시 나타나면 안 된다.
    // 다 끝난 뒤에는 원소 나이를 같은 자리에 남긴다 — 순위가 얼마나 믿을 만한지는
    // 그 값 하나로 갈린다 (실측: 38일 낡음에서 6건 중 5건이 오답 1위).
    {
        const bool fetching = !g_want_tle.empty();
        const bool running  = (st.st == DopplerScan::State::Running);
        if(fetching || running){
            char ov[64];
            float frac;
            if(fetching){
                // 수신은 진행률을 모른다 (Central 이 청크만 흘린다). 앞 1/3 을
                // 불확정 구간으로 쓰되 시간에 따라 차오르게 해 멈춘 것처럼 안 보이게.
                const double el = (g_want_since > 0) ? (ImGui::GetTime() - g_want_since) : 0.0;
                frac = (float)std::min(0.30, el / kTleWaitSec * 0.30);
                snprintf(ov, sizeof ov, "Getting orbit data %d%%", (int)(frac*100));
            } else {
                frac = 0.30f + st.progress*0.70f;
                snprintf(ov, sizeof ov, "%s %d%%",
                         st.stage[0] ? st.stage : "Scanning", (int)(frac*100));
            }
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.30f,0.62f,0.85f,1.0f));
            ImGui::ProgressBar(frac, ImVec2(-1, 16), ov);
            ImGui::PopStyleColor();
        } else if(g_have_match){
            const double age = g_match.tle_age_days;
            ImVec4 col = (age > 10.0) ? ImVec4(0.95f,0.35f,0.30f,1)
                       : (age > 3.0)  ? ImVec4(0.95f,0.72f,0.25f,1)
                                      : ImVec4(0.45f,0.75f,0.50f,1);
            char ov[64];
            if(age < 1.5) snprintf(ov, sizeof ov, "Orbit data: same day  (%d satellites)",
                                   g_match.n_loaded);
            else          snprintf(ov, sizeof ov, "Orbit data: %.0f days off  (%d satellites)",
                                   age, g_match.n_loaded);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, col);
            ImGui::ProgressBar(1.0f, ImVec2(-1, 16), ov);
            ImGui::PopStyleColor();
        }
    }

    // 나이/진행 상태는 위 바 하나가 다 말한다 — 같은 내용을 문장으로 또 쓰지 않는다.
    // Starlink 는 항상 후보에서 뺀다 — 다운링크가 Ku 밴드(10.7~12.7 GHz)라 이 플랫폼이
    // 보는 대역(126~930 MHz)엔 안 나오는데, 카탈로그의 67% 를 차지하며 같은 셸 수십 개가
    // 분 단위로 지나가 분리도를 1 근처로 깎는다 (실측: 제외 시 sep 15.24 -> 57.17).
    // g_mp.include_starlink 는 기본 false 이고 이제 켤 수단이 없다.
    if(st.st == DopplerScan::State::Failed && !st.err.empty())
        ImGui::TextColored(ImVec4(0.95f,0.4f,0.35f,1), "%s", st.err.c_str());
    if(R.is_live())
        ImGui::TextDisabled("Recording still running - box a signal with Ctrl+drag, then REFINE");

    // ── 트랙 목록 ────────────────────────────────────────────────────────
    // 이건 위성을 찾는 기능이다. 지상 신호로 표를 채우면 정작 위성이 묻힌다 (실측:
    // 145MHz 1시간 파일에서 85건이 잡혔는데 전부 지상이었다). 기본은 위성만 보이고,
    // "왜 이 신호가 안 잡혔나" 를 따질 때만 토글로 나머지를 꺼내 본다.
    std::vector<int> vis_trk;
    for(size_t i = 0; i < g_tracks.size(); i++)
        if(g_show_rejected || g_tracks[i].score > 0.0f) vis_trk.push_back((int)i);
    {
        int n_sat = 0;
        for(const auto& t : g_tracks) if(t.score > 0.0f) n_sat++;
        const int n_rej = (int)g_tracks.size() - n_sat;
        if(n_rej > 0){
            if(ImGui::Checkbox("Show non-satellites", &g_show_rejected)){
                // 끄면서 선택이 표 밖으로 나가면 놓아 준다 — 안 그러면 보이지도 않는
                // 행의 요약이 아래에 계속 남는다.
                if(!g_show_rejected && g_sel_track >= 0
                   && g_sel_track < (int)g_tracks.size()
                   && g_tracks[g_sel_track].score <= 0.0f){
                    g_sel_track = -1; g_sel.clear();
                    g_have_match = false; g_match_of = 0xFFFFFFFFu;
                }
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(%d)", n_rej);
        }
    }
    // 한 건이어도 표를 낸다 — 그 한 줄이 시각·길이·주파수를 담고 있고, 행을 눌러야
    // 이미지에 영역이 그려진다. (>1 조건이던 시절엔 1건일 때 표가 통째로 사라졌다.)
    if(!vis_trk.empty()){
        // ── 방향키로 행 이동 ────────────────────────────────────────────
        // 표에 보이는 순서(vis_trk) 기준이다. g_tracks 인덱스로 움직이면 숨겨진
        // 행을 밟아 선택이 표 밖으로 나간다.
        if(g_sel_track >= 0 && !ImGui::IsAnyItemActive()){
            int cur = -1;
            for(size_t k = 0; k < vis_trk.size(); k++)
                if(vis_trk[k] == g_sel_track){ cur = (int)k; break; }
            if(cur >= 0){
                int nxt = cur;
                if(ImGui::IsKeyPressed(ImGuiKey_DownArrow)) nxt = cur + 1;
                if(ImGui::IsKeyPressed(ImGuiKey_UpArrow))   nxt = cur - 1;
                if(nxt != cur && nxt >= 0 && nxt < (int)vis_trk.size()){
                    g_sel_track = vis_trk[nxt];
                    g_match_of = 0xFFFFFFFFu;
                    g_scroll_to_sel = true;
                }
            }
        }
        const float th = ImGui::GetTextLineHeightWithSpacing()*std::min<size_t>(6, vis_trk.size()) + 28;
        if(ImGui::BeginTable("##dop_trk", 5,
               ImGuiTableFlags_Borders|ImGuiTableFlags_RowBg|ImGuiTableFlags_ScrollY,
               ImVec2(0, th))){
            ImGui::TableSetupScrollFreeze(0,1);
            ImGui::TableSetupColumn("#",          ImGuiTableColumnFlags_WidthFixed, 30.0f);
            ImGui::TableSetupColumn("Time",       ImGuiTableColumnFlags_WidthFixed, 70.0f);
            ImGui::TableSetupColumn("Length",     ImGuiTableColumnFlags_WidthFixed, 66.0f);
            ImGui::TableSetupColumn("Frequency",  ImGuiTableColumnFlags_WidthStretch);
            // "continuous" 가 잘리지 않을 만큼. modview::cell 이 중앙정렬이라
            // 좁으면 양끝이 먹힌다.
            ImGui::TableSetupColumn("Type",       ImGuiTableColumnFlags_WidthFixed, 88.0f);
            ImGui::TableHeadersRow();
            for(size_t k = 0; k < vis_trk.size(); k++){
                const int i = vis_trk[k];
                const Doppler::Candidate& c = g_tracks[i];
                ImGui::TableNextRow();
                // 번호는 표에 보이는 순번이다 — 오버레이 박스 라벨도 같은 번호를 쓴다.
                char idx[8]; snprintf(idx, sizeof idx, "%zu", k+1);
                // 같은 행을 다시 누르면 선택을 푼다 — 이미지의 강조(굵은 테두리 +
                // 관측/적합 곡선)를 끄는 수단이 이것뿐이다.
                if(modview::row_col0(i, i == g_sel_track, idx)){
                    if(i == g_sel_track){
                        g_sel_track = -1; g_sel.clear();
                        g_have_match = false;
                    } else {
                        g_sel_track = i;
                    }
                    g_match_of = 0xFFFFFFFFu;
                }
                // 방향키로 옮겨 왔으면 그 행이 보이도록 표를 따라 스크롤한다.
                if(g_scroll_to_sel && i == g_sel_track) ImGui::SetScrollHereY(0.5f);
                ImGui::TableSetColumnIndex(1); modview::cell(hhmmss(c.t_start_utc).c_str());
                char b[32];
                ImGui::TableSetColumnIndex(2);
                { const double d = c.t_end_utc-c.t_start_utc;
                  if(d >= 60) snprintf(b,sizeof b,"%dm %02ds",(int)(d/60),(int)d%60);
                  else        snprintf(b,sizeof b,"%ds",(int)d);
                  modview::cell(b); }
                ImGui::TableSetColumnIndex(3);
                snprintf(b,sizeof b,"%.3f MHz", c.fit.f_center_hz/1e6); modview::cell(b);
                ImGui::TableSetColumnIndex(4);
                // 어느 패스가 찾았나. 두 패스가 같은 신호를 잡으면 중복제거가 점수
                // 높은 쪽만 남기므로, 살아남은 트랙의 플래그가 곧 "더 잘 맞은 쪽"이다.
                // 위성이 아니면 종류를 따질 게 없어 그대로 no 를 쓴다.
                const char* ty = (c.score <= 0.0f) ? "no"
                               : (c.is_burst ? "burst" : "continuous");
                modview::cell(ty, c.score > 0.0f ? ImVec4(0.55f,0.85f,0.55f,1)
                                                 : ImVec4(0.6f,0.6f,0.6f,1));
            }
            ImGui::EndTable();
        }
        g_scroll_to_sel = false;
    }

    // ── 선택 트랙 요약 ───────────────────────────────────────────────────
    if(g_sel_track >= 0 && g_sel_track < (int)g_tracks.size()){
        const Doppler::Candidate& c = g_tracks[g_sel_track];
        ImGui::Separator();
        ImGui::Text("Signal     : %.3f MHz", c.fit.f_center_hz/1e6);
        ImGui::Text("Seen       : %s to %s", hhmmss(c.t_start_utc).c_str(),
                    hhmmss(c.t_end_utc).c_str());
        // 스윙은 "관측 중 주파수가 얼마나 흘렀나" 로 풀어 쓴다. 그게 위성 판별의
        // 근거이므로 값 자체는 남긴다 (설명문이 아니라 정보다).
        ImGui::Text("Drifted    : %.1f kHz down", std::fabs(c.fit.half_swing_hz)*2.0/1000.0);
        ImGui::Text("Closest at : %s", hhmmss(c.fit.t_tca_utc).c_str());
        if(!c.reject_reason.empty())
            ImGui::TextColored(ImVec4(0.9f,0.6f,0.3f,1), "Not a satellite - %s",
                               c.reject_reason.c_str());
    }

    // ── 후보 표 ──────────────────────────────────────────────────────────
    if(g_have_match && !g_match.cands.empty()){
        ImGui::Separator();
        // 이 관측의 측정잡음 = 트랙 자신의 곡선 적합 잔차. FIT 등급의 기준선이다.
        double sel_meas = 0.0;
        if(g_sel_track >= 0 && g_sel_track < (int)g_tracks.size()
           && g_tracks[g_sel_track].fit.valid)
            sel_meas = g_tracks[g_sel_track].fit.rms_resid_hz;
        std::vector<int> vis;
        for(size_t i = 0; i < g_match.cands.size(); i++) vis.push_back((int)i);
        modview::sort_vis(vis, g_sort_col, g_sort_asc, [&](int col, int a, int b)->int{
            const auto& A = g_match.cands[a]; const auto& B = g_match.cands[b];
            switch(col){
                case 1: return A.name.compare(B.name);
                case 2: return A.norad<B.norad?-1:(A.norad>B.norad?1:0);
                case 3: return A.rms_hz<B.rms_hz?-1:(A.rms_hz>B.rms_hz?1:0);
                case 4: return A.max_el_deg<B.max_el_deg?-1:(A.max_el_deg>B.max_el_deg?1:0);
                case 5: return A.az_tca_deg<B.az_tca_deg?-1:(A.az_tca_deg>B.az_tca_deg?1:0);
                default: return 0;
            }
        });
        const float rest = ImGui::GetContentRegionAvail().y - 100.0f;
        if(ImGui::BeginTable("##dop_cand", 6,
               ImGuiTableFlags_Borders|ImGuiTableFlags_RowBg|ImGuiTableFlags_ScrollY,
               ImVec2(0, rest > 60 ? rest : 60))){
            ImGui::TableSetupScrollFreeze(0,1);
            // 폭을 명시하지 않으면 6열이 균등분배돼 순위 숫자가 위성 이름과 같은 폭을
            // 먹는다. 숫자/등급/방위는 내용에 맞춰 고정하고 이름이 나머지를 다 갖는다.
            ImGui::TableSetupColumn("#",         ImGuiTableColumnFlags_WidthFixed, 30.0f);
            ImGui::TableSetupColumn("SATELLITE", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("ID",        ImGuiTableColumnFlags_WidthFixed, 52.0f);
            ImGui::TableSetupColumn("FIT",       ImGuiTableColumnFlags_WidthFixed, 46.0f);
            ImGui::TableSetupColumn("HEIGHT",    ImGuiTableColumnFlags_WidthFixed, 56.0f);
            ImGui::TableSetupColumn("DIR",       ImGuiTableColumnFlags_WidthFixed, 40.0f);
            modview::sortable_headers(6, g_sort_col, g_sort_asc, 1);
            for(size_t k = 0; k < vis.size(); k++){
                const DopplerMatch::Cand& c = g_match.cands[vis[k]];
                char key[24]; snprintf(key, sizeof key, "%d", c.norad);
                ImGui::TableNextRow();
                char rank[8]; snprintf(rank, sizeof rank, "%d", vis[k]+1);
                if(modview::row_col0(vis[k], g_sel.selected(key), rank)){
                    const bool ctrl = ImGui::GetIO().KeyCtrl, shift = ImGui::GetIO().KeyShift;
                    g_sel.click(key, (int)k,
                        [&](int i)->std::string{
                            char t[24]; snprintf(t,sizeof t,"%d", g_match.cands[vis[i]].norad);
                            return t; },
                        (int)vis.size(), ctrl, shift);
                    // 더블클릭 = 확정. 단일클릭은 이미 modview 다중선택 관례가 소유한다.
                    if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) &&
                       g_sel_track >= 0 && g_sel_track < (int)g_tracks.size()){
                        DopplerMatch::Obs obs;
                        obs.lat_deg = R.station_lat();
                        obs.lon_east_deg = R.station_lon_east();
                        obs.station.assign(R.hdr().station_name,
                            strnlen(R.hdr().station_name, sizeof(R.hdr().station_name)));
                        DopplerMatch::archive_confirm(g_tracks[g_sel_track], obs, c, vis[k]+1);
                    }
                }
                char b[32];
                ImGui::TableSetColumnIndex(1); cell_name(c.name.c_str());
                ImGui::TableSetColumnIndex(2); snprintf(b,sizeof b,"%d",c.norad); modview::cell(b);
                // FIT: 잔차를 이 관측의 측정잡음과 견준 말. 절대 Hz 는 대역마다 달라
                // 비교가 안 되므로 숫자 대신 등급으로 준다 (상세창에 원값이 있다).
                ImGui::TableSetColumnIndex(3);
                { const double meas = (sel_meas > 0) ? sel_meas : c.rms_hz;
                  const double r = c.rms_hz / std::max(1.0, meas);
                  const char*  w = (r < 1.5) ? "BEST" : (r < 3.0) ? "GOOD" : "WEAK";
                  modview::cell(w, r < 1.5 ? ImVec4(0.55f,0.85f,0.55f,1)
                                : r < 3.0 ? ImVec4(0.85f,0.85f,0.60f,1)
                                          : ImVec4(0.75f,0.60f,0.55f,1)); }
                // 하늘에서 얼마나 높이 떴나 — 90도가 머리 위.
                ImGui::TableSetColumnIndex(4);
                snprintf(b,sizeof b,"%.0f\xC2\xB0",c.max_el_deg); modview::cell(b);
                // 방위를 나침반 글자로. 각도는 상세창에 있다.
                ImGui::TableSetColumnIndex(5);
                { static const char* C16[16]={"N","NNE","NE","ENE","E","ESE","SE","SSE",
                                              "S","SSW","SW","WSW","W","WNW","NW","NNW"};
                  int q=(int)((c.az_tca_deg+11.25)/22.5)%16; if(q<0)q+=16;
                  modview::cell(C16[q]); }
            }
            ImGui::EndTable();
        }

        // ── 상세 ─────────────────────────────────────────────────────────
        int det = -1;
        if(g_sel.has_focus)
            for(size_t i = 0; i < g_match.cands.size(); i++){
                char k[24]; snprintf(k,sizeof k,"%d",g_match.cands[i].norad);
                if(g_sel.focus == k){ det = (int)i; break; }
            }
        if(det >= 0){
            const DopplerMatch::Cand& c = g_match.cands[det];
            modview::detail_begin("dop", 0, 0, 92);
            const ImVec4 vc(0.80f,0.86f,0.95f,1);
            char b[48];
            snprintf(b,sizeof b,"%d",c.norad);               modview::kv("Catalog no",b,vc,false);
            snprintf(b,sizeof b,"%.0f km",c.alt_km);         modview::kv("Orbit height",b,vc);
            snprintf(b,sizeof b,"%.1f\xC2\xB0",c.incl_deg);  modview::kv("Orbit tilt",b,vc);
            snprintf(b,sizeof b,"%.4f MHz",c.f0_fit_hz/1e6); modview::kv("Transmits at",b,vc,false);
            snprintf(b,sizeof b,"%.0f km",c.range_tca_km);   modview::kv("Distance",b,vc);
            snprintf(b,sizeof b,"%.0f\xC2\xB0 up, %.0f\xC2\xB0 az",
                     c.el_tca_deg, c.az_tca_deg);            modview::kv("Position",b,vc);
            snprintf(b,sizeof b,"%.0f Hz",c.rms_hz);         modview::kv("Curve error",b,vc,false);
            snprintf(b,sizeof b,"%+.0f s",c.dtca_s);         modview::kv("Timing off by",b,vc);
            snprintf(b,sizeof b,"%.0f%% of pass",c.cov*100.0); modview::kv("Visible",b,vc);
            modview::detail_end();
        }
    } else if(st.st == DopplerScan::State::Done && g_tracks.empty()){
        ImGui::Separator();
        ImGui::TextDisabled("no tracks");
    }

    ImGui::EndChild();
    return W;
}

void draw_overlay(ImDrawList* dl, const HistReader& R,
                  const std::function<float(double)>& row_to_px,
                  const std::function<float(double)>& lin_to_py){
    if(!g_open || g_tracks.empty()) return;
    const double rr = (R.hdr().row_rate_hz > 0.0f) ? (double)R.hdr().row_rate_hz : 1.0;
    const double t0 = (double)R.hdr().start_utc_unix;

    // ── 선택 트랙의 박스 ────────────────────────────────────────────────
    // 줌아웃하면 트랙 선이 1px 밑으로 눌려 안 보인다. 박스는 "이 근처에 뭐가 있다"를
    // 한눈에 준다. 색은 위성=노랑 / 위성아님=회색 — 빨강은 Ctrl+우드래그 측정영역이
    // 이미 쓰고 있어 사용자가 친 박스와 헷갈리면 안 된다.
    const ImVec2 clip0 = dl->GetClipRectMin();

    auto draw_one = [&](size_t i){
        const Doppler::Candidate& b = g_tracks[i];
        double fmin = b.pts[0].f_hz, fmax = fmin;
        for(const Doppler::TrackPoint& p : b.pts){
            if(p.f_hz < fmin) fmin = p.f_hz;
            if(p.f_hz > fmax) fmax = p.f_hz;
        }
        float x0 = row_to_px((b.t_start_utc - t0)*rr);
        float x1 = row_to_px((b.t_end_utc   - t0)*rr);
        float ylo = lin_to_py(R.linear_of_freq_hz(fmin));
        float yhi = lin_to_py(R.linear_of_freq_hz(fmax));
        if(x1 < x0)  std::swap(x0, x1);
        if(yhi > ylo) std::swap(yhi, ylo);
        // 화면 여유 — 줌아웃에서 박스가 선에 딱 붙어 안 보이는 걸 막는다.
        const float pad = 3.0f;
        x0 -= pad; x1 += pad; yhi -= pad; ylo += pad;
        // 아주 작게 눌려도 시인성이 남게 최소 크기를 준다.
        if(x1 - x0 < 6.0f){ const float m=(x0+x1)*0.5f; x0=m-3.0f; x1=m+3.0f; }
        if(ylo - yhi < 6.0f){ const float m=(yhi+ylo)*0.5f; yhi=m-3.0f; ylo=m+3.0f; }

        const bool sel = ((int)i == g_sel_track);
        // 위성은 노랑, 위성 아님은 회색. 같은 노랑으로 그리면 "Show non-satellites"
        // 를 켠 순간 지상 신호가 위성처럼 보인다. (빨강은 Ctrl+우드래그 측정영역이
        // 이미 쓰고 있어 못 쓴다 — 사용자가 친 박스와 헷갈리면 안 된다.)
        const bool is_sat = (b.score > 0.0f);
        const ImU32 fill = is_sat ? (sel ? IM_COL32(255,200,60,46) : IM_COL32(255,200,60,26))
                                  : (sel ? IM_COL32(170,175,185,42) : IM_COL32(170,175,185,22));
        const ImU32 line = is_sat ? IM_COL32(255,200,60, sel ? 255 : 190)
                                  : IM_COL32(170,175,185, sel ? 235 : 150);
        dl->AddRectFilled(ImVec2(x0,yhi), ImVec2(x1,ylo), fill);
        dl->AddRect(ImVec2(x0,yhi), ImVec2(x1,ylo), line, 0.f, 0, sel ? 2.0f : 1.4f);

        // 라벨: 번호 + 1위 위성 이름. 번호는 **표에 보이는 순번**이어야 한다 — 표가
        // 위성만 싣는데 여기서 전체 인덱스를 쓰면 둘이 어긋나 서로 못 짚는다.
        int shown = 0;
        for(size_t q = 0; q <= i; q++)
            if(g_show_rejected || g_tracks[q].score > 0.0f) shown++;
        char lab[80];
        const char* nm = (i < g_top_name.size() && !g_top_name[i].empty())
                       ? g_top_name[i].c_str() : nullptr;
        if(nm) snprintf(lab, sizeof lab, "%d %s", shown, nm);
        else   snprintf(lab, sizeof lab, "%d", shown);
        const ImVec2 ts = ImGui::CalcTextSize(lab);
        const float tx = x0;
        float ty = yhi - ts.y - 3.0f;
        if(ty < clip0.y + 2.0f) ty = ylo + 3.0f;   // 위가 막히면 박스 아래로
        dl->AddRectFilled(ImVec2(tx-3, ty-2), ImVec2(tx+ts.x+3, ty+ts.y+2),
                          IM_COL32(0,0,0,180));
        dl->AddText(ImVec2(tx,ty), is_sat ? IM_COL32(255,225,140,255)
                                          : IM_COL32(195,200,210,255), lab);
    };

    // **고른 것 하나만 그린다.** 전부 겹쳐 그리면 137.7MHz 대역처럼 트랙이 몰린
    // 파일에서 박스와 라벨이 포개져 어느 게 어느 행인지 못 짚는다. 표에서 행을
    // 누르면 그 항목만 뜨고, 다시 누르면(선택 해제) 이미지가 깨끗해진다.
    auto visible = [&](size_t i){
        return (g_show_rejected || g_tracks[i].score > 0.0f) && !g_tracks[i].pts.empty();
    };
    if(g_sel_track < 0 || g_sel_track >= (int)g_tracks.size()) return;
    if(!visible((size_t)g_sel_track)) return;
    draw_one((size_t)g_sel_track);
    const Doppler::Candidate& c = g_tracks[g_sel_track];

    // 관측 트랙 (빨강). f -> 선형 인덱스는 HistReader 의 역변환을 그대로 쓴다.
    // **fftshift 언랩은 저장 접근에만 쓰고 축에는 안 쓴다** — 여기서 lin 은 이미 축이다.
    ImVec2 prev(0,0); bool have = false;
    for(const Doppler::TrackPoint& p : c.pts){
        const float x = row_to_px((p.t_utc - t0)*rr);
        const float y = lin_to_py(R.linear_of_freq_hz(p.f_hz));
        const ImVec2 q(x,y);
        if(have) dl->AddLine(prev, q, IM_COL32(235,80,70,220), 1.6f);
        prev = q; have = true;
    }
    // 적합 곡선 (녹색) — 200점 샘플
    if(c.fit.valid){
        have = false;
        const double ta = c.t_start_utc, tb = c.t_end_utc;
        for(int i = 0; i <= 200; i++){
            const double t = ta + (tb-ta)*i/200.0;
            const double xx = (t - c.fit.t_tca_utc)/c.fit.tau_s;
            const double f  = c.fit.f_center_hz - c.fit.half_swing_hz*xx/std::sqrt(1.0+xx*xx);
            const ImVec2 q(row_to_px((t - t0)*rr), lin_to_py(R.linear_of_freq_hz(f)));
            if(have) dl->AddLine(prev, q, IM_COL32(90,220,120,200), 1.4f);
            prev = q; have = true;
        }
    }
}

// 패널만 접는다. 결과는 남긴다 — 같은 녹화를 다시 열었을 때 스캔을 또 돌리지 않기
// 위해서다. 다른 녹화로 갈아타면 note_file_changed 가 버린다.
void on_close(){
    g_open = false;
}

// 뷰어를 완전히 닫을 때 (모달 종료). 여기서는 다 버린다 — 리더가 사라지므로
// 트랙이 가리키는 파일도 없다.
void on_viewer_closed(){
    g_open = false;
    wipe();
    g_owner_path.clear(); g_owner_rows = 0;
}

} // namespace DopplerView
