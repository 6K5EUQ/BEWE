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
#include <string>
#include <vector>

namespace DopplerView {

namespace {

bool  g_open = false;
char  g_filter[64] = {0};
int   g_sel_track = -1;
int   g_sort_col  = -1;
bool  g_sort_asc  = true;
modview::Selection g_sel;

std::vector<Doppler::Candidate> g_tracks;
DopplerMatch::Result            g_match;
bool                            g_have_match = false;
uint32_t                        g_match_of = 0xFFFFFFFFu;

std::string tle_dir(){ return BEWEPaths::assets_dir() + "/tle"; }

std::string hhmmss(double t){
    time_t s = (time_t)t; struct tm tv{}; KST::to_tm(s, tv);
    char b[16]; snprintf(b, sizeof b, "%02d:%02d:%02d", tv.tm_hour, tv.tm_min, tv.tm_sec);
    return b;
}

void refresh_from_worker(){
    std::vector<Doppler::Candidate> t;
    if(DopplerScan::results(t)) g_tracks = std::move(t);
    if(g_sel_track >= 0 && g_sel_track < (int)g_tracks.size()){
        const uint32_t id = g_tracks[g_sel_track].id;
        if(id != g_match_of){
            g_have_match = DopplerScan::match_of(id, g_match);
            g_match_of = id;
        }
    }
}

// 첫 통과 트랙을 자동 선택 (없으면 첫 트랙)
void auto_select(){
    if(g_sel_track >= 0 || g_tracks.empty()) return;
    for(size_t i = 0; i < g_tracks.size(); i++)
        if(g_tracks[i].score > 0.0f){ g_sel_track = (int)i; return; }
    g_sel_track = 0;
}

} // anon

bool panel_open(){ return g_open; }

bool toolbar_button(const HistReader& R){
    // 좌표계 없는 파일(v2 헤더/미설정 기지)은 look-angle 을 못 구한다.
    const bool ok = R.is_open() && R.has_station_pos();
    const DopplerScan::Status st = DopplerScan::status();

    char label[32];
    if(st.st == DopplerScan::State::Running){
        if(st.stage[0]) snprintf(label, sizeof label, "%s %d%%", st.stage, (int)(st.progress*100));
        else            snprintf(label, sizeof label, "SCAN %d%%", (int)(st.progress*100));
    } else {
        snprintf(label, sizeof label, "DOPPLER");
    }

    if(!ok) ImGui::BeginDisabled();
    if(g_open) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f,0.42f,0.22f,1.0f));
    const bool hit = ImGui::Button(label);
    if(g_open) ImGui::PopStyleColor();
    if(!ok) ImGui::EndDisabled();
    return hit && ok;
}

void toggle(const HistReader& R, const Doppler::ExtractParams& P){
    if(g_open){ g_open = false; return; }
    g_open = true;
    g_sel_track = -1; g_sel.clear(); g_have_match = false; g_match_of = 0xFFFFFFFFu;
    g_tracks.clear();
    if(!R.is_open() || !R.has_station_pos()) return;
    // LIVE 는 계속 자라서 선해제도 안 되고 행수가 스캔 중에 바뀐다 — 자동 스캔 대상 아님.
    if(R.is_live()) return;
    DopplerMatch::Params MP;
    DopplerScan::start_full(R, P, MP, tle_dir());
}

void start_refine(const HistReader& R, uint32_t row_lo, uint32_t row_hi,
                  uint32_t lin_lo, uint32_t lin_hi, const Doppler::ExtractParams& P){
    if(!R.is_open() || !R.has_station_pos()) return;
    g_open = true;
    g_sel_track = -1; g_sel.clear(); g_have_match = false; g_match_of = 0xFFFFFFFFu;
    g_tracks.clear();
    DopplerMatch::Params MP;
    DopplerScan::start_refine(R, row_lo, row_hi, lin_lo, lin_hi, P, MP, tle_dir());
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

    // ── 헤더 (modview::header_bar 의 겉모습만 재현) ──────────────────────
    // header_bar 자체는 못 쓴다 — 시그니처가 모듈 프레임워크(bewe_mod_recv/hist_mode)에
    // 묶여 있어 여기서 의미 없는 RECV/DB/PLAYBACK 버튼을 그린다.
    {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f,0.12f,0.16f,1.0f));
        ImGui::BeginChild("##dop_hdr", ImVec2(0, 30), false);
        ImGui::SetCursorPos(ImVec2(12, 4));
        ImGui::SetNextItemWidth(200);
        ImGui::InputText("##dopfilter", g_filter, sizeof g_filter);
        ImGui::SameLine(0, 8);
        ImGui::Text("%zu trk", g_tracks.size());
        if(st.st == DopplerScan::State::Running){
            ImGui::SameLine(0, 8);
            ImGui::TextDisabled("%s", st.stage[0] ? st.stage : "scanning");
        }
        const char* rl = (st.st == DopplerScan::State::Running) ? "CANCEL" : "RESCAN";
        const float bw = ImGui::CalcTextSize(rl).x + ImGui::GetStyle().FramePadding.x*2;
        ImGui::SameLine(W - bw - 12);
        if(ImGui::Button(rl)){
            if(st.st == DopplerScan::State::Running) DopplerScan::cancel();
            else {
                Doppler::ExtractParams P;
                DopplerMatch::Params MP;
                DopplerScan::start_full(R, P, MP, tle_dir());
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();      // BeginChild 가 ChildBg 를 소비하므로 여기서 해제
    }

    // ── TLE 나이 (데이터지 설명문이 아니다) ─────────────────────────────
    if(g_have_match){
        const double age = g_match.tle_age_days;
        ImVec4 col = (age > 10.0) ? ImVec4(0.95f,0.35f,0.30f,1)
                   : (age > 3.0)  ? ImVec4(0.95f,0.72f,0.25f,1)
                                  : ImVec4(0.65f,0.70f,0.78f,1);
        ImGui::Text("TLE %s", g_match.tle_src.c_str());
        ImGui::SameLine();
        ImGui::TextColored(col, "age %.1f d", age);
        ImGui::SameLine();
        ImGui::TextDisabled("n=%d", g_match.n_loaded);
    }
    // 신뢰도 판정 — 표보다 위에 둔다. 낡은 카탈로그는 순위표를 그럴듯하게 채우면서
    // 조용히 틀리므로(실측: 38일 낡음에서 6건 중 5건이 오답 1위), 표만 보면 속는다.
    if(g_have_match && g_match.verdict != DopplerMatch::Verdict::Reliable){
        const bool bad = (g_match.verdict == DopplerMatch::Verdict::Unreliable);
        ImGui::TextColored(bad ? ImVec4(0.95f,0.35f,0.30f,1) : ImVec4(0.95f,0.72f,0.25f,1),
                           "%s", bad ? "UNRELIABLE" : "AMBIGUOUS");
        if(!g_match.verdict_why.empty()){
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(ImVec4(0.78f,0.72f,0.62f,1), "%s", g_match.verdict_why.c_str());
            ImGui::PopTextWrapPos();
        }
    }
    if(st.st == DopplerScan::State::Failed && !st.err.empty())
        ImGui::TextColored(ImVec4(0.95f,0.4f,0.35f,1), "%s", st.err.c_str());
    if(R.is_live())
        ImGui::TextDisabled("LIVE file - use Ctrl+drag then REFINE");

    // ── 트랙 목록 ────────────────────────────────────────────────────────
    if(g_tracks.size() > 1){
        const float th = ImGui::GetTextLineHeightWithSpacing()*std::min<size_t>(6, g_tracks.size()) + 28;
        if(ImGui::BeginTable("##dop_trk", 5,
               ImGuiTableFlags_Borders|ImGuiTableFlags_RowBg|ImGuiTableFlags_ScrollY,
               ImVec2(0, th))){
            ImGui::TableSetupScrollFreeze(0,1);
            ImGui::TableSetupColumn("#");
            ImGui::TableSetupColumn("Start");
            ImGui::TableSetupColumn("Dur");
            ImGui::TableSetupColumn("Pts");
            ImGui::TableSetupColumn("Score");
            ImGui::TableHeadersRow();
            for(size_t i = 0; i < g_tracks.size(); i++){
                const Doppler::Candidate& c = g_tracks[i];
                ImGui::TableNextRow();
                char idx[8]; snprintf(idx, sizeof idx, "%zu", i+1);
                if(modview::row_col0((int)i, (int)i == g_sel_track, idx)){
                    g_sel_track = (int)i; g_match_of = 0xFFFFFFFFu;
                }
                ImGui::TableSetColumnIndex(1); modview::cell(hhmmss(c.t_start_utc).c_str());
                char b[32];
                ImGui::TableSetColumnIndex(2);
                snprintf(b,sizeof b,"%.0fs", c.t_end_utc-c.t_start_utc); modview::cell(b);
                ImGui::TableSetColumnIndex(3);
                snprintf(b,sizeof b,"%zu", c.pts.size()); modview::cell(b);
                ImGui::TableSetColumnIndex(4);
                snprintf(b,sizeof b,"%.2f", c.score);
                modview::cell(b, c.score > 0.0f ? ImVec4(0.55f,0.85f,0.55f,1)
                                                : ImVec4(0.6f,0.6f,0.6f,1));
            }
            ImGui::EndTable();
        }
    }

    // ── 선택 트랙 요약 ───────────────────────────────────────────────────
    if(g_sel_track >= 0 && g_sel_track < (int)g_tracks.size()){
        const Doppler::Candidate& c = g_tracks[g_sel_track];
        ImGui::Separator();
        ImGui::Text("%s-%s  %.3f MHz", hhmmss(c.t_start_utc).c_str(),
                    hhmmss(c.t_end_utc).c_str(), c.fit.f_center_hz/1e6);
        ImGui::Text("swing %.1f kHz  tau %.0fs  slope %.1f Hz/s  rms %.2f bin",
                    std::fabs(c.fit.half_swing_hz)/1000.0, c.fit.tau_s,
                    c.fit.max_slope_hz_s, c.fit.rms_resid_bins);
        if(!c.reject_reason.empty())
            ImGui::TextColored(ImVec4(0.9f,0.6f,0.3f,1), "%s", c.reject_reason.c_str());
    }

    // ── 후보 표 ──────────────────────────────────────────────────────────
    if(g_have_match && !g_match.cands.empty()){
        ImGui::Separator();
        std::vector<int> vis;
        for(size_t i = 0; i < g_match.cands.size(); i++){
            if(g_filter[0] && !modview::ci_find(g_match.cands[i].name.c_str(), g_filter)) continue;
            vis.push_back((int)i);
        }
        modview::sort_vis(vis, g_sort_col, g_sort_asc, [&](int col, int a, int b)->int{
            const auto& A = g_match.cands[a]; const auto& B = g_match.cands[b];
            switch(col){
                case 1: return A.name.compare(B.name);
                case 2: return A.norad<B.norad?-1:(A.norad>B.norad?1:0);
                case 3: return A.rms_hz<B.rms_hz?-1:(A.rms_hz>B.rms_hz?1:0);
                case 4: return A.sep<B.sep?-1:(A.sep>B.sep?1:0);
                case 5: return A.max_el_deg<B.max_el_deg?-1:(A.max_el_deg>B.max_el_deg?1:0);
                case 6: return A.dtca_s<B.dtca_s?-1:(A.dtca_s>B.dtca_s?1:0);
                case 7: return A.cov<B.cov?-1:(A.cov>B.cov?1:0);
                default: return 0;
            }
        });
        const float rest = ImGui::GetContentRegionAvail().y - 100.0f;
        if(ImGui::BeginTable("##dop_cand", 8,
               ImGuiTableFlags_Borders|ImGuiTableFlags_RowBg|ImGuiTableFlags_ScrollY,
               ImVec2(0, rest > 60 ? rest : 60))){
            ImGui::TableSetupScrollFreeze(0,1);
            ImGui::TableSetupColumn("#");    ImGui::TableSetupColumn("NAME");
            ImGui::TableSetupColumn("NORAD");ImGui::TableSetupColumn("RMS");
            ImGui::TableSetupColumn("x");    ImGui::TableSetupColumn("MaxEl");
            ImGui::TableSetupColumn("dTCA"); ImGui::TableSetupColumn("Cov");
            modview::sortable_headers(8, g_sort_col, g_sort_asc, 1);
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
                ImGui::TableSetColumnIndex(1); modview::cell_left(c.name.c_str());
                ImGui::TableSetColumnIndex(2); snprintf(b,sizeof b,"%d",c.norad); modview::cell(b);
                ImGui::TableSetColumnIndex(3); snprintf(b,sizeof b,"%.0f",c.rms_hz); modview::cell(b);
                ImGui::TableSetColumnIndex(4); snprintf(b,sizeof b,"%.2f",c.sep);
                modview::cell(b, c.sep >= 3.0 || vis[k] == 0 ? ImVec4(0.85f,0.85f,0.85f,1)
                                                             : ImVec4(0.9f,0.7f,0.35f,1));
                ImGui::TableSetColumnIndex(5);
                snprintf(b,sizeof b,"%.0f\xC2\xB0",c.max_el_deg); modview::cell(b);
                ImGui::TableSetColumnIndex(6); snprintf(b,sizeof b,"%+.0fs",c.dtca_s); modview::cell(b);
                ImGui::TableSetColumnIndex(7); snprintf(b,sizeof b,"%.2f",c.cov); modview::cell(b);
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
            snprintf(b,sizeof b,"%d",c.norad);              modview::kv("NORAD",b,vc,false);
            snprintf(b,sizeof b,"%.2f\xC2\xB0",c.incl_deg); modview::kv("Incl",b,vc);
            snprintf(b,sizeof b,"%.0f km",c.alt_km);        modview::kv("Alt",b,vc);
            snprintf(b,sizeof b,"%.4f MHz",c.f0_fit_hz/1e6);modview::kv("f0",b,vc,false);
            snprintf(b,sizeof b,"%.1f\xC2\xB0",c.az_tca_deg);modview::kv("Az",b,vc);
            snprintf(b,sizeof b,"%.1f\xC2\xB0",c.el_tca_deg);modview::kv("El",b,vc);
            snprintf(b,sizeof b,"%.0f km",c.range_tca_km);  modview::kv("Range",b,vc);
            snprintf(b,sizeof b,"%+.1f %%",c.slope_err_pct);modview::kv("Slope err",b,vc,false);
            snprintf(b,sizeof b,"%.2f d",c.tle_age_days);   modview::kv("TLE age",b,vc);
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
    if(!g_open || g_sel_track < 0 || g_sel_track >= (int)g_tracks.size()) return;
    const Doppler::Candidate& c = g_tracks[g_sel_track];
    const double rr = (R.hdr().row_rate_hz > 0.0f) ? (double)R.hdr().row_rate_hz : 1.0;
    const double t0 = (double)R.hdr().start_utc_unix;

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

void on_close(){
    g_open = false;
    g_tracks.clear();
    g_sel.clear();
    g_sel_track = -1;
    g_have_match = false;
    g_match_of = 0xFFFFFFFFu;
}

} // namespace DopplerView
