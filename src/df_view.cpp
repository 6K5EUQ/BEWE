// ── DF(방탐) 설정 오버레이 ────────────────────────────────────────────────
// 하단 상태바의 DF 를 누르면 열린다. 전체영역 오버레이라 다른 오버레이와
// 상호배타이고, 열려 있는 동안 메인페이지 키보드/마우스는 막힌다.
//
// 구조는 sig_lib_view.cpp 의 draw_overlay 와 같다 — 상태바(32 px)는 남겨서
// 다시 눌러 닫을 수 있게 한다.
//
// 설정은 전부 HOST 소유다. JOIN 은 HOST 가 방송한 정본을 보여주고, 바꾸면
// HOST 로 요청만 보낸다 — 측정을 실제로 하는 쪽과 화면이 어긋나지 않게.

#include "fft_viewer.hpp"
#include "net_server.hpp"
#include "imgui.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

// 5소자 UCA 를 위에서 본 그림 + 마지막 측정 방위. 배선 방향(CW/CCW)이
// 화면에서 바로 보여야 운용자가 설정을 틀리게 두는 걸 알아챈다.
void draw_array_diagram(ImDrawList* dl, ImVec2 c, float R, int elements,
                        int sense, double bearing_deg, bool have_bearing){
    const float PI = 3.14159265358979f;
    dl->AddCircle(c, R, IM_COL32(90,90,90,255), 64, 1.5f);
    dl->AddLine(ImVec2(c.x, c.y-R-14), ImVec2(c.x, c.y-R+6), IM_COL32(120,120,120,255), 1.0f);
    dl->AddText(ImVec2(c.x-4, c.y-R-30), IM_COL32(150,150,150,255), "0");

    const float dir = (sense == 0) ? 1.0f : -1.0f;   // 0=CW
    if(elements < 1) elements = 1;
    for(int m = 0; m < elements; m++){
        // 화면은 나침반 배치: 0도가 위, 시계방향이 오른쪽.
        const float phi = dir * 2.0f * PI * m / (float)elements;
        const ImVec2 p(c.x + R*std::sin(phi), c.y - R*std::cos(phi));
        const ImU32 col = (m == 0) ? IM_COL32(255,200,0,255) : IM_COL32(160,160,160,255);
        dl->AddCircleFilled(p, 5.0f, col);
        char lb[8]; snprintf(lb, sizeof lb, "%d", m);
        dl->AddText(ImVec2(p.x+7, p.y-7), col, lb);
    }
    if(have_bearing){
        const float b = (float)(bearing_deg * PI / 180.0);
        const ImVec2 tip(c.x + (R+26)*std::sin(b), c.y - (R+26)*std::cos(b));
        dl->AddLine(c, tip, IM_COL32(80,220,80,255), 2.5f);
        dl->AddCircleFilled(tip, 4.0f, IM_COL32(80,220,80,255));
    }
}

// 360빈 의사스펙트럼을 극좌표로. 봉우리가 얼마나 뾰족한지가 신뢰도를 눈으로 준다.
void draw_polar_spectrum(ImDrawList* dl, ImVec2 c, float R, const float* db, int n){
    if(!db) return;
    ImVec2 prev;
    for(int i = 0; i <= n; i++){
        const int k = i % n;
        const float PI = 3.14159265358979f;
        float r = (db[k] + 40.0f) / 40.0f;      // -40 dB 를 중심, 0 dB 를 반지름으로
        if(r < 0.f) r = 0.f;
        if(r > 1.f) r = 1.f;
        const float a = k * 2.0f * PI / n;
        const ImVec2 p(c.x + R*r*std::sin(a), c.y - R*r*std::cos(a));
        if(i > 0) dl->AddLine(prev, p, IM_COL32(80,180,255,200), 1.2f);
        prev = p;
    }
}

const char* link_text(int l){
    return l == 2 ? "STREAMING" : (l == 1 ? "CALIBRATING" : "DOWN");
}

void help(const char* txt){
    ImGui::SameLine(); ImGui::TextDisabled("(?)");
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("%s", txt);
}

} // namespace

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
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    if(ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
       ImGui::IsKeyPressed(ImGuiKey_Escape, false))
        v.df_panel_open = false;

    FFTViewer::DFLive L{};
    v.df_get_live(L);
    PktDfConfig c{};
    v.df_get_cfg(c);
    const PktDfConfig before = c;
    const bool is_join = (v.net_cli != nullptr);

    ImGui::TextColored(ImVec4(0.9f,0.9f,0.9f,1.f), "DIRECTION FINDING");
    ImGui::SameLine(); ImGui::TextDisabled("(ESC or click DF to close)");
    ImGui::SameLine(ImGui::GetWindowWidth() - 320);
    ImGui::TextDisabled(is_join ? "settings live on the HOST - shared by every station"
                                : "settings are shared with every JOIN station");
    ImGui::Separator();

    if(!is_join && v.hw.type != HWType::KRAKEN){
        ImGui::TextColored(ImVec4(1.f,0.4f,0.4f,1.f),
            "DF requires the KrakenSDR backend. Start BEWE with  --sdr kraken");
        ImGui::TextDisabled("The heimdall DAQ chain must be running:");
        ImGui::TextDisabled("  /home/ku/krakensdr_doa/bewe_df_start.sh");
        ImGui::End();
        return;
    }

    ImGui::Columns(2, "##df_cols", true);

    // ══════════════════════ 왼쪽 ══════════════════════
    // ── DAQ 상태 ────────────────────────────────────────────────────────
    if(is_join){
        ImGui::TextColored(ImVec4(0.7f,0.7f,0.7f,1.f),
            "DAQ status is shown on the HOST station.");
    } else {
        const ImVec4 col_link = (L.link==2) ? ImVec4(0.3f,0.9f,0.3f,1.f)
                              : (L.link==1) ? ImVec4(1.f,0.8f,0.f,1.f)
                                            : ImVec4(0.9f,0.3f,0.3f,1.f);
        ImGui::TextColored(col_link, "DAQ LINK: %s", link_text(L.link));
        if(L.last_error[0]) ImGui::TextColored(ImVec4(1.f,0.5f,0.5f,1.f), "%s", L.last_error);
        if(L.link > 0){
            ImGui::Text("hw=%s  ch=%u  %.4f MHz  %.3f MSPS",
                        L.hw_id, L.channels, L.daq_cf_mhz, L.daq_fs_msps);
            ImGui::Text("sync_state=%u/6  delay_sync=%u  iq_sync=%u  noise_src=%u",
                        L.sync_state, L.delay_sync, L.iq_sync, L.noise_src);
            if(L.sync_state < 6)
                ImGui::TextColored(ImVec4(1.f,0.8f,0.f,1.f),
                    "calibrating - DF is refused until sync_state reaches 6");
            ImGui::Text("%.2f frames/s  %.1f MB/s   ok=%llu cal=%llu bad=%llu gaps=%llu reconn=%llu",
                        L.frame_rate_hz, L.recv_mbps, L.frames_ok, L.frames_cal,
                        L.frames_bad, L.gaps, L.reconnects);
            if(L.overdrive)
                ImGui::TextColored(ImVec4(1.f,0.5f,0.f,1.f),
                                   "ADC OVERDRIVE mask 0x%x", L.overdrive);
            ImGui::Text("gains(dB):");
            for(unsigned i = 0; i < L.channels && i < 8; i++){
                ImGui::SameLine(); ImGui::Text("%.1f", L.gain_tenths[i]/10.0);
            }
        }
    }

    // ── DAQ 제어 ────────────────────────────────────────────────────────
    ImGui::Dummy(ImVec2(0,8)); ImGui::Separator();
    ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "DAQ CONTROL");
    bool ctrl = (c.enable_control != 0);
    if(ImGui::Checkbox("allow BEWE to retune the DAQ", &ctrl)) c.enable_control = ctrl ? 1 : 0;
    help("When on, changing the frequency in BEWE sends FREQ to heimdall (:5001).\n"
         "heimdall then recalibrates from scratch - a few seconds with no DF.\n"
         "Sample rate is owned by daq_chain_config.ini and cannot be set here.");
    if(!is_join && L.link > 0){
        static float retune_mhz = 0.f;
        if(retune_mhz <= 0.f) retune_mhz = (float)L.daq_cf_mhz;
        ImGui::SetNextItemWidth(160);
        ImGui::InputFloat("center (MHz)", &retune_mhz, 0.1f, 1.0f, "%.4f");
        ImGui::SameLine();
        if(ImGui::Button("Apply##daqfreq") && retune_mhz > 0.f)
            v.set_frequency(retune_mhz, false);
        ImGui::SameLine();
        if(ImGui::Button("Now##daqfreqnow")) retune_mhz = (float)L.daq_cf_mhz;
        ImGui::TextDisabled("  the BEWE frequency box does the same thing");
    }

    // ── 배열 기하 ───────────────────────────────────────────────────────
    ImGui::Dummy(ImVec2(0,8)); ImGui::Separator();
    ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "ARRAY GEOMETRY");

    ImGui::SetNextItemWidth(160);
    ImGui::InputFloat("radius (m)", &c.radius_m, 0.005f, 0.05f, "%.4f");
    help("Distance from the array centre to ONE antenna.\n"
         "NOT the antenna-to-antenna spacing.\n"
         "If you measured the spacing d:  radius = d / 1.1756  (5 elements)");

    int el = c.elements;
    ImGui::SetNextItemWidth(160);
    if(ImGui::SliderInt("elements", &el, 3, 8)) c.elements = (uint8_t)el;
    if(!is_join && L.channels > 0 && (unsigned)c.elements != L.channels)
        ImGui::TextColored(ImVec4(1.f,0.5f,0.5f,1.f),
            "  mismatch: the DAQ reports %u channels - DF will refuse", L.channels);

    int sense = c.sense;
    ImGui::SetNextItemWidth(160);
    const char* sense_items[] = { "CW  (clockwise)", "CCW (counter-clockwise)" };
    if(ImGui::Combo("numbering", &sense, sense_items, 2)) c.sense = (uint8_t)sense;
    help("Which way antenna numbers advance seen FROM ABOVE.\n"
         "If every bearing comes out mirrored about the 0-180 axis,\n"
         "this is the setting to flip.");

    ImGui::SetNextItemWidth(160);
    ImGui::InputFloat("heading offset (deg)", &c.heading_deg, 1.0f, 10.0f, "%.1f");
    help("Added to the reported bearing.\n"
         "Use it when antenna 0 does not point along the vehicle heading.\n"
         "0 means bearings are relative to antenna 0.");

    if(L.lambda_m > 0.0){
        ImGui::Text("lambda %.3f m at the DAQ centre;  ambiguity ratio %.3f",
                    L.lambda_m, L.ambiguity);
        if(L.ambiguity > 1.0)
            ImGui::TextColored(ImVec4(1.f,0.6f,0.f,1.f),
                "  >1: grating lobes - bearings may be ambiguous at this frequency");
    }

    ImGui::NextColumn();

    // ══════════════════════ 오른쪽 ══════════════════════
    ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "ESTIMATION");

    int algo = c.algo;
    ImGui::SetNextItemWidth(160);
    const char* algo_items[] = { "Bartlett", "Capon (MVDR)", "MUSIC" };
    if(ImGui::Combo("algorithm", &algo, algo_items, 3)) c.algo = (uint8_t)algo;
    help("Bartlett : broadest peak, most robust, no eigen decomposition\n"
         "Capon    : sharper, needs a well conditioned R\n"
         "MUSIC    : sharpest, needs the source count below");

    int sd = c.signal_dim;
    ImGui::SetNextItemWidth(160);
    if(ImGui::SliderInt("sources (MUSIC)", &sd, 1, c.elements > 1 ? c.elements-1 : 1))
        c.signal_dim = (uint8_t)sd;
    help("How many signals MUSIC should assume are inside the channel.\n"
         "1 is right for a single emitter. Too high eats the signal subspace.");

    int af = c.avg_frames;
    ImGui::SetNextItemWidth(160);
    if(ImGui::SliderInt("frames to average", &af, 1, 30)) c.avg_frames = (uint8_t)af;
    ImGui::SameLine();
    ImGui::TextDisabled("~%.1f s", af * 0.437);
    help("More frames = steadier bearing, longer wait.\n"
         "One heimdall frame is about 437 ms.");

    int mf = c.max_frames;
    ImGui::SetNextItemWidth(160);
    if(ImGui::SliderInt("frame budget", &mf, c.avg_frames, 60)) c.max_frames = (uint8_t)mf;
    help("Upper bound on frames consumed while trying to collect the average.\n"
         "Calibration bursts (about every 5 min) are discarded and eat budget.");

    // ── 수락 규칙 ───────────────────────────────────────────────────────
    ImGui::Dummy(ImVec2(0,8)); ImGui::Separator();
    ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "ACCEPTANCE");
    ImGui::SetNextItemWidth(160);
    ImGui::SliderFloat("SNR threshold (dB)", &c.snr_thr_db, -10.0f, 40.0f, "%.0f");
    help("A measurement below this SNR is reported as \"No signal\".\n"
         "Observed here: real signals 18-23 dB, noise well below 0 dB.\n"
         "This value lives on the HOST and is shared by every station.");
    ImGui::TextDisabled("a statistical floor also runs underneath - see c_papr below");

    // ── 신호 추출 ───────────────────────────────────────────────────────
    ImGui::Dummy(ImVec2(0,8)); ImGui::Separator();
    ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "SIGNAL EXTRACTION");
    ImGui::SetNextItemWidth(160);
    ImGui::InputFloat("DC guard (Hz)", &c.dc_guard_hz, 100.f, 1000.f, "%.0f");
    help("Bins closer than this to the DAQ centre are dropped.\n"
         "The LO leaks a strong spike there that is not a real signal.");

    int tl = c.target_looks;
    ImGui::SetNextItemWidth(160);
    if(ImGui::SliderInt("target looks", &tl, 128, 16384)) c.target_looks = (uint16_t)tl;
    help("How many independent spectral looks to gather per frame.\n"
         "This is the compute budget: more looks = steadier R, more FFTs.\n"
         "Cost is nearly independent of channel width.");

    int kfs = c.fft_size;
    ImGui::SetNextItemWidth(160);
    const char* k_items[] = { "1024", "2048", "4096", "8192", "16384", "32768" };
    int k_idx = 3;
    for(int i = 0; i < 6; i++) if(kfs == (1024 << i)) k_idx = i;
    if(ImGui::Combo("segment FFT", &k_idx, k_items, 6)) c.fft_size = (uint16_t)(1024 << k_idx);
    help("Starting segment size. The engine raises it on its own when the\n"
         "channel is too narrow to hold at least 8 bins.");

    ImGui::Dummy(ImVec2(0,4));
    if(ImGui::TreeNode("Advanced")){
        ImGui::SetNextItemWidth(160);
        ImGui::InputFloat("c_papr", &c.c_papr, 1.0f, 10.0f, "%.1f");
        help("Statistical floor constant. Reject threshold = c_papr / sqrt(n_eff).\n"
             "This is what stops a random bearing being reported on pure noise.\n"
             "Measured: noise 0/200 false accepts, -10 dB signal 200/200 detected.\n"
             "Lower it only if you know why you are doing it.");
        ImGui::TreePop();
    }

    // 변경분이 있으면 한 번에 보낸다. HOST 는 즉시 적용 + 방송,
    // JOIN 은 요청만 보내고 HOST 가 돌려주는 값을 정본으로 삼는다.
    if(memcmp(&before, &c, sizeof c) != 0) v.df_set_cfg(c);

    // ── 배열 그림 + 마지막 결과 ─────────────────────────────────────────
    ImGui::Dummy(ImVec2(0,8)); ImGui::Separator();
    ImGui::TextColored(ImVec4(0.8f,0.8f,1.f,1.f), "LAST RESULT");
    const ImVec2 org = ImGui::GetCursorScreenPos();
    const float  R   = 100.0f;
    const ImVec2 ctr(org.x + R + 30.0f, org.y + R + 14.0f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const bool have = v.df_last_valid;
    if(have) draw_polar_spectrum(dl, ctr, R, v.df_last_spectrum, 360);
    draw_array_diagram(dl, ctr, R*0.55f, c.elements, c.sense, v.df_last_bearing, have);
    ImGui::Dummy(ImVec2(0, R*2 + 26));

    if(L.measuring){
        ImGui::TextColored(ImVec4(1.f,0.8f,0.f,1.f), "measuring...");
        ImGui::ProgressBar(L.progress, ImVec2(-1, 0));
    } else if(have){
        ImGui::Text("CH%d   %.4f MHz   BW %.1f kHz",
                    v.df_last_dnum, v.df_last_cf_mhz, v.df_last_bw_khz);
        ImGui::TextColored(ImVec4(0.3f,0.9f,0.3f,1.f), "bearing  %.1f deg", v.df_last_bearing);
        ImGui::Text("SNR %.1f dB   confidence %.2f dB   power %.1f dBFS",
                    v.df_last_snr, v.df_last_conf, v.df_last_pwr);
        ImGui::TextDisabled("relative to antenna 0: %.1f deg", v.df_last_bearing_rel);
    } else {
        ImGui::TextDisabled("no measurement yet");
    }

    ImGui::Columns(1);
    ImGui::Separator();
    ImGui::TextDisabled("Press a number key on the main page to measure that channel filter.");
    ImGui::TextDisabled("The number is the one drawn on the filter (frequency order), not a slot id."
                        "   0 targets filter #10.");
    ImGui::End();
}
