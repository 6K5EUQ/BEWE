// ── AMC 모듈 GUI: 코어 STATUS 패널의 접이식 섹션 ───────────────────────────
#include "amc_module.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <ctime>

namespace amc_mod {

namespace {
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}

// 신뢰도 색. 낮은 값을 초록으로 칠하면 운용자가 확정으로 오해한다.
// 0.8↑ 초록 / 0.5↑ 노랑 / 그 아래 회색.
ImVec4 conf_col(float c){
    if(c >= 0.80f) return ImVec4(0.45f,0.90f,0.50f,1.f);
    if(c >= 0.50f) return ImVec4(1.00f,0.85f,0.30f,1.f);
    return ImVec4(0.65f,0.65f,0.65f,1.f);
}

} // anonymous


// ── 코어 STATUS 접이식 섹션 ─────────────────────────────────────────────────
// AMC 를 켜 둔 채널마다 "지금 무엇으로 잡히는가"를 12클래스 막대로 보여준다.
// 1위만 숫자로 보여주면 61% 와 95% 가 같은 무게로 읽힌다 — 막대를 다 그려야
// 운용자가 판정이 아슬아슬한지 확실한지를 한눈에 안다.
void draw_panel(FFTViewer& v){
    const char* stn = bewe_mod_my_station();

    // 채널별 최신 레코드 1건 (log 은 시간순 append 라 뒤에서부터 찾는다).
    // 락을 짧게 잡고 값만 복사한다 — ImGui 호출을 락 안에서 하면 안 된다.
    AmcRecord last[MAX_CHANNELS];
    bool has[MAX_CHANNELS] = {};
    {
        std::lock_guard<std::mutex> lk(mtx);
        for(int i=(int)log.size()-1; i>=0; i--){
            const AmcRecord& m = log[i];
            if(m.ch < 0 || m.ch >= MAX_CHANNELS || has[m.ch]) continue;
            last[m.ch] = m; has[m.ch] = true;
        }
    }

    int shown = 0;
    for(int ci=0; ci<MAX_CHANNELS; ci++){
        if(!v.channels[ci].filter_active) continue;
        if(!bewe_mod_ch_on("amc", stn, ci)) continue;
        shown++;

        float cf = (v.channels[ci].s + v.channels[ci].e) * 0.5f;
        ImGui::PushID(ci);
        if(has[ci]){
            const AmcRecord& m = last[ci];
            char ts[12]; hms(m.t_ms, ts);
            ImGui::Text("[%2d] %9.4f MHz", v.freq_sorted_display_num(ci), cf);
            ImGui::SameLine();
            ImGui::TextColored(conf_col(m.conf), "%s %.0f%%", amc_class_name(m.cls), m.conf*100.f);
            ImGui::SameLine(); ImGui::TextDisabled("%s", ts);

            // 막대: 클래스 이름 + 비율. 1위만 색을 준다.
            int top = 0;
            for(int k=1;k<AMC_NCLASS;k++) if(m.p[k] > m.p[top]) top = k;
            for(int k=0;k<AMC_NCLASS;k++){
                if(m.p[k] < 0.005f && k != top) continue;   // 0에 가까운 건 줄만 낭비
                ImGui::Text("  %-6s", amc_class_name(k));
                ImGui::SameLine(88.f);
                ImVec4 c = (k==top) ? conf_col(m.p[k]) : ImVec4(0.45f,0.45f,0.5f,1.f);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, c);
                char ov[16]; snprintf(ov,sizeof(ov),"%.0f%%", m.p[k]*100.f);
                ImGui::ProgressBar(m.p[k], ImVec2(-1.f, 12.f), ov);
                ImGui::PopStyleColor();
            }
        } else {
            ImGui::Text("[%2d] %9.4f MHz", v.freq_sorted_display_num(ci), cf);
            ImGui::SameLine(); ImGui::TextDisabled("waiting for a burst");
        }
        ImGui::Spacing();
        ImGui::PopID();
    }

    if(!shown){
        if(!bewe_mod_avail("amc", stn)) ImGui::TextDisabled("  not available on this station");
        else                            ImGui::TextDisabled("  (none)");
    }
}

} // namespace amc_mod
