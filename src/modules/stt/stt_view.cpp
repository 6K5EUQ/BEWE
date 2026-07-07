// ── STT 자막 뷰: DEMOD 패널 "STT" 탭 내용 (GUI 전용) ──────────────────────
// squelch 발화마다 한 줄 자막. [시각][채널] 대사. 최신이 아래로 쌓이고 자동 스크롤.
#include "stt_meta.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "../modview.hpp"
#include "imgui.h"
#include <mutex>
#include <vector>
#include <string>
#include <ctime>
#include <cstring>
#include <cstdio>

namespace stt_mod {

extern std::mutex mtx;
extern std::vector<SttRecord> log;
extern char filter[64];

static void hms(int64_t t_ms, char* out){
    time_t s=(time_t)(t_ms/1000); struct tm tmv; localtime_r(&s,&tmv);
    snprintf(out,12,"%02d:%02d:%02d",tmv.tm_hour,tmv.tm_min,tmv.tm_sec);
}

void draw_content(FFTViewer& v, bool just_opened){
    (void)just_opened;
    bool remote = bewe_mod_my_station()[0] != 0;
    bool win_focus = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); };
    modview::space_toggle_recv(v, "stt", remote, win_focus, on_clear);

    int cnt; { std::lock_guard<std::mutex> lk(mtx); cnt=(int)log.size(); }
    modview::header_bar(v, "stt", filter, sizeof(filter), cnt, remote, false, on_clear);

    ImGui::BeginChild("##stt_log", ImVec2(0,0), false, ImGuiWindowFlags_HorizontalScrollbar);
    {
        std::lock_guard<std::mutex> lk(mtx);
        for(const SttRecord& m : log){
            if(filter[0] && !strstr(m.text, filter)) continue;
            char ts[12]; hms(m.t_ms, ts);
            ImGui::TextColored(ImVec4(0.55f,0.60f,0.70f,1.f), "%s", ts);
            ImGui::SameLine(0,8);
            ImGui::TextColored(ImVec4(0.45f,0.85f,0.55f,1.f), "CH%d", m.ch);
            ImGui::SameLine(0,10);
            ImGui::TextWrapped("%s", m.text);
        }
    }
    // 새 자막 오면 자동 하단 스크롤 (사용자가 위로 올려두면 유지)
    if(ImGui::GetScrollY() >= ImGui::GetScrollMaxY()-4.f)
        ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();
}

} // namespace stt_mod
