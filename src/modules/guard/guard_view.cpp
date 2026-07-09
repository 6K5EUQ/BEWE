// ── GUARD 모듈 GUI: 경보 테이블 + 의사결정 지원 패널 (DEMOD 데이터 뷰) ───────
// 데이터는 guard_module.cpp 가 유지하는 스냅샷(snapshot())만 읽는다 (뷰 자체 상태 없음).
// 좌(60%) 경보 테이블(최신순) | 우(40%) 선택 경보 상세 + 권고 조치 + ACK(로컬 마킹).
#include "guard_meta.hpp"
#include "../modview.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <ctime>
#include <set>
#include <vector>
#include <algorithm>

namespace guard_mod {

namespace {
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
// 유형별 색: 충돌=빨강 좌초=주황 이상항적=노랑 위협=보라 보고=파랑
ImVec4 typ_col(uint8_t t){
    switch(t){
        case 1: return ImVec4(1.00f,0.38f,0.32f,1.f);
        case 2: return ImVec4(1.00f,0.62f,0.25f,1.f);
        case 3: return ImVec4(0.95f,0.85f,0.35f,1.f);
        case 4: return ImVec4(0.80f,0.55f,1.00f,1.f);
        case 5: return ImVec4(0.45f,0.70f,1.00f,1.f);
        default:return ImVec4(0.60f,0.60f,0.60f,1.f);
    }
}
const char* state_name(uint8_t s){
    switch(s){ case 1: return "NEW"; case 2: return "UPDATE"; case 3: return "CLEAR"; default: return "-"; }
}
// 유형 배지: 옅은 유형색 배경 라운드 박스 + 유형색 텍스트 (셀 중앙). CLEAR 는 회색.
void type_badge(uint8_t typ, bool active){
    ImVec4 c = active ? typ_col(typ) : ImVec4(0.48f,0.48f,0.48f,1.f);
    const char* lbl = guard_typ_name(typ); if(!lbl[0]) lbl="?";
    ImDrawList* dl=ImGui::GetWindowDrawList();
    float cw=ImGui::GetContentRegionAvail().x;
    ImVec2 ts=ImGui::CalcTextSize(lbl);
    ImVec2 p=ImGui::GetCursorScreenPos();
    float bw=ts.x+12.f, bh=ts.y+2.f, x=p.x+(bw<cw?(cw-bw)*0.5f:0.f);
    dl->AddRectFilled(ImVec2(x,p.y), ImVec2(x+bw,p.y+bh),
                      ImGui::GetColorU32(ImVec4(c.x,c.y,c.z,0.20f)), 3.f);
    dl->AddText(ImVec2(x+6.f,p.y+1.f), ImGui::GetColorU32(c), lbl);
    ImGui::Dummy(ImVec2(cw>bw?cw:bw, bh));
}
// 심각도 색 (1=주의 노랑, 2=경고 주황, 3=긴급 빨강)
ImVec4 sev_col(uint8_t sev){
    return sev>=3? ImVec4(1.f,0.32f,0.28f,1.f)
         : sev==2? ImVec4(1.f,0.63f,0.25f,1.f) : ImVec4(0.92f,0.82f,0.35f,1.f);
}
// 심각도 1-3 점등 (sev 개 점 켜짐). 활성 sev3 는 시간기반 알파 펄스, CLEAR 는 회색.
void sev_dots(uint8_t sev, bool active){
    ImDrawList* dl=ImGui::GetWindowDrawList();
    float cw=ImGui::GetContentRegionAvail().x, th=ImGui::GetTextLineHeight();
    ImVec2 p=ImGui::GetCursorScreenPos();
    const float R=3.5f, GAP=11.f;
    float x0=p.x+(cw-GAP*2.f)*0.5f, cy=p.y+th*0.5f;
    ImVec4 lit = active ? sev_col(sev) : ImVec4(0.45f,0.45f,0.45f,1.f);
    if(active && sev>=3) lit.w = 0.55f + 0.45f*(0.5f+0.5f*sinf((float)ImGui::GetTime()*6.f));  // 긴급 펄스
    for(int i=0;i<3;i++)
        dl->AddCircleFilled(ImVec2(x0+i*GAP,cy), R,
            i<(int)sev? ImGui::GetColorU32(lit) : IM_COL32(70,76,88,255));
    ImGui::Dummy(ImVec2(cw, th));
}
// 중앙정렬 고정 헤더 (정렬은 항상 최신순 고정 — 클릭 없음)
void plain_headers(int ncol){
    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
    for(int c=0;c<ncol;c++){
        ImGui::TableSetColumnIndex(c);
        const char* nm=ImGui::TableGetColumnName(c);
        float cw=ImGui::GetContentRegionAvail().x, tw=ImGui::CalcTextSize(nm).x;
        ImVec2 hp=ImGui::GetCursorPos();
        ImGui::SetCursorPos(ImVec2(hp.x+(tw<cw?(cw-tw)*0.5f:0.f), hp.y+2));
        ImGui::TextUnformatted(nm);
    }
}
} // anonymous

void draw_content(FFTViewer& v, bool just_opened){
    (void)v; (void)just_opened;
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float W=avail.x, H=avail.y, x0=ImGui::GetCursorPosX(), y0=ImGui::GetCursorPosY();

    static uint32_t sel_aid=0;               // 선택 경보 aid (0=없음)
    static std::set<uint32_t> local_ack;     // 로컬 ACK 마킹 (aid; 뷰 세션 한정)

    // ── 스냅샷 (모듈이 aid 별 최신상태 유지) → 최신순 정렬 ──
    std::vector<AlertRow> rows = snapshot();
    std::stable_sort(rows.begin(), rows.end(),
        [](const AlertRow& x, const AlertRow& y){ return x.a.t_ms > y.a.t_ms; });
    int total=(int)rows.size(), act=active_count();

    // ── 상단 헤더바 (modview::header_bar 간이판): "N alerts (M active)" + [ACK ALL] ──
    {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f,0.12f,0.16f,1.f));
        ImGui::BeginChild("##guard_hdr", ImVec2(W,30), false);
        float fh=ImGui::GetFrameHeight(), th=ImGui::GetTextLineHeight();
        const float VCEN=16.5f;                       // header_bar 와 동일 광학중심
        float tcy=VCEN-th*0.5f, fy=VCEN-fh*0.5f;
        ImGui::SetCursorPos(ImVec2(12, tcy));
        ImGui::Text("%d alerts", total);
        ImGui::SameLine(0,8); ImGui::SetCursorPosY(tcy);
        ImGui::TextColored(act? ImVec4(1.f,0.42f,0.36f,1.f):ImVec4(0.5f,0.55f,0.62f,1.f),
                           "(%d active)", act);
        float pad2=ImGui::GetStyle().FramePadding.x*2;
        float bw=ImGui::CalcTextSize("ACK ALL").x+pad2;
        ImGui::SameLine(); ImGui::SetCursorPos(ImVec2(W-bw-12.f, fy));
        if(ImGui::Button("ACK ALL", ImVec2(bw,0)))
            for(const AlertRow& r : rows) local_ack.insert(r.a.aid);
        ImGui::EndChild();
        ImGui::PopStyleColor();
    }

    // ── 좌(경보 테이블 60%) | 우(의사결정 지원 40%) ──
    ImGui::SetCursorPos(ImVec2(x0, y0+32.f));
    float body_h = H-36.f; if(body_h<80) body_h=80;
    float tw = (W-6.f)*0.60f; if(tw<260) tw=260;
    float rw = W-tw-6.f;      if(rw<160) rw=160;

    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_RowBg|ImGuiTableFlags_BordersInnerV;
    if(ImGui::BeginTable("##guard_tbl", 5, tf, ImVec2(tw, body_h))){
        ImGui::TableSetupScrollFreeze(0,1);
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("유형", ImGuiTableColumnFlags_WidthFixed, 86);
        ImGui::TableSetupColumn("Sev",  ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("MMSI", ImGuiTableColumnFlags_WidthFixed, 82);
        ImGui::TableSetupColumn("요약", ImGuiTableColumnFlags_WidthStretch);
        plain_headers(5);

        ImGuiListClipper clip; clip.Begin((int)rows.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const AlertRow& R=rows[r]; const GuardAlert& A=R.a;
            bool acked = local_ack.count(A.aid)!=0;
            ImGui::TableNextRow();
            char ts[12]; hms(A.t_ms,ts);
            if(modview::row_col0(r, sel_aid==A.aid, ts)) sel_aid=A.aid;   // 행 클릭 → 선택
            ImGui::TableSetColumnIndex(1); type_badge(A.typ, R.active);
            ImGui::TableSetColumnIndex(2); sev_dots(A.sev, R.active);
            char b[16];
            // 활성=밝게(굵기 대체), ACK=중간톤, CLEAR=회색
            ImVec4 tc = !R.active ? ImVec4(0.48f,0.48f,0.48f,1.f)
                      : acked     ? ImVec4(0.65f,0.68f,0.74f,1.f)
                                  : ImVec4(0.90f,0.92f,0.96f,1.f);
            ImGui::TableSetColumnIndex(3);
            if(A.mmsi){ snprintf(b,sizeof(b),"%u",A.mmsi); modview::cell(b, tc); }
            ImGui::TableSetColumnIndex(4);
            // 요약은 유형색으로 강조 (활성+미ACK 만; ACK/CLEAR 는 톤다운)
            ImVec4 mc = (R.active && !acked) ? typ_col(A.typ) : tc;
            if(acked){ ImGui::TextColored(ImVec4(0.45f,0.62f,0.48f,1.f),"ACK"); ImGui::SameLine(0,6); }
            ImGui::TextColored(mc, "%.95s", A.msg);
        }
        ImGui::EndTable();
    }

    // ── 우측: 의사결정 지원 패널 (선택 경보 상세 + 권고 조치 + ACK) ──
    ImGui::SameLine(0,0);
    modview::detail_begin("guard", x0+tw+6.f, rw, body_h);
    const AlertRow* S=nullptr;
    for(const AlertRow& r : rows) if(r.a.aid==sel_aid){ S=&r; break; }
    if(!S){
        ImGui::TextDisabled("경보를 선택하면 상세 정보와");
        ImGui::TextDisabled("권고 조치가 표시됩니다.");
    } else {
        const GuardAlert& A=S->a;
        bool acked = local_ack.count(A.aid)!=0;
        ImVec4 C=typ_col(A.typ), V(0.85f,0.85f,0.90f,1.f), K(0.55f,0.62f,0.72f,1.f);
        auto row=[&](const char* k, const char* val, ImVec4 vc){
            ImGui::TextColored(K,"%s",k); ImGui::SameLine(0,8); ImGui::TextColored(vc,"%s",val);
        };
        // 제목: 유형 + 상태 칩
        ImGui::TextColored(C, "%s", guard_typ_name(A.typ));
        ImGui::SameLine();
        ImGui::TextColored(S->active? ImVec4(0.95f,0.75f,0.30f,1.f):ImVec4(0.5f,0.5f,0.5f,1.f),
                           "[%s%s]", state_name(A.state), acked?" ACK":"");
        // 심각도 점등 + 라벨
        ImGui::TextColored(K,"심각도"); ImGui::SameLine(0,8);
        { ImDrawList* dl=ImGui::GetWindowDrawList(); ImVec2 p=ImGui::GetCursorScreenPos();
          float cy=p.y+ImGui::GetTextLineHeight()*0.5f;
          ImVec4 lit = S->active ? sev_col(A.sev) : ImVec4(0.45f,0.45f,0.45f,1.f);
          for(int i=0;i<3;i++)
              dl->AddCircleFilled(ImVec2(p.x+5.f+i*13.f,cy), 4.f,
                  i<(int)A.sev? ImGui::GetColorU32(lit) : IM_COL32(70,76,88,255));
          ImGui::Dummy(ImVec2(44, ImGui::GetTextLineHeight()));
          ImGui::SameLine(0,6); ImGui::TextColored(lit, "%s", guard_sev_name(A.sev)); }
        char b[48], ts[12]; hms(A.t_ms,ts);
        row("시각", ts, V);
        snprintf(b,sizeof(b),"%u",A.mmsi); row("MMSI", A.mmsi? b:"-", ImVec4(0.5f,0.8f,1.f,1.f));
        if(A.mmsi2){ snprintf(b,sizeof(b),"%u",A.mmsi2); row("상대선박", b, ImVec4(0.5f,0.8f,1.f,1.f)); }
        if(A.station[0]) row("기지", A.station, ImVec4(0.6f,0.6f,0.6f,1.f));
        snprintf(b,sizeof(b),"%.1f / 100",A.score); row("Score", b, A.score>=70.f? C:V);
        if(A.cpa_m>=0){ snprintf(b,sizeof(b),"%.0f m",A.cpa_m); row("CPA", b, V); }
        if(A.tcpa_s>=0){ int tt=(int)A.tcpa_s; snprintf(b,sizeof(b),"%d분 %02d초",tt/60,tt%60); row("TCPA", b, V); }
        if(A.lat!=0.f||A.lon!=0.f){ snprintf(b,sizeof(b),"%.5f, %.5f",A.lat,A.lon); row("위치", b, V); }
        ImGui::Separator();
        if(A.typ==5){
            // REPORT: msg = 일일 요약 전체 표시, reco = 보고서 파일 경로
            ImGui::TextColored(K,"일일 요약");
            ImGui::TextWrapped("%.95s", A.msg);
            ImGui::Separator();
            row("보고서", A.reco, ImVec4(0.82f,0.76f,0.55f,1.f));
        } else {
            ImGui::TextColored(K,"상황"); ImGui::TextWrapped("%.95s", A.msg);
            // 권고 조치 강조 박스 (유형색 틴트+테두리)
            ImGui::Dummy(ImVec2(0,4));
            ImGui::TextColored(C, "권고 조치");
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(C.x,C.y,C.z,0.10f));
            ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(C.x,C.y,C.z,0.55f));
            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10,8));
            ImGui::BeginChild("##guard_reco", ImVec2(0,0),
                ImGuiChildFlags_Borders|ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
            ImGui::TextWrapped("%.127s", A.reco[0]? A.reco : "-");
            ImGui::EndChild();
            ImGui::PopStyleVar(2); ImGui::PopStyleColor(2);
        }
        // ACK 버튼 (로컬 마킹; 확인했음 표시) — 이미 ACK 면 비활성
        ImGui::Dummy(ImVec2(0,6));
        ImGui::BeginDisabled(acked);
        if(ImGui::Button(acked? "ACKED":"ACK", ImVec2(-1,0))) local_ack.insert(A.aid);
        ImGui::EndDisabled();
    }
    modview::detail_end();
}

} // namespace guard_mod
