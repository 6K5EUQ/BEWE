// ── AIS 모듈 GUI: DEMOD 데이터 뷰 (공용 modview 컴포넌트로 통일) ────────────
#include "ais_module.hpp"
#include "../modview.hpp"
#include "../common/modview_map.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <set>
#include <deque>
#include <array>
#include <unordered_map>
#include <ctime>
#include <chrono>
#include <algorithm>
#ifdef BEWE_MODULE_GUARD
#include "../guard/guard_meta.hpp"   // guard_mod::vessel_alert (경보 선박 펄스 링)
#endif

// Match_AI 열(테이블 idx 11)은 AI 모듈(ais_ai.cpp) 있을 때만 표시. 없으면 열 개수·Info
// 인덱스가 하나씩 줄어든다. 컴파일 타임 상수로 인덱스 밀림을 일괄 처리.
// (비-DL Match 열은 제거됨 — 기본 열 = Up~Cnt 11개 + Info.)
// Behavior 열은 항상 존재(규칙층은 DL 아님). Match_AI 는 AI 모듈 있을 때만.
#ifdef BEWE_MODULE_AIS_AI
  #define AIS_TBL_NCOL  14
  #define AIS_TBL_BEHAV 12   // Match_AI(11) 다음
  #define AIS_TBL_INFO  13
#else
  #define AIS_TBL_NCOL  13
  #define AIS_TBL_BEHAV 11
  #define AIS_TBL_INFO  12
#endif

namespace ais_mod {

namespace {
#ifdef BEWE_MODULE_GUARD
inline int64_t wall_now_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
#endif
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
// 0 Time 1 MMSI 2 Type 3 Name 4 Lat 5 Lon 6 SOG 7 COG 8 Info
bool match(const AisRecord& m, const char* f){
    if(!f||!f[0]) return true;
    char ts[12]; hms(m.t_ms,ts);
    char mm[16]; snprintf(mm,sizeof(mm),"%u",m.mmsi);
    char ty[8]; snprintf(ty,sizeof(ty),"%d",m.msg_type);
    return modview::ci_find(ts,f)||modview::ci_find(mm,f)||modview::ci_find(ty,f)
        || modview::ci_find(m.name,f)||modview::ci_find(m.callsign,f)||modview::ci_find(m.station,f)
        || modview::ci_find(ais_mid_country(m.mmsi),f)||modview::ci_find(ais_navstatus(m.nav_status),f);
}
int col_cmp(int c, const AisRecord& a, const AisRecord& b){
    switch(c){
        case 0:  return a.t_ms<b.t_ms?-1:(a.t_ms>b.t_ms?1:0);
        case 1:  return a.mmsi<b.mmsi?-1:(a.mmsi>b.mmsi?1:0);
        case 2:  return a.msg_type-b.msg_type;
        case 3:  return strcmp(a.name,b.name);
        case 4:  return a.lat<b.lat?-1:(a.lat>b.lat?1:0);
        case 5:  return a.lon<b.lon?-1:(a.lon>b.lon?1:0);
        case 6:  return a.sog<b.sog?-1:(a.sog>b.sog?1:0);
        case 7:  return a.cog<b.cog?-1:(a.cog>b.cog?1:0);
        default: return a.nav_status-b.nav_status;
    }
}
std::string msg_key(const AisRecord& m){
    char b[48]; snprintf(b,sizeof(b),"%lld|%u|%d",(long long)m.t_ms,m.mmsi,m.msg_type);
    return std::string(b);
}
// Info 컬럼/복사용 한 줄 요약
void info_str(const AisRecord& m, char* o, size_t n){
    if(m.nav_status>=0) snprintf(o,n,"%s",ais_navstatus(m.nav_status));
    else if(m.ship_type>0){ const char* st=ais_shiptype(m.ship_type);
        if(m.callsign[0]) snprintf(o,n,"%s  %s",m.callsign,st); else snprintf(o,n,"%s",st); }
    else if(m.callsign[0]) snprintf(o,n,"%s",m.callsign);
    else o[0]=0;
}
// MMSI별 최신 head + 항적 꼬리 (지도용)
struct TrailPt { float lat, lon; int64_t t_ms; };   // t_ms 는 int64 (float 로 epoch 분 저장 시 ULP≈4분 → prune 오작동)
struct AisTrack { AisRecord head; std::deque<TrailPt> trail; int ship_type=0; char name[24]={0}; };
// ── MMSI 그룹 (표: 동일 MMSI 묶음, 최신값 + Up/Down/Cnt) ──
struct AisGrp {
    uint32_t mmsi;
    int      cnt;          // 수신 누계 (레코드수 집계; auth_cnt>0 이면 그 값으로 대체)
    uint32_t auth_cnt=0;   // Central 요약이 실어보낸 실제 누계 (요약 rx_cnt) — 있으면 권위
    int64_t  first;        // Up — 최초 수신(불변)
    int64_t  last;         // Down — 최근 수신(갱신)
    AisRecord latest;      // last 시점의 최신 레코드 (Type/Lat/Lon/SOG/COG/Info)
    char     name[21]={};  // 최신 비공백 선박명
    uint8_t  ai_status=0;  // Match_AI (최신 non-zero; sticky)
    uint32_t ai_mmsi=0;
    uint16_t ai_conf=0;    // decipercent (percent*10)
    uint8_t  anom_flag=0;  // Behavior 이상탐지 (최신 non-zero; sticky)
    uint8_t  anom_reason=0;
    uint16_t anom_score=0;
    int      ship_type=0;  // sticky (static msg 5/24) — 지도 마커 색용
};
// 컬럼: 0 Up 1 Down 2 MMSI 3 Type 4 Name 5 Country 6 Lat 7 Lon 8 SOG 9 COG 10 Cnt [11 Match_AI] Behavior Info
int grp_cmp(int c, const AisGrp& a, const AisGrp& b){
    switch(c){
        case 0:  return a.first<b.first?-1:(a.first>b.first?1:0);
        case 1:  return a.last<b.last?-1:(a.last>b.last?1:0);
        case 2:  return a.mmsi<b.mmsi?-1:(a.mmsi>b.mmsi?1:0);
        case 3:  return a.latest.msg_type-b.latest.msg_type;
        case 4:  return strcmp(a.name,b.name);
        case 5:  return strcmp(ais_mid_country(a.mmsi),ais_mid_country(b.mmsi));
        case 6:  return a.latest.lat<b.latest.lat?-1:(a.latest.lat>b.latest.lat?1:0);
        case 7:  return a.latest.lon<b.latest.lon?-1:(a.latest.lon>b.latest.lon?1:0);
        case 8:  return a.latest.sog<b.latest.sog?-1:(a.latest.sog>b.latest.sog?1:0);
        case 9:  return a.latest.cog<b.latest.cog?-1:(a.latest.cog>b.latest.cog?1:0);
        case 10: return a.cnt-b.cnt;
#ifdef BEWE_MODULE_AIS_AI
        case 11: return a.ai_mmsi<b.ai_mmsi?-1:(a.ai_mmsi>b.ai_mmsi?1:0);
#endif
        default:
            if(c==AIS_TBL_BEHAV){   // Behavior: flag 우선, 동급이면 score 순
                if(a.anom_flag!=b.anom_flag) return a.anom_flag-b.anom_flag;
                return a.anom_score<b.anom_score?-1:(a.anom_score>b.anom_score?1:0);
            }
            return a.latest.nav_status-b.latest.nav_status;   // Info
    }
}
// 선종(ITU shiptype)별 마커 색 — 화물/유조/여객/어선 등 한눈 구분
ImU32 type_color(int t){
    if(t>=60&&t<=69) return IM_COL32(120,200,255,255);  // passenger 파랑
    if(t>=70&&t<=79) return IM_COL32(110,230,140,255);  // cargo 초록
    if(t>=80&&t<=89) return IM_COL32(255,160, 90,255);  // tanker 주황
    if(t>=30&&t<=39) return IM_COL32(235,215,120,255);  // fishing/특수 노랑
    if(t>=40&&t<=59) return IM_COL32(200,150,255,255);  // 고속/특수 보라
    return IM_COL32(90,200,255,255);                    // 미상 시안
}
// "YYYYMMDD" y/m/d → 그날 00:00 KST 의 epoch ms (타임라인 하루축 기준점)
int64_t kst_day_base_ms(int y,int mo,int d){
    struct tm tv{}; tv.tm_year=y-1900; tv.tm_mon=mo-1; tv.tm_mday=d;
    time_t utc = timegm(&tv);                     // 그 날짜 00:00 을 UTC 로 해석
    return ((int64_t)utc - 9*3600) * 1000;        // KST(UTC+9) 자정 = UTC-9h
}
// 재생용 위치 표본 (MMSI별 시간순)
struct PosPt { int64_t t; float lat, lon, cog, hdg, sog; };
} // anonymous

void draw_content(FFTViewer& v, bool just_opened){
    ImGuiIO& io = ImGui::GetIO();
    bool remote = bewe_mod_my_station()[0] != 0;
    if(just_opened && !remote) local_load_today(v);

    ImVec2 avail = ImGui::GetContentRegionAvail();
    float W=avail.x, H=avail.y, x0=ImGui::GetCursorPosX(), y0=ImGui::GetCursorPosY();
    bool win_focus = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
#ifdef BEWE_MODULE_GUARD
    // guard 자동구독: AIS 구독 중이면 경보 스트림도 따라 켬 — 지도 카드/링/행 강조가 바로 뜬다.
    if(remote && bewe_mod_recv("ais") && !bewe_mod_recv("guard")) bewe_mod_set_recv(v, "guard", true);
    // G키: 데모 온/오프 토글 (창 포커스 + 텍스트입력 아님; 기본 OFF)
    if(win_focus && !io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_G,false)) guard_mod::demo_toggle();
#endif

    static std::set<std::string> sel; static std::string anchor;
    static AisRecord focus{}; static bool has_focus=false;
    static int sort_col=-1; static bool sort_asc=true;
    static bool atb=true; static size_t lastn=0;
    static uint32_t map_pin=0;                 // 지도에서 핀(필터)된 MMSI (0=없음)
    static uint32_t sel_mmsi=0;                // 표/지도에서 선택된 MMSI (0=없음) — 강조+full꼬리 공통
    static std::string sel_station;            // 클릭된 수신소 이름 (그 기지 수신선박 점선; 빈=없음)
    static std::set<uint32_t> vloaded;         // JOIN: 전체 이력 온디맨드 이미 요청한 MMSI (중복 요청 방지)
    // 과거조회(Hist) 모드 전환 감지 → 선택/캐시 리셋 (log 통째 교체되므로)
    char hdate[9]={}; bool histm = remote && bewe_mod_hist_mode("ais", hdate);
    { static bool prev_hist=false;
      if(histm!=prev_hist){ prev_hist=histm; vloaded.clear(); sel.clear(); anchor.clear();
                            has_focus=false; sel_mmsi=0; map_pin=0; } }
    // 필터 박스를 수동으로 바꾸면 핀 해제 (지도 토글 일관성)
    if(map_pin){ char ms[16]; snprintf(ms,sizeof(ms),"%u",map_pin); if(strcmp(filter,ms)!=0) map_pin=0; }

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; vloaded.clear(); };

    // 스페이스바 Recv 토글 + Ctrl+F/Tab 필터 포커스
    modview::space_toggle_recv(v, "ais", remote, win_focus, on_clear);
    bool focus_filter = win_focus && !io.WantTextInput &&
        ((io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F,false)) || ImGui::IsKeyPressed(ImGuiKey_Tab,false));

    // Ctrl+C → 선택 행 복사 (탭 구분, 한 행당 한 줄)
    if(win_focus && !io.WantTextInput && io.KeyCtrl && !io.KeyShift &&
       ImGui::IsKeyPressed(ImGuiKey_C,false) && !sel.empty()){
        std::string out; std::lock_guard<std::mutex> lk(mtx);
        for(const AisRecord& m : log){ if(!sel.count(msg_key(m))) continue;
            char ts[12]; hms(m.t_ms,ts); char inf[64]; info_str(m,inf,sizeof(inf));
            char lat[16]={0},lon[16]={0},sog[12]={0},cog[12]={0};
            if(m.has_pos){ snprintf(lat,sizeof(lat),"%.5f",m.lat); snprintf(lon,sizeof(lon),"%.5f",m.lon); }
            if(m.sog>=0) snprintf(sog,sizeof(sog),"%.1f",m.sog);
            if(m.cog>=0) snprintf(cog,sizeof(cog),"%.0f",m.cog);
            char line[320];
            snprintf(line,sizeof(line),"%s\t%s\t%u\t%d\t%s\t%s\t%s\t%s\t%s\t%s\n",
                ts,m.station[0]?m.station:"LOCAL",m.mmsi,m.msg_type,m.name,lat,lon,sog,cog,inf);
            out+=line;
        }
        if(!out.empty()) ImGui::SetClipboardText(out.c_str());
    }

    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    // ── 뷰 네비게이션: 그룹목록(0) ↔ 선박이력(해당 MMSI). 스택 없이 현재 화면만 기억 ──
    // 목록에서든 지도에서든 배를 몇 번 갈아타든 뒤로가기는 항상 목록으로 1회 복귀.
    static uint32_t cur_view = 0;                    // 0=목록, else=MMSI 이력
    auto nav_go = [&](uint32_t mmsi){                // 선박 상세뷰로 이동
        // JOIN: 배 활성화 시 그 MMSI 전체 이력 온디맨드 다운로드 (구독 요약엔 최초/최근10분만 있음)
        // Hist 모드는 제외 — 과거 아카이브는 이미 전체 데이터라 요청 불필요 (요청하면 오늘 데이터가 섞임)
        if(remote && mmsi && !histm && !vloaded.count(mmsi)){ bewe_mod_req_vessel("ais", mmsi); vloaded.insert(mmsi); }
        cur_view = mmsi;
    };
    // 플레이백(타임라인) 표시 토글 — 기본 OFF. OFF면 지도/데이터가 하단까지 꽉 참.
    static bool tl_show=false; bool tl_toggle=false;
    bool nav_back=false;
    modview::header_bar(v, "ais", filter, sizeof(filter), total, remote, focus_filter, on_clear,
                        true, cur_view!=0, &nav_back,
                        true, tl_show, &tl_toggle);
    if(tl_toggle) tl_show=!tl_show;
    if(nav_back){ cur_view=0; sel_mmsi=0; map_pin=0; has_focus=false; }

    // ── 타임라인 스크러버 + 재생 (00~24시 하루축, 구간 A/B, Play) ────────────
    // 재생버튼(위)+배속입력칸(밑) 2단을 담아 TL_H 확정. 스크러버 축은 버튼단 중앙.
    const float TL_BH  = 18.f;      // 재생버튼 높이 (스크러버 축 기준)
    const float TL_IN_H= 22.f;      // 배속 입력칸(InputInt 프레임) 높이
    const float TL_H = TL_BH + 4.f + TL_IN_H;   // 타임라인 전체 세로 (=44)
    static float tl_a=0.f, tl_b=1440.f, tl_head=0.f;   // 분(0~1440)
    static bool  tl_play=false;
    static int   tl_spd=1;          // 재생 배속: 실1초당 진행할 데이터 분(min). 기본 1m, 정수·≥1
    // 하루 기준점: DB(과거)모드=그 날짜 / 라이브=오늘, KST 자정 epoch ms
    int64_t day_base;
    {
        int y,mo,d;
        if(histm && hdate[0]){
            y=(hdate[0]-'0')*1000+(hdate[1]-'0')*100+(hdate[2]-'0')*10+(hdate[3]-'0');
            mo=(hdate[4]-'0')*10+(hdate[5]-'0'); d=(hdate[6]-'0')*10+(hdate[7]-'0');
        } else {
            int64_t nowms=(int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            time_t tt=(time_t)(nowms/1000); struct tm kt; KST::to_tm(tt,kt);
            y=kt.tm_year+1900; mo=kt.tm_mon+1; d=kt.tm_mday;
        }
        day_base=kst_day_base_ms(y,mo,d);
    }
    // 기준일 변경 시 리셋 — 단 재생 중이면 진행 보존(자정 넘어가도 replay 안 끊김)
    { static int64_t prev_base=-1; if(day_base!=prev_base){ prev_base=day_base; if(!tl_play){ tl_a=0.f;tl_b=1440.f;tl_head=0.f; } } }
    if(!tl_show){ tl_play=false; tl_a=0.f; tl_b=1440.f; tl_head=0.f; }   // 플레이백 꺼짐 → 재생/구간 무효
    if(tl_play){ tl_head += io.DeltaTime * (float)tl_spd; if(tl_head>=tl_b){ tl_head=tl_b; tl_play=false; } }   // 실1초=데이터 tl_spd분
    // 지도 카메라/크게보기 상태 (아래 지도 렌더에서도 사용). big 여부가 body_h 배분에 필요.
    static modview_map::MapView mv;
    mv.show_all_trails = true;   // AIS: 선택 배가 있어도 전 선박 10분 꼬리 상시 표시
    // 표/지도 세로 배분: 헤더(32) 아래부터. 플레이백 켜지면 하단 TL_H 만큼 비워 타임라인 자리 확보.
    // 크게보기(big)면 하단 여백(4px) 제거 → 지도가 패널 맨 아래까지 꽉 참 (표는 숨겨져 영향 없음).
    float bot = mv.big ? 32.f : 36.f;
    float body_h = tl_show ? (H-bot-TL_H) : (H-bot); if(body_h<80) body_h=80;
    // 타임라인 바 UI — 지도/데이터 아래(하단바 바로 위)에 그린다. 켜졌을 때만.
    if(tl_show){
        const float PAD=12.f;     // 좌우 대칭 여백 (버튼 왼쪽 = 눈금 오른쪽)
        const float BH=TL_BH;      // 재생버튼 높이 (스크러버 축 기준)
        ImGui::SetCursorPos(ImVec2(x0+PAD, y0+32.f+body_h+4.f));
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float PB=28.f;
        if(ImGui::Button(tl_play?"##tlpause":"##tlplay", ImVec2(PB, BH))){
            if(tl_head>=tl_b-0.02f) tl_head=tl_a;      // 끝에서 재생 → 처음부터
            tl_play=!tl_play;
        }
        ImDrawList* dl = ImGui::GetWindowDrawList();
        { // Play/Pause 아이콘
            ImVec2 c(p0.x+PB*0.5f, p0.y+BH*0.5f); ImU32 ic=IM_COL32(220,230,245,255);
            if(tl_play){ dl->AddRectFilled(ImVec2(c.x-5,c.y-6),ImVec2(c.x-1,c.y+6),ic); dl->AddRectFilled(ImVec2(c.x+1,c.y-6),ImVec2(c.x+5,c.y+6),ic); }
            else dl->AddTriangleFilled(ImVec2(c.x-4,c.y-6),ImVec2(c.x-4,c.y+6),ImVec2(c.x+6,c.y),ic);
        }
        // 배속 입력칸 — 재생버튼 바로 밑, 폭=버튼폭(PB). 정수·기본 1m, min/sec.
        ImGui::SetCursorScreenPos(ImVec2(p0.x, p0.y+BH+4.f));
        ImGui::SetNextItemWidth(PB);
        if(ImGui::InputInt("##tlspd", &tl_spd, 0, 0)){ if(tl_spd<1) tl_spd=1; if(tl_spd>1440) tl_spd=1440; }
        float tx0=p0.x+PB+12, tx1=p0.x+W-2.f*PAD, tw_=tx1-tx0; if(tw_<20)tw_=20;   // p0.x=x0+PAD 이므로 창우측=p0.x-PAD+W, 여기서 우측여백 PAD → p0.x+W-2*PAD
        float ty=p0.y+BH*0.5f;
        auto m2x=[&](float m){ return tx0+tw_*m/1440.f; };
        auto x2m=[&](float x){ float m=(x-tx0)/tw_*1440.f; return m<0.f?0.f:(m>1440.f?1440.f:m); };
        dl->AddLine(ImVec2(tx0,ty),ImVec2(tx1,ty),IM_COL32(80,90,105,255),2.f);
        for(int hh=0;hh<=24;hh+=3){ float x=m2x(hh*60.f);   // 눈금선 3시간마다
            dl->AddLine(ImVec2(x,ty-4),ImVec2(x,ty+4),IM_COL32(110,120,135,255),1.f);
            if(hh%6==0){ char lb[4]; snprintf(lb,sizeof(lb),"%02d",hh);   // 숫자 라벨은 6시간마다
                dl->AddText(ImVec2(x-6,ty+7),IM_COL32(130,140,155,255),lb); }
        }
        float hi_min_v = tl_play? tl_head : tl_b;
        dl->AddRectFilled(ImVec2(m2x(tl_a),ty-3),ImVec2(m2x(hi_min_v),ty+3),IM_COL32(90,150,220,110));  // 선택밴드
        auto handle=[&](const char* hid,float& mv,float lo,float hi){
            float hx=m2x(mv);
            ImGui::SetCursorScreenPos(ImVec2(hx-6,ty-12));
            ImGui::InvisibleButton(hid,ImVec2(12,24));
            bool act=ImGui::IsItemActive();
            if(act){ float m=x2m(io.MousePos.x); if(m<lo)m=lo; if(m>hi)m=hi; mv=m; }  // clamp → 교차 방지(커서 이탈 없음)
            ImU32 c=act?IM_COL32(255,220,120,255):IM_COL32(205,215,232,255);
            dl->AddTriangleFilled(ImVec2(hx-5,ty-12),ImVec2(hx+5,ty-12),ImVec2(hx,ty-4),c);
            dl->AddLine(ImVec2(hx,ty-6),ImVec2(hx,ty+8),c,2.f);
            if(ImGui::IsItemHovered()||act) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        };
        handle("##tlA",tl_a, 0.f, tl_b);   handle("##tlB",tl_b, tl_a, 1440.f);   // A≤B 유지
        if(tl_head<tl_a)tl_head=tl_a; if(tl_head>tl_b)tl_head=tl_b;
        if(tl_play){ float hx=m2x(tl_head); dl->AddLine(ImVec2(hx,ty-13),ImVec2(hx,ty+13),IM_COL32(255,120,90,255),2.f); }
    }
    // 표/지도는 헤더 바로 아래(y0+32)부터 그린다 — 타임라인은 이 아래(하단)에 그려짐
    ImGui::SetCursorPos(ImVec2(x0, y0+32.f));
    // 재생/구간 필터 범위 (표·지도 공용). 플레이백 꺼짐이면 위에서 tl_* 리셋되어 필터 없음.
    bool    tl_filt = tl_play || tl_a>0.5f || tl_b<1439.5f;
    float   hi_min  = tl_play? tl_head : tl_b;
    int64_t lo_ms = day_base + (int64_t)(tl_a*60000.0f);
    int64_t hi_ms = day_base + (int64_t)(hi_min*60000.0f);

    // 세부패널 제거 — 표/지도 모두 body_h(헤더 아래, 플레이백 켜지면 하단 TL_H 뺀) 전체 높이 사용
    float upper_h = body_h; if(upper_h<80) upper_h=80;   // 표 높이
    float map_h   = body_h + 2.f; if(map_h<80) map_h=80; // 지도 높이 (기존과 동일 오프셋 유지)

    // ── 표(grps) + 지도(mlatest/mtrail) 공유 캐시 — 아래 한 번의 필터링 패스에서 함께 채움 (표⟺지도 100% 동기) ──
    static std::vector<AisGrp> grps;
    static std::set<std::string> rx_stations;      // 실제 AIS 복조한 기지 이름들 ("" = LOCAL)
    static float info_w = 60.f;                     // Info 컬럼 동적폭
    static std::unordered_map<uint32_t, AisRecord>            mlatest;  // MMSI별 최신 위치 레코드 (마커·정보)
    static std::unordered_map<uint32_t, std::vector<TrailPt>> mtrail;   // MMSI별 최근10분 위치점(int64) — 연속 꼬리
    static std::vector<modview_map::MapPoint> pts;
    static std::vector<std::vector<float>>    trailbuf;
    static std::vector<std::vector<int64_t>>  trailt;   // 선택 배 항적 점별 수신시각(ms) — hover 시각 툴팁
    static std::vector<std::string>           tip1, tip2;
    // (지도 pts 는 아래 grps/mlatest/mtrail 집계 직후 "지도 오버레이 빌드" 블록에서 채움)

    // ── MMSI 그룹 집계 (표: 동일 MMSI 묶음, Up/Down/Cnt + 최신값. 캐시 게이팅) ──
    // (grps/rx_stations/info_w 선언은 위 "공유 캐시" 블록으로 이동)
    {
        std::lock_guard<std::mutex> lk(mtx);
        static size_t c_n=(size_t)-1; static int64_t c_last=-1;
        static char c_filter[64]={'\xff'}; static int c_sc=-99; static bool c_asc=false;
        static int64_t c_lo=-1, c_hi=-1;
        // 게이팅은 분 단위로 양자화 — 재생 중 hi_ms 매프레임 변해도 재집계는 1분(=실1초)당 1회
        int64_t lo_q=(int64_t)tl_a, hi_q=(int64_t)hi_min;
        size_t n_now=log.size(); int64_t last_t=n_now?log.back().t_ms:0;
        bool datachg = (n_now!=c_n||last_t!=c_last);   // tl_filt(재생/구간) 중엔 라이브 변경 무시 = 스냅샷
        if((!tl_filt && datachg)||strncmp(c_filter,filter,sizeof(c_filter))||c_sc!=sort_col||c_asc!=sort_asc
           ||c_lo!=lo_q||c_hi!=hi_q){
            std::vector<AisGrp> g; g.reserve(256);
            std::unordered_map<uint32_t,int> idx; idx.reserve(512);
            std::unordered_map<uint32_t, AisRecord> ml; ml.reserve(512);
            std::unordered_map<uint32_t, std::vector<TrailPt>> mt; mt.reserve(512);
            int64_t data_now=0; for(size_t i=0;i<n_now;i++) if(log[i].t_ms>data_now) data_now=log[i].t_ms;  // 현재시각=최댓값 (log.back()은 vessel-hist append 로 재정렬돼 신뢰불가 → 클릭시 꼬리 전체 튐 방지)
            int64_t tcut = (tl_filt ? hi_ms : data_now) - 600000;   // 최근 10분 꼬리 창
            rx_stations.clear();
            for(size_t i=0;i<n_now;i++){
                const AisRecord& m=log[i];
                if(!match(m,filter)) continue;
                if(tl_filt && (m.t_ms<lo_ms||m.t_ms>hi_ms)) continue;   // 타임라인 구간 필터
                rx_stations.insert(m.station);   // 복조한 기지 수집 (빈문자=LOCAL)
                auto it=idx.find(m.mmsi); int gi;
                if(it==idx.end()){ gi=(int)g.size(); idx[m.mmsi]=gi;
                    AisGrp ng{}; ng.mmsi=m.mmsi; ng.first=m.t_ms; ng.last=-1; g.push_back(ng); }
                else gi=it->second;
                AisGrp& G=g[gi]; G.cnt++;
                if(m.rx_cnt>G.auth_cnt) G.auth_cnt=m.rx_cnt;   // Central 요약 실제 누계 (권위)
                if(m.t_ms<G.first) G.first=m.t_ms;          // Up = 최초(불변)
                if(m.t_ms>=G.last){ G.last=m.t_ms; G.latest=m; }   // Down = 최근
                if(m.ai_status){ G.ai_status=m.ai_status; G.ai_mmsi=m.ai_mmsi; G.ai_conf=m.ai_conf; }  // Match_AI sticky: non-zero 만 갱신
                if(m.anom_flag){ G.anom_flag=m.anom_flag; G.anom_reason=m.anom_reason; G.anom_score=m.anom_score; }  // Behavior sticky
                if(m.name[0] && !G.name[0]){ strncpy(G.name,m.name,sizeof(G.name)-1); G.name[sizeof(G.name)-1]=0; }
                if(m.ship_type>0) G.ship_type=m.ship_type;   // 지도 마커 색 sticky
                if(m.has_pos){ AisRecord& L=ml[m.mmsi];
                    if(L.t_ms==0 || m.t_ms>=L.t_ms) L=m;                                   // 최신 위치 레코드(마커)
                    if(m.t_ms>=tcut) mt[m.mmsi].push_back({(float)m.lat,(float)m.lon,m.t_ms}); }  // 10분 꼬리 점
            }
            for(AisGrp& G : g) if(G.auth_cnt) G.cnt=(int)G.auth_cnt;   // 요약 상태: 실제 누계로 표시/정렬 통일
            std::stable_sort(g.begin(),g.end(),[&](const AisGrp&a,const AisGrp&b){ int c=grp_cmp(sort_col<0?0:sort_col,a,b); return (sort_col<0?true:sort_asc)? c<0:c>0; });
            grps.swap(g); mlatest.swap(ml); mtrail.swap(mt);
            c_n=n_now; c_last=last_t; strncpy(c_filter,filter,sizeof(c_filter)-1); c_filter[sizeof(c_filter)-1]=0; c_sc=sort_col; c_asc=sort_asc; c_lo=lo_q; c_hi=hi_q;
            // Info 폭 = 실제 데이터 최대 길이에 맞춤 (헤더 글자폭 무시)
            float mw=8.f;
            for(const AisGrp& G : grps){ char inf[64]; info_str(G.latest,inf,sizeof(inf));
                if(inf[0]){ float w=ImGui::CalcTextSize(inf).x; if(w>mw) mw=w; } }
            info_w = mw + 10.f;
        }
    }

    // ── 지도 오버레이 빌드: grps(표)와 같은 필터/타임라인 윈도우 → 표에 뜬 배만 지도에 (100% 동기) ──
    //    마커=mlatest[최신 위치], 꼬리=mtrail[최근10분 전점]. 선택 배는 log 전체(전 항적).
    {
        pts.clear(); trailbuf.clear(); trailt.clear(); tip1.clear(); tip2.clear();
        for(const AisGrp& G : grps){
            auto lit = mlatest.find(G.mmsi);
            if(lit==mlatest.end()) continue;                  // 최근 위치 없음(정적 only) → 마커 못 그림
            const AisRecord& m = lit->second;
            modview_map::MapPoint mpt;
            mpt.lat=m.lat; mpt.lon=m.lon; mpt.id=G.mmsi;
            mpt.heading = (m.cog>=0.f)? m.cog : (m.heading!=511? (float)m.heading : -1.f);
            mpt.selected = (sel_mmsi==G.mmsi);
            mpt.color = mpt.selected? IM_COL32(255,210,80,255) : type_color(G.ship_type);
            mpt.label = G.name[0]? G.name : nullptr;
            trailbuf.emplace_back(); trailt.emplace_back();
            if(sel_mmsi==G.mmsi){                             // 선택 배 = 전체 항적(윈도우 무시) + 점별 수신시각
                std::lock_guard<std::mutex> lk(mtx);
                for(const AisRecord& r : log)
                    if(r.mmsi==G.mmsi && r.has_pos){ trailbuf.back().push_back((float)r.lat); trailbuf.back().push_back((float)r.lon); trailt.back().push_back(r.t_ms); }
            } else {
                auto tit = mtrail.find(G.mmsi);               // 최근 10분 연속 꼬리(전점)
                if(tit!=mtrail.end()) for(const TrailPt& p : tit->second){ trailbuf.back().push_back(p.lat); trailbuf.back().push_back(p.lon); }
            }
            char b1[32],b2[96],sog[16],cog[16],hdg[16];
            snprintf(b1,sizeof(b1),"%u",G.mmsi);
            if(m.sog>=0) snprintf(sog,sizeof(sog),"%.1f kt",m.sog); else snprintf(sog,sizeof(sog),"-");
            if(m.cog>=0) snprintf(cog,sizeof(cog),"%.1f°",m.cog); else snprintf(cog,sizeof(cog),"-");
            if(m.heading!=511) snprintf(hdg,sizeof(hdg),"%d°",m.heading); else snprintf(hdg,sizeof(hdg),"-");
            snprintf(b2,sizeof(b2),"SOG : %s\nCOG : %s\nHDG : %s",sog,cog,hdg);
            tip1.emplace_back(b1); tip2.emplace_back(b2);
            pts.push_back(mpt);
        }
        for(size_t i=0;i<pts.size();i++){
            pts[i].trail   = trailbuf[i].empty()? nullptr : trailbuf[i].data();
            pts[i].trail_n = (int)trailbuf[i].size()/2;
            pts[i].trail_t = trailt[i].empty()? nullptr : trailt[i].data();
            pts[i].tip_l1  = tip1[i].c_str();
            pts[i].tip_l2  = tip2[i].empty()? nullptr : tip2[i].c_str();
        }
#ifdef BEWE_MODULE_GUARD
        // 경보 선박 마커를 뒤로 정렬 → 밀집 해역에서도 경보 배가 위에 그려짐
        std::stable_partition(pts.begin(), pts.end(), [](const modview_map::MapPoint& p){
            uint8_t gt, gs; return !guard_mod::vessel_alert((uint32_t)p.id, gt, gs); });
#endif
    }

    // ── 좌(표) | 우(지도) ──  (지도 크게보기 mv.big 면 표 숨기고 지도 전폭; mv 는 위에서 선언)
    static float split_tw=-1.f;   // 사용자가 스플리터로 정한 표 폭(px). <0 = 미설정(컬럼합 자동)
    float tw, mapw;
    if(mv.big){ tw=0.f; mapw=W; }
    else {
        const float FIXED_COLS = 70+70+82+40+130+64+78+84+48+48+46  // Up Down MMSI Type Name Country Lat Lon SOG COG Cnt
#ifdef BEWE_MODULE_AIS_AI
            +96                                                     // Match_AI (모듈 있을 때만)
#endif
            +96                                                     // Behavior (항상)
            ;
        // 컬럼 cell padding(~8px each) + inner border + 세로스크롤바(~14). 이만큼만 더해
        // Info 셀 뒤 빈 여백 없이 마지막 Info 가 데이터폭에서 끝나게.
        float auto_tw = FIXED_COLS + info_w + AIS_TBL_NCOL*8.f + (float)AIS_TBL_NCOL;
        tw = (split_tw>0.f) ? split_tw : auto_tw;       // 드래그로 조절했으면 그 값
        float maxtw = W-50; if(tw>maxtw) tw=maxtw; if(tw<200) tw=200;
        mapw=W-tw-6; if(mapw<10)mapw=10;                // 6px = 스플리터
    }

    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(!mv.big && cur_view==0 && ImGui::BeginTable("##ais_tbl", AIS_TBL_NCOL, tf, ImVec2(tw, upper_h))){
        ImGui::TableSetupScrollFreeze(3,1);
        ImGui::TableSetupColumn("Up",      ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Down",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("MMSI",    ImGuiTableColumnFlags_WidthFixed, 82);
        ImGui::TableSetupColumn("Type",    ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Name",    ImGuiTableColumnFlags_WidthFixed, 130);
        ImGui::TableSetupColumn("Country", ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("Lat",     ImGuiTableColumnFlags_WidthFixed, 78);
        ImGui::TableSetupColumn("Lon",     ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("SOG",     ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("COG",     ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("Cnt",     ImGuiTableColumnFlags_WidthFixed, 46);
#ifdef BEWE_MODULE_AIS_AI
        ImGui::TableSetupColumn("Match_AI",ImGuiTableColumnFlags_WidthFixed, 96);  // DL 지문 예측 MMSI+% (AI 모듈)
#endif
        ImGui::TableSetupColumn("Behavior",ImGuiTableColumnFlags_WidthFixed, 96);  // 이상탐지 규칙층 (GAP/JUMP/SPEED)
        ImGui::TableSetupColumn("Info",    ImGuiTableColumnFlags_WidthStretch);  // 남는 폭 흡수 → Info 뒤 빈 칸 없음
        modview::sortable_headers(AIS_TBL_NCOL, sort_col, sort_asc, AIS_TBL_INFO);  // Info 만 좌측, 나머지 중앙

        ImGuiListClipper clip; clip.Begin((int)grps.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const AisGrp& G=grps[r];
            const AisRecord& m=G.latest;
            ImGui::TableNextRow();
#ifdef BEWE_MODULE_GUARD
            { uint8_t gt,gs;                                     // 경보 선박 행 강조 (sev 색)
              if(guard_mod::vessel_alert(G.mmsi,gt,gs))
                  ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                      gs>=3? IM_COL32(120,32,26,95) : gs==2? IM_COL32(115,78,22,85) : IM_COL32(105,96,26,70)); }
#endif
            char up[12]; hms(G.first,up);
            bool selrow = (sel_mmsi==G.mmsi);
            if(modview::row_col0(r, selrow, up)){               // col0 = Up time
                sel_mmsi=G.mmsi; map_pin=G.mmsi;                // 지도 핀(필터링 안 함)
                focus=m; has_focus=true;
                nav_go(G.mmsi);                                 // 그 선박 전체 이력 화면으로
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); { char dn[12]; hms(G.last,dn); modview::cell(dn); }
            ImGui::TableSetColumnIndex(2); snprintf(b,sizeof(b),"%u",G.mmsi); modview::cell(b);
            ImGui::TableSetColumnIndex(3); snprintf(b,sizeof(b),"%d",m.msg_type); modview::cell(b);
            ImGui::TableSetColumnIndex(4);
            if(G.name[0]) modview::cell(G.name); else { const char* cc=ais_mid_country(G.mmsi); if(cc[0]) modview::cell(cc, ImVec4(0.6f,0.6f,0.6f,1.f)); }
            ImGui::TableSetColumnIndex(5); { const char* cc=ais_mid_country(G.mmsi); if(cc[0]) modview::cell(cc, ImVec4(0.6f,0.6f,0.6f,1.f)); }
            ImGui::TableSetColumnIndex(6); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lat); modview::cell(b); }
            ImGui::TableSetColumnIndex(7); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lon); modview::cell(b); }
            ImGui::TableSetColumnIndex(8); if(m.sog>=0){ snprintf(b,sizeof(b),"%.1f",m.sog); modview::cell(b); }
            ImGui::TableSetColumnIndex(9); if(m.cog>=0){ snprintf(b,sizeof(b),"%.0f",m.cog); modview::cell(b); }
            ImGui::TableSetColumnIndex(10); snprintf(b,sizeof(b),"%d",G.cnt); modview::cell(b);
#ifdef BEWE_MODULE_AIS_AI
            ImGui::TableSetColumnIndex(11);   // Match_AI: DL 지문 예측 (불일치=빨강, 불확실=UNKNOWN)
            if(G.ai_status==2){ snprintf(b,sizeof(b),"%u %.1f%%",G.ai_mmsi,G.ai_conf/10.0);
                modview::cell(b, G.ai_mmsi==G.mmsi? ImVec4(0.55f,0.8f,0.55f,1.f):ImVec4(1.f,0.5f,0.4f,1.f)); }
            else if(G.ai_status==1) modview::cell("UNKNOWN", ImVec4(0.75f,0.72f,0.5f,1.f));
            else modview::cell("-", ImVec4(0.4f,0.4f,0.4f,1.f));
#endif
            ImGui::TableSetColumnIndex(AIS_TBL_BEHAV);   // Behavior: 이상탐지 규칙 발화 (alert=빨강, watch=앰버)
            if(G.anom_flag==2){ snprintf(b,sizeof(b),"ALERT %s",ais_anom_reason(G.anom_reason));
                modview::cell(b, ImVec4(1.f,0.4f,0.35f,1.f)); }
            else if(G.anom_flag==1){ snprintf(b,sizeof(b),"watch %s",ais_anom_reason(G.anom_reason));
                modview::cell(b, ImVec4(0.9f,0.7f,0.35f,1.f)); }
            else modview::cell("-", ImVec4(0.4f,0.4f,0.4f,1.f));
            ImGui::TableSetColumnIndex(AIS_TBL_INFO); { char inf[64]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        ImGui::EndTable();
    }
    // ── 선박 이력 뷰: 선택 MMSI 의 Up~Down 사이 모든 메시지 (시간순) ──
    // 기록별 히스토리 테이블. Match_AI 열은 AI 모듈 있을 때만, Behavior 는 항상 → 인덱스 가변.
#ifdef BEWE_MODULE_AIS_AI
    #define AIS_HIST_NCOL  9
    #define AIS_HIST_BEHAV 7   // Match_AI(6) 다음
    #define AIS_HIST_INFO  8
#else
    #define AIS_HIST_NCOL  8
    #define AIS_HIST_BEHAV 6
    #define AIS_HIST_INFO  7
#endif
    else if(!mv.big && cur_view!=0 && ImGui::BeginTable("##ais_hist", AIS_HIST_NCOL, tf, ImVec2(tw, upper_h))){
        ImGui::TableSetupScrollFreeze(1,1);
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Lat",  ImGuiTableColumnFlags_WidthFixed, 78);
        ImGui::TableSetupColumn("Lon",  ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("SOG",  ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("COG",  ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("HDG",  ImGuiTableColumnFlags_WidthFixed, 44);
#ifdef BEWE_MODULE_AIS_AI
        ImGui::TableSetupColumn("Match_AI", ImGuiTableColumnFlags_WidthFixed, 96);  // 기록별 DL 지문 예측
#endif
        ImGui::TableSetupColumn("Behavior", ImGuiTableColumnFlags_WidthFixed, 96);  // 기록별 이상탐지 규칙
        ImGui::TableSetupColumn("Info", ImGuiTableColumnFlags_WidthStretch);
        static int hsc=-1; static bool hsa=true;
        modview::sortable_headers(AIS_HIST_NCOL, hsc, hsa, AIS_HIST_INFO);   // Info 만 좌측, 나머지 중앙

        std::lock_guard<std::mutex> lk(mtx);
        static std::vector<int> hvis; hvis.clear();
        for(int i=0;i<(int)log.size();i++) if(log[i].mmsi==cur_view &&
               (!tl_filt || (log[i].t_ms>=lo_ms && log[i].t_ms<=hi_ms))) hvis.push_back(i);

        ImGuiListClipper hc; hc.Begin((int)hvis.size());
        while(hc.Step()) for(int r=hc.DisplayStart;r<hc.DisplayEnd;r++){
            const AisRecord& m=log[hvis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            char b[24];
            ImGui::TableSetColumnIndex(0); modview::cell(ts);   // Time 중앙정렬
            ImGui::TableSetColumnIndex(1); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lat); modview::cell(b); }
            ImGui::TableSetColumnIndex(2); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lon); modview::cell(b); }
            ImGui::TableSetColumnIndex(3); if(m.sog>=0){ snprintf(b,sizeof(b),"%.1f",m.sog); modview::cell(b); }
            ImGui::TableSetColumnIndex(4); if(m.cog>=0){ snprintf(b,sizeof(b),"%.0f",m.cog); modview::cell(b); }
            ImGui::TableSetColumnIndex(5); if(m.heading!=511){ snprintf(b,sizeof(b),"%d",m.heading); modview::cell(b); }
#ifdef BEWE_MODULE_AIS_AI
            ImGui::TableSetColumnIndex(6);   // Match_AI: 이 기록의 DL 지문 예측 (불일치=빨강, 불확실=UNKNOWN)
            if(m.ai_status==2){ snprintf(b,sizeof(b),"%u %.1f%%",m.ai_mmsi,m.ai_conf/10.0);
                modview::cell(b, m.ai_mmsi==m.mmsi? ImVec4(0.55f,0.8f,0.55f,1.f):ImVec4(1.f,0.5f,0.4f,1.f)); }
            else if(m.ai_status==1) modview::cell("UNKNOWN", ImVec4(0.75f,0.72f,0.5f,1.f));
            else modview::cell("-", ImVec4(0.4f,0.4f,0.4f,1.f));
#endif
            ImGui::TableSetColumnIndex(AIS_HIST_BEHAV);   // Behavior: 이 기록의 이상탐지 규칙 발화
            if(m.anom_flag==2){ snprintf(b,sizeof(b),"ALERT %s",ais_anom_reason(m.anom_reason));
                modview::cell(b, ImVec4(1.f,0.4f,0.35f,1.f)); }
            else if(m.anom_flag==1){ snprintf(b,sizeof(b),"watch %s",ais_anom_reason(m.anom_reason));
                modview::cell(b, ImVec4(0.9f,0.7f,0.35f,1.f)); }
            else modview::cell("-", ImVec4(0.4f,0.4f,0.4f,1.f));
            ImGui::TableSetColumnIndex(AIS_HIST_INFO); { char inf[64]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        modview::tail_follow(atb, false);
        ImGui::EndTable();
    }
#undef AIS_HIST_NCOL
#undef AIS_HIST_BEHAV
#undef AIS_HIST_INFO

    // ── 수신소(기지) 마커: 지구본과 동일하게 discovered_stations 전부 + 내 위치(HOST) 오버레이 ──
    static std::vector<modview_map::MapStation> stns;
    static std::vector<std::string> stn_names;   // 라벨 문자열 수명 보관
    stns.clear(); stn_names.clear();
    {
        // 한국 운용 전제 좌표 정규화 — 부호반전/음수 lon 보정 (서경으로 저장된 값 방어)
        auto norm=[](float& la, float& lo){
            if(lo<0.f) lo=-lo;       // 서경(-128) → 동경(128)
            if(la<0.f) la=-la;
        };
        // AIS 를 실제 복조한 기지(rx_stations)만, station_geo_cache(로그인때 받아 보관)서 좌표 조회.
        std::lock_guard<std::mutex> ck(v.station_geo_cache_mtx);
        for(const std::string& sn : rx_stations){
            std::string key = (sn.empty()||sn=="LOCAL") ? v.station_name : sn;
            float la=0.f, lo=0.f; bool got=false;
            auto it=v.station_geo_cache.find(key);
            if(it!=v.station_geo_cache.end()){ la=it->second.first; lo=it->second.second; got=true; }
            else if((sn.empty()||sn=="LOCAL") && (v.station_lat!=0.f||v.station_lon!=0.f)){
                la=v.station_lat; lo=v.station_lon; got=true;   // 내 위치 폴백
            }
            if(!got || (la==0.f&&lo==0.f)) continue;
            norm(la,lo);
            stn_names.push_back(key.empty()? sn : key);
            modview_map::MapStation ms; ms.lat=la; ms.lon=lo;
            ms.selected = (!sel_station.empty() && (key==sel_station || sn==sel_station));
            stns.push_back(ms);
        }
        for(size_t i=0;i<stns.size();i++) stns[i].name=stn_names[i].c_str();
    }

    // 선택 기지가 수신한 선박 점선 (기지좌표 → 각 선박 head)
    static std::vector<modview_map::MapLink> links;
    links.clear();
    if(!sel_station.empty()){
        // 선택 기지 좌표
        double slat=0,slon=0; bool sgot=false;
        for(size_t i=0;i<stns.size();i++) if(stns[i].selected){ slat=stns[i].lat; slon=stns[i].lon; sgot=true; break; }
        if(sgot){
            std::lock_guard<std::mutex> lk(mtx);
            std::set<uint32_t> rxmmsi;   // 그 기지가 수신한 MMSI
            for(const AisRecord& m : log){
                std::string st = (m.station[0] && strcmp(m.station,"LOCAL")) ? m.station : v.station_name;
                if(st==sel_station) rxmmsi.insert(m.mmsi);
            }
            for(const auto& mp : pts)
                if(rxmmsi.count((uint32_t)mp.id)){
                    double la=mp.lat, lo=mp.lon; if(lo<0)lo=-lo; if(la<0)la=-la;
                    links.push_back({slat,slon, la,lo});
                }
        }
    }

    // ── 스플리터 (표↔지도 크기 조절; 크게보기 아닐 때만) ──
    if(!mv.big){
        ImGui::SameLine(0,0);
        ImGui::InvisibleButton("##ais_split", ImVec2(6, map_h));
        bool sphov=ImGui::IsItemHovered(), spact=ImGui::IsItemActive();
        if(spact){ float ntw=(split_tw>0.f?split_tw:tw)+io.MouseDelta.x;
                   if(ntw<200)ntw=200; if(ntw>W-50)ntw=W-50; split_tw=ntw; }
        if(sphov||spact){
            ImVec2 a=ImGui::GetItemRectMin(), b=ImGui::GetItemRectMax();
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2((a.x+b.x)*0.5f-1,a.y),ImVec2((a.x+b.x)*0.5f+1,b.y),IM_COL32(120,140,160,200));
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        }
        ImGui::SameLine(0,0);
    } else {
        ImGui::SetCursorPosX(x0);
    }
#ifdef BEWE_MODULE_GUARD
    ImVec2 map_p0 = ImGui::GetCursorScreenPos();   // draw_map 캔버스 좌상단 (투영 기준점)
#endif
    auto mres = modview_map::draw_map("##ais_map", mv, pts, ImVec2(mapw, map_h), just_opened, &stns, &links);
#ifdef BEWE_MODULE_GUARD
    // ── GUARD 오버레이: 위험구역 폴리곤 + 데모항적 + 경보 펄스 링 (전부 지도 위) ──
    // 유형색: 충돌=빨강 좌초=주황 이상항적=노랑 위협=보라 (카드/링/구역 공용)
    auto guard_typ_col=[](uint8_t t)->ImU32{
        switch(t){ case 1: return IM_COL32(255, 97, 82,0);
                   case 2: return IM_COL32(255,158, 64,0);
                   case 3: return IM_COL32(242,217, 89,0);
                   case 4: return IM_COL32(204,140,255,0);
                   default:return IM_COL32(115,179,255,0); } };
    static std::vector<guard_mod::AlertRow> galerts;   // 이번 프레임 경보 스냅샷 (카드/링/상세 공용)
    galerts = guard_mod::snapshot();
    // 플레이백/Hist 중엔 데몬(원격) 라이브 경보는 그 시각과 무관 → 로컬(구역·플레이백계산)만 표시
    if(tl_filt) galerts.erase(std::remove_if(galerts.begin(),galerts.end(),
        [](const guard_mod::AlertRow& r){ return strcmp(r.a.station,"LOCAL")!=0; }), galerts.end());
    // GUARD 평가 (1초 주기): 구역 진입은 라이브·플레이백 공통. 충돌·위조는 플레이백에서 GUI 계산.
    {
        static int64_t last_eval=0; static bool prev_pb=false;
        int64_t nowm=(int64_t)(ImGui::GetTime()*1000);
        if(prev_pb && !tl_filt) guard_mod::clear_playback_alerts();   // 플레이백→라이브: PB경보 정리
        prev_pb = tl_filt;
        if(nowm-last_eval>=1000){
            last_eval=nowm;
            std::vector<guard_mod::VesselSnap> vs; vs.reserve(pts.size());
            for(const auto& mp : pts){ guard_mod::VesselSnap s{};
                s.mmsi=(uint32_t)mp.id; s.lat=mp.lat; s.lon=mp.lon; s.sog=-1; s.cog=-1;
                vs.push_back(s); }
            // sog/cog/ai: 라이브=최신, 플레이백=재생커서(hi_ms) 이하 가장 최근 레코드
            int64_t cut = tl_filt ? hi_ms : (int64_t)4e18;
            { std::lock_guard<std::mutex> lk(mtx);
              for(auto& s : vs){ for(auto it=log.rbegin(); it!=log.rend(); ++it){
                    if(it->mmsi!=s.mmsi || it->t_ms>cut) continue;
                    s.sog=it->sog; s.cog=it->cog; s.ai_status=it->ai_status; s.ai_mmsi=it->ai_mmsi; break; } } }
            guard_mod::eval_local_zones(vs, wall_now_ms());
            if(tl_filt) guard_mod::eval_playback_alerts(vs, wall_now_ms());
        }
    }
    {
        ImDrawList* gdl = ImGui::GetWindowDrawList();
        gdl->PushClipRect(map_p0, ImVec2(map_p0.x+mapw, map_p0.y+map_h), true);
        auto ll2px=[&](double lat,double lon){                 // map_view.cpp 와 동일 등거리원통 투영
            return ImVec2((float)(map_p0.x+(lon-mv.lon0)/(mv.lon1-mv.lon0)*mapw),
                          (float)(map_p0.y+(mv.lat1-lat)/(mv.lat1-mv.lat0)*map_h)); };
        auto px2ll=[&](ImVec2 p, double& lat, double& lon){
            lon = mv.lon0 + (double)(p.x-map_p0.x)/mapw*(mv.lon1-mv.lon0);
            lat = mv.lat1 - (double)(p.y-map_p0.y)/map_h*(mv.lat1-mv.lat0); };
        float tnow=(float)ImGui::GetTime();
        auto kind_col=[](uint8_t k)->ImU32{                    // 위험수준 색 (구역 공용)
            return k>=3? IM_COL32(255, 80, 70,0) : k==2? IM_COL32(255,158, 64,0)
                                                        : IM_COL32(242,217, 89,0); };
        // ── 로컬 위험구역 렌더 (박스 채움+외곽+이름) + 삭제 hover ──
        static int lz_del=-1; lz_del=-1;
        { auto lzs=guard_mod::local_zones();
          for(size_t i=0;i<lzs.size();i++){
            const auto& z=lzs[i]; ImU32 zc=kind_col(z.kind);
            ImVec2 a=ll2px(z.lat1,z.lon0), b=ll2px(z.lat0,z.lon1);   // 좌상~우하
            gdl->AddRectFilled(a,b, zc|(30u<<24), 0);
            gdl->AddRect(a,b, zc|(210u<<24), 0, 0, 1.8f);
            gdl->AddText(ImVec2(a.x+5,a.y+4), zc|(235u<<24), z.name);
            bool hov = io.MousePos.x>=a.x&&io.MousePos.x<=b.x&&io.MousePos.y>=a.y&&io.MousePos.y<=b.y;
            if(hov){                                            // 우상단 [x] 삭제
                ImVec2 xc(b.x-11,a.y+9);
                gdl->AddText(ImVec2(xc.x-3,xc.y-7), IM_COL32(255,120,120,255), "x");
                if(io.MousePos.x>=xc.x-8&&io.MousePos.x<=xc.x+8&&io.MousePos.y>=xc.y-8&&io.MousePos.y<=xc.y+8
                   && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) lz_del=(int)i;
            }
          }
          if(lz_del>=0) guard_mod::del_local_zone((uint32_t)lz_del);
        }
        // ── 우클릭 드래그 → 신규 구역 박스 (놓으면 이름·색 입력 팝업) ──
        static bool drag=false; static ImVec2 dstart;
        static double nz_la0,nz_la1,nz_lo0,nz_lo1; static bool open_zone_popup=false;
        bool over_map = io.MousePos.x>=map_p0.x&&io.MousePos.x<=map_p0.x+mapw
                     && io.MousePos.y>=map_p0.y&&io.MousePos.y<=map_p0.y+map_h;
        if(over_map && ImGui::IsMouseClicked(ImGuiMouseButton_Right)){ drag=true; dstart=io.MousePos; }
        if(drag){
            ImVec2 cur=io.MousePos;
            gdl->AddRectFilled(dstart,cur, IM_COL32(255,158,64,36), 0);
            gdl->AddRect(dstart,cur, IM_COL32(255,158,64,220), 0, 0, 1.8f);
            if(ImGui::IsMouseReleased(ImGuiMouseButton_Right)){
                drag=false;
                if(fabsf(cur.x-dstart.x)>6 && fabsf(cur.y-dstart.y)>6){
                    px2ll(dstart,nz_la0,nz_lo0); px2ll(cur,nz_la1,nz_lo1);
                    open_zone_popup=true;
                }
            }
        }
        auto dash=[&](ImVec2 a, ImVec2 b, ImU32 col, float th){   // 8px 대시 점선
            float dx=b.x-a.x, dy=b.y-a.y, len=sqrtf(dx*dx+dy*dy);
            int nseg=(int)(len/8.f)+1;
            for(int k=0;k<nseg;k+=2){
                float t0=(float)k/nseg, t1=std::min(1.f,(float)(k+1)/nseg);
                gdl->AddLine(ImVec2(a.x+dx*t0,a.y+dy*t0), ImVec2(a.x+dx*t1,a.y+dy*t1), col, th);
            }
        };
        // 1) 위험구역(typ6, 닫힌 폴리곤) + 항적(typ7) — 머리 위치는 경보 투영선 앵커로 기억
        static std::vector<ImVec2> ppx;
        struct GHead { uint32_t mmsi; ImVec2 px; };
        static std::vector<GHead> gheads; gheads.clear();
        for(const auto& ov : guard_mod::overlays()){
            ppx.clear();
            for(size_t i=0;i+1<ov.ll.size(); i+=2) ppx.push_back(ll2px(ov.ll[i], ov.ll[i+1]));
            if(ppx.size()<2) continue;
            if(ov.typ==6){                                     // 위험구역: 채움+외곽+이름
                ImU32 zc = ov.kind>=3? IM_COL32(255, 90, 80,0) :
                           ov.kind==2? IM_COL32(255,158, 64,0) : IM_COL32(242,217, 89,0);
                gdl->AddConcavePolyFilled(ppx.data(), (int)ppx.size(), zc | (26u<<24));
                gdl->AddPolyline(ppx.data(), (int)ppx.size(), zc | (150u<<24), ImDrawFlags_Closed, 1.6f);
                ImVec2 ctr(0,0); for(const auto& p : ppx){ ctr.x+=p.x; ctr.y+=p.y; }
                ctr.x/=ppx.size(); ctr.y/=ppx.size();
                ImVec2 ts=ImGui::CalcTextSize(ov.name);
                gdl->AddText(ImVec2(ctr.x-ts.x*0.5f, ctr.y-ts.y*0.5f), zc | (210u<<24), ov.name);
            } else {                                           // 항적: 점선 + 위치보고 fix 점 + 머리
                ImU32 dc = IM_COL32(150,210,255,0);
                for(size_t i=0;i+1<ppx.size(); i++) dash(ppx[i], ppx[i+1], dc | (115u<<24), 1.3f);
                for(const auto& p : ppx) gdl->AddCircleFilled(p, 1.8f, dc | (190u<<24));  // fix 점
                ImVec2 hd=ppx.back();
                gdl->AddCircleFilled(hd, 2.6f, dc | (255u<<24));           // 머리(현재 위치)
                gdl->AddCircle(hd, 5.5f, dc | (200u<<24), 0, 1.6f);
                gdl->AddText(ImVec2(hd.x+8, hd.y-7), dc | (220u<<24), ov.name);
                if(ov.mmsi) gheads.push_back({ov.mmsi, hd});
            }
        }
        // 선박 머리 조회: typ7 항적 우선, 없으면 실선박 지도 마커 위치
        auto head_of=[&](uint32_t mm, ImVec2& out)->bool{
            if(!mm) return false;
            for(const auto& h : gheads) if(h.mmsi==mm){ out=h.px; return true; }
            for(const auto& mp : pts) if((uint32_t)mp.id==mm){ out=ll2px(mp.lat,mp.lon); return true; }
            return false;
        };
        // 2) 경보 — 예측 이벤트 지점(경보 좌표)에 정밀 마커 + 선박 머리→지점 점선 투영
        for(const auto& r : galerts){
            if(!r.active || r.a.typ>=5) continue;
            ImU32 base = guard_typ_col(r.a.typ);
            ImVec2 c = ll2px(r.a.lat, r.a.lon);
            ImVec2 hp;                                          // 경로 고려 접근 시각화
            if(head_of(r.a.mmsi,  hp)) dash(hp, c, base | (150u<<24), 1.2f);
            if(head_of(r.a.mmsi2, hp)) dash(hp, c, base | (150u<<24), 1.2f);
            float spd = r.a.sev>=3? 1.6f : 1.0f;               // 긴급(sev3)은 빠른 펄스
            for(int k=0;k<3;k++){
                float ph = fmodf(tnow*spd + k/3.f, 1.f);       // 0→1 확장 위상 (은은하게)
                gdl->AddCircle(c, 8.f+ph*14.f, base | ((ImU32)((1.f-ph)*110.f)<<24), 0, 1.6f);
            }
            // 정밀 이벤트 마커: 중심 점 + 얇은 링 + 십자 틱 (화면고정 크기 — 줌 시 점처럼 정확)
            gdl->AddCircleFilled(c, 2.4f, base | (255u<<24));
            gdl->AddCircle(c, 6.5f, base | (220u<<24), 0, 1.3f);
            for(int k=0;k<4;k++){
                float ax = (float)((k==0)-(k==1)), ay = (float)((k==2)-(k==3));
                gdl->AddLine(ImVec2(c.x+ax*8.5f, c.y+ay*8.5f), ImVec2(c.x+ax*13.f, c.y+ay*13.f),
                             base | (230u<<24), 1.3f);
            }
        }
        gdl->PopClipRect();

        // ── 신규 구역 입력 팝업 ──
        static bool zfocus=false;
        if(open_zone_popup){ ImGui::OpenPopup("##guardzone"); open_zone_popup=false; zfocus=true; }
        ImVec2 vc=ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(vc, ImGuiCond_Appearing, ImVec2(0.5f,0.5f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(22,20));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8,11));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.f);
        if(ImGui::BeginPopupModal("##guardzone", nullptr,
               ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoTitleBar)){
            static char zname[48]=""; static int zkind=2;
            const float W=300.f;
            ImGui::TextColored(ImVec4(0.95f,0.80f,0.45f,1.f), "위험구역 추가");
            ImGui::Separator();
            ImGui::Dummy(ImVec2(0,3));

            ImGui::TextDisabled("이름");
            ImGui::SetNextItemWidth(W);
            if(zfocus){ ImGui::SetKeyboardFocusHere(); zfocus=false; }
            bool enter = ImGui::InputTextWithHint("##zname", "예: 마산항 방파제, 어장, 사격구역",
                             zname, sizeof(zname), ImGuiInputTextFlags_EnterReturnsTrue);

            ImGui::Dummy(ImVec2(0,5));
            ImGui::TextDisabled("위험 수준");
            struct KOpt{ int k; const char* t; ImU32 c; };
            static const KOpt ko[3]={{1,"주의",IM_COL32(242,217,89,255)},
                                     {2,"경고",IM_COL32(255,158,64,255)},
                                     {3,"금지",IM_COL32(255,80,70,255)}};
            float bw=(W-16.f)/3.f;
            for(int i=0;i<3;i++){
                if(i) ImGui::SameLine();
                bool sel=(zkind==ko[i].k); ImU32 c=ko[i].c;
                ImGui::PushStyleColor(ImGuiCol_Button,        (c&0x00FFFFFFu)|((sel?255u:55u)<<24));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (c&0x00FFFFFFu)|(255u<<24));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (c&0x00FFFFFFu)|(255u<<24));
                ImGui::PushStyleColor(ImGuiCol_Text, sel? IM_COL32(20,20,20,255):IM_COL32(228,228,234,255));
                if(ImGui::Button(ko[i].t, ImVec2(bw, 30))) zkind=ko[i].k;
                ImGui::PopStyleColor(4);
            }

            ImGui::Dummy(ImVec2(0,9));
            bool ok = zname[0]!=0;
            ImGui::BeginDisabled(!ok);
            if(ImGui::Button("추가", ImVec2((W-8.f)/2.f, 34)) || (enter && ok)){
                guard_mod::add_local_zone(zname, (uint8_t)zkind, nz_la0,nz_la1,nz_lo0,nz_lo1);
                zname[0]=0; ImGui::CloseCurrentPopup();
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if(ImGui::Button("취소", ImVec2((W-8.f)/2.f, 34)) || ImGui::IsKeyPressed(ImGuiKey_Escape)){
                zname[0]=0; ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        ImGui::PopStyleVar(3);
    }
#endif
    if(mres.clicked_station>=0 && mres.clicked_station<(int)stn_names.size()){
        // 기지 아이콘 클릭 → 그 기지 수신선박 점선 토글
        const std::string& cs = stn_names[mres.clicked_station];
        sel_station = (sel_station==cs) ? std::string() : cs;
    }
    if(mres.clicked_id){
        uint32_t id=(uint32_t)mres.clicked_id;
        if(sel_mmsi==id){                               // 활성 배 재클릭 → 비활성 (목록 복귀)
            sel_mmsi=0; map_pin=0; has_focus=false; nav_go(0);
        } else {
            sel_mmsi=id; map_pin=id;
            { std::lock_guard<std::mutex> lk(mtx);
              for(const AisRecord& m : log) if(m.mmsi==id && m.has_pos) focus=m; }
            has_focus=true;
            nav_go(id);                                 // 표 행 클릭과 동일 — 그 선박 전체 이력 화면으로
        }
    }
#ifdef BEWE_MODULE_GUARD
    // ── GUARD 경보 카드 스택 (지도 좌상단) + 상태 칩 (지도 우하단) ──────────────
    // 활성 경보만, sev↓→최신↓ 상위 5장. 클릭=선박 선택, hover=상세(점수/CPA/권고) 툴팁.
    {
        std::vector<const guard_mod::AlertRow*> act;
        for(const auto& r : galerts) if(r.active && r.a.typ<5) act.push_back(&r);
        std::sort(act.begin(), act.end(), [](const guard_mod::AlertRow* x, const guard_mod::AlertRow* y){
            if(x->a.sev!=y->a.sev) return x->a.sev>y->a.sev;
            return x->a.t_ms>y->a.t_ms; });
        // MMSI → 선명 조회 (AIS log; 없으면 빈문자). "MMSI (이름)" 라벨 생성.
        auto name_of=[&](uint32_t mm, char* o, size_t cap){
            o[0]=0; if(!mm) return;
            std::lock_guard<std::mutex> lk(mtx);
            for(auto it=log.rbegin(); it!=log.rend(); ++it)
                if(it->mmsi==mm && it->name[0]){ strncpy(o,it->name,cap-1); o[cap-1]=0; return; }
        };
        auto label=[&](uint32_t mm, char* o, size_t cap){   // 이름 있으면 이름만, 없으면 MMSI
            char nm[24]; name_of(mm,nm,sizeof(nm));
            if(nm[0]) snprintf(o,cap,"%s",nm); else snprintf(o,cap,"%u",mm);
        };
        ImDrawList* gdl = ImGui::GetWindowDrawList();
        { // 상태 칩: 우하단 스케일바 위 — 플랫폼 가동 표시 (경보 0=녹색, N=빨강)
            char chip[24]; int n=(int)act.size();
            if(n) snprintf(chip,sizeof(chip),"GUARD %d",n); else snprintf(chip,sizeof(chip),"GUARD");
            ImVec2 ts=ImGui::CalcTextSize(chip);
            ImVec2 c0(map_p0.x+mapw-ts.x-30, map_p0.y+map_h-46), c1(c0.x+ts.x+14, c0.y+ts.y+6);
            gdl->AddRectFilled(c0, c1, n? IM_COL32(110,28,24,205) : IM_COL32(18,52,32,185), 4.f);
            gdl->AddText(ImVec2(c0.x+7,c0.y+3), n? IM_COL32(255,165,150,255) : IM_COL32(120,205,150,220), chip);
        }
        const float CW=286.f, CH=38.f, GAP=5.f;
        float cx=map_p0.x+8.f, cy=map_p0.y+34.f;               // ⛶(6..26) 아래부터
        int shown=0;
        for(const guard_mod::AlertRow* pr : act){
            if(shown>=5){ char more[24]; snprintf(more,sizeof(more),"+%d건", (int)act.size()-shown);
                gdl->AddText(ImVec2(cx+4,cy+2), IM_COL32(180,190,205,200), more); break; }
            const GuardAlert& a=pr->a;
            ImVec2 p0c(cx,cy), p1c(cx+CW, cy+CH);
            bool hov = io.MousePos.x>=p0c.x&&io.MousePos.x<=p1c.x&&io.MousePos.y>=p0c.y&&io.MousePos.y<=p1c.y;
            ImU32 tc = guard_typ_col(a.typ);
            gdl->AddRectFilled(p0c,p1c, hov? IM_COL32(26,35,48,238):IM_COL32(14,21,31,216), 4.f);
            gdl->AddRectFilled(p0c, ImVec2(p0c.x+3,p1c.y), tc | (255u<<24), 2.f);
            // 상단: "유형 MMSI (이름)"  +  우측 심각도
            char lbl[48]; label(a.mmsi,lbl,sizeof(lbl));
            char l1[80]; snprintf(l1,sizeof(l1),"%s  %s", guard_typ_name(a.typ), lbl);
            const char* sv=guard_sev_name(a.sev);
            ImVec2 svs=ImGui::CalcTextSize(sv);
            ImU32 svc = a.sev>=3? IM_COL32(255,95,85,255) : a.sev==2? IM_COL32(255,172,72,255) : IM_COL32(232,212,95,255);
            gdl->PushClipRect(p0c, ImVec2(p1c.x-svs.x-12,p1c.y), true);
            gdl->AddText(ImVec2(p0c.x+9,p0c.y+4), tc | (255u<<24), l1);
            gdl->PopClipRect();
            gdl->AddText(ImVec2(p1c.x-svs.x-8,p0c.y+4), svc, sv);
            // 하단: 짧은 상황. 위조의심(typ4)은 "추정 선박: 이름(없으면 MMSI)"
            char m2buf[80]; const char* mline=a.msg;
            if(a.typ==4){ char lb[32]; label(a.mmsi2, lb, sizeof(lb));
                          snprintf(m2buf,sizeof(m2buf),"추정 선박: %s", lb); mline=m2buf; }
            gdl->PushClipRect(ImVec2(p0c.x+9,p0c.y+20), ImVec2(p1c.x-6,p1c.y-1), true);
            gdl->AddText(ImVec2(p0c.x+9,p0c.y+20), IM_COL32(198,208,222,230), mline);
            gdl->PopClipRect();
            if(hov){
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14,12));
                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8,7));
                ImGui::BeginTooltip();
                ImGui::TextColored(ImVec4(0.95f,0.75f,0.35f,1.f), "%s", guard_typ_name(a.typ));
                ImGui::SameLine(); ImGui::TextColored(
                    a.sev>=3?ImVec4(1,0.37f,0.33f,1):a.sev==2?ImVec4(1,0.67f,0.28f,1):ImVec4(0.9f,0.83f,0.37f,1),
                    " %s", sv);
                { char lb[48]; label(a.mmsi,lb,sizeof(lb)); ImGui::Text("선박  %s", lb); }
                if(a.mmsi2){ char lb[48]; label(a.mmsi2,lb,sizeof(lb));
                             ImGui::Text("%s  %s", a.typ==4? "추정 선박":"상대  ", lb); }
                if(a.typ!=4 && a.msg[0]) ImGui::TextUnformatted(a.msg);
                if(a.reco[0]){
                    ImGui::Dummy(ImVec2(0,1));
                    ImGui::PushTextWrapPos(300.f);
                    ImGui::TextColored(ImVec4(0.55f,0.90f,0.58f,1.f), "%s", a.reco);   // 초록 권고
                    ImGui::PopTextWrapPos();
                }
                ImGui::EndTooltip();
                ImGui::PopStyleVar(2);
                if(ImGui::IsMouseClicked(ImGuiMouseButton_Left) && a.mmsi){
                    sel_mmsi=a.mmsi; map_pin=a.mmsi;            // 카드 클릭 → 그 선박 선택 (지도클릭 덮음)
                    { std::lock_guard<std::mutex> lk(mtx);
                      for(const AisRecord& m : log) if(m.mmsi==a.mmsi && m.has_pos) focus=m; }
                    has_focus=true; nav_go(a.mmsi);
                }
            }
            cy += CH+GAP; shown++;
        }
    }
#endif

    // 선택 배 focus 를 최신 수신 레코드로 매 프레임 갱신 (GPS 등 계속 업데이트)
    if(has_focus && sel_mmsi){
        std::lock_guard<std::mutex> lk(mtx);
        AisRecord nm{}; bool got=false;
        // 최신 레코드 = 동적(위치/속도) 베이스. 정적(이름/호출/선종/IMO/목적지/ETA/흘수)은
        // 그 필드를 가진 가장 최근 레코드에서 채움 (정적은 Type5/19/24 에만 있어 위치msg엔 없음).
        for(auto it=log.rbegin(); it!=log.rend(); ++it){
            if(it->mmsi!=sel_mmsi) continue;
            const AisRecord& r=*it;
            if(!got){ nm=r; got=true; }
            if(!nm.name[0]&&r.name[0]) strncpy(nm.name,r.name,sizeof(nm.name)-1);
            if(!nm.callsign[0]&&r.callsign[0]) strncpy(nm.callsign,r.callsign,sizeof(nm.callsign)-1);
            if(nm.ship_type<=0&&r.ship_type>0) nm.ship_type=r.ship_type;
            if(!nm.imo&&r.imo) nm.imo=r.imo;
            if(!nm.dest[0]&&r.dest[0]) strncpy(nm.dest,r.dest,sizeof(nm.dest)-1);
            if(nm.draught<0&&r.draught>=0) nm.draught=r.draught;
            if(!nm.eta_mon&&r.eta_mon){ nm.eta_mon=r.eta_mon; nm.eta_day=r.eta_day; nm.eta_hour=r.eta_hour; nm.eta_min=r.eta_min; }
            if(!nm.ai_status&&r.ai_status){ nm.ai_status=r.ai_status; nm.ai_mmsi=r.ai_mmsi; nm.ai_conf=r.ai_conf; }  // Match_AI sticky
            if(nm.name[0]&&nm.imo&&nm.dest[0]&&nm.draught>=0&&nm.eta_mon) break;   // 다 채우면 조기 종료
        }
        if(got) focus=nm;
    }
    // ── 선택 선박 정보: 일반 모드=하단 세부패널 / 전체화면=우상단 카드 (flightradar 스타일) ──
    auto draw_detail_body=[&](){
        ImVec4 V(0.85f,0.85f,0.9f,1.f), K(0.55f,0.62f,0.72f,1.f);
        auto row=[&](const char* k, const char* val, ImVec4 vc){    // 라벨/값 한 줄, 값 잘림 방지
            ImGui::TextColored(K,"%s",k); ImGui::SameLine(); ImGui::TextColored(vc,"%s",val);
        };
        { char mm[16]; snprintf(mm,sizeof(mm),"%u",focus.mmsi); row("MMSI", mm, ImVec4(0.5f,0.8f,1.f,1.f)); }
        if(focus.name[0]) row("Name", focus.name, V);
        if(focus.callsign[0]) row("Call", focus.callsign, V);
        row("Country", ais_mid_country(focus.mmsi), ImVec4(0.6f,0.6f,0.6f,1.f));
        if(focus.ship_type>0) row("Ship", ais_shiptype(focus.ship_type), ImVec4(0.62f,0.7f,0.62f,1.f));
        if(focus.imo){ char s[16]; snprintf(s,sizeof(s),"%u",focus.imo); row("IMO", s, V); }
        ImGui::Separator();
        if(focus.has_pos){ char p[40]; snprintf(p,sizeof(p),"%.5f",focus.lat); row("Lat", p, V);
                           snprintf(p,sizeof(p),"%.5f",focus.lon); row("Lon", p, V); }
        if(focus.sog>=0){ char s[16]; snprintf(s,sizeof(s),"%.1f kt",focus.sog); row("SOG", s, V); }
        if(focus.cog>=0){ char s[16]; snprintf(s,sizeof(s),"%.1f°",focus.cog); row("COG", s, V); }
        if(focus.heading!=511){ char s[16]; snprintf(s,sizeof(s),"%d°",focus.heading); row("HDG", s, V); }
        if(focus.nav_status>=0) row("Status", ais_navstatus(focus.nav_status), V);
        if(focus.dest[0]) row("Dest", focus.dest, ImVec4(0.82f,0.76f,0.55f,1.f));
        if(focus.draught>=0){ char s[16]; snprintf(s,sizeof(s),"%.1f m",focus.draught); row("Draught", s, V); }
        if(focus.eta_mon){ char s[24]; snprintf(s,sizeof(s),"%02d-%02d %02d:%02d",focus.eta_mon,focus.eta_day,focus.eta_hour,focus.eta_min); row("ETA", s, V); }
        // ── RF 진단 (CFO — 버스트 특징) ──
        if(focus.has_rf){
            ImGui::Separator();
            char s[24]; snprintf(s,sizeof(s),"%.0f Hz",focus.cfo_hz); row("CFO", s, V);
        }
#ifdef BEWE_MODULE_GUARD
        // ── GUARD 활성 경보 (이 선박 관련; 상황+상대선박+초록 권고) ──
        for(const auto& r : galerts){
            if(!r.active || r.a.typ>=5 || (r.a.mmsi!=focus.mmsi && r.a.mmsi2!=focus.mmsi)) continue;
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.95f,0.62f,0.45f,1.f), "%s", guard_typ_name(r.a.typ));
            ImGui::SameLine();
            ImGui::TextColored(r.a.sev>=3?ImVec4(1,0.37f,0.33f,1):r.a.sev==2?ImVec4(1,0.67f,0.28f,1):ImVec4(0.9f,0.83f,0.37f,1),
                               " %s", guard_sev_name(r.a.sev));
            if(r.a.msg[0]) ImGui::TextColored(V, "%s", r.a.msg);
            if(r.a.mmsi2){                                   // 상대선박 MMSI (이름)
                uint32_t om=(r.a.mmsi2==focus.mmsi)? r.a.mmsi : r.a.mmsi2;
                char onm[24]={0}; { std::lock_guard<std::mutex> lk(mtx);
                    for(auto it=log.rbegin(); it!=log.rend(); ++it) if(it->mmsi==om && it->name[0]){ strncpy(onm,it->name,23); break; } }
                char s[48]; if(onm[0]) snprintf(s,sizeof(s),"%s",onm); else snprintf(s,sizeof(s),"%u",om);
                row("상대", s, V);
            }
            if(r.a.reco[0]){
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX()+ImGui::GetContentRegionAvail().x);
                ImGui::TextColored(ImVec4(0.55f,0.90f,0.58f,1.f), "%s", r.a.reco);
                ImGui::PopTextWrapPos();
            }
        }
#endif
#ifdef BEWE_MODULE_AIS_AI
        // ── Match_AI (DL 지문; has_rf 와 독립) ──
        if(focus.ai_status==2){ char s[32]; snprintf(s,sizeof(s),"%u (%.1f%%)",focus.ai_mmsi,focus.ai_conf/10.0);
            row("Match_AI", s, focus.ai_mmsi==focus.mmsi? ImVec4(0.55f,0.8f,0.55f,1.f):ImVec4(1.f,0.5f,0.4f,1.f)); }
        else if(focus.ai_status==1) row("Match_AI","UNKNOWN", ImVec4(0.75f,0.72f,0.5f,1.f));
#endif
    };
    if(has_focus && mv.big){
        // 전체화면: 지도 우상단 박스 카드. 상단은 헤더바(30) 아래 + 여백, 우측 여백 동일.
        const float CW=150.f, MX=16.f;   // 폭은 데이터 최대(MMSI/"SOG 10.2 kt")에 맞춤, 높이 자동
        ImGui::SetCursorPos(ImVec2(x0 + W-CW-MX, y0+30.f+MX));  // 헤더바(30) 아래 + MX, 우측도 MX
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.06f,0.09f,0.13f,0.94f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12,10));
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.f);
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8,6));
        ImGui::BeginChild("##ais_card", ImVec2(CW,0), ImGuiChildFlags_Borders|ImGuiChildFlags_AutoResizeY,
                          ImGuiWindowFlags_NoScrollbar);   // 높이=내용맞춤 (Status 에서 끝, 아래 여백 없음)
        draw_detail_body();
        ImGui::EndChild();
        ImGui::PopStyleVar(3); ImGui::PopStyleColor();
    }
    // 크게보기 상태를 FFTViewer 로 미러 → 앱 상단바/DEMOD 탭바가 읽어 숨김 (지도만 전체화면).
    v.ais_fullscreen = mv.big;
}

} // namespace ais_mod
