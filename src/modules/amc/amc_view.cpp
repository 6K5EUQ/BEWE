// ── AMC 모듈 GUI: DEMOD 데이터 뷰 (공용 modview 컴포넌트로 통일) ────────────
#include "amc_module.hpp"
#include "../modview.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <set>
#include <ctime>

namespace amc_mod {

namespace {
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
const char* trig_name(uint8_t t){ return t==AMC_TRIG_MANUAL ? "MAN" : "SQ"; }

// 신뢰도 색. 낮은 값을 초록으로 칠하면 운용자가 확정으로 오해한다.
// 0.8↑ 초록 / 0.5↑ 노랑 / 그 아래 회색.
ImVec4 conf_col(float c){
    if(c >= 0.80f) return ImVec4(0.45f,0.90f,0.50f,1.f);
    if(c >= 0.50f) return ImVec4(1.00f,0.85f,0.30f,1.f);
    return ImVec4(0.65f,0.65f,0.65f,1.f);
}

// 0 Time 1 Sta 2 CH 3 Freq 4 BW 5 Class 6 Conf 7 SNR 8 Trig
bool match(const AmcRecord& m, const char* f){
    if(!f||!f[0]) return true;
    char fq[16]; snprintf(fq,sizeof(fq),"%.4f",m.freq);
    return modview::ci_find(fq,f)
        || modview::ci_find(amc_class_name(m.cls),f)
        || modview::ci_find(m.station,f)
        || modview::ci_find(m.model,f)
        || modview::ci_find(trig_name(m.trig),f);
}
int col_cmp(int c, const AmcRecord& a, const AmcRecord& b){
    switch(c){
        case 0:  return a.t_ms<b.t_ms?-1:(a.t_ms>b.t_ms?1:0);
        case 1:  return strcmp(a.station,b.station);
        case 2:  return a.ch-b.ch;
        case 3:  return a.freq<b.freq?-1:(a.freq>b.freq?1:0);
        case 4:  return a.bw_khz<b.bw_khz?-1:(a.bw_khz>b.bw_khz?1:0);
        case 5:  return strcmp(amc_class_name(a.cls),amc_class_name(b.cls));
        case 6:  return a.conf<b.conf?-1:(a.conf>b.conf?1:0);
        case 7:  return a.snr_db<b.snr_db?-1:(a.snr_db>b.snr_db?1:0);
        default: return (int)a.trig-(int)b.trig;
    }
}
std::string msg_key(const AmcRecord& m){
    char b[48]; snprintf(b,sizeof(b),"%lld|%d|%d",(long long)m.t_ms,m.ch,m.cls);
    return std::string(b);
}
} // anonymous

void local_load_today(FFTViewer& v){
    (void)v;
    std::string raw;
    if(!store_read_today(raw) || raw.empty()) return;
    std::vector<AmcRecord> recs;
    store_parse_jsonl(raw.data(), raw.size(), recs);
    std::lock_guard<std::mutex> lk(mtx);
    if(!log.empty()) return;                 // 이미 라이브가 쌓였으면 덮지 않는다
    for(auto& m : recs){ strncpy(m.station,"LOCAL",sizeof(m.station)-1); log.push_back(m); }
}

void draw_content(FFTViewer& v, bool just_opened){
    ImGuiIO& io = ImGui::GetIO();
    bool remote = bewe_mod_my_station()[0] != 0;
    if(just_opened && !remote) local_load_today(v);

    ImVec2 avail = ImGui::GetContentRegionAvail();
    float W=avail.x, H=avail.y, x0=ImGui::GetCursorPosX();
    bool win_focus = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    static std::set<std::string> sel; static std::string anchor;
    static AmcRecord focus{}; static bool has_focus=false;
    static int sort_col=-1; static bool sort_asc=true;
    static bool atb=true; static size_t lastn=0;

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; };

    modview::space_toggle_recv(v, "amc", remote, win_focus, on_clear);
    bool focus_filter = win_focus && !io.WantTextInput &&
        ((io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F,false)) || ImGui::IsKeyPressed(ImGuiKey_Tab,false));

    if(win_focus && !io.WantTextInput && io.KeyCtrl && !io.KeyShift &&
       ImGui::IsKeyPressed(ImGuiKey_C,false) && !sel.empty()){
        std::string out; std::lock_guard<std::mutex> lk(mtx);
        for(const AmcRecord& m : log){ if(!sel.count(msg_key(m))) continue;
            char ts[12]; hms(m.t_ms,ts);
            char line[256];
            snprintf(line,sizeof(line),"%s\t%s\t%d\t%.4f\t%.1f\t%s\t%.0f%%\t%.1f\t%s\n",
                ts,m.station,m.ch,m.freq,m.bw_khz,amc_class_name(m.cls),
                m.conf*100.f,m.snr_db,trig_name(m.trig));
            out+=line;
        }
        if(!out.empty()) ImGui::SetClipboardText(out.c_str());
    }

    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    modview::header_bar(v, "amc", filter, sizeof(filter), total, remote, focus_filter, on_clear);

    float detail_h = has_focus ? 110.f : 0.f;
    float table_h = H - 30 - detail_h - 16; if(table_h<60) table_h=60;

    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##amc_tbl", 9, tf, ImVec2(W, table_h))){
        ImGui::TableSetupScrollFreeze(2,1);
        ImGui::TableSetupColumn("Time",  ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Sta",   ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("CH",    ImGuiTableColumnFlags_WidthFixed, 36);
        ImGui::TableSetupColumn("Freq",  ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("BW",    ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("Conf",  ImGuiTableColumnFlags_WidthFixed, 56);
        ImGui::TableSetupColumn("SNR",   ImGuiTableColumnFlags_WidthFixed, 56);
        ImGui::TableSetupColumn("Trig",  ImGuiTableColumnFlags_WidthStretch);
        modview::sortable_headers(9, sort_col, sort_asc, -1);

        std::lock_guard<std::mutex> lk(mtx);
        static std::vector<int> vis;
        static size_t c_n=(size_t)-1; static int64_t c_last=-1;
        static char c_filter[64]={'\xff'}; static int c_sc=-99; static bool c_asc=false;
        size_t n_now=log.size(); int64_t last_t=n_now?log.back().t_ms:0;
        if(n_now!=c_n||last_t!=c_last||strncmp(c_filter,filter,sizeof(c_filter))||c_sc!=sort_col||c_asc!=sort_asc){
            vis.clear(); for(int i=0;i<(int)n_now;i++) if(match(log[i],filter)) vis.push_back(i);
            modview::sort_vis(vis, sort_col, sort_asc, [&](int c,int a,int b){ return col_cmp(c,log[a],log[b]); });
            c_n=n_now; c_last=last_t; strncpy(c_filter,filter,sizeof(c_filter)-1); c_filter[sizeof(c_filter)-1]=0; c_sc=sort_col; c_asc=sort_asc;
        }

        ImGuiListClipper clip; clip.Begin((int)vis.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const AmcRecord& m = log[vis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            std::string k=msg_key(m);
            if(modview::row_col0(vis[r], sel.count(k)>0, ts)){
                modview::apply_click(sel, anchor, k, r, [&](int i){return msg_key(log[vis[i]]);}, (int)vis.size(), io.KeyCtrl, io.KeyShift);
                focus=m; has_focus=!sel.empty();
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); modview::cell(m.station);
            ImGui::TableSetColumnIndex(2); snprintf(b,sizeof(b),"%d",m.ch); modview::cell(b);
            ImGui::TableSetColumnIndex(3); snprintf(b,sizeof(b),"%.4f",m.freq); modview::cell(b);
            ImGui::TableSetColumnIndex(4); snprintf(b,sizeof(b),"%.1f k",m.bw_khz); modview::cell(b);
            ImGui::TableSetColumnIndex(5); modview::cell(amc_class_name(m.cls), conf_col(m.conf));
            ImGui::TableSetColumnIndex(6); snprintf(b,sizeof(b),"%.0f%%",m.conf*100.f); modview::cell(b, conf_col(m.conf));
            ImGui::TableSetColumnIndex(7); snprintf(b,sizeof(b),"%.1f dB",m.snr_db); modview::cell(b);
            ImGui::TableSetColumnIndex(8); modview::cell(trig_name(m.trig));
        }
        bool grew = log.size()>lastn; lastn=log.size();
        modview::tail_follow(atb, grew && sort_col<0);
        ImGui::EndTable();
    }

    // ── 세부 패널 ──
    if(has_focus){
        modview::detail_begin("amc", x0, W, detail_h);
        char tsd[12]; hms(focus.t_ms,tsd);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);
        ImGui::Text("Time:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",tsd);
        modview::kv("Station:", focus.station, V);
        { char c[8]; snprintf(c,sizeof(c),"%d",focus.ch); modview::kv("CH:", c, V); }
        { char fq[20]; snprintf(fq,sizeof(fq),"%.4f MHz",focus.freq); modview::kv("Freq:", fq, V); }
        { char bwv[20]; snprintf(bwv,sizeof(bwv),"%.2f kHz",focus.bw_khz); modview::kv("BW:", bwv, V); }
        ImGui::Separator();
        { char s[32]; snprintf(s,sizeof(s),"%s  %.1f%%",amc_class_name(focus.cls),focus.conf*100.f);
          ImGui::Text("Class:"); ImGui::SameLine(); ImGui::TextColored(conf_col(focus.conf),"%s",s); }
        // 2순위를 같이 보여주는 이유: QAM16↔QAM64, QPSK↔PSK8 은 성좌가 포함관계라
        // 낮은 SNR 에서 본질적으로 헷갈린다. 1순위만 보면 그 불확실성이 안 보인다.
        if(focus.cls2 >= 0){
            char s[32]; snprintf(s,sizeof(s),"%s  %.1f%%",amc_class_name(focus.cls2),focus.conf2*100.f);
            modview::kv("2nd:", s, ImVec4(0.7f,0.7f,0.75f,1.f));
        }
        { char s[16]; snprintf(s,sizeof(s),"%.1f dB",focus.snr_db); modview::kv("SNR:", s, V); }
        modview::kv("Trigger:", focus.trig==AMC_TRIG_MANUAL?"manual (a)":"squelch", V);
        if(focus.model[0]) modview::kv("Model:", focus.model, V);
        modview::detail_end();
    }
}

} // namespace amc_mod
