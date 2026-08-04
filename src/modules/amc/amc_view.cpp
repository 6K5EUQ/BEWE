// ── AMC 모듈 GUI: 코어 STATUS 패널의 접이식 섹션 ───────────────────────────
#include "amc_module.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <cstring>
#include <cmath>
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

    // 모듈 데이터는 Recv 를 켠 JOIN 에게만 흐른다. 예전엔 DEMOD 탭 헤더바의 Recv
    // 버튼이 그 역할을 했지만 이 패널이 유일한 소비자가 됐으므로, 섹션을 펼치는
    // 것 자체를 구독 의사로 본다. 안 그러면 HOST 는 멀쩡히 분류하는데 화면은
    // 영원히 "waiting for a burst" 다.
    // 접으면 draw_panel 이 안 불리니 구독은 그대로 유지된다 — 레코드가 버스트당
    // 1건뿐이라 트래픽이 무시할 수준이고, 껐다 켤 때마다 오늘 이력을 다시 받는
    // 편이 더 비싸다.
    if(!bewe_mod_recv("amc")) bewe_mod_set_recv(v, "amc", true);

    // 채널별 집계. 한 번의 측정은 2.4 ms 스냅샷이라 값이 크게 흔들린다 — 같은
    // 주파수·대역폭으로 잰 것들을 평균해 그 분산을 줄인다 (독립 표본의 확률 평균).
    // 원본 레코드와 아카이브는 건드리지 않는다: 여기서만 모아 보여줄 뿐이다.
    // 주파수나 폭이 바뀌면 다른 신호이므로 거기서 끊는다.
    struct Agg {
        float   p[AMC_NCLASS] = {};
        int     n = 0;
        int64_t t_ms = 0;          // 가장 최근 측정 시각
        float   freq = 0, bw = 0;
        char    model[16] = {};
    };
    Agg agg[MAX_CHANNELS];
    bool has[MAX_CHANNELS] = {};
    {
        std::lock_guard<std::mutex> lk(mtx);
        // 최근 것부터 거슬러 올라가되, 채널당 상한을 둔다 — log 가 10만 건까지
        // 자라므로 매 프레임 전부 훑으면 안 된다.
        const int MAX_AGG = 32;
        int scanned = 0;
        for(int i=(int)log.size()-1; i>=0 && scanned<4000; i--, scanned++){
            const AmcRecord& m = log[i];
            if(m.ch < 0 || m.ch >= MAX_CHANNELS) continue;
            // AMC 를 켠 시점 이후 것만 — 껐다 켜면 평균이 리셋된다.
            const int64_t since = local_on_since(m.ch);
            if(since && m.t_ms < since) continue;
            Agg& a = agg[m.ch];
            if(!has[m.ch]){                         // 이 채널의 최신 = 기준
                has[m.ch]=true; a.freq=m.freq; a.bw=m.bw_khz; a.t_ms=m.t_ms;
                memcpy(a.model, m.model, sizeof(a.model));
            } else {
                if(a.n >= MAX_AGG) continue;
                // 같은 신호인지: 중심주파수 1 kHz, 폭 2 % 이내
                if(fabsf(m.freq - a.freq) > 0.001f) continue;
                if(a.bw > 0 && fabsf(m.bw_khz - a.bw) > a.bw*0.02f) continue;
            }
            for(int k=0;k<AMC_NCLASS;k++) a.p[k] += m.p[k];
            a.n++;
        }
        for(int c=0;c<MAX_CHANNELS;c++)
            if(agg[c].n>0) for(int k=0;k<AMC_NCLASS;k++) agg[c].p[k] /= (float)agg[c].n;
    }

    // 표시 순서는 Active Channels 와 같게 — 표시번호(주파수 정렬) 오름차순.
    // 슬롯 인덱스 순으로 두면 위 목록과 줄 순서가 어긋나 눈이 두 번 찾는다.
    int order[MAX_CHANNELS], n_ord = 0;
    for(int ci=0; ci<MAX_CHANNELS; ci++){
        if(!v.channels[ci].filter_active) continue;
        if(!local_on(ci)) continue;              // 내가 켠 것만 (운용자별)
        order[n_ord++] = ci;
    }
    for(int a=1; a<n_ord; a++){                       // 삽입정렬 (n 이 작다)
        int k = order[a], d = v.freq_sorted_display_num(k), b = a-1;
        while(b>=0 && v.freq_sorted_display_num(order[b]) > d){ order[b+1]=order[b]; b--; }
        order[b+1] = k;
    }

    for(int oi=0; oi<n_ord; oi++){
        const int ci = order[oi];
        float cf = (v.channels[ci].s + v.channels[ci].e) * 0.5f;
        ImGui::PushID(ci);
        if(has[ci]){
            const Agg& a = agg[ci];
            char ts[12]; hms(a.t_ms, ts);
            // 헤더는 "어느 채널의 언제 측정인가"만. 판정 결과는 바로 아래 막대가
            // 이미 말하고 있어서 같은 값을 두 번 쓰면 줄만 시끄러워진다.
            ImGui::Text("[%2d] %9.4f MHz", v.freq_sorted_display_num(ci), cf);
            ImGui::SameLine(); ImGui::TextDisabled("%s  n=%d", ts, a.n);

            // 막대는 확률 내림차순 — 가장 유력한 후보가 항상 맨 위에 온다.
            int idx[AMC_NCLASS];
            for(int k=0;k<AMC_NCLASS;k++) idx[k]=k;
            for(int x=1;x<AMC_NCLASS;x++){
                int k=idx[x]; float pv=a.p[k]; int b=x-1;
                while(b>=0 && a.p[idx[b]] < pv){ idx[b+1]=idx[b]; b--; }
                idx[b+1]=k;
            }
            for(int r=0;r<AMC_NCLASS;r++){
                const int k = idx[r];
                if(a.p[k] < 0.005f && r>0) continue;   // 0에 가까운 건 줄만 낭비
                ImGui::Text("  %-6s", amc_class_name(k));
                ImGui::SameLine(88.f);
                ImVec4 c = (r==0) ? conf_col(a.p[k]) : ImVec4(0.45f,0.45f,0.5f,1.f);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, c);
                char ov[16]; snprintf(ov,sizeof(ov),"%.0f%%", a.p[k]*100.f);
                ImGui::ProgressBar(a.p[k], ImVec2(-1.f, 12.f), ov);
                ImGui::PopStyleColor();
            }
        } else {
            ImGui::Text("[%2d] %9.4f MHz", v.freq_sorted_display_num(ci), cf);
            ImGui::SameLine();
            // 구독 직후엔 오늘 이력을 받아오는 중이다. 그걸 "신호 대기"로 적으면
            // 운용자가 안테나나 스퀄치를 의심하게 된다 — 다른 상태다.
            if(bewe_mod_hist_loading("amc")) ImGui::TextDisabled("loading ...");
            else                             ImGui::TextDisabled("waiting for a burst");
        }
        ImGui::Spacing();
        ImGui::PopID();
    }

    if(!n_ord){
        if(!bewe_mod_avail("amc", stn)) ImGui::TextDisabled("  not available on this station");
        else                            ImGui::TextDisabled("  (none)");
    }
}

} // namespace amc_mod
