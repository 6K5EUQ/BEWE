#pragma once
// ── GUARD 경보 레코드 + wire 포맷 + 라벨 헬퍼 ───────────────────────────────
// Python ais_guard 데몬이 BEAE/ais/data/guard/alerts_live.jsonl 에 쓰는 경보를
// C++ 모듈이 tail → wire 로 framework 방출. 같은 사건(aid)은 UPDATE/CLEAR 갱신.
#include <cstdint>
#include <cstring>
#include <vector>

struct FFTViewer;

// 경보 레코드 (뷰용; AisRecord 와 동형 스타일)
struct GuardAlert {
    int64_t  t_ms  = 0;       // 발화/갱신 시각 (ms)
    uint32_t aid   = 0;       // 사건 고유 id (typ,mmsi,mmsi2 안정 해시)
    uint8_t  typ   = 0;       // 1=충돌 2=좌초 3=이상항적 4=위협 5=일일보고
    uint8_t  sev   = 0;       // 1=주의 2=경고 3=긴급
    uint8_t  state = 0;       // 1=NEW 2=UPDATE 3=CLEAR
    uint32_t mmsi  = 0;       // 대상 선박
    uint32_t mmsi2 = 0;       // 상대 선박 (0 = 없음; COLLISION 등 선박간)
    float    lat = 0.f, lon = 0.f;   // 사건 위치 (도)
    float    score = 0.f;     // 위험도 0~100
    float    cpa_m  = -1.f;   // 최근접거리 m  (-1 = n/a)
    float    tcpa_s = -1.f;   // 최근접까지 s (-1 = n/a)
    char     msg[96]  = {};   // 한국어 짧은 상황 문장 (관제사용)
    char     reco[128]= {};   // 권고 조치 (REPORT 는 보고서 파일 경로)
    char     station[16] = {};// 수신 기지 표시명 (DGS-2 / LOCAL)
};

// wire 포맷 (framework BEWE_MK_DATA payload; station 은 MpData 봉투가 운반)
struct __attribute__((packed)) GuardWireMsg {
    int64_t  t_ms; uint32_t aid; uint8_t typ, sev, state, _pad;
    uint32_t mmsi, mmsi2; float lat, lon, score, cpa_m, tcpa_s;
    char msg[96]; char reco[128];
};
static_assert(sizeof(GuardWireMsg)==268, "GuardWireMsg must be 268 bytes");

inline void guard_msg_to_wire(const GuardAlert& a, GuardWireMsg& w){
    memset(&w, 0, sizeof(w));
    w.t_ms=a.t_ms; w.aid=a.aid; w.typ=a.typ; w.sev=a.sev; w.state=a.state;
    w.mmsi=a.mmsi; w.mmsi2=a.mmsi2; w.lat=a.lat; w.lon=a.lon;
    w.score=a.score; w.cpa_m=a.cpa_m; w.tcpa_s=a.tcpa_s;
    memcpy(w.msg,a.msg,sizeof(w.msg)); memcpy(w.reco,a.reco,sizeof(w.reco));
}
inline void guard_wire_to_msg(const GuardWireMsg& w, GuardAlert& a){
    a = GuardAlert{};
    a.t_ms=w.t_ms; a.aid=w.aid; a.typ=w.typ; a.sev=w.sev; a.state=w.state;
    a.mmsi=w.mmsi; a.mmsi2=w.mmsi2; a.lat=w.lat; a.lon=w.lon;
    a.score=w.score; a.cpa_m=w.cpa_m; a.tcpa_s=w.tcpa_s;
    memcpy(a.msg,w.msg,sizeof(a.msg)); a.msg[sizeof(a.msg)-1]=0;
    memcpy(a.reco,w.reco,sizeof(a.reco)); a.reco[sizeof(a.reco)-1]=0;
}

// ── 경보 유형/심각도 라벨 (한국어; 뷰/오버레이 공용) ────────────────────────
inline const char* guard_typ_name(uint8_t t){
    switch(t){
        case 1: return "충돌위험";
        case 2: return "좌초위험";
        case 3: return "이상항적";
        case 4: return "위협선박";
        case 5: return "일일보고";
        default: return "";
    }
}
inline const char* guard_sev_name(uint8_t s){
    switch(s){
        case 1: return "주의";
        case 2: return "경고";
        case 3: return "긴급";
        default: return "";
    }
}

// ── 모듈 공개 API (guard_module.cpp) ────────────────────────────────────────
namespace guard_mod {

// 뷰 스냅샷 행: 경보 레코드 + 활성 여부 (CLEAR 수신 시 active=false, 기록 보존)
struct AlertRow {
    GuardAlert a;
    bool       active = true;
};

std::vector<AlertRow> snapshot();   // 내부 log 사본 (aid 별 최신상태, 시간순)
int  active_count();                // 활성 경보 수 (일일보고 typ=5 제외)
// mmsi 가 관련된 활성 경보 중 최고 sev 1건 → typ/sev 채움 (ais_view 오버레이용)
bool vessel_alert(uint32_t mmsi, uint8_t& typ, uint8_t& sev);

// HOST: ais_guard 데몬 확보 + alerts_live.jsonl tail 시작 (idempotent).
// 반드시 단일스레드 시점(ais host_start 의 g_mgmt 락 내부)에서 호출 — fork 안전.
void host_ensure(FFTViewer& v);

#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);   // guard_view.cpp (별도 작성)
#endif

} // namespace guard_mod
