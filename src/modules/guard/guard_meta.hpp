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
// 한글 라벨 — ui.cpp 가 나눔폰트를 기본 폰트에 병합해 렌더 (assets/fonts 동봉)
inline const char* guard_typ_name(uint8_t t){
    switch(t){
        case 1: return "충돌위험";
        case 2: return "진입위험";   // 위험구역(암초/기상/군사/항행금지 등) 진입·진입예정
        case 3: return "이상항적";
        case 4: return "위조의심";
        case 5: return "일일보고";
        case 6: return "ZONE";       // 보조 레코드: zones.json 폴리곤 (경보 아님)
        case 7: return "PATH";       // 보조 레코드: 데모/가상 선박 경로 (경보 아님)
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

std::vector<AlertRow> snapshot();   // 내부 log 사본 (aid 별 최신상태, 시간순; typ 1~5)
int  active_count();                // 활성 경보 수 (보조 typ>=5 제외)
// mmsi 가 관련된 활성 경보 중 최고 sev 1건 → typ/sev 채움 (ais_view 행 강조용)
bool vessel_alert(uint32_t mmsi, uint8_t& typ, uint8_t& sev);

// 보조 오버레이 (typ=6 위험구역 / typ=7 데모 항적) — reco/msg 텍스트에
// "lat,lon;lat,lon;..." 꼭짓점 패킹, 청크(mmsi=idx, mmsi2=총수) 조립 완료분만 반환.
struct OverlayPath {
    uint32_t aid = 0;
    uint8_t  typ = 0;                // 6=폴리곤(닫힘) 7=폴리라인(항적, 끝점=현재)
    uint8_t  kind = 0;               // sev 필드 재활용 (구역 종류/데모 심각도)
    uint32_t mmsi = 0;               // typ=7: 가상 MMSI (라벨용)
    char     name[40] = {};          // 구역명 / "[예시] 선명"
    std::vector<float> ll;           // lat,lon 인터리브
};
std::vector<OverlayPath> overlays();   // 조립 완료 스냅샷 (지도 오버레이용)

// ── 뷰어 로컬 기능 (wire 안 탐 — 이 PC 화면에서만) ─────────────────────────
// 로컬 주입: G키 데모·로컬 구역 경보를 같은 경보 파이프(카드/링/행강조)로 표시
void upsert_local(const GuardAlert& a);

// G키 데모: 예시 4종 + 가상 항적 5개 토글 (기본 OFF)
bool demo_on();
void demo_toggle();

// 로컬 위험구역 (우클릭 드래그 등록, 파일 영속: data_dir/guard_zones_local.json)
struct LocalZone {
    char    name[48] = {};           // UTF-8 (한글 가능)
    uint8_t kind = 2;                // 1=주의(노랑) 2=경고(주황) 3=금지(빨강)
    double  lat0=0, lat1=0, lon0=0, lon1=0;   // 정규화된 박스 (lat0<lat1, lon0<lon1)
};
std::vector<LocalZone> local_zones();
void add_local_zone(const char* name, uint8_t kind, double la0,double la1,double lo0,double lo1);
bool del_local_zone(uint32_t idx);

// 진입/진입예정 판정 (뷰어 ~1초 주기 호출; dead-reckon 10분 투영 → 경보 발화/해제)
struct VesselSnap { uint32_t mmsi; double lat, lon; float sog, cog; };
void eval_local_zones(const std::vector<VesselSnap>& vs, int64_t now_ms);

// HOST: ais_guard 데몬 확보 + alerts_live.jsonl tail 시작 (idempotent).
// 반드시 단일스레드 시점(ais host_start 의 g_mgmt 락 내부)에서 호출 — fork 안전.
void host_ensure(FFTViewer& v);

} // namespace guard_mod
