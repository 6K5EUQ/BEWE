#pragma once
// ── AMC(자동 변조분류) 레코드 + wire 포맷 ────────────────────────────────────
// 다른 해독 모듈과 성격이 다르다. ACARS/AIS/BTLE 는 "무슨 내용인가"를 뽑지만
// AMC 는 "무슨 변조인가"만 판정한다 — 내용은 모른 채 물리계층 형태만 본다.
// 그래서 레코드가 짧고, 채널당 1 버스트 = 1 레코드다.
//
// 추론은 C++ 이 아니라 BEAE/amc 의 Python 데몬이 한다 (PyTorch 체크포인트).
// 여기 있는 건 그 결과를 나르는 그릇뿐이다.
#include <cstdint>
#include <cstring>

// 클래스 순서는 BEAE/amc/amc_ai/synth.py 의 CLASSES 와 **반드시** 같아야 한다.
// 그쪽 주석에 못이 박혀 있다: 추가는 안전, 재배열은 학습된 체크포인트를 조용히
// 무효화한다. 여기서도 같은 규칙이다 — 인덱스가 곧 라벨이라 순서를 바꾸면
// 저장된 이력과 새 추론이 어긋난다.
static const char* const AMC_CLASSES[] = {
    "BPSK", "QPSK", "PSK8", "QAM16", "QAM64",
    "2FSK", "4FSK", "GMSK", "MSK",
    "AM", "FM", "OFDM",
};
static constexpr int AMC_NCLASS = (int)(sizeof(AMC_CLASSES)/sizeof(AMC_CLASSES[0]));

inline const char* amc_class_name(int i){
    return (i >= 0 && i < AMC_NCLASS) ? AMC_CLASSES[i] : "?";
}

// 트리거 종류. 지금은 스퀄치 하나뿐이지만 필드는 남겨 둔다 — 레코드/아카이브
// 포맷이라 나중에 트리거가 늘어도 기존 이력을 다시 파싱할 필요가 없다.
enum : uint8_t {
    AMC_TRIG_SQUELCH = 0,   // sq_gate 상승엣지 (버스트 시작)
};

struct AmcRecord {
    int64_t  t_ms  = 0;     // 호스트 스탬프 (epoch ms)
    float    freq  = 0.f;   // 채널 중심 MHz
    float    bw_khz= 0.f;   // 채널 폭 kHz (추론 입력 대역)
    int      ch    = 0;     // 채널필터 인덱스
    int      cls   = -1;    // AMC_CLASSES 인덱스 (-1 = 판정불가)
    float    conf  = 0.f;   // 최상위 확률 0..1
    float    snr_db= 0.f;   // 버스트 SNR 추정 (기준선 대비 dB)
    uint8_t  trig  = AMC_TRIG_SQUELCH;
    // 2순위 후보. QAM16↔QAM64, QPSK↔PSK8 처럼 성좌가 포함관계인 쌍은 낮은 SNR
    // 에서 본질적으로 헷갈린다(모델 혼동행렬로 확인됨). 1순위만 보여주면 운용자가
    // 그 불확실성을 못 본다 — 2순위를 같이 준다.
    int      cls2  = -1;
    float    conf2 = 0.f;
    char     model[16] = {};  // 판정에 쓴 모델 버전 ("BEAEv6") — 사후 추적용
    // 전 클래스 확률 (0~1). 상위 2개만으론 막대그래프를 못 그린다 — STATUS 패널이
    // 12개를 다 보여줘야 운용자가 "얼마나 아슬아슬한 판정인지"를 눈으로 안다.
    float    p[AMC_NCLASS] = {};
    // 표시 전용. wire 로 안 나간다 — station 은 MpData 봉투가 운반하고 수신측이 채운다.
    char     station[16] = {};
};

// ── wire 포맷 (MODULE_PIPE payload) ──────────────────────────────────────────
// 구조체를 그대로 흘린다. packed 라 패딩 차이가 없고, 필드가 전부 고정폭이다.
#pragma pack(push, 1)
struct AmcWire {
    int64_t  t_ms;
    float    freq, bw_khz;
    int32_t  ch;
    int32_t  cls, cls2;
    float    conf, conf2, snr_db;
    uint8_t  trig;
    char     model[16];
    // 전 클래스 확률을 permille(0~1000) 로. float 12개(48B) 대신 u16 12개(24B) —
    // 표시용이라 0.1% 해상도면 충분하고, 레코드가 커지면 Central .dat 가 그만큼 부푼다.
    uint16_t p_permille[AMC_NCLASS];
};
#pragma pack(pop)

inline void amc_to_wire(const AmcRecord& r, AmcWire& w){
    w = AmcWire{};
    w.t_ms = r.t_ms; w.freq = r.freq; w.bw_khz = r.bw_khz;
    w.ch = r.ch; w.cls = r.cls; w.cls2 = r.cls2;
    w.conf = r.conf; w.conf2 = r.conf2; w.snr_db = r.snr_db;
    w.trig = r.trig;
    memcpy(w.model, r.model, sizeof(w.model));
    for(int i=0;i<AMC_NCLASS;i++){
        float v = r.p[i]*1000.f + 0.5f;
        w.p_permille[i] = (uint16_t)(v < 0.f ? 0.f : (v > 1000.f ? 1000.f : v));
    }
}

inline void amc_from_wire(const AmcWire& w, AmcRecord& r){
    r = AmcRecord{};
    r.t_ms = w.t_ms; r.freq = w.freq; r.bw_khz = w.bw_khz;
    r.ch = w.ch; r.cls = w.cls; r.cls2 = w.cls2;
    r.conf = w.conf; r.conf2 = w.conf2; r.snr_db = w.snr_db;
    r.trig = w.trig;
    memcpy(r.model, w.model, sizeof(r.model));
    r.model[sizeof(r.model)-1] = 0;
    for(int i=0;i<AMC_NCLASS;i++) r.p[i] = w.p_permille[i]/1000.f;
}
