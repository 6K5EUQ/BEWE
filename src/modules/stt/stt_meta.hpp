#pragma once
// ── STT 자막 레코드 + wire 포맷 ────────────────────────────────────────────
// squelch open→close 발화 하나 = 자막 한 줄. HOST 가 로컬 whisper 워커로 전사한
// 결과를 이 레코드로 emit → Central → 모든 JOIN 동일 표시 (AIS 와 동형 append-only).
#include <cstdint>
#include <cstring>

namespace stt_mod {

struct SttRecord {
    int64_t  t_ms = 0;        // 전사 완료 wall time
    int32_t  ch   = 0;        // 채널필터 인덱스
    float    dur  = 0.f;      // 발화 길이(초)
    char     station[16] = {}; // 복조 기지 (봉투가 운반; 로컬 표시용)
    char     text[240]   = {}; // 전사 문장 (UTF-8, 잘림 허용)
};

// wire 포맷 (station 은 MpData 봉투가 운반 — payload 엔 미포함). append-only.
struct __attribute__((packed)) SttWireMsg {
    int64_t  t_ms;
    int32_t  ch;
    float    dur;
    char     text[240];
};

inline void stt_msg_to_wire(const SttRecord& m, SttWireMsg& w){
    memset(&w, 0, sizeof(w));
    w.t_ms=m.t_ms; w.ch=m.ch; w.dur=m.dur;
    memcpy(w.text, m.text, sizeof(w.text)); w.text[sizeof(w.text)-1]=0;
}
inline void stt_wire_to_msg(const SttWireMsg& w, SttRecord& m){
    m = SttRecord{};
    m.t_ms=w.t_ms; m.ch=w.ch; m.dur=w.dur;
    memcpy(m.text, w.text, sizeof(m.text)); m.text[sizeof(m.text)-1]=0;
}

} // namespace stt_mod
