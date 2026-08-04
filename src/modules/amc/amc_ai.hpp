#pragma once
// ── AMC 추론 IPC 선언 ───────────────────────────────────────────────────────
// 이 파일(과 amc_ai.cpp)이 없으면 CMake 가 BEWE_MODULE_AMC_AI 를 정의하지 않고,
// 아래 선언과 호출부가 전부 사라진다 → 모듈은 캡처만 하고 분류는 안 한다.
#include <cstdint>
#include <cstddef>

namespace amc_mod {
#ifdef BEWE_MODULE_AMC_AI

bool amc_ai_enabled();
// 데몬 기동 보장. **단일스레드 시점에서만** 부를 것 (host_start 의 g_mgmt 락 안) —
// 워커 스레드에서 fork 하면 락 상태가 복제돼 자식이 굳는다.
void amc_ai_ensure_daemon();
void amc_ai_stop_daemon();

// 버스트 1건 분류. iq = interleaved float32 I/Q, n_complex 개.
// 반환 false = 판정 못 함(데몬 없음/타임아웃/타 채널 점유). 그 경우 out_* 는 안 건드림.
// p_out = 전 클래스 확률(0~1) 배열, np 개 자리. 상위 2개만으론 막대그래프를 못 그린다.
bool amc_ai_infer(int ch, int64_t t_ms, uint32_t out_sr, uint32_t bw_hz, uint8_t trig,
                  const float* iq, int n_complex,
                  int& cls, float& conf, int& cls2, float& conf2, char* model, size_t model_cap,
                  float* p_out, int np);

#else
inline bool amc_ai_enabled(){ return false; }
inline void amc_ai_ensure_daemon(){}
inline void amc_ai_stop_daemon(){}
inline bool amc_ai_infer(int, int64_t, uint32_t, uint32_t, uint8_t,
                         const float*, int, int&, float&, int&, float&, char*, size_t,
                         float*, int){ return false; }
#endif
} // namespace amc_mod
