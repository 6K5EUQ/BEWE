#pragma once
// ── AIS 모듈 내부 공유 선언 (모듈 밖에서 include 금지) ──────────────────────
#include "ais_meta.hpp"
#include "config.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

struct FFTViewer;

namespace ais_mod {

extern std::mutex              mtx;
extern std::vector<AisRecord>  log;
extern uint64_t                log_gen;   // log 구조 변경 세대 (mtx 보호 — 뷰 캐시 무효화)
extern char                    filter[64];
constexpr int LOG_MAX = 100000;

// 워커 슬롯 접근자 (ais_module.cpp)
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

// HOST 워커 (ais_decode.cpp)
void worker(FFTViewer& v, int ch_idx);
// 워커 → 스탬프 + 호스트 아카이브 + framework emit
// ai_iq: Match_AI 버스트 캡처 (데시메이트 복소 f32 I/Q interleave, n=복소샘플수) — nullptr 면 비활성
void host_emit(FFTViewer& v, AisRecord m, const float* ai_iq = nullptr,
               int ai_n = 0, uint32_t ai_sr = 0);
// RF 지문 raw 시리즈 캡처 (BEWE_AIS_FPCAP; 학습데이터 사이드카)
void host_fpcap(uint32_t mmsi, const float* series, int n);

// Behavior 이상탐지 규칙층 (ais_anom.cpp): GAP/JUMP/SPEED 결정적 규칙. m.anom_* in-place 채움.
void host_anom(AisRecord& m);

// Match_AI (ais_ai.cpp): venv 존재 게이트(env BEWE_AIS_AI 로 강제 오버라이드). aicap append + UDS 질의
// 독립 판매 모듈 — ais_ai.cpp 없으면 BEWE_MODULE_AIS_AI 미정의 → 선언·호출·열 전부 제외.
#ifdef BEWE_MODULE_AIS_AI
bool ai_enabled();
void host_ai(AisRecord& m, uint32_t out_sr, const float* iq, int n_complex);
void ai_ensure_daemon();   // AIS decode 워커 켤 때 추론 데몬 spawn (외부 데몬 있으면 no-op)
void ai_stop_daemon();     // BEWE 종료 시 자기 spawn 데몬 정리
#endif

// 공통 (ais_module.cpp)
void append_log(const AisRecord& m);
void store_append(const AisRecord& m);          // ~/BEWE/modules/ais/ais_YYYYMMDD.jsonl
bool store_read_today(std::string& out);
void store_parse_jsonl(const char* data, size_t n, std::vector<AisRecord>& out);

#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);
void local_load_today(FFTViewer& v);
#endif

} // namespace ais_mod
