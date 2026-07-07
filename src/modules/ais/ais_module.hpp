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

// Match_AI (ais_ai.cpp): env BEWE_AIS_AI=1 게이트. aicap 파일 append + UDS 데몬 질의
bool ai_enabled();
void host_ai(AisRecord& m, uint32_t out_sr, const float* iq, int n_complex);

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
