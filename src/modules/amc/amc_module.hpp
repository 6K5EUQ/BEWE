#pragma once
// ── AMC 모듈 내부 공유 선언 (모듈 밖에서 include 금지) ───────────────────────
#include "amc_meta.hpp"
#include "config.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

struct FFTViewer;

namespace amc_mod {

extern std::mutex              mtx;
extern std::vector<AmcRecord>  log;
constexpr int LOG_MAX = 100000;

// 워커 슬롯 접근자 (amc_module.cpp)
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

// HOST 워커 (amc_decode.cpp)
void worker(FFTViewer& v, int ch_idx);
// 워커 → 스탬프 + 호스트 아카이브 + framework emit
void host_emit(FFTViewer& v, AmcRecord m);

// 공통 (amc_module.cpp)
void append_log(const AmcRecord& m);
void store_append(const AmcRecord& m);          // ~/BEWE/modules/amc/amc_YYYYMMDD.jsonl
bool store_read_today(std::string& out);
void store_parse_jsonl(const char* data, size_t n, std::vector<AmcRecord>& out);

#ifndef BEWE_HEADLESS
void draw_panel(FFTViewer& v);          // 코어 STATUS 접이식 섹션 내용
#endif

} // namespace amc_mod
