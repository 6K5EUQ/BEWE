// ── HIST 뷰어 도플러 패널 진입점 (GUI 전용) ──────────────────────────────────
#pragma once
#include "doppler_types.hpp"
#include "doppler_match.hpp"
#include <functional>

struct ImDrawList;
class HistReader;

namespace DopplerView {

bool  panel_open();

// 정보줄에 그리는 토글 버튼. 눌렸으면 true (좌표 없는 파일이면 비활성).
bool  toolbar_button(const HistReader& R);
// 패널 열기/닫기. 열면 전 파일 스캔을 시작한다 (LIVE 는 제외 — 계속 자란다).
void  toggle(const HistReader& R, const Doppler::ExtractParams& P);
// Meas 박스 정밀분석.
void  start_refine(const HistReader& R, uint32_t row_lo, uint32_t row_hi,
                   uint32_t lin_lo, uint32_t lin_hi, const Doppler::ExtractParams& P);

// 우측 패널. 반환 = 쓴 폭 (0 = 닫힘).
float draw_panel(const HistReader& R, float h);

// 스펙트로그램 오버레이. 뷰어의 좌표 클로저를 그대로 받아 줌/팬 상태를 상속한다.
void  draw_overlay(ImDrawList* dl, const HistReader& R,
                   const std::function<float(double)>& row_to_px,
                   const std::function<float(double)>& lin_to_py);

void  on_close();

} // namespace DopplerView
