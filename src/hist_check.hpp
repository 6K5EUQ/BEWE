#pragma once
// HIST 정합성 대조 (v13.12) — CLI HOST 전용.
//
// 로컬 .bewehist 는 이제 "Central 이 전 행을 받았음이 증명될 때까지 남겨두는 사본" 이다
// (long_waterfall.cpp 의 finalize DIRTY 경로). 이 모듈이 그 증명을 세우고 뒤처리를 한다.
//
// 동작은 파일 단위다 — 하루치 일괄 스캔이 아니라 정각에 파일 하나가 finalize 될 때
// 그 파일만 Central 과 대조한다. 대조는 헤더 start_utc 로 매칭한다 (Central 은 스트림이
// 일찍 끊기면 끝시각 기준 이름을 쓰므로 파일명이 다를 수 있다).
//
//   Central rows >= local rows  → 이미 다 갖고 있음 → 로컬 삭제
//   Central rows <  local rows  → 빠진 [central_rows, local_rows) 만 세그먼트 파일로
//                                 떼어 MissionPush 로 업로드 (통파일 재전송 안 함)
//   Central 에 아예 없음        → 통파일 업로드
//   Central 무응답/구버전       → 아무것도 안 함, 로컬 보존 (안전 측 실패)
//
// 원본 삭제는 "다음 대조에서 커버가 증명될 때" 일어난다 — 세그먼트 ACK 하나만 보고
// 지우지 않는다. 그래서 몇 번을 돌려도 안전하고(idempotent), 중간에 죽어도 데이터가
// 사라지지 않는다.

#include <cstdint>
#include <string>

class FFTViewer;
class CentralClient;

namespace HistCheck {

void start(FFTViewer* v, CentralClient* cli);
void stop();

// Central → HOST HIST_STAT(0x5D) 수신 주입 (central_client 콜백에서 호출).
void on_hist_stat(const uint8_t* bewe_pkt, size_t len);

// 정각 finalize 로 파일 하나가 닫힌 직후 호출 (보존된 경우만).
// 그 파일 + 같은 미션 dir 에 남아 있던 이전 보존분을 함께 대조한다.
void notify_finalized(const std::string& path);

// "/hist check [station] [code]" — 운용자 수동 실행. 인자 없으면 활성 미션 기준.
// 결과를 CLI 로그로 출력한다.
void run_command(const char* args);


} // namespace HistCheck
