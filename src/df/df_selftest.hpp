#pragma once
// DF 수치 검증. 모든 빌드에 포함되고 --df-selftest 로 돌린다.
// 반환값 = 실패한 검사 수 (0 이면 전부 통과).
namespace df { int run_selftest(int verbosity); }
