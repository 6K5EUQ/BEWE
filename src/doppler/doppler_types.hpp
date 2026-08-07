// ── 도플러 탐지 자료형 (GUI 무관, 헤드리스 빌드 가능) ─────────────────────────
//
// HIST 스펙트로그램에서 뽑은 주파수-시간 트랙과 그 S곡선 적합 결과. 위성 매칭기가
// 이걸 받아 후보 목록을 만든다.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace Doppler {

// TrackPoint::flags
enum : uint8_t {
    TP_INTERP    = 1,   // 서브빈 보간 성공
    TP_COASTED   = 2,   // 이 프레임엔 피크가 없어 예측으로 이었음
    TP_SATURATED = 4,   // 바이트 255 포화 — 포물선 대신 중심 사용
    TP_EDGE      = 8,   // 대역 가장자리 근접
};

struct TrackPoint {
    double   t_utc  = 0;   // 절대 UTC 초 (소수)
    uint32_t row    = 0;   // 원본 행 — UI 가 되짚어 갈 수 있게
    double   f_hz   = 0;   // 절대 주파수, 서브빈 보정 후
    float    snr_db = 0;   // bin별 잡음바닥 위 dB
    float    sigma_hz = 0; // 점별 주파수 불확실도 (적합 가중치)
    uint8_t  flags  = 0;
};

// truncation 비트 — HIST 는 매시 회전해 실제 패스가 두 파일로 쪼개진다.
enum : uint8_t { TRUNC_HEAD = 1, TRUNC_TAIL = 2 };

// f(t) = A - B*x/sqrt(1+x^2),  x = (t - t_tca)/tau
// TCA 에서 시선속도가 0 이므로 f(t_tca) = A 가 정확히 성립 — 속도를 몰라도 A 가 곧
// 정지주파수다. 최대 기울기는 B/tau (TCA 에서).
struct SCurveFit {
    double t_tca_utc        = 0;   // 매칭 1차 키
    double f_center_hz      = 0;   // A — 정지주파수 추정
    double f_center_sigma_hz= 0;
    double half_swing_hz    = 0;   // B
    double tau_s            = 0;   // d/v
    double max_slope_hz_s   = 0;   // B/tau
    double slant_km         = 0;   // tau * v_assumed
    // t_tca 정확도는 적합 시그마가 아니라 row_rate 오차가 지배한다 (1초 창 평균 +
    // 업링크 드롭 구멍). 600s 트랙에서 1% 면 TCA 6초. 매처가 창을 이만큼 열어야 한다.
    double time_sigma_s     = 0;
    double rms_resid_hz     = 0;
    double rms_resid_bins   = 0;
    float  r2          = 0;
    float  lin_ratio   = 0;   // RMS_line / RMS_S — 선형 드리프트 배제
    float  quad_ratio  = 0;   // RMS_quad / RMS_S
    float  antisym     = 0;   // 잔차 반대칭성
    float  mono_frac   = 0;   // 단조 감소 비율
    float  swing_ratio = 0;   // B / (A*v/c) — 지상 이동체 배제의 주력
    bool   tca_inside  = false;
    uint8_t truncation = 0;
    bool   valid       = false;
};

struct Candidate {
    uint32_t    id = 0;
    std::string file_path, station;
    double      station_lat_deg = 0;       // +N
    double      station_lon_east_deg = 0;  // +E — 헤더의 서경 양수를 여기서 이미 뒤집음
    uint64_t    center_freq_hz = 0, sample_rate_hz = 0;
    uint32_t    fft_size = 0, fft_input_size = 0;
    double      bin_hz = 0, row_rate_hz = 0;
    double      t_start_utc = 0, t_end_utc = 0;
    double      f_min_hz = 0, f_max_hz = 0;
    float       snr_med_db = 0, snr_max_db = 0, occupancy = 0;
    SCurveFit   fit;
    float       score = 0;
    std::string reject_reason;             // 비면 통과
    std::vector<TrackPoint> pts;
};

// 파일 1회 보정 결과. 전부 **선형(fftshift된) bin 인덱스** 기준.
struct Calib {
    std::vector<float>   nf_db;        // bin별 잡음바닥 (25 퍼센타일)
    std::vector<uint8_t> mask;         // 1 = 쓰지 않음 (DC/스퍼/대역끝)
    float thr_db       = 0;            // 검출 임계 (바닥 위 dB)
    float sigma_db     = 0;            // 바닥 주변 요동 추정
    float mainlobe_bins= 0;            // 측정된 -3dB 메인로브 폭
    float fold_bias    = 0;            // max-hold 폴드 서브빈 보정 (0 또는 0.375)
    bool  fold_detected= false;
    uint32_t n_masked  = 0;
    uint64_t rows_sampled = 0;
    bool  valid = false;
};

struct ExtractParams {
    // 자동 전수 스캔 기본값. 정밀분석(Meas 박스)은 호출부에서 완화한다.
    float  thr_relax_db   = 0.0f;   // 임계 완화량 (정밀분석에서 2.0)
    double max_gap_s      = 20.0;   // 이 시간 매칭 없으면 트랙 종료
    double min_dur_s      = 90.0;
    float  min_occupancy  = 0.55f;
    int    max_tracks     = 32;
    int    max_peaks_frame= 64;
    uint32_t dc_mask_bins = 16;     // DC 험프는 ±10bin 폭 +16dB — 기존 1bin 마스크는 부족
    float  edge_frac      = 0.015f; // 대역 양끝 컷
    uint32_t force_L      = 0;      // 0 = 자동 (정밀분석은 1)
};

struct ExtractStats {
    uint32_t frames = 0, peaks = 0, tracks_seeded = 0, tracks_kept = 0;
    uint32_t interference_frames = 0;
    uint32_t L = 0;
    double   frame_dt_s = 0;
    double   ms_calib = 0, ms_extract = 0;
};

} // namespace Doppler
