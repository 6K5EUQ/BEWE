#pragma once
// ── DF(방탐) UI 대면 어휘 ─────────────────────────────────────────────────
// UI 계층이 아는 타입은 전부 여기 있다. DSP 내부 타입은 새어나가지 않는다.

#include <complex>
#include <cstdint>

namespace df {

// 지원하는 최대 배열 소자 수. 고정 크기 배열 여러 곳이 이걸 쓴다.
inline constexpr int kMaxElements = 8;

enum class Algo : uint8_t { Bartlett = 0, Capon = 1, Music = 2 };

// 배열 소자 번호가 도는 방향. 물리 배선을 데스크에서 확정할 수 없어서 설정으로
// 뺀다 — 실측(V8)에서 방위가 거울로 나오면 재빌드 없이 이걸 뒤집는다.
//   CW  : 안테나 0,1,2,.. 이 위에서 볼 때 시계 방향 (0°, 72°, 144°, ...)
//   CCW : 반시계 방향
enum class Sense : uint8_t { CW = 0, CCW = 1 };

// 배열 기하. UCA 는 반경 하나로 표현되지만 그 밖의 배치는 소자 좌표가 필요하다.
//   Uca    : 반경 + sense 로 좌표를 자동 생성 (구버전과 동일 동작)
//   Ula    : 일직선 등간격. 배열 축에 대해 대칭이라 좌우 모호성이 원리적으로 있다
//            (β 와 180-β 의 조향벡터가 같다) — 그래서 방위를 반평면으로만 읽거나
//            기체를 틀어 두 번 재야 한다.
//   UlaPlus: 일직선 M-1 개 + 1 개를 축 밖으로. 대칭이 깨져 좌우가 갈린다.
//   Custom : 좌표를 직접 준다 (드래그 에디터)
enum class ArrayType : uint8_t { Uca = 0, Ula = 1, UlaPlus = 2, Custom = 3 };

// 소자 좌표 (m). 배열 중심 기준, x=동 y=북. 방위 β 의 평면파에 대한 위상은
// k*(x sin β + y cos β) 이므로 UCA 도 이 표현의 특수해다.
struct ArrayGeom {
    int    n = 0;
    double x[kMaxElements] = {};
    double y[kMaxElements] = {};
};

// 프리셋 → 좌표. spacing 은 UCA 면 반경, 그 밖에는 소자 간격이다.
// 좌표는 항상 무게중심이 원점이 되게 만든다 (heading offset 과 독립).
ArrayGeom make_geom(ArrayType t, int elements, double spacing_m, Sense sense);

enum class LinkState : uint8_t {
    Down = 0,      // 소켓 없음
    Connecting,    // 접속/핸드셰이크 중
    Calibrating,   // 스트림은 오는데 아직 못 쓴다 (플래그 미충족 / CAL 버스트)
    Streaming,     // 정상. DF 가능
};

enum class Status : uint8_t {
    Ok = 0,
    NoSignal,        // 수락 규칙이 거부 — 이 채널엔 잡음뿐
    LinkDown,        // DAQ 연결 없음
    NotCalibrated,   // delay_sync/iq_sync 미성립이 너무 오래
    BandOutOfSpan,   // 요청 채널이 DAQ 의 ±fs/2 밖
    DcOverlap,       // LO 누설 위에 얹혀 있어 쓸 빈이 너무 적다
    TooFewChannels,  // active_ant_chs < 3
    ArrayMismatch,   // 헤더의 M 이 설정된 소자 수와 다름
    Timeout,         // 프레임 예산 안에 충분히 못 모음
    BadRequest,      // 대역폭 0/음수 등
    Cancelled,
    Overdrive,       // ADC 클리핑으로 쓸 프레임이 없었다 (gain 을 내려야 한다)
};

const char* status_text(Status s);

// ── 측정 요청 ─────────────────────────────────────────────────────────────
struct Request {
    double   center_hz    = 0.0;   // 채널 절대 중심주파수
    double   bandwidth_hz = 0.0;   // 채널 폭
    int      frames       = 3;     // 평균낼 "쓸 수 있는" DATA 프레임 수
    Algo     algo         = Algo::Music;
    int      signal_dim   = 1;     // MUSIC 모델 차수
    uint32_t seq          = 0;     // Result 에 그대로 돌아온다
    int      ui_tag       = -1;    // BEWE 채널 배열 인덱스. 엔진은 안 읽는다
    int      ui_dnum      = 0;     // 화면 표시번호. 엔진은 안 읽는다
    // 버스트 신호용. 요청이 도착한 뒤 오는 프레임을 기다리지 않고, 엔진이 들고
    // 있는 **직전 프레임들**부터 적분한다. AIS 처럼 26 ms 만 켜졌다 꺼지는 신호는
    // 스컬치가 열린 걸 보고 요청을 보내는 사이 이미 끝나 있어, 앞만 보는 기존
    // 경로로는 잡을 수가 없다.
    bool     use_backlog  = false;
};

// ── 측정 결과 ─────────────────────────────────────────────────────────────
struct Result {
    uint32_t seq    = 0;
    int      ui_tag = -1;
    int      ui_dnum = 0;
    Status   status = Status::Cancelled;

    double bearing_deg     = 0;  // 최종 보고값 (heading offset 반영)
    double bearing_rel_deg = 0;  // 안테나 0 기준, offset 반영 전 (진단용)
    double confidence_db   = 0;  // Bartlett PAPR — 수락 판정에 쓰는 값
    double eig_snr_db      = 0;  // 10log10((lmax-lmin)/lmin)
    double power_dbfs      = 0;  // 채널 내 전력, full-scale = 1.0

    double   center_hz = 0, bandwidth_hz = 0, effective_bw_hz = 0;
    uint64_t daq_center_hz = 0, daq_fs_hz = 0;

    int      frames_used = 0, frames_discarded = 0;
    double   n_eff_looks = 0;
    int      elements    = 0;
    Algo     algo        = Algo::Music;
    uint32_t overdrive_mask = 0;
    double   ambiguity_ratio = 0;   // (2 r sin(pi/M)) / (lambda/2). >1 이면 격자엽

    // ── 진단 (추정기가 이미 계산하던 값들) ────────────────────────────────
    double   algo_papr_db  = 0;     // 보고 알고리즘의 PAPR. confidence_db 는 Bartlett 것
    double   eval[kMaxElements] = {};   // 오름차순 고유값
    double   diag_spread_db = 0;    // max/min diag(R) (dB)
    bool     imbalance = false;     // 소자 전력이 중앙값 대비 10배 밖

    // 주엽 밖 국소최대 상위 2개. alt_deg 는 heading offset 반영된 최종 방위,
    // alt_db 는 피크 대비 dB (<=0). 5소자 UCA 는 700 MHz 에서 정확히 180도에
    // -4.8 dB 사이드로브가 있는데 ambiguity_ratio 로는 안 잡힌다 (ULA 휴리스틱).
    double   alt_deg[2] = {};
    double   alt_db[2]  = {};
    int      alt_n      = 0;

    // 배열이 실제로 본 조향벡터 (주 고유벡터, 소자 0 위상 0 으로 정규화).
    // 매니폴드 캘리브레이션이 이론값과 비교해 방위별 보정을 만든다.
    std::complex<double> principal[kMaxElements] = {};

    float    spectrum_db[360] = {}; // 보고 알고리즘, 최대 정규화 dB
    int64_t  t_start_ms = 0, t_end_ms = 0;
    char     note[96] = {};
};

// ── DAQ 링크 상태 (설정 패널·상태바가 폴링) ───────────────────────────────
struct DaqStatus {
    LinkState link   = LinkState::Down;
    bool      usable = false;   // 마지막 프레임이 DF 에 쓸 수 있었나

    uint32_t sync_state = 0, delay_sync_flag = 0, iq_sync_flag = 0;
    uint32_t noise_source_state = 0, adc_overdrive_flags = 0;
    uint32_t active_ant_chs = 0, cpi_length = 0;
    uint64_t rf_center_hz = 0, sampling_hz = 0;
    uint32_t if_gain_tenths[8] = {};
    char     hardware_id[17] = {};

    uint64_t frames_ok = 0, frames_cal = 0, frames_dummy = 0, frames_bad = 0;
    uint64_t cpi_gaps = 0, reconnects = 0;
    double   frame_rate_hz = 0, recv_mbps = 0;
    int64_t  last_frame_wall_ms = 0;
    char     last_error[96] = {};
};

} // namespace df
