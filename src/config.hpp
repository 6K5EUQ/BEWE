#pragma once
#include <cstdint>
#include <array>

// ── BEWE version (창 제목 및 About 표시용) ──────────────────────────────────
// SemVer: vMAJOR.MINOR.PATCH — 자세한 정책은 CLAUDE.md 참조
#define BEWE_VERSION "v15.26.0"

// ── 역할 축 ────────────────────────────────────────────────────────────────
// HOST = CLI 빌드, JOIN = GUI 빌드. BEWE_HEADLESS 에서 파생하므로 CMake 노브가
// 하나뿐이고 두 마커가 어긋날 수 없다 (BEWE_HEADLESS 없는 BEWE_HOST_BUILD 는
// 존재하지 않는 조합). BEWE_HEADLESS = "UI 없음", BEWE_HOST_BUILD = "HOST 역할".
#ifdef BEWE_HEADLESS
  #define BEWE_HOST_BUILD 1
#endif

#ifdef BEWE_HEADLESS
  typedef uint32_t ImU32;
  #define IM_COL32(R,G,B,A) \
      (((ImU32)(A)<<24)|((ImU32)(B)<<16)|((ImU32)(G)<<8)|((ImU32)(R)))
#else
  #include <imgui.h>
#endif

// ── Hardware (hw_config.hpp 참조) ─────────────────────────────────────────
// RX_GAIN: BladeRF=10, RTL-SDR=396 (39.6dB, 0.1dB 단위), Pluto=35dB
#define BLADERF_RX_GAIN        10
#define RTLSDR_RX_GAIN_TENTHS  396   // 39.6 dB
#define PLUTO_RX_GAIN_DB       35    // AD9363 manual gain 0~71 dB

// ── FFT / Display ─────────────────────────────────────────────────────────
#define DEFAULT_FFT_SIZE       8192
// zero-padding 배수 — 순수 시각용(bin 사이 sinc 보간). 헤드리스는 로컬 디스플레이가
// 없으므로 1: FFT 크기 4배·행 변환/양자화/브로드캐스트 bin 수 4배·fft_data RAM 4배가
// 전부 사라진다. JOIN 은 와이어의 fft_input_size 로 pad=1 프레임을 인지하고 깊은 줌에서
// Catmull-Rom 보간으로 시각 해상도를 복원한다 (ui.cpp draw_spectrum_area).
#ifdef BEWE_HEADLESS
  #define FFT_PAD_FACTOR       1
#else
  #define FFT_PAD_FACTOR       4
#endif
#define TIME_AVERAGE           200
#define MAX_FFTS_MEMORY        2500   // ~1분
// fft_data 히스토리 깊이: GUI 는 워터폴 스크롤백/TM 용 2500행, 헤드리스 CLI 는
// 최신행 소비자(스퀄치/브로드캐스트) + LWF catch-up 마진(~24-56s)만 필요 → 1024행.
// 메타 배열(row_write_pos/row_wall_ms/iq_row_avail)은 양쪽 모두 MAX_FFTS_MEMORY 유지.
#ifdef BEWE_HEADLESS
  #define FFT_HISTORY_ROWS     1024
#else
  #define FFT_HISTORY_ROWS     MAX_FFTS_MEMORY
#endif
#define HANN_WINDOW_CORRECTION 2.67f
#define NUTTALL_WINDOW_CORRECTION 3.91f  // 1/(a0²+(a1²+a2²+a3²)/2) for Nuttall
#define COLORMAP_LUT_SIZE      65536  // 워터폴 컬러맵 해상도 (was 4096)
#define AXIS_LABEL_WIDTH       50
#define BOTTOM_LABEL_HEIGHT    30
#define TOPBAR_H               32.0f

// ── IQ Ring ───────────────────────────────────────────────────────────────
#define IQ_RING_CAPACITY       (1 << 22)
#define IQ_RING_MASK           (IQ_RING_CAPACITY - 1)

// ── Decoder gate (dec_gate) — 디코드 워커 무신호 스킵용 ────────────────────
// 오디오 스컬치(sq_gate)보다 관대하게 잡는다: 약신호 버스트를 절대 놓치지 않는 게
// 우선이고, 절감은 "확실히 아무것도 없는 구간"에서만 취한다.
#define DEC_GATE_MARGIN_DB     6.0f   // sq_threshold 대비 추가 여유 (thr-6dB 넘으면 열림)
#define DEC_GATE_HOLD_MS       2000   // 마지막 검출 이후 유지 (버스트 간 짧은 공백 흡수)
#define DEC_GATE_PREROLL_MS    300    // 열림 시 IQ ring 되감기 — FFT 검출지연 흡수

// ── Detect (에너지 디텍션 / 자동 채널 협대역화) ────────────────────────────
// Detect 는 간헐 버스트를 잡는 기능이다 (연속 신호는 사용자가 수동 필터를 건다).
// 검출 임계로 sq_threshold(대역 전체 공통 스칼라 하나)를 쓰면, 탐색 대역이 넓고 그 안에
// 상시 강한 신호(인접국/스퍼/DC)가 하나라도 있을 때 임계가 그놈을 따라 올라가 약한
// 협대역 버스트를 영영 못 본다 — 대역을 넓힐수록 감도가 떨어지는 구조였다.
// → arm 시점의 스펙트럼을 bin 별로 평균내어 기준선으로 굳히고(detect_base.hpp),
//    bin 마다 "기준선 + 마진" 을 넘는 것만 신호로 본다. 상시 존재하는 것은 기준선에
//    흡수되어 자동 무시되고, 새로 나타난 것만 잡힌다.
#define DET_BASE_MS            1000   // 기준선 수집 창 (arm 후 이만큼 평균)
// 기준선 EMA 시정수. 이동하는 스퍼(전원 노이즈 등)를 흡수하되, 수초짜리 교신은 흡수하지
// 않을 만큼 느려야 한다. lock/hold 중에는 갱신을 멈추므로 듣고 있는 신호는 영향 없다.
#define DET_BASE_EMA_TAU_MS    8000
#define DET_MIN_RUN_BINS       2      // 1-bin 스파이크/스퍼는 신호로 치지 않는다
// 임계 아래로 내려가는 짧은 갭은 같은 신호로 이어붙인다 (AM 반송파-사이드밴드 딥, FM
// 편이 널). 갭을 bin 수로 재면 안 된다 — bin 폭이 sample_rate/fft_size 라 설정마다
// 수백 Hz ~ 수 kHz 로 달라져서, 넓은 대역에서는 나란한 두 교신까지 한 덩어리로 묶어
// 필터가 둘을 통째로 감싸 버린다. 주파수로 재서 설정과 무관하게 같은 폭을 잇는다.
// 2kHz: 변조 딥보다 넓고, 채널 간격(12.5/25kHz)보다는 한참 좁다.
#define DET_GAP_KHZ            2.0f
// 마진(기준선 대비 몇 dB 를 넘어야 신호로 보나)은 채널마다 조절한다. detect 채널에서는
// sq_threshold 필드를 절대 dB 가 아니라 이 마진값으로 재해석한다 — 어차피 detect 채널은
// 잡고 있는 동안 스컬치 게이트를 검출기가 대신 열어 주므로 절대 임계가 놀고 있다.
// (필드를 재사용하니 와이어/host_state 포맷이 그대로다. 슬라이더·CH_SYNC·저장이 전부
//  기존 경로를 탄다.) 자동 캘리브레이션은 detect 채널을 건너뛴다 — 안 그러면 마진값을
// 절대 dB 로 덮어쓴다.
#define DET_MARGIN_DEF_DB      10.0f  // arm 시 기본 마진
#define DET_MARGIN_MIN_DB      0.0f
#define DET_MARGIN_MAX_DB      40.0f

// ── Audio ─────────────────────────────────────────────────────────────────
#define AUDIO_SR               48000u
#define AUDIO_DEVICE           "default"

// ── Channel ───────────────────────────────────────────────────────────────
// Active(가시대역 안) + Holding(밖) 합산 채널 필터 슬롯 상한.
// active/holding 분리 쿼터가 아니라 단일 풀 — 분류는 가시대역 안/밖으로 동적 결정.
#define MAX_CHANNELS           50

// ── Per-channel colors ────────────────────────────────────────────────────
// ch 0~9 는 기존 고정 팔레트 유지, 10~MAX_CHANNELS-1 은 golden-angle HSV 자동 생성.
static const ImU32 CH_BORD_BASE[10] = {
    IM_COL32(255,220, 50,220), IM_COL32( 50,200,255,220),
    IM_COL32(255, 90, 50,220), IM_COL32(180, 60,255,220),
    IM_COL32( 50,255,110,220), IM_COL32(255,120,200,220),
    IM_COL32(255,200,  0,220), IM_COL32(  0,230,200,220),
    IM_COL32(200,100, 50,220), IM_COL32(100,180,255,220)
};
static const ImU32 CH_FILL_BASE[10] = {
    IM_COL32(255,220, 50, 30), IM_COL32( 50,200,255, 30),
    IM_COL32(255, 90, 50, 30), IM_COL32(180, 60,255, 30),
    IM_COL32( 50,255,110, 30), IM_COL32(255,120,200, 30),
    IM_COL32(255,200,  0, 30), IM_COL32(  0,230,200, 30),
    IM_COL32(200,100, 50, 30), IM_COL32(100,180,255, 30)
};
static const ImU32 CH_SFIL_BASE[10] = {
    IM_COL32(255,220, 50, 75), IM_COL32( 50,200,255, 75),
    IM_COL32(255, 90, 50, 75), IM_COL32(180, 60,255, 75),
    IM_COL32( 50,255,110, 75), IM_COL32(255,120,200, 75),
    IM_COL32(255,200,  0, 75), IM_COL32(  0,230,200, 75),
    IM_COL32(200,100, 50, 75), IM_COL32(100,180,255, 75)
};
inline ImU32 ch_hsv(int i, uint8_t a){
    float h = i * 0.61803398875f; h -= (float)(long)h;   // golden-angle frac, [0,1)
    float v = 1.0f, s = (a >= 200 ? 0.85f : 0.80f);      // 테두리는 살짝 더 채도
    float hf = h * 6.0f; int seg = (int)hf; float f = hf - seg;
    float p = v*(1-s), q = v*(1-s*f), t = v*(1-s*(1-f)), r=v,g=v,b=v;
    switch(seg % 6){
        case 0: r=v; g=t; b=p; break; case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break; case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break; default: r=v; g=p; b=q; break;
    }
    return IM_COL32((int)(r*255), (int)(g*255), (int)(b*255), a);
}
inline std::array<ImU32,MAX_CHANNELS> make_ch_palette(const ImU32* base, uint8_t a){
    std::array<ImU32,MAX_CHANNELS> arr{};
    for(int i=0;i<MAX_CHANNELS;i++) arr[i] = (i<10) ? base[i] : ch_hsv(i,a);
    return arr;
}
static const std::array<ImU32,MAX_CHANNELS> CH_BORD = make_ch_palette(CH_BORD_BASE,220);
static const std::array<ImU32,MAX_CHANNELS> CH_FILL = make_ch_palette(CH_FILL_BASE, 30);
static const std::array<ImU32,MAX_CHANNELS> CH_SFIL = make_ch_palette(CH_SFIL_BASE, 75);

// ── FFT File Header ───────────────────────────────────────────────────────
struct FFTHeader {
    char     magic[4];
    uint32_t version, fft_size, sample_rate;
    uint64_t center_frequency;
    uint32_t num_ffts, time_average;
    float    power_min, power_max, reserved[8];
};