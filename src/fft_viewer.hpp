#pragma once
#include "config.hpp"
#include "bewe_paths.hpp"
#ifdef BEWE_HOST_BUILD
#include "net_server.hpp"   // HOST 전용. JOIN(GUI) 빌드는 NetServer 를 아예 모른다 —
                            // 이 가드가 서버 코드의 GUI 재유입을 컴파일 단계에서 막는다.
                            // (net_protocol/channel/config 는 바로 아래 net_client 가 공급.)
#endif
#include "net_client.hpp"
#include "hw_config.hpp"
#include "channel.hpp"
#include "audio_playback.hpp"
#include "mission.hpp"

#ifndef BEWE_HEADLESS
  #include <GL/glew.h>
  #include <GLFW/glfw3.h>
  #include <imgui.h>
#else
  typedef unsigned int GLuint;
#endif
#ifdef BEWE_HOST_BUILD
// SDR 드라이버 헤더는 HOST 빌드만 본다. JOIN 은 하드웨어를 열지 않으므로
// 이 헤더도, 링크할 .so 도 필요 없다 (JOIN PC 에 -dev 패키지 불필요).
#include <libbladeRF.h>
#include <rtl-sdr.h>
#endif
#include <fftw3.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <climits>
#include <functional>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <set>
#include <deque>
#include <condition_variable>
#include <memory>
#include <unordered_map>
#include <utility>
#include <sys/types.h>

// ── autoscale dB 창 (HOST 실시간 / JOIN 실시간 / HIST 파일 공용) ─────────────
// **노이즈플로어가 창 바닥 25% 에 오도록** 창을 잡는다. 예전엔 창을 peak 기준으로
// 잡았는데, 그러면 신호가 없는 대역에서 잡을 게 없어 최소 스팬(20 dB)에 걸리고
// 노이즈가 화면의 4분의 1 높이까지 올라와 워터폴이 통짜 하늘색이 됐다. 반대로
// 신호가 센 대역(FM 방송)에서는 우연히 창이 넓어져 보기 좋았는데, 그건 공식이
// 맞아서가 아니라 peak 가 높았던 덕이다.
//
// lo = 그 대역에서 실제로 관측된 **최솟값**이다. 안티앨리어싱 필터 스커트(대역
// 좌우 끝)가 여기 들어온다 — 그 아래로 창을 잡아야 가장자리가 잘리지 않는다.
// noise(하위 분위수)가 아니라 lo 를 하한 기준으로 삼는 이유가 이것이다.
//
// 세 경로가 반드시 같은 창을 써야 한다 — 어긋나면 같은 신호가 HOST·JOIN·HIST 에서
// 다른 색으로 보인다.
inline void autoscale_db_window(float noise, float peak, float lo,
                                float& out_min, float& out_max){
    // 관측 최솟값보다 조금 아래에서 시작 (가장자리가 창 밖으로 나가지 않게)
    if(!(lo < noise)) lo = noise - 1.f;          // 방어: lo 가 이상하면 noise 기준
    out_min = lo - 2.f;

    // 잡음 **상단**을 안다. noise(하위 15%)는 잡음 분포의 아래쪽이라 그것만 보면
    // 잡음의 위쪽 절반이 어디까지 올라오는지 모른다.
    //
    // 상단은 상위 분위수로 못 잰다 — 신호가 대역의 몇 %를 차지하는지 모르므로
    // 99% 분위수가 신호권으로 끌려간다 (실측: 신호 5% 만 있어도 -65 -> -38).
    // 대신 하위 두 분위수의 간격으로 잡음의 폭을 재고 그만큼 위로 올린다.
    // 계수 2.2 는 정규분포에서 0.5%~15% 간격 대비 15%~99.5% 간격의 비율이다.
    // 합성 검증: 신호가 대역의 0/5/30% 를 차지해도 오차 2 dB 이내.
    const float width = noise - lo;              // >= 0
    const float noise_top = noise + 2.2f * width;

    // 신호가 있느냐 없느냐로 창을 다르게 잡는다. 하나의 공식으로는 둘 다 만족할
    // 수 없기 때문이다 — 잡음이 바닥 25% 를 차지하게 하면 창이 그만큼 넓어지고,
    // 그 넓은 창 안에서 신호는 필연적으로 낮은 곳에 눌린다 (100 MHz 실측:
    // 잡음상단과 신호피크 간격이 34 dB 뿐이라 신호가 화면 32% 에 머물렀다).
    //
    // 판정 마진은 넉넉하다 (실측: 신호 있는 대역 +34/+57 dB, 없는 대역 -2/-4 dB).
    // 잡음보다 6 dB 도 안 높은 신호를 놓치더라도, 그런 신호는 대비를 키워 봐야
    // 잡음에 묻혀 안 보이므로 오판의 대가가 작다.
    if(peak > noise_top + 6.f){
        // 신호 있음 — 신호가 화면 위쪽까지 차게 (대비 우선).
        float headroom = (peak - noise) * 0.5f;
        if(headroom <  5.f) headroom =  5.f;
        if(headroom > 20.f) headroom = 20.f;
        out_max = peak + headroom;
        if(out_max - out_min < 20.f) out_max = out_min + 20.f;
    } else {
        // 신호 없음 — 잡음 중심선이 화면 바닥 25% 에 오게 (워터폴 대비 우선).
        // 기준을 noise_top 이 아니라 noise(하위 15% 분위수) 로 잡는 이유: noise_top 은
        // width(=noise-lo) 에 2.2 배로 끌려가므로, 같은 상수를 써도 대역마다 눈에 보이는
        // 잡음 띠의 높이가 제각각이 된다. 운용자가 실제로 보는 건 띠의 중심이다.
        float span = (noise - out_min) / 0.25f;
        if(span < 20.f) span = 20.f;             // 너무 좁으면 색이 뭉갠다
        out_max = out_min + span;
        if(peak + 3.f > out_max) out_max = peak + 3.f;
    }
}

// ── Global log helper (ui.cpp에서 정의, 모든 .cpp에서 사용 가능) ─────────
extern std::string g_sdr_force; // "" = 자동, "bladerf"|"rtlsdr"|"pluto"
extern bool scan_sdr_present_quiet();  // 상태표시 전용 저빈도 체크 (로그 스팸 없음, hw_detect.cpp 참고)
extern void bewe_log(const char* fmt, ...);
// LOG 오버레이용 글로벌 로그 (col: 0=HOST 1=SERVER 2=JOIN)
extern void bewe_log_push(int col, const char* fmt, ...);

// ── 녹음 .info 자동 생성 (이미 존재하면 덮어쓰지 않음) ─────────────────────
// source_type: "IQ Recording" / "Audio Recording" / "Region IQ" / "Scheduled IQ"
// modulation: "AM"/"FM"/"MAGIC"/"" 등
// freq/bw/duration이 0이면 해당 줄 빈 값으로 둠
void write_default_info_file(const std::string& wav_path,
                             const char* source_type,
                             double freq_mhz,
                             double bw_khz,
                             double duration_sec,
                             const char* modulation,
                             const char* operator_name,
                             const char* station_name,
                             time_t start_wall_time,
                             int utc_offset_hours = INT_MIN,
                             uint32_t sample_rate = 0);  // SigMF meta용 (IQ .sigmf-data); .wav는 무시

// ── FFTViewer ─────────────────────────────────────────────────────────────
// 캡처 FFT 플랜 (wisdom 캐시 + 플래너 thread-safe). learn=true 는 기동 시 MEASURE 학습,
// false 는 런타임 size 변경 — wisdom 있으면 사용, 없으면 ESTIMATE 즉시 폴백(무정지).
fftwf_plan bewe_fft_plan(int n, fftwf_complex* in, fftwf_complex* out, int sign, bool learn);

class FFTViewer {
public:
    // ── RAII: 모든 스레드 멤버가 joinable인 채로 파괴되지 않도록 방어 ────────
    // 정상 경로(ui.cpp / cli_host.cpp cleanup)에서는 스레드가 이미 join된 상태
    // 예외/이탈/정리 누락 시에도 std::terminate 방지
    ~FFTViewer();

    // ── FFT / waterfall data ──────────────────────────────────────────────
    FFTHeader            header;
    std::vector<float>   fft_data;
    GLuint               waterfall_texture=0;
    std::vector<uint32_t> wf_row_buf;

    int   fft_size=DEFAULT_FFT_SIZE*FFT_PAD_FACTOR, time_average=TIME_AVERAGE;
    int   fft_input_size=DEFAULT_FFT_SIZE;  // 실제 입력 샘플 수 (윈도우 길이)
    float* win_buf=nullptr;                 // pre-computed Nuttall window (fft_input_size)
    float* mag_sq_buf=nullptr;              // VOLK magnitude squared buffer (fft_size)
    bool  fft_size_change_req=false; int pending_fft_size=DEFAULT_FFT_SIZE;
    bool  sr_change_req=false; float pending_sr_msps=61.44f; // 샘플레이트 변경 요청
    bool  texture_needs_recreate=false;
    int   current_fft_idx=0, last_wf_update_idx=-1;
    // 마지막으로 FFT 행이 갱신된 시각 (steady ms) — dec_gate fail-open 판정용.
    // 행이 멎으면(spectrum_pause/rx stop/SDR 에러) 게이트를 강제로 열어 디코더가
    // 조용히 정지하는 것을 막는다. update_channel_squelch 가 갱신.
    int64_t sq_row_change_ms = 0;
    float freq_zoom=1, freq_pan=0;
    float display_power_min=-80, display_power_max=0;
    bool  join_manual_scale=false; // JOIN 수동 스케일: true면 HOST pmin/pmax 덮어쓰기 금지
    float spectrum_height_ratio=0.2f;
    float right_panel_ratio=0.0f;
    std::atomic<bool> render_visible{true}; // false=좌측 패널 완전 숨김 → FFT/WF 연산 중단

    // ── 파워 스펙트럼 Max Decay (스펙트럼 빈 영역 좌클릭 On/Off 토글) ─────
    // 0=Off, 1=Max Decay (피크 즉시 반영 + 5 dB/s로 천천히 감쇠)
    // per-bin 저장 > pan/zoom 변경에도 유지, cf/fft_size 변경 시만 리셋
    int  max_hold_mode = 0;
    std::vector<float> max_hold_spectrum;   // per-bin peak (현재 표시값)
    int  last_maxhold_sp_idx = -1;          // 중복 업데이트 방지 (FFT row 변경 감지)
    uint64_t last_maxhold_cf = 0;
    int      last_maxhold_fft_size = 0;

    // ── 노치필터 (파워스펙트럼 Ctrl+우클릭 드래그로 생성, 세션 한정) ───────
    // fft_data는 pristine 유지 - 렌더/워터폴 매핑 시점에만 영역을 주변 노이즈
    // 플로어(±32 bin 평균)로 치환하고 빨간색으로 표시
    struct NotchFilter {
        float freq_lo_mhz;
        float freq_hi_mhz;
        // 프레임 간 안정화용 EMA 상태 (렌더 경로에서 매 프레임 블렌딩 업데이트)
        float lo_lvl    = -80.0f;  // 좌측 인접 median EMA (녹색 선용)
        float hi_lvl    = -80.0f;  // 우측 인접 median EMA
        float lo_spread = 2.0f;    // 좌측 인접 변동 폭 EMA
        float hi_spread = 2.0f;    // 우측 인접 변동 폭 EMA
        float mh_lo_lvl    = -200.0f; // Max Decay용 동일
        float mh_hi_lvl    = -200.0f;
        float mh_lo_spread = 2.0f;
        float mh_hi_spread = 2.0f;
        // 경계 anchor EMA (edge 인접 픽셀의 median을 EMA 블렌딩) > 1프레임 transient 방지
        float edge_lvl_L    = -80.0f;
        float edge_lvl_R    = -80.0f;
        float mh_edge_lvl_L = -200.0f;
        float mh_edge_lvl_R = -200.0f;
        bool  inited    = false;
        bool  mh_inited = false;
        bool  edge_inited    = false;
        bool  mh_edge_inited = false;
    };
    std::vector<NotchFilter> notches;
    std::mutex               notches_mtx;
    static constexpr int     NOTCH_NF_NEIGHBOR_BINS = 32;
    struct NotchDrag {
        bool  selecting = false;
        float drag_x0 = 0, drag_x1 = 0;
    } notch_drag;

    // ── Frequency Band Plan (Central 공유 — 라벨 띠) ─────────────────────
    struct BandSegment {
        float   freq_lo_mhz = 0;
        float   freq_hi_mhz = 0;
        char    label[24]   = {};
        uint8_t category    = 10;       // 0=Broadcast 1=Aero 2=Marine 3=Amateur
                                        // 4=Cell 5=ISM 6=WiFi-BT 7=Mil
                                        // 8=Public-Safety 9=Government 10=Other
        char    description[128] = {};
    };
    std::vector<BandSegment> band_segments;
    std::mutex               band_mtx;
    bool                     band_show = false; // 기본 off (세션 한정 토글)

    // ── System monitor (bottom bar) ───────────────────────────────────────
    float sysmon_cpu=0, sysmon_ghz=0, sysmon_ram=0, sysmon_io=0;
    std::atomic<int>     sysmon_cpu_temp_c{0};   // CPU 온도 (정수 °C, heartbeat 전송용)
    std::atomic<uint8_t> sysmon_bat{255};         // 배터리 % (255=읽을 값 없음)
    std::atomic<uint8_t> sysmon_bat_ac{2};        // 0=방전 중, 1=AC 연결, 2=알 수 없음
    std::atomic<uint8_t> sysmon_uplink_kind{0};   // 상행 회선 0=미상 1=LTE 2=WiFi 3=유선
    std::atomic<uint8_t> sysmon_uplink_bars{255}; // 상행 신호 0~5, 255=세기 없음
    // 네트워크 레이트 (STATUS 패널 / heartbeat 송신용, 1초 창)
    // HOST: 자기 → Central 업로드. JOIN: Central → 자기(접속 기지 1개) 다운로드.
    std::atomic<float> net_up_kbps{0.f};          // HOST 모드에서만 유효
    std::atomic<float> net_down_kbps{0.f};        // JOIN 모드에서만 유효
    // 자기 머신의 recordings 디스크 여유 (JOIN 의 "Local" 표시용 / HOST 자기 disk_stat 송신 시 참조 가능)
    std::atomic<uint64_t> local_disk_free{0};
    std::atomic<uint64_t> local_disk_total{0};

    // ── Timemachine ───────────────────────────────────────────────────────
    // 워터폴/스펙트럼: 항상 2500행 메모리 유지 (위 MAX_FFTS_MEMORY)
    // IQ 롤링: T키로 활성화
    std::atomic<bool> tm_iq_on{false};     // T키: IQ SSD 롤링 활성
    // HIST(.bewehist) 기록은 TM IQ 롤링과 독립 — 미션 활성(active_hist_dir)이면 기록
    // (게이트는 long_waterfall.cpp worker_loop)
    // ── Signal Library / Emitter DB ───────────────────────────────────
    bool                            sig_lib_panel_open = false;   // M키 토글
    std::mutex                      sig_lib_mtx;                  // emitters/sightings 보호
    std::vector<PktEmitterEntry>    sig_lib_emitters;             // Central 캐시
    std::vector<PktSightingEntry>   sig_lib_sightings;            // 선택된 emitter의 sightings (또는 전체)
    bool                            sig_lib_dirty = false;        // overlay 진입 시 list_req 트리거
    std::string                     sig_lib_selected_uid;         // 좌측 테이블 선택 emitter
    std::string                     sig_lib_sightings_filter_uid; // sightings 캐시의 현재 필터
    char                            sig_lib_search[128] = {};
    bool                            sig_lib_show_pending = true;
    bool                            sig_lib_show_auto = true;
    bool                            sig_lib_show_confirmed = true;
    // ── SIGINT Mission System ─────────────────────────────────────────
    // 'A03' (월+일) 코드, UTC 0시 자동 rollover, 신규 녹음은 미션 dir로 라우팅
    struct MissionEntry {
        int      year       = 0;
        char     code[8]    = {};       // "A03"
        time_t   start_utc  = 0;
        time_t   end_utc    = 0;        // 0 = open
        char     started_by[32] = {};
        uint8_t  op_index   = 0;        // 0=HOST, 1..N=JOIN
        uint8_t  rollover   = 0;        // 1 = UTC0 자동 시작
        // Mission metadata captured at start (station ctx, written to mission.json)
        char     station_name[64] = {};
        char     host_name[32]    = {};
        float    lat = 0.f, lon = 0.f;
        char     sdr_kind[24]     = {};   // "BladeRF" / "RTL-SDR" / "Pluto"
        char     antenna[64]      = {};
    };
    mutable std::mutex mission_mtx;
    Mission::State mission_state = Mission::State::IDLE;
    int            mission_year = 0;
    char           mission_code[8]        = {};
    char           mission_started_by[32] = {};
    uint8_t        mission_op_index       = 0;
    time_t         mission_start_utc      = 0;
    time_t         mission_end_utc        = 0;
    // 활성 미션 메타데이터 (start 시점 캡처)
    char           mission_station_name[64] = {};
    char           mission_host_name[32]    = {};
    float          mission_lat = 0.f, mission_lon = 0.f;
    char           mission_sdr_kind[24]     = {};
    char           mission_antenna[64]      = {};
    std::vector<MissionEntry> mission_history;
    bool           mission_modal_open       = false;
    bool           mission_start_modal_open = false;
    bool           mission_end_confirm_open = false;

    // mission_view 의 LOCAL 우클릭 → main 페이지 file_ctx 메뉴 (Signal Analysis/
    // Info/Report/Save DB/Delete) 를 동일하게 트리거. mission_view 가 set 하고,
    // run_streaming_viewer() 매 프레임 루프 시작 시 소비.
    struct PendingFileCtx {
        std::atomic<bool> pending{false};
        std::string filepath;
        std::string filename;
        float       x = 0, y = 0;
    } pending_file_ctx;

    // Mission lifecycle (mission.cpp에 정의, thread-safe)
    bool mission_start(const char* started_by, uint8_t op_index, bool rollover);
    bool mission_end();
    void mission_rollover_utc0();
    void mission_load_history();
    void mission_save_meta_to_disk();
    void mission_broadcast_sync();
    // v3.20.0 migration: 기존 recordings/missions/<YYYY>/<code>/ 구조를
    // station-keyed recordings/missions/<station>/<YYYY>/<code>/ 로 한 번만 이동.
    void mission_migrate_old_layout();
    // 특정 미션의 디렉토리(파일 전체) + history 엔트리 영구 삭제.
    // 활성 미션이면 먼저 mission_end() 후 삭제.
    bool mission_delete(int year, const char* code);
    // 현재 활성 미션 디렉토리. IDLE이면 빈 문자열 (호출자가 차단).
    std::string active_iq_dir() const;
    std::string active_audio_dir() const;
    std::string active_hist_dir() const;
    // 현재 활성 미션 station_name (MissionPush key 채우기용). IDLE이면 빈.
    std::string mission_active_station_name() const;
    // 현재 활성 미션 year/code (MissionPush + file LIST_REQ 용).

    // ── Long Waterfall (24h+ FFT magnitude image) ─────────────────────
    bool              lwf_modal_open = false;     // IMG 버튼 토글 → viewer 모달
    std::atomic<int>  lwf_rotate_seq{0};          // SR/CF/fft_size/IQ on-off 변경 시 ++ → worker가 새 파일
    std::atomic<bool> tm_active{false};    // 스페이스바: 타임머신 뷰 모드
    std::atomic<bool> capture_pause{false};// 캡처 스레드 pause (타임머신과 무관)
    std::atomic<bool> net_bcast_pause{false}; // /chassis 2 reset: 방송 일시 중단
    std::atomic<bool> sdr_stream_error{false};  // SDR 스트림 오류 (뽑힘/초기화 실패)
    // 캡처 스레드가 루프를 빠져나왔는지. /powercycle 이 join 을 무한정 기다리지 않고
    // (블로킹 read 에 갇혔을 수 있다) 포기 시점을 판단하는 데 쓴다.
    std::atomic<bool> cap_exited{false};
    std::atomic<bool> dem_restart_needed{false}; // SR 변경 후 demod 재시작 필요
    std::atomic<bool> wf_area_visible{true};    // 워터폴 영역 실제 표시 여부 (수평바 포함)
    bool tm_iq_was_stopped=false;
    int  tm_freeze_idx=0;                  // 스페이스바 누른 시점의 fft 인덱스
    float tm_offset=0.0f;                  // 현재 보는 과거 오프셋 (초)
    float tm_max_sec=0.0f;                     // 현재 사용 가능한 최대 과거 초

    // 행별 IQ 데이터 존재 플래그 (MAX_FFTS_MEMORY 크기)
    bool iq_row_avail[MAX_FFTS_MEMORY]={};
    // 각 FFT 행 커밋 시점의 tm_iq_write_sample 기록
    // row_write_pos[fi % MAX_FFTS_MEMORY] = 그 행의 IQ 데이터가 끝나는 파일 위치
    int64_t row_write_pos[MAX_FFTS_MEMORY]={};
    // 각 FFT 행 커밋 시점의 wall_time (밀리초)
    int64_t row_wall_ms[MAX_FFTS_MEMORY]={};

    // ── 워터폴 좌측 이벤트 태그 ───────────────────────────────────────────
    struct WfEvent {
        int   fft_idx;   // 이벤트 발생 시점의 fft 인덱스
        time_t wall_time;// 실제 시각
        int   type;      // 0=시간태그(5초단위), 1=TM_IQ Start, 2=TM_IQ Stop
        char  label[32];
    };
    std::vector<WfEvent> wf_events;
    mutable std::mutex   wf_events_mtx;
    int                  last_tagged_sec=-1; // 마지막으로 5초태그 붙인 초

    // fft_idx → wall_time 변환 (wf_events 보간, 없으면 rps 기반 추정)
    // 반환값: Unix timestamp (time_t), 0이면 변환 불가
    // fft_idx → wall_time_ms 변환 (row_wall_ms 직접 참조, 밀리초 정밀도)
    // 반환값: Unix timestamp * 1000 (int64_t), 0이면 변환 불가
    int64_t fft_idx_to_wall_time_ms(int fft_idx) const;

    // IQ 롤링 파일 관리
    // TM_IQ_DIR: BEWEPaths::time_temp_dir() 로 런타임 결정
    static constexpr size_t      TM_IQ_SECS = 60;     // 롤링 길이 (초)
    int      tm_iq_fd=-1;   // unbuffered POSIX fd
    // TM IQ 비동기 writer: 캡처 스레드는 복사+enqueue만, ×16 스케일링/pwrite 는 전용
    // 스레드. 디스크가 느리면(SD 실속) 큐 초과분 drop-oldest — 캡처는 절대 블로킹 안 함.
    static constexpr size_t TM_IQ_QUEUE_MAX_BYTES = 64ull<<20; // 64 MiB
    static constexpr size_t TM_IQ_FREE_MAX = 8;   // 재활용 버퍼 보관 상한 (캡처 청크 단위)
    struct TmIqChunk { int64_t start_sample; std::vector<int16_t> data; };
    std::thread             tm_iq_wr_thr;
    std::atomic<bool>       tm_iq_wr_run{false};
    std::mutex              tm_iq_q_mtx;
    std::condition_variable tm_iq_q_cv;
    std::deque<TmIqChunk>   tm_iq_q;          // tm_iq_q_mtx 보호
    size_t                  tm_iq_q_bytes=0;  // tm_iq_q_mtx 보호
    std::deque<std::vector<int16_t>> tm_iq_free; // 청크 버퍼 재활용 (tm_iq_q_mtx 보호) — 캡처 콜백당 수백 KB 재할당 방지
    std::atomic<uint64_t>   tm_iq_dropped_bytes{0}; // 디스크 지연으로 버린 바이트 누계
    std::atomic<bool>       tm_iq_write_failed{0};  // writer pwrite 실패 → 다음 open 이 close/재생성
    std::mutex              tm_iq_oc_mtx;     // open/close 직렬화 (동시 토글 → thread 이중 assign 방지)
    std::atomic<int64_t> tm_iq_write_sample{0}; // 현재 파일 내 쓰기 샘플 위치 (enqueue 시 증가)
    // 디스크에 실제 기록 완료된 샘플 위치 (writer 갱신) — 읽기(region/TM replay)는
    // enqueue 카운터가 아니라 이 워터마크까지만 신뢰해야 함 (큐 적체분은 아직 파일에 없음)
    std::atomic<int64_t> tm_iq_flushed_sample{0};
    int64_t  tm_iq_total_samples=0;        // 파일 전체 샘플 수 (미리 할당)
    // 초 단위 타임스탬프 배열 [0..TM_IQ_SECS-1]: 각 초 청크의 시작 시각
    time_t   tm_iq_chunk_time[TM_IQ_SECS]={};
    int      tm_iq_chunk_write=0;          // 현재 쓰고 있는 청크 인덱스
    int64_t  tm_iq_chunk_sample_start=0;   // 현재 청크 샘플 시작
    bool     tm_iq_file_ready=false;

    // 영역 녹음 진행 상태
    enum RecState { REC_IDLE, REC_BUSY, REC_SUCCESS } rec_state=REC_IDLE;
    float    rec_anim_timer=0.0f;   // 점 애니메이션용
    float    rec_success_timer=0.0f;// 성공 메시지 표시 시간
    std::atomic<bool> rec_busy_flag{false};

    // 디스플레이용 타임머신 오프셋 FFT 인덱스
    int tm_display_fft_idx=0;

    void tm_iq_open();
    // IQ 롤링 토글 (상단바 IQ LED 클릭 / I 키 공용). JOIN 이면 HOST 에 원격 요청.
    void toggle_tm_iq();
    void tm_iq_close();
    void tm_iq_close_locked();   // tm_iq_oc_mtx 보유 상태에서만 호출 (open 의 실패복구 경로)
    void tm_iq_write(const int16_t* samples, int n_pairs);
    void tm_iq_writer_loop();
    void tm_mark_rows(int fft_idx);
    void tm_update_display();
    bool tm_rec_start();
    void tm_add_time_tag(int fft_idx);
    void tm_add_event_tag(int type); // 1=Start, 2=Stop

    // ── 영역 IQ 녹음 (Ctrl+우클릭 드래그) ───────────────────────────────
    struct RegionSel {
        bool   selecting=false;
        bool   active=false;
        float  drag_x0=0, drag_y0=0, drag_x1=0, drag_y1=0;
        float  freq_lo=0, freq_hi=0;
        int    fft_top=0, fft_bot=0;
        int64_t time_start_ms=0, time_end_ms=0;
        int64_t samp_start=0, samp_end=0; // HOST IQ 좌표 직접 지정 시 사용
        int    lclick_count=0;
        float  lclick_timer=0;

        // 이동/리사이즈 상태
        enum EditMode { EDIT_NONE, EDIT_MOVE,
                        EDIT_RESIZE_L, EDIT_RESIZE_R,
                        EDIT_RESIZE_T, EDIT_RESIZE_B } edit_mode=EDIT_NONE;
        float  edit_mx0=0, edit_my0=0;   // 드래그 시작 마우스 위치
        float  edit_flo0=0, edit_fhi0=0; // 드래그 시작 주파수
        int    edit_ftop0=0, edit_fbot0=0; // 드래그 시작 fft 인덱스
    } region;

    std::string do_region_save_work();

    // ── SA (Signal Analyzer) 패널 ─────────────────────────────────────────
    bool              sa_panel_open  = false;
    int               sa_fft_size    = 1024;
    int               sa_window_type = 0;   // 0=Blackman-Harris 1=Hann 2=Nuttall
    GLuint            sa_texture     = 0;
    int               sa_tex_w       = 0;
    int               sa_tex_h       = 0;
    std::atomic<bool> sa_computing   {false};
    std::thread       sa_thread;
    std::string       sa_temp_path;
    bool              sa_mode        = false;
    float             sa_anim_timer  = 0.0f;  // 로딩 점 애니메이션
    float right_panel_x   = 0.0f;

    // SA 픽셀 버퍼 (스레드 → 메인 스레드 전달)
    std::vector<uint32_t> sa_pixel_buf;
    std::mutex            sa_pixel_mtx;
    std::atomic<bool>     sa_pixel_ready{false};

    void sa_start(const std::string& wav_path);  // 비동기 FFT 계산 시작
    void sa_cleanup();                            // 임시파일 삭제 + 텍스처 해제
    void sa_upload_texture();                     // 메인스레드에서 GL 업로드

    // SA 메타데이터 (WAV BEWE 청크에서 읽음)
    uint64_t sa_center_freq_hz = 0;   // 중심주파수 (Hz)
    int64_t  sa_start_time     = 0;   // 시작 시각 (Unix timestamp)
    uint32_t sa_sample_rate    = 0;   // 출력 샘플레이트 (데시메이션 후)
    int64_t  sa_total_rows     = 0;   // 총 FFT 행 수 (시간축 길이)
    int      sa_actual_fft_n   = 0;   // 실제 FFT 크기 (주파수축 길이)

    // SA 뷰 상태 (줌/팬)
    float    sa_view_x0 = 0.0f;   // 텍스처 UV: 주파수축 시작 [0,1]
    float    sa_view_x1 = 1.0f;   // 텍스처 UV: 주파수축 끝
    float    sa_view_y0 = 0.0f;   // 텍스처 UV: 시간축 시작
    float    sa_view_y1 = 1.0f;   // 텍스처 UV: 시간축 끝

    // SA 범위 선택 (Ctrl+우클릭 드래그)
    bool     sa_sel_active = false;
    float    sa_sel_x0 = 0.f, sa_sel_x1 = 0.f; // 텍스처 UV 좌표
    float    sa_sel_y0 = 0.f, sa_sel_y1 = 0.f;
    bool     sa_sel_dragging = false;
    float    sa_sel_drag_ox = 0.f, sa_sel_drag_oy = 0.f; // 드래그 시작 UV

    // SA 복조 재생

    // ── Scheduled IQ Recording ──────────────────────────────────────────
    struct SchedEntry {
        time_t  start_time   = 0;
        float   duration_sec = 0;
        float   freq_mhz     = 0;
        float   bw_khz       = 0;
        enum Status : int { WAITING=0, ARMED=1, RECORDING=2, DONE=3, FAILED=4 } status = WAITING;
        int     temp_ch_idx  = -1;
        std::chrono::steady_clock::time_point rec_started;
        char    operator_name[32] = {};  // 예약자 (HOST면 login_get_id(), JOIN이면 op_name)
        uint8_t op_index          = 0;   // 0=HOST, 1..N=JOIN op_index
        char    target[32]        = {};  // free-form 식별 라벨 (UI에서 운용자 입력)
        // Mission context — HOST stamps these at add time so mission_view can
        // filter the list. Empty year/code = added outside any mission.
        int     mission_year      = 0;
        char    mission_code[8]   = {};
    };
    static constexpr float SCHED_PRE_ARM_SEC = 5.0f;
    std::vector<SchedEntry> sched_entries;
    std::mutex              sched_mtx;
    int   sched_active_idx  = -1;
    float sched_saved_cf    = 0;
    void sched_tick();
    void sched_arm_entry(int idx);
    void sched_begin_rec(int idx);
    void sched_stop_entry(int idx);
    // Overlap 검사 — pre-arm 윈도우 포함 [start - PRE_ARM, start+dur) 가 기존
    // active entry와 겹치는지 (sched_mtx 잡은 채로 호출).
    // JOIN(GUI)도 ADD 버튼 활성화 판정에 쓰므로 헤더 인라인 — sched_record.cpp 는 HOST 전용.
    bool sched_has_overlap(time_t start, float dur) const {
        time_t a0 = start - (time_t)SCHED_PRE_ARM_SEC;
        time_t a1 = start + (time_t)dur;
        for(const auto& e : sched_entries){
            if(e.status == SchedEntry::DONE || e.status == SchedEntry::FAILED) continue;
            time_t b0 = e.start_time - (time_t)SCHED_PRE_ARM_SEC;
            time_t b1 = e.start_time + (time_t)e.duration_sec;
            if(a0 < b1 && b0 < a1) return true;
        }
        return false;
    }
    // 전체 sched 리스트를 JOIN 클라이언트에 브로드캐스트 (SCHED_SYNC)
    void broadcast_sched_list();         // 내부에서 sched_mtx 잡음
    void broadcast_sched_list_locked();  // 호출자가 이미 sched_mtx를 잡은 상태여야 함
    // 예약 녹음 완료 시 자동 DB 업로드 콜백 (cli_host/ui.cpp에서 설정)
    // args: (file_path, operator_name, info_text)
    std::function<void(const std::string& path, const std::string& op, const std::string& info)> sched_db_upload_fn;
    // 동시에 여러 sched가 끝날 때 업로드를 직렬화 (Central 측 동일-룸 단일 db_fp 슬롯 보호)
    std::mutex sched_db_upload_mtx;

    // sched_begin_rec → start_iq_rec 핸드오프: 다음 IQ 녹음이 SCHED 형식 파일명을 쓰도록.
    // sched_begin_rec에서 채워두고 start_iq_rec가 한 번 사용 후 active=false로 클리어.
    struct PendingSchedMeta {
        bool   active = false;
        time_t start_utc = 0;
        time_t end_utc   = 0;
    } pending_sched_meta;

    // ── LOG 오버레이 (L키 토글) ──────────────────────────────────────────
    bool log_panel_open = false;
    struct LogEntry { char msg[512]; };
    static constexpr int LOG_MAX = 500;
    std::deque<LogEntry> log_buf[3];   // 0=HOST 1=SERVER 2=JOIN (pop_front O(1) — vector erase 는 500×512B memmove)
    std::mutex log_mtx;
    bool log_scroll[3] = {true,true,true};
    void log_push(int col, const char* fmt, ...);

    // ── DEMOD 모듈 패널 (src/modules/ 설치형 모듈 컨테이너) ──────────────
    bool                 demod_panel_open = false;
    // DF(방탐) 설정 전체영역 오버레이. 다른 오버레이와 상호배타.
    bool                 df_panel_open    = false;
    // AIS 지도 크게보기(mv.big) 상태 미러 — 켜지면 앱 상단바+DEMOD 탭바 숨겨 지도만 전체화면.
    // AIS 뷰가 매 프레임 mv.big 값을 여기 반영; 다른 모듈 탭/패널에선 항상 false 로 리셋.
    bool                 ais_fullscreen = false;

    // ── EID (Emitter ID / RF Fingerprint) 패널 ─────────────────────────────
    bool              eid_panel_open = false;
    std::atomic<bool> eid_computing  {false};
    std::thread       eid_thread;
    float             eid_anim_timer = 0.0f;

    // 뷰 모드: 0=Signal(envelope), 1=I/Q, 2=Phase, 3=Frequency
    int eid_view_mode = 0;

    // envelope 데이터
    std::vector<float> eid_envelope;       // sqrt(I²+Q²), 전체 샘플
    std::mutex         eid_data_mtx;
    std::atomic<bool>  eid_data_ready{false};
    int64_t            eid_total_samples = 0;
    uint32_t           eid_sample_rate   = 0;

    // I/Q 분리 데이터
    std::vector<float> eid_ch_i;       // I(t) normalized
    std::vector<float> eid_ch_q;       // Q(t) normalized

    // 순시 위상/주파수
    std::vector<float> eid_phase;      // atan2(Q,I)
    std::vector<float> eid_inst_freq;  // d(phase)/dt

    float eid_phase_detrend_hz = 0.0f;  // sweep line 주파수 오프셋 (Hz)

    // 노이즈 레벨 (Signal 모드용)
    float eid_noise_level = 0.f;

    // center freq (표시용)
    uint64_t eid_center_freq_hz = 0;
    // 원본 WAV bewe 청크 start_time (Save File 시 새 WAV에 보존)
    int64_t  eid_start_time_meta = 0;

    // 뷰 상태 (double: 대용량 샘플 인덱스 정밀도)
    double  eid_view_t0 = 0.0;   // 보이는 시작 (샘플 인덱스)
    double  eid_view_t1 = 0.0;   // 보이는 끝
    float   eid_amp_min = 0.0f;  // 자동 스케일 최소
    float   eid_amp_max = 1.0f;  // 자동 스케일 최대

    // 좌클릭 드래그 영역 선택 (줌)
    bool    eid_sel_active = false;
    float   eid_sel_x0 = 0.f, eid_sel_x1 = 0.f;

    // 뷰 히스토리 (우클릭 뒤로가기)
    std::vector<std::pair<double,double>> eid_view_stack;

    // 태그 영역
    struct EidTag {
        double s0, s1;       // 샘플 인덱스 시작/끝
        ImU32  color;
        char   label[32];
        bool   selected = false;    // 활성화 상태 (클릭 토글)
        float  auto_pri_us = 0;     // 자동 검출 PRI (us)
        float  auto_prf_hz = 0;     // 자동 검출 PRF (Hz)
        int    auto_pulse_count = 0; // 검출된 펄스 수
    };
    void eid_auto_analyze_tag(EidTag& tag);

    // 성상도 재생 상태
    double eid_const_pos = 0;       // 현재 재생 위치 (샘플 인덱스)
    bool   eid_const_playing = false; // 자동 재생 중
    int    eid_const_win = 4096;    // 윈도우 크기 (샘플)
    float  eid_const_zoom = 0.0f;   // 0 = 자동 스케일, >0 = 수동 줌 배율


    // M-th power spectrum 분석 상태
    int    eid_power_order = 1;     // M 값 (1, 2, 4, 8)
    int    eid_power_fft_n = 4096;  // FFT 크기

    // Y축 수동 스케일 (per mode: 0=Amp, 1=I/Q, 2=Phase, 3=Freq)
    float  eid_y_min[4] = {0.f,   -1.f,              -(float)M_PI, -1.f};
    float  eid_y_max[4] = {1.f,    1.f,               (float)M_PI,  1.f};

    // 비트 판단선 (baud mode에서 우클릭 Make Baseline으로 설정)
    bool   eid_baseline_active = false;
    float  eid_baseline_val = 0.f;   // Y축 데이터 값 기준
    int    eid_baseline_imode = 0;   // 설정된 모드 (0=envelope,1=IQ,2=phase,3=freq)

    // 비트 구분 모드 (B키 토글)
    bool   eid_baud_mode = false;
    double eid_baud_s0 = -1;     // 시작 샘플 인덱스 (-1=미설정)
    double eid_baud_s1 = -1;     // 끝 샘플 인덱스 (-1=미설정)
    int    eid_baud_click = 0;   // 0=대기, 1=시작설정됨
    int    eid_baud_drag = -1;   // 드래그 중 선 (-1=없음, 0=시작, 1=끝)
    bool   eid_baud_drag_band = false;    // 밴드 전체 드래그 중
    double eid_baud_band_drag_offset = 0.0; // 드래그 시작 시 s0 기준 마우스 오프셋
    bool    eid_tag_dragging = false;
    float   eid_tag_drag_x0 = 0.f, eid_tag_drag_x1 = 0.f;
    // 임시 선택 영역 (우클릭 드래그 후 확정 전)
    bool    eid_pending_active = false;
    double  eid_pending_s0 = 0.0, eid_pending_s1 = 0.0;
    std::vector<EidTag> eid_tags;

    // Bits 탭 상태
    int    eid_bits_per_row = 128;   // 한 줄당 비트 수 (8~512)
    int    eid_bits_offset = 0;      // 수동 비트 오프셋
    int    eid_bits_view = 0;        // 0=BIN(이진수), 1=HEX(16진수), 2=BITMAP(픽셀)
    int    eid_bits_scroll = 0;      // 스크롤 위치 (줄 단위)
    float  eid_bits_zoom = 1.0f;    // 줌 배율 (1.0 = 기본, Ctrl+휠로 조절)
    float  eid_bits_hscroll = 0.0f; // 수평 스크롤 (픽셀 단위)

    // 스펙트로그램 통합 뷰 히스토리
    struct SaViewEntry { float x0,x1,y0,y1; bool had_bpf; };
    std::vector<SaViewEntry> sa_view_history;

    // BPF 상태 (원본 IQ 백업 + 필터 상태)
    std::vector<float> eid_orig_ch_i;   // 필터 전 원본 I
    std::vector<float> eid_orig_ch_q;   // 필터 전 원본 Q
    bool eid_bpf_active = false;

    // ── Undo/Redo 시스템 ──────────────────────────────────────────────
    struct EidUndoEntry {
        bool has_data = true;   // false = 대형 배열 7개 미보관 (뷰/태그 전용 light 엔트리)
        std::vector<float> envelope, ch_i, ch_q, phase, inst_freq;
        std::vector<float> orig_ch_i, orig_ch_q;
        std::vector<EidTag> tags;
        std::vector<std::pair<double,double>> view_stack;
        std::vector<SaViewEntry> sa_history;
        int64_t total_samples;
        double view_t0, view_t1;
        float sa_vx0, sa_vx1, sa_vy0, sa_vy1;
        bool bpf_active;
        bool baud_mode;
        double baud_s0, baud_s1;
        int baud_click;
        bool baseline_active;
        float baseline_val;
        int baseline_imode;
        bool pending_active;
        double pending_s0, pending_s1;
        float y_min[4], y_max[4];
        float amp_min, amp_max;
        float noise_level;
    };
    std::deque<EidUndoEntry> eid_undo_stack;
    std::deque<EidUndoEntry> eid_redo_stack;
    static constexpr int EID_UNDO_MAX = 10;
    EidUndoEntry eid_snapshot(bool with_data = true) const;
    void eid_restore(const EidUndoEntry& e);
    void eid_push_undo(bool with_data = true);
    void eid_do_undo();
    void eid_do_redo();

    void eid_start(const std::string& wav_path);
    void eid_cleanup();
    void eid_remove_samples(double s0, double s1);
    void eid_select_samples(double s0, double s1);
    void eid_apply_bpf(float uv_lo, float uv_hi);  // UV [0,1] 주파수 좌표
    void eid_undo_bpf();
    void eid_recompute_derived();
    void sa_recompute_from_iq(bool reset_view = false);
    // Save File: 현재 eid_ch_i/q 상태(필터·샘플 수정 반영)를 원본 폴더에 새 WAV로 저장.
    // 파일명: IQ_Filtered_... / Audio_Filtered_... 형식, 중복 시 _2, _3 접미.
    // 원본 .info가 있으면 같은 규칙으로 복사. 반환: 생성 경로(실패 시 "").
    // 기본 저장 경로 계산만 수행 (파일명 결정, 중복 _N 처리)
    std::string eid_default_filtered_path();
    // 지정 경로에 WAV만 저장 (헤더/bewe 청크/stereo int16). .info는 호출자 책임.
    std::string eid_save_filtered_to(const std::string& out_path);

    // .info 파일의 Recorder 필드용 장비 표시명 (HOST/Local=hw.type, JOIN=remote_hw)
    const char* recorder_name() const {
        if(remote_mode && net_cli){
            uint8_t rh = net_cli->remote_hw.load();
            if(rh == 0) return "BladeRF 2.0 micro xA9 (12bit ADC)";
            if(rh == 1) return "RTL-SDR v4 (8bit ADC)";
            if(rh == 2) return "ADALM Pluto SDR (12bit ADC)";
            if(rh == 3) return "KrakenSDR 5ch coherent (8bit ADC, heimdall DAQ)";
            return "";
        }
        return hw_recorder_name(hw.type);
    }
    // 시간 기준: 항상 KST (UTC+9). 시스템 TZ나 station 위치와 무관.
    int utc_offset_hours() const { return 9; }

    // tm_rec 내부 상태
    bool    tm_rec_active=false;
    int64_t tm_rec_read_pos=0; // R키 실행 시 파일 추출
    std::vector<float> current_spectrum;
    int   cached_sp_idx=-1; float cached_pan=-999, cached_zoom=-999;
    int   cached_px=-1;     float cached_pmin=-999, cached_pmax=-999;
    // autoscale: 고정 크기 순환 버퍼 (push_back per FFT 제거)
    // 최대 fft_size*100 샘플 고정 → nth_element 시 재할당 없음
    std::vector<float> autoscale_accum;
    size_t             autoscale_wp=0;   // 순환 버퍼 write pointer
    bool               autoscale_buf_full=false;
    std::chrono::steady_clock::time_point autoscale_last;
    std::chrono::steady_clock::time_point autoscale_check_last{};  // 10s 천장초과 감시 타이머 (JOIN 표시용)
    // HOST 양자화 창의 천장초과 감시 (캡처 스레드 소유 — JOIN 것과 별개다).
    std::chrono::steady_clock::time_point autoscale_ceil_last{};
    // 데드라인 — autoscale 이 "처음" 켜진 시각. 재트리거는 autoscale_last(1초 창)만 되감고
    // 이건 건드리지 않는다. 주파수축 드래그처럼 매 프레임 재트리거가 쏟아지면 1초 창이
    // 영영 못 차서 autoscale 이 굶어죽는데, 이 데드라인이 지나면 모인 만큼으로 강제 확정한다.
    std::chrono::steady_clock::time_point autoscale_start{};
    static constexpr float AUTOSCALE_DEADLINE_S = 6.0f;
    bool  autoscale_init=false, autoscale_active=true;
    // 비-캡처 스레드(set_frequency/init)가 autoscale 재트리거를 요청 → 캡처 스레드가 처리.
    // autoscale_accum/active/init 를 캡처 스레드 밖에서 직접 건드리면 레이스 → 이 플래그로 위임.
    std::atomic<bool> autoscale_req{false};
    std::atomic<bool> spectrum_pause{false};

    // ── Network ──────────────────────────────────────────────────────────
#ifdef BEWE_HOST_BUILD
    NetServer*  net_srv   = nullptr;  // HOST 모드 (JOIN 빌드엔 존재하지 않는다)
#endif
    NetClient*  net_cli   = nullptr;  // CONNECT 모드
    bool        remote_mode = false;  // true = CONNECT 모드 (하드웨어 없음)
    char        host_name[32] = {};   // 접속한 유저 ID (표시용)
    char        host_antenna[32] = {}; // HOST 안테나 자유텍스트 (Central 통해 동기화)
    // 런타임 SDR 교체: UI 클릭 시 아래 플래그/이름 세팅 → 메인 루프가 교체 수행
    std::atomic<bool> pending_sdr_switch{false};
    std::mutex        pending_sdr_mtx;
    std::string       pending_sdr_name; // "bladerf" | "pluto" | "rtlsdr"
    uint8_t     my_op_index   = 0;

    // ── Globe / Station Discovery ─────────────────────────────────────────
    struct DiscoveredStation {
        std::string name;
        std::string station_id;    // relay 모드: 룸 ID; LAN 모드: ""
        float       lat        = 0.f;
        float       lon        = 0.f;
        uint16_t    tcp_port   = 0;
        std::string ip;
        uint8_t     user_count = 0;
        uint8_t     host_tier  = 1;
        double      last_seen  = 0.0; // glfwGetTime()
    };
    std::vector<DiscoveredStation> discovered_stations;
    std::mutex                     discovered_stations_mtx;
    // 기지 좌표 캐시 (name → lat/lon). 로그인/globe 에서 본 host 좌표를 만료 없이 보관 →
    // JOIN 이 room 진입 후(discovered 가 stale 로 비어도) AIS/지도 마커에 계속 활용.
    std::unordered_map<std::string, std::pair<float,float>> station_geo_cache;
    std::mutex                     station_geo_cache_mtx;

    // Station identity (set during HOST mode placement on globe)
    std::string station_name;
    float       station_lat          = 0.f;
    float       station_lon          = 0.f;
    bool        station_location_set = false;

    // 로컬 오디오 출력 선택 (각 PC 독립): 0=L, 1=L+R, 2=R, 3=M(mute)
    // JOIN에서 M이면 cmd_toggle_recv(ch, false) 전송
    std::array<int,MAX_CHANNELS> local_ch_out = []{ std::array<int,MAX_CHANNELS> a; a.fill(1); return a; }(); // 기본: L+R
    bool ch_created_by_me[MAX_CHANNELS] = {}; // JOIN: 내가 생성한 채널 여부
    bool ch_pending_create[MAX_CHANNELS] = {}; // JOIN: CMD_CREATE_CH 송신 후 HOST 확인 전 (stale sync 무시용)
    // JOIN: 서버에서 수신한 전체 audio_mask (리스너 표시용)
    uint32_t srv_audio_mask[MAX_CHANNELS] = {};
    // JOIN: 오디오 녹음 시작 전 뮤트 상태 저장 (녹음 후 복원용)
    bool join_rec_was_muted[MAX_CHANNELS] = {};
    // AUTO DF (각 창 로컬 — L/L+R/R/M 과 같은 성격, wire 로 동기화하지 않는다):
    // 켠 채널은 스컬치 게이트가 닫힘→열림으로 바뀔 때 DF 를 1회 자동 요청한다.
    bool  auto_df_on[MAX_CHANNELS] = {};
    bool  auto_df_gate_prev[MAX_CHANNELS] = {};   // edge 검출용 직전 게이트 상태
    float auto_df_last_t[MAX_CHANNELS] = {};      // 마지막 요청 시각 (ImGui::GetTime)
    static constexpr float AUTO_DF_COOLDOWN_S = 5.0f;   // 열린 채널 재측정 주기

    // 파일 전송 진행상태 (HOST: 전송 중, JOIN: 수신 중)
    struct FileXfer {
        std::string filename;
        uint64_t    total_bytes = 0;
        uint64_t    done_bytes  = 0;
        bool        finished    = false;
        bool        is_sa       = false;   // SA로 열 수 있는 파일
        std::string local_path;
        enum Dir : uint8_t { DIR_UNKNOWN=0, DIR_DOWNLOAD=1, DIR_UPLOAD=2 } dir = DIR_UNKNOWN;
        // 속도 측정 (EWMA bytes/sec) — render에서 갱신
        int64_t     last_done_bytes = 0;
        int64_t     last_steady_us  = 0;   // steady_clock microseconds
        double      bps_ewma        = 0.0;
    };
    std::vector<FileXfer> file_xfers;
    std::mutex            file_xfer_mtx;

    // 파일 리스트 한 줄 정보 포맷 — "HH:MM:SS ###.#M" / "HH:MM:SS ###.#G".
    // M/G 모두 5글자 폭으로 통일되어 컬럼 정렬됨.
    // Archive / HIST / DB Archive 공통 사용.
    static inline std::string format_file_info(double sec, uint64_t bytes){
        char tbuf[16] = "";
        if(sec > 0){
            uint64_t s = (uint64_t)sec;
            snprintf(tbuf, sizeof(tbuf), "%02llu:%02llu:%02llu",
                (unsigned long long)(s/3600),
                (unsigned long long)((s/60)%60),
                (unsigned long long)(s%60));
        }
        char sbuf[16];
        // 통일 형식: "%.1f MB" (IQ/DEMOD 동일).
        if(bytes >= (1ULL<<30))      snprintf(sbuf, sizeof(sbuf), "%.1f GB", bytes/(1024.0*1024.0*1024.0));
        else if(bytes >= (1ULL<<20)) snprintf(sbuf, sizeof(sbuf), "%.1f MB", bytes/(1024.0*1024.0));
        else if(bytes >= (1ULL<<10)) snprintf(sbuf, sizeof(sbuf), "%.1f KB", bytes/1024.0);
        else                          snprintf(sbuf, sizeof(sbuf), "%llu B", (unsigned long long)bytes);
        char buf[40];
        if(tbuf[0]) snprintf(buf, sizeof(buf), "%s %s", tbuf, sbuf);
        else        snprintf(buf, sizeof(buf), "%s", sbuf);
        return buf;
    }

    // 단일 전송 항목 렌더링 (Archive / HIST 공통 사용).
    // x의 EWMA 속도 필드(last_*, bps_ewma)는 호출마다 갱신됨.
    static inline void render_file_xfer_row(FileXfer& x){
    #ifndef BEWE_HEADLESS
        const char* dir_label =
            (x.dir == FileXfer::DIR_UPLOAD)   ? "Upload" :
            (x.dir == FileXfer::DIR_DOWNLOAD) ? "Download" :
                                                "Transfer";
        ImVec4 col = (x.dir == FileXfer::DIR_UPLOAD)
            ? ImVec4(1.0f, 0.85f, 0.4f, 1.f)
            : ImVec4(0.5f, 0.95f, 1.0f, 1.f);
        ImGui::PushStyleColor(ImGuiCol_Text, col);
        ImGui::Text("%s", dir_label);
        ImGui::PopStyleColor();
        float frac = (x.total_bytes > 0)
            ? (float)((double)x.done_bytes / (double)x.total_bytes) : 0.f;
        if(frac < 0.f) frac = 0.f;
        if(frac > 1.f) frac = 1.f;
        auto fmt_sz = [](uint64_t b, char* o, size_t sz){
            if(b < 1024)                    snprintf(o,sz,"%llu B",(unsigned long long)b);
            else if(b < 1024*1024)          snprintf(o,sz,"%.1f KB",(double)b/1024);
            else if(b < 1024ULL*1024*1024)  snprintf(o,sz,"%.1f MB",(double)b/(1024*1024));
            else                            snprintf(o,sz,"%.2f GB",(double)b/(1024ULL*1024*1024));
        };
        char dn[32], tn[32];
        fmt_sz(x.done_bytes, dn, sizeof(dn));
        fmt_sz(x.total_bytes, tn, sizeof(tn));
        int64_t now_us = (int64_t)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if(x.last_steady_us == 0){
            x.last_steady_us = now_us; x.last_done_bytes = (int64_t)x.done_bytes;
        } else if(now_us - x.last_steady_us >= 200000){
            int64_t db = (int64_t)x.done_bytes - x.last_done_bytes;
            double  dt = (now_us - x.last_steady_us) / 1e6;
            if(dt > 0){
                double inst_bps = (db > 0) ? (double)db / dt : 0.0;
                x.bps_ewma = (x.bps_ewma == 0.0) ? inst_bps
                                                 : x.bps_ewma * 0.7 + inst_bps * 0.3;
            }
            x.last_steady_us  = now_us;
            x.last_done_bytes = (int64_t)x.done_bytes;
        }
        char sn[24] = "";
        if(x.bps_ewma > 0.0){
            if(x.bps_ewma < 1024)                  snprintf(sn,sizeof(sn)," (%.0f B/s)",  x.bps_ewma);
            else if(x.bps_ewma < 1024*1024)        snprintf(sn,sizeof(sn)," (%.1f KB/s)", x.bps_ewma/1024);
            else if(x.bps_ewma < 1024.0*1024*1024) snprintf(sn,sizeof(sn)," (%.2f MB/s)", x.bps_ewma/(1024*1024));
            else                                   snprintf(sn,sizeof(sn)," (%.2f GB/s)", x.bps_ewma/(1024.0*1024*1024));
        }
        char buf[128];
        snprintf(buf,sizeof(buf),"%s / %s%s  %.0f%%", dn, tn, sn, frac*100.f);
        ImGui::ProgressBar(frac, ImVec2(-1, 0), buf);
        ImGui::TextDisabled("  %s", x.filename.c_str());
    #else
        (void)x;
    #endif
    }

    // ── 브로드캐스트 전용 스레드 (캡처 스레드와 분리) ──────────────────
    std::atomic<int>        net_bcast_seq{0};   // 캡처 스레드가 올림
    // 실측 FFT 행레이트 (net_bcast_worker 가 1초 창으로 갱신). Central 이 FFT 를
    // .bewehist 로 아카이브하므로 이 값이 파일 헤더의 row_rate_hz 가 된다 — 상수 5Hz 를
    // 쓰면 뷰어 시간축이 어긋난다. 0 = 아직 미측정 (LIVE_START 시 5Hz 폴백).
    std::atomic<float>      fft_row_rate_hz{0.f};
    std::mutex              net_bcast_mtx;
    std::condition_variable net_bcast_cv;
    std::atomic<bool>       net_bcast_stop{false};
    std::thread             net_bcast_thr;

    void net_bcast_worker();  // 선언

    // ── Hardware (공통) ───────────────────────────────────────────────────
    HWConfig hw;                          // 런타임 HW 파라미터
#ifdef BEWE_HOST_BUILD
    struct bladerf*  dev_blade = nullptr; // BladeRF 디바이스
    rtlsdr_dev_t*    dev_rtl   = nullptr; // RTL-SDR 디바이스
    // ADALM-Pluto (libiio) — void* 로 선언해 header include 오염 방지
    void*            pluto_ctx       = nullptr; // iio_context*
    void*            pluto_phy_dev   = nullptr; // iio_device* (ad9361-phy)
    void*            pluto_rx_dev    = nullptr; // iio_device* (cf-ad9361-lpc)
    void*            pluto_rx_i_ch   = nullptr; // iio_channel* voltage0
    void*            pluto_rx_q_ch   = nullptr; // iio_channel* voltage1
    void*            pluto_rx_buf    = nullptr; // iio_buffer*
#endif
    fftwf_plan      fft_plan=nullptr;
    fftwf_complex  *fft_in=nullptr, *fft_out=nullptr;
    bool  is_running=true;
    std::atomic<bool> rx_stopped{false};  // /rx stop: SDR 의도적 중단 (자동 재연결 방지)
    std::atomic<bool> sdr_hw_present{false};  // rx_stopped 중 저빈도 presence-only 스캔 결과 (자동 시작 안 함, 표시용)
    int   total_ffts=0;
    std::string window_title;
    std::mutex  data_mtx;
    // 주파수 변경 요청 — 비-캡처 스레드(UI/네트워크/스케줄)가 세우고 캡처 스레드가 처리한다.
    // plain bool/float 이면 캡처 hot loop 가 레지스터에 들고 있어 요청을 영영 못 볼 수 있다.
    // freq_prog 는 캡처 스레드 전용 (진행중 표시).
    std::atomic<float> pending_cf{0.f};
    std::atomic<bool>  freq_req{false};
    bool  freq_prog=false;
    bool  sc8_mode=false; // SC8_Q7 모드 (122.88 MSPS)
    std::atomic<uint64_t> live_cf_hz{0};  // 스레드 안전 현재 중심주파수 (Hz)

    // ── IQ Ring ───────────────────────────────────────────────────────────
    std::vector<int16_t> ring;
    std::atomic<size_t>  ring_wp{0};

    // ── Channels ──────────────────────────────────────────────────────────
    Channel channels[MAX_CHANNELS];
    int     selected_ch=-1;

    struct NewDrag{ bool active=false; float anch=0,s=0,e=0; } new_drag;

    // ── Record 탭 표시용 항목 ─────────────────────────────────────────────
    struct RecEntry {
        std::string path;       // 전체 경로
        std::string filename;   // 표시용 파일명
        bool        finished = false;
        bool        is_audio = false; // false=IQ, true=복조오디오
        bool        is_region = false; // true=선택영역 IQ
        // 요청 상태 (JOIN 측 region IQ 요청 및 HOST 측 표시용)
        enum ReqState { REQ_NONE=0, REQ_PENDING, REQ_CONFIRMED, REQ_DENIED, REQ_TRANSFERRING } req_state = REQ_NONE;
        float       req_deny_timer = 5.f; // DENY 후 자동 제거 카운트다운
        uint64_t    xfer_done = 0, xfer_total = 0; // 전송 진행
        uint8_t     req_op_idx = 0;  // HOST: 요청한 op_idx
        char        req_op_name[32] = {}; // HOST: 요청한 op 이름
        int32_t     req_fft_top = 0, req_fft_bot = 0;
        float       req_freq_lo = 0, req_freq_hi = 0;
        int32_t     req_time_start = 0, req_time_end = 0; // Unix timestamps from JOIN
        uint32_t    req_sr = 0;       // JOIN: region IQ SR (IQ_CHUNK START 동봉, .sigmf-meta용)
        std::string local_path_to_delete; // HOST: 전송 후 삭제할 파일 경로
        std::chrono::steady_clock::time_point t_start; // 시작 시각
        std::chrono::steady_clock::time_point t_last_tick; // 이전 프레임 시각 (Holding 중 정지용)
        float       total_elapsed = 0.f; // Holding 정지 반영된 경과 시간(초)
        int         ch_idx = -1;  // 오디오 녹음 채널 인덱스
    };
    std::vector<RecEntry> rec_entries;
    std::mutex            rec_entries_mtx;

    // 광대역 모듈(WiFi 등)이 full-rate IQ ring 공급을 요청 — rec/dem 없이 ring 채움.
    // (복조 없는 채널필터에 모듈 활성 시 host_start 가 raise, host_stop/on_ch_stop 가 lower)
    std::atomic<bool>     mod_wants_ring{false};

    // ── IQ Recording ──────────────────────────────────────────────────────
    std::atomic<bool>     rec_on{false}, rec_stop{false};
    std::thread           rec_thr;
    std::atomic<size_t>   rec_rp{0};
    float                 rec_cf_mhz=0;
    uint32_t              rec_sr=0;
    int                   rec_ch=-1;
    std::string           rec_filename;
    std::atomic<uint64_t> rec_frames{0};
    std::chrono::steady_clock::time_point rec_t0;

    // ── Audio mix ─────────────────────────────────────────────────────────
    std::atomic<bool> mix_stop{false};
    std::thread       mix_thr;

    // ── EID Audio 탭 재생 ─────────────────────────────────────────────────
    // EID 패널 Audio 탭에서 좌클릭 cursor → 스페이스로 재생/일시정지
    std::unique_ptr<AudioPlayback> audio_player;
    int64_t eid_audio_cursor_sample = 0;  // 좌클릭으로 설정된 cursor (샘플 인덱스)
    void  audio_play_start(const std::string& path, double offset_sec);
    void  audio_play_stop();
    void  audio_play_pause();
    void  audio_play_resume();
    bool  audio_play_active() const;
    bool  audio_play_paused() const;
    float audio_play_pos_sec() const;
    // ── IQ 파일 Audio 탭: AM/FM 복조 후 재생 ──────────────────────────────
    // IQ(stereo) 녹음을 Audio 탭에서 AM/FM 복조해 임시 mono WAV 로 듣기.
    bool        eid_is_iq = false;       // 현재 로드된 EID 파일이 IQ(stereo)인가
    int         eid_audio_demod = 1;     // 0=AM 1=FM (Audio 탭 상단 버튼)
    float       eid_bpf_center_uv = 0.5f;// 활성 BPF 대역 중심 UV (0.5=DC) — 복조 전 재중심용
    uint64_t    eid_edit_gen = 0;        // eid_ch_i/q 수정(BPF/remove/undo) 세대 — 복조 캐시 무효화용
    std::string eid_iq_tmp_path;         // 복조 결과 임시 wav
    std::string eid_iq_tmp_src;          // 그 임시 wav 가 어느 소스/모드로 만들어졌는지
    int         eid_iq_tmp_mode = -1;
    uint64_t    eid_iq_tmp_gen = (uint64_t)-1;  // 그 임시 wav 가 만들어진 시점의 edit_gen
    std::string eid_iq_demod_tempwav(int am_fm);  // eid_ch_i/q → AM/FM mono wav, path 반환
    void        eid_audio_play(double off_sec);    // IQ면 복조 wav, 아니면 원본 재생

    // ── hw_detect / bladerf_io / rtlsdr_io ───────────────────────────────
    bool initialize(float cf_mhz, float sr_msps = 0.f);  // sr_msps=0 → HW별 기본 (BladeRF 61.44, Pluto 3.2)
    bool initialize_bladerf(float cf_mhz, float sr_msps);
    bool initialize_rtlsdr(float cf_mhz);
    bool initialize_pluto(float cf_mhz, float sr_msps);
    bool initialize_kraken(float cf_mhz);   // heimdall DAQ (TCP :5000) — kraken_io.cpp
    void pluto_release();            // iio buffer/context 해제 (idempotent)
    float pluto_get_temp_c() const;  // AD9361 내부 온도 °C (실패 시 음수)
    void capture_and_process();
    void capture_and_process_rtl();
    void capture_and_process_pluto();
    void capture_and_process_kraken();

    // ── DF (KrakenSDR 방탐) ──────────────────────────────────────────────
    // 구현은 kraken_io.cpp. 이 헤더가 df/ 를 include 하지 않도록 plain 스칼라만
    // 주고받는다 (df:: 타입이 여기 새면 거의 모든 .cpp 가 df/ 에 재컴파일 의존).
    bool df_engine_ready() const;
    int  df_link_state()   const;   // 0=down  1=calibrating  2=streaming
    bool df_measuring()    const;
    bool df_submit(double center_hz, double bw_hz, int arr_idx, int dnum,
                   bool use_backlog = false);
    void df_stop_engine();
    // 설정 접근 (설정 패널·HostState 용). algo 0=Bartlett 1=Capon 2=MUSIC,
    // sense 0=CW 1=CCW.
    // DF 설정 전체. HOST 소유이고 JOIN 은 HOST 가 방송한 정본을 본다.
    // 개별 인자로 넘기던 걸 구조체로 바꿨다 — 항목이 늘면서 호출부마다
    // 인자를 빼먹는 사고가 나기 쉬웠다.
    //   df_get_cfg : HOST/LOCAL 은 자기 값, JOIN 은 HOST 가 방송한 값
    //   df_set_cfg : HOST/LOCAL 은 즉시 적용 + 방송, JOIN 은 HOST 로 요청만
    void df_get_cfg(PktDfConfig& out) const;
    void df_set_cfg(const PktDfConfig& c);
    // KrakenSDR 튜너 게인 (RTL 이산 스텝 인덱스). 5채널 동시 적용이고, DAQ 가
    // 받으면 재캘리브레이션을 돌려 수 초간 DF 가 멈춘다. HOST(CLI) 전용 —
    // JOIN 은 df_set_cfg 의 gain_idx 로 HOST 에 요청만 한다.
    bool df_set_gain_index(int idx, char* err, size_t errn);
    // 설정이 바뀌었을 때 HOST 가 정본을 뿌린다 (JOIN 접속 시에도).
    void df_broadcast_cfg() const;

    // ── 매니폴드 캘리브레이션 ────────────────────────────────────────────
    // df_cal_get : 현재 요약 (HOST 는 엔진에서, JOIN 은 HOST 방송본에서)
    // df_cal_cmd : capture/clear/remove. HOST 는 즉시 실행 + 방송, JOIN 은 요청만
    void df_cal_get(PktDfCalib& out) const;
    void df_cal_cmd(const PktDfCalib& c);
    void df_broadcast_cal() const;
    // 캘리브 세트 영속화. host_state 와 수명이 달라(이륙 전 1회 vs 상시) 파일을
    // 따로 쓴다. 경로는 호출측(cli_host)이 스테이션 이름으로 만들어 여기 박아두고,
    // 이후 캘리브가 바뀔 때마다 df_cal_cmd 가 알아서 저장한다 — 운용자가 저장을
    // 따로 눌러야 한다면 언젠가 잊는다.
    bool df_cal_save(const char* path) const;
    bool df_cal_load(const char* path);
    std::string df_cal_path;
    double df_snr_threshold() const;   // 편의 접근 (수락 규칙 표시용)
    // 설정 패널 실시간 판독. 문자열 하나 + 스칼라 몇 개로 끝낸다.
    struct DFLive {
        int      link = 0;            // 0=down 1=calibrating 2=streaming
        bool     usable = false;
        unsigned sync_state = 0, delay_sync = 0, iq_sync = 0, noise_src = 0;
        unsigned channels = 0, overdrive = 0;
        double   daq_cf_mhz = 0, daq_fs_msps = 0;
        double   frame_rate_hz = 0, recv_mbps = 0;
        unsigned long long frames_ok = 0, frames_cal = 0, frames_bad = 0, gaps = 0, reconnects = 0;
        unsigned gain_tenths[8] = {};
        char     hw_id[20] = {};
        char     last_error[96] = {};
        double   lambda_m = 0, ambiguity = 0;   // 현재 DAQ 중심주파수 기준
        bool     measuring = false;
        float    progress = 0.f;
    };
    void df_get_live(DFLive& out) const;
    std::atomic<uint32_t> df_seq{0};

    // 표시번호(freq_sorted_display_num) -> 배열 인덱스 -> 측정 요청.
    // 키·패널 버튼·/df·CLI 가 전부 이 한 경로를 쓴다. 거절 사유는 항상 한 줄로
    // 채팅/로그에 나간다 — 조용히 실패하면 운용자가 눌렀는지도 모른다.
    // from_auto = AUTO DF 자동 발사. HOST 까지 따라가 결과의 origin 이 되고,
    // 채팅 방송 여부를 가른다 — 자동분까지 방송하면 채널 10개 x 10초 쿨다운에서
    // 최악 분당 60줄이 모든 기지의 대화 로그를 덮는다.
    bool df_request_by_display_num(int dnum, bool from_auto = false);
    void df_post_refusal(int dnum, const char* msg);
    // pending_df_result 를 사람이 읽는 한 줄로. 접두사는 붙이지 않는다 —
    // 채팅은 발신자 이름("DF")이, 로그는 호출부가 "[DF] " 를 붙인다.
    // 양쪽이 같은 문자열을 쓰도록 여기 한 곳에서만 만든다.
    //   detailed=false : 채팅용. 실패 사유는 세 문구로만 압축한다.
    //   detailed=true  : 로그용. 압축 문구 뒤에 원인을 괄호로 덧붙인다.
    void df_format_line(char* out, size_t n, bool detailed = false) const;
    static const char* df_short_reason(const char* detail);
    // 엔진 결과를 pending_df_result 로 옮긴다. 메인 루프가 매 프레임 부른다.
    void df_pump();

    // DF 측정 결과 — 엔진 스레드가 채우고 메인 루프가 exchange(false) 로 소비한다.
    // pending_file_ctx 와 동일 관용구. 측정이 직렬화되므로 단일 슬롯으로 충분하다.
    struct PendingDFResult {
        std::atomic<bool> pending{false};
        int   dnum = 0, arr_idx = -1;
        float cf_mhz = 0, bw_khz = 0;
        float bearing = 0, bearing_rel = 0, conf = 0, snr = 0, pwr = 0;
        int   frames_used = 0, frames_discarded = 0;
        bool  ok = false, overdrive = false;
        char  err[80] = {};
        // ── DF_RESULT 와이어 패킷을 만드는 데 필요한 나머지 ──────────────
        // 예전엔 df_pump 가 df::Result 를 여기로 좁히면서 스펙트럼·모호집합·
        // 진단값을 전부 버렸다. HOST 프로세스 밖으로 나갈 길이 없었기 때문인데,
        // 이제 나간다.
        float algo_papr = 0, eff_bw_khz = 0, n_eff = 0, ambiguity = 0, diag_spread_db = 0;
        float alt_deg[2] = {}, alt_db[2] = {};
        int   alt_n = 0, elements = 0, algo = 0;
        bool  imbalance = false;
        bool  from_auto = false;   // AUTO DF 발사분 — 채팅으로 방송하지 않는다
        int64_t t_end_ms = 0;
        int   dur_ms = 0;
        uint8_t spec_q[360] = {};  // dB_below_peak = -0.5 * q
        bool  has_spec = false;
    } pending_df_result;

    // ── 방위 이력 ─────────────────────────────────────────────────────────
    // AUTO DF 가 들어온 뒤로 측정이 잦다. 한 칸짜리 "마지막 결과" 로는 방위가
    // 안정적인지 표류하는지 표적이 움직이는지 전혀 안 보인다. 각 행이 자기
    // 스펙트럼을 들고 있어야 옛 fix 를 클릭했을 때 그때의 극좌표를 볼 수 있다
    // (스펙트럼을 최신 것만 두면 표의 절반이 죽는다).
    // 64 x ~460 B = 29 KB 고정, 할당 없음. UI 스레드 전용.
    struct DFFix {
        // 단조 증가 고유 id. t_end_ms 를 키로 쓰면 안 된다 — 측정 전 거절
        // (df_post_refusal) 은 t_end_ms 가 0 이라 전부 같은 키가 되고, 하나를
        // 클릭하면 거절 행 전체가 선택된다.
        uint32_t seq = 0;
        int64_t t_end_ms = 0;
        float   bearing_deg = 0, bearing_rel_deg = 0;
        float   snr_db = 0, conf_db = 0, power_dbfs = 0;
        float   cf_mhz = 0, bw_khz = 0, ambiguity = 0;
        float   station_lat = 0, station_lon = 0;   // lon: 동경 양수
        float   alt_deg[2] = {}, alt_db[2] = {};
        uint8_t dnum = 0, elements = 0, algo = 0, overdrive_mask = 0;
        uint8_t kind = 0, frames_used = 0, frames_discarded = 0, alt_n = 0;
        uint8_t origin = 0, imbalance = 0, has_spec = 0, manual_lob = 0;
        char    note[64] = {};
        uint8_t spec_q[360] = {};
    };
    // 이력 링. 64 는 AUTO DF 가 5초 주기로 도는 지금 너무 짧다 — 채널 둘만
    // 켜도 3분이면 한 바퀴가 돌아 앞의 측정이 사라진다 (운용자 눈에는 "LOB 이
    // 저절로 지워지는" 현상으로 보인다). DFFix 가 512 B 라 512 칸이 262 KB 고,
    // FFTViewer 는 3.25 MB 짜리 스택 객체이며 스택 상한이 8 MB 라 여유가 있다.
    static constexpr int DF_HIST_MAX = 512;
    DFFix df_hist[DF_HIST_MAX];
    int   df_hist_n = 0;          // 채워진 개수 (<= DF_HIST_MAX)
    int   df_hist_head = 0;       // 다음에 쓸 위치
    // 총 push 횟수. df_hist_n 은 64 에서 포화하므로 "새 결과가 왔나" 를 그걸로
    // 판정하면 링이 찬 뒤로 영영 안 바뀐다 (거절 배너가 죽는다).
    uint32_t df_hist_seq = 0;
    // 링에서 i 번째로 오래된 항목 (0 = 가장 오래됨)
    const DFFix& df_hist_at(int i) const {
        const int base = (df_hist_n < DF_HIST_MAX) ? 0 : df_hist_head;
        return df_hist[(base + i) % DF_HIST_MAX];
    }
    // HOST/JOIN 양 arm 이 같은 함수로 결과를 받아들인다 (fft_viewer.cpp).
    // 두 곳에서 각자 필드를 옮기면 반드시 갈라진다.
    void df_apply_result(const PktDfResult& r, const uint8_t* spec_q);
    void df_push_fix(const DFFix& f);
    // seq 집합에 든 항목을 이력에서 지운다 (반환 = 지운 개수).
    // 링 중간을 비우면 df_hist_at 의 순서 계산이 깨지므로, 남은 것을 앞으로
    // 모아 head/n 을 다시 잡는다. 링 인덱스가 아니라 seq 로 받는 이유는 UI 의
    // 선택이 seq 문자열 키라서다 (인덱스는 새 결과가 들어오면 밀린다).
    int  df_hist_erase(const std::set<uint32_t>& seqs);

    // 설정 패널이 그리는 "마지막 결과". UI 스레드만 읽고 쓴다.
    bool  df_last_valid = false;
    int   df_last_dnum = 0;
    float df_last_cf_mhz = 0, df_last_bw_khz = 0;
    float df_last_bearing = 0, df_last_bearing_rel = 0;
    float df_last_conf = 0, df_last_snr = 0, df_last_pwr = 0;
    float df_last_spectrum[360] = {};
    // 진행 중인(또는 방금 끝난) 측정이 AUTO DF 발사분인지. 엔진은 한 번에 하나만
    // 재고 그동안 다른 요청은 "already running" 으로 거절되므로 한 칸이면 충분하다.
    // 요청 스레드와 드레인 스레드가 같다 (UI/메인).
    bool df_req_auto = false;

    // JOIN: 마지막으로 소비한 DF_RESULT 세대. net_cli->df_res_seq 와 다르면
    // 새 결과가 온 것이다 (한 칸 슬롯이라 항상 최신).
    uint32_t df_res_seen = 0;
    // 캡처 스레드에 LO 변경을 위임한다. wait=true 면 캡처 스레드가 실제로 적용할 때까지
    // 블록한다 (스케줄 녹화처럼 "바뀐 주파수로" 곧바로 녹화를 시작하는 호출자용).
    void set_frequency(float cf_mhz, bool wait=false);
    void set_gain(float db);
    float gain_db = 0.0f;

    // ── 채널 스컬치 (UI 스레드, FFT 기반) ──────────────────────────────────
    int  sq_last_total_ffts = -1;   // new-row guard: 같은 FFT 행 재스캔 방지
    // SDR (재)시작/autoscale 로 노이즈플로어가 바뀌면 자동 캘리브 채널을 다시 잡는다.
    // capture 스레드가 세우고 update_channel_squelch()(UI 스레드)가 소비 —
    // 채널 상태를 capture 스레드에서 직접 만지지 않기 위한 요청 플래그.
    std::atomic<bool> sq_recalib_req{false};
    void update_channel_squelch();

    // ── 에너지 디텍션 (Detect 모드) ────────────────────────────────────────
    // 복조 없는 채널을 detect 모드로 전환/해제 (HOST 로컬 적용).
    void set_channel_detect(int ch_idx, bool on);

    // ── demod.cpp ─────────────────────────────────────────────────────────
    void dem_worker(int ch_idx);
    void start_dem(int ch_idx, Channel::DemodMode mode);
    void stop_dem(int ch_idx, bool stop_decoders=true);  // stop_decoders=false: 재튜닝(디코더 보존)

    void stop_all_dem();
    void update_dem_by_freq(float new_cf_mhz); // 주파수 변경 시 복조 pause/resume

    // ── iq_record.cpp ─────────────────────────────────────────────────────
    void rec_worker();
    void start_rec();
    void stop_rec();
    void stop_audio_rec(int ch_idx);
    void start_iq_rec(int ch_idx);
    void stop_iq_rec(int ch_idx);
    void iq_only_worker(int ch_idx);  // demod 우회 IQ-only 녹음 worker

    void start_join_audio_rec(int ch_idx); // JOIN 모드 로컬 오디오 녹음
    void stop_join_audio_rec(int ch_idx);

    // ── audio.cpp ─────────────────────────────────────────────────────────
    void mix_worker();

    // ── fft_viewer.cpp (waterfall + display helpers) ──────────────────────
    void create_waterfall_texture();
    void update_wf_row(int fi);
    // 캡처 공용 dB 행 커밋 (4개 SDR 백엔드 공통 — 정의는 bladerf_io.cpp, CLI 전용)
    void commit_fft_row(const std::vector<float>& pacc, int fcnt);
    void get_disp(float& ds, float& de) const;
    float x_to_abs(float x, float gx, float gw) const;
    float abs_to_x(float abs_mhz, float gx, float gw) const;
    int   channel_at_x(float mx, float gx, float gw) const;
    int   freq_sorted_display_num(int arr_idx) const;

#ifndef BEWE_HEADLESS
    // 전체영역 오버레이(SA/LOG/HIST/LIB/MSN/DEMOD) 가 하나라도 열려 있는가.
    // 열려 있으면 메인페이지 입력(마우스/키보드)은 전부 격리 차단한다 — 렌더와
    // 내부 상태는 그대로 유지해서, 오버레이를 닫으면 즉시 정상 동작.
    //
    // MSN 도 여기 포함된다: 한때 78%/80% floating 창이라 제외했었는데, 그러면
    // "창 밖" 클릭이 뒤 UI 로 새고 클릭 순간 뒤 창이 한 프레임 위로 튀는 문제가
    // 계속 남았다. MSN 을 전체화면으로 바꿔 '밖'을 없앤 뒤 원래 규칙으로 복귀.
    bool overlay_blocking() const {
        return eid_panel_open || log_panel_open || lwf_modal_open
            || sig_lib_panel_open || mission_modal_open || demod_panel_open
            || df_panel_open;
    }

    // 마우스가 메인창(##main) 이 아닌 별도 ImGui 창(MSN 모달·서브모달·팝업 등)
    // 위에 있는가. 메인페이지의 "직접 좌표 검사" 입력 경로들은 ImGui 의 창 우선순위를
    // 모르므로, 이 검사로 창 위 클릭이 뒤로 새는 걸 막는다.
    // (ImGui 위젯 기반 경로는 IsItemHovered() 가 이미 처리해준다.)
    static bool float_win_capturing_mouse();

    // 메인페이지 마우스 입력을 지금 받아도 되는가의 반대 — 입력 경로는 이걸 쓴다.
    // 전체영역 오버레이가 떠 있거나, 마우스가 얹힌 창(채팅·팝업 등) 위에 있으면 차단.
    bool main_mouse_blocked() const {
        return overlay_blocking() || float_win_capturing_mouse();
    }

    // ── ui.cpp (rendering — GUI only) ────────────────────────────────────
    void handle_new_channel_drag(float gx, float gw);
    void handle_channel_interactions(float gx, float gw, float gy, float gh);
    void draw_all_channels(ImDrawList* dl, float gx, float gw, float gy, float gh, bool show_label);
    void draw_freq_axis(ImDrawList* dl, float gx, float gw, float gy, float gh, bool ticks_only=false);
    void handle_zoom_scroll(float gx, float gw, float mouse_x);
    void draw_spectrum_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h);
#endif

    // ── 주파수 축 드래그 (center frequency 이동) ────────────────────────
    bool   freq_drag_active = false;
    float  freq_drag_start_x = 0;     // 드래그 시작 시 마우스 x
    float  freq_drag_start_cf = 0;    // 드래그 시작 시 center_frequency (MHz)
#ifndef BEWE_HEADLESS
    void draw_waterfall_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h);
#endif
};

// ── Entry point ───────────────────────────────────────────────────────────
#ifdef BEWE_HEADLESS
void run_cli_host();
#else
void run_streaming_viewer();
#endif

// 캡처 스레드 수명 표시 RAII. 생성 시 cap_exited=false, 소멸(모든 return/예외
// 경로) 시 true. /powercycle 이 join 을 포기할지 판단하는 데 쓴다.
struct CapLifeGuard {
    std::atomic<bool>* f;
    explicit CapLifeGuard(std::atomic<bool>* p) : f(p) { f->store(false); }
    ~CapLifeGuard(){ f->store(true); }
};

// ── 캡처 스레드 기동 (백엔드 디스패치) ────────────────────────────────────
// 이 if/else 사슬은 예전에 10곳에 복붙돼 있었고 마지막 else 가 전부 RTL 이었다.
// 백엔드를 하나 추가하면 전부 RTL 루프로 떨어져 null 핸들을 친다. 실제로
// ui.cpp 의 /rx start 경로는 BLADERF/else 만 분기해서 Pluto 에서 RTL 루프가
// 떴다. 한 곳으로 모아 그런 사고가 구조적으로 안 나게 한다.
inline void bewe_spawn_capture(FFTViewer& v, std::thread& cap){
    switch(v.hw.type){
        case HWType::BLADERF: cap = std::thread(&FFTViewer::capture_and_process,        &v); return;
        case HWType::PLUTO:   cap = std::thread(&FFTViewer::capture_and_process_pluto,  &v); return;
        case HWType::RTLSDR:  cap = std::thread(&FFTViewer::capture_and_process_rtl,    &v); return;
        case HWType::KRAKEN:  cap = std::thread(&FFTViewer::capture_and_process_kraken, &v); return;
        case HWType::NONE:
        default:
            // 하드웨어가 없으면 아예 띄우지 않는다. 예전엔 여기서도 RTL 루프를
            // 띄웠고 dev_rtl 이 null 이라 즉시 스트림 에러로 자멸했다.
            bewe_log_push(2,"[SDR] no backend selected - capture thread not started\n");
            return;
    }
}

// ── USB 소프트 리셋 (sudo 불필요, udev rule 권한 사용) ────────────────────
// USBDEVFS_RESET ioctl: 물리적으로 뽑았다 꽂는 것과 동일한 효과
bool usb_reset_vidpid(uint16_t vid, uint16_t pid, const char* label);

// SDR 종류 → USB VID/PID. /chassis 1 reset · /powercycle · stall watchdog 이
// 공유한다 (한 곳에서 빠지면 그 SDR 만 복구가 안 되는 사고가 난다).
inline bool sdr_usb_ids(HWType t, uint16_t* vid, uint16_t* pid, const char** label){
    switch(t){
        case HWType::BLADERF: *vid=0x2cf0; *pid=0x5250; *label="BladeRF";     return true;
        case HWType::RTLSDR:  *vid=0x0bda; *pid=0x2838; *label="RTL-SDR";     return true;
        case HWType::PLUTO:   *vid=0x0456; *pid=0xb673; *label="ADALM-Pluto"; return true;
        // KRAKEN: 이 모드에서 BEWE 는 USB 장치를 하나도 소유하지 않는다 —
        // heimdall 의 rtl_daq.out 가 동글 5개를 잡고 있다. false 를 반환하면
        // 모든 USB 리셋 경로가 무장해제된다 (전부 이 반환값이나 if(vid) 로 가드됨).
        // 절대 여기에 0bda:2838 을 넣지 말 것 — 동글 5개가 전부 매칭되고
        // 리셋 코드는 "첫 번째"를 잡는다 = 돌고 있는 DAQ 를 임의로 파괴한다.
        case HWType::KRAKEN:  return false;
        default: return false;
    }
}

// ── BEWE 가 실제로 연 RTL 동글의 EEPROM 시리얼 ────────────────────────────
// 0bda:2838 은 KrakenSDR 처럼 동일 동글이 여러 개 꽂힌 환경에서 유일하지 않다.
// "첫 번째 매칭"으로 리셋하면 남의 동글(= 돌고 있는 heimdall DAQ)을 파괴한다.
// initialize_rtlsdr 이 open 성공 직후 채우고, close/실패 시 비운다.
// 비어 있으면 리셋 함수들이 후보 개수를 세서 1개일 때만 진행한다.
void        rtl_set_owned_serial(const char* s);   // nullptr/"" => 소유 없음
const char* rtl_owned_serial();                    // 항상 유효한 C 문자열

// ── USB 딥 파워사이클 (/powercycle) ───────────────────────────────────────
// USBDEVFS_RESET 은 장치 핸들을 거치므로 펌웨어 링크가 죽으면(BladeRF NIOS II
// timeout 등) 리셋 요청 자체가 장치에 닿지 않는다 — /chassis 1 reset 이 안 먹는
// 상황이 이것. 여기서는 커널 쪽에서 끊는다: sysfs authorized 0 > 1 로 드라이버를
// unbind 했다가 재열거한다 (물리적 재삽입과 동일).
//   반환 0=성공, 1=장치 못 찾음, 2=권한 없음(udev rule 미배포), 3=write 실패
// 권한: /sys/bus/usb/devices/<X-Y>/authorized 는 기본 root 전용이라 udev rule 로
// plugdev 쓰기를 열어야 한다 (assets/udev/99-bewe-usb-powercycle.rules).
int usb_deep_powercycle(uint16_t vid, uint16_t pid, int off_ms);

// ── DEMOD 모듈 패널 렌더 (demod_panel.cpp) ────────────────────────────────
void demod_draw_panel(FFTViewer& v, bool just_opened);

// ── DF 설정 오버레이 렌더 (df_view.cpp) ───────────────────────────────────
void df_draw_panel(FFTViewer& v, bool just_opened);

// PktDfConfig 의 array_type/elements/radius_m/sense 로 elem_x/elem_y 를 다시 만든다.
// array_type 이 Custom 이면 좌표를 건드리지 않는다. GUI(JOIN)도 HOST 도 같은 규칙을
//써야 하므로 df/df_config.hpp 가 아니라 여기 둔다 — GUI 는 df/ 를 링크하지 않는다.
// df::make_geom 과 같은 식이며, 어느 한쪽을 고치면 다른 쪽도 고쳐야 한다
// (df_selftest 가 두 경로의 좌표 일치를 확인한다).
void df_preset_coords(PktDfConfig& p);