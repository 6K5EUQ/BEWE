#pragma once
// ── DF 엔진 ───────────────────────────────────────────────────────────────
// UI 계층이 include 하는 유일한 DF 헤더.
//
// 스레드 1개가 heimdall 스트림을 계속 소비한다. 계속 받는 이유는 두 가지다:
//  - ch0 이 BEWE 의 스펙트럼/워터폴/ring 을 먹여야 한다
//  - 안 받으면 delay_sync 가 drop_mode 로 프레임을 버리고, 다시 붙었을 때
//    묵은 데이터부터 나온다
//
// 측정은 단발이다. submit() 으로 무장하면 그 다음 "쓸 수 있는" 프레임부터
// N 개를 모아 R 을 누산하고, 끝나면 결과를 한 칸짜리 슬롯에 실어 둔다.
// UI 는 poll() 로 가져간다 (FFTViewer::pending_file_ctx 와 같은 관용구).

#include "df_types.hpp"
#include "df_config.hpp"
#include "df_calib.hpp"
#include <atomic>
#include <complex>
#include <functional>
#include <memory>

namespace df {

class Engine {
public:
    Engine();
    ~Engine();
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // ── 수명 ─────────────────────────────────────────────────────────────
    bool start(const Config& cfg);   // 스레드 기동. 논블로킹 (접속은 스레드가 한다)
    void stop();                     // "q" 전송 + join
    bool running() const;

    // ── 측정 (단발) ──────────────────────────────────────────────────────
    // 이미 측정 중이면 false. UI 스레드에서 호출한다.
    bool  submit(const Request& r);
    // 완료된 결과를 최대 1건 꺼낸다. 없으면 false.
    bool  poll(Result& out);
    void  cancel();
    bool  armed() const;
    float progress() const;          // 0..1

    // ── 상태 ─────────────────────────────────────────────────────────────
    DaqStatus status() const;
    Config    config() const;
    void      apply_config(const Config& c);   // host/port 외 전부 즉시 반영

    // ── 매니폴드 캘리브레이션 ────────────────────────────────────────────
    // 직전 측정의 주 고유벡터를 "이 방위의 실측 조향벡터" 로 받아 보정을 만든다.
    // 측정을 새로 돌리지 않고 마지막 결과를 쓰므로, 운용자는 평소처럼 측정한 뒤
    // 방위를 입력하고 이 함수를 부르면 된다. 아직 측정이 없거나 그 측정이
    // 거부됐으면 false.
    bool cal_add(double bearing_deg, char* err, size_t errn);
    void cal_clear();
    void cal_remove(int idx);
    // UI 표시용 요약 (점 개수·주파수·잔차). 포인터로 내주면 수명이 얽히므로 복사.
    struct CalInfo {
        int    n = 0;
        double freq_hz = 0;
        int    elements = 0;
        double worst_dev_db = 0;
        bool   active = false;              // 지금 매니폴드에 실제로 걸려 있는가
        double bearing[kMaxCalPoints]  = {};   // 운용자가 신고한 참 방위
        double measured[kMaxCalPoints] = {};   // 그때 배열이 보고한 방위 (보정 전)
        double snr_db[kMaxCalPoints]   = {};
    };
    CalInfo cal_info() const;
    // 파일로 남기고 되읽는다. 경로는 호출측(스테이션별 파일)이 정한다.
    bool cal_save(const char* path) const;
    bool cal_load(const char* path);

    // ── 프레임 탭 (BEWE 스펙트럼용) ──────────────────────────────────────
    // DAQ 스레드에서 프레임마다 호출된다. 블로킹 금지 — 필요한 만큼 복사하고
    // 즉시 반환할 것. iq 는 콜백 동안만 유효하고 **읽기 전용**이다 (DF 측정이
    // 같은 버퍼를 그대로 쓴다 — 여기서 고치면 방탐이 깨진다).
    //
    // 전 채널을 넘긴다. 소비자가 ch0 만 쓰든 5채널을 합치든(MRC) 고를 수 있게
    // 하기 위해서다. channel(m) = iq + m*samples_per_ch (channel-major).
    struct FrameView {
        const std::complex<float>* iq = nullptr;
        uint32_t channels = 0;
        uint32_t samples_per_ch = 0;
        uint64_t center_hz = 0, fs_hz = 0;
        uint32_t overdrive_flags = 0;
        // heimdall 이 채널 간 지연·IQ 보정을 건 프레임인가. false 면 위상이
        // 아직 틀리므로 채널 간 통계에 넣으면 안 된다 (ch0 만 쓰는 건 무방).
        bool     usable = false;
        int64_t  wall_ms = 0;
        const std::complex<float>* channel(uint32_t m) const {
            return (iq && m < channels) ? iq + (size_t)m * samples_per_ch : nullptr;
        }
    };
    using FrameSink = std::function<void(const FrameView&)>;
    void set_frame_sink(FrameSink s);

    // ── DAQ 제어 (:5001) ─────────────────────────────────────────────────
    // 전부 heimdall 을 STATE_INIT 으로 되돌린다 = 수 초간 DF 불가.
    bool set_center_freq(uint64_t hz, char* err, size_t errn);
    bool set_gain_tenths(const uint32_t* per_ch, int n, char* err, size_t errn);

private:
    struct Impl;
    std::unique_ptr<Impl> d_;
};

} // namespace df
