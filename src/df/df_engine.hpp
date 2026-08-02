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
        double bearing[kMaxCalPoints] = {};
        double snr_db[kMaxCalPoints]  = {};
    };
    CalInfo cal_info() const;

    // ── ch0 탭 (BEWE 스펙트럼용) ─────────────────────────────────────────
    // DAQ 스레드에서 프레임마다 호출된다. 블로킹 금지 — 필요한 만큼 복사하고
    // 즉시 반환할 것. 포인터는 콜백 동안만 유효하다.
    using Ch0Sink = std::function<void(const std::complex<float>* ch0, size_t n,
                                       uint64_t center_hz, uint64_t fs_hz,
                                       uint32_t overdrive_flags, int64_t wall_ms)>;
    void set_ch0_sink(Ch0Sink s);

    // ── DAQ 제어 (:5001) ─────────────────────────────────────────────────
    // 전부 heimdall 을 STATE_INIT 으로 되돌린다 = 수 초간 DF 불가.
    bool set_center_freq(uint64_t hz, char* err, size_t errn);
    bool set_gain_tenths(const uint32_t* per_ch, int n, char* err, size_t errn);

private:
    struct Impl;
    std::unique_ptr<Impl> d_;
};

} // namespace df
