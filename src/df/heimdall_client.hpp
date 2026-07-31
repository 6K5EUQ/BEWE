#pragma once
// ── heimdall DAQ TCP 클라이언트 ───────────────────────────────────────────
//
// 데이터: iq_server.out, 기본 :5000
//   connect → "streaming"(9B) 송신 → 서버가 프레임 1개를 무요청으로 푸시
//   → 이후 프레임마다 "IQDownload"(10B) 송신 후 수신
//   → "q" 로 정상 종료 ("IQDownload" 가 아닌 건 뭐든 세션을 끝낸다)
//   프레임 = 1024B 헤더 + cpi_length*M*8 바이트 (DUMMY 는 헤더만)
//
// 제어: hw_controller.py, 기본 :5001
//   고정 128바이트 [4 ASCII 명령][124 B 페이로드], 회신은 항상 "FNSD"+124*0
//
// 실측에서 나온 세 가지 (2026-07-31):
//  1) 무요청 푸시되는 첫 프레임은 stale 이다. iq_server 는 클라이언트가 없어도
//     계속 돌기 때문에 접속 시점에 버퍼에 남아 있던 걸 준다 (cpi 가 56 점프 =
//     유휴 24초). 그래서 여기서 조용히 버린다.
//  2) cpi_index 는 CAL 버스트를 가로질러 연속이 아니다 (CAL 은 cpi_length 가
//     corr_size=65536 이라 증가 속도가 다르다). 비-DATA 프레임을 만나면
//     갭 카운터를 리셋해야 5분마다 가짜 드롭이 잡히지 않는다.
//  3) 소켓은 오류 즉시 닫아야 한다. daq_start_sm.sh 의 포트 게이트는
//     `lsof -i:5000` 을 쓰는데 그건 *클라이언트* 소켓도 매칭한다. CLOSE_WAIT
//     로 남겨두면 DAQ 재기동이 영원히 안 끝난다 (타임아웃도 상한도 없다).
//
// 서버는 한 번에 클라이언트 1개만 받는다. 두 번째 접속자는 거부가 아니라
// 백로그에 걸려 조용히 행 하므로, 수신 타임아웃이 필수다.

#include "heimdall_header.hpp"
#include <complex>
#include <cstdint>
#include <cstddef>
#include <vector>

namespace df {

class HeimdallClient {
public:
    struct Frame {
        const IqHeader*            hdr = nullptr;
        const std::complex<float>* iq  = nullptr;  // channel-major, 페이로드 없으면 nullptr
        uint32_t                   channels = 0;
        uint32_t                   samples_per_ch = 0;
        // 접속 시점에 서버 버퍼(A/B)에 남아 있던 묵은 프레임. 실측상 2개까지
        // 나오고 27초 묵은 것도 있었다. DF 평균에 섞이면 안 된다.
        bool                       stale  = false;
        int64_t                    age_ms = 0;     // now - header.time_stamp
        const std::complex<float>* channel(uint32_t m) const {
            return (iq && m < channels) ? iq + (size_t)m * samples_per_ch : nullptr;
        }
    };

    HeimdallClient() = default;
    ~HeimdallClient(){ disconnect(); }
    HeimdallClient(const HeimdallClient&) = delete;
    HeimdallClient& operator=(const HeimdallClient&) = delete;

    // 접속 + 핸드셰이크 + stale 첫 프레임 폐기. 실패 시 소켓을 남기지 않는다.
    bool connect(const char* host, uint16_t port, int timeout_ms, char* err, size_t errn);
    void disconnect();                       // "q" 시도 후 즉시 close
    bool connected() const { return fd_ >= 0; }

    // 다음 프레임을 받는다. 반환된 Frame 의 포인터는 다음 호출까지만 유효하다.
    // false 면 스트림이 깨진 것 — 호출부는 disconnect 후 재접속해야 한다.
    bool next_frame(Frame& out, char* err, size_t errn);

    uint64_t gaps()        const { return gaps_; }
    uint64_t frames_read() const { return frames_; }
    uint64_t bytes_read()  const { return bytes_; }

private:
    bool recv_exact(void* dst, size_t n, char* err, size_t errn);
    bool send_all(const void* src, size_t n);

    int                   fd_ = -1;
    std::vector<uint8_t>  buf_;        // 페이로드. 첫 프레임에서 한 번만 잡는다
    IqHeader              hdr_{};
    bool                  primed_ = false;   // 첫(stale) 프레임을 소비했나
    uint32_t              prev_cpi_ = 0;
    bool                  have_prev_cpi_ = false;
    uint64_t              gaps_ = 0, frames_ = 0, bytes_ = 0;
};

// ── 제어 소켓 (:5001) ─────────────────────────────────────────────────────
// 매 호출이 접속→송신→회신→종료다. hw_controller 는 listen(1) 이라 붙은 채로
// 두면 다른 프로세스가 붙지 못한다. 어차피 자주 쓰는 경로가 아니다.
//
// 주의: FREQ/GAIN 은 heimdall 을 STATE_INIT 으로 되돌린다 = 수 초간 DF 불가.
// 그리고 이 호출은 DAQ 가 요청을 집을 때까지 블로킹된다.
bool heimdall_ctrl_send(const char* host, uint16_t port, const char cmd[4],
                        const void* payload, size_t payload_len,
                        int timeout_ms, char* err, size_t errn);

bool heimdall_set_freq(const char* host, uint16_t port, uint64_t hz,
                       int timeout_ms, char* err, size_t errn);
bool heimdall_set_gain_tenths(const char* host, uint16_t port,
                              const uint32_t* per_ch, int n,
                              int timeout_ms, char* err, size_t errn);

} // namespace df
