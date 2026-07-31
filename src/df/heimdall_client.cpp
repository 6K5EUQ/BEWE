#include "heimdall_client.hpp"

#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace df {

using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

namespace {

void set_err(char* err, size_t n, const char* fmt, ...) __attribute__((format(printf,3,4)));
void set_err(char* err, size_t n, const char* fmt, ...){
    if(!err || !n) return;
    va_list ap; va_start(ap, fmt);
    vsnprintf(err, n, fmt, ap);
    va_end(ap);
}

// 서버가 쓰는 정확한 문자열. 종단 NUL 도 길이 접두도 없다.
constexpr char kHello[]    = "streaming";   // 9 바이트
constexpr char kDownload[] = "IQDownload";  // 10 바이트
constexpr char kQuit[]     = "q";

// heimdall 이 프레임 하나 만드는 데 437 ms 걸린다. 그보다 넉넉히 잡되,
// 두 번째 클라이언트가 백로그에 걸려 조용히 행 하는 걸 반드시 깨야 한다.
constexpr int kRecvTimeoutMs = 8000;

// 접속 직후 버릴 수 있는 묵은 프레임의 상한. 실측상 2개면 충분한데,
// 무한 루프만 막으면 되므로 여유 있게 둔다. 42MB/프레임이라 크게 잡을 값은
// 아니다 — 이걸 다 쓰면 스트림이 이상한 것이고, 그때는 그냥 진행한다.
constexpr int kMaxStaleDrain = 6;

bool set_timeouts(int fd, int ms){
    timeval tv{ ms/1000, (ms%1000)*1000 };
    if(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv) < 0) return false;
    if(setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv) < 0) return false;
    return true;
}

} // namespace

bool HeimdallClient::send_all(const void* src, size_t n){
    const uint8_t* p = (const uint8_t*)src;
    size_t sent = 0;
    while(sent < n){
        ssize_t k = ::send(fd_, p + sent, n - sent, MSG_NOSIGNAL);
        if(k > 0){ sent += (size_t)k; continue; }
        if(k < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

bool HeimdallClient::recv_exact(void* dst, size_t n, char* err, size_t errn){
    uint8_t* p = (uint8_t*)dst;
    size_t got = 0;
    while(got < n){
        ssize_t k = ::recv(fd_, p + got, n - got, 0);
        if(k > 0){ got += (size_t)k; continue; }
        if(k == 0){
            set_err(err, errn, "peer closed after %zu/%zu bytes", got, n);
            return false;
        }
        if(errno == EINTR) continue;
        if(errno == EAGAIN || errno == EWOULDBLOCK){
            // 여기 걸리면 십중팔구 다른 클라이언트가 이미 스트림을 물고 있다.
            // 서버는 두 번째 접속을 거부하지 않고 백로그에 방치한다.
            set_err(err, errn, "recv timeout after %zu/%zu bytes "
                               "(another client may hold the stream)", got, n);
            return false;
        }
        set_err(err, errn, "recv: %s", strerror(errno));
        return false;
    }
    bytes_ += n;
    return true;
}

bool HeimdallClient::connect(const char* host, uint16_t port, int timeout_ms,
                             char* err, size_t errn){
    disconnect();

    char portstr[8];
    snprintf(portstr, sizeof portstr, "%u", (unsigned)port);
    addrinfo hints{}; hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    int gai = getaddrinfo(host, portstr, &hints, &res);
    if(gai != 0 || !res){
        set_err(err, errn, "getaddrinfo %s:%u: %s", host, (unsigned)port, gai_strerror(gai));
        return false;
    }

    int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(fd < 0){
        freeaddrinfo(res);
        set_err(err, errn, "socket: %s", strerror(errno));
        return false;
    }
    if(!set_timeouts(fd, timeout_ms > 0 ? timeout_ms : kRecvTimeoutMs)){
        ::close(fd); freeaddrinfo(res);
        set_err(err, errn, "setsockopt: %s", strerror(errno));
        return false;
    }
    if(::connect(fd, res->ai_addr, res->ai_addrlen) < 0){
        ::close(fd); freeaddrinfo(res);
        set_err(err, errn, "connect %s:%u: %s", host, (unsigned)port, strerror(errno));
        return false;
    }
    freeaddrinfo(res);

    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    // 96 MB/s 를 루프백으로 받는다. 커널 버퍼가 작으면 iq_server 의 단일
    // send() 가 잘게 쪼개져 왕복이 늘어난다.
    int rcvbuf = 16 << 20;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);

    fd_ = fd;
    // 이후 수신은 프레임 주기(437ms)를 견뎌야 하므로 별도 타임아웃.
    set_timeouts(fd_, kRecvTimeoutMs);

    if(!send_all(kHello, sizeof(kHello) - 1)){
        set_err(err, errn, "handshake send failed: %s", strerror(errno));
        disconnect();
        return false;
    }

    primed_ = false;
    have_prev_cpi_ = false;
    gaps_ = frames_ = bytes_ = 0;

    // 서버가 무요청으로 밀어주는 첫 프레임을 소비하고, 이어서 남아 있는
    // 묵은 프레임을 신선한 게 나올 때까지 버린다. 버퍼 깊이(A/B)를 상수로
    // 박지 않는 이유는 그게 heimdall 내부 구현이기 때문이다 — 헤더의
    // time_stamp 로 판정하면 깊이가 바뀌어도 그대로 맞는다.
    Frame f{};
    for(int i = 0; i < kMaxStaleDrain; i++){
        if(!next_frame(f, err, errn)){ disconnect(); return false; }
        if(i == 0){
            if(!sync_ok(hdr_)){
                set_err(err, errn, "bad sync word 0x%08X (not a heimdall stream?)", hdr_.sync_word);
                disconnect(); return false;
            }
            if(hdr_.header_version != kHeaderVersion){
                set_err(err, errn, "header_version %u, expected %u",
                        hdr_.header_version, kHeaderVersion);
                disconnect(); return false;
            }
            primed_ = true;   // 이후로는 매 프레임 IQDownload 를 보내야 한다
        }
        if(!f.stale) break;
    }
    // 버린 프레임은 통계에 넣지 않는다.
    have_prev_cpi_ = false;
    frames_ = 0;
    bytes_ = 0;
    gaps_  = 0;
    return true;
}

void HeimdallClient::disconnect(){
    if(fd_ < 0) return;
    // "IQDownload" 가 아닌 건 뭐든 세션을 끝낸다. 실패해도 무시하고 닫는다 —
    // 소켓을 살려두면 daq_start_sm.sh 의 포트 게이트가 영원히 안 끝난다.
    send_all(kQuit, 1);
    ::shutdown(fd_, SHUT_RDWR);
    ::close(fd_);
    fd_ = -1;
    have_prev_cpi_ = false;
    primed_ = false;
}

bool HeimdallClient::next_frame(Frame& out, char* err, size_t errn){
    out = Frame{};
    if(fd_ < 0){ set_err(err, errn, "not connected"); return false; }

    // 첫 프레임만 무요청 푸시. 나머지는 매번 요청해야 한다.
    if(primed_ && !send_all(kDownload, sizeof(kDownload) - 1)){
        set_err(err, errn, "IQDownload send failed: %s", strerror(errno));
        return false;
    }

    if(!recv_exact(&hdr_, sizeof(hdr_), err, errn)) return false;
    if(!sync_ok(hdr_)){
        set_err(err, errn, "lost frame sync (sync_word 0x%08X)", hdr_.sync_word);
        return false;
    }

    const uint64_t need = payload_bytes(hdr_);
    if(need > 0){
        // 첫 프레임 크기로 한 번만 잡고 그대로 재사용한다. CAL 프레임은
        // cpi_length 가 corr_size(65536) 라 더 작으므로 여기 안 걸린다.
        if(buf_.size() < need) buf_.resize((size_t)need);
        if(!recv_exact(buf_.data(), (size_t)need, err, errn)) return false;
        out.iq = reinterpret_cast<const std::complex<float>*>(buf_.data());
    }

    // ── 신선도 ───────────────────────────────────────────────────────────
    // iq_server 는 클라이언트가 없어도 계속 돌고, delay_sync 는 drop_mode 라
    // 버퍼 A/B 를 채운 뒤 나머지를 버린다. 그래서 접속하면 그 두 개(최대)가
    // 먼저 쏟아진다 — 실측: cpi 가 63 점프, 27초 묵은 데이터.
    // 버퍼 깊이를 상수로 박는 대신 헤더 time_stamp(unix ms)로 판정한다.
    int64_t period_ms = 450;
    if(hdr_.sampling_freq > 0 && hdr_.cpi_length > 0)
        period_ms = (int64_t)(1000.0 * hdr_.cpi_length / (double)hdr_.sampling_freq);
    const int64_t now_ms = (int64_t)duration_cast<milliseconds>(
                               system_clock::now().time_since_epoch()).count();
    out.age_ms = (hdr_.time_stamp > 0) ? (now_ms - (int64_t)hdr_.time_stamp) : 0;
    // 3 프레임 주기 + 여유. 정상 프레임은 수십 ms 안쪽이라 넉넉하다.
    const int64_t stale_ms = period_ms * 3 + 500;
    out.stale = (hdr_.time_stamp > 0) && (out.age_ms > stale_ms);

    // cpi 갭 판정은 "중간에 CAL/DUMMY/stale 이 없는 연속 DATA" 에서만 유효하다.
    // CAL 은 cpi_length 가 달라 cpi_index 증가 속도가 다르기 때문에, 그대로
    // 비교하면 캘리브레이션 버스트마다 가짜 드롭이 잡힌다 (실측 +62).
    if(hdr_.frame_type == FRAME_DATA && !out.stale){
        if(have_prev_cpi_ && hdr_.cpi_index != prev_cpi_ + 1u) gaps_++;
        prev_cpi_ = hdr_.cpi_index;
        have_prev_cpi_ = true;
    } else {
        have_prev_cpi_ = false;
    }

    out.hdr            = &hdr_;
    out.channels       = hdr_.active_ant_chs;
    out.samples_per_ch = hdr_.cpi_length;
    frames_++;
    return true;
}

// ── 제어 소켓 ─────────────────────────────────────────────────────────────

bool heimdall_ctrl_send(const char* host, uint16_t port, const char cmd[4],
                        const void* payload, size_t payload_len,
                        int timeout_ms, char* err, size_t errn){
    if(payload_len > 124){ set_err(err, errn, "control payload too large (%zu > 124)", payload_len); return false; }

    char portstr[8];
    snprintf(portstr, sizeof portstr, "%u", (unsigned)port);
    addrinfo hints{}; hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    if(getaddrinfo(host, portstr, &hints, &res) != 0 || !res){
        set_err(err, errn, "getaddrinfo %s:%u failed", host, (unsigned)port);
        return false;
    }
    int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(fd < 0){ freeaddrinfo(res); set_err(err, errn, "socket: %s", strerror(errno)); return false; }
    set_timeouts(fd, timeout_ms > 0 ? timeout_ms : 15000);
    if(::connect(fd, res->ai_addr, res->ai_addrlen) < 0){
        ::close(fd); freeaddrinfo(res);
        set_err(err, errn, "connect %s:%u: %s (is the Kraken web UI holding it?)",
                host, (unsigned)port, strerror(errno));
        return false;
    }
    freeaddrinfo(res);

    uint8_t msg[128] = {};
    memcpy(msg, cmd, 4);
    if(payload && payload_len) memcpy(msg + 4, payload, payload_len);

    bool ok = true;
    for(size_t sent = 0; sent < sizeof msg && ok; ){
        ssize_t k = ::send(fd, msg + sent, sizeof(msg) - sent, MSG_NOSIGNAL);
        if(k > 0) sent += (size_t)k;
        else if(!(k < 0 && errno == EINTR)) ok = false;
    }
    if(!ok){ ::close(fd); set_err(err, errn, "control send failed: %s", strerror(errno)); return false; }

    // 회신은 항상 "FNSD" + 124 * 0. DAQ 가 요청을 집을 때까지 블로킹된다.
    uint8_t reply[128] = {};
    for(size_t got = 0; got < sizeof reply; ){
        ssize_t k = ::recv(fd, reply + got, sizeof(reply) - got, 0);
        if(k > 0){ got += (size_t)k; continue; }
        if(k < 0 && errno == EINTR) continue;
        ::close(fd);
        set_err(err, errn, (k == 0) ? "control peer closed early" : "control recv: %s",
                strerror(errno));
        return false;
    }
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);

    if(memcmp(reply, "FNSD", 4) != 0){
        set_err(err, errn, "unexpected control reply '%.4s'", (const char*)reply);
        return false;
    }
    return true;
}

bool heimdall_set_freq(const char* host, uint16_t port, uint64_t hz,
                       int timeout_ms, char* err, size_t errn){
    return heimdall_ctrl_send(host, port, "FREQ", &hz, sizeof hz, timeout_ms, err, errn);
}

bool heimdall_set_gain_tenths(const char* host, uint16_t port,
                              const uint32_t* per_ch, int n,
                              int timeout_ms, char* err, size_t errn){
    if(n <= 0 || n > 31){ set_err(err, errn, "bad channel count %d", n); return false; }
    return heimdall_ctrl_send(host, port, "GAIN", per_ch, (size_t)n * sizeof(uint32_t),
                              timeout_ms, err, errn);
}

} // namespace df
