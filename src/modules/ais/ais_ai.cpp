// ── AIS Match_AI (DL 지문) — 버스트 캡처 사이드카 + 추론 데몬 UDS 클라이언트 ──
// env BEWE_AIS_AI=1 일 때만 활성 (미설정 = 완전 비활성, 타 기지 영향 zero).
//  - aicap_append: 라벨링된 버스트 복소 베이스밴드를 BEAE/ais/data/aicap_YYYYMMDD.bin 에 append
//    (학습데이터; 레코드 = 24B 헤더 'AIC1' + f32 I/Q interleave)
//  - ai_query: BEAE/ais/data/ai.sock 의 Python 데몬에 버스트 전송 → 예측 MMSI/신뢰 수신.
//    지속 연결 + 전체 트랜잭션 단일 데드라인(60ms < 워커 MAX_LAG 80ms) + 실패 5초 래치.
// 워커 스레드(2채널)에서 host_emit 경유 호출 — g_cap_mtx 로 파일 직렬화,
// g_sock_mtx try_lock 으로 소켓 직렬화(경합 시 그 버스트만 포기, 스톨 중첩 방지).
#include "ais_module.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

namespace ais_mod {

// BEAE/ais/.venv/bin/python — 이 파일 존재 = AI 스택 설치된 기지 (DGS-N).
// 미설치(개발PC 등)면 경로 없음 → AI 완전 비활성 (타 기지 영향 zero).
static std::string venv_python(){ return BEWEPaths::data_dir()+"/BEAE/ais/.venv/bin/python"; }
static std::string ais_pkg_dir(){ return BEWEPaths::data_dir()+"/BEAE/ais"; }

bool ai_enabled(){
    // 게이트: venv 존재 여부. env BEWE_AIS_AI 는 강제 오버라이드(1=강제 on, 0=강제 off).
    static int c=-1;
    if(c<0){
        const char* e=getenv("BEWE_AIS_AI");
        if(e && e[0]=='0')      c=0;                       // 명시적 비활성
        else if(e && e[0]=='1') c=1;                       // 명시적 활성 (venv 없어도 시도)
        else                    c = (access(venv_python().c_str(), X_OK)==0) ? 1 : 0;  // 자동: venv 존재 시
    }
    return c==1;
}

// ── 파일/와이어 공용 24B 헤더 (Python aicap.py / proto.py 와 거울) ──────────
static constexpr uint32_t AICAP_MAGIC = 0x31434941;  // 'AIC1' (LE)
static constexpr uint32_t AIRQ_MAGIC  = 0x51524941;  // 'AIRQ'
static constexpr uint32_t AIRP_MAGIC  = 0x50524941;  // 'AIRP'
struct __attribute__((packed)) AiCapHdr {
    uint32_t magic; uint32_t mmsi; int64_t t_ms;
    uint32_t out_sr; uint16_t n; uint8_t ch; uint8_t flags;
};
static_assert(sizeof(AiCapHdr)==24, "AiCapHdr must be 24 bytes");
struct __attribute__((packed)) AiReqHdr {   // len 프리픽스 뒤 28B + f32 IQ[2n]
    uint32_t magic; uint16_t ver; uint16_t type;
    uint32_t mmsi; int64_t t_ms; uint32_t out_sr;
    uint16_t n; uint8_t ch; uint8_t flags;
};
static_assert(sizeof(AiReqHdr)==28, "AiReqHdr must be 28 bytes");
struct __attribute__((packed)) AiResp {     // 총 22B (len 포함); conf_decipct = 신뢰(%)*10 (소수점 1자리)
    // u16 필드라 최대 65535 — *1000(3자리) 였을 때 신뢰 65.535%↑ 예측마다 오버플로로
    // 데몬 struct.pack 이 죽어 서버 루프 전체 다운(2026-07-07). *10 이면 100.0%까지 안전.
    uint32_t len; uint32_t magic; uint16_t ver; uint16_t type;
    uint8_t status; uint8_t pad; uint16_t conf_decipct; uint16_t model_ver; uint32_t pred_mmsi;
};
static_assert(sizeof(AiResp)==22, "AiResp must be 22 bytes");

static constexpr int AI_IO_BUDGET_MS = 60;    // connect+send+recv 총 예산 (< MAX_LAG 80ms)
static constexpr int64_t AI_RETRY_MS = 5000;  // 실패 후 재시도 금지 래치

static std::mutex g_cap_mtx;    // aicap 파일 (12KB 레코드 > stdio 버퍼 → 인터리브 방지 필수)
static std::mutex g_sock_mtx;   // 단일 지속 UDS 연결
static int        g_fd = -1;
static int64_t    g_retry_at_ms = 0;

static int64_t mono_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ── 학습데이터 append (일 단위 바이너리; host_fpcap 패턴 + mutex) ────────────
static void aicap_append(const AisRecord& m, uint32_t out_sr, const float* iq, int n){
    std::lock_guard<std::mutex> lk(g_cap_mtx);
    mkdir((BEWEPaths::data_dir()+"/BEAE").c_str(),0755);
    mkdir((BEWEPaths::data_dir()+"/BEAE/ais").c_str(),0755);
    mkdir((BEWEPaths::data_dir()+"/BEAE/ais/data").c_str(),0755);
    struct tm tmv; KST::to_tm((time_t)(m.t_ms/1000),tmv);
    char d[16]; strftime(d,sizeof(d),"%Y%m%d",&tmv);
    FILE* f=fopen((BEWEPaths::data_dir()+"/BEAE/ais/data/aicap_"+d+".bin").c_str(),"ab");
    if(!f) return;
    AiCapHdr h{}; h.magic=AICAP_MAGIC; h.mmsi=m.mmsi; h.t_ms=m.t_ms;
    h.out_sr=out_sr; h.n=(uint16_t)n; h.ch=(uint8_t)m.ch; h.flags=0;
    fwrite(&h,sizeof(h),1,f); fwrite(iq,sizeof(float),(size_t)n*2,f);
    fclose(f);
}

// ── UDS 트랜잭션 헬퍼 (nonblocking + poll, 공용 데드라인) ────────────────────
static bool io_wait(int fd, short ev, int64_t deadline){
    int64_t left = deadline - mono_ms(); if(left<=0) return false;
    struct pollfd p{fd, ev, 0};
    return poll(&p,1,(int)left)==1 && (p.revents & ev);
}
static bool send_all(int fd, const void* buf, size_t n, int64_t deadline){
    const uint8_t* p=(const uint8_t*)buf; size_t off=0;
    while(off<n){
        ssize_t w=send(fd,p+off,n-off,MSG_NOSIGNAL);
        if(w>0){ off+=(size_t)w; continue; }
        if(w<0 && (errno==EAGAIN||errno==EWOULDBLOCK)){ if(!io_wait(fd,POLLOUT,deadline)) return false; continue; }
        if(w<0 && errno==EINTR) continue;
        return false;
    }
    return true;
}
static bool recv_all(int fd, void* buf, size_t n, int64_t deadline){
    uint8_t* p=(uint8_t*)buf; size_t off=0;
    while(off<n){
        ssize_t r=recv(fd,p+off,n-off,0);
        if(r>0){ off+=(size_t)r; continue; }
        if(r==0) return false;   // 데몬 연결 종료
        if(errno==EAGAIN||errno==EWOULDBLOCK){ if(!io_wait(fd,POLLIN,deadline)) return false; continue; }
        if(errno==EINTR) continue;
        return false;
    }
    return true;
}
static void sock_drop(){ if(g_fd>=0){ close(g_fd); g_fd=-1; } g_retry_at_ms = mono_ms()+AI_RETRY_MS; }

// ── 추론 데몬 spawn (BEWE 자식 프로세스) ────────────────────────────────────
// 정책: 소켓이 이미 응답하면(systemd 등 외부 데몬 존재) spawn 안 함 → 안전 공존.
//       자식이 살아있으면 재spawn 안 함. 크래시하면 다음 호출 때 재spawn.
// 반드시 단일스레드 시점(host_start 의 g_mgmt 락 내부)에서만 호출 —
// 워커 스레드에서 fork 하면 멀티스레드 fork 위험(락 상태 복제)에 걸림.
static pid_t g_daemon_pid = -1;

static bool sock_alive(){   // 논블록 connect 로 소켓 응답 여부만 확인 (외부 데몬 감지)
    int fd=socket(AF_UNIX, SOCK_STREAM|SOCK_CLOEXEC, 0);
    if(fd<0) return false;
    struct sockaddr_un a{}; a.sun_family=AF_UNIX;
    strncpy(a.sun_path,(BEWEPaths::data_dir()+"/BEAE/ais/data/ai.sock").c_str(),sizeof(a.sun_path)-1);
    bool ok = connect(fd,(struct sockaddr*)&a,sizeof(a))==0;
    close(fd); return ok;
}

void ai_ensure_daemon(){
    if(!ai_enabled()) return;
    if(g_daemon_pid>0){                       // 이미 spawn 한 자식 — 살아있나 확인
        if(waitpid(g_daemon_pid, nullptr, WNOHANG)==0) return;   // 살아있음
        g_daemon_pid=-1;                      // 죽었음 → 수확 완료, 재spawn 진행
    }
    if(sock_alive()) return;                  // 외부(systemd) 데몬이 이미 응답 → spawn 불필요
    std::string py=venv_python(), cwd=ais_pkg_dir();
    if(access(py.c_str(), X_OK)!=0) return;   // venv 없음 → no-op (강제 on 이어도 실행 불가)
    static bool s_atexit=false;               // BEWE 종료 시 자식 데몬 정리 (최초 spawn 시 1회 등록)
    if(!s_atexit){ atexit(ai_stop_daemon); s_atexit=true; }
    pid_t pid=fork();
    if(pid<0) return;
    if(pid==0){                               // 자식: cwd=BEAE/ais 로 -m ais_ai daemon
        setsid();                             // 세션 분리 (BEWE 종료 시그널 전파 격리)
        if(chdir(cwd.c_str())!=0) _exit(127);
        int nul=open("/dev/null",O_RDONLY); if(nul>=0){ dup2(nul,0); if(nul>0) close(nul); }
        execl(py.c_str(), py.c_str(), "-m", "ais_ai", "daemon", (char*)nullptr);
        _exit(127);
    }
    g_daemon_pid=pid;                         // 부모: pid 기억 (종료 시 kill, 좀비 수확용)
    g_retry_at_ms=mono_ms()+AI_RETRY_MS;      // 소켓 뜰 시간 확보 (즉시 질의 실패 방지)
}

void ai_stop_daemon(){                        // BEWE 종료 시 자식 데몬 정리 (자기 spawn 분만)
    if(g_daemon_pid>0){
        kill(g_daemon_pid, SIGTERM);
        waitpid(g_daemon_pid, nullptr, 0);
        g_daemon_pid=-1;
    }
}

static bool sock_connect(int64_t deadline){
    std::string path = BEWEPaths::data_dir()+"/BEAE/ais/data/ai.sock";
    int fd=socket(AF_UNIX, SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC, 0);
    if(fd<0) return false;
    struct sockaddr_un a{}; a.sun_family=AF_UNIX;
    strncpy(a.sun_path,path.c_str(),sizeof(a.sun_path)-1);
    if(connect(fd,(struct sockaddr*)&a,sizeof(a))<0){
        if(errno!=EINPROGRESS){ close(fd); return false; }
        if(!io_wait(fd,POLLOUT,deadline)){ close(fd); return false; }
        int err=0; socklen_t el=sizeof(err);
        if(getsockopt(fd,SOL_SOCKET,SO_ERROR,&err,&el)<0 || err){ close(fd); return false; }
    }
    g_fd=fd; return true;
}

static bool ai_query(const AisRecord& m, uint32_t out_sr, const float* iq, int n,
                     uint8_t& st, uint32_t& pm, uint16_t& cf){
    if(mono_ms() < g_retry_at_ms) return false;
    if(!g_sock_mtx.try_lock()) return false;   // 타 채널 트랜잭션 중 → 이 버스트 포기
    std::lock_guard<std::mutex> lk(g_sock_mtx, std::adopt_lock);
    int64_t deadline = mono_ms()+AI_IO_BUDGET_MS;
    if(g_fd<0 && !sock_connect(deadline)){ g_retry_at_ms=mono_ms()+AI_RETRY_MS; return false; }
    // 프레이밍 규약(proto.py 거울): u32 len | AiReqHdr(28B) | f32 IQ[2n]. len = 28+8n.
    uint32_t len = (uint32_t)sizeof(AiReqHdr) + (uint32_t)n*8;
    AiReqHdr h{}; h.magic=AIRQ_MAGIC; h.ver=1; h.type=1;
    h.mmsi=m.mmsi; h.t_ms=m.t_ms; h.out_sr=out_sr; h.n=(uint16_t)n; h.ch=(uint8_t)m.ch; h.flags=0;
    if(!send_all(g_fd,&len,4,deadline) || !send_all(g_fd,&h,sizeof(h),deadline)
       || !send_all(g_fd,iq,(size_t)n*8,deadline)){ sock_drop(); return false; }
    AiResp r{};
    if(!recv_all(g_fd,&r,sizeof(r),deadline) || r.magic!=AIRP_MAGIC || r.len!=18){ sock_drop(); return false; }
    st=r.status; pm=r.pred_mmsi; cf=r.conf_decipct;
    return true;
}

// ── host_emit 경유 진입점 ───────────────────────────────────────────────────
void host_ai(AisRecord& m, uint32_t out_sr, const float* iq, int n_complex){
    m.ai_status=0; m.ai_mmsi=0; m.ai_conf=0;
    if(!ai_enabled() || !iq || n_complex<=0 || m.mmsi==0) return;
    aicap_append(m, out_sr, iq, n_complex);
    uint8_t st=0; uint16_t cf=0; uint32_t pm=0;
    if(ai_query(m, out_sr, iq, n_complex, st, pm, cf)){
        if(st==2){ m.ai_status=2; m.ai_mmsi=pm; m.ai_conf=cf; }
        else if(st==1){ m.ai_status=1; m.ai_mmsi=pm; m.ai_conf=cf; }
        // st==0(NO_MODEL)/3(ERROR) → ai_status 0 유지 ("-")
    }
}

} // namespace ais_mod
