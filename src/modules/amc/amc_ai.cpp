// ── AMC 추론 IPC: BEAE/amc 의 Python 데몬과 UDS 로 주고받는다 ────────────────
//
// 왜 C++ 이 직접 추론하지 않나: 모델이 PyTorch 체크포인트다. 프로세스를 분리하면
// 재학습·모델교체가 BEWE 재컴파일 없이 되고, torch 가 죽어도 기지는 안 죽는다.
//
// src/modules/ais/ais_ai.cpp 의 구조를 그대로 따른다 (검증된 경로). 다른 점은
// 요청/응답 페이로드뿐 — MMSI 대신 클래스 인덱스 2개를 받는다.
#ifdef BEWE_MODULE_AMC_AI

#include "amc_ai.hpp"
#include "amc_meta.hpp"
#include "bewe_paths.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <poll.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

namespace amc_mod {

static std::string venv_python(){ return BEWEPaths::data_dir()+"/BEAE/amc/.venv/bin/python"; }
static std::string amc_pkg_dir(){ return BEWEPaths::data_dir()+"/BEAE/amc"; }
static std::string sock_path()  { return BEWEPaths::data_dir()+"/BEAE/amc/data/ai.sock"; }

bool amc_ai_enabled(){
    // 게이트는 venv 존재 여부. env BEWE_AMC_AI 로 강제 on/off (1/0).
    // venv 가 없는 개발 PC 에서는 완전 비활성 — 다른 기지에 영향 zero.
    static int c=-1;
    if(c<0){
        const char* e=getenv("BEWE_AMC_AI");
        if(e && e[0]=='0')      c=0;
        else if(e && e[0]=='1') c=1;
        else                    c = (access(venv_python().c_str(), X_OK)==0) ? 1 : 0;
    }
    return c==1;
}

// ── 와이어 (BEAE/amc/amc_ai/proto.py 와 거울) ───────────────────────────────
static constexpr uint32_t AMRQ_MAGIC = 0x51524D41;  // 'AMRQ' (LE)
static constexpr uint32_t AMRP_MAGIC = 0x50524D41;  // 'AMRP'

struct __attribute__((packed)) AmcReqHdr {   // len 프리픽스 뒤 28B + f32 IQ[2n]
    uint32_t magic; uint16_t ver; uint16_t type;
    uint32_t bw_hz; int64_t t_ms; uint32_t out_sr;
    uint16_t n; uint8_t ch; uint8_t trig;
};
static_assert(sizeof(AmcReqHdr)==28, "AmcReqHdr must be 28 bytes");

struct __attribute__((packed)) AmcResp {     // 총 22B (len 포함)
    // conf 는 0..1 확률 → permille(0..1000) 로 싣는다. u16 범위 안이지만 Python 쪽에서
    // 반드시 클램프할 것 — AIS 는 백분율*1000 이 u16 을 넘겨 struct.pack 이 죽고
    // 데몬 select 루프 전체가 내려앉은 적이 있다 (2026-07-07).
    uint32_t len; uint32_t magic; uint16_t ver; uint16_t type;
    uint8_t status; uint8_t cls; uint8_t cls2; uint8_t ncls;
    uint16_t conf_permille; uint16_t conf2_permille; uint16_t model_ver;
    // 전 클래스 확률. ncls = 데몬이 실제로 채운 개수 — 모델이 클래스를 늘려도
    // 앞쪽만 읽으면 되므로 C++ 을 다시 컴파일하지 않아도 된다.
    uint16_t p_permille[AMC_NCLASS];
};
static_assert(sizeof(AmcResp)==22+2*AMC_NCLASS, "AmcResp size changed - update proto.py NCLASS");

static constexpr int     AI_IO_BUDGET_MS = 60;    // connect+send+recv 총 예산
static constexpr int64_t AI_RETRY_MS     = 5000;  // 실패 후 재시도 금지 래치

static std::mutex g_sock_mtx;
static int        g_fd = -1;
static int64_t    g_retry_at_ms = 0;

static int64_t mono_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

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
        if(r==0) return false;
        if(errno==EAGAIN||errno==EWOULDBLOCK){ if(!io_wait(fd,POLLIN,deadline)) return false; continue; }
        if(errno==EINTR) continue;
        return false;
    }
    return true;
}
static void sock_drop(){ if(g_fd>=0){ close(g_fd); g_fd=-1; } g_retry_at_ms = mono_ms()+AI_RETRY_MS; }

static bool sock_alive(){
    int fd=socket(AF_UNIX, SOCK_STREAM|SOCK_CLOEXEC, 0);
    if(fd<0) return false;
    struct sockaddr_un a{}; a.sun_family=AF_UNIX;
    strncpy(a.sun_path,sock_path().c_str(),sizeof(a.sun_path)-1);
    bool ok = connect(fd,(struct sockaddr*)&a,sizeof(a))==0;
    close(fd); return ok;
}

static pid_t g_daemon_pid = -1;

void amc_ai_stop_daemon(){
    if(g_daemon_pid>0){
        kill(g_daemon_pid, SIGTERM);
        waitpid(g_daemon_pid, nullptr, 0);
        g_daemon_pid=-1;
    }
}

void amc_ai_ensure_daemon(){
    if(!amc_ai_enabled()) return;
    if(g_daemon_pid>0){
        if(waitpid(g_daemon_pid, nullptr, WNOHANG)==0) return;   // 살아있음
        g_daemon_pid=-1;
    }
    if(sock_alive()) return;                  // 외부(systemd) 데몬이 이미 응답
    std::string py=venv_python(), cwd=amc_pkg_dir();
    if(access(py.c_str(), X_OK)!=0) return;
    static bool s_atexit=false;
    if(!s_atexit){ atexit(amc_ai_stop_daemon); s_atexit=true; }
    pid_t pid=fork();
    if(pid<0) return;
    if(pid==0){
        setsid();
        if(chdir(cwd.c_str())!=0) _exit(127);
        int nul=open("/dev/null",O_RDONLY); if(nul>=0){ dup2(nul,0); if(nul>0) close(nul); }
        execl(py.c_str(), py.c_str(), "-m", "amc_ai", "daemon", (char*)nullptr);
        _exit(127);
    }
    g_daemon_pid=pid;
    g_retry_at_ms=mono_ms()+AI_RETRY_MS;      // 소켓 뜰 시간 확보
}

static bool sock_connect(int64_t deadline){
    std::string path = sock_path();
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

// ── 실측 IQ 덤프 (기본 꺼짐, BEWE_AMC_CAP=1 로 켠다) ────────────────────────
// 합성으로만 학습한 모델이 실제 전파에서 무너질 때, 추측 대신 그 입력을 그대로
// 들여다보기 위한 것이다 (BEAE/amc/amc_ai/realcap.py 가 읽는다). 파일 포맷은
// 요청 헤더와 같은 28B + f32 IQ[2n] — 소켓으로 보낸 것과 바이트 동일하다.
static std::mutex g_cap_mtx;
static void cap_append(const AmcReqHdr& h, const float* iq, int n){
    const char* e = getenv("BEWE_AMC_CAP");
    if(!e || e[0] != '1') return;
    std::lock_guard<std::mutex> lk(g_cap_mtx);
    std::string dir = BEWEPaths::data_dir()+"/BEAE/amc/data";
    mkdir((BEWEPaths::data_dir()+"/BEAE").c_str(),0755);
    mkdir((BEWEPaths::data_dir()+"/BEAE/amc").c_str(),0755);
    mkdir(dir.c_str(),0755);
    time_t t=(time_t)(h.t_ms/1000); struct tm lt{}; localtime_r(&t,&lt);
    char d[16]; strftime(d,sizeof(d),"%Y%m%d",&lt);
    FILE* f=fopen((dir+"/amccap_"+d+".bin").c_str(),"ab"); if(!f) return;
    fwrite(&h,sizeof(h),1,f); fwrite(iq,sizeof(float),(size_t)n*2,f);
    fclose(f);
}

bool amc_ai_infer(int ch, int64_t t_ms, uint32_t out_sr, uint32_t bw_hz, uint8_t trig,
                  const float* iq, int n_complex,
                  int& cls, float& conf, int& cls2, float& conf2, char* model, size_t model_cap,
                  float* p_out, int np){
    if(!amc_ai_enabled() || !iq || n_complex<=0) return false;
    if(mono_ms() < g_retry_at_ms) return false;
    // try_lock: 다른 채널이 트랜잭션 중이면 이 버스트는 포기한다. 기다리면 워커가
    // 밀려 ring lag 이 쌓이고, 그게 쌓이면 경계점프로 신호를 통째로 놓친다.
    if(!g_sock_mtx.try_lock()) return false;
    std::lock_guard<std::mutex> lk(g_sock_mtx, std::adopt_lock);

    int64_t deadline = mono_ms()+AI_IO_BUDGET_MS;
    if(g_fd<0 && !sock_connect(deadline)){ g_retry_at_ms=mono_ms()+AI_RETRY_MS; return false; }

    // 프레이밍(proto.py 거울): u32 len | AmcReqHdr(28B) | f32 IQ[2n]. len = 28+8n.
    uint32_t len = (uint32_t)sizeof(AmcReqHdr) + (uint32_t)n_complex*8;
    AmcReqHdr h{}; h.magic=AMRQ_MAGIC; h.ver=1; h.type=1;
    h.bw_hz=bw_hz; h.t_ms=t_ms; h.out_sr=out_sr;
    h.n=(uint16_t)n_complex; h.ch=(uint8_t)ch; h.trig=trig;
    cap_append(h, iq, n_complex);             // 켜져 있을 때만 (BEWE_AMC_CAP=1)
    if(!send_all(g_fd,&len,4,deadline) || !send_all(g_fd,&h,sizeof(h),deadline)
       || !send_all(g_fd,iq,(size_t)n_complex*8,deadline)){ sock_drop(); return false; }

    AmcResp r{};
    if(!recv_all(g_fd,&r,sizeof(r),deadline) || r.magic!=AMRP_MAGIC
       || r.len!=(uint32_t)(sizeof(AmcResp)-4)){ sock_drop(); return false; }
    if(r.status!=1) return false;             // 0=NO_MODEL, 3=ERROR → 판정 없음
    if(r.cls==0xFF) return false;             // 데몬이 판정불가로 회신

    cls   = (int)r.cls;
    conf  = (float)r.conf_permille  / 1000.f;
    cls2  = (r.cls2==0xFF) ? -1 : (int)r.cls2;
    conf2 = (float)r.conf2_permille / 1000.f;
    if(model && model_cap) snprintf(model, model_cap, "BEAEv%u", (unsigned)r.model_ver);
    if(p_out && np>0){
        int k = (int)r.ncls; if(k>np) k=np; if(k>AMC_NCLASS) k=AMC_NCLASS;
        for(int i=0;i<np;i++) p_out[i] = (i<k) ? r.p_permille[i]/1000.f : 0.f;
    }
    return true;
}

} // namespace amc_mod

#endif // BEWE_MODULE_AMC_AI
