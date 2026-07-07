// ── STT 모듈 본체: 등록, 공유 whisper 워커 관리, 발화 전사 → framework 방출 ──
// AM/FM 채널필터에 "STT" decode 로 활성화. IQ 를 탭하지 않고 이미 복조된 mono
// 오디오(demod.cpp 의 maybe_stt_audio)를 squelch 발화 단위로 캡처만 한다 →
// 오디오 비소유(ext_audio 안 건드림) → 음성은 원래대로 AM/FM 재생.
//
// 여러 채널이 whisper 모델 1개(공유 워커 프로세스)를 공유:
//   STT 채널 ≥1 → 워커 fork(모델 로드) → 발화들이 채널ID 태그 달고 큐→stdin,
//   워커 stdout 자막 JSON → emit(Central→모든 join). 마지막 STT 채널 끄면 워커 종료.
#include "stt_meta.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "channel.hpp"
#include "bewe_paths.hpp"
#include <atomic>
#include <mutex>
#include <thread>
#include <deque>
#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <chrono>

namespace stt_mod {

std::mutex              mtx;
std::vector<SttRecord>  log;       // 뷰가 그리는 자막 로그 (최신 아래로 append)
char                    filter[64] = {};

// ── 공유 워커 프로세스 ──────────────────────────────────────────────────────
struct Worker {
    std::mutex   mgmt;                 // start/stop 직렬화
    pid_t        pid   = -1;
    int          in_fd = -1;           // → 워커 stdin (발화 프레임 write)
    int          out_fd= -1;           // ← 워커 stdout (자막 JSON read)
    std::thread  reader;
    std::atomic<bool> ready{false};    // 모델 로드+워밍업 완료 → State RUN
    std::atomic<bool> stop{false};
    std::atomic<int>  active_ch{0};    // STT 켠 채널 수 (0 되면 워커 종료)

    std::mutex   wq_mtx;               // 발화 큐 (write 스레드가 소비)
    std::deque<std::vector<uint8_t>> wq;   // 완성 프레임(헤더+오디오)들
    std::thread  writer;
};
static Worker W;
static FFTViewer* g_v = nullptr;       // reader 스레드가 emit 에 쓸 뷰

static constexpr uint32_t STT_MAGIC = 0x31545453;  // 'STT1' LE

// 워커 실행 커맨드: BEAE/stt/.venv/bin/python -m/파일. LD_LIBRARY_PATH(cublas/cudnn) 필요.
static std::string stt_dir(){ return BEWEPaths::data_dir()+"/BEAE/stt"; }

// ── 발화 완성 콜백 (demod 스레드에서 호출) → 프레임 만들어 큐에 push ─────────
static void on_utterance(int ch, uint32_t sr, const float* a, size_t n){
    if(!W.ready.load(std::memory_order_relaxed)) return;   // 모델 준비 전엔 버림
    if(n==0 || n>16000000) return;
    std::vector<uint8_t> frame(16 + n*4);
    uint32_t* h=(uint32_t*)frame.data();
    h[0]=STT_MAGIC; h[1]=(uint32_t)ch; h[2]=sr; h[3]=(uint32_t)n;
    memcpy(frame.data()+16, a, n*4);
    std::lock_guard<std::mutex> lk(W.wq_mtx);
    if(W.wq.size() < 64) W.wq.push_back(std::move(frame));   // 폭주 시 드롭
}

// ── writer 스레드: 큐 → 워커 stdin ─────────────────────────────────────────
static void writer_loop(){
    while(!W.stop.load(std::memory_order_relaxed)){
        std::vector<uint8_t> f;
        {
            std::lock_guard<std::mutex> lk(W.wq_mtx);
            if(!W.wq.empty()){ f=std::move(W.wq.front()); W.wq.pop_front(); }
        }
        if(f.empty()){ std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue; }
        size_t off=0;
        while(off<f.size() && !W.stop.load(std::memory_order_relaxed)){
            ssize_t w=write(W.in_fd, f.data()+off, f.size()-off);
            if(w<=0) break;
            off+=(size_t)w;
        }
    }
}

// ── JSON line 파서 (최소): "ready" / "text" 추출 ─────────────────────────────
static bool json_str(const char* l, const char* key, char* out, size_t cap){
    std::string pat=std::string("\"")+key+"\":\"";
    const char* p=strstr(l, pat.c_str()); if(!p) return false;
    p+=pat.size(); size_t o=0;
    while(*p && *p!='"' && o+1<cap){
        if(*p=='\\' && p[1]){ p++; if(*p=='n')out[o++]='\n'; else out[o++]=*p; p++; continue; }
        out[o++]=*p++;
    }
    out[o]=0; return true;
}
static long json_int(const char* l, const char* key){
    std::string pat=std::string("\"")+key+"\":";
    const char* p=strstr(l, pat.c_str()); if(!p) return -1;
    return atol(p+pat.size());
}

// ── reader 스레드: 워커 stdout 자막 JSON → 레코드 방출 ───────────────────────
static void reader_loop(){
    std::string acc;
    char buf[4096];
    while(!W.stop.load(std::memory_order_relaxed)){
        ssize_t r=read(W.out_fd, buf, sizeof(buf));
        if(r<=0) break;                     // 워커 종료 → EOF
        acc.append(buf, (size_t)r);
        size_t nl;
        while((nl=acc.find('\n'))!=std::string::npos){
            std::string line=acc.substr(0, nl); acc.erase(0, nl+1);
            if(line.empty()) continue;
            const char* l=line.c_str();
            if(strstr(l, "\"ready\"")){
                W.ready.store(true);
                // 모든 STT 채널 RUN 표시 (State 열 READY→RUN)
                if(g_v) for(int i=0;i<MAX_CHANNELS;i++)
                    if(g_v->channels[i].stt_on.load()) g_v->channels[i].stt_ready.store(true);
                continue;
            }
            char text[240];
            if(json_str(l, "text", text, sizeof(text)) && text[0]){
                SttRecord m{};
                m.t_ms = json_int(l, "t_ms");
                if(m.t_ms<=0) m.t_ms=(int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                m.ch   = (int32_t)json_int(l, "ch");
                long du= json_int(l, "dur");                // dur 은 실수지만 정수부만
                m.dur  = du>0 ? (float)du : 0.f;
                strncpy(m.text, text, sizeof(m.text)-1);
                // framework 방출: Central → 모든 join (+ 로컬 뷰는 on_data 로 반영)
                SttWireMsg w; stt_msg_to_wire(m, w);
                if(g_v) bewe_mod_emit(*g_v, "stt", &w, sizeof(w));
            }
        }
    }
}

// ── 워커 spawn / kill (mgmt 락 하에) ─────────────────────────────────────────
static bool spawn_worker(){
    if(W.pid>0) return true;
    int inp[2], outp[2];               // inp: 우리→워커stdin, outp: 워커stdout→우리
    if(pipe(inp)<0) return false;
    if(pipe(outp)<0){ close(inp[0]); close(inp[1]); return false; }
    pid_t pid=fork();
    if(pid<0){ close(inp[0]);close(inp[1]);close(outp[0]);close(outp[1]); return false; }
    if(pid==0){
        // child: stdin=inp[0], stdout=outp[1]
        dup2(inp[0], 0); dup2(outp[1], 1);
        close(inp[0]);close(inp[1]);close(outp[0]);close(outp[1]);
        // 래퍼 셸이 LD_LIBRARY_PATH(cublas/cudnn, python 버전 무관 glob) 잡고 워커 실행.
        std::string sh=stt_dir()+"/stt_run.sh";
        execl("/bin/sh", "sh", sh.c_str(), (char*)nullptr);
        _exit(127);                    // exec 실패
    }
    // parent
    close(inp[0]); close(outp[1]);
    W.in_fd=inp[1]; W.out_fd=outp[0]; W.pid=pid;
    W.stop.store(false); W.ready.store(false);
    W.reader=std::thread(reader_loop);
    W.writer=std::thread(writer_loop);
    return true;
}
static void kill_worker(){
    if(W.pid<=0) return;
    W.stop.store(true);
    if(W.in_fd>=0){ close(W.in_fd); W.in_fd=-1; }   // stdin EOF → 워커 자연 종료
    // 워커 정리 대기(최대 ~3초) 후 강제
    for(int i=0;i<30 && W.pid>0;i++){
        int st; pid_t r=waitpid(W.pid, &st, WNOHANG);
        if(r==W.pid){ W.pid=-1; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if(W.pid>0){ kill(W.pid, SIGTERM); waitpid(W.pid, nullptr, 0); W.pid=-1; }
    if(W.out_fd>=0){ close(W.out_fd); W.out_fd=-1; }
    if(W.reader.joinable()) W.reader.join();
    if(W.writer.joinable()) W.writer.join();
    W.ready.store(false);
    { std::lock_guard<std::mutex> lk(W.wq_mtx); W.wq.clear(); }
}

// ── HOST hooks ──────────────────────────────────────────────────────────────
static bool host_start(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return false;
    std::lock_guard<std::mutex> lk(W.mgmt);
    g_v=&v;
    Channel& c=v.channels[ch];
    if(!c.filter_active) return false;
    // 채널에 STT 발화 캡처 연결
    c.stt_ch_idx=ch; c.stt_sr=AUDIO_SR;
    c.stt_on_utt=&on_utterance;
    c.stt_state=Channel::SQR_IDLE; c.stt_utt.clear();
    c.stt_ready.store(W.ready.load());   // 워커 이미 준비됐으면 즉시 RUN, 아니면 READY
    c.stt_on.store(true, std::memory_order_release);
    // 첫 STT 채널이면 워커 확보
    if(W.active_ch.fetch_add(1)==0){
        if(!spawn_worker()){ W.active_ch.fetch_sub(1); c.stt_on.store(false); return false; }
    }
    return true;
}
static void stop_ch(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    Channel& c=v.channels[ch];
    if(!c.stt_on.exchange(false)) return;    // 이미 꺼짐 → 중복 감소 방지
    c.stt_ready.store(false);
    c.stt_on_utt=nullptr; c.stt_utt.clear(); c.stt_state=Channel::SQR_IDLE;
    if(W.active_ch.fetch_sub(1)==1) kill_worker();   // 마지막 채널 → 워커 종료
    bewe_mod_host_mask_clear(v, "stt", ch);
}
static void host_stop(FFTViewer& v, int ch){
    std::lock_guard<std::mutex> lk(W.mgmt);
    stop_ch(v, ch);
}
static void on_ch_stop(FFTViewer& v, int ch){
    std::lock_guard<std::mutex> lk(W.mgmt);
    stop_ch(v, ch);
}

// ── 데이터 수신 (JOIN/뷰어): 자막 레코드 → log append ────────────────────────
static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v;
    if(n < sizeof(SttWireMsg)) return;
    SttWireMsg w; memcpy(&w, d, sizeof(w));
    SttRecord m; stt_wire_to_msg(w, m);
    if(station) strncpy(m.station, station, sizeof(m.station)-1);
    std::lock_guard<std::mutex> lk(mtx);
    log.push_back(m);
    if(log.size()>5000) log.erase(log.begin(), log.begin()+(log.size()-5000));
}

// State RUN/READY 판정에 쓰이는 워커 준비상태 (demod_panel 이 조회)
bool stt_worker_ready(){ return W.ready.load(std::memory_order_relaxed); }
bool stt_worker_active(){ return W.active_ch.load(std::memory_order_relaxed)>0; }

#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);   // stt_view.cpp
#endif

// ── 모듈 등록 (static-init) ────────────────────────────────────────────────
static bool s_registered = [](){
    BeweModule m{};
    m.id    = "stt";
    m.label = "STT";
    m.planned = false;
    m.target_modes = (uint8_t)((1u<<Channel::DM_FM) | (1u<<Channel::DM_AM));  // AM/FM 음성
#ifndef BEWE_HEADLESS
    m.draw_content = &draw_content;
#endif
    m.host_start = &host_start;
    m.host_stop  = &host_stop;
    m.on_ch_stop = &on_ch_stop;
    m.on_data    = &on_data;
    bewe_register_module(m);
    return true;
}();

} // namespace stt_mod
