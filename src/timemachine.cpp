#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "long_waterfall.hpp"
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>

static char s_iq_path[256]={};
static constexpr off_t WAV_HDR_SIZE = 44; // WAV 헤더 크기

// WAV 헤더 작성 (stereo int16: L=I, R=Q)
static void write_rolling_wav_header(int fd, uint32_t sample_rate, uint32_t n_frames){
    uint32_t data_bytes  = n_frames * 4;
    uint32_t chunk_size  = 36 + data_bytes;
    uint16_t audio_fmt   = 1, channels = 2;
    uint32_t byte_rate   = sample_rate * 4;
    uint16_t block_align = 4, bits = 16;
    uint8_t hdr[44]={};
    memcpy(hdr+0,"RIFF",4); memcpy(hdr+4,&chunk_size,4);
    memcpy(hdr+8,"WAVE",4); memcpy(hdr+12,"fmt ",4);
    uint32_t sc1=16; memcpy(hdr+16,&sc1,4);
    memcpy(hdr+20,&audio_fmt,2); memcpy(hdr+22,&channels,2);
    memcpy(hdr+24,&sample_rate,4); memcpy(hdr+28,&byte_rate,4);
    memcpy(hdr+32,&block_align,2); memcpy(hdr+34,&bits,2);
    memcpy(hdr+36,"data",4); memcpy(hdr+40,&data_bytes,4);
    pwrite(fd, hdr, 44, 0);
}

void FFTViewer::tm_iq_open(){
    // open/close 직렬화 — 두 JOIN 이 동시에 토글하거나 SR 변경(캡처 스레드)과
    // 토글(net/UI 스레드)이 겹치면 tm_iq_wr_thr 이중 assign → std::terminate 방지.
    std::lock_guard<std::mutex> oc(tm_iq_oc_mtx);
    if(tm_iq_file_ready){
        // writer pwrite 실패로 멈춘 상태에서 재토글 → close/재생성으로 복구
        // (아니면 file_ready 가 true 라 조용히 전량 drop 상태로 재개장됨)
        if(!tm_iq_write_failed.load()) return;
        tm_iq_close_locked();
    }
    struct stat st{};
    std::string tm_dir=BEWEPaths::time_temp_dir();
    const char* TM_IQ_DIR=tm_dir.c_str();
    if(stat(TM_IQ_DIR,&st)!=0) mkdir(TM_IQ_DIR,0755);
    uint32_t sr=header.sample_rate;
    if(sr==0){ fprintf(stderr,"TM: sample_rate 0\n"); return; }
    // 디스크 속도 사전 거부는 하지 않는다 — 기지마다 저장장치가 달라 상수로 못 박고,
    // 실측 게이트는 기동을 느리게 한다. 스펙을 넘는 SR 은 drop 로그와 끊긴 녹음으로
    // 사용자가 즉시 알아챈다 (tm_iq_dropped_bytes / write_failed).
    tm_iq_total_samples=(int64_t)sr*(int64_t)TM_IQ_SECS;
    snprintf(s_iq_path,sizeof(s_iq_path),"%s/iq_rolling_%uMSPS.wav",TM_IQ_DIR,sr/1000000);
    // 기존 파일 항상 삭제 후 새로 생성
    if(access(s_iq_path,F_OK)==0){ remove(s_iq_path); bewe_log_push(0,"TM: removed old %s\n",s_iq_path); }
    tm_iq_fd=open(s_iq_path, O_RDWR|O_CREAT|O_TRUNC, 0644);
    if(tm_iq_fd<0){ fprintf(stderr,"TM: open failed: %s\n",strerror(errno)); return; }
    // WAV 헤더 placeholder (n_frames=0, Stop 시 갱신)
    write_rolling_wav_header(tm_iq_fd, sr, 0);
    tm_iq_write_sample=0; tm_iq_flushed_sample=0;
    tm_iq_chunk_write=0; tm_iq_chunk_sample_start=0;
    memset(tm_iq_chunk_time,0,sizeof(tm_iq_chunk_time));
    { std::lock_guard<std::mutex> lk(tm_iq_q_mtx); tm_iq_q.clear(); tm_iq_q_bytes=0; }
    tm_iq_dropped_bytes.store(0);
    tm_iq_write_failed.store(false);
    tm_iq_wr_run.store(true);
    tm_iq_wr_thr = std::thread(&FFTViewer::tm_iq_writer_loop, this);
    tm_iq_file_ready=true;
    bewe_log_push(0,"TM IQ rolling: ready (wav)  max %.1f GB\n",
           (double)(tm_iq_total_samples*2*sizeof(int16_t))/1e9);
    LongWaterfall::request_rotate();   // start a fresh long-waterfall file
}

void FFTViewer::tm_iq_close(){
    std::lock_guard<std::mutex> oc(tm_iq_oc_mtx);
    tm_iq_close_locked();
}

void FFTViewer::tm_iq_close_locked(){
    // writer 스레드 종료 — 큐 잔여분을 모두 쓴 뒤 빠져나옴 (flush 보장)
    if(tm_iq_wr_run.load()){
        tm_iq_wr_run.store(false);
        tm_iq_q_cv.notify_all();
    }
    if(tm_iq_wr_thr.joinable()) tm_iq_wr_thr.join();
    if(tm_iq_fd>=0){
        // Stop: WAV 헤더를 실제 샘플 수로 갱신
        uint32_t actual = (uint32_t)std::min((int64_t)tm_iq_write_sample, tm_iq_total_samples);
        write_rolling_wav_header(tm_iq_fd, header.sample_rate, actual);
        close(tm_iq_fd); tm_iq_fd=-1;
        uint64_t dropped = tm_iq_dropped_bytes.load();
        if(dropped)
            bewe_log_push(0,"TM IQ rolling: %.1f MB dropped total (disk too slow)\n",dropped/1e6);
        bewe_log_push(0,"TM IQ rolling: closed  %.2f sec\n",(double)actual/header.sample_rate);
    }
    tm_iq_file_ready=false; tm_iq_write_sample=0; tm_iq_flushed_sample=0;
    memset(tm_iq_chunk_time,0,sizeof(tm_iq_chunk_time));
    LongWaterfall::request_rotate();   // close current long-waterfall file
}

// writer 스레드: 큐에서 청크를 꺼내 ×16 스케일링 후 링 위치에 pwrite.
// tm_iq_wr_run=false 후에도 큐 잔여분을 모두 쓰고 종료.
void FFTViewer::tm_iq_writer_loop(){
    bool write_failed=false;
    for(;;){
        TmIqChunk ck;
        {
            std::unique_lock<std::mutex> lk(tm_iq_q_mtx);
            tm_iq_q_cv.wait(lk,[&]{
                return !tm_iq_q.empty() || !tm_iq_wr_run.load(std::memory_order_relaxed); });
            if(tm_iq_q.empty()){
                if(!tm_iq_wr_run.load(std::memory_order_relaxed)) return;
                continue;
            }
            ck=std::move(tm_iq_q.front());
            tm_iq_q.pop_front();
            tm_iq_q_bytes -= ck.data.size()*sizeof(int16_t);
        }
        if(write_failed){ // pwrite 실패 후: 큐만 비움 (재개는 close→open)
            tm_iq_dropped_bytes.fetch_add(ck.data.size()*sizeof(int16_t),std::memory_order_relaxed);
            continue;
        }
        // SC16_Q11 → ×16 스케일링: ±2048 → ±32768 (URH 풀스케일 정규화)
        for(size_t i=0;i<ck.data.size();i++){
            int32_t v=(int32_t)ck.data[i]*16;
            ck.data[i]=(int16_t)std::max(-32768,std::min(32767,v));
        }
        int n=(int)(ck.data.size()/2);
        int written=0;
        int64_t sample_pos=ck.start_sample;
        while(written<n){
            int64_t max_total=tm_iq_total_samples;
            int64_t pos=(sample_pos<max_total)?sample_pos:sample_pos%max_total;
            int64_t avail=max_total-pos;
            int chunk=(int)std::min((int64_t)(n-written),avail);
            off_t offset = WAV_HDR_SIZE + pos*2*(off_t)sizeof(int16_t);
            ssize_t bytes=(ssize_t)chunk*2*(ssize_t)sizeof(int16_t);
            if(pwrite(tm_iq_fd, ck.data.data()+written*2, (size_t)bytes, offset)!=bytes){
                bewe_log_push(0,"TM IQ: pwrite failed (%s) - rolling stopped\n",strerror(errno));
                write_failed=true;
                tm_iq_write_failed.store(true);   // 다음 tm_iq_open 이 close/재생성으로 복구
                tm_iq_on.store(false);
                break;
            }
            written+=chunk; sample_pos+=chunk;
            int64_t cur_sec=sample_pos/(int64_t)header.sample_rate;
            int ci=(int)(cur_sec%(int64_t)TM_IQ_SECS);
            if(ci!=tm_iq_chunk_write){ tm_iq_chunk_write=ci; tm_iq_chunk_time[ci]=time(nullptr); }
        }
        // 디스크 기록 완료 워터마크 — 읽기 소비자(region/TM replay)는 여기까지만 신뢰
        if(!write_failed)
            tm_iq_flushed_sample.store(ck.start_sample+n, std::memory_order_release);
    }
}

void FFTViewer::tm_iq_write(const int16_t* buf, int n_pairs){
    if(!tm_iq_file_ready||n_pairs<=0) return;
    // 캡처 스레드: 복사+enqueue만. 스케일링/디스크는 writer 스레드 (캡처 비블로킹).
    TmIqChunk ck;
    ck.start_sample = tm_iq_write_sample.load(std::memory_order_relaxed);
    ck.data.assign(buf, buf+(size_t)n_pairs*2);
    tm_iq_write_sample.store(ck.start_sample + n_pairs, std::memory_order_relaxed);
    size_t bytes = ck.data.size()*sizeof(int16_t);
    uint64_t dropped_now=0;
    {
        std::lock_guard<std::mutex> lk(tm_iq_q_mtx);
        while(tm_iq_q_bytes+bytes > TM_IQ_QUEUE_MAX_BYTES && !tm_iq_q.empty()){
            size_t b = tm_iq_q.front().data.size()*sizeof(int16_t);
            tm_iq_q_bytes -= b; dropped_now += b;
            tm_iq_q.pop_front();
        }
        tm_iq_q.push_back(std::move(ck));
        tm_iq_q_bytes += bytes;
    }
    tm_iq_q_cv.notify_one();
    if(dropped_now){
        tm_iq_dropped_bytes.fetch_add(dropped_now, std::memory_order_relaxed);
        static time_t s_last_drop_log=0;
        time_t now=time(nullptr);
        if(now!=s_last_drop_log){
            s_last_drop_log=now;
            bewe_log_push(0,"TM IQ: disk too slow - dropped %.1f MB total\n",
                          (double)tm_iq_dropped_bytes.load()/1e6);
        }
    }
}

void FFTViewer::tm_mark_rows(int fi){
    if(!tm_iq_file_ready) return;
    iq_row_avail[fi%MAX_FFTS_MEMORY]=true;
}

void FFTViewer::tm_add_time_tag(int fft_idx){
    time_t now=time(nullptr);
    struct tm t; KST::to_tm(now, t);
    int cur5=t.tm_hour*720+t.tm_min*12+t.tm_sec/5; // 5초 단위 카운터
    if(cur5==last_tagged_sec&&last_tagged_sec!=-1) return;
    last_tagged_sec=cur5;
    WfEvent ev; ev.fft_idx=fft_idx; ev.wall_time=now; ev.type=0;
    strftime(ev.label,sizeof(ev.label),"%M:%S",&t);
    std::lock_guard<std::mutex> lk(wf_events_mtx);
    wf_events.push_back(ev);
    int cutoff=fft_idx-MAX_FFTS_MEMORY;
    wf_events.erase(std::remove_if(wf_events.begin(),wf_events.end(),
        [&](const WfEvent& e){ return e.fft_idx<cutoff; }),wf_events.end());
}

void FFTViewer::tm_add_event_tag(int type){
    time_t now=time(nullptr); struct tm t; KST::to_tm(now, t);
    WfEvent ev; ev.fft_idx=current_fft_idx; ev.wall_time=now; ev.type=type;
    snprintf(ev.label,sizeof(ev.label),"%s  %02d:%02d:%02d",
             type==1?"IQ Start":"IQ Stop",t.tm_hour,t.tm_min,t.tm_sec);
    std::lock_guard<std::mutex> lk(wf_events_mtx);
    wf_events.push_back(ev);
}

// IQ 롤링 토글 — 상단바 IQ LED 클릭과 I 키가 공용으로 부른다.
// JOIN 이면 HOST 에 원격 요청만 보내고 로컬 상태는 CH/IQ 동기화로 따라간다.
void FFTViewer::toggle_tm_iq(){
    if(remote_mode && net_cli){ net_cli->cmd_toggle_tm_iq(); return; }
    if(tm_iq_on.load()){
        tm_iq_on.store(false);
        tm_add_event_tag(2);
        tm_iq_was_stopped = true;
#ifdef BEWE_HOST_BUILD
        if(net_srv) net_srv->broadcast_wf_event(0,(int64_t)time(nullptr),2,"IQ Stop");
#endif
    } else {
        if(tm_iq_was_stopped){ tm_iq_close(); tm_iq_was_stopped = false; }
        tm_iq_open();
        if(tm_iq_file_ready){          // open 거부(디스크 예산 초과) 시 OFF 유지
            tm_iq_on.store(true);
            tm_add_event_tag(1);
#ifdef BEWE_HOST_BUILD
            if(net_srv) net_srv->broadcast_wf_event(0,(int64_t)time(nullptr),1,"IQ Start");
#endif
        }
    }
}

time_t FFTViewer::fft_idx_to_wall_time(int fft_idx) const {
    std::lock_guard<std::mutex> lk(wf_events_mtx);
    if(wf_events.empty()) return 0;

    // fft_idx 기준으로 가장 가까운 두 이벤트 찾기 (앞뒤)
    // wf_events는 fft_idx 오름차순이라고 가정
    const WfEvent* prev = nullptr;
    const WfEvent* next = nullptr;
    for(const auto& ev : wf_events){
        if(ev.fft_idx <= fft_idx) prev = &ev;
        if(ev.fft_idx >= fft_idx && !next) next = &ev;
    }

    if(prev && next && prev != next){
        // 두 이벤트 사이 보간: 실제 wall_time 차이로 rps 추정
        int64_t fi_diff = next->fft_idx - prev->fft_idx;
        int64_t wt_diff = (int64_t)next->wall_time - (int64_t)prev->wall_time;
        if(fi_diff > 0 && wt_diff > 0){
            int64_t offset = fft_idx - prev->fft_idx;
            return (time_t)(prev->wall_time + offset * wt_diff / fi_diff);
        }
    }
    if(prev){
        // prev만 있으면 rps로 외삽
        float rps = (float)header.sample_rate / (float)fft_input_size / (float)time_average;
        if(rps <= 0) rps = 37.5f;
        int64_t offset = fft_idx - prev->fft_idx;
        return (time_t)(prev->wall_time + (int64_t)(offset / rps));
    }
    if(next){
        float rps = (float)header.sample_rate / (float)fft_input_size / (float)time_average;
        if(rps <= 0) rps = 37.5f;
        int64_t offset = fft_idx - next->fft_idx; // 음수
        return (time_t)(next->wall_time + (int64_t)(offset / rps));
    }
    return 0;
}

int64_t FFTViewer::fft_idx_to_wall_time_ms(int fft_idx) const {
    int slot = fft_idx % MAX_FFTS_MEMORY;
    int64_t ms = row_wall_ms[slot];
    return (ms > 0) ? ms : 0;
}

void FFTViewer::tm_update_display(){
    // wf_events 기반 실제 rps 계산 (JOIN/HOST 모두 정확)
    float rps = 0.0f;
    {
        std::lock_guard<std::mutex> lk(wf_events_mtx);
        // 가장 멀리 떨어진 두 이벤트로 실제 rps 추정
        if(wf_events.size() >= 2){
            const WfEvent& first = wf_events.front();
            const WfEvent& last  = wf_events.back();
            int64_t fi_diff = last.fft_idx - first.fft_idx;
            int64_t wt_diff = (int64_t)last.wall_time - (int64_t)first.wall_time;
            if(fi_diff > 0 && wt_diff > 0)
                rps = (float)fi_diff / (float)wt_diff;
        }
    }
    if(rps <= 0){
        rps = (float)header.sample_rate / (float)fft_input_size / (float)time_average;
        if(rps <= 0) rps = 37.5f;
    }

    // 최대 오프셋 = freeze 시점 기준 버퍼 용량 (최대 MAX_FFTS_MEMORY-1 행)
    int max_rows=std::min(tm_freeze_idx, MAX_FFTS_MEMORY-1);
    tm_max_sec=(float)max_rows/rps;
    // 60초(1분)로 스크롤 상한 고정
    const float TM_MAX_SCROLL_SEC = 60.0f;
    tm_max_sec = std::min(tm_max_sec, TM_MAX_SCROLL_SEC);

    // 60초 한계에 도달한 상태에서 새 FFT가 들어오면 freeze_idx를 현재로 갱신 → 최근 1분 follow
    const float FOLLOW_EPSILON = 0.5f;
    if(tm_offset >= tm_max_sec - FOLLOW_EPSILON && current_fft_idx > tm_freeze_idx){
        tm_freeze_idx = current_fft_idx;
        tm_offset = tm_max_sec;
    }

    tm_offset=std::max(0.0f,std::min(tm_offset,tm_max_sec));
    int row_offset=(int)(tm_offset*rps);
    tm_display_fft_idx=tm_freeze_idx - row_offset;
    if(tm_display_fft_idx<0) tm_display_fft_idx=0;
}

bool FFTViewer::tm_rec_start(){
#ifndef BEWE_HOST_BUILD
    // TM 롤링 IQ 파일은 HOST 가 쓴다 — JOIN 엔 잘라낼 원본이 없다.
    return false;
#else
    if(!tm_iq_file_ready||tm_iq_fd<0){ return false; }
    int disp_row=tm_display_fft_idx%MAX_FFTS_MEMORY;
    if(!iq_row_avail[disp_row]){ return false; }
    int fi=selected_ch;
    if(fi<0||!channels[fi].filter_active){ return false; }
    int64_t samp_offset=(int64_t)((double)header.sample_rate*tm_offset);
    // 비동기 writer: 디스크 기록 완료 지점(flushed)까지만 읽기 — 큐 적체분(아직
    // 파일에 없는 최신 구간)을 읽으면 이전 pass 잔재가 나옴.
    int64_t head = tm_iq_write_sample.load(std::memory_order_relaxed);
    { int64_t flushed = tm_iq_flushed_sample.load(std::memory_order_acquire);
      if(flushed > 0 && flushed < head) head = flushed; }
    int64_t read_pos=head-samp_offset;
    if(read_pos<0) read_pos=tm_iq_total_samples+read_pos;
    read_pos=read_pos%tm_iq_total_samples;
    tm_rec_read_pos=read_pos; tm_rec_active=true;
    start_rec(); return true;
#endif
}