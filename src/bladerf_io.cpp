#include "fft_viewer.hpp"
#include <thread>
#include "net_server.hpp"
#include "long_waterfall.hpp"
#include <volk/volk.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

// ── USB 소프트 리셋 (USBDEVFS_RESET ioctl) ───────────────────────────────
// vid/pid 장치를 /dev/bus/usb에서 찾아 리셋
// 효과: 물리적으로 뽑았다 꽂는 것과 동일 (드라이버 unbind>reenumerate)
// sudo 불필요 - udev rule로 plugdev 그룹에 rw 권한 부여됨
bool usb_reset_vidpid(uint16_t want_vid, uint16_t want_pid, const char* label){
    // /dev/bus/usb/NNN/MMM 파일 순회해서 vendor/product 매칭
    DIR* bus_dir = opendir("/dev/bus/usb");
    if(!bus_dir){ perror("[USBreset] opendir /dev/bus/usb"); return false; }

    struct dirent* bus_ent;
    bool found = false;
    while(!found && (bus_ent = readdir(bus_dir))){
        if(bus_ent->d_name[0] == '.') continue;
        char bus_path[64];
        snprintf(bus_path, sizeof(bus_path), "/dev/bus/usb/%s", bus_ent->d_name);
        DIR* dev_dir = opendir(bus_path);
        if(!dev_dir) continue;
        struct dirent* dev_ent;
        while(!found && (dev_ent = readdir(dev_dir))){
            if(dev_ent->d_name[0] == '.') continue;
            char dev_path[128];
            snprintf(dev_path, sizeof(dev_path), "%s/%s", bus_path, dev_ent->d_name);
            int fd = open(dev_path, O_RDWR);
            if(fd < 0) continue;

            // USB descriptor: byte 8=vendor(LE16), byte 10=product(LE16)
            uint8_t desc[18] = {};
            if(read(fd, desc, sizeof(desc)) == (ssize_t)sizeof(desc)){
                uint16_t vid = (uint16_t)(desc[8]  | (desc[9]  << 8));
                uint16_t pid = (uint16_t)(desc[10] | (desc[11] << 8));
                if(vid == want_vid && pid == want_pid){
                    bewe_log_push(0,"[USBreset] found %s at %s - issuing USBDEVFS_RESET\n",
                                  label, dev_path);
                    if(ioctl(fd, USBDEVFS_RESET, nullptr) == 0){
                        bewe_log_push(0,"[USBreset] reset OK\n");
                        found = true;
                    } else {
                        perror("[USBreset] ioctl USBDEVFS_RESET");
                    }
                }
            }
            close(fd);
        }
        closedir(dev_dir);
    }
    closedir(bus_dir);
    if(!found) bewe_log_push(0,"[USBreset] %s not found in /dev/bus/usb\n", label);
    return found;
}

bool bladerf_usb_reset(){ return usb_reset_vidpid(0x2cf0, 0x5250, "BladeRF"); }

// ── USB 딥 파워사이클 (/powercycle) ──────────────────────────────────────
// bladerf_usb_reset() 의 USBDEVFS_RESET 은 /dev/bus/usb 노드를 열어 ioctl 을 쏘는
// 방식이라 장치가 응답해야 성립한다. NIOS II 링크가 죽으면 ("Failed to receive
// NIOS II response: Operation timed out") 그 리셋조차 장치에 안 닿아 /chassis 1
// reset 을 몇 번 쳐도 복구되지 않는다.
//
// 여기서는 장치를 우회해 커널에게 시킨다. sysfs 의 authorized 에 0 을 쓰면 커널이
// 드라이버를 unbind 하고 장치를 사실상 죽인다. 1 을 쓰면 재열거(re-enumerate) —
// 케이블을 뽑았다 꽂는 것과 같은 경로다. 드론 탑재라 사람이 못 뽑는 기지에서
// 이게 유일한 상위 복구 수단이다.
//
// sysfs 경로는 /dev/bus/usb 노드가 아니라 /sys/bus/usb/devices/<X-Y>/ 이고,
// idVendor/idProduct 가 텍스트 파일(16진 4자리)로 들어있다.
static bool read_sysfs_hex4(const char* path, uint16_t* out){
    int fd = open(path, O_RDONLY);
    if(fd < 0) return false;
    char buf[16] = {};
    ssize_t n = read(fd, buf, sizeof(buf)-1);
    close(fd);
    if(n <= 0) return false;
    *out = (uint16_t)strtoul(buf, nullptr, 16);
    return true;
}

int usb_deep_powercycle(uint16_t vid, uint16_t pid, int off_ms){
    const char* base = "/sys/bus/usb/devices";
    DIR* d = opendir(base);
    if(!d){ bewe_log_push(2,"[PWRCYCLE] opendir %s failed\n", base); return 1; }

    char dev_dir[256] = {};
    struct dirent* e;
    while((e = readdir(d))){
        if(e->d_name[0] == '.') continue;
        // 인터페이스 노드(3-1:1.0)와 루트허브(usb3)는 건너뛴다 — 장치 노드만 본다.
        if(strchr(e->d_name, ':') || strncmp(e->d_name, "usb", 3) == 0) continue;
        char p[512]; uint16_t v = 0, pd = 0;
        snprintf(p, sizeof(p), "%s/%s/idVendor", base, e->d_name);
        if(!read_sysfs_hex4(p, &v) || v != vid) continue;
        snprintf(p, sizeof(p), "%s/%s/idProduct", base, e->d_name);
        if(!read_sysfs_hex4(p, &pd) || pd != pid) continue;
        snprintf(dev_dir, sizeof(dev_dir), "%s/%s", base, e->d_name);
        break;
    }
    closedir(d);

    if(!dev_dir[0]){
        bewe_log_push(2,"[PWRCYCLE] device %04x:%04x not found in %s\n", vid, pid, base);
        return 1;
    }

    char auth[512];
    snprintf(auth, sizeof(auth), "%s/authorized", dev_dir);
    if(access(auth, W_OK) != 0){
        bewe_log_push(2,"[PWRCYCLE] %s not writable - deploy udev rule "
                        "(assets/udev/99-bewe-usb-powercycle.rules)\n", auth);
        return 2;
    }

    auto write_auth = [&](const char* val)->bool{
        int fd = open(auth, O_WRONLY);
        if(fd < 0) return false;
        ssize_t w = write(fd, val, 1);
        close(fd);
        return w == 1;
    };

    bewe_log_push(0,"[PWRCYCLE] %s: deauthorize (%04x:%04x)\n", dev_dir, vid, pid);
    if(!write_auth("0")){
        bewe_log_push(2,"[PWRCYCLE] write 0 failed: %s\n", strerror(errno));
        return 3;
    }
    // FX3 같은 USB 컨트롤러가 완전히 내려갔다 올라올 시간을 준다. 너무 짧으면
    // 재열거는 되는데 펌웨어가 절반만 올라와 다시 NIOS timeout 이 난다.
    std::this_thread::sleep_for(std::chrono::milliseconds(off_ms));

    bewe_log_push(0,"[PWRCYCLE] reauthorize\n");
    if(!write_auth("1")){
        // 여기서 실패하면 장치가 deauthorized 로 남아 아예 안 보인다. 재시도로
        // 반드시 되살려야 한다 — 실패해도 로그로 남겨 운용자가 알게 한다.
        for(int i = 0; i < 5; i++){
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if(write_auth("1")){
                bewe_log_push(0,"[PWRCYCLE] reauthorize OK (retry %d)\n", i+1);
                return 0;
            }
        }
        bewe_log_push(2,"[PWRCYCLE] REAUTHORIZE FAILED - device left deauthorized!\n");
        return 3;
    }
    bewe_log_push(0,"[PWRCYCLE] done - device re-enumerating\n");
    return 0;
}

bool FFTViewer::initialize_bladerf(float cf_mhz, float sr_msps){
    int s=bladerf_open(&dev_blade,nullptr);
    if(s){ bewe_log("bladerf_open: %s\n",bladerf_strerror(s)); return false; }

    s=bladerf_set_frequency(dev_blade,BLADERF_CHANNEL_RX(0),(uint64_t)(cf_mhz*1e6));
    if(s){ bewe_log("set_freq: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    uint32_t actual=0;
    s=bladerf_set_sample_rate(dev_blade,BLADERF_CHANNEL_RX(0),(uint32_t)(sr_msps*1e6),&actual);
    if(s){ bewe_log("set_sr: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    uint32_t actual_bw=0;
    s=bladerf_set_bandwidth(dev_blade,BLADERF_CHANNEL_RX(0),(uint32_t)(sr_msps*1e6*0.8f),&actual_bw);
    if(s){ bewe_log("set_bw: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    // Manual Gain Control (AGC 비활성화)
    s=bladerf_set_gain_mode(dev_blade,BLADERF_CHANNEL_RX(0),BLADERF_GAIN_MGC);
    if(s) bewe_log("set_gain_mode: %s\n",bladerf_strerror(s));

    s=bladerf_set_gain(dev_blade,BLADERF_CHANNEL_RX(0),BLADERF_RX_GAIN);
    if(s){ bewe_log("set_gain: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    s=bladerf_enable_module(dev_blade,BLADERF_CHANNEL_RX(0),true);
    if(s){ bewe_log("enable: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    s=bladerf_sync_config(dev_blade,BLADERF_RX_X1,BLADERF_FORMAT_SC16_Q11,512,16384,128,5000);
    if(s){ bewe_log("sync: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    bewe_log("BladeRF: %.2f MHz  %.2f MSPS  BW %.2f MHz\n",cf_mhz,actual/1e6f,actual_bw/1e6f);

    hw = make_bladerf_config(actual);
    // 재초기화(chassis 1 / powercycle / 자동 재연결)에서는 운용자가 맞춰 둔 gain 을
    // 지켜야 한다. 무조건 gain_default 로 덮으면 복구 때마다 감도가 조용히 바뀐다
    // (DGS-X 실측: 21.0 dB > 10.0 dB). 최초 오픈일 때만 기본값을 채운다.
    if(gain_db <= 0.f) gain_db = hw.gain_default;
    std::memcpy(header.magic,"FFTD",4);
    fft_input_size = fft_size / FFT_PAD_FACTOR;  // fft_size is already padded in member init
    header.version=1; header.fft_size=fft_size; header.sample_rate=actual;
    header.center_frequency=(uint64_t)(cf_mhz*1e6);
    live_cf_hz.store((uint64_t)(cf_mhz*1e6), std::memory_order_release);
    time_average=hw.compute_time_average(fft_input_size);
    header.time_average=time_average; header.power_min=-100; header.power_max=0; header.num_ffts=0;
    fft_data.resize((size_t)FFT_HISTORY_ROWS*fft_size);
    current_spectrum.resize(fft_size,-100.0f);

    char title[256]; snprintf(title,256,"BEWE (" BEWE_VERSION ")");
    (void)cf_mhz;
    window_title=title; display_power_min=-100; display_power_max=0;
    fft_in =fftwf_alloc_complex(fft_size);
    fft_out=fftwf_alloc_complex(fft_size);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex));  // zero-pad region
    fft_plan=bewe_fft_plan(fft_size,fft_in,fft_out,FFTW_FORWARD,/*learn=*/true);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex)); // MEASURE가 입력 파괴 > 재초기화
    // Pre-compute Nuttall window + allocate VOLK mag_sq buffer
    if(win_buf) free(win_buf);
    win_buf=(float*)volk_malloc(fft_input_size*sizeof(float), volk_get_alignment());
    fill_nuttall_window(win_buf, fft_input_size);
    if(mag_sq_buf) volk_free(mag_sq_buf);
    mag_sq_buf=(float*)volk_malloc(fft_size*sizeof(float), volk_get_alignment());
    ring.resize(IQ_RING_CAPACITY*2,0);
    autoscale_req.store(true, std::memory_order_relaxed); // SDR (재)시작 시 자동 autoscale
    return true;
}

void FFTViewer::capture_and_process(){
    CapLifeGuard cap_life(&cap_exited);
    // RX 버퍼: fft_size와 무관하게 최소 32768 샘플 고정 > USB 오버헤드 최소화
    // + sync_rx 호출/캡처 스레드 웨이크업 1/4 (고SR context switch 절감, 지연 +0.8ms@40MSPS)
    static constexpr int RX_MIN = 32768;
    int rx_chunk = std::max(fft_input_size, RX_MIN);
    int16_t* iq_buf=new int16_t[rx_chunk*2];
    // FFT 처리용 오프셋 (rx_chunk 내 슬라이딩)
    int rx_pos = 0; // iq_buf 내 현재 읽기 위치 (샘플 단위)
    int rx_avail = 0; // iq_buf에 유효한 샘플 수

    std::vector<float> pacc(fft_size,0.0f); int fcnt=0;
    // 초기 안정화: 처음 N번 FFT 결과 버림
    static constexpr int WARMUP_FFTS = 30;
    int warmup_cnt = 0;
    // FFT 서브샘플링: waterfall 행당 최대 MAX_ROW_FFTS개 윈도우만 FFT, 나머지 스킵.
    // 행 rate(~18/s)는 유지, 행당 평균 표본 수만 감소 → 고SR 캡처 스레드 CPU 수배 절감.
    static constexpr int MAX_ROW_FFTS = 32;
    int win_skip = 0;

    // ── 스트림 에러 시 장치 정리 (핸들 누수 방지) ────────────────────────
    // 예전엔 dev_blade 를 nullptr 로 만들기만 하고 bladerf_close() 를 안 했다.
    // 그러면 libbladeRF 핸들이 누수돼 USB 장치가 이 프로세스에 claim 된 채 남고,
    // 재연결 경로의 `if(dev_blade){ bladerf_close(...) }` 는 이미 nullptr 이라 안 돌며,
    // hw_detect 의 bladerf_get_device_list() 가 0 을 반환 → "No SDR device found" 를
    // 프로세스 재시작 전까지 무한 반복한다 (2026-07-12 DGS-5 실장애: 일시적 RX 에러
    // 하나가 64분 영구 장애로 굳음). 반드시 닫고 나간다.
    auto blade_teardown = [&](){
        if(dev_blade){
            bladerf_enable_module(dev_blade, BLADERF_CHANNEL_RX(0), false);
            bladerf_close(dev_blade);
        }
        dev_blade = nullptr;
    };
    // sync_rx 연속 타임아웃 상한 (3s x 3 = 9s). FFT-stall watchdog(10s)보다 먼저 스스로 감지.
    static constexpr int RX_TIMEOUT_MAX = 3;
    int rx_timeouts = 0;

    // sdr_stream_error 를 루프 조건에 포함: watchdog 등 외부에서 에러를 세팅했을 때
    // 캡처 스레드가 스스로 빠져나와야 cap.join() 이 걸리지 않는다 (안 그러면 재연결
    // 스레드가 join 에서 영구 블록 → USB reset/재초기화가 아예 실행되지 않음).
    while(is_running && !sdr_stream_error.load(std::memory_order_relaxed)){
        // ── Pause (타임머신 모드) ─────────────────────────────────────────
        if(capture_pause.load(std::memory_order_relaxed)){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            rx_avail=0; rx_pos=0;
            continue;
        }

        // ── FFT size change — off-lock 재할당 + atomic swap (broadcast race 방지)
        if(fft_size_change_req){
            fft_size_change_req=false; int ns=pending_fft_size;
            int new_input = ns;
            int new_fft_sz = ns * FFT_PAD_FACTOR;
            // demod 스레드 일시 정지: ring 접근 충돌 방지
            size_t cur_wp = ring_wp.load(std::memory_order_relaxed);
            for(int ci=0;ci<MAX_CHANNELS;ci++)
                channels[ci].dem_rp.store(cur_wp, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // ① Off-lock: capture 전용 FFTW/VOLK 자원 재구성
            fftwf_destroy_plan(fft_plan); fftwf_free(fft_in); fftwf_free(fft_out);
            fft_in =fftwf_alloc_complex(new_fft_sz);
            fft_out=fftwf_alloc_complex(new_fft_sz);
            memset(fft_in, 0, new_fft_sz*sizeof(fftwf_complex));
            fft_plan=bewe_fft_plan(new_fft_sz,fft_in,fft_out,FFTW_FORWARD,/*learn=*/false);
            memset(fft_in, 0, new_fft_sz*sizeof(fftwf_complex)); // MEASURE가 입력 파괴 > 재초기화
            if(win_buf) volk_free(win_buf);
            win_buf=(float*)volk_malloc(new_input*sizeof(float), volk_get_alignment());
            fill_nuttall_window(win_buf, new_input);
            if(mag_sq_buf) volk_free(mag_sq_buf);
            mag_sq_buf=(float*)volk_malloc(new_fft_sz*sizeof(float), volk_get_alignment());
            rx_chunk = std::max(new_input, RX_MIN);
            delete[] iq_buf; iq_buf=new int16_t[rx_chunk*2];
            rx_pos=0; rx_avail=0;
            pacc.assign(new_fft_sz,0.0f); fcnt=0;
            // ② 원자적 스왑: fft_size와 fft_data 동시 교체
            {std::lock_guard<std::mutex> lk(data_mtx);
             fft_input_size=new_input;
             fft_size=new_fft_sz;
             time_average=hw.compute_time_average(fft_input_size);
             header.fft_size=fft_size;
             fft_data.assign((size_t)FFT_HISTORY_ROWS*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;}
            texture_needs_recreate=true;
            LongWaterfall::request_rotate();   // fft_size changed → new file
            continue;
        }

        // ── Sample rate change ────────────────────────────────────────────
        if(sr_change_req){
            sr_change_req=false;
            uint32_t new_sr = (uint32_t)(pending_sr_msps * 1e6f);

            // TM IQ 리셋: on/off 여부 관계없이 기존 롤링 파일 삭제 후 재생성
            bool tm_was_on = tm_iq_on.load(std::memory_order_relaxed);
            if(tm_was_on) tm_iq_on.store(false);
            tm_iq_close(); // fd 닫기 + 상태 초기화 (이미 closed면 no-op)
            // 기존 SR 롤링 파일 삭제 (SR 불일치 방지)
            {
                std::string tm_dir = BEWEPaths::time_temp_dir();
                char old_path[256];
                snprintf(old_path, sizeof(old_path), "%s/iq_rolling_%uMSPS.wav",
                         tm_dir.c_str(), header.sample_rate/1000000);
                if(access(old_path, F_OK)==0){ remove(old_path); }
            }

            // 122.88M 이상 > SC8_Q7 (8bit) + OVERSAMPLE, 그 외 > SC16_Q11 (16bit)
            bool was_sc8 = sc8_mode;
            sc8_mode = (new_sr >= 122880000);
            bladerf_format fmt = sc8_mode ? BLADERF_FORMAT_SC8_Q7 : BLADERF_FORMAT_SC16_Q11;

            bladerf_enable_module(dev_blade,BLADERF_CHANNEL_RX(0),false);

            // OVERSAMPLE 피쳐: SC8 진입 시 enable, 복귀 시 disable
            if(sc8_mode != was_sc8){
                int fe = bladerf_enable_feature(dev_blade, BLADERF_FEATURE_OVERSAMPLE, sc8_mode);
                if(fe) bewe_log("enable_feature(OVERSAMPLE,%d): %s\n", sc8_mode, bladerf_strerror(fe));
            }

            uint32_t actual_sr=0, actual_bw=0;
            int sr_err = bladerf_set_sample_rate(dev_blade,BLADERF_CHANNEL_RX(0),new_sr,&actual_sr);
            if(sr_err) bewe_log("set_sr(%u): %s\n", new_sr, bladerf_strerror(sr_err));

            // BW: 최대한 열기 (AD9361이 지원하는 범위로 자동 클램프됨)
            uint32_t req_bw = (uint32_t)(actual_sr*0.8f);
            int bw_err = bladerf_set_bandwidth(dev_blade,BLADERF_CHANNEL_RX(0),req_bw,&actual_bw);
            if(bw_err) bewe_log("set_bw(%u): %s\n", req_bw, bladerf_strerror(bw_err));

            bladerf_enable_module(dev_blade,BLADERF_CHANNEL_RX(0),true);
            int sc_err = bladerf_sync_config(dev_blade,BLADERF_RX_X1,fmt,512,16384,128,5000);
            if(sc_err) bewe_log("sync_config(fmt=%d): %s\n", (int)fmt, bladerf_strerror(sc_err));

            bewe_log("SC8=%d req=%u actual_sr=%u actual_bw=%u\n", sc8_mode, new_sr, actual_sr, actual_bw);

            // HW 파라미터 갱신
            hw = make_bladerf_config(actual_sr);
            if(sc8_mode) hw.iq_scale = 128.0f; // SC8_Q7: 7bit > 128
            time_average = hw.compute_time_average(fft_input_size);

            {std::lock_guard<std::mutex> lk(data_mtx);
             header.sample_rate = actual_sr;
             header.time_average = time_average;
             fft_data.assign((size_t)FFT_HISTORY_ROWS*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;}
            {std::lock_guard<std::mutex> lk(wf_events_mtx);
             wf_events.clear(); last_tagged_sec=-1;}

            rx_chunk = std::max(fft_input_size, RX_MIN);
            delete[] iq_buf; iq_buf = new int16_t[rx_chunk*2];
            rx_pos=0; rx_avail=0;
            pacc.assign(fft_size,0.0f); fcnt=0; warmup_cnt=0;
            texture_needs_recreate=true;
            // SR 변경 > 신호 크기 스케일이 달라질 수 있어 오토스케일 재트리거
            autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
            autoscale_wp=0; autoscale_buf_full=false;
            sq_recalib_req.store(true, std::memory_order_relaxed);  // 노이즈플로어 변동 → 자동 스컬치 재캘리브
            // TM IQ: SC8 모드(122.88M)에서는 롤링 IQ 비활성화
            if(tm_was_on && !sc8_mode){
                tm_iq_open();
                // open 거부(디스크 예산 초과) 시 OFF 유지
                if(tm_iq_file_ready) tm_iq_on.store(true);
            }
            // SR 변경 후 게인 재적용 (BladeRF가 SR 변경 시 게인을 리셋할 수 있음)
            set_gain(gain_db);
            dem_restart_needed.store(true); // demod가 새 SR로 재초기화되도록
            bewe_log("SR > %.2f MSPS  BW > %.2f MHz\n", actual_sr/1e6f, actual_bw/1e6f);
            // SR 변경으로 가시 대역폭이 달라짐 → 범위 재평가 (Holding/Active 전환)
            update_dem_by_freq(header.center_frequency/1e6f);
            continue;
        }

        // ── Frequency change ──────────────────────────────────────────────
        if(freq_req.load(std::memory_order_acquire)&&!freq_prog){
            freq_prog=true;
            const float cf = pending_cf.load(std::memory_order_relaxed);
            int s=bladerf_set_frequency(dev_blade,BLADERF_CHANNEL_RX(0),(uint64_t)(cf*1e6));
            if(!s){
                {std::lock_guard<std::mutex> lk(data_mtx);
                 header.center_frequency=(uint64_t)(cf*1e6);}
                live_cf_hz.store((uint64_t)(cf*1e6), std::memory_order_release);
                LongWaterfall::request_rotate();   // CF changed → new file
                bewe_log("Freq > %.2f MHz\n",cf);
                autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
                autoscale_wp=0; autoscale_buf_full=false;
                autoscale_req.store(false, std::memory_order_relaxed);  // 방금 리셋했으니 중복 트리거 소거
                sq_recalib_req.store(true, std::memory_order_relaxed);  // 노이즈플로어 변동 → 자동 스컬치 재캘리브
                warmup_cnt=0;
                update_dem_by_freq(cf);
            }
            freq_req.store(false, std::memory_order_relaxed); freq_prog=false;
        }

        // ── RX: 고정 청크(min 8192)로 읽기 > fft_size 무관 일정 throughput ──
        if(rx_avail == 0){
            int status=bladerf_sync_rx(dev_blade,iq_buf,rx_chunk,nullptr,3000);
            if(status){
                if(status==BLADERF_ERR_TIMEOUT){
                    // is_fpga_configured: 1=정상, 0=미구성, <0=조회 실패(장치 이상).
                    // 예전 코드는 `!bladerf_is_fpga_configured(...)` 라 음수(조회 실패)를
                    // 0(=거짓) 이 아닌 참으로 봐서 "정상" 으로 오판 → 타임아웃 루프에 갇힘.
                    int fc = dev_blade ? bladerf_is_fpga_configured(dev_blade) : 0;
                    if(fc <= 0){
                        fprintf(stderr,"BladeRF: device lost during timeout (fpga=%d)\n", fc);
                        bewe_log("BladeRF: device lost (fpga=%d)\n", fc);
                        blade_teardown();
                        sdr_stream_error.store(true);
                        break;
                    }
                    // 연속 타임아웃 = 스트림 사망. 예전엔 여기서 무한히 10ms sleep 만 돌아
                    // CPU ~0% 로 조용히 멎은 채 sdr_stream_error 도 안 세워 재연결이 영영
                    // 트리거되지 않았다 (silent death).
                    if(++rx_timeouts >= RX_TIMEOUT_MAX){
                        fprintf(stderr,"BladeRF: RX stalled (%d consecutive timeouts) - reconnecting\n",
                                rx_timeouts);
                        bewe_log("BladeRF: RX stalled (%dx timeout) - reconnecting\n", rx_timeouts);
                        blade_teardown();
                        sdr_stream_error.store(true);
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                bewe_log("RX error: %s\n",bladerf_strerror(status));
                fprintf(stderr,"BladeRF: fatal RX error (%d) - SDR disconnected\n", status);
                blade_teardown();
                sdr_stream_error.store(true);
                break;
            }
            rx_timeouts = 0;   // 정상 수신 → 연속 타임아웃 카운터 리셋
            // SC8_Q7: int8 샘플을 int16으로 확장 (뒤에서부터 > in-place 안전)
            if(sc8_mode){
                int8_t* i8 = (int8_t*)iq_buf;
                for(int k = rx_chunk*2 - 1; k >= 0; k--)
                    iq_buf[k] = (int16_t)i8[k];
            }
            // IQ Ring write: 전체 청크를 한 번에 ring에 추가
            bool need_ring=rec_on.load(std::memory_order_relaxed)
                          ||mod_wants_ring.load(std::memory_order_relaxed); // 광대역 모듈(WiFi)이 full-rate ring 요청
            if(!need_ring) for(int i=0;i<MAX_CHANNELS;i++){
                if(channels[i].dem_run.load()){need_ring=true;break;}
            }
            bool need_tm=!sc8_mode&&tm_iq_on.load(std::memory_order_relaxed)&&(warmup_cnt>=WARMUP_FFTS);
            if(need_ring||need_tm){
                size_t wp=ring_wp.load(std::memory_order_relaxed);
                size_t n=(size_t)rx_chunk, cap=IQ_RING_CAPACITY;
                if(wp+n<=cap) memcpy(&ring[wp*2],iq_buf,n*2*sizeof(int16_t));
                else{
                    size_t p1=cap-wp, p2=n-p1;
                    memcpy(&ring[wp*2],iq_buf,p1*2*sizeof(int16_t));
                    memcpy(&ring[0],iq_buf+p1*2,p2*2*sizeof(int16_t));
                }
                ring_wp.store((wp+n)&IQ_RING_MASK,std::memory_order_release);
                if(need_tm) tm_iq_write(iq_buf,(int)n);
            }
            rx_pos=0; rx_avail=rx_chunk;
        }

        // ── FFT: 버퍼에서 fft_input_size씩 처리 ─────────────────────────
        // 남은 샘플이 fft_input_size 미만이면 다음 RX로
        if(rx_avail < fft_input_size){ rx_avail=0; rx_pos=0; continue; }

        const int16_t* iq = iq_buf + rx_pos*2;

        if(!render_visible.load(std::memory_order_relaxed)){
            rx_pos+=fft_input_size; rx_avail-=fft_input_size;
            std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            continue;
        }
        if(!spectrum_pause.load(std::memory_order_relaxed)){
            const int fft_stride = time_average>MAX_ROW_FFTS
                                 ? (time_average+MAX_ROW_FFTS-1)/MAX_ROW_FFTS : 1;
            if(++win_skip < fft_stride){
                rx_pos+=fft_input_size; rx_avail-=fft_input_size;
                continue;
            }
            win_skip=0;
            // Fill input samples (first fft_input_size), rest stays zero (zero-padding)
            for(int i=0;i<fft_input_size;i++){
                fft_in[i][0]=iq[i*2]/hw.iq_scale;
                fft_in[i][1]=iq[i*2+1]/hw.iq_scale;
            }
            // Nuttall window via VOLK SIMD (complex × real element-wise)
            volk_32fc_32f_multiply_32fc((lv_32fc_t*)fft_in, (lv_32fc_t*)fft_in,
                                        win_buf, fft_input_size);
            // pad region은 init/resize 시 한 번만 0 초기화 (out-of-place FFT > fft_in 불변)
            fftwf_execute(fft_plan);
            {
                // VOLK magnitude squared: |X[k]|² for all bins
                volk_32fc_magnitude_squared_32f(mag_sq_buf, (lv_32fc_t*)fft_out, fft_size);
                const float scale=NUTTALL_WINDOW_CORRECTION/((float)fft_input_size*(float)fft_input_size);
                for(int i=0;i<fft_size;i++){
                    pacc[i] += mag_sq_buf[i]*scale + 1e-10f;
                }
            }
            pacc[0]=(pacc[1]+pacc[fft_size-1])*0.5f; fcnt++;
            if(fcnt>=(time_average+fft_stride-1)/fft_stride){
                if(warmup_cnt < WARMUP_FFTS){
                    warmup_cnt++;
                    std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
                    rx_pos+=fft_input_size; rx_avail-=fft_input_size;
                    continue;
                }
                int fi=total_ffts%FFT_HISTORY_ROWS;
                float* rowp=fft_data.data()+fi*fft_size;
                {std::lock_guard<std::mutex> lk(data_mtx);
                 // current_spectrum은 UI 스레드 전용(픽셀별 peak) > 캡처가 절대 쓰지 않음
                 // (과거 bin별 avg를 여기에 덮어써 UI 파워스펙트럼에 1프레임 깨짐 유발했음)
                 // dB 변환: 10*log10(pacc/fcnt) = 3.0103*log2(pacc) - 10*log10(fcnt).
                 // 스칼라 log10f 루프(bin 당 1회) → VOLK SIMD log2 로 교체 (NEON/AVX).
                 // pacc 는 항상 >=1e-10 (누적 시 +1e-10f) 이라 -inf 없음. 오차 <1e-3 dB.
                 {
                     volk_32f_log2_32f(rowp, pacc.data(), (unsigned int)fft_size);
                     volk_32f_s32f_multiply_32f(rowp, rowp, 3.01029996f, (unsigned int)fft_size);
                     const float row_off = 10.0f*log10f((float)fcnt);
                     for(int i=0;i<fft_size;i++) rowp[i] -= row_off;
                 }
                 // 비-캡처 스레드 요청 처리 (set_frequency/init) — 여기서만 autoscale 상태 변경 (레이스 X)
                 if(autoscale_req.exchange(false)){
                     autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
                     autoscale_wp=0; autoscale_buf_full=false;
                     sq_recalib_req.store(true, std::memory_order_relaxed);  // 노이즈플로어 변동 → 자동 스컬치 재캘리브
                 }
                 if(autoscale_active){
                     auto now_as=std::chrono::steady_clock::now();
                     if(autoscale_start==std::chrono::steady_clock::time_point{})
                         autoscale_start=now_as;   // 데드라인 기준 — 재트리거로 되감지 않는다
                     if(!autoscale_init){
                         size_t cap=(size_t)fft_size*100;
                         if(autoscale_accum.size()!=cap) autoscale_accum.assign(cap,0.0f);
                         autoscale_wp=0; autoscale_buf_full=false;
                         autoscale_last=now_as;
                         autoscale_init=true;
                     }
                     size_t cap=autoscale_accum.size();
                     for(int i=1;i<fft_size;i++){
                         autoscale_accum[autoscale_wp]=rowp[i];  // current_spectrum 대신 rowp 직접 사용
                         if(++autoscale_wp>=cap){ autoscale_wp=0; autoscale_buf_full=true; }
                     }
                     float el=std::chrono::duration<float>(now_as-autoscale_last).count();
                     float el_total=std::chrono::duration<float>(now_as-autoscale_start).count();
                     bool  deadline=el_total>=AUTOSCALE_DEADLINE_S;   // 재트리거 폭주 시 강제 확정
                     if((el>=1.0f||deadline)&&(autoscale_buf_full||autoscale_wp>0)){
                         size_t n=autoscale_buf_full?cap:autoscale_wp;
                         std::vector<float> tmp(autoscale_accum.begin(),
                                                autoscale_accum.begin()+(ptrdiff_t)n);
                         // 노이즈 플로어: 15% 분위수 → pmin = noise - 5dB
                         // 피크: 99% 분위수 → pmax = peak + 20dB
                         size_t idx_lo=(size_t)(n*0.15f);
                         std::nth_element(tmp.begin(),tmp.begin()+(ptrdiff_t)idx_lo,tmp.end());
                         float noise=tmp[idx_lo];
                         float peak=*std::max_element(tmp.begin(),tmp.end());
                         display_power_min=noise-5.0f;
                         display_power_max=peak+20.0f;
                         if(display_power_max-display_power_min<20.f)
                             display_power_max=display_power_min+20.f;
                         header.power_min=display_power_min;
                         header.power_max=display_power_max;
                         bewe_log_push(0,"[autoscale]%s noise=%.1f peak=%.1f → pmin=%.1f pmax=%.1f\n",
                             deadline?" (deadline)":"", noise, peak, display_power_min, display_power_max);
                         autoscale_active=false; autoscale_init=false;
                         autoscale_wp=0; autoscale_buf_full=false;
                         autoscale_start=std::chrono::steady_clock::time_point{};
                         cached_sp_idx=-1;
                     }
                 }
                 total_ffts++; current_fft_idx=total_ffts-1;
                 header.num_ffts=std::min(total_ffts,FFT_HISTORY_ROWS);
                 row_write_pos[current_fft_idx%MAX_FFTS_MEMORY]=tm_iq_write_sample;
                 row_wall_ms[current_fft_idx%MAX_FFTS_MEMORY]=(int64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                 if(tm_iq_on.load(std::memory_order_relaxed))
                     tm_mark_rows(current_fft_idx%MAX_FFTS_MEMORY);
                 else
                     iq_row_avail[current_fft_idx%MAX_FFTS_MEMORY]=false;
                 tm_add_time_tag(current_fft_idx);
                 net_bcast_seq.fetch_add(1, std::memory_order_release);
                 net_bcast_cv.notify_one();
                }
                std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            }
        } // end !spectrum_pause
        rx_pos+=fft_input_size; rx_avail-=fft_input_size;
    }
    delete[] iq_buf;
    if(dev_blade){
        bladerf_enable_module(dev_blade, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(dev_blade);
        dev_blade = nullptr;
    }
}