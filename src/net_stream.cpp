#include <ctime>
#include "fft_viewer.hpp"
#include "net_server.hpp"

// ── 브로드캐스트 전용 스레드 ─────────────────────────────────────────────
// 캡처 스레드에서 TCP send를 절대 호출하지 않도록 분리.
// net_bcast_seq가 증가하면 깨어나서 최신 FFT 행을 복사 후 전송.
void FFTViewer::net_bcast_worker(){
    int last_seq = -1;
    // 전송 버퍼 (로컬 복사 → send 중 data_mtx 불필요)
    std::vector<float> local_fft;
    uint64_t local_cf  = 0;
    uint32_t local_sr  = 0;
    float    local_min = -80.f, local_max = 0.f;
    int      local_sz  = 0;
    int64_t  local_wt  = 0;
    int64_t  local_iq_pos = 0;
    int64_t  local_iq_total = 0;

    while(!net_bcast_stop.load(std::memory_order_relaxed)){
        // 새 FFT 행 대기
        {
            std::unique_lock<std::mutex> lk(net_bcast_mtx);
            net_bcast_cv.wait_for(lk, std::chrono::milliseconds(100), [&]{
                return net_bcast_seq.load(std::memory_order_acquire) != last_seq
                    || net_bcast_stop.load(std::memory_order_relaxed);
            });
        }
        if(net_bcast_stop.load(std::memory_order_relaxed)) break;

        int cur_seq = net_bcast_seq.load(std::memory_order_acquire);
        if(cur_seq == last_seq) continue;
        last_seq = cur_seq;

        if(!net_srv) continue;
        // 로컬 client도 Central relay JOIN도 없으면 스킵 (idle 양자화/uplink 낭비 방지)
        if(net_srv->client_count() == 0 && !net_srv->has_relay()) continue;
        if(net_bcast_pause.load(std::memory_order_relaxed)) continue;

        // 최신 FFT 행을 로컬 버퍼로 빠르게 복사 (data_mtx는 최소 시간만 점유)
        {
            std::lock_guard<std::mutex> lk(data_mtx);
            local_sz  = fft_size;
            local_cf  = header.center_frequency;
            local_sr  = header.sample_rate;
            // 캡처 양자화 범위만 전송 (HOST 화면 스케일 아님 → JOIN 독립 스케일 유지)
            local_min = header.power_min;
            local_max = header.power_max;
            local_wt  = (int64_t)time(nullptr);
            // fft_data 링(FFT_HISTORY_ROWS)과 메타 링(MAX_FFTS_MEMORY)은 깊이가
            // 다를 수 있어 인덱스 분리 — IQ 좌표는 반드시 메타 링 기준.
            int fi_row  = (current_fft_idx) % FFT_HISTORY_ROWS;
            int fi_meta = (current_fft_idx) % MAX_FFTS_MEMORY;
            // 이 프레임을 생성한 시점의 HOST IQ 좌표 스냅샷
            local_iq_pos   = row_write_pos[fi_meta];
            local_iq_total = tm_iq_total_samples;
            const float* rowp = fft_data.data() + (size_t)fi_row * fft_size;
            // assign() 대신 resize()+memcpy: fft_size 불변 시 heap 재할당 없음
            if((int)local_fft.size() != fft_size) local_fft.resize(fft_size);
            memcpy(local_fft.data(), rowp, (size_t)fft_size * sizeof(float));
        }

        // TCP 전송 (블로킹이어도 캡처 스레드와 무관)
        net_srv->broadcast_fft(local_fft.data(), local_sz,
                               local_wt,
                               local_cf, local_sr,
                               local_min, local_max,
                               local_iq_pos, local_iq_total);
    }
}