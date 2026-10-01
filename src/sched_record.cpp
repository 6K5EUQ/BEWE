#include "fft_viewer.hpp"
#include "net_server.hpp"
#include "bewe_paths.hpp"
#include "login.hpp"
#include "kst_time.hpp"
#include "sigmf.hpp"
#include "sat_sched.hpp"
#include "sat_trim.hpp"
#include "mission_push.hpp"
#include <ctime>
#include <chrono>
#include <thread>
#include <cstdio>

// ── sched 리스트를 SCHED_SYNC 패킷으로 변환 ──────────────────────────────
// 호출자는 sched_mtx를 잡은 상태여야 함
static PktSchedSync build_sched_sync_pkt(const std::vector<FFTViewer::SchedEntry>& list){
    PktSchedSync pkt{};
    int n = (int)list.size();
    if(n > MAX_SCHED_ENTRIES) n = MAX_SCHED_ENTRIES;
    pkt.count = (uint8_t)n;
    for(int i=0; i<n; i++){
        const auto& e = list[i];
        auto& se = pkt.entries[i];
        se.valid        = 1;
        se.status       = (uint8_t)e.status;
        se.op_index     = e.op_index;
        se.start_time   = (int64_t)e.start_time;
        se.duration_sec = e.duration_sec;
        se.freq_mhz     = e.freq_mhz;
        se.bw_khz       = e.bw_khz;
        strncpy(se.operator_name, e.operator_name, sizeof(se.operator_name)-1);
        strncpy(se.target,        e.target,        sizeof(se.target)-1);
        se.mission_year = (uint16_t)e.mission_year;
        memcpy(se.mission_code, e.mission_code, sizeof(se.mission_code));
        se.sr_hz        = e.sr_hz;
    }
    return pkt;
}

// 외부에서 호출 가능한 헬퍼 (cli_host.cpp / ui.cpp에서 사용)
// sched_mtx는 이 함수가 직접 잡음
void FFTViewer::broadcast_sched_list(){
    if(!net_srv) return;
    PktSchedSync pkt;
    {
        std::lock_guard<std::mutex> lk(sched_mtx);
        pkt = build_sched_sync_pkt(sched_entries);
    }
    net_srv->broadcast_sched_sync(pkt);
}

// sched_has_overlap 은 fft_viewer.hpp 로 옮겨 인라인이 되었다 — JOIN(GUI)도
// ADD 버튼 활성화 판정에 쓰는데, 이 파일은 두 타겟 모두에 컴파일되므로
// 여기 정의를 남겨두면 중복 정의가 된다.

void FFTViewer::sched_tick(){
    std::lock_guard<std::mutex> lk(sched_mtx);
    time_t now = time(nullptr);

    // Active slot (ARMED or RECORDING)
    if(sched_active_idx >= 0 && sched_active_idx < (int)sched_entries.size()){
        auto& e = sched_entries[sched_active_idx];
        if(e.status == SchedEntry::ARMED){
            if(now >= e.start_time)
                sched_begin_rec(sched_active_idx);
        } else if(e.status == SchedEntry::RECORDING){
            float elapsed = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - e.rec_started).count();
            if(elapsed >= e.duration_sec)
                sched_stop_entry(sched_active_idx);
        }
    }

    // No active slot: find next WAITING entry eligible for pre-arm
    if(sched_active_idx < 0){
        for(int i = 0; i < (int)sched_entries.size(); i++){
            auto& e = sched_entries[i];
            if(e.status != SchedEntry::WAITING) continue;
            // Entirely missed?
            if(now > e.start_time + (time_t)e.duration_sec){
                e.status = SchedEntry::FAILED;
                broadcast_sched_list_locked();
                bewe_log_push(0, "[SCHED] Entry %d missed (time passed)\n", i);
                continue;
            }
            // Pre-arm window reached?
            if(now >= e.start_time - (time_t)SCHED_PRE_ARM_SEC){
                sched_arm_entry(i);
                break;
            }
        }
    }
}

// 예약이 바꾼 SDR 상태(SR·CF)를 예약 전으로 되돌린다.
static void sched_restore_sdr(FFTViewer& v){
    if(v.sched_saved_sr_msps > 0.f){
        v.pending_sr_msps = v.sched_saved_sr_msps;
        v.sr_change_req   = true;
        bewe_log_push(0, "[SCHED] SR restored > %.3f MSPS\n", v.sched_saved_sr_msps);
        v.sched_saved_sr_msps = 0.f;
    }
    v.set_frequency(v.sched_saved_cf, /*wait=*/true);
    bewe_log_push(0, "[SCHED] Freq restored > %.3f MHz\n", v.sched_saved_cf);
}

// Pre-arm: start_time - SCHED_PRE_ARM_SEC 시점에 호출됨.
// SDR 튠(DC 오프셋 적용), 채널 할당, demod 시작만 수행. IQ 기록은 아직.
// 이 구간 동안 PLL lock / IIR 과도응답 / squelch 보정이 안정화됨.
void FFTViewer::sched_arm_entry(int idx){
    auto& e = sched_entries[idx];

    if(remote_mode){ e.status=SchedEntry::FAILED; broadcast_sched_list_locked(); bewe_log_push(0,"[SCHED] Failed: JOIN mode\n"); return; }
    bool sdr_ok = (dev_blade != nullptr)
               || (dev_rtl   != nullptr)
               || (hw.type == HWType::PLUTO && pluto_ctx != nullptr);
    if(!sdr_ok){ e.status=SchedEntry::FAILED; broadcast_sched_list_locked(); bewe_log_push(0,"[SCHED] Failed: no SDR\n"); return; }

    sched_saved_cf = (float)(header.center_frequency / 1e6);
    sched_saved_sr_msps = 0.f;

    // 전대역 모드(sr_hz>0): SDR 을 CF=목표, SR=요청값으로 바꿔 두고 끝. 채널 없음.
    // 캡처 스레드가 SR 변경을 소비하고 워밍업할 시간이 pre-arm 구간이다.
    if(e.sr_hz > 0){
        if(rec_on.load()){
            e.status = SchedEntry::FAILED;
            broadcast_sched_list_locked();
            bewe_log_push(0, "[SCHED] Failed: full-band recorder busy\n");
            return;
        }
        float want_msps = e.sr_hz / 1e6f;
        float cur_msps  = header.sample_rate / 1e6f;
        if(fabsf(want_msps - cur_msps) > 1e-6f){
            sched_saved_sr_msps = cur_msps;
            pending_sr_msps     = want_msps;
            sr_change_req       = true;
        }
        set_frequency(e.freq_mhz, /*wait=*/true);
        e.status = SchedEntry::ARMED;
        sched_active_idx = idx;
        broadcast_sched_list_locked();
        bewe_log_push(0, "[SCHED] ARMED: full-band %.4f MHz SR=%.3f MSPS dur=%.0fs target='%s' (T-%.1fs)\n",
                      e.freq_mhz, want_msps, e.duration_sec, e.target, SCHED_PRE_ARM_SEC);
        return;
    }

    // DC 오프셋: SDR CF를 target + offset 로 이동 → DC 스파이크가 채널 베이스밴드에서 -offset 위치로
    // 밀려 채널 LPF 바깥이 되어 제거됨.
    float bw_mhz     = e.bw_khz / 1000.0f;
    float sdr_sr_mhz = (float)(header.sample_rate) / 1e6f;
    float min_off    = bw_mhz;                                    // 채널 필터 바깥으로 밀어냄
    float max_off    = sdr_sr_mhz * 0.45f - bw_mhz * 0.5f;        // SDR 유효 대역 헤드룸
    float offset;
    if(max_off <= 0.f){
        offset = 0.f;
        bewe_log_push(0,"[SCHED] Warn: BW exceeds SDR headroom; DC may intrude\n");
    } else {
        offset = (max_off < min_off) ? max_off : min_off;
    }
    float sdr_cf = e.freq_mhz + offset;

    set_frequency(sdr_cf, /*wait=*/true);
    bewe_log_push(0, "[SCHED] ARM: SDR CF %.3f MHz (target %.3f + DC offset %.3f)\n",
                  sdr_cf, e.freq_mhz, offset);

    int slot = -1;
    for(int i = 0; i < MAX_CHANNELS; i++){
        if(!channels[i].filter_active){ slot = i; break; }
    }
    if(slot < 0){
        e.status = SchedEntry::FAILED;
        set_frequency(sched_saved_cf, /*wait=*/true);
        broadcast_sched_list_locked();
        bewe_log_push(0, "[SCHED] Failed: no free channel slot\n");
        return;
    }

    // 채널 [s, e]는 target 기준 — IQ-only worker가 SDR CF에서 target으로 mixer
    float half_bw = e.bw_khz / 2000.0f;
    channels[slot].reset_slot();
    channels[slot].s = e.freq_mhz - half_bw;
    channels[slot].e = e.freq_mhz + half_bw;
    channels[slot].filter_active = true;
    const char* owner_src = (e.operator_name[0]) ? e.operator_name : "SCHED";
    strncpy(channels[slot].owner, owner_src, 31);
    channels[slot].audio_mask.store(0x0);  // 스케줄 녹음은 audio 재생 안 함
    e.temp_ch_idx = slot;

    // demod 안 시작 — start_iq_rec이 IQ-only worker로 직접 IQ ring 소비.
    // squelch 무시하고 전 구간 녹음 (예약 녹음 의도).
    channels[slot].iq_rec_force_all.store(true);

    // 위성 예약: 채널 믹서가 TLE 도플러를 따라가게 한다 (패스 동안 수신 주파수가 ±수십 kHz 움직임)
    if(int norad = SatSched::norad_of(e.target)){
        iq_doppler_fn[slot] = SatSched::doppler_fn(norad, station_lat, -station_lon,
                                                   (double)e.freq_mhz * 1e6);
        if(!iq_doppler_fn[slot])
            bewe_log_push(0, "[SCHED] Warn: no TLE for %d - recording without Doppler tracking\n", norad);
    }

    e.status = SchedEntry::ARMED;
    sched_active_idx = idx;

    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
    broadcast_sched_list_locked();
    bewe_log_push(0, "[SCHED] ARMED: CH%d %.3f MHz BW=%.0f kHz dur=%.0fs%s (T-%.1fs)\n",
                  slot, e.freq_mhz, e.bw_khz, e.duration_sec,
                  iq_doppler_fn[slot] ? " Doppler-tracked" : "", SCHED_PRE_ARM_SEC);
}

// sched_mtx를 이미 잡은 상태에서 호출 가능한 브로드캐스트 (내부 잠금 없음)
void FFTViewer::broadcast_sched_list_locked(){
    if(!net_srv) return;
    PktSchedSync pkt = build_sched_sync_pkt(sched_entries);
    net_srv->broadcast_sched_sync(pkt);
}

// start_time 도달 시 호출 — ARM된 엔트리의 IQ 기록만 실제로 시작.
void FFTViewer::sched_begin_rec(int idx){
    auto& e = sched_entries[idx];
    if(e.sr_hz > 0){
        pending_sched_meta.active    = true;
        pending_sched_meta.start_utc = e.start_time;
        pending_sched_meta.end_utc   = e.start_time + (time_t)e.duration_sec;
        if(!start_sched_fullband_rec()){
            pending_sched_meta.active = false;
            bewe_log_push(0, "[SCHED] REC start failed: full-band recorder busy\n");
            sched_restore_sdr(*this);
            e.status = SchedEntry::FAILED;
            sched_active_idx = -1;
            broadcast_sched_list_locked();
            return;
        }
        e.status      = SchedEntry::RECORDING;
        e.rec_started = std::chrono::steady_clock::now();
        broadcast_sched_list_locked();
        bewe_log_push(0, "[SCHED] REC start: full-band %.4f MHz SR=%u (req %u) dur=%.0fs\n",
                      header.center_frequency/1e6, header.sample_rate, e.sr_hz, e.duration_sec);
        return;
    }
    int slot = e.temp_ch_idx;
    if(slot < 0 || slot >= MAX_CHANNELS){
        e.status = SchedEntry::FAILED;
        sched_active_idx = -1;
        broadcast_sched_list_locked();
        return;
    }
    // SCHED 파일명 핸드오프: start_iq_rec이 IQ_ 대신 SCHED_ 포맷 사용하게.
    pending_sched_meta.active    = true;
    pending_sched_meta.start_utc = e.start_time;
    pending_sched_meta.end_utc   = e.start_time + (time_t)e.duration_sec;
    start_iq_rec(slot);
    if(!channels[slot].iq_rec_on.load()){
        bewe_log_push(0, "[SCHED] REC start failed: CH%d\n", slot);
        stop_dem(slot);
        channels[slot].reset_slot();
        set_frequency(sched_saved_cf, /*wait=*/true);
        e.status = SchedEntry::FAILED;
        sched_active_idx = -1;
        if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
        broadcast_sched_list_locked();
        return;
    }
    e.status      = SchedEntry::RECORDING;
    e.rec_started = std::chrono::steady_clock::now();
    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
    broadcast_sched_list_locked();
    bewe_log_push(0, "[SCHED] REC start: CH%d %.3f MHz dur=%.0fs\n",
                  slot, e.freq_mhz, e.duration_sec);
}

void FFTViewer::sched_stop_entry(int idx){
    auto& e = sched_entries[idx];
    int slot = e.temp_ch_idx;

    // 녹음 경로와 메타정보 스냅샷 (stop 이후 채널 reset되기 전에 캡처).
    // SCHED_ 파일은 stop 때 rename 되지 않으므로 이 경로가 최종 경로다.
    std::string iq_path;
    if(e.sr_hz > 0)
        iq_path = rec_filename;
    else if(slot >= 0 && slot < MAX_CHANNELS)
        iq_path = channels[slot].iq_rec_path;
    char entry_op[32] = {};
    strncpy(entry_op, e.operator_name, sizeof(entry_op)-1);
    time_t entry_start = e.start_time;
    float  entry_dur   = e.duration_sec;
    float  entry_freq  = e.freq_mhz;
    float  entry_bw    = e.bw_khz;
    // 위성 채널 녹음은 재절단 뒤에 올린다 — 먼저 push 하면 Central ACK 시 로컬본이 지워져 자를 게 없다
    const bool sat_trim = (e.sr_hz == 0) && SatSched::norad_of(e.target) > 0;
    // 재절단의 위성/지상 판별에 녹음 때 따라간 도플러를 그대로 쓴다 (아래에서 채널 것은 비운다)
    std::function<double(double)> trim_dop;
    if(sat_trim && slot >= 0 && slot < MAX_CHANNELS) trim_dop = iq_doppler_fn[slot];

    if(e.sr_hz > 0){
        stop_rec();   // rec_worker join + .sigmf-meta Duration 갱신
    } else {
        // IQ-only worker stop & wav finalize (stop_iq_rec 안에서 worker join)
        if(slot >= 0 && slot < MAX_CHANNELS){
            stop_iq_rec(slot, /*push=*/!sat_trim);
            iq_doppler_fn[slot] = nullptr;
            channels[slot].reset_slot();
        }
    }

    sched_restore_sdr(*this);

    e.status = SchedEntry::DONE;
    sched_active_idx = -1;

    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
    broadcast_sched_list_locked();
    bewe_log_push(0, "[SCHED] Recording complete: entry %d\n", idx);

    // 자동 DB 업로드 (별도 스레드) — 파일 finalize 대기 후 전송
    if(!iq_path.empty() && (sched_db_upload_fn || sat_trim)){
        auto upload_fn = sched_db_upload_fn;
        std::thread([upload_fn, iq_path, entry_op, entry_start, entry_dur, entry_freq, entry_bw, sat_trim, trim_dop](){
            if(sat_trim){
                // 수백 MB 를 훑는 일이라 이 스레드에서. 실패해도 1차 파일은 그대로 남는다.
                SatTrim::Result tr = SatTrim::trim(iq_path, trim_dop);
                if(tr.trimmed)
                    bewe_log_push(0, "[SCHED] Trimmed: occupied %.2f kHz (centre %+.2f kHz, %d s with signal) -> SR %u\n",
                                  tr.occ_bw_hz/1e3, tr.center_off_hz/1e3, tr.signal_rows, tr.sr_out);
                else
                    bewe_log_push(0, "[SCHED] Not trimmed (%s) - keeping capture-width file\n", tr.why.c_str());
                MissionPush::enqueue(iq_path, MFS_IQ);
                if(!upload_fn) return;
            }
            // stop_iq_rec()가 동기로 fclose+.info Duration 갱신까지 끝냈으므로 sleep 불필요.

            // sched 특유 메타 (예약자·시각·길이·폭)
            char tbuf[64] = {};
            struct tm tm_kst; KST::to_tm(entry_start, tm_kst);
            strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%S", &tm_kst);
            char note_body[192];
            snprintf(note_body, sizeof(note_body),
                "Scheduled by %s @ %s, dur=%.0fs, BW=%.1fkHz",
                entry_op[0] ? entry_op : "?", tbuf, entry_dur, entry_bw);
            // SigMF 사이드카는 JSON 이라 아래 Key:Value 줄 교체를 하면 깨진다 —
            // 노트를 meta 에 먼저 기록하고 그 파일을 그대로 보낸다.
            bool sigmf = SigMF::is_sigmf_data(iq_path);
            if(sigmf){
                // 재절단이 남긴 노트(도플러 보정·점유폭)를 지우지 않게 뒤에 잇는다
                std::string prev = SigMF::read_note(iq_path);
                SigMF::update_note(iq_path, prev.empty() ? std::string(note_body)
                                                         : std::string(note_body) + "; " + prev);
            }

            // stop_iq_rec()가 이미 표준 형식 사이드카를 생성/갱신했음.
            // 그 내용을 그대로 read해서 업로드 (free-form 텍스트 대신 표준 포맷 보장)
            std::string info_str;
            std::string ipath = SigMF::sidecar_path(iq_path);
            FILE* fi = fopen(ipath.c_str(), "r");
            if(fi){
                char buf[2048];
                size_t n = fread(buf, 1, sizeof(buf)-1, fi);
                buf[n] = 0;
                info_str = buf;
                fclose(fi);
            }
            if(sigmf){ upload_fn(iq_path, entry_op, info_str); return; }
            char sched_note[256];
            snprintf(sched_note, sizeof(sched_note), "Notes: %s\n", note_body);
            // Notes: 라인 교체 (없으면 끝에 append)
            std::string out;
            bool note_replaced = false;
            size_t pos = 0;
            while(pos < info_str.size()){
                size_t eol = info_str.find('\n', pos);
                if(eol == std::string::npos) eol = info_str.size();
                std::string line = info_str.substr(pos, eol - pos);
                if(!note_replaced && line.rfind("Notes:", 0) == 0){
                    out += sched_note;
                    note_replaced = true;
                } else {
                    out += line;
                    if(eol < info_str.size()) out += "\n";
                }
                pos = eol + 1;
            }
            if(!note_replaced) out += sched_note;
            (void)entry_freq; // freq는 표준 .info의 Freq 필드에 이미 들어있음
            upload_fn(iq_path, entry_op, out);
        }).detach();
    }
}
