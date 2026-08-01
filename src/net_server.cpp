#include "net_server.hpp"
#include "../central/central_proto.hpp"
#include "module_api.hpp"   // bewe_mod_host_ch_decstat (디코드 통계 → ChSyncEntry)

// CHANNEL_SYNC 와이어 배열은 MAX_CHANNELS 와 정확히 일치해야 함 (오버런/언더런 방지).
static_assert(sizeof(((PktChannelSync*)0)->ch)/sizeof(ChSyncEntry) == MAX_CHANNELS,
              "PktChannelSync::ch[] size must equal MAX_CHANNELS");
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <tuple>
#include <algorithm>
#include <chrono>
#include <thread>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>        // getenv/atoi
#include <opus/opus.h>    // 오디오 Opus 인코더
#include <zstd.h>         // FFT uint8 무손실 추가압축

extern void bewe_log_push(int col, const char* fmt, ...);

// ── 압축 노브 (env 킬스위치) ────────────────────────────────────────────────
// BEWE_OPUS=0 → 오디오 raw float32 (구동작). 기본 ON.
// BEWE_OPUS_BR=<bps> → Opus 비트레이트 (기본 48000, 클램프 6k~256k).
// BEWE_FFT_ZSTD=0 → FFT uint8 그대로 (zstd 미적용). 기본 ON.
static constexpr int OPUS_SR    = 48000;
static constexpr int OPUS_FRAME = 960;   // 20ms @48kHz mono (Opus 합법 프레임)
static bool audio_opus_enabled(){ const char* e=getenv("BEWE_OPUS");     return !(e && e[0]=='0'); }
static bool fft_zstd_enabled(){   const char* e=getenv("BEWE_FFT_ZSTD"); return !(e && e[0]=='0'); }
// 6bit 팩: 기본 OFF. 1.6dB/step 이라 워터폴에서는 안 보이지만 파워스펙트럼(선 그래프)
// 에서는 계단이 눈에 띈다 — 실기에서 화질 저하가 체감돼 되돌렸다 (v13.3.1).
// 대역폭이 급한 링크에서만 BEWE_FFT_U6=1 로 켠다.
static bool fft_u6_enabled(){     const char* e=getenv("BEWE_FFT_U6");   return (e && e[0]=='1'); }
static bool chsync_zstd_enabled(){const char* e=getenv("BEWE_CHSYNC_ZSTD"); return !(e && e[0]=='0'); }

// CHANNEL_SYNC 패킷 빌드. zstd_on 이면 body(ChSyncEntry×50) 를 압축.
// 수신측 감지 = PktHdr.len(=body 크기): ==sizeof(PktChannelSync) → raw, else → zstd 해제.
// 빈슬롯(비활성 35개)이 0이라 압축이 "활성만 전송" 효과 + 활성 엔트리도 무손실 압축.
static std::vector<uint8_t> build_chsync_pkt(const PktChannelSync& sync, bool zstd_on){
    const uint32_t raw = (uint32_t)sizeof(PktChannelSync);
    const uint8_t* body = reinterpret_cast<const uint8_t*>(&sync);
    uint32_t body_len = raw;
    static thread_local std::vector<uint8_t> comp;
    if(zstd_on){
        comp.resize(ZSTD_compressBound(raw));
        size_t z = ZSTD_compress(comp.data(), comp.size(), &sync, raw, 1);
        if(!ZSTD_isError(z) && z < raw){ body = comp.data(); body_len = (uint32_t)z; }
    }
    std::vector<uint8_t> pkt(PKT_HDR_SIZE + body_len);
    PktHdr* ph = reinterpret_cast<PktHdr*>(pkt.data());
    memcpy(ph->magic, BEWE_MAGIC, 4);
    ph->type = static_cast<uint8_t>(PacketType::CHANNEL_SYNC);
    ph->len  = body_len;
    memcpy(pkt.data() + PKT_HDR_SIZE, body, body_len);
    return pkt;
}
static int  opus_bitrate(){
    const char* e=getenv("BEWE_OPUS_BR"); int b = e ? atoi(e) : 48000;
    if(b < 6000)   b = 6000;
    if(b > 256000) b = 256000;
    return b;
}

// IQ 파일 전송 속도: TCP send 실측 기반 적응형.
// send_all이 blocking이므로 네트워크 병목(HOST 업로드, JOIN 다운로드)에 자동 적응.
// FFT 스트림 보호를 위해 실측 속도의 90%만 사용.
static constexpr uint64_t FILE_RATE_FLOOR = 1 * 1024 * 1024;   // 최솟값 1 MB/s
static constexpr uint64_t FILE_RATE_INIT  = 5 * 1024 * 1024;   // 초기값 5 MB/s (보수적 시작, 빠르게 수렴)
static constexpr double   FILE_RATE_EWMA_ALPHA = 0.4;           // EWMA 빠른 수렴

// ── start / stop ──────────────────────────────────────────────────────────
bool NetServer::start(int port){
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd_ < 0){ perror("socket"); return false; }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    // 루프백 전용. JOIN 은 이 TCP 리스너로 붙지 않는다 — Central 경유가 유일 경로이고
    // (central_client.cpp:425) 그쪽은 AF_UNIX socketpair 를 inject_fd() 로 주입한다.
    // NetClient 에는 host/port 로 connect 하는 메서드 자체가 없다(connect_fd(int fd) 뿐).
    // INADDR_ANY 였을 때 3기지가 무인증 제어 포트를 LAN·공인망에 열어두고 있었다.
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = htons((uint16_t)port);

    if(bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0){
        perror("bind"); close(server_fd_); server_fd_=-1; return false;
    }
    // port==0 이면 OS가 할당한 실제 포트를 가져옴
    {
        sockaddr_in bound{};
        socklen_t blen = sizeof(bound);
        if(getsockname(server_fd_, (sockaddr*)&bound, &blen) == 0)
            listen_port_ = ntohs(bound.sin_port);
        else
            listen_port_ = port;
    }
    if(listen(server_fd_, 8) < 0){
        perror("listen"); close(server_fd_); server_fd_=-1; return false;
    }

    running_.store(true);
    accept_thr_ = std::thread(&NetServer::accept_loop, this);
    bewe_log_push(0, "[NetServer] listening on port %d\n", listen_port_);
    return true;
}

void NetServer::stop(){
    running_.store(false);
    if(server_fd_ >= 0){ shutdown(server_fd_, SHUT_RDWR); close(server_fd_); server_fd_=-1; }
    if(accept_thr_.joinable()) accept_thr_.join();

    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        for(auto& c : clients_){
            c->alive.store(false);
            c->stop_send_worker();
            if(c->fd >= 0){ shutdown(c->fd, SHUT_RDWR); close(c->fd); c->fd=-1; }
            if(c->thr.joinable()) c->thr.join();
        }
        clients_.clear();
    }
    // 반드시 clients_mtx_ 밖에서 — send_audio(opus) 는 audio_enc_mtx_→clients_mtx_ 순으로
    // 잠그므로, 여기서 clients_mtx_ 잡은 채 audio_enc_mtx_ 를 잡으면 락순서 역전(데드락).
    free_audio_encoders();
}

// per-ch Opus 인코더 해제.
// ⚠ 불변조건: 호출 전에 모든 send_audio 생산자(FM dem_worker / DMR 워커)가 정지돼 있어야
// 한다 — 이들은 NetServer 소유가 아니라 FFTViewer 파이프라인(stop_all_dem/on_ch_stop)이
// 정지시킨다. CLI 종료는 cli_host 가 stop_all_dem() → net_srv->stop() 순서라 만족.
// 뮤텍스는 직렬화만 보장하고 '정지'는 보장 못 하므로, 생산자가 살아있는 채로 부르면 UAF.
void NetServer::free_audio_encoders(){
    for(int i = 0; i < MAX_CHANNELS; i++){
        std::lock_guard<std::mutex> elk(audio_enc_mtx_[i]);
        if(audio_enc_[i]){ opus_encoder_destroy((OpusEncoder*)audio_enc_[i]); audio_enc_[i] = nullptr; }
        audio_acc_[i].clear();
    }
}

// 채널 종료/모드전환 시 per-ch Opus 인코더 상태 + 누적버퍼 리셋 (stale 잔여/warble 방지).
void NetServer::reset_audio_ch(uint8_t ch_idx){
    if(ch_idx >= MAX_CHANNELS) return;
    std::lock_guard<std::mutex> elk(audio_enc_mtx_[ch_idx]);
    audio_acc_[ch_idx].clear();
    if(audio_enc_[ch_idx])
        opus_encoder_ctl((OpusEncoder*)audio_enc_[ch_idx], OPUS_RESET_STATE);
}


int NetServer::client_count() const {
    std::lock_guard<std::mutex> lk(clients_mtx_);
    int n = 0;
    for(auto& c : clients_) if(c->authed && c->alive.load()) ++n;
    return n;
}

// ── Accept loop ───────────────────────────────────────────────────────────
void NetServer::accept_loop(){
    while(running_.load()){
        sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
        int cfd = accept(server_fd_, (sockaddr*)&caddr, &clen);
        if(cfd < 0){
            if(running_.load()) perror("accept");
            break;
        }
        // TCP_NODELAY: Nagle 알고리즘 비활성화 > FFT 스트림 지연 방지
        int nd=1; setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
        // set TCP keepalive
        int ka=1; setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        // SO_SNDTIMEO: 느린 클라이언트로 인한 send 블로킹 방지
        timeval stv{2, 0};
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

        auto conn = std::make_shared<ClientConn>();
        conn->fd = cfd;
        conn->alive.store(true);
        conn->start_send_worker();

        {
            std::lock_guard<std::mutex> lk(clients_mtx_);
            clients_.push_back(conn);
        }
        conn->thr = std::thread(&NetServer::client_loop, this, conn);
        conn->thr.detach();
    }
}

// ── Inject fd (relay MUX 모드) ────────────────────────────────────────────
void NetServer::inject_fd(int fd){
    auto conn = std::make_shared<ClientConn>();
    conn->fd = fd;
    conn->is_relay = true;
    conn->alive.store(true);
    conn->start_send_worker();
    relay_client_count_.fetch_add(1);
    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        clients_.push_back(conn);
    }
    conn->thr = std::thread(&NetServer::client_loop, this, conn);
    conn->thr.detach();
}

// ── Client loop ───────────────────────────────────────────────────────────
void NetServer::client_loop(std::shared_ptr<ClientConn> c){
    uint64_t pkt_count = 0;
    std::vector<uint8_t> payload;  // 패킷당 재할당 방지 — 루프 밖에서 재사용
    while(c->alive.load()){
        PktHdr hdr{};
        if(!recv_all(c->fd, &hdr, PKT_HDR_SIZE)){
            int e = errno;
            if(c->alive.load())  // 정상 stop이면 로그 생략
                bewe_log_push(0, "[NetServer] recv hdr failed op=%d('%s') fd=%d errno=%d(%s) pkts=%llu\n",
                       c->op_index, c->name, c->fd, e, strerror(e), (unsigned long long)pkt_count);
            break;
        }
        if(memcmp(hdr.magic, BEWE_MAGIC, 4) != 0){
            bewe_log_push(0, "[NetServer] bad magic from op=%d('%s') fd=%d (type=0x%02x)\n",
                   c->op_index, c->name, c->fd, hdr.type);
            break;
        }
        uint32_t len = hdr.len;
        if(len > 1024*1024){
            bewe_log_push(0, "[NetServer] oversized pkt op=%d type=0x%02x len=%u\n",
                   c->op_index, (uint8_t)hdr.type, len);
            break;
        }
        payload.resize(len);
        if(len > 0 && !recv_all(c->fd, payload.data(), len)){
            int e = errno;
            bewe_log_push(0, "[NetServer] recv payload failed op=%d type=0x%02x len=%u errno=%d(%s)\n",
                   c->op_index, (uint8_t)hdr.type, len, e, strerror(e));
            break;
        }
        pkt_count++;
        stat_rx_bytes_.fetch_add(PKT_HDR_SIZE + len, std::memory_order_relaxed);
        handle_packet(c, static_cast<PacketType>(hdr.type),
                      payload.data(), len);
    }
    drop_client(c);
}

// ── Packet handler ────────────────────────────────────────────────────────
void NetServer::handle_packet(std::shared_ptr<ClientConn> c,
                               PacketType type,
                               const uint8_t* payload, uint32_t len){
    switch(type){

    case PacketType::AUTH_REQ: {
        if(len < sizeof(PktAuthReq)) break;
        auto* req = reinterpret_cast<const PktAuthReq*>(payload);
        PktAuthAck ack{};
        uint8_t idx = 0;
        if(cb.on_auth && cb.on_auth(req->id, req->pw, req->tier, idx)){
            ack.ok        = 1;
            ack.op_index  = idx;
            c->op_index   = idx;
            c->tier       = req->tier;
            strncpy(c->name, req->id, 31);
            c->authed     = true;
            strncpy(ack.reason, "OK", sizeof(ack.reason));
            bewe_log_push(0, "[NetServer] op %d '%s' (Tier%d) connected\n",
                   idx, c->name, c->tier);
        } else {
            ack.ok = 0;
            strncpy(ack.reason, "Auth failed", sizeof(ack.reason));
        }
        bewe_log_push(0, "[NetServer] AUTH_ACK sending op=%d ok=%u is_relay=%d fd=%d\n",
               idx, ack.ok, (int)c->is_relay, c->fd);
        send_to(*c, PacketType::AUTH_ACK, &ack, sizeof(ack));
        bewe_log_push(0, "[NetServer] AUTH_ACK sent op=%d ok=%u\n", idx, ack.ok);
        if(ack.ok){
            broadcast_operator_list();
            // 신규 JOIN 이 CHANNEL_SYNC 해시 게이트에 걸려 최대 1초 기다리지 않도록
            // 다음 주기 틱(≤100ms)에 전체 테이블 강제 송신
            chsync_force_.store(true, std::memory_order_relaxed);
            fftmeta_force_.store(true, std::memory_order_relaxed);  // FFT 메타도 즉시 재송신
        }
        break;
    }

    case PacketType::CMD: {
        if(!c->authed || len < sizeof(PktCmd)) break;
        auto* cmd = reinterpret_cast<const PktCmd*>(payload);
        switch(static_cast<CmdType>(cmd->cmd)){
            case CmdType::SET_FREQ:
                if(cb.on_set_freq) cb.on_set_freq(c->name, cmd->set_freq.cf_mhz);
                break;
            case CmdType::SET_GAIN:
                if(cb.on_set_gain) cb.on_set_gain(c->name, cmd->set_gain.db);
                break;
            case CmdType::CREATE_CH:
                if(cb.on_create_ch)
                    cb.on_create_ch(cmd->create_ch.idx, cmd->create_ch.s, cmd->create_ch.e, c->name);
                break;
            case CmdType::DELETE_CH:
                if(cb.on_delete_ch) cb.on_delete_ch(c->name, cmd->delete_ch.idx);
                break;
            case CmdType::SET_CH_MODE:
                if(cb.on_set_ch_mode)
                    cb.on_set_ch_mode(c->name, cmd->set_ch_mode.idx, cmd->set_ch_mode.mode);
                break;
            case CmdType::SET_CH_AUDIO:
                if(cb.on_set_ch_audio)
                    cb.on_set_ch_audio(cmd->set_ch_audio.idx, cmd->set_ch_audio.mask);
                break;
            case CmdType::START_REC:
                if(cb.on_start_rec) cb.on_start_rec(cmd->start_rec.ch_idx);
                break;
            case CmdType::STOP_REC:
                if(cb.on_stop_rec) cb.on_stop_rec();
                break;
            case CmdType::SET_CH_PAN:
                if(cb.on_set_ch_pan)
                    cb.on_set_ch_pan(cmd->set_ch_pan.idx, cmd->set_ch_pan.pan);
                break;
            case CmdType::SET_SQ_THRESH:
                if(cb.on_set_sq_thresh)
                    cb.on_set_sq_thresh(cmd->set_sq_thresh.idx, cmd->set_sq_thresh.thr);
                break;
            case CmdType::SET_AUTOSCALE:
                if(cb.on_set_autoscale) cb.on_set_autoscale();
                break;
            case CmdType::DF_SET_SNR:
                if(cb.on_df_set_snr) cb.on_df_set_snr((int)cmd->df_set_snr.snr_db);
                break;
            case CmdType::DF_MEASURE:
                // origin 은 PktCmd 의 고정 크기 유니온 안이라 구 JOIN 도 0
                // (= 수동) 으로 채워 보낸다. 별도 len 게이트가 필요 없다.
                if(cb.on_df_measure)
                    cb.on_df_measure((int)cmd->df_measure.dnum,
                                     cmd->df_measure.origin != 0);
                break;
            case CmdType::SET_CH_DETECT:
                if(cb.on_set_ch_detect)
                    cb.on_set_ch_detect(cmd->set_ch_detect.idx,
                                        cmd->set_ch_detect.enable != 0);
                break;
            case CmdType::TOGGLE_RECV:
                if(cb.on_toggle_recv)
                    cb.on_toggle_recv(cmd->toggle_recv.idx, c->op_index,
                                      cmd->toggle_recv.enable != 0);
                break;
            case CmdType::UPDATE_CH_RANGE:
                if(cb.on_update_ch_range)
                    cb.on_update_ch_range(cmd->update_ch_range.idx,
                                          cmd->update_ch_range.s,
                                          cmd->update_ch_range.e);
                break;
            case CmdType::TOGGLE_TM_IQ:
                if(cb.on_toggle_tm_iq) cb.on_toggle_tm_iq();
                break;
            case CmdType::SET_CAPTURE_PAUSE:
                if(cb.on_set_capture_pause)
                    cb.on_set_capture_pause(cmd->set_capture_pause.pause != 0);
                break;
            case CmdType::SET_SPECTRUM_PAUSE:
                if(cb.on_set_spectrum_pause)
                    cb.on_set_spectrum_pause(cmd->set_spectrum_pause.pause != 0);
                break;
            case CmdType::REQUEST_REGION:
                if(cb.on_request_region)
                    cb.on_request_region(c->op_index, c->name,
                        cmd->request_region.fft_top, cmd->request_region.fft_bot,
                        cmd->request_region.freq_lo, cmd->request_region.freq_hi,
                        cmd->request_region.time_start_ms, cmd->request_region.time_end_ms,
                        cmd->request_region.samp_start, cmd->request_region.samp_end);
                break;
            case CmdType::CHASSIS_RESET:
                if(cb.on_chassis_reset) cb.on_chassis_reset(c->name);
                break;
            case CmdType::NET_RESET:
                if(cb.on_net_reset) cb.on_net_reset(c->name);
                break;
            case CmdType::RX_STOP:
                if(cb.on_rx_stop) cb.on_rx_stop(c->name);
                break;
            case CmdType::RX_START:
                if(cb.on_rx_start) cb.on_rx_start(c->name);
                break;
            case CmdType::START_IQ_REC:
                if(cb.on_start_iq_rec) cb.on_start_iq_rec(c->op_index, c->name, cmd->start_iq_rec.idx);
                break;
            case CmdType::STOP_IQ_REC:
                if(cb.on_stop_iq_rec) cb.on_stop_iq_rec(c->op_index, c->name, cmd->stop_iq_rec.idx);
                break;
            case CmdType::SET_FFT_SIZE:
                if(cb.on_set_fft_size) cb.on_set_fft_size(c->name, cmd->set_fft_size.size);
                break;
            case CmdType::SET_SR:
                if(cb.on_set_sr) cb.on_set_sr(c->name, cmd->set_sr.msps);
                break;
            case CmdType::SET_ANTENNA:
                if(cb.on_set_antenna) cb.on_set_antenna(c->name, cmd->set_antenna.antenna);
                break;
            case CmdType::ADD_SCHED:
                if(cb.on_add_sched) cb.on_add_sched(c->op_index, c->name,
                    cmd->add_sched.start_time, cmd->add_sched.duration_sec,
                    cmd->add_sched.freq_mhz, cmd->add_sched.bw_khz,
                    cmd->add_sched.target);
                break;
            case CmdType::REMOVE_SCHED:
                if(cb.on_remove_sched) cb.on_remove_sched(c->op_index, c->name,
                    cmd->remove_sched.start_time, cmd->remove_sched.freq_mhz);
                break;
            case CmdType::SET_HW:
                if(cb.on_set_hw) cb.on_set_hw(c->name, cmd->set_hw.name);
                break;
            default: break;
        }
        // ACK
        PktCmdAck ack{}; ack.ok=1; ack.cmd=cmd->cmd;
        strncpy(ack.msg,"OK",sizeof(ack.msg));
        send_to(*c, PacketType::CMD_ACK, &ack, sizeof(ack));
        break;
    }

    case PacketType::DB_SAVE_META: {
        if(!c->authed || len < sizeof(PktDbSaveMeta)) break;
        auto* meta = reinterpret_cast<const PktDbSaveMeta*>(payload);
        if(cb.on_db_save) cb.on_db_save(c->op_index, c->name, meta, payload + sizeof(PktDbSaveMeta), len - (uint32_t)sizeof(PktDbSaveMeta));
        break;
    }

    case PacketType::DB_SAVE_DATA: {
        if(!c->authed || len < sizeof(PktDbSaveData)) break;
        auto* d = reinterpret_cast<const PktDbSaveData*>(payload);
        uint32_t data_bytes = d->chunk_bytes;
        if(len < sizeof(PktDbSaveData)+data_bytes) break;
        if(cb.on_db_save) cb.on_db_save(c->op_index, c->name, nullptr, payload, len);
        break;
    }

    case PacketType::CHAT: {
        if(!c->authed || len < sizeof(PktChat)) break;
        auto* chat = reinterpret_cast<const PktChat*>(payload);
        // override 'from' with actual connected name
        PktChat out{}; strncpy(out.from, c->name, 31);
        strncpy(out.msg, chat->msg, sizeof(out.msg)-1);
        if(cb.on_chat) cb.on_chat(out.from, out.msg);
        // broadcast to all (including server UI)
        broadcast_chat(out.from, out.msg);
        break;
    }

    case PacketType::DISCONNECT:
        c->alive.store(false);
        break;

    case PacketType::DB_DELETE_REQ: {
        if(!c->authed || len < sizeof(PktDbDeleteReq)) break;
        auto* req = reinterpret_cast<const PktDbDeleteReq*>(payload);
        if(cb.on_db_delete) cb.on_db_delete(c->name, req->filename, req->operator_name);
        break;
    }

    case PacketType::DB_DOWNLOAD_REQ: {
        if(!c->authed || len < sizeof(PktDbDownloadReq)) break;
        auto* req = reinterpret_cast<const PktDbDownloadReq*>(payload);
        if(cb.on_db_download_req) cb.on_db_download_req(c->op_index, c->name, req->filename, req->operator_name);
        break;
    }

    // ── Band plan: any client (LAN-direct or relay-injected) → host ──────
    case PacketType::DF_CONFIG: {
        if(!c->authed || len < sizeof(PktDfConfig)) break;
        if(cb.on_df_set_config) cb.on_df_set_config(*reinterpret_cast<const PktDfConfig*>(payload));
        break;
    }
    case PacketType::BAND_ADD: {
        if(!c->authed || len < sizeof(PktBandEntry)) break;
        if(cb.on_band_add) cb.on_band_add(*reinterpret_cast<const PktBandEntry*>(payload));
        break;
    }
    case PacketType::BAND_UPDATE: {
        if(!c->authed || len < sizeof(PktBandEntry)) break;
        if(cb.on_band_update) cb.on_band_update(*reinterpret_cast<const PktBandEntry*>(payload));
        break;
    }
    case PacketType::BAND_REMOVE: {
        if(!c->authed || len < sizeof(PktBandRemove)) break;
        if(cb.on_band_remove) cb.on_band_remove(*reinterpret_cast<const PktBandRemove*>(payload));
        break;
    }

    case PacketType::BAND_CAT_UPSERT: {
        if(!c->authed || len < sizeof(PktBandCategory)) break;
        if(cb.on_band_cat_upsert)
            cb.on_band_cat_upsert(*reinterpret_cast<const PktBandCategory*>(payload));
        break;
    }
    case PacketType::BAND_CAT_DELETE: {
        if(!c->authed || len < sizeof(PktBandCatDelete)) break;
        auto* d = reinterpret_cast<const PktBandCatDelete*>(payload);
        if(cb.on_band_cat_delete) cb.on_band_cat_delete(d->id);
        break;
    }

    case PacketType::LWF_LIST_REQ: {
        if(!c->authed) break;
        if(cb.on_lwf_list_req) cb.on_lwf_list_req(c->op_index, c->name);
        break;
    }
    case PacketType::LWF_DL_REQ: {
        if(!c->authed || len < sizeof(PktLwfDlReq)) break;
        auto* req = reinterpret_cast<const PktLwfDlReq*>(payload);
        if(cb.on_lwf_dl_req) cb.on_lwf_dl_req(c->op_index, c->name, req->filename);
        break;
    }
    // LWF_LIVE_REQ (v4.6.0 제거): JOIN STREAM opt-in 기능 폐기. 구버전 JOIN이 보내도 silently drop.
    case PacketType::LWF_DELETE_REQ: {
        if(!c->authed || len < sizeof(PktLwfDlReq)) break;
        auto* req = reinterpret_cast<const PktLwfDlReq*>(payload);
        if(cb.on_lwf_delete_req) cb.on_lwf_delete_req(c->op_index, c->name, req->filename);
        break;
    }

    case PacketType::MISSION_START: {
        printf("[NetServer] MISSION_START recv: op=%u name='%s' authed=%d cb=%d\n",
               c->op_index, c->name, (int)c->authed, cb.on_mission_start ? 1 : 0);
        if(!c->authed) break;
        (void)payload; (void)len;   // payload는 op_index padding 뿐 — 호스트 컨텍스트로 직접 시작
        if(cb.on_mission_start) cb.on_mission_start(c->op_index, c->name);
        break;
    }
    case PacketType::MISSION_END: {
        if(!c->authed) break;
        if(cb.on_mission_end) cb.on_mission_end(c->op_index, c->name);
        break;
    }
    case PacketType::MISSION_LIST_REQ: {
        if(!c->authed) break;
        if(cb.on_mission_list_req) cb.on_mission_list_req(c->op_index, c->name);
        break;
    }
    case PacketType::MISSION_DELETE: {
        if(!c->authed || len < sizeof(PktMissionDelete)) break;
        const auto* req = reinterpret_cast<const PktMissionDelete*>(payload);
        if(cb.on_mission_delete) cb.on_mission_delete(c->op_index, c->name, *req);
        break;
    }
    // MISSION_UPDATE 제거 — 자동 캡처 모델에서 운영자 편집 필드 없음.

    case PacketType::MODULE_PIPE: {
        if(!c->authed || len < sizeof(PktModulePipe)) break;
        if(cb.on_module_pipe) cb.on_module_pipe(payload, len);
        break;
    }

    default: break;
    }
}

// ── drop_client ───────────────────────────────────────────────────────────
void NetServer::drop_client(std::shared_ptr<ClientConn> c){
    if(c->is_relay) relay_client_count_.fetch_sub(1);
    bool was_authed = c->authed;
    uint8_t idx = c->op_index;
    char name[32]; strncpy(name, c->name, 31);

    bewe_log_push(0, "[NetServer] drop_client op=%d '%s' fd=%d authed=%d is_relay=%d\n",
           idx, name, c->fd, (int)was_authed, (int)c->is_relay);

    c->alive.store(false);
    c->stop_send_worker();
    if(c->fd >= 0){ shutdown(c->fd, SHUT_RDWR); close(c->fd); c->fd=-1; }

    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        clients_.erase(std::remove_if(clients_.begin(), clients_.end(),
            [&](const std::shared_ptr<ClientConn>& x){ return x.get()==c.get(); }),
            clients_.end());
    }

    if(was_authed){
        bewe_log_push(0, "[NetServer] op %d '%s' disconnected\n", idx, name);
        broadcast_operator_list();
    }
}

// ── send_to ───────────────────────────────────────────────────────────────
void NetServer::send_to(ClientConn& c, PacketType type,
                         const void* payload, uint32_t len){
    if(!c.alive.load() || c.fd < 0) return;
    c.enqueue(make_packet(type, payload, len), false);
}

// ── Broadcast FFT ─────────────────────────────────────────────────────────
void NetServer::broadcast_fft(const float* data, int fft_size,
                               int64_t wall_time,
                               uint64_t center_hz, uint32_t sr,
                               float pmin, float pmax,
                               int64_t iq_write_sample, int64_t iq_total_samples){
    if(bcast_pause_.load(std::memory_order_relaxed)) return;
    // uint8 quantize 로 4배 압축 (dB [pmin..pmax] → 0..255, ~0.4dB 해상도, 시각손실 거의0).
    // v12: 그 위에 zstd 무손실 추가압축 옵션 (bit30 FFT_FLAG_ZSTD). 노이즈플로어 평탄+
    // 신호 sparse 라 통상 2~3배 더 줄어듦. 실패/증가 시 raw uint8 로 폴백.
    // ① 양자화본을 스크래치 버퍼에 생성
    static thread_local std::vector<uint8_t> qbuf;
    qbuf.resize(fft_size);
    float range = pmax - pmin;
    if(!(range > 0.f)) range = 1.f;  // 안전망
    float inv = 255.f / range;
    for(int i = 0; i < fft_size; i++){
        float v = (data[i] - pmin) * inv;
        if(v < 0.f) v = 0.f;
        if(v > 255.f) v = 255.f;
        qbuf[i] = (uint8_t)v;
    }
    // ②-a 6bit 팩 (v13.2): zstd 입력을 8bit→6bit 로 줄인다. 1.6dB/step, 실측
    //     오차 평균 0.59dB. zstd 압축비가 2.22x→3.76x 로 올라 전송량 ~41% 감소.
    static const bool u6_on   = fft_u6_enabled();
    static const bool zstd_on = fft_zstd_enabled();
    const uint8_t* src     = qbuf.data();     // zstd 에 넣을 원본
    size_t         src_len = (size_t)fft_size;
    static thread_local std::vector<uint8_t> u6buf;
    if(u6_on){
        u6buf.resize(u6_packed_bytes(fft_size));
        u6_pack(qbuf.data(), (size_t)fft_size, u6buf.data());
        src = u6buf.data(); src_len = u6buf.size();
    }
    // ② 패킷 빌드 (payload = zstd 압축본 또는 raw)
    static thread_local std::vector<uint8_t> pkt;
    size_t cap = zstd_on ? ZSTD_compressBound(src_len) : src_len;
    pkt.resize(PKT_HDR_SIZE + sizeof(PktFftFrame) + cap);   // 워스트케이스 확보
    uint8_t* dst = pkt.data() + PKT_HDR_SIZE + sizeof(PktFftFrame);
    uint32_t flags = FFT_FLAG_QUANT_U8 | (u6_on ? FFT_FLAG_QUANT_U6 : 0u);
    uint32_t data_bytes;
    if(zstd_on){
        size_t z = ZSTD_compress(dst, cap, src, src_len, 1);
        if(!ZSTD_isError(z) && z < src_len){            // 실제로 줄었을 때만 채택
            data_bytes = (uint32_t)z; flags |= FFT_FLAG_ZSTD;
        } else {                                        // 압축 실패/무이득 → 팩본 그대로
            memcpy(dst, src, src_len); data_bytes = (uint32_t)src_len;
        }
    } else {
        memcpy(dst, src, src_len); data_bytes = (uint32_t)src_len;
    }
    uint32_t total = (uint32_t)(sizeof(PktFftFrame) + data_bytes);
    pkt.resize(PKT_HDR_SIZE + total);   // 축소만 → 재할당 없음, dst 데이터 유지
    PktHdr* ph = reinterpret_cast<PktHdr*>(pkt.data());
    memcpy(ph->magic, BEWE_MAGIC, 4);
    ph->type = static_cast<uint8_t>(PacketType::FFT_FRAME);
    ph->len  = total;
    PktFftFrame hdr{};
    hdr.center_freq_hz = center_hz;
    hdr.sample_rate    = sr;
    hdr.fft_size       = (uint32_t)fft_size | flags;
    hdr.power_min      = pmin;
    hdr.power_max      = pmax;
    hdr.wall_time      = wall_time;
    hdr.iq_write_sample  = iq_write_sample;
    hdr.iq_total_samples = iq_total_samples;
    memcpy(pkt.data() + PKT_HDR_SIZE, &hdr, sizeof(PktFftFrame));
    if(cb.on_relay_broadcast){
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
    }
}

// ── 완성된 오디오 패킷 1개 방출 (raw/opus 공용) ───────────────────────────
// body = raw float32 PCM 바이트 또는 opus 프레임 바이트. n_field = PktAudioFrame.n_samples
// (opus 면 AUDIO_FLAG_OPUS | 디코드샘플수). op_mask 매칭 클라이언트 + relay 로 전송.
void NetServer::emit_audio(uint32_t op_mask, uint8_t ch_idx, int8_t pan,
                           const uint8_t* body, uint32_t body_len, uint32_t n_field){
    uint32_t payload_size = (uint32_t)(sizeof(PktAudioFrame) + body_len);
    static thread_local std::vector<uint8_t> pkt;
    pkt.resize(PKT_HDR_SIZE + payload_size);
    PktHdr* ph = reinterpret_cast<PktHdr*>(pkt.data());
    memcpy(ph->magic, BEWE_MAGIC, 4);
    ph->type = static_cast<uint8_t>(PacketType::AUDIO_FRAME);
    ph->len  = payload_size;
    auto* ah = reinterpret_cast<PktAudioFrame*>(pkt.data() + PKT_HDR_SIZE);
    ah->ch_idx    = ch_idx;
    ah->pan       = (uint8_t)(int8_t)pan;
    ah->n_samples = n_field;
    memcpy(pkt.data() + PKT_HDR_SIZE + sizeof(PktAudioFrame), body, body_len);

    // relay JOIN이 있을 때만 중앙서버로 전송 (없으면 큐 낭비 방지)
    if(cb.on_relay_broadcast && has_relay())
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);

}

// ── Send audio to specific operators (legacy, mask-based) ────────────────
// v12: 기본적으로 per-ch_idx Opus 인코딩 (48kHz mono, 20ms 프레임). 콜러는 256샘플씩
// 넘기고, 여기서 960샘플로 리버퍼해 Opus 인코딩 → 오디오 대역 ~20배 절감. BEWE_OPUS=0
// 이면 raw float32 (구동작). DMR 경로는 ~48.3kHz 라 미세 피치드리프트가 있으나 기존
// raw 경로도 JOIN 이 48k 로 재생하던 것과 동일 (Opus 가 악화시키지 않음).
void NetServer::send_audio(uint32_t op_mask, uint8_t ch_idx, int8_t pan,
                            const float* pcm, uint32_t n_samples){
    if(!op_mask || !n_samples) return;
    if(bcast_pause_.load(std::memory_order_relaxed)) return;

    static const bool opus_on = audio_opus_enabled();
    if(opus_on && ch_idx < MAX_CHANNELS){
        std::lock_guard<std::mutex> elk(audio_enc_mtx_[ch_idx]);
        OpusEncoder* enc = (OpusEncoder*)audio_enc_[ch_idx];
        if(!enc){
            int err = OPUS_OK;
            enc = opus_encoder_create(OPUS_SR, 1, OPUS_APPLICATION_AUDIO, &err);
            if(err == OPUS_OK && enc){
                opus_encoder_ctl(enc, OPUS_SET_BITRATE(opus_bitrate()));
                audio_enc_[ch_idx] = enc;
            } else {
                if(enc){ opus_encoder_destroy(enc); enc = nullptr; }
            }
        }
        if(enc){
            auto& acc = audio_acc_[ch_idx];
            acc.insert(acc.end(), pcm, pcm + n_samples);
            unsigned char obuf[4000];
            size_t off = 0;
            while(acc.size() - off >= (size_t)OPUS_FRAME){
                int nb = opus_encode_float(enc, acc.data() + off, OPUS_FRAME,
                                           obuf, (opus_int32)sizeof(obuf));
                off += OPUS_FRAME;
                if(nb > 0)
                    emit_audio(op_mask, ch_idx, pan, obuf, (uint32_t)nb,
                               (uint32_t)OPUS_FRAME | AUDIO_FLAG_OPUS);
                // nb<=0: 인코드 실패 프레임은 스킵 (해당 20ms 무음 처리)
            }
            if(off) acc.erase(acc.begin(), acc.begin() + off);
            return;
        }
        // 인코더 생성 실패 → raw 폴백
    }
    // raw float32 PCM (구동작 / BEWE_OPUS=0 / 인코더 실패)
    emit_audio(op_mask, ch_idx, pan, (const uint8_t*)pcm,
               n_samples * (uint32_t)sizeof(float), n_samples);
}



// ── Broadcast FFT meta (입력 크기) ────────────────────────────────────────
void NetServer::broadcast_fft_meta(int fft_size, int fft_input_size){
    if(bcast_pause_.load(std::memory_order_relaxed)) return;
    PktFftMeta m{};
    m.fft_input_size = (uint32_t)fft_input_size;
    m.fft_size       = (uint32_t)fft_size;
    auto pkt = make_packet(PacketType::FFT_META, &m, sizeof(m));
    if(cb.on_relay_broadcast && has_relay())
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

// ── Broadcast channel sync ────────────────────────────────────────────────
void NetServer::broadcast_channel_sync(const Channel* chs, int n, bool periodic){
    // 주의: 무구독(JOIN 0)이라도 early-return 금지. 이 함수는 Central 로도 relay 되어
    // 타 기지 demod 통합목록(cached_ch_sync 기반 CH_LIST)을 채운다. JOIN 0 기지가 여기서
    // 빠지면 그 기지 채널이 다른 기지 목록에서 사라짐.
    // periodic=true(메인루프 10Hz)만 아래 해시/relay 감속 게이트를 타고,
    // 이벤트성 호출(채널 op)은 항상 즉시 송신 — Central 캐시에 바로 반영돼야 함.
    PktChannelSync sync{};
    for(int i=0; i<n && i<MAX_CHANNELS; i++){
        sync.ch[i].idx        = (uint8_t)i;
        sync.ch[i].active     = chs[i].filter_active ? 1 : 0;
        sync.ch[i].s          = chs[i].s;
        sync.ch[i].e          = chs[i].e;
        sync.ch[i].mode       = (uint8_t)chs[i].mode;
        sync.ch[i].pan        = (int8_t)chs[i].pan;
        sync.ch[i].audio_mask    = chs[i].audio_mask.load();
        sync.ch[i].sq_threshold  = chs[i].sq_threshold.load(std::memory_order_relaxed);
        sync.ch[i].sq_sig        = chs[i].sq_sig.load(std::memory_order_relaxed);
        sync.ch[i].sq_gate       = chs[i].sq_gate.load(std::memory_order_relaxed) ? 1 : 0;
        sync.ch[i].dem_paused    = chs[i].dem_paused.load(std::memory_order_relaxed) ? 1 : 0;
        sync.ch[i].det_state     = !chs[i].det_on.load(std::memory_order_relaxed) ? 0
                                 : (chs[i].det_locked.load(std::memory_order_relaxed) ? 2 : 1);
        strncpy(sync.ch[i].owner_name, chs[i].owner, 31);
        sync.ch[i].iq_rec_secs    = (chs[i].iq_rec_sr > 0) ? (uint32_t)(chs[i].iq_rec_frames / chs[i].iq_rec_sr) : 0;
        sync.ch[i].audio_rec_secs = (chs[i].audio_rec_sr > 0) ? (uint32_t)(chs[i].audio_rec_frames / chs[i].audio_rec_sr) : 0;
        sync.ch[i].iq_rec_on      = chs[i].iq_rec_on.load() ? 1 : 0;
        sync.ch[i].audio_rec_on   = chs[i].audio_rec_on.load() ? 1 : 0;
        // 디코드 통계 — 활성 슬롯만 (비활성 슬롯은 디코더가 없어 항상 0; sync{} zero-init).
        // decstat 는 호출당 락 2회 + 자정 롤오버 검사라 50슬롯×10Hz 는 순수 낭비였음.
        if(sync.ch[i].active){
            uint32_t dc=0,dr=0; bewe_mod_host_ch_decstat(i,dc,dr);
            sync.ch[i].dec_count=dc; sync.ch[i].dec_runtime_s=dr;
        }
    }
    auto now = std::chrono::steady_clock::now();
    if(periodic){
        // 내용 동일 시 1Hz 감속. 활성 채널이 있으면 sq_sig/초 카운터가 매 틱 변해
        // 항상 통과 → 라이브 뷰어 10Hz 유지. (해시 게이트는 유휴 상태 전용)
        uint64_t h = 1469598103934665603ull;                    // FNV-1a 64
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&sync);
        for(size_t k=0;k<sizeof(sync);k++){ h ^= p[k]; h *= 1099511628211ull; }
        bool force = chsync_force_.exchange(false, std::memory_order_relaxed); // 신규 JOIN 즉시 시드
        float since = std::chrono::duration<float>(now - chsync_last_send_).count();
        if(!force && h == chsync_last_hash_ && since < 1.0f) return;           // 1Hz keepalive 하한
        chsync_last_hash_ = h; chsync_last_send_ = now;
    }
    static const bool chsync_zstd = chsync_zstd_enabled();
    auto pkt = build_chsync_pkt(sync, chsync_zstd);
    if(cb.on_relay_broadcast){
        // 원격 JOIN 없으면 주기분 relay 는 1Hz (Central cached_ch_sync/CH_LIST 통계 갱신용).
        bool send_relay = !periodic || has_relay()
            || std::chrono::duration<float>(now - chsync_relay_last_).count() >= 1.0f;
        if(send_relay){
            cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
            if(periodic) chsync_relay_last_ = now;
        }
    }
}

// ── Broadcast scheduled recording list → all clients ────────────────────
void NetServer::broadcast_sched_sync(const PktSchedSync& sync){
    auto pkt = make_packet(PacketType::SCHED_SYNC, &sync, sizeof(sync));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

// ── Broadcast mission snapshot → all clients (LAN + relay) ─────────────
// Central은 이 패킷을 가로채 station 캐시 + missions.json 영속화 (D5).
void NetServer::broadcast_mission_sync(const PktMissionSync& sync){
    auto pkt = make_packet(PacketType::MISSION_SYNC, &sync, sizeof(sync));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true /*no_drop*/);
}

// ── Broadcast module pipe payload (HOST → all JOINs, LAN + relay) ────────
// payload = PktModulePipe + data. 모듈 상태/라이브/파일청크 공용 (no_drop).
void NetServer::broadcast_module_pipe(const void* payload, uint32_t len){
    auto pkt = make_packet(PacketType::MODULE_PIPE, payload, len);
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true /*no_drop*/);
}

// ── Broadcast band plan (HOST → all JOINs, LAN + relay) ──────────────────
// Host owns ~/BEWE/band_plan.json. Whenever state changes, host calls this.
// LAN-direct JOINs receive via per-client enqueue; relay-side JOINs receive
// via on_relay_broadcast (Central fans out to N joins).
void NetServer::broadcast_band_plan(const PktBandPlan& bp){
    auto pkt = make_packet(PacketType::BAND_PLAN_SYNC, &bp, sizeof(bp));
    if(cb.on_relay_broadcast && has_relay())
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true /*no_drop*/);
}

void NetServer::broadcast_band_categories(const PktBandCatSync& cs){
    auto pkt = make_packet(PacketType::BAND_CAT_SYNC, &cs, sizeof(cs));
    if(cb.on_relay_broadcast && has_relay())
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true /*no_drop*/);
}

// ── Broadcast chat ────────────────────────────────────────────────────────
void NetServer::broadcast_chat(const char* from, const char* msg){
    PktChat chat{};
    strncpy(chat.from, from, 31);
    strncpy(chat.msg,  msg,  sizeof(chat.msg)-1);
    auto pkt = make_packet(PacketType::CHAT, &chat, sizeof(chat));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

// ── Broadcast heartbeat ───────────────────────────────────────────────────
void NetServer::broadcast_heartbeat(uint8_t host_state, uint8_t sdr_temp_c, uint8_t sdr_state, uint8_t iq_on,
                                    uint8_t host_cpu_pct, uint8_t host_ram_pct, uint8_t host_cpu_temp_c,
                                    const char* antenna, const char* sdr_kind, uint8_t host_bat_pct,
                                    uint32_t host_up_x100, uint8_t host_bat_ac, uint8_t df_state,
                                    int8_t df_snr_thr){
    PktHeartbeat hb{}; hb.host_state = host_state; hb.sdr_temp_c = sdr_temp_c; hb.sdr_state = sdr_state; hb.iq_on = iq_on;
    hb.host_cpu_pct = host_cpu_pct; hb.host_ram_pct = host_ram_pct; hb.host_cpu_temp_c = host_cpu_temp_c;
    hb.host_bat_pct = host_bat_pct; hb.host_up_x100 = host_up_x100; hb.host_bat_ac = host_bat_ac;
    hb.df_state = df_state; hb.df_snr_thr = df_snr_thr;
    if(antenna)  strncpy(hb.antenna,  antenna,  sizeof(hb.antenna)-1);
    if(sdr_kind) strncpy(hb.sdr_kind, sdr_kind, sizeof(hb.sdr_kind)-1);
    auto pkt = make_packet(PacketType::HEARTBEAT, &hb, sizeof(hb));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true); // no_drop=true — HB 유실은 LINK 끊김
}

// ── Broadcast disk stat ───────────────────────────────────────────────────
void NetServer::broadcast_disk_stat(uint64_t free_bytes, uint64_t total_bytes, const char* station){
    PktDiskStat s{};
    s.source      = 0;   // HOST
    s.free_bytes  = free_bytes;
    s.total_bytes = total_bytes;
    if(station) strncpy(s.station, station, sizeof(s.station)-1);
    auto pkt = make_packet(PacketType::DISK_STAT, &s, sizeof(s));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

// ── Broadcast DF config ───────────────────────────────────────────────────
// HOST 가 적용한 설정을 정본으로 뿌린다. 변경 시 + JOIN 접속 시 보낸다.
void NetServer::broadcast_df_config(const PktDfConfig& c){
    auto pkt = make_packet(PacketType::DF_CONFIG, &c, sizeof(c));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

// 의사스펙트럼은 구조체 뒤에 그대로 이어붙인다. 수신 측은 payload 말미 360 B 로
// 읽으므로, 나중에 PktDfResult 에 필드를 덧붙여도 오프셋이 안 밀린다.
void NetServer::broadcast_df_result(const PktDfResult& r, const uint8_t* spec360){
    std::vector<uint8_t> body(sizeof(r) + (spec360 ? 360 : 0));
    memcpy(body.data(), &r, sizeof(r));
    if(spec360) memcpy(body.data() + sizeof(r), spec360, 360);
    auto pkt = make_packet(PacketType::DF_RESULT, body.data(), body.size());
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

void NetServer::broadcast_df_status(const PktDfStatus& s){
    auto pkt = make_packet(PacketType::DF_STATUS, &s, sizeof(s));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

// ── Broadcast status ──────────────────────────────────────────────────────
void NetServer::broadcast_status(float cf_mhz, float gain_db,
                                  uint32_t sr, uint8_t hw_type){
    PktStatus s{}; s.cf_mhz=cf_mhz; s.gain_db=gain_db;
    s.sample_rate=sr; s.hw_type=hw_type;
    auto pkt = make_packet(PacketType::STATUS, &s, sizeof(s));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

// ── Broadcast operator list ───────────────────────────────────────────────
void NetServer::broadcast_operator_list(){
    PktOperatorList ol{};
    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        int cnt = 0;
        // index=0: HOST 본인
        if(cnt < MAX_OPERATORS){
            ol.ops[cnt].index = 0;
            ol.ops[cnt].tier  = host_tier_;
            strncpy(ol.ops[cnt].name, host_name_, 31);
            ++cnt;
        }
        for(auto& c : clients_){
            // is_relay JOIN 도 포함 — Central 경유 접속이라도 c->name 은 AUTH_REQ 시 login_id 로 set
            if(!c->authed || !c->alive.load()) continue;
            if(cnt >= MAX_OPERATORS) break;
            ol.ops[cnt].index = c->op_index;
            ol.ops[cnt].tier  = c->tier;
            strncpy(ol.ops[cnt].name, c->name, 31);
            ++cnt;
        }
        ol.count = (uint8_t)cnt;
    }
    // relay 경유 JOIN에도 전달: on_relay_broadcast 콜백 사용
    if(cb.on_relay_broadcast){
        auto pkt = make_packet(PacketType::OPERATOR_LIST, &ol, sizeof(ol));
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
    }
}

// ── Get operators (for UI) ────────────────────────────────────────────────
void NetServer::broadcast_wf_event(int32_t fft_offset, int64_t wall_time,
                                    uint8_t type, const char* label){
    PktWfEvent ev{};
    ev.fft_idx_offset = fft_offset;
    ev.wall_time      = wall_time;
    ev.type           = type;
    strncpy(ev.label, label, 31);
    auto pkt = make_packet(PacketType::WF_EVENT, &ev, sizeof(ev));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), false);
}

void NetServer::send_region_response(int op_index, bool allowed){
    PktRegionResponse resp{}; resp.allowed = allowed ? 1 : 0;
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& cli : clients_){
        if(cli->authed && cli->alive.load() && cli->op_index==(uint8_t)op_index){
            cli->enqueue(make_packet(PacketType::REGION_RESPONSE, &resp, sizeof(resp)), false);
            break;
        }
    }
}

void NetServer::send_file_to(int op_index, const char* path, uint8_t transfer_id,
                              std::function<void(uint64_t,uint64_t)> progress_cb){
    FILE* fp = fopen(path, "rb");
    if(!fp){ bewe_log_push(0, "send_file_to: open failed %s\n",path); return; }
    fseek(fp,0,SEEK_END); uint64_t total=(uint64_t)ftell(fp); fseek(fp,0,SEEK_SET);

    // find target client
    std::shared_ptr<ClientConn> target;
    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        for(auto& cli : clients_)
            if(cli->authed && cli->alive.load() && cli->op_index==(uint8_t)op_index)
                { target=cli; break; }
    }
    if(!target){ fclose(fp); return; }

    // send meta
    PktFileMeta meta{};
    const char* fn = strrchr(path,'/'); fn = fn ? fn+1 : path;
    strncpy(meta.filename, fn, 127);
    meta.total_bytes  = total;
    meta.transfer_id  = transfer_id;
    {
        auto pkt = make_packet(PacketType::FILE_META, &meta, sizeof(meta));
        std::lock_guard<std::mutex> slk(target->fd_write_mtx);
        send_all(target->fd, pkt.data(), pkt.size());
    }

    // send chunks: 64KB 청크로 TCP 효율 극대화
    // send_file_to는 detach 스레드에서 실행되므로 캡처·오디오 스레드 차단 없음
    // 속도: 측정 속도의 80% 사용 > FFT 스트림 보호하되 전송 속도 확보
    const uint32_t CHUNK = 256 * 1024;  // 256KB 청크
    std::vector<uint8_t> buf(sizeof(PktFileData)+CHUNK);
    uint64_t offset=0;
    // EWMA로 측정한 실제 TCP send 속도 (bytes/sec)
    double measured_bps = (double)FILE_RATE_INIT;
    auto rate_epoch = std::chrono::steady_clock::now();
    uint64_t rate_sent = 0;
    while(true){
        size_t n = fread(buf.data()+sizeof(PktFileData), 1, CHUNK, fp);
        if(n==0) break;
        PktFileData* d = reinterpret_cast<PktFileData*>(buf.data());
        d->transfer_id  = transfer_id;
        // feof()는 fread가 EOF를 만났을 때만 set — 파일 크기가 CHUNK의 정확한 배수면
        // 마지막 정상 chunk에서 feof()=0이고 다음 iteration에서 n=0으로 break되어
        // is_last=1이 절대 전송되지 않음 → client가 99%(또는 100%)에서 done 처리 못 함.
        // 총 크기를 알고 있으니 offset 기반으로 정확히 판정.
        d->is_last      = (offset + n >= total) ? 1 : 0;
        d->chunk_bytes  = (uint32_t)n;
        d->offset       = offset;
        offset += n;
        uint32_t total_payload = (uint32_t)(sizeof(PktFileData)+n);
        auto pkt = make_packet(PacketType::FILE_DATA, buf.data(), total_payload);
        // send 에 걸리는 시간 측정 > 실 TCP throughput
        auto t0 = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> slk(target->fd_write_mtx);
            send_all(target->fd, pkt.data(), pkt.size());
        }
        double send_us = (double)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if(send_us > 0.0){
            double chunk_bps = (double)pkt.size() / (send_us * 1e-6);
            measured_bps = measured_bps * (1.0 - FILE_RATE_EWMA_ALPHA)
                         + chunk_bps    *        FILE_RATE_EWMA_ALPHA;
        }
        if(progress_cb) progress_cb(offset, total);
        // 목표: 측정 속도의 90% 사용 > 나머지 10%를 FFT 스트림에 양보
        uint64_t target_bps = (uint64_t)(measured_bps * 0.90);
        if(target_bps < FILE_RATE_FLOOR) target_bps = FILE_RATE_FLOOR;
        // 누적 기준으로 sleep (drift 방지)
        rate_sent += n;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - rate_epoch).count();
        int64_t want_us = (int64_t)(rate_sent * 1000000ULL / target_bps);
        if(want_us > elapsed_us)
            std::this_thread::sleep_for(std::chrono::microseconds(want_us - elapsed_us));
    }
    fclose(fp);
}

void NetServer::broadcast_iq_progress(const PktIqProgress& prog){
    auto pkt = make_packet(PacketType::IQ_PROGRESS, &prog, sizeof(prog));
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& cli : clients_){
        if(!cli->alive.load() || !cli->authed) continue;
        cli->enqueue(pkt, false);
    }
}


void NetServer::broadcast_db_list(const std::vector<DbFileEntry>& entries){
    uint16_t cnt = (uint16_t)std::min(entries.size(), (size_t)UINT16_MAX);
    size_t payload_size = sizeof(PktDbList) + cnt * sizeof(DbFileEntry);
    std::vector<uint8_t> payload(payload_size, 0);
    auto* hdr = reinterpret_cast<PktDbList*>(payload.data());
    hdr->count = cnt;
    if(cnt > 0)
        memcpy(payload.data() + sizeof(PktDbList), entries.data(), cnt * sizeof(DbFileEntry));
    auto pkt = make_packet(PacketType::DB_LIST, payload.data(), (uint32_t)payload_size);
    // Central relay JOINs에는 Central이 직접 전송하므로, on_relay_broadcast 호출 안 함
    // 직접 접속 JOIN에만 전달
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load() || c->is_relay) continue;
        c->enqueue(pkt, false);
    }
}

std::vector<OpEntry> NetServer::get_operators() const {
    std::vector<OpEntry> ops;
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load()) continue;
        OpEntry e{}; e.index=c->op_index; e.tier=c->tier;
        strncpy(e.name, c->name, 31);
        ops.push_back(e);
    }
    return ops;
}

// ── Long-Waterfall send (host → single op) ───────────────────────────────
// 둘 다 send_queue를 우회하고 fd에 직접 send_all로 씀 (send_file_to 패턴).
// 이유: send_queue는 SEND_QUEUE_MAX 초과 시 drop함. LWF chunk를 큐로 보내면
// 큰 파일에서 청크가 drop되어 JOIN에서 stream 깨짐 (bad magic).

void NetServer::send_lwf_list_to_op(int op_index, const PktLwfList& list){
    auto pkt = make_packet(PacketType::LWF_LIST, &list, sizeof(list));
    std::shared_ptr<ClientConn> target;
    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        for(auto& cli : clients_)
            if(cli->authed && cli->alive.load() && cli->op_index==(uint8_t)op_index){
                target = cli; break;
            }
    }
    if(!target) return;
    std::lock_guard<std::mutex> wlk(target->fd_write_mtx);
    send_all(target->fd, pkt.data(), pkt.size());
}

// ── LIVE 스트리밍 broadcast (v4.6.0): Central archive 전용 ───────────────
// 직접 fan-out 대상 (LAN 직결 JOIN) 은 제거됨 — Central 만 on_relay_broadcast 로 수신.
// Central 측은 archive_hist_on_live_* 로 mirror 파일만 만들고 JOIN 으로 relay 안 함.
void NetServer::broadcast_lwf_live_start(const PktLwfLiveStart& s){
    auto pkt = make_packet(PacketType::LWF_LIVE_START, &s, sizeof(s));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
}

// HIST 행 (HOST → Central 아카이브 전용; JOIN 으로는 안 나간다).
// v13.2: 6bit 팩 + zstd 로 전송 → Central 이 8bit 로 복원해 기록하므로 .bewehist
// 디스크 포맷은 불변 (기존 뷰어 그대로). 수신측 판별은 길이:
//   payload_len == fft_size → raw(구 HOST), 그 외 → 6bit+zstd 압축본.
// (LWF_LIVE_START.fft_size 로 Central 이 행폭을 이미 알고 있어 길이 판별이 성립)
void NetServer::broadcast_lwf_live_row(const PktLwfLiveRowHdr& hdr,
                                        const uint8_t* row, uint32_t row_bytes){
    static const bool u6_on   = fft_u6_enabled();
    static const bool zstd_on = fft_zstd_enabled();
    const uint8_t* payload = row;
    uint32_t       plen    = row_bytes;
    std::vector<uint8_t> u6buf, comp;
    if(row_bytes && row && u6_on && zstd_on){
        u6buf.resize(u6_packed_bytes(row_bytes));
        u6_pack(row, row_bytes, u6buf.data());
        comp.resize(ZSTD_compressBound(u6buf.size()));
        size_t z = ZSTD_compress(comp.data(), comp.size(), u6buf.data(), u6buf.size(), 1);
        // raw 와 같은 길이면 Central 이 raw 로 오인한다 → 그 경우만 raw 폴백.
        if(!ZSTD_isError(z) && z < row_bytes && z != row_bytes){
            payload = comp.data(); plen = (uint32_t)z;
        }
    }
    std::vector<uint8_t> body(sizeof(PktLwfLiveRowHdr) + plen);
    memcpy(body.data(), &hdr, sizeof(hdr));
    if(plen && payload) memcpy(body.data() + sizeof(hdr), payload, plen);
    auto pkt = make_packet(PacketType::LWF_LIVE_ROW, body.data(), (uint32_t)body.size());
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
}

void NetServer::broadcast_lwf_live_stop(const PktLwfLiveStop& s){
    auto pkt = make_packet(PacketType::LWF_LIVE_STOP, &s, sizeof(s));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
}

// (stream_lwf_file_to_op removed — host now uses send_file_to which streams via
//  FILE_META + FILE_DATA, JOIN's on_get_save_dir routes to long_waterfall_dir.)

