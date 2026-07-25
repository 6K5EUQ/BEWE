// Mission File Archive (Phase 1, v3.8.0)
// Layout: ~/BEWE/DataBase/missions/<station>/<year>/<code>/{iq,audio,hist}/
// HOST PUSH (IQ/audio):
//   PUSH_META(transfer_id, mode, total, info)  ->  archive_dir/filename + .info
//   PUSH_DATA(offset, chunk) xN
//   PUSH_DATA(is_last=1)     -> Central close + PUSH_ACK(status=0)
// HIST: PUSH 없이 LWF_LIVE_START/ROW/STOP 스트림 tap.
#include "central_server.hpp"
#include "../src/net_protocol.hpp"
#include "../src/sigmf.hpp"
#include "../src/long_waterfall.hpp"   // build_hist_filename_finalize
#include <zstd.h>                      // HIST 행 6bit+zstd 해제 (v13.2)
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <vector>
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cstdlib>              // getenv, atoi
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>

namespace {
// stale -LIVE 판정: mtime 이 이 시간만큼 안 변하면 host stream 이 끊긴 것으로 본다.
// 30초는 너무 짧아 Wi-Fi 재접속·Central 재시작·HOST 재배포 같은 일시 단절을
// 종료로 오판했다 (실측: Central 재시작 후 기지 재접속에 15초). 3분이면 그런
// 일시 단절은 넘기고, 진짜 끊긴 파일만 닫는다.
constexpr time_t STALE_LIVE_SEC = 180;

// 파일명/경로 컴포넌트 sanitization.
// 슬래시 '/' 와 ".." 컴포넌트 차단 (디렉토리 escape 방지).
// 빈 문자열·null 입력 → false.
bool path_component_safe(const char* s, size_t maxlen){
    if(!s || !s[0]) return false;
    size_t n = strnlen(s, maxlen);
    if(n == 0 || n >= maxlen) return false;
    if(strcmp(s, ".") == 0 || strcmp(s, "..") == 0) return false;
    for(size_t i=0;i<n;i++){
        unsigned char c = (unsigned char)s[i];
        if(c == '/' || c == '\\' || c == 0) return false;
        if(c < 0x20) return false;     // 제어문자 금지
    }
    return true;
}

void mkdir_p(const std::string& p){
    if(p.empty()) return;
    std::string cur;
    cur.reserve(p.size());
    for(size_t i=0;i<p.size();i++){
        cur.push_back(p[i]);
        if(p[i] == '/' && cur.size() > 1){
            mkdir(cur.c_str(), 0755);
        }
    }
    mkdir(p.c_str(), 0755);
}

void rm_rf(const std::string& path){
    DIR* d = opendir(path.c_str());
    if(!d){ unlink(path.c_str()); return; }
    struct dirent* ent;
    while((ent = readdir(d)) != nullptr){
        const char* n = ent->d_name;
        if(!n || (n[0]=='.' && (n[1]==0 || (n[1]=='.' && n[2]==0)))) continue;
        std::string full = path + "/" + n;
        struct stat st;
        if(stat(full.c_str(), &st) != 0) continue;
        if(S_ISDIR(st.st_mode)) rm_rf(full);
        else                    unlink(full.c_str());
    }
    closedir(d);
    rmdir(path.c_str());
}

const char* subdir_name(uint8_t s){
    switch(s){
        case MFS_IQ:    return "iq";
        case MFS_AUDIO: return "audio";
        case MFS_HIST:  return "hist";
        default:        return nullptr;
    }
}
} // anonymous namespace

// ── HIST 압축 백그라운드 워커 ─────────────────────────────────────────────
// finalize(-LIVE→-HHMM rename) 직후, 완료된 raw v3 .bewehist 를 블록 zstd + footer
// index (v4) 로 압축한다. room 수신 스레드([archive_hist_on_live_stop])를 막지 않도록
// 별도 스레드에서 처리 — 580MB 파일이 Pi 에서 10~20s 걸려도 FFT ingestion 무영향.
// 실패 시 raw v3 원본을 그대로 남긴다(무손실 보장). BEWE_HIST_COMPRESS 게이트는 호출부에서.
namespace {

std::mutex               g_hc_mtx;
std::condition_variable  g_hc_cv;
std::deque<std::string>  g_hc_q;      // 압축 대상 파일 절대경로
std::thread              g_hc_thr;
std::atomic<bool>        g_hc_running{false};

// raw v3 .bewehist → v4 (블록 zstd + index). 성공 시 원자적 rename-over.
bool compress_hist_file(const std::string& path){
    FILE* in = fopen(path.c_str(), "rb");
    if(!in) return false;
    LongWaterfall::FileHeader h{};
    if(fread(&h, 1, sizeof(h), in) != sizeof(h) ||
       memcmp(h.magic, "BWWF", 4) != 0 ||
       h.version != LongWaterfall::FILE_VERSION ||   // 이미 v4 등은 skip
       h.fft_size == 0){
        fclose(in); return false;
    }
    const uint32_t fft_size = h.fft_size;
    if(fseek(in, 0, SEEK_END) != 0){ fclose(in); return false; }
    long endpos = ftell(in);
    if(endpos < (long)sizeof(h)){ fclose(in); return false; }
    const uint64_t body_bytes = (uint64_t)endpos - sizeof(h);
    const uint64_t num_rows   = body_bytes / fft_size;   // torn 마지막 행은 버림(가드)
    if(num_rows == 0){ fclose(in); return false; }       // 빈 파일 — 압축 안 함
    if(fseek(in, (long)sizeof(h), SEEK_SET) != 0){ fclose(in); return false; }

    const uint32_t block_rows = LongWaterfall::hist_default_block_rows(fft_size);
    const uint32_t num_blocks = (uint32_t)((num_rows + block_rows - 1) / block_rows);

    int level = 3;
    if(const char* e = getenv("BEWE_HIST_ZSTD_LEVEL")){
        int l = atoi(e); if(l >= 1 && l <= 19) level = l;
    }

    const std::string tmp = path + ".zst.tmp";
    FILE* out = fopen(tmp.c_str(), "wb");
    if(!out){ fclose(in); return false; }

    // v4 헤더 — v3 필드 그대로, version 만 교체 + reserved_v3 확장 채움.
    LongWaterfall::FileHeader vh = h;
    vh.version = LongWaterfall::FILE_VERSION_ZSTD;
    LongWaterfall::V4Ext& ext = LongWaterfall::v4ext(vh);
    ext.codec        = LongWaterfall::HIST_CODEC_ZSTD;
    ext.flags        = LongWaterfall::HIST_V4_FLAG_COL_DELTA;
    ext.block_rows   = block_rows;
    ext.num_blocks   = num_blocks;
    ext.num_rows     = num_rows;
    ext.index_offset = 0;   // 마지막에 patch

    bool ok = (fwrite(&vh, 1, sizeof(vh), out) == sizeof(vh));

    std::vector<uint8_t> rawbuf((size_t)block_rows * fft_size);
    std::vector<uint8_t> deltabuf(rawbuf.size());   // col-delta 변환본
    std::vector<uint8_t> compbuf(ZSTD_compressBound(rawbuf.size()));
    std::vector<LongWaterfall::HistBlockIndex> index;
    index.reserve(num_blocks);

    uint64_t file_off = sizeof(vh);   // out 파일 내 현재 오프셋
    for(uint32_t b = 0; b < num_blocks && ok; b++){
        const uint64_t rows_here = std::min<uint64_t>(block_rows,
                                       num_rows - (uint64_t)b * block_rows);
        const size_t raw_len = (size_t)rows_here * fft_size;
        if(fread(rawbuf.data(), 1, raw_len, in) != raw_len){ ok = false; break; }
        // col-delta: 각 행에서 d[i]=x[i]-x[i-1], d[0]=x[0] (uint8 wrap) — 압축비 개선.
        for(uint64_t rr = 0; rr < rows_here; rr++){
            const uint8_t* s = rawbuf.data()   + (size_t)rr * fft_size;
            uint8_t*       d = deltabuf.data() + (size_t)rr * fft_size;
            uint8_t prev = 0;
            for(uint32_t i = 0; i < fft_size; i++){ d[i] = (uint8_t)(s[i] - prev); prev = s[i]; }
        }
        const size_t cz = ZSTD_compress(compbuf.data(), compbuf.size(),
                                        deltabuf.data(), raw_len, level);
        if(ZSTD_isError(cz)){ ok = false; break; }
        if(fwrite(compbuf.data(), 1, cz, out) != cz){ ok = false; break; }
        LongWaterfall::HistBlockIndex e{};
        e.frame_offset = file_off;
        e.comp_len     = (uint32_t)cz;
        e.raw_rows     = (uint32_t)rows_here;
        index.push_back(e);
        file_off += cz;
    }

    const uint64_t index_off = file_off;
    if(ok){
        const size_t iw = fwrite(index.data(), sizeof(LongWaterfall::HistBlockIndex),
                                 index.size(), out);
        if(iw != index.size()) ok = false;
    }
    if(ok){
        ext.index_offset = index_off;                 // vh.reserved_v3 갱신
        fflush(out);
        if(fseek(out, 0, SEEK_SET) != 0) ok = false;
        else if(fwrite(&vh, 1, sizeof(vh), out) != sizeof(vh)) ok = false;
    }
    if(ok){
        fflush(out);
        int fd = fileno(out);
        if(fd >= 0) fsync(fd);
    }
    fclose(in);
    fclose(out);

    if(!ok){
        unlink(tmp.c_str());
        fprintf(stderr, "[Central][Archive] HIST compress FAIL %s — keep raw v3\n",
                path.c_str());
        return false;
    }
    if(rename(tmp.c_str(), path.c_str()) != 0){
        unlink(tmp.c_str());
        fprintf(stderr, "[Central][Archive] HIST compress rename FAIL %s errno=%d — keep raw v3\n",
                path.c_str(), errno);
        return false;
    }
    const uint64_t raw_sz = sizeof(h) + body_bytes;
    const uint64_t new_sz = index_off + (uint64_t)index.size() * sizeof(LongWaterfall::HistBlockIndex);
    printf("[Central][Archive] HIST compress OK %s: %.1f→%.1f MB (%.2fx, %llu rows, %u blocks, L%d)\n",
           path.c_str(), raw_sz/1e6, new_sz/1e6,
           new_sz ? (double)raw_sz/(double)new_sz : 0.0,
           (unsigned long long)num_rows, num_blocks, level);
    return true;
}

void hist_compress_loop(){
    while(g_hc_running.load()){
        std::string job;
        {
            std::unique_lock<std::mutex> lk(g_hc_mtx);
            g_hc_cv.wait_for(lk, std::chrono::seconds(2), []{
                return !g_hc_running.load() || !g_hc_q.empty();
            });
            if(!g_hc_running.load()) break;
            if(g_hc_q.empty()) continue;
            job = std::move(g_hc_q.front());
            g_hc_q.pop_front();
        }
        compress_hist_file(job);
    }
}

// finalize 훅에서 호출. 최초 호출 시 프로세스 수명 워커를 lazy-start.
void hist_compress_enqueue(const std::string& path){
    std::lock_guard<std::mutex> lk(g_hc_mtx);
    if(!g_hc_running.exchange(true)){
        g_hc_thr = std::thread(hist_compress_loop);
        g_hc_thr.detach();   // Central 프로세스 수명동안 상주
    }
    g_hc_q.push_back(path);
    g_hc_cv.notify_all();
}

} // anonymous namespace (HIST 압축 워커)

std::string CentralServer::archive_root() const {
    const char* home = getenv("HOME");
    std::string base = home ? std::string(home) + "/BEWE/DataBase"
                            : std::string("/tmp/BEWE/DataBase");
    return base + "/missions";
}

std::string CentralServer::archive_dir(const char* station, uint16_t year,
                                       const char* code, uint8_t subdir) const {
    const char* sub = subdir_name(subdir);
    if(!sub) return std::string();
    if(!path_component_safe(station, 64)) return std::string();
    if(!path_component_safe(code, 8))     return std::string();
    if(year < 1970 || year > 3000)        return std::string();
    char y[8]; snprintf(y, sizeof(y), "%04u", (unsigned)year);
    return archive_root() + "/" + station + "/" + y + "/" + code + "/" + sub;
}

void CentralServer::archive_wipe_mission(const char* station, uint16_t year,
                                         const char* code){
    if(!path_component_safe(station, 64)) return;
    if(!path_component_safe(code, 8))     return;
    if(year < 1970 || year > 3000)        return;
    char y[8]; snprintf(y, sizeof(y), "%04u", (unsigned)year);
    std::string dir = archive_root() + "/" + station + "/" + y + "/" + code;
    rm_rf(dir);
    printf("[Central][Archive] WIPE %s\n", dir.c_str());
}

// ── HOST → Central: PUSH_META ────────────────────────────────────────────
void CentralServer::handle_mission_file_push_meta(std::shared_ptr<HostRoom> room,
                                                   const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionFilePushMeta)){
        printf("[Central][Archive] PUSH_META short payload=%zu < %zu\n",
               plen, sizeof(PktMissionFilePushMeta));
        return;
    }
    const auto* m = reinterpret_cast<const PktMissionFilePushMeta*>(payload);

    // key 유효성 검증
    char station[65]={}; memcpy(station, m->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    m->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   m->key.filename,128); fname[128]=0;

    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128) || subdir_name(m->key.subdir) == nullptr){
        printf("[Central][Archive] PUSH_META invalid key station='%s' year=%u code='%s' subdir=%u file='%s'\n",
               station, m->key.year, code, m->key.subdir, fname);
        MissionFileKey k = m->key;
        send_push_ack(room, k, m->transfer_id, 3, 0, "invalid key");
        return;
    }

    std::string dir = archive_dir(station, m->key.year, code, m->key.subdir);
    if(dir.empty()){
        send_push_ack(room, m->key, m->transfer_id, 3, 0, "bad path");
        return;
    }
    mkdir_p(dir);

    std::string fullpath = dir + "/" + fname;

    // 기존 transfer 있으면 닫음 (id 충돌 → 이전 transfer 폐기)
    auto it = room->mission_xfers.find(m->transfer_id);
    if(it != room->mission_xfers.end() && it->second.fp){
        fclose(it->second.fp);
        room->mission_xfers.erase(it);
    }

    // mode==1(append) 일 때 + 기존 파일 있으면 append, 없으면 new
    // mode==0(replace) 일 때 truncate
    const char* fmode = (m->mode == 1) ? "ab" : "wb";
    FILE* fp = fopen(fullpath.c_str(), fmode);
    if(!fp){
        printf("[Central][Archive] PUSH_META fopen FAIL '%s' errno=%d (%s)\n",
               fullpath.c_str(), errno, strerror(errno));
        send_push_ack(room, m->key, m->transfer_id, 1, 0, strerror(errno));
        return;
    }

    MissionFileTransfer xf;
    xf.key = m->key;
    xf.fp = fp;
    xf.expected_bytes = m->total_bytes;
    xf.written_bytes = 0;
    xf.high_water = 0;
    xf.archive_path = fullpath;
    xf.info_data.assign(m->info_data, strnlen(m->info_data, sizeof(m->info_data)));
    auto ins = room->mission_xfers.emplace(m->transfer_id, std::move(xf));

    // info sidecar 즉시 작성 (있으면)
    if(!ins.first->second.info_data.empty()){
        FILE* fi = fopen(SigMF::sidecar_path(fullpath).c_str(), "w");
        if(fi){
            const auto& s = ins.first->second.info_data;
            fwrite(s.data(), 1, s.size(), fi);
            fclose(fi);
        }
    }

    printf("[Central][Archive] PUSH_META xfer=%u mode=%u total=%lu → %s\n",
           m->transfer_id, m->mode, (unsigned long)m->total_bytes, fullpath.c_str());
}

// ── HOST → Central: PUSH_DATA ────────────────────────────────────────────
void CentralServer::handle_mission_file_push_data(std::shared_ptr<HostRoom> room,
                                                   const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionFilePushData)) return;
    const auto* d = reinterpret_cast<const PktMissionFilePushData*>(payload);
    const uint8_t* raw = payload + sizeof(PktMissionFilePushData);
    size_t raw_avail = plen - sizeof(PktMissionFilePushData);
    if(d->chunk_bytes > raw_avail){
        printf("[Central][Archive] PUSH_DATA truncated xfer=%u chunk=%u avail=%zu\n",
               d->transfer_id, d->chunk_bytes, raw_avail);
        return;
    }

    auto it = room->mission_xfers.find(d->transfer_id);
    if(it == room->mission_xfers.end() || !it->second.fp){
        printf("[Central][Archive] PUSH_DATA unknown xfer=%u (no META)\n", d->transfer_id);
        return;
    }
    auto& xf = it->second;

    // offset 위치로 seek 후 write. append 모드라도 fseek 가능.
    if(fseeko(xf.fp, (off_t)d->offset, SEEK_SET) != 0){
        printf("[Central][Archive] PUSH_DATA seek FAIL xfer=%u offset=%lu errno=%d\n",
               d->transfer_id, (unsigned long)d->offset, errno);
        send_push_ack(room, xf.key, d->transfer_id, 1, xf.written_bytes, "seek failed");
        fclose(xf.fp); xf.fp = nullptr;
        room->mission_xfers.erase(it);
        return;
    }
    if(d->chunk_bytes > 0){
        size_t w = fwrite(raw, 1, d->chunk_bytes, xf.fp);
        if(w != d->chunk_bytes){
            printf("[Central][Archive] PUSH_DATA write short xfer=%u want=%u got=%zu\n",
                   d->transfer_id, d->chunk_bytes, w);
            send_push_ack(room, xf.key, d->transfer_id, 1, xf.written_bytes, "short write");
            fclose(xf.fp); xf.fp = nullptr;
            room->mission_xfers.erase(it);
            return;
        }
        xf.written_bytes += w;
        uint64_t hw = d->offset + d->chunk_bytes;
        if(hw > xf.high_water) xf.high_water = hw;
    }

    if(d->is_last){
        fflush(xf.fp);
        fclose(xf.fp); xf.fp = nullptr;
        // 디스크 commit 후 ack
        struct stat st{}; uint64_t disk_bytes = 0;
        if(stat(xf.archive_path.c_str(), &st) == 0) disk_bytes = (uint64_t)st.st_size;
        printf("[Central][Archive] PUSH_DATA last xfer=%u written=%lu disk=%lu → ACK\n",
               d->transfer_id, (unsigned long)xf.written_bytes, (unsigned long)disk_bytes);
        send_push_ack(room, xf.key, d->transfer_id, 0, disk_bytes, "");
        room->mission_xfers.erase(it);
    }
}

void CentralServer::send_push_ack(std::shared_ptr<HostRoom> room,
                                   const MissionFileKey& key, uint8_t transfer_id,
                                   uint8_t status, uint64_t total_bytes,
                                   const char* err){
    PktMissionFilePushAck ack{};
    ack.key = key;
    ack.transfer_id = transfer_id;
    ack.status = status;
    ack.total_bytes = total_bytes;
    if(err) strncpy(ack.error_msg, err, sizeof(ack.error_msg)-1);
    auto bewe = CentralServer::make_bewe_packet(BEWE_TYPE_MISSION_FILE_PUSH_ACK,
                                                  &ack, sizeof(ack));
    enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                      bewe.data(), (uint32_t)bewe.size());
}

// ── LIST_REQ ─────────────────────────────────────────────────────────────
// 스캔 후 PktMissionFileList 페이지(들) 전송. count > MAX 면 다중 page.
// requester != nullptr → JOIN, == nullptr → HOST.
void CentralServer::handle_mission_file_list_req(std::shared_ptr<HostRoom> room,
                                                  std::shared_ptr<JoinEntry> requester,
                                                  const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionFileListReq)) return;
    const auto* r = reinterpret_cast<const PktMissionFileListReq*>(payload);
    char fstation[65]={}; memcpy(fstation, r->station, 64); fstation[64]=0;
    char fcode[9]={};     memcpy(fcode,    r->code,    8);  fcode[8]=0;
    uint16_t fyear = r->year;
    uint8_t  fsub  = r->subdir;

    std::string root = archive_root();
    std::vector<MissionFileEntry> rows;

    auto add_if = [&](const std::string& station_s, uint16_t y, const std::string& code_s,
                      uint8_t sub){
        std::string dir = archive_dir(station_s.c_str(), y, code_s.c_str(), sub);
        if(dir.empty()) return;
        DIR* d = opendir(dir.c_str());
        if(!d) return;
        struct dirent* ent;
        while((ent = readdir(d)) != nullptr){
            if(!ent->d_name || ent->d_name[0] == '.') continue;
            // stale-LIVE finalize 시 이름이 바뀌므로 소유권 있는 문자열로 받는다.
            std::string nm = ent->d_name;
            const char* n = nm.c_str();
            // sidecar(.info / .sigmf-meta) 는 list에서 제외
            size_t nlen = nm.size();
            if(nlen >= 5  && strcmp(n + nlen - 5,  ".info")       == 0) continue;
            if(nlen >= 11 && strcmp(n + nlen - 11, ".sigmf-meta") == 0) continue;
            std::string full = dir + "/" + n;
            struct stat st;
            if(stat(full.c_str(), &st) != 0) continue;
            if(!S_ISREG(st.st_mode)) continue;
            // Stale -LIVE.bewehist: mtime 이 STALE_LIVE_SEC 이상 안 변경됐으면 host stream
            // 이 끊긴 것으로 판단. 예전엔 unlink 했으나 HIST 는 Central 이 유일본이라
            // (HOST 는 업로드 후 로컬을 지운다) 삭제하면 그 시간대 기록이 영구 소실된다.
            // → HOST 의 finalize_stale_live_in_dir() 과 동일하게 mtime 을 종료시각으로 삼아
            //   -HHMM 이름으로 rename 한다. 끊긴 시점까지의 데이터는 유효하다.
            if(strstr(n, "-LIVE.bewehist")){
                time_t now = time(nullptr);
                if(st.st_mtime + STALE_LIVE_SEC < now){
                    std::string fin = LongWaterfall::build_hist_filename_finalize(
                                        std::string(n), (uint64_t)st.st_mtime, 0);
                    if(fin != n){
                        std::string fin_full = dir + "/" + fin;
                        // 충돌 회피: 같은 이름이 이미 있으면 _2, _3 ... (HOST 와 동일 규칙)
                        std::string try_fin = fin, try_full = fin_full;
                        for(int k = 2; access(try_full.c_str(), F_OK) == 0 && k < 100; ++k){
                            auto dot = fin.rfind(".bewehist");
                            if(dot == std::string::npos) break;
                            try_fin  = fin.substr(0, dot) + "_" + std::to_string(k) + ".bewehist";
                            try_full = dir + "/" + try_fin;
                        }
                        if(rename(full.c_str(), try_full.c_str()) == 0){
                            printf("[Central][Archive] stale-LIVE finalize %s -> %s (idle %lds)\n",
                                   n, try_fin.c_str(), (long)(now - st.st_mtime));
                            std::string sc_old = SigMF::sidecar_path(full);
                            std::string sc_new = SigMF::sidecar_path(try_full);
                            if(access(sc_old.c_str(), F_OK) == 0)
                                rename(sc_old.c_str(), sc_new.c_str());
                            // 이번 LIST 응답에 최종 이름으로 싣는다.
                            nm = try_fin;  n = nm.c_str();  nlen = nm.size();
                            full = try_full;
                            if(stat(full.c_str(), &st) != 0) continue;
                            if(getenv("BEWE_HIST_COMPRESS")) hist_compress_enqueue(full);
                        } else {
                            printf("[Central][Archive] stale-LIVE rename FAIL %s -> %s errno=%d\n",
                                   n, try_fin.c_str(), errno);
                        }
                    }
                }
            }
            MissionFileEntry e{};
            strncpy(e.station, station_s.c_str(), sizeof(e.station)-1);
            e.year = y;
            e.subdir = sub;
            strncpy(e.code, code_s.c_str(), sizeof(e.code)-1);
            strncpy(e.filename, n, sizeof(e.filename)-1);
            e.size_bytes = (uint64_t)st.st_size;
            e.mtime_unix = (int64_t)st.st_mtime;
            // .info sidecar 의 "Operator:" 추출 — DB 탭과 동일 표시용.
            FILE* fi = fopen(SigMF::sidecar_path(full).c_str(), "r");
            if(fi){
                char buf[1024]; size_t br = fread(buf, 1, sizeof(buf)-1, fi); buf[br]=0;
                fclose(fi);
                const char* p = buf;
                while(p && *p){
                    char k[64]={}, val[128]={};
                    if(sscanf(p, "%63[^:]: %127[^\n]", k, val) >= 2){
                        if(strcmp(k, "Operator") == 0){
                            strncpy(e.operator_name, val, sizeof(e.operator_name)-1);
                            break;
                        }
                    }
                    const char* nl = strchr(p, '\n');
                    if(!nl) break;
                    p = nl + 1;
                }
                // 파일별 note (IQ=bewe:notes / 그 외=Note: 라인) — 호버 툴팁용
                std::string nt = SigMF::note_from_text(std::string(buf, br));
                strncpy(e.note, nt.c_str(), sizeof(e.note)-1);
            }
            rows.push_back(e);
        }
        closedir(d);
    };

    auto enum_dir = [](const std::string& path, std::vector<std::string>& out){
        DIR* d = opendir(path.c_str());
        if(!d) return;
        struct dirent* ent;
        while((ent = readdir(d)) != nullptr){
            const char* n = ent->d_name;
            if(!n || n[0] == '.') continue;
            std::string full = path + "/" + n;
            struct stat st;
            if(stat(full.c_str(), &st) != 0) continue;
            if(S_ISDIR(st.st_mode)) out.push_back(n);
        }
        closedir(d);
    };

    // station 트리 walk: fstation 빈 → 모든 station
    std::vector<std::string> stations;
    if(fstation[0] && path_component_safe(fstation, 64)){
        stations.push_back(fstation);
    } else {
        enum_dir(root, stations);
    }
    for(auto& st_name : stations){
        std::vector<std::string> years;
        if(fyear){
            char yb[8]; snprintf(yb, sizeof(yb), "%04u", (unsigned)fyear);
            years.push_back(yb);
        } else {
            enum_dir(root + "/" + st_name, years);
        }
        for(auto& ys : years){
            uint16_t y = (uint16_t)atoi(ys.c_str());
            if(y < 1970 || y > 3000) continue;
            std::vector<std::string> codes;
            if(fcode[0] && path_component_safe(fcode, 8)){
                codes.push_back(fcode);
            } else {
                enum_dir(root + "/" + st_name + "/" + ys, codes);
            }
            for(auto& cs : codes){
                if(fsub){
                    add_if(st_name, y, cs, fsub);
                } else {
                    add_if(st_name, y, cs, MFS_IQ);
                    add_if(st_name, y, cs, MFS_AUDIO);
                    add_if(st_name, y, cs, MFS_HIST);
                }
            }
        }
    }

    printf("[Central][Archive] LIST_REQ station='%s' year=%u code='%s' sub=%u → %zu rows\n",
           fstation, fyear, fcode, fsub, rows.size());

    // 페이지 분할 전송
    size_t per_pkt = (size_t)MAX_MISSION_FILES_PER_PKT;
    size_t total = rows.size();
    size_t pages = total == 0 ? 1 : (total + per_pkt - 1) / per_pkt;
    for(size_t pg = 0; pg < pages; pg++){
        PktMissionFileList list{};
        size_t off = pg * per_pkt;
        size_t n = std::min(per_pkt, total - off);
        list.count = (uint16_t)n;
        list.is_last_page = (pg + 1 == pages) ? 1 : 0;
        for(size_t i = 0; i < n; i++) list.entries[i] = rows[off + i];
        auto bewe = CentralServer::make_bewe_packet(
            BEWE_TYPE_MISSION_FILE_LIST, &list, sizeof(list));
        if(requester){
            requester->enqueue_ctrl(bewe.data(), bewe.size());
        } else {
            enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                              bewe.data(), (uint32_t)bewe.size());
        }
    }
}

// ── missions.json 저장 키 (v13.4.1) ──────────────────────────────────────
// MISSION_SYNC blob 안의 station_name (active 우선, 없으면 첫 유효 history) 을 키로 쓴다.
// room->station_id 는 "<station>_<login_id>" 라 로그인 ID 가 바뀌면 같은 기지가
// 새 항목으로 쌓인다. 이름을 못 찾으면 station_id 로 폴백 (기록 유실보다 낫다).
std::string CentralServer::mission_sync_station_key(std::shared_ptr<HostRoom> room,
                                                     const uint8_t* bewe_pkt, size_t bewe_len){
    if(bewe_len >= BEWE_HDR_SIZE + sizeof(PktMissionSync)){
        const auto* s = reinterpret_cast<const PktMissionSync*>(bewe_pkt + BEWE_HDR_SIZE);
        auto name_of = [](const MissionSyncEntry& e) -> const char* {
            return (e.valid && e.station_name[0]) ? e.station_name : nullptr;
        };
        if(s->active_valid)
            if(const char* n = name_of(s->active)) return std::string(n);
        uint16_t cnt = s->history_count;
        if(cnt > MAX_MISSION_HISTORY_PER_PKT) cnt = MAX_MISSION_HISTORY_PER_PKT;
        for(uint16_t i = 0; i < cnt; i++)
            if(const char* n = name_of(s->entries[i])) return std::string(n);
    }
    return room ? room->station_id : std::string();
}

// ── SYNC_REQ (v13.4) ─────────────────────────────────────────────────────
// JOIN 이 접속하지 않은 station 의 미션 메타(started_by/lat/lon/SDR/안테나/ACTIVE)를
// 요청한다. missions_by_station_ 에 HOST 가 보낸 MISSION_SYNC 패킷이 통째로 캐시돼
// 있으므로 그 payload 를 그대로 회신에 실어 보낸다 (재파싱 불필요).
void CentralServer::handle_mission_sync_req(std::shared_ptr<HostRoom> room,
                                             std::shared_ptr<JoinEntry> requester,
                                             const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionSyncReq)){
        printf("[Central][Mission] SYNC_REQ short plen=%zu\n", plen);
        return;
    }
    const auto* r = reinterpret_cast<const PktMissionSyncReq*>(payload);
    char st[65] = {}; memcpy(st, r->station, 64); st[64] = 0;
    if(!st[0]){ printf("[Central][Mission] SYNC_REQ empty station\n"); return; }

    PktMissionSyncFor out{};
    strncpy(out.station, st, sizeof(out.station) - 1);
    int scanned = 0;
    {
        std::lock_guard<std::mutex> jlk(missions_json_mtx_);
        // 캐시 키는 station 이름이 아니라 room id ("DGS-2_SW" 처럼 <station>_<host>) 이고,
        // 같은 station 이 host_name 을 바꿔 접속할 때마다 새 키가 생긴다. 즉 한 station 의
        // blob 이 여러 개 흩어져 있고, 그중 아무거나 집으면 몇 달 전 미션이 ACTIVE 로 뜬다.
        // → station_name 이 일치하는 entry 를 전부 모아 start_utc 기준 최신순으로 합성한다.
        std::vector<MissionSyncEntry> hist;
        bool have_active = false;

        auto match = [&](const MissionSyncEntry& e){
            return e.valid && strncmp(e.station_name, st, sizeof(e.station_name)) == 0;
        };
        auto push_hist = [&](const MissionSyncEntry& e){
            // 같은 (year, code) 는 최신 start_utc 하나만 남긴다.
            for(auto& h : hist){
                if(h.year == e.year && strncmp(h.code, e.code, sizeof(h.code)) == 0){
                    if(e.start_utc > h.start_utc) h = e;
                    return;
                }
            }
            hist.push_back(e);
        };

        for(const auto& kv : missions_by_station_){
            if(kv.second.size() < BEWE_HDR_SIZE + sizeof(PktMissionSync)) continue;
            const auto* s = reinterpret_cast<const PktMissionSync*>(
                                kv.second.data() + BEWE_HDR_SIZE);
            scanned++;
            // ACTIVE 는 가장 늦게 시작된 것 하나만 (여러 blob 이 각자 옛 ACTIVE 를 들고 있다).
            if(s->active_valid && match(s->active) &&
               s->active.state == 1 /* Mission::State::ACTIVE */){
                if(!have_active || s->active.start_utc > out.sync.active.start_utc){
                    out.sync.active = s->active;
                    out.sync.active_valid = 1;
                    have_active = true;
                }
            }
            uint16_t n = s->history_count;
            if(n > MAX_MISSION_HISTORY_PER_PKT) n = MAX_MISSION_HISTORY_PER_PKT;
            for(uint16_t i = 0; i < n; i++)
                if(match(s->entries[i])) push_hist(s->entries[i]);
            // active 도 종료됐다면 history 로 취급 (blob 이 갱신되기 전 상태일 수 있다).
            if(s->active_valid && match(s->active) && s->active.state != 1)
                push_hist(s->active);
        }

        std::sort(hist.begin(), hist.end(),
                  [](const MissionSyncEntry& a, const MissionSyncEntry& b){
                      return a.start_utc > b.start_utc;
                  });
        uint16_t n = (uint16_t)std::min(hist.size(), (size_t)MAX_MISSION_HISTORY_PER_PKT);
        for(uint16_t i = 0; i < n; i++) out.sync.entries[i] = hist[i];
        out.sync.history_count = n;
        out.found = (have_active || n > 0) ? 1 : 0;
    }
    printf("[Central][Mission] SYNC_REQ station='%s' → found=%u active=%u hist=%u (scanned %d blobs)\n",
           st, out.found, out.sync.active_valid, out.sync.history_count, scanned);

    auto bewe = CentralServer::make_bewe_packet(
        BEWE_TYPE_MISSION_SYNC_FOR, &out, sizeof(out));
    if(requester){
        requester->enqueue_ctrl(bewe.data(), bewe.size());
    } else {
        enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                          bewe.data(), (uint32_t)bewe.size());
    }
}

// ── DL_REQ ───────────────────────────────────────────────────────────────
void CentralServer::handle_mission_file_dl_req(std::shared_ptr<HostRoom> room,
                                                std::shared_ptr<JoinEntry> requester,
                                                const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionFileDlReq)) return;
    const auto* r = reinterpret_cast<const PktMissionFileDlReq*>(payload);

    char station[65]={}; memcpy(station, r->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    r->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   r->key.filename,128); fname[128]=0;

    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128) || subdir_name(r->key.subdir) == nullptr){
        printf("[Central][Archive] DL_REQ invalid key\n");
        return;
    }
    std::string dir = archive_dir(station, r->key.year, code, r->key.subdir);
    if(dir.empty()) return;
    std::string full = dir + "/" + fname;
    FILE* fp = fopen(full.c_str(), "rb");
    if(!fp){
        printf("[Central][Archive] DL_REQ open FAIL %s errno=%d\n", full.c_str(), errno);
        // 빈 응답 (chunk_bytes=0, is_first=1, is_last=1) → caller가 실패 인지
        PktMissionFileDlData head{};
        head.key = r->key;
        head.is_first = 1; head.is_last = 1;
        head.total_bytes = 0; head.offset = 0; head.chunk_bytes = 0;
        auto bewe = CentralServer::make_bewe_packet(
            BEWE_TYPE_MISSION_FILE_DL_DATA, &head, sizeof(head));
        if(requester) requester->enqueue_ctrl(bewe.data(), bewe.size());
        else          enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                                        bewe.data(), (uint32_t)bewe.size());
        return;
    }
    fseeko(fp, 0, SEEK_END); uint64_t total = (uint64_t)ftello(fp);
    uint64_t start = r->start_offset;
    if(start > total) start = total;
    fseeko(fp, (off_t)start, SEEK_SET);

    // sidecar(.sigmf-meta/.info) — start==0(처음부터 받기) 일 때만 동봉. resume 시엔 이미 받았다고 간주.
    char info[1024] = {};
    if(start == 0){
        FILE* fi = fopen(SigMF::sidecar_path(full).c_str(), "r");
        if(fi){ fread(info, 1, sizeof(info)-1, fi); fclose(fi); }
    }

    // 이미 받은 양이 현재 size 와 같거나 더 큰 경우: 빈 응답 (chunk_bytes=0, is_first/last=1)
    if(start >= total){
        PktMissionFileDlData head{};
        head.key = r->key;
        head.total_bytes = total;
        head.offset = start;
        head.chunk_bytes = 0;
        head.is_first = 1; head.is_last = 1;
        auto bewe = CentralServer::make_bewe_packet(
            BEWE_TYPE_MISSION_FILE_DL_DATA, &head, sizeof(head));
        if(requester) requester->enqueue_ctrl(bewe.data(), bewe.size());
        else          enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                                        bewe.data(), (uint32_t)bewe.size());
        fclose(fp);
        printf("[Central][Archive] DL_REQ %s up-to-date (start=%lu total=%lu)\n",
               full.c_str(), (unsigned long)start, (unsigned long)total);
        return;
    }

    constexpr uint32_t CHUNK = 256 * 1024;  // 256 KB
    uint64_t off = start;
    bool first = true;
    while(off < total){
        uint32_t want = (uint32_t)std::min((uint64_t)CHUNK, total - off);
        PktMissionFileDlData head{};
        head.key = r->key;
        head.total_bytes = total;
        head.offset = off;
        head.chunk_bytes = want;
        head.is_first = first ? 1 : 0;
        head.is_last  = (off + want >= total) ? 1 : 0;
        if(first && start == 0) memcpy(head.info_data, info, sizeof(head.info_data));
        // BEWE 패킷에 직접 빌드 — 중간 chunk/payload 버퍼 복사 제거 (fread → 패킷 직행)
        uint32_t pkt_plen = (uint32_t)(sizeof(head) + want);
        std::vector<uint8_t> bewe(BEWE_HDR_SIZE + pkt_plen);
        memcpy(bewe.data(), "BEWE", 4);
        bewe[4] = BEWE_TYPE_MISSION_FILE_DL_DATA;
        memcpy(bewe.data()+5, &pkt_plen, 4);
        memcpy(bewe.data()+BEWE_HDR_SIZE, &head, sizeof(head));
        size_t got = fread(bewe.data()+BEWE_HDR_SIZE+sizeof(head), 1, want, fp);
        if(got != want){
            printf("[Central][Archive] DL_REQ read short off=%lu want=%u got=%zu\n",
                   (unsigned long)off, want, got);
            break;
        }
        if(requester){
            requester->enqueue_file(bewe.data(), bewe.size());
        } else {
            enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                              bewe.data(), (uint32_t)bewe.size());
        }
        off += want;
        first = false;
    }
    fclose(fp);
    printf("[Central][Archive] DL_REQ done %s start=%lu sent=%lu total=%lu\n",
           full.c_str(), (unsigned long)start, (unsigned long)(off - start),
           (unsigned long)total);
}

// ── DELETE ───────────────────────────────────────────────────────────────
void CentralServer::handle_mission_file_delete(std::shared_ptr<HostRoom> room,
                                                std::shared_ptr<JoinEntry> requester,
                                                const uint8_t* payload, size_t plen){
    (void)room; (void)requester;
    if(plen < sizeof(PktMissionFileDelete)) return;
    const auto* r = reinterpret_cast<const PktMissionFileDelete*>(payload);
    char station[65]={}; memcpy(station, r->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    r->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   r->key.filename,128); fname[128]=0;
    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128) || subdir_name(r->key.subdir) == nullptr){
        printf("[Central][Archive] DELETE invalid key\n");
        return;
    }
    std::string dir = archive_dir(station, r->key.year, code, r->key.subdir);
    if(dir.empty()) return;
    std::string full = dir + "/" + fname;
    int rc1 = unlink(full.c_str());
    int rc2 = unlink(SigMF::sidecar_path(full).c_str());
    printf("[Central][Archive] DELETE %s (file=%d info=%d)\n", full.c_str(), rc1, rc2);
}

// ── RENAME ───────────────────────────────────────────────────────────────
void CentralServer::handle_mission_file_rename(std::shared_ptr<HostRoom> room,
                                                std::shared_ptr<JoinEntry> requester,
                                                const uint8_t* payload, size_t plen){
    (void)room; (void)requester;
    if(plen < sizeof(PktMissionFileRename)) return;
    const auto* r = reinterpret_cast<const PktMissionFileRename*>(payload);
    char station[65]={}; memcpy(station, r->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    r->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   r->key.filename,128); fname[128]=0;
    char newfn[129]={}; memcpy(newfn,   r->new_filename,128); newfn[128]=0;
    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128)  || !path_component_safe(newfn, 128) ||
       subdir_name(r->key.subdir) == nullptr){
        printf("[Central][Archive] RENAME invalid key\n");
        return;
    }
    std::string dir = archive_dir(station, r->key.year, code, r->key.subdir);
    if(dir.empty()) return;
    std::string src = dir + "/" + fname;
    std::string dst = dir + "/" + newfn;
    int rc1 = rename(src.c_str(), dst.c_str());
    int rc2 = rename(SigMF::sidecar_path(src).c_str(), SigMF::sidecar_path(dst).c_str());
    printf("[Central][Archive] RENAME %s → %s (file=%d info=%d)\n",
           src.c_str(), newfn, rc1, rc2);
}

// ── SET_NOTE: 파일별 메모를 사이드카에 기입 (IQ=bewe:notes / 그 외=Note: 라인) ──
// 클라가 다음 LIST_REQ(2초 폴링)에서 갱신된 note 를 받아 호버 툴팁 반영.
void CentralServer::handle_mission_file_set_note(std::shared_ptr<HostRoom> room,
                                                 std::shared_ptr<JoinEntry> requester,
                                                 const uint8_t* payload, size_t plen){
    (void)room; (void)requester;
    if(plen < sizeof(PktMissionFileSetNote)) return;
    const auto* r = reinterpret_cast<const PktMissionFileSetNote*>(payload);
    char station[65]={}; memcpy(station, r->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    r->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   r->key.filename,128); fname[128]=0;
    char note[257]={};  memcpy(note,    r->note,        256); note[256]=0;
    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128) || subdir_name(r->key.subdir) == nullptr){
        printf("[Central][Archive] SET_NOTE invalid key\n");
        return;
    }
    std::string dir = archive_dir(station, r->key.year, code, r->key.subdir);
    if(dir.empty()) return;
    std::string full = dir + "/" + fname;
    bool ok = SigMF::update_note(full, note);
    printf("[Central][Archive] SET_NOTE %s (ok=%d, %zu chars)\n", full.c_str(), (int)ok, strlen(note));
}

// ── LWF live stream → HIST archive tap ───────────────────────────────────
void CentralServer::archive_hist_on_live_start(std::shared_ptr<HostRoom> room,
                                                const PktLwfLiveStart& ls){
    if(!room->active_mission_valid) return;  // 미션 활성 아니면 archive 안 함
    char fname[65]={}; memcpy(fname, ls.filename, 64); fname[64]=0;
    if(!path_component_safe(fname, 128)){
        printf("[Central][Archive] HIST live_start invalid filename\n");
        return;
    }
    std::string dir = archive_dir(room->active_mission_station,
                                  room->active_mission_year,
                                  room->active_mission_code, MFS_HIST);
    if(dir.empty()) return;
    mkdir_p(dir);
    std::string full = dir + "/" + fname;

    // 기존 stream이 같은 filename으로 열려있으면 닫음
    auto it = room->hist_streams.find(fname);
    if(it != room->hist_streams.end() && it->second.fp){
        fclose(it->second.fp);
        room->hist_streams.erase(it);
    }

    FILE* fp = fopen(full.c_str(), "wb");
    if(!fp){
        printf("[Central][Archive] HIST live_start open FAIL %s errno=%d\n",
               full.c_str(), errno);
        return;
    }
    // FileHeader 작성 (long_waterfall::FileHeader v3 128B와 동일 layout 재현)
    // 이걸 직접 헤더 import 없이 inline으로 박는다 — 대신 fields 만큼 정확히.
    struct __attribute__((packed)) LwfHeader128 {
        char     magic[4];          // "BWWF"
        uint16_t version;           // 0x0003
        uint32_t fft_size;
        uint64_t sample_rate_hz;
        uint64_t center_freq_hz;
        float    row_rate_hz;
        float    db_min;
        float    db_max;
        uint64_t start_utc_unix;
        float    station_lon;
        uint32_t fft_input_size;
        int32_t  utc_offset_hours;
        uint8_t  reserved_v2[6];
        char     station_name[32];
        float    station_lat;
        uint8_t  reserved_v3[28];
    } h{};
    static_assert(sizeof(LwfHeader128) == 128, "LwfHeader128 must be 128 bytes");
    memcpy(h.magic, "BWWF", 4);
    h.version = 0x0003;
    h.fft_size = ls.fft_size;
    h.sample_rate_hz = ls.sample_rate_hz;
    h.center_freq_hz = ls.center_freq_hz;
    // v13.3: Central 은 FFT_FRAME 을 행으로 기록한다 (HOST 의 5Hz max-hold flush 가 아니라
    // FFT 행마다 1행) → 헤더의 row_rate 도 FFT 행레이트여야 뷰어 시간축이 맞는다.
    // 구 HOST(필드 없음/0)면 종전 row_rate_hz 로 폴백.
    h.row_rate_hz = (ls.fft_row_rate_hz > 0.5f) ? ls.fft_row_rate_hz : ls.row_rate_hz;
    h.db_min = ls.db_min;
    h.db_max = ls.db_max;
    h.start_utc_unix = ls.start_utc_unix;
    h.station_lon = ls.station_lon;
    h.fft_input_size = ls.fft_input_size;
    h.utc_offset_hours = ls.utc_offset_hours;
    memcpy(h.station_name, ls.station_name, sizeof(h.station_name));
    h.station_lat = ls.station_lat;
    fwrite(&h, 1, sizeof(h), fp);
    fflush(fp);

    MissionHistStream st;
    st.archive_path = full;
    st.fp = fp;
    strncpy(st.station, room->active_mission_station, sizeof(st.station)-1);
    st.year = room->active_mission_year;
    strncpy(st.code, room->active_mission_code, sizeof(st.code)-1);
    st.fft_size = ls.fft_size;
    st.rows_written = 0;
    st.start_utc = ls.start_utc_unix;   // 짧은-수명 조각 판별용
    room->hist_streams.emplace(fname, std::move(st));

    printf("[Central][Archive] HIST stream OPEN %s (fft=%u, %.3fMHz)\n",
           full.c_str(), ls.fft_size, ls.center_freq_hz / 1e6);
}

// v13.3: FFT_FRAME 을 그대로 HIST 행으로 아카이브한다.
// HOST 가 같은 데이터를 FFT + LWF_LIVE_ROW 로 두 번 보내던 것을 없앴다 — 미션이 켜져
// 있으면 HOST 는 JOIN 이 없어도 FFT 를 계속 보내고, Central 이 그 스트림을 기록한다.
// 결과적으로 아카이브 시간해상도가 5Hz(max-hold 접힘) → FFT 행레이트(수십 Hz)로 올라간다.
//
// payload = PktFftFrame 헤더 + [6bit 팩 + zstd] uint8 양자화본. 디스크에는 종전과 같이
// uint8 1B/bin 으로 풀어 쓴다 → .bewehist 포맷 불변.
void CentralServer::archive_hist_on_fft(std::shared_ptr<HostRoom> room,
                                         const uint8_t* bewe_pkt, uint32_t bewe_len){
    if(room->hist_streams.empty()) return;                   // 미션 비활성 → 기록 안 함
    if(bewe_len < BEWE_HDR_SIZE + sizeof(PktFftFrame)) return;
    const auto* fh = reinterpret_cast<const PktFftFrame*>(bewe_pkt + BEWE_HDR_SIZE);

    const uint32_t flags     = fh->fft_size;
    const uint32_t fft_size  = flags & FFT_FFT_SIZE_MASK;
    const bool     quantized = (flags & FFT_FLAG_QUANT_U8) != 0;
    const bool     zstd_c    = (flags & FFT_FLAG_ZSTD)     != 0;
    const bool     u6_c      = (flags & FFT_FLAG_QUANT_U6) != 0;
    if(!quantized || fft_size == 0) return;                  // float 프레임(구버전)은 무시

    const uint8_t* body     = bewe_pkt + BEWE_HDR_SIZE + sizeof(PktFftFrame);
    uint32_t       body_len = bewe_len - BEWE_HDR_SIZE - (uint32_t)sizeof(PktFftFrame);
    const size_t   inner    = u6_c ? u6_packed_bytes(fft_size) : (size_t)fft_size;

    std::vector<uint8_t> zbuf, row(fft_size);
    const uint8_t* packed = nullptr;
    if(zstd_c){
        zbuf.resize(inner);
        size_t d = ZSTD_decompress(zbuf.data(), inner, body, body_len);
        if(ZSTD_isError(d) || d != inner) return;
        packed = zbuf.data();
    } else {
        if(body_len != inner) return;
        packed = body;
    }
    if(u6_c) u6_unpack(packed, fft_size, row.data());
    else     memcpy(row.data(), packed, fft_size);

    // 이 room 에 열려 있는 모든 hist stream 에 행 추가 (통상 1개).
    // FFT 는 padded 폭(fft_size)으로 오지만 디스크 행은 1x 폭(st.fft_size = HOST 의
    // fft_input_size)이다. pad 묶음 max-hold 로 접는다 — zero-pad 는 sinc 보간일 뿐
    // 새 정보가 없으므로 손실이 아니고, max 라 좁은 피크가 살아남는다. HOST 로컬
    // .bewehist 가 쓰는 폴딩(long_waterfall.cpp ingest_new_rows)과 같은 규칙이다.
    // CLI HOST 는 PAD=1 이라 pad=1 → 항등.
    std::vector<uint8_t> folded;
    for(auto& [fname, st] : room->hist_streams){
        if(!st.fp || !st.fft_size) continue;
        const uint8_t* out     = row.data();
        uint32_t       out_len = fft_size;
        if(st.fft_size != fft_size){
            if(fft_size % st.fft_size != 0) continue;   // 배수 아님 → 기록 불가
            const uint32_t pad = fft_size / st.fft_size;
            folded.resize(st.fft_size);
            for(uint32_t o = 0; o < st.fft_size; o++){
                const uint8_t* gp = row.data() + (size_t)o * pad;
                uint8_t mx = gp[0];
                for(uint32_t k = 1; k < pad; k++) if(gp[k] > mx) mx = gp[k];
                folded[o] = mx;
            }
            out = folded.data(); out_len = st.fft_size;
        }
        fwrite(out, 1, out_len, st.fp);
        st.rows_written++;
        fflush(st.fp);
    }
}

void CentralServer::archive_hist_on_live_row(std::shared_ptr<HostRoom> room,
                                              const PktLwfLiveRowHdr& hdr,
                                              const uint8_t* row, uint32_t row_bytes){
    char fname[65]={}; memcpy(fname, hdr.filename, 64); fname[64]=0;
    auto it = room->hist_streams.find(fname);
    if(it == room->hist_streams.end() || !it->second.fp) return;
    auto& st = it->second;
    if(row_bytes == 0) return;
    // v13.2: HOST 가 6bit 팩 + zstd 로 보낸다 (row_bytes != fft_size). 여기서 8bit 로
    // 복원해 기록 → .bewehist 디스크 포맷은 불변 (기존 뷰어/파서 그대로).
    // row_bytes == fft_size 면 구 HOST 의 raw 행 → 그대로 기록.
    std::vector<uint8_t> restored;
    if(st.fft_size && row_bytes != st.fft_size){
        std::vector<uint8_t> packed(u6_packed_bytes(st.fft_size));
        size_t d = ZSTD_decompress(packed.data(), packed.size(), row, row_bytes);
        if(ZSTD_isError(d) || d != packed.size()){
            printf("[Central][Archive] HIST row decode fail (%u B, expect %zu) — drop\n",
                   row_bytes, packed.size());
            return;
        }
        restored.resize(st.fft_size);
        u6_unpack(packed.data(), st.fft_size, restored.data());
        row = restored.data(); row_bytes = st.fft_size;
    }
    fwrite(row, 1, row_bytes, st.fp);
    st.rows_written++;
    // 매 row fflush — row_rate 5Hz 라 부담 없음. UI LIST_REQ 가 stat() 으로 size
    // 가져올 때 libc user-buffer 가 비어있어야 정확한 progress 표시.
    fflush(st.fp);
}

void CentralServer::archive_hist_on_live_stop(std::shared_ptr<HostRoom> room,
                                               const PktLwfLiveStop& stop){
    char fname[65]={}; memcpy(fname, stop.filename, 64); fname[64]=0;
    auto it = room->hist_streams.find(fname);
    if(it == room->hist_streams.end()) return;
    if(it->second.fp){
        fflush(it->second.fp);
        fclose(it->second.fp);
        it->second.fp = nullptr;
    }
    // 조각 파일 discard — finalize 대신 버린다 (host long_waterfall.cpp 와 동일 규칙).
    //  (1) 0행 (header-only): 정각 rotate 에서 열자마자 닫힌 파일.
    //  (2) 짧은-수명 조각: 정각 rotate 로 연 파일이 스케줄 retune / Central 재연결의
    //      추가 rotate 로 정각 직후 몇십 초 만에 닫힌 파일(실측 12~77초, 예 0000-0001).
    //      종전엔 rows==0 만 잡아 몇백KB~2MB 조각이 finalize 되어 남았다.
    //  수명(open→close) 기준이라 분 경계를 넘겨 닫혀도 잡힌다. 정상 세그먼트는 수십 분+.
    bool short_stub =
        LongWaterfall::hist_is_short_stub(it->second.start_utc, (uint64_t)time(nullptr));
    if(it->second.rows_written == 0 || short_stub){
        printf("[Central][Archive] HIST discard %s %s (%u rows)\n",
               it->second.rows_written == 0 ? "empty" : "short-stub",
               it->second.archive_path.c_str(), it->second.rows_written);
        unlink(it->second.archive_path.c_str());
        unlink((it->second.archive_path + ".info").c_str());
        room->hist_streams.erase(it);
        return;
    }
    // -LIVE → finalize 이름으로 rename. Host long_waterfall.cpp 와 동일 규칙.
    // UTC offset = 0 (Central wall-clock 기준; host 측 offset 도 wallclock 과 매우 가까움).
    std::string base = fname;
    std::string fin  = LongWaterfall::build_hist_filename_finalize(
                        base, (uint64_t)time(nullptr), 0);
    std::string fin_path;   // 압축 대상: 성공적으로 finalize된 최종 파일
    if(fin != base){
        auto slash = it->second.archive_path.find_last_of('/');
        std::string dir2 = (slash == std::string::npos) ? ""
                         : it->second.archive_path.substr(0, slash + 1);
        std::string finalp = dir2 + fin;
        if(rename(it->second.archive_path.c_str(), finalp.c_str()) == 0){
            printf("[Central][Archive] HIST finalize %s -> %s\n", base.c_str(), fin.c_str());
            fin_path = finalp;
        } else {
            printf("[Central][Archive] HIST finalize rename FAIL %s -> %s errno=%d (%s)\n",
                   it->second.archive_path.c_str(), finalp.c_str(),
                   errno, strerror(errno));
        }
    } else {
        fin_path = it->second.archive_path;   // 이미 최종 이름
    }
    printf("[Central][Archive] HIST stream CLOSE %s (%u rows)\n",
           it->second.archive_path.c_str(), it->second.rows_written);
    // v13.2: 완료 파일을 백그라운드에서 블록 zstd(v4)로 압축. 게이트: BEWE_HIST_COMPRESS.
    // 실패해도 raw v3 원본 유지(무손실). LIVE 핫 경로는 무변경.
    if(!fin_path.empty() && getenv("BEWE_HIST_COMPRESS"))
        hist_compress_enqueue(fin_path);
    room->hist_streams.erase(it);
}

// ── MISSION_SYNC.active shadow ───────────────────────────────────────────
void CentralServer::update_active_mission_shadow(std::shared_ptr<HostRoom> room,
                                                  const uint8_t* bewe_pkt, size_t bewe_len){
    if(bewe_len < BEWE_HDR_SIZE + sizeof(PktMissionSync)) {
        room->active_mission_valid = false;
        return;
    }
    const auto* m = reinterpret_cast<const PktMissionSync*>(bewe_pkt + BEWE_HDR_SIZE);
    if(!m->active_valid || m->active.valid == 0){
        if(room->active_mission_valid){
            printf("[Central][Archive] active mission CLEAR (was %04u/%s)\n",
                   room->active_mission_year, room->active_mission_code);
        }
        room->active_mission_valid = false;
        room->active_mission_year = 0;
        memset(room->active_mission_code, 0, sizeof(room->active_mission_code));
        memset(room->active_mission_station, 0, sizeof(room->active_mission_station));
        return;
    }
    room->active_mission_valid = true;
    room->active_mission_year = m->active.year;
    memcpy(room->active_mission_code,    m->active.code,         sizeof(room->active_mission_code));
    memcpy(room->active_mission_station, m->active.station_name, sizeof(room->active_mission_station));
    room->active_mission_code[sizeof(room->active_mission_code)-1] = 0;
    room->active_mission_station[sizeof(room->active_mission_station)-1] = 0;
    printf("[Central][Archive] active mission = %04u/%s @ %s\n",
           room->active_mission_year, room->active_mission_code,
           room->active_mission_station);
}

