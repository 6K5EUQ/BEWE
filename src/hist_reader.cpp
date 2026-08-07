#include "hist_reader.hpp"
#include <zstd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cstring>
#include <new>

HistReader::~HistReader(){ close(); }

HistReader::HistReader(HistReader&& o) noexcept { *this = std::move(o); }

HistReader& HistReader::operator=(HistReader&& o) noexcept {
    if(this == &o) return *this;
    close();
    path_        = std::move(o.path_);
    hdr_         = o.hdr_;
    total_size_  = o.total_size_;
    num_rows_    = o.num_rows_;
    fp_          = o.fp_;         o.fp_ = nullptr;
    fd_          = o.fd_;         o.fd_ = -1;
    map_         = o.map_;        o.map_ = nullptr;
    map_size_    = o.map_size_;   o.map_size_ = 0;
    rowbuf_      = std::move(o.rowbuf_);
    is_v4_       = o.is_v4_;
    v4_flags_    = o.v4_flags_;
    block_rows_  = o.block_rows_;
    index_       = std::move(o.index_);
    cache_block_ = o.cache_block_;
    cache_buf_   = std::move(o.cache_buf_);
    frame_buf_   = std::move(o.frame_buf_);
    full_        = std::move(o.full_);
    return *this;
}

void HistReader::unmap_file(){
    if(map_){ munmap((void*)map_, map_size_); map_ = nullptr; map_size_ = 0; }
    if(fd_ >= 0){ ::close(fd_); fd_ = -1; }
}

void HistReader::close(){
    unmap_file();
    if(fp_){ fclose(fp_); fp_ = nullptr; }
    path_.clear();
    hdr_ = LongWaterfall::FileHeader{};
    total_size_ = 0; num_rows_ = 0;
    rowbuf_.clear(); rowbuf_.shrink_to_fit();
    is_v4_ = false; v4_flags_ = 0; block_rows_ = 0;
    index_.clear(); index_.shrink_to_fit();
    cache_block_ = -1;
    cache_buf_.clear(); cache_buf_.shrink_to_fit();
    frame_buf_.clear(); frame_buf_.shrink_to_fit();
    full_.reset();
}

bool HistReader::map_file(){
    // mmap (RDONLY, SHARED) — 실패해도 fp fallback 으로 동작.
    fd_ = ::open(path_.c_str(), O_RDONLY);
    if(fd_ < 0 || total_size_ == 0) return false;
    void* m = mmap(nullptr, total_size_, PROT_READ, MAP_SHARED, fd_, 0);
    if(m == MAP_FAILED){ ::close(fd_); fd_ = -1; return false; }
    map_ = (const uint8_t*)m;
    map_size_ = total_size_;
    // jump-around 패턴이라 커널 readahead 가 별 도움 안 됨.
    posix_madvise(m, total_size_, POSIX_MADV_RANDOM);
    return true;
}

bool HistReader::read_header(const std::string& path, LongWaterfall::FileHeader& h, uint64_t& size){
    FILE* fp = fopen(path.c_str(), "rb");
    if(!fp) return false;
    if(fread(&h, 1, sizeof(h), fp) != sizeof(h) || memcmp(h.magic,"BWWF",4)!=0
       || (h.version != LongWaterfall::FILE_VERSION
           && h.version != LongWaterfall::FILE_VERSION_ZSTD)){
        fclose(fp); return false;
    }
    fseek(fp, 0, SEEK_END);
    size = (uint64_t)ftell(fp);
    fclose(fp);
    return true;
}

uint64_t HistReader::rows_from_header(const LongWaterfall::FileHeader& h, uint64_t file_size){
    if(h.version == LongWaterfall::FILE_VERSION_ZSTD)
        return LongWaterfall::v4ext(h).num_rows;
    if(h.fft_size == 0) return 0;
    return (file_size - sizeof(h)) / h.fft_size;
}

bool HistReader::open(const std::string& path){
    close();
    FILE* fp = fopen(path.c_str(), "rb");
    if(!fp) return false;
    LongWaterfall::FileHeader h{};
    if(fread(&h, 1, sizeof(h), fp) != sizeof(h) || memcmp(h.magic,"BWWF",4)!=0
       || (h.version != LongWaterfall::FILE_VERSION
           && h.version != LongWaterfall::FILE_VERSION_ZSTD)){
        fclose(fp); return false;
    }
    fseek(fp, 0, SEEK_END);
    const uint64_t sz = (uint64_t)ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if(h.fft_size == 0 || sz < sizeof(h)){ fclose(fp); return false; }

    path_ = path;
    hdr_ = h;
    total_size_ = sz;
    fp_ = fp;
    map_file();

    if(h.version == LongWaterfall::FILE_VERSION_ZSTD){
        // v4: 헤더의 authoritative num_rows + footer index. size 산술 안 씀.
        const LongWaterfall::V4Ext& ext = LongWaterfall::v4ext(h);
        const uint32_t nb   = ext.num_blocks;
        const uint64_t ioff = ext.index_offset;
        const size_t   need = (size_t)nb * sizeof(LongWaterfall::HistBlockIndex);
        if(ext.codec != LongWaterfall::HIST_CODEC_ZSTD || ext.block_rows == 0 ||
           nb == 0 || ioff < sizeof(h) || ioff + need > sz){
            close(); return false;
        }
        is_v4_      = true;
        v4_flags_   = ext.flags;
        block_rows_ = ext.block_rows;
        num_rows_   = (uint32_t)ext.num_rows;
        index_.resize(nb);
        if(map_ && ioff + need <= map_size_){
            memcpy(index_.data(), map_ + ioff, need);
        } else if(fseek(fp_, (long)ioff, SEEK_SET) != 0 ||
                  fread(index_.data(), 1, need, fp_) != need){
            close(); return false;
        }
    } else {
        num_rows_ = (uint32_t)((sz - sizeof(h)) / h.fft_size);
    }
    return true;
}

bool HistReader::refresh_live(){
    if(!fp_) return false;
    if(is_v4_) return false;         // v4(완료·압축)는 성장하지 않음
    struct stat st{};
    if(stat(path_.c_str(), &st) != 0) return false;
    const uint64_t sz = (uint64_t)st.st_size;
    if(sz == total_size_) return false;
    total_size_ = sz;
    const uint32_t old_rows = num_rows_;
    num_rows_ = (uint32_t)((sz - sizeof(LongWaterfall::FileHeader)) / hdr_.fft_size);
    if(num_rows_ == old_rows) return false;
    // mmap 확장: LIVE 는 계속 자라므로 새 크기로 remap.
    if(fd_ >= 0){
        if(map_){ munmap((void*)map_, map_size_); map_ = nullptr; map_size_ = 0; }
        void* m = mmap(nullptr, sz, PROT_READ, MAP_SHARED, fd_, 0);
        if(m != MAP_FAILED){
            map_ = (const uint8_t*)m;
            map_size_ = sz;
            posix_madvise(m, sz, POSIX_MADV_RANDOM);
        }
    }
    return true;
}

const uint8_t* HistReader::get_row(uint32_t r){
    const uint32_t fft_sz = hdr_.fft_size;
    if(fft_sz == 0 || r >= num_rows_) return nullptr;

    if(!is_v4_){
        const uint64_t off = sizeof(LongWaterfall::FileHeader) + (uint64_t)r * fft_sz;
        if(map_ && off + fft_sz <= map_size_)
            return map_ + off;                       // mmap fast path (syscall 없음)
        if(!fp_) return nullptr;
        if(rowbuf_.size() != fft_sz) rowbuf_.resize(fft_sz);
        if(fseek(fp_, (long)off, SEEK_SET) != 0) return nullptr;
        if(fread(rowbuf_.data(), 1, fft_sz, fp_) != fft_sz) return nullptr;
        return rowbuf_.data();
    }

    // 선해제본이 있으면 해제 없이 바로 포인터.
    if(full_ && !full_->empty()){
        const size_t off = (size_t)r * fft_sz;
        if(off + fft_sz <= full_->size()) return full_->data() + off;
        return nullptr;
    }

    // v4: 블록 zstd 해제 + 단일 블록 캐시.
    if(block_rows_ == 0) return nullptr;
    const uint32_t block = r / block_rows_;
    if(block != (uint32_t)cache_block_){
        if(block >= index_.size()) return nullptr;
        const auto& e = index_[block];
        const uint8_t* src;
        if(map_ && e.frame_offset + e.comp_len <= map_size_){
            src = map_ + e.frame_offset;
        } else {
            if(!fp_) return nullptr;
            if(frame_buf_.size() < e.comp_len) frame_buf_.resize(e.comp_len);
            if(fseek(fp_, (long)e.frame_offset, SEEK_SET) != 0) return nullptr;
            if(fread(frame_buf_.data(), 1, e.comp_len, fp_) != e.comp_len) return nullptr;
            src = frame_buf_.data();
        }
        const size_t cap  = (size_t)block_rows_ * fft_sz;
        const size_t want = (size_t)e.raw_rows * fft_sz;
        if(cache_buf_.size() < cap) cache_buf_.resize(cap);
        const size_t d = ZSTD_decompress(cache_buf_.data(), cap, src, e.comp_len);
        if(ZSTD_isError(d) || d != want){ cache_block_ = -1; return nullptr; }
        if(v4_flags_ & LongWaterfall::HIST_V4_FLAG_COL_DELTA){
            // un-delta (freq): x[i]=d[i]+x[i-1] per row (블록 로드 1회만).
            for(uint32_t rr = 0; rr < e.raw_rows; rr++){
                uint8_t* row = cache_buf_.data() + (size_t)rr * fft_sz;
                for(uint32_t i = 1; i < fft_sz; i++) row[i] = (uint8_t)(row[i] + row[i-1]);
            }
        }
        cache_block_ = (int)block;
    }
    const uint32_t within = r - block * block_rows_;
    return cache_buf_.data() + (size_t)within * fft_sz;
}

bool HistReader::preload_full(uint64_t max_bytes){
    if(!is_v4_ || full_ready()) return full_ready();
    if(num_rows_ == 0 || hdr_.fft_size == 0) return false;
    const uint64_t need = (uint64_t)num_rows_ * hdr_.fft_size;
    if(need > max_bytes) return false;              // 과대 — 종전 블록캐시 유지

    auto buf = std::make_shared<std::vector<uint8_t>>();
    try { buf->resize((size_t)need); }
    catch(const std::bad_alloc&){ return false; }   // RAM 부족 — 종전 경로 유지

    // 블록 순서대로 해제 (get_row 의 블록캐시 경로 재사용 — full_ 이 아직 비어 있어
    // 재귀하지 않는다). 블록당 1회 해제로 전체를 채운다.
    const uint32_t fft_sz = hdr_.fft_size;
    for(uint32_t r = 0; r < num_rows_; r++){
        const uint8_t* p = get_row(r);
        if(!p) return false;                        // 손상/실패 — 선해제 포기
        memcpy(buf->data() + (size_t)r * fft_sz, p, fft_sz);
    }
    full_ = std::move(buf);
    // 블록캐시는 이제 안 쓰므로 메모리 반환.
    cache_buf_.clear(); cache_buf_.shrink_to_fit();
    cache_block_ = -1;
    return true;
}

HistReader HistReader::clone_for_thread() const {
    HistReader c;
    c.path_       = path_;
    c.hdr_        = hdr_;
    c.total_size_ = total_size_;
    c.num_rows_   = num_rows_;
    c.is_v4_      = is_v4_;
    c.v4_flags_   = v4_flags_;
    c.block_rows_ = block_rows_;
    c.index_      = index_;
    c.full_       = full_;          // 선해제본 공유 — 재해제 금지의 핵심

    // 선해제본이 있으면 파일 핸들이 아예 필요 없다 (v4 는 전 행이 RAM 에 있다).
    if(c.full_ && !c.full_->empty()) return c;

    // 없으면 자기 fd/fp/mmap 을 따로 연다 — 캐시 버퍼를 공유하면 안 되기 때문.
    c.fp_ = fopen(path_.c_str(), "rb");
    if(c.fp_) c.map_file();
    return c;
}
