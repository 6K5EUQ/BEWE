// 프레임 구조 추출 회귀 하네스 (합성 비트열, 하드웨어 불필요).
//   g++ -O2 -I src -o /tmp/frmtest tools/frame_test.cpp && /tmp/frmtest
// 실패 개수를 종료코드로 낸다.
#include "frame_analyze.hpp"
#include <cstdio>
#include <cstdarg>
#include <random>

static int g_fail = 0, g_run = 0;
static void check(bool ok, const char* fmt, ...) __attribute__((format(printf,2,3)));
static void check(bool ok, const char* fmt, ...){
    g_run++; if(!ok) g_fail++;
    va_list ap; va_start(ap, fmt);
    printf(ok ? "  PASS  " : "  FAIL  ");
    vprintf(fmt, ap); printf("\n");
    va_end(ap);
}

// 프레임 = [프리앰블 0101…][동기어][난수 페이로드] 를 nfr 번 반복.
static std::vector<uint8_t> make_frames(int pre_len, uint64_t sync, int sync_len,
                                        int payload_len, int nfr, uint32_t seed)
{
    std::mt19937 rng(seed);
    std::vector<uint8_t> b;
    for(int f=0; f<nfr; f++){
        for(int i=0;i<pre_len;i++)   b.push_back((uint8_t)(i & 1));           // 0101…
        for(int i=0;i<sync_len;i++)  b.push_back((uint8_t)((sync >> (sync_len-1-i)) & 1));
        for(int i=0;i<payload_len;i++) b.push_back((uint8_t)(rng() & 1));
    }
    return b;
}

static void t_frames(const char* nm, const std::vector<uint8_t>& b,
                     int want_pre, uint64_t want_sync, int want_sync_len,
                     int want_period, int want_payload)
{
    frame_analyze::Result r = frame_analyze::analyze(b);
    bool ok = r.preamble_len == want_pre
           && r.period       == want_period
           && r.sync_len     == want_sync_len
           && r.sync_bits    == want_sync
           && r.payload_len  == want_payload;
    check(ok, "%-30s pre=%d/%d period=%d/%d sync=%d bits=0x%llX/0x%llX pay=%d/%d",
          nm, r.preamble_len, want_pre, r.period, want_period,
          r.sync_len, (unsigned long long)r.sync_bits, (unsigned long long)want_sync,
          r.payload_len, want_payload);
}

int main(){
    printf("=== frame_analyze selftest ===\n");

    // 기본형 — 프리앰블 16, 동기어 24비트, 페이로드 64, 8프레임
    {
        auto b = make_frames(16, 0x9A7D2CULL, 24, 64, 8, 111);
        t_frames("basic 16/24/64 x8", b, 16, 0x9A7D2CULL, 24, 104, 64);
    }
    // 극성 불변성 — 전 비트를 뒤집어도 구조 판정이 하나도 안 바뀌고, 동기어 값만
    // 보수로 나와야 한다. 이 성질이 성립하므로 양극성 2회 분석이 불필요하다
    // (frame_analyze.hpp 의 "극성" 주석). 성질이 깨지면 여기서 잡힌다.
    {
        auto b0 = make_frames(16, 0x9A7D2CULL, 24, 64, 8, 111);
        auto b1 = b0;
        for(auto& v : b1) v = (uint8_t)(v ? 0 : 1);
        frame_analyze::Result r0 = frame_analyze::analyze(b0);
        frame_analyze::Result r1 = frame_analyze::analyze(b1);
        uint64_t mask = (r0.sync_len >= 64) ? ~0ULL : ((1ULL<<r0.sync_len)-1ULL);
        bool ok = r0.preamble_len == r1.preamble_len
               && r0.period       == r1.period
               && r0.sync_len     == r1.sync_len
               && r0.sync_off     == r1.sync_off
               && r0.payload_len  == r1.payload_len
               && r1.sync_bits    == ((~r0.sync_bits) & mask);
        check(ok, "%-30s struct identical, sync 0x%llX -> 0x%llX",
              "polarity invariance", (unsigned long long)r0.sync_bits,
              (unsigned long long)r1.sync_bits);
    }
    // 짧은 프리앰블 / 긴 동기어 / 큰 페이로드
    {
        auto b = make_frames(8, 0xFEEDBEEFULL, 32, 128, 6, 222);
        t_frames("8/32/128 x6", b, 8, 0xFEEDBEEFULL, 32, 168, 128);
    }
    // 프리앰블 없음 — 동기어와 주기는 여전히 나와야 한다.
    // 동기어는 실제 규격들처럼 **비주기적**인 것을 쓴다 (CCSDS ASM 0x1ACFFC1D).
    // 0xA5A5A5A5 같은 반복 패턴은 그 자체가 프리앰블과 구분되지 않는다 — 그건
    // 분석기 결함이 아니라 그런 동기어를 쓰지 않는 이유다.
    {
        auto b = make_frames(0, 0x1ACFFC1DULL, 32, 96, 8, 333);
        frame_analyze::Result r = frame_analyze::analyze(b);
        check(r.period == 128 && r.sync_len == 32 && r.sync_bits == 0x1ACFFC1DULL,
              "%-30s pre=%d period=%d sync=%d bits=0x%llX",
              "no preamble", r.preamble_len, r.period, r.sync_len,
              (unsigned long long)r.sync_bits);
    }

    // ── 오검출이 나면 안 되는 것들 ─────────────────────────────────────────
    {
        std::mt19937 rng(999);
        std::vector<uint8_t> b(4096);
        for(auto& v : b) v = (uint8_t)(rng() & 1);
        frame_analyze::Result r = frame_analyze::analyze(b);
        check(r.period == 0 && r.sync_len == 0,
              "%-30s period=%d sync=%d (want 0/0)", "random bits", r.period, r.sync_len);
    }
    {
        // 프레임 하나뿐 — 반복이 없으므로 주기·동기어를 보고하면 안 된다
        auto b = make_frames(16, 0x9A7D2CULL, 24, 64, 1, 444);
        frame_analyze::Result r = frame_analyze::analyze(b);
        check(r.period == 0 && r.sync_len == 0,
              "%-30s period=%d sync=%d (want 0/0)", "single frame", r.period, r.sync_len);
    }
    {
        std::vector<uint8_t> b(512, 0);            // 상수 0 — 프리앰블이 아니다
        frame_analyze::Result r = frame_analyze::analyze(b);
        check(r.preamble_len == 0,
              "%-30s pre=%d (want 0)", "all zeros", r.preamble_len);
    }
    {
        std::vector<uint8_t> tiny(32, 0);
        frame_analyze::Result r = frame_analyze::analyze(tiny);
        check(r.period == 0 && r.preamble_len == 0 && r.sync_len == 0,
              "%-30s (want all 0)", "too short");
    }

    printf("=== %d/%d passed ===\n", g_run-g_fail, g_run);
    return g_fail;
}
