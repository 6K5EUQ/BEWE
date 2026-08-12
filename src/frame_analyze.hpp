#pragma once
// ── 범용 프레임 구조 자동 추출 ────────────────────────────────────────────────
// 복조된 비트열 하나에서 프리앰블 / 프레임 주기 / 동기어 / 페이로드 길이를 뽑는다.
// **프로토콜을 모른 채** 하는 분석이다 — acars/ais/adsb/btle/dmr/wifi 처럼 규격을
// 아는 신호는 각자 전용 디코더가 있고, 이건 그게 없는 미지 신호용이다.
//
// 입력은 바이트당 1비트(0/1), 인덱스 = 시간순 비트번호. ui.cpp 의 Bits 뷰가 쓰는
// bits_cache 와 같은 표현이라 그대로 넘길 수 있다.
//
// 의존이 없다 (표준 헤더뿐). ImGui 도 모듈도 안 부르므로 tools/ 에서 직접
// 컴파일해 회귀를 돌릴 수 있고, 코어가 모듈 헤더를 include 하면 안 된다는
// 규칙(module_api.hpp)에도 걸리지 않는다.
//
// ── 무엇을 근거로 판정하나 ─────────────────────────────────────────────────
// 프레임 구조가 있는 신호는 같은 비트열이 주기적으로 되풀이된다. 그 되풀이가
// 유일한 단서다:
//
//   1) 프레임 주기 L — 비트열을 L 만큼 밀어 자기 자신과 겹쳤을 때 불일치가
//      최소가 되는 L. 프레임이 반복되면 프리앰블·동기어 자리가 정확히 포개진다.
//   2) 동기어 — L 로 프레임을 정렬한 뒤 **모든 프레임에서 값이 같은 비트**의
//      최장 연속 구간. 페이로드는 프레임마다 달라 여기서 끊긴다.
//   3) 프리앰블 — 선두의 짧은 주기 반복(0101…, 00110011… 등). 동기어와 달리
//      "값이 같다"가 아니라 "자기 안에서 되풀이된다"로 찾는다.
//   4) 페이로드 길이 — L 에서 프리앰블·동기어를 뺀 나머지.
//
// ── 극성은 두 번 볼 필요가 없다 ────────────────────────────────────────────
// 비트열은 전 비트가 반전된 채로 들어오기 일쑤다 (슬라이서 기준선이 반대쪽으로
// 잡히거나 복조 극성이 뒤집힌 경우). 그래서 반전본도 같이 돌리는 코드를 넣었다가
// 뺐다 — 위 네 판정이 **전부 보수(complement)에 불변**이기 때문이다:
//   · 일치율은 두 열을 같이 뒤집어도 그대로다 (a[i]==b[i] 가 보존된다)
//   · "모든 프레임에서 값이 같은 비트" 도 같은 이유로 그대로다
//   · 프리앰블의 반복 판정도 마찬가지
// 달라지는 건 보고되는 동기어 **값** 하나뿐이고, 어느 쪽이 "옳은" 극성인지는
// 프로토콜을 모르는 이상 알 수 없다. 그래서 받은 그대로의 값을 보고한다.
// (극성이 어긋나면 값이 보수로 나오는데, 규격을 아는 운용자는 그걸 바로 알아본다.)
//
// ── 프레임이 하나뿐이면 아무것도 보고하지 않는다 ────────────────────────────
// 주기와 동기어는 **반복**에서만 나온다. 한 프레임짜리 캡처에서 억지로 답을 내면
// 그건 잡음에 맞춘 숫자다. 반복이 없으면 period/sync 를 비워 둔다 — 운용자에게
// 틀린 값을 주느니 없다고 하는 편이 낫다.
#include <cstdint>
#include <cstddef>
#include <vector>
#include <algorithm>

namespace frame_analyze {

struct Result {
    // 프리앰블 (선두의 주기적 반복). len==0 = 미검출
    int  preamble_len  = 0;    // 비트 수
    int  preamble_unit = 0;    // 반복 단위 길이 (2 = 0101…)
    uint32_t preamble_pat = 0; // 단위 패턴, 최상위 비트가 먼저 온 비트

    // 프레임 주기. 0 = 미검출 (반복 없음)
    int   period      = 0;     // 비트
    float period_conf = 0.f;   // 0..1. 정렬했을 때의 일치율에서 나온다

    // 동기어. len==0 = 미검출
    int      sync_off = 0;     // 프레임 시작 기준 오프셋(비트)
    int      sync_len = 0;
    uint64_t sync_bits = 0;    // 최상위 비트가 먼저 온 비트 (sync_len ≤ 64).
                               // 입력 극성 그대로다 (위 "극성" 주석 참조).

    int  payload_len = 0;      // period - (프리앰블+동기어). 미검출이면 0
    int  frames      = 0;      // 입력에 들어 있던 프레임 수
};

namespace detail {

// 프레임 주기 탐색에서 요구하는 최소 프레임 수. 2개로는 우연한 일치를 못 가른다.
constexpr int MIN_FRAMES = 3;
// 정렬 후 이 비율 이상 맞아야 "같은 프레임"으로 본다. 무작위 비트열은 0.5 근처다.
constexpr double PERIOD_MATCH_MIN = 0.62;
// 동기어로 인정하는 최소 길이. 이보다 짧으면 우연히 겹친 비트와 구분이 안 된다.
constexpr int SYNC_MIN = 8;
// 프리앰블로 인정하는 최소 반복 횟수.
constexpr int PRE_MIN_REPS = 4;
// 프리앰블 단위 길이 상한. 이보다 길면 그건 프리앰블이 아니라 프레임 구조다.
constexpr int PRE_UNIT_MAX = 16;

// 두 구간의 일치 비율.
inline double agree(const uint8_t* a, const uint8_t* b, int n){
    if(n <= 0) return 0.0;
    int same = 0;
    for(int i=0;i<n;i++) same += (a[i] == b[i]);
    return (double)same / (double)n;
}

// ── 1) 프레임 주기 ────────────────────────────────────────────────────────
// 지연 L 로 겹쳤을 때 일치율이 가장 높은 L. L 이 참 주기의 배수여도 일치율이
// 높으므로 **가장 작은** L 을 택한다 — 안 그러면 두 프레임을 한 프레임으로 본다.
inline int find_period(const std::vector<uint8_t>& b, double& out_conf)
{
    out_conf = 0.0;
    const int n = (int)b.size();
    if(n < 64) return 0;
    const int max_p = n / MIN_FRAMES;     // 최소 MIN_FRAMES 개는 들어 있어야
    if(max_p < 16) return 0;

    int best = 0; double best_a = 0.0;
    for(int L=8; L<=max_p; L++){
        double a = agree(b.data(), b.data()+L, n-L);
        if(a > best_a){ best_a = a; best = L; }
    }
    if(best == 0 || best_a < PERIOD_MATCH_MIN) return 0;

    // 배수로 잡혔을 수 있다 — 약수 중에 거의 같은 일치율을 내는 게 있으면 그쪽이
    // 참 주기다. 일치율이 떨어지지 않는 한 계속 내려간다.
    for(int d=2; d<=best/8; d++){
        if(best % d) continue;
        int cand = best/d;
        double a = agree(b.data(), b.data()+cand, n-cand);
        if(a >= best_a - 0.02){ best = cand; best_a = a; d = 1; }
    }
    out_conf = (best_a - 0.5) / 0.5;             // 0.5(무작위) → 0, 1.0 → 1
    if(out_conf < 0.0) out_conf = 0.0;
    if(out_conf > 1.0) out_conf = 1.0;
    return best;
}

// ── 2) 동기어 ─────────────────────────────────────────────────────────────
// 주기 L 로 프레임을 쌓고, 모든 프레임에서 값이 같은 비트를 1 로 표시한 뒤
// 그 1 의 최장 연속 구간을 고른다. 프리앰블도 "항상 같음" 이라 함께 잡히므로
// 호출자가 프리앰블 길이만큼 건너뛴 위치부터 찾게 한다.
inline void find_sync(const std::vector<uint8_t>& b, int L, int skip,
                      int& out_off, int& out_len, uint64_t& out_bits)
{
    out_off = out_len = 0; out_bits = 0;
    const int n = (int)b.size();
    const int nf = n / L;
    if(nf < MIN_FRAMES || L <= skip) return;

    std::vector<uint8_t> stable((size_t)L, 1), val((size_t)L, 0);
    for(int i=0;i<L;i++) val[(size_t)i] = b[(size_t)i];
    for(int f=1; f<nf; f++){
        const uint8_t* p = b.data() + (size_t)f*L;
        for(int i=0;i<L;i++) if(p[i] != val[(size_t)i]) stable[(size_t)i] = 0;
    }

    int run = 0, best_len = 0, best_end = -1;
    for(int i=skip;i<L;i++){
        if(stable[(size_t)i]){ run++; if(run > best_len){ best_len = run; best_end = i; } }
        else run = 0;
    }
    if(best_len < SYNC_MIN) return;
    if(best_len > 64) best_len = 64;             // uint64_t 에 담기는 만큼만
    int off = best_end - best_len + 1;

    uint64_t bits = 0;
    for(int i=0;i<best_len;i++) bits = (bits<<1) | (uint64_t)(val[(size_t)(off+i)] & 1);
    out_off = off; out_len = best_len; out_bits = bits;
}

// ── 3) 프리앰블 ───────────────────────────────────────────────────────────
// 선두에서 단위 u 가 몇 번 되풀이되는지 본다. 가장 긴 반복을 내는 u 를 고르되,
// 같은 길이면 짧은 단위를 택한다 (0101… 을 단위 4 로 읽지 않기 위해).
inline void find_preamble(const std::vector<uint8_t>& b,
                          int& out_len, int& out_unit, uint32_t& out_pat)
{
    out_len = out_unit = 0; out_pat = 0;
    const int n = (int)b.size();
    if(n < 16) return;

    for(int u=1; u<=PRE_UNIT_MAX; u++){
        if(n < u*PRE_MIN_REPS) break;
        int reps = 1;
        while((reps+1)*u <= n &&
              agree(b.data(), b.data()+(size_t)reps*u, u) == 1.0) reps++;
        if(reps < PRE_MIN_REPS) continue;
        // 단위 안이 전부 같은 값이면(000000… ) 그건 반복이 아니라 그냥 상수다.
        bool varies = false;
        for(int i=1;i<u;i++) if(b[(size_t)i] != b[0]){ varies = true; break; }
        if(u > 1 && !varies) continue;
        int len = reps*u;
        if(len > out_len){
            out_len = len; out_unit = u;
            uint32_t pat = 0;
            for(int i=0;i<u && i<32;i++) pat = (pat<<1) | (uint32_t)(b[(size_t)i] & 1);
            out_pat = pat;
        }
    }
    // 단위 1 은 "같은 값이 계속" 이라 프리앰블로 치지 않는다.
    if(out_unit == 1){ out_len = out_unit = 0; out_pat = 0; }
}

} // namespace detail

// bits : 바이트당 1비트(0/1), 시간순
inline Result analyze(const std::vector<uint8_t>& bits)
{
    Result r;
    if(bits.size() < 64) return r;

    detail::find_preamble(bits, r.preamble_len, r.preamble_unit, r.preamble_pat);

    double conf = 0.0;
    r.period = detail::find_period(bits, conf);
    r.period_conf = (float)conf;
    if(r.period > 0){
        r.frames = (int)bits.size() / r.period;
        // 프리앰블은 프레임 안에서도 "항상 같음" 이라 동기어 탐색에 같이 걸린다.
        // 프레임 길이를 넘지 않는 선에서 그만큼 건너뛴다.
        int skip = std::min(r.preamble_len, r.period - 1);
        if(skip < 0) skip = 0;
        detail::find_sync(bits, r.period, skip, r.sync_off, r.sync_len, r.sync_bits);
        int used = r.preamble_len + r.sync_len;
        r.payload_len = (r.period > used) ? (r.period - used) : 0;
    }
    return r;
}

} // namespace frame_analyze
