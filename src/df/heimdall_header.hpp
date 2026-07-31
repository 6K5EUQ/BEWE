#pragma once
// ── heimdall DAQ IQ 프레임 헤더 (1024 바이트) ─────────────────────────────
//
// 이 구조체는 heimdall(GPLv3)의 소스를 복사한 게 아니라, 그 바이너리 프로토콜과
// 대화하기 위해 필드 이름·타입·오프셋이라는 "사실"만 보고 독립적으로 작성했다.
// 일치 여부는 베끼는 대신 아래 static_assert 로 컴파일 타임에 검증한다.
//
// 레이아웃은 x86-64/ARM64 자연 정렬 기준이고, 모든 필드는 호스트 엔디언
// (리틀엔디언)이다. 네트워크 바이트 오더가 아니다.
//
// 실측 확인 (2026-07-31, tcp://127.0.0.1:5000):
//   hardware_id="kraken5" header_version=7 active_ant_chs=5
//   rf_center_freq=700000000 adc_sampling_freq=sampling_freq=2400000
//   cpi_length=1048576 data_type=3 sample_bit_depth=32

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace df {

inline constexpr uint32_t kSyncWord     = 0x2BF7B95Au;
inline constexpr size_t   kHeaderBytes  = 1024;
inline constexpr uint32_t kHeaderVersion = 7;

enum FrameType : uint32_t {
    FRAME_DATA  = 0,
    FRAME_DUMMY = 1,   // cpi_length == 0, 페이로드 없음. 모든 제어 동작 뒤 5개
    FRAME_RAMP  = 2,   // 테스트 생성기 전용. 이 체인은 안 만든다
    FRAME_CAL   = 3,   // 노이즈 소스 주입 중. 안테나 신호가 아니다 — DF 에서 제외
    FRAME_TRIGW = 4,
};

// delay_sync 의 상태기계. 6 = STATE_TRACK = 완전 캘리브레이션 후 추적 중.
inline constexpr uint32_t kSyncStateTrack = 6;

#pragma pack(push, 1)
struct IqHeader {
    uint32_t sync_word;              //   0
    uint32_t frame_type;             //   4
    char     hardware_id[16];        //   8  NUL 패딩
    uint32_t unit_id;                //  24
    uint32_t active_ant_chs;         //  28  M
    uint32_t ioo_type;               //  32
    uint32_t _pad0;                  //  36  uint64 정렬용
    uint64_t rf_center_freq;         //  40  Hz
    uint64_t adc_sampling_freq;      //  48  Hz
    uint64_t sampling_freq;          //  56  Hz (데시메이션 후)
    uint32_t cpi_length;             //  64  채널당 샘플 수
    uint32_t _pad1;                  //  68
    uint64_t time_stamp;             //  72  unix epoch ms
    uint32_t daq_block_index;        //  80
    uint32_t cpi_index;              //  84  단조 증가. 드롭 검출용
    uint64_t ext_integration_cntr;   //  88
    uint32_t data_type;              //  96  3 = decimated CF32
    uint32_t sample_bit_depth;       // 100  32
    uint32_t adc_overdrive_flags;    // 104  bit m = 채널 m 클리핑
    uint32_t if_gains[32];           // 108  0.1 dB 단위
    uint32_t delay_sync_flag;        // 236
    uint32_t iq_sync_flag;           // 240
    uint32_t sync_state;             // 244
    uint32_t noise_source_state;     // 248
    uint32_t reserved[192];          // 252
    uint32_t header_version;         //1020
};
#pragma pack(pop)

// 오프셋을 하나씩 못 박는다. heimdall 이 필드를 끼워넣으면 여기서 빌드가 깨지고,
// 런타임에 조용히 엉뚱한 값을 읽는 사고가 안 난다.
static_assert(sizeof(IqHeader) == kHeaderBytes, "IqHeader must be exactly 1024 bytes");
#define DF_OFF(f, n) static_assert(offsetof(IqHeader, f) == (n), "offset of " #f)
DF_OFF(sync_word,             0);
DF_OFF(frame_type,            4);
DF_OFF(hardware_id,           8);
DF_OFF(unit_id,              24);
DF_OFF(active_ant_chs,       28);
DF_OFF(ioo_type,             32);
DF_OFF(rf_center_freq,       40);
DF_OFF(adc_sampling_freq,    48);
DF_OFF(sampling_freq,        56);
DF_OFF(cpi_length,           64);
DF_OFF(time_stamp,           72);
DF_OFF(daq_block_index,      80);
DF_OFF(cpi_index,            84);
DF_OFF(ext_integration_cntr, 88);
DF_OFF(data_type,            96);
DF_OFF(sample_bit_depth,    100);
DF_OFF(adc_overdrive_flags, 104);
DF_OFF(if_gains,            108);
DF_OFF(delay_sync_flag,     236);
DF_OFF(iq_sync_flag,        240);
DF_OFF(sync_state,          244);
DF_OFF(noise_source_state,  248);
DF_OFF(reserved,            252);
DF_OFF(header_version,     1020);
#undef DF_OFF

// 페이로드 크기. DUMMY 는 cpi_length==0 이라 0 이 되고, 그때 서버는 헤더만 보낸다.
// 레이아웃: channel-major, [ch0: I0 Q0 I1 Q1 ...][ch1: ...] ..., 각 성분 float32.
inline uint64_t payload_bytes(const IqHeader& h){
    return (uint64_t)h.cpi_length * h.active_ant_chs * 8ull;
}

inline bool sync_ok(const IqHeader& h){ return h.sync_word == kSyncWord; }

// DF 에 쓸 수 있는 프레임인가.
//   - DATA 여야 한다 (CAL 은 노이즈 소스, DUMMY 는 알맹이가 없다)
//   - 정수 샘플지연 · 진폭/위상 보정이 둘 다 성립해야 한다
//   - 노이즈 소스가 꺼져 있어야 한다 (켜져 있으면 안테나를 안 보는 것)
// sync_state==6 은 별도로 본다 — 여기서 요구하면 재캘리브레이션 직후
// 몇 프레임을 쓸데없이 버리게 되고, 위 플래그가 이미 그 조건을 담고 있다.
inline bool usable_for_df(const IqHeader& h){
    return h.frame_type == FRAME_DATA
        && h.delay_sync_flag == 1
        && h.iq_sync_flag == 1
        && h.noise_source_state == 0
        && h.cpi_length > 0
        && h.active_ant_chs >= 2;
}

inline const char* frame_type_name(uint32_t t){
    switch(t){
        case FRAME_DATA:  return "DATA";
        case FRAME_DUMMY: return "DUMMY";
        case FRAME_RAMP:  return "RAMP";
        case FRAME_CAL:   return "CAL";
        case FRAME_TRIGW: return "TRIGW";
        default:          return "?";
    }
}

} // namespace df
