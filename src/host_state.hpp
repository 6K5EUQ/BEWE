#pragma once
// ── HOST 상태 영속화 (재시작 시 직전 상태 그대로 복원) ─────────────────────
// 기지(headless cli_host)가 SIGINT 등으로 꺼졌다 켜져도 사용자 입장에서
// "잠깐 깜빡인" 것처럼 보이도록 center freq / sample rate / gain / 채널 필터를
// $HOME/BEWE/host_state_<station>.json 에 저장하고 부팅 시 복원한다.
//   - 저장: 변경 시마다 (메인 루프에서 fingerprint 비교 → 바뀌면 write)
//   - 복원: cf/sr/gain 은 initialize 단계, 채널은 net_srv 기동 후
// Central 은 stateless relay 라 상태 보관 불가 — 각 HOST 로컬에만 저장.

#include "net_protocol.hpp"   // PktDfConfig — DF 설정 전체를 그대로 저장한다
#include <string>
#include <cstdint>
#include "config.hpp"   // MAX_CHANNELS

class FFTViewer;

namespace HostState {

// $HOME/BEWE/host_state_<station>.json
std::string file_path(const std::string& station);

struct ChanSnap {
    float    s = 0, e = 0;            // 필터 경계 (절대 MHz)
    int      mode = 0;               // Channel::DemodMode (0=NONE,1=AM,2=FM)
    char     owner[32] = {};
    uint32_t audio_mask = 0xFFFFFFFFu;
    int      pan = 0;
    float    sq = -50.0f;
    bool     sq_manual = false;      // 사용자가 직접 조정한 값? (구버전 파일엔 없음 → 자동으로 취급)
    char     decode_mods[64] = {};   // 이 채널에 켜진 디코드 모듈 id (콤마구분, 예 "wifi,acars")
    // 디텍션 필터 (구버전 파일엔 없음 → det_on=false 로 취급, 기존 AM/FM 채널처럼 복원됨)
    bool     det_on = false;         // 저장 시점에 armed 였는지 (lock 여부와 무관)
    float    det_s = 0, det_e = 0;   // 탐색 대역 (s/e 는 lock 중이면 좁아진 현재 폭)
};

struct Snapshot {
    bool     ok = false;             // 파일이 있고 파싱됨
    float    cf_mhz = 0;
    float    sr_msps = 0;            // 0 = 미지정 (HW 기본 사용)
    bool     has_gain = false;
    float    gain_db = 0;
    int      n_chans = 0;
    ChanSnap chans[MAX_CHANNELS];
    // DF(방탐) 설정. 구버전 파일엔 없으므로 has_df=false 로 남고 기본값이 쓰인다.
    bool        has_df = false;
    // enable_control 은 bool 이라 0 이 유효값이다 — "파일에 없음"과 "사용자가 끔"을
    // 값만 보고는 구분할 수 없다. 키를 실제로 봤는지 따로 기록한다.
    bool        has_df_enable_control = false;
    PktDfConfig df{};      // 전체 DF 설정. 필드가 늘어도 여기만 손대면 된다.
};

// 현재 v 의 persistent 상태를 station 별 파일에 기록 (tmp+rename 으로 원자적).
void save(const FFTViewer& v, const std::string& station);

// 디스크에서 읽기 (파일 없으면 ok=false).
Snapshot load(const std::string& station);

// persistent 필드 fingerprint — 변경 감지용. dem_paused 같은 런타임 상태는 제외.
uint64_t fingerprint(const FFTViewer& v);

// 스냅샷의 채널들을 v.channels[] 에 복원 + 복조 시작 + 범위 판정.
// (CF/SR/gain 은 initialize 단계에서 이미 적용됨 — 여기선 채널만)
void apply_channels(FFTViewer& v, const Snapshot& st);

// DF 설정 복원. 채널과 분리한 이유는 SDR 백엔드와 무관하게 항상 적용해야 하기
// 때문이다 (Kraken 이 아니어도 설정은 보존된다).
void apply_df(FFTViewer& v, const Snapshot& st);

} // namespace HostState
