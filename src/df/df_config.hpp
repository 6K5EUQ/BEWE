#pragma once
// ── DF 설정 ───────────────────────────────────────────────────────────────
// 설정 패널이 노출하는 값 전부. HostState 로 영속화된다.

#include "df_types.hpp"
#include <cstdint>
#include <cstring>

namespace df {

struct Config {
    // ── DAQ 접속 ─────────────────────────────────────────────────────────
    char     host[64]  = "127.0.0.1";
    uint16_t data_port = 5000;   // iq_server.out
    uint16_t ctrl_port = 5001;   // hw_controller.py
    // :5001 로 FREQ/GAIN 을 보낼지. BEWE 주파수축/상단바에서 재튠하는 게 정상
    // 사용법이라 기본 켬. 재튠 1회당 heimdall 이 STATE_INIT 으로 돌아가 수 초간
    // DF 가 불가해지지만, 그 구간은 DF 램프가 노랑으로 알려준다. 드래그 폭주는
    // 캡처 루프의 1초 코얼레싱이 막는다.
    bool     enable_control = true;

    // ── 배열 기하 ────────────────────────────────────────────────────────
    int    elements  = 5;        // UCA 소자 수 = DAQ 채널 수여야 한다
    double radius_m  = 0.175;    // 원 반경(미터). 반경이지 소자간 거리가 아니다.
    Sense  sense     = Sense::CW;
    double heading_deg = 0.0;    // 배열 0° 가 기수와 안 맞을 때 더할 값

    // ── 추정 ─────────────────────────────────────────────────────────────
    Algo algo       = Algo::Music;
    int  signal_dim = 1;
    int  avg_frames = 3;         // 기본 3 x 437ms ~= 1.3초
    int  max_frames = 12;        // CAL 버스트를 만나면 여기까지 늘린다

    // ── 수락 규칙 ────────────────────────────────────────────────────────
    // 두 단계다.
    //  (1) 통계적 바닥: Bartlett PAPR 이 잡음 귀무분포를 넘는가.
    //      임계 = c_papr / sqrt(n_eff) — look 수가 달라져도 재튜닝이 필요 없다.
    //      이건 "잡음에 대고 임의 각도를 보고하는" 최악의 실패를 막는 안전장치라
    //      운용자에게 노출하지 않는다.
    //  (2) 운용자 임계: 고유값비 SNR 이 snr_threshold_db 이상인가.
    //      DF 탭에서 조정한다. HOST 소유 값이고 하트비트로 JOIN 에도 공유된다.
    double c_papr          = 30.0;
    double snr_threshold_db = 10.0;   // 관측상 실신호 18~23 dB, 잡음은 음수

    // ── 스펙트럼 추출 ────────────────────────────────────────────────────
    double dc_guard_hz  = 2000.0;   // LO 누설 배제 반폭
    int    target_looks = 2048;     // 프레임당 목표 look 수. 비용을 여기서 잡는다
    int    fft_size     = 8192;     // 기본 K. 채널이 좁으면 엔진이 키운다

    bool validate(char* err, size_t n) const {
        auto fail = [&](const char* m){ if(err && n) { strncpy(err, m, n-1); err[n-1]=0; } return false; };
        if(elements   < 3 || elements   > 8)    return fail("elements must be 3..8");
        if(radius_m  <= 0.0 || radius_m  > 5.0) return fail("radius_m must be in (0, 5]");
        if(signal_dim < 1 || signal_dim >= elements) return fail("signal_dim must be 1..elements-1");
        if(avg_frames < 1 || avg_frames > 60)   return fail("avg_frames must be 1..60");
        if(max_frames < avg_frames)             return fail("max_frames must be >= avg_frames");
        if(snr_threshold_db < -20.0 || snr_threshold_db > 60.0)
            return fail("snr_threshold_db must be in [-20, 60]");
        // 다른 필드는 전부 경계가 있는데 이것만 빠져 있었다. pkt_to_cfg 가
        // 와이어 값을 무클램프로 통과시키므로 validate 가 유일한 관문이다 —
        // JOIN 이 0 을 밀면 papr_thr 이 0 이 되어 통계 바닥이 완전히 꺼진다
        // (= 순수 잡음에 임의 방위를 보고). 이 상수가 존재하는 이유 그 자체다.
        if(c_papr < 1.0 || c_papr > 1000.0)     return fail("c_papr must be in [1, 1000]");
        if(target_looks < 64)                   return fail("target_looks must be >= 64");
        if(fft_size < 256 || (fft_size & (fft_size-1))) return fail("fft_size must be a power of two >= 256");
        if(dc_guard_hz < 0.0)                   return fail("dc_guard_hz must be >= 0");
        if(data_port == 0 || ctrl_port == 0)    return fail("ports must be non-zero");
        return true;
    }
};

} // namespace df
