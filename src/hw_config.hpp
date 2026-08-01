#pragma once
#include "config.hpp"   // *_RX_GAIN 기본값
#include <cstdint>
#include <cstdlib>  // abs(int)

// ── 하드웨어 타입 ──────────────────────────────────────────────────────────
// KRAKEN 은 BEWE 가 USB 장치를 직접 열지 않는 유일한 타입이다. heimdall DAQ
// (rtl_daq.out) 가 동글 5개를 소유하고, BEWE 는 TCP :5000 으로 위상보정된
// 5채널 IQ 를 받아 ch0 만 스펙트럼/워터폴/복조에 쓴다. 나머지는 DF 전용.
// 새 값은 반드시 끝에 추가할 것 — 와이어는 파생 hw_type 바이트라 안전하다.
enum class HWType { NONE, BLADERF, RTLSDR, PLUTO, KRAKEN };

// ── 런타임 HW 파라미터 (초기화 시 채워짐) ────────────────────────────────
struct HWConfig {
    HWType   type            = HWType::NONE;

    // 샘플링
    uint32_t sample_rate     = 0;       // Hz (실제 설정값)
    float    sample_rate_mhz = 0.0f;    // sample_rate / 1e6

    // 주파수 범위
    double   freq_min_hz     = 0.0;
    double   freq_max_hz     = 0.0;

    // IQ 스케일: BladeRF SC16_Q11 = 2048.0, RTL-SDR uint8 offset = 128
    float    iq_scale        = 2048.0f; // int16→float 정규화
    float    iq_offset       = 0.0f;    // uint8 중심값 (RTL=127.5, BladeRF=0)

    // 유효 대역폭 비율. v4.4.2 — full nyquist (1.0) 로 통일.
    // 가장자리 12.5% 도 실 운용 시 노이즈/진폭 약함 큰 문제 없어 채널 demod 도 허용.
    // (이전엔 0.875 — 가장자리에 채널 두면 update_dem_by_freq 가 Holding 처리)
    float    eff_bw_ratio    = 1.0f;

    // 캡처가 ring 에 한 번에 밀어넣는 최대 샘플 수 (0 = 작은 버퍼 연속 공급).
    // demod/decode 워커의 lag 리미터가 "과부하"와 "정상 버스트"를 구별하는 데 쓴다.
    // KrakenSDR 은 heimdall 이 1,048,576 샘플(2.4 MSPS 에서 437 ms)을 통째로 주므로
    // 이 값을 모르면 워커가 프레임마다 리미터를 때려 95% 를 버린다 (실측 audio 1.2 KB/s).
    uint32_t burst_samples   = 0;
    // 표시용 이름
    const char* name         = "Unknown";

    // ── 게인 범위 ──────────────────────────────────────────────────────────
    float    gain_min        = 0.0f;    // dB
    float    gain_max        = 49.6f;   // dB
    float    gain_default    = 0.0f;    // dB (초기값)

    // RTL-SDR R828D 이산 게인값 (0.1dB 단위 → /10 = dB)
    // librtlsdr에서 0.1dB 단위 정수 배열로 반환
    static constexpr int RTL_GAIN_STEPS = 29;
    static constexpr int RTL_GAINS_TENTHS[RTL_GAIN_STEPS] = {
        0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166,
        197, 207, 229, 254, 280, 297, 328, 338, 364,
        372, 386, 402, 421, 434, 439, 445, 480, 496
    };

    // tenths → 가장 가까운 이산 스텝의 **인덱스** (와이어로 1바이트에 싣는 값).
    static int rtl_gain_index(int tenths){
        int best = 0, best_diff = abs(tenths - RTL_GAINS_TENTHS[0]);
        for(int i=1;i<RTL_GAIN_STEPS;i++){
            int d = abs(tenths - RTL_GAINS_TENTHS[i]);
            if(d < best_diff){ best_diff=d; best=i; }
        }
        return best;
    }
    // 인덱스 → tenths (범위 밖이면 클램프)
    static int rtl_gain_tenths_at(int idx){
        if(idx < 0) idx = 0;
        if(idx >= RTL_GAIN_STEPS) idx = RTL_GAIN_STEPS - 1;
        return RTL_GAINS_TENTHS[idx];
    }

    // 연속 dB 값 → RTL-SDR 가장 가까운 이산값(0.1dB 단위 정수) 반환
    static int rtl_snap_gain(float db){
        int tenths = (int)(db * 10.0f + 0.5f);
        int best = RTL_GAINS_TENTHS[0], best_diff = abs(tenths - best);
        for(int i=1;i<RTL_GAIN_STEPS;i++){
            int d = abs(tenths - RTL_GAINS_TENTHS[i]);
            if(d < best_diff){ best_diff=d; best=RTL_GAINS_TENTHS[i]; }
        }
        return best;
    }
    // 워터폴 행 속도를 HW에 관계없이 동일하게 유지 (37.5 rows/sec 기준)
    // 워터폴 갱신 속도 (1초당 행 수). 18 Hz 이상이면 인간 시각에 부드럽게 보임.
    // 네트워크 트래픽 절약을 위해 기본 18 Hz. 너무 낮추면 (<10) 끊겨 보임.
    static constexpr float TARGET_ROWS_PER_SEC = 18.0f;

    float eff_bw_mhz() const { return sample_rate_mhz * eff_bw_ratio; }
    float nyq_mhz()    const { return sample_rate_mhz * 0.5f; }

    // fft_size에 맞는 time_average 자동 계산
    // Pluto는 USB 2.0 실효 ~8 MSPS라 명목 SR이 그 이상이면 ta가 과도해져서 row 갱신이 느려짐.
    // 실효 throughput 기준으로 계산해 워터폴/파워스펙트럼 갱신 속도를 일정하게 유지.
    int compute_time_average(int fft_sz) const {
        uint32_t eff_sr = sample_rate;
        if(type == HWType::PLUTO && eff_sr > 8000000u) eff_sr = 8000000u;
        int ta = (int)((float)eff_sr / (float)fft_sz / TARGET_ROWS_PER_SEC);
        return ta < 1 ? 1 : ta;
    }
};

// BladeRF 기본값
inline HWConfig make_bladerf_config(uint32_t actual_sr){
    HWConfig c;
    c.type            = HWType::BLADERF;
    c.sample_rate     = actual_sr;
    c.sample_rate_mhz = actual_sr / 1e6f;
    c.freq_min_hz     = 47e6;
    c.freq_max_hz     = 6000e6;
    c.iq_scale        = 2048.0f;
    c.iq_offset       = 0.0f;
    c.eff_bw_ratio    = 1.0f;
    c.name            = "BladeRF";
    c.gain_min        = 0.0f;
    c.gain_max        = 60.0f;
    c.gain_default    = (float)BLADERF_RX_GAIN;
    return c;
}

// ADALM-Pluto (AD936x) 기본값
inline HWConfig make_pluto_config(uint32_t actual_sr){
    HWConfig c;
    c.type            = HWType::PLUTO;
    c.sample_rate     = actual_sr;
    c.sample_rate_mhz = actual_sr / 1e6f;
    c.freq_min_hz     = 70e6;       // firmware hack 없으면 325MHz
    c.freq_max_hz     = 6000e6;     // firmware hack 없으면 3800MHz
    c.iq_scale        = 2048.0f;    // libiio 12-bit signed
    c.iq_offset       = 0.0f;
    c.eff_bw_ratio    = 1.0f;
    c.name            = "ADALM-Pluto";
    c.gain_min        = 0.0f;       // AD9363 manual gain 0..71
    c.gain_max        = 71.0f;
    c.gain_default    = (float)PLUTO_RX_GAIN_DB;
    return c;
}

// 녹음 Recorder 필드용 장비 표시명 (.info 파일)
inline const char* hw_recorder_name(HWType t){
    switch(t){
        case HWType::BLADERF: return "BladeRF 2.0 micro xA9 (12bit ADC)";
        case HWType::PLUTO:   return "ADALM Pluto SDR (12bit ADC)";
        case HWType::RTLSDR:  return "RTL-SDR v4 (8bit ADC)";
        case HWType::KRAKEN:  return "KrakenSDR 5ch coherent (8bit ADC, heimdall DAQ)";
        default:              return "";
    }
}

// RTL-SDR 기본값 (2.56 MSPS)
inline HWConfig make_rtlsdr_config(uint32_t actual_sr){
    HWConfig c;
    c.type            = HWType::RTLSDR;
    c.sample_rate     = actual_sr;
    c.sample_rate_mhz = actual_sr / 1e6f;
    c.freq_min_hz     = 500e3;
    c.freq_max_hz     = 1766e6;
    c.iq_scale        = 127.5f;
    c.iq_offset       = 127.5f;
    c.eff_bw_ratio    = 1.0f;
    c.name            = "RTL-SDR";
    c.gain_min        = 0.0f;
    c.gain_max        = 49.6f;
    c.gain_default    = (float)RTLSDR_RX_GAIN_TENTHS / 10.0f;
    return c;
}

// KrakenSDR / heimdall DAQ (TCP :5000). 샘플레이트는 heimdall 의
// daq_chain_config.ini 소유라 여기서 바꿀 수 없다 — 헤더가 준 값을 그대로 쓴다.
//
// iq_scale 2048 인 이유: heimdall 은 공칭 ±1.0 의 complex float32 를 주는데
// ring/TM/demod/IQ녹음은 전부 int16 interleaved 를 전제한다. BladeRF/Pluto 와
// 같은 SC16_Q11 스케일로 맞춰야 dB 눈금·스퀠치 임계·녹음 파일이 백엔드마다
// 달라지지 않는다. ±1.0 → ±2048 은 int16 포화까지 24 dB 여유.
inline HWConfig make_kraken_config(uint32_t actual_sr){
    HWConfig c;
    c.type            = HWType::KRAKEN;
    c.sample_rate     = actual_sr;
    c.sample_rate_mhz = actual_sr / 1e6f;
    c.freq_min_hz     = 24e6;       // R820T2. heimdall 은 direct sampling 을 안 쓴다
    c.freq_max_hz     = 1766e6;
    c.iq_scale        = 2048.0f;
    c.iq_offset       = 0.0f;
    c.eff_bw_ratio    = 1.0f;
    // heimdall 프레임 = 1,048,576 샘플. 워커 lag 리미터가 이걸 정상으로 봐야 한다.
    c.burst_samples   = 1u << 20;
    c.name            = "KrakenSDR";
    c.gain_min        = 0.0f;
    c.gain_max        = 49.6f;
    c.gain_default    = (float)RTLSDR_RX_GAIN_TENTHS / 10.0f;
    return c;
}