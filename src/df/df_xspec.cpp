#include "df_xspec.hpp"

#include <fftw3.h>
#include <cmath>
#include <mutex>
#include <algorithm>

namespace df {

namespace {
constexpr double kPi = 3.14159265358979323846;

// FFTW 플랜 생성은 스레드 안전하지 않다. BEWE 본체도 같은 이유로 자체 뮤텍스를
// 쓴다. 여기서 그쪽 헬퍼를 부르지 않는 건 df/ 를 단독으로 링크해 테스트할 수
// 있게 유지하려는 것 — 대신 프로세스에 이미 로드된 wisdom 은 그대로 쓴다
// (같은 크기 8192 를 BEWE 스펙트럼 경로가 이미 학습해 둔다).
std::mutex g_plan_mtx;

fftwf_plan make_plan(int n, fftwf_complex* in, fftwf_complex* out){
    std::lock_guard<std::mutex> lk(g_plan_mtx);
    if(fftwf_plan p = fftwf_plan_dft_1d(n, in, out, FFTW_FORWARD,
                                        FFTW_WISDOM_ONLY | FFTW_MEASURE))
        return p;
    return fftwf_plan_dft_1d(n, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
}
} // namespace

XSpec::~XSpec(){ destroy_plan(); }

void XSpec::destroy_plan(){
    std::lock_guard<std::mutex> lk(g_plan_mtx);
    if(plan_){ fftwf_destroy_plan((fftwf_plan)plan_); plan_ = nullptr; }
    if(fft_in_){  fftwf_free((fftwf_complex*)fft_in_);  fft_in_  = nullptr; }
    if(fft_out_){ fftwf_free((fftwf_complex*)fft_out_); fft_out_ = nullptr; }
}

double XSpec::n_eff() const {
    // Hann 창은 인접 빈을 상관시킨다 (등가잡음대역폭 ~1.5 빈). 독립 look 수는
    // 그만큼 적다. 수락 임계가 1/sqrt(n_eff) 로 걸리므로 여기서 과대평가하면
    // 오경보가 늘어난다 — 보수적으로 나눈다.
    return (double)bins_.size() * (double)segs_done_ / 1.5;
}

double XSpec::effective_bw_hz() const {
    if(k_ <= 0 || p_.daq_fs_hz <= 0.0) return 0.0;
    return (double)bins_.size() * p_.daq_fs_hz / (double)k_;
}

void XSpec::reset(){
    std::fill(R_.begin(), R_.end(), std::complex<double>(0.0, 0.0));
    frames_ = 0;
    segs_done_ = 0;
}

bool XSpec::prepare(const Params& p, Status& why){
    why = Status::Ok;
    if(p.elements < 2 || p.elements > kMaxElements){ why = Status::TooFewChannels; return false; }
    if(p.ch_bw_hz <= 0.0 || p.daq_fs_hz <= 0.0)     { why = Status::BadRequest;     return false; }

    const double fs   = p.daq_fs_hz;
    const double half = 0.45 * fs;                      // RTL 자체 롤오프 회피
    const double off  = p.ch_center_hz - p.daq_center_hz;   // 채널의 기저대역 오프셋
    const double lo   = off - 0.5 * p.ch_bw_hz;
    const double hi   = off + 0.5 * p.ch_bw_hz;
    if(hi < -half || lo > half){ why = Status::BandOutOfSpan; return false; }

    // 채널 안에 빈이 최소 몇 개는 들어오도록 K 를 키운다. 빈이 1~2개면
    // R 추정이 매우 거칠고 창 누설이 그대로 들어온다.
    const int kMinBins = 8;
    int k = std::max(256, p.fft_size);
    while(k < (1 << 20) && (p.ch_bw_hz * k / fs) < kMinBins) k <<= 1;

    // fft_out_ 은 k_*m_ 로 잡히므로 소자 수가 늘어도 재할당해야 한다.
    // 가드가 k 만 보던 시절엔 같은 K + 더 큰 elements 로 두 번째 prepare 가
    // 들어오면 옛 크기 버퍼가 남고 add_frame 이 그 밖에 썼다 (힙 손상).
    // DAQ 가 채널 수를 늘려 재시작하면 실제로 도달한다.
    const int prev_m = m_;
    m_ = p.elements;
    p_ = p;
    if(k != k_ || m_ != prev_m || !plan_){
        destroy_plan();
        k_ = k;
        fft_in_  = fftwf_malloc(sizeof(fftwf_complex) * (size_t)k_);
        fft_out_ = fftwf_malloc(sizeof(fftwf_complex) * (size_t)k_ * m_);
        if(!fft_in_ || !fft_out_){ destroy_plan(); why = Status::BadRequest; return false; }
        plan_ = make_plan(k_, (fftwf_complex*)fft_in_, (fftwf_complex*)fft_out_);
        if(!plan_){ destroy_plan(); why = Status::BadRequest; return false; }

        win_.resize((size_t)k_);
        win_pow_ = 0.0;
        for(int n = 0; n < k_; n++){
            const float w = (float)(0.5 * (1.0 - std::cos(2.0 * kPi * n / (k_ - 1))));
            win_[n] = w;
            win_pow_ += (double)w * w;
        }
    }

    // 빈 선택
    const double df_hz = fs / (double)k_;
    bins_.clear();
    for(int kk = 0; kk < k_; kk++){
        const double fb = ((kk < k_/2) ? kk : kk - k_) * df_hz;   // 부호 있는 기저대역
        if(fb < lo || fb > hi)            continue;
        if(std::abs(fb) <= p_.dc_guard_hz) continue;              // LO 누설
        if(std::abs(fb) > half)            continue;
        bins_.push_back(kk);
    }
    if(bins_.empty()){
        // 채널이 스팬 안에는 있는데 DC 가드/롤오프에 다 먹힌 경우.
        why = (std::abs(off) <= p_.dc_guard_hz) ? Status::DcOverlap : Status::BandOutOfSpan;
        return false;
    }

    R_.assign((size_t)m_ * m_, std::complex<double>(0.0, 0.0));

    // 프레임당 필요한 세그먼트 수. 빈이 많으면 몇 개면 충분하다.
    const int per_seg = (int)bins_.size();
    seg_needed_ = (per_seg > 0) ? (int)std::ceil((double)p_.target_looks * 1.5 / per_seg) : 1;
    if(seg_needed_ < 1) seg_needed_ = 1;
    frames_ = 0;
    segs_done_ = 0;
    return true;
}

bool XSpec::add_frame(const std::complex<float>* iq, size_t samples_per_ch){
    if(!plan_ || bins_.empty() || m_ < 2) return false;
    if(samples_per_ch < (size_t)k_) return false;

    const int seg_avail = (int)(samples_per_ch / (size_t)k_);   // 겹치지 않는 세그먼트
    // 겹치지 않게 자른다 — look 이 서로 독립이어야 1/sqrt(N) 임계가 성립한다.
    const int seg_use = std::min(seg_avail, seg_needed_);
    if(seg_use <= 0) return false;

    // ── 세그먼트를 CPI 전체에 흩는다 ──────────────────────────────────────
    // 예전엔 프레임 앞쪽에서 연속으로 seg_use 개를 떼어 썼다. 그러면 매
    // 436.9 ms 프레임의 앞부분만 본다: 25 kHz 채널이면 앞 126 ms(28.9%),
    // 200 kHz 면 17 ms(3.9%), 1 MHz 면 3.4 ms(0.78%) 뿐이다 (빈이 많아질수록
    // seg_needed_ 가 작아져서 더 심해진다). 버스티/PTT/TDMA 방출체면 측정이
    // 침묵 구간에 통째로 떨어질 수 있는 실패모드다.
    //
    // stride 로 흩으면 FFT 개수가 그대로라 비용이 정확히 같으면서 각 프레임의
    // 추정이 CPI 전체를 대표하고, 세그먼트끼리 시간상 더 멀어져 독립성 가정도
    // 오히려 좋아진다.
    // stride 만으로는 seg_use==1 일 때 아무 일도 안 일어난다 (s=0 뿐이라 항상
    // 프레임 맨 앞). 그리고 그게 하필 커버리지가 최악인 경우다 — 빈이 많으면
    // seg_needed_ = ceil(target_looks*1.5/|B|) 가 1 로 떨어져서, 1 MHz 채널이면
    // 436.9 ms 중 3.4 ms(0.78%)만 본다. 그래서 프레임마다 시작 오프셋을 한 칸씩
    // 돌린다: seg_use==1 이어도 avg_frames 개 프레임이 서로 다른 지점을 보고,
    // seg_use 가 클 때도 프레임 간에 남은 틈을 메운다.
    // 최대 인덱스 = (seg_use-1)*stride + (stride-1) = seg_use*stride - 1 <= seg_avail-1.
    const int stride = seg_avail / seg_use;      // >= 1 (seg_use <= seg_avail)
    const int rot    = (stride > 1) ? (frames_ % stride) : 0;

    auto* in  = (fftwf_complex*)fft_in_;
    auto* out = (fftwf_complex*)fft_out_;

    for(int s = 0; s < seg_use; s++){
        const size_t base = ((size_t)s * (size_t)stride + (size_t)rot) * (size_t)k_;
        // 채널마다 창 씌우고 FFT. 결과는 out[m*k_ .. ] 에 모아둔다.
        for(int m = 0; m < m_; m++){
            const std::complex<float>* src = iq + (size_t)m * samples_per_ch + base;
            for(int n = 0; n < k_; n++){
                in[n][0] = src[n].real() * win_[n];
                in[n][1] = src[n].imag() * win_[n];
            }
            fftwf_execute_dft((fftwf_plan)plan_, in, out + (size_t)m * k_);
        }
        // 채널 대역 빈만 골라 외적 누산
        for(int b : bins_){
            for(int i = 0; i < m_; i++){
                const std::complex<double> xi(out[(size_t)i*k_ + b][0], out[(size_t)i*k_ + b][1]);
                for(int j = 0; j < m_; j++){
                    const std::complex<double> xj(out[(size_t)j*k_ + b][0], out[(size_t)j*k_ + b][1]);
                    R_[(size_t)i*m_ + j] += xi * std::conj(xj);
                }
            }
        }
        segs_done_++;
    }
    frames_++;
    return true;
}

bool XSpec::snapshot_R(std::complex<double>* out) const {
    if(!out || m_ < 2 || segs_done_ <= 0 || win_pow_ <= 0.0 || k_ <= 0) return false;
    // 백색 입력 분산 sigma^2 에 대해 E|X[k]|^2 = win_pow * sigma^2 이므로,
    // (세그먼트수 * win_pow * K) 로 나누면 tr/M 이 sigma^2 * |B|/K, 즉 대역 내
    // 평균전력이 된다.
    const double s = 1.0 / ((double)segs_done_ * win_pow_ * (double)k_);
    for(size_t i = 0; i < R_.size(); i++) out[i] = R_[i] * s;
    return true;
}

} // namespace df
