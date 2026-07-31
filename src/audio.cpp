#include "fft_viewer.hpp"
#include "audio.hpp"
#include "audio_playback.hpp"
#include "net_client.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>


// ── Mix worker ─────────────────────────────────────────────────────────────
void FFTViewer::mix_worker(){
#ifdef BEWE_HEADLESS
    // Headless HOST: 로컬 스피커 없음 → 로컬 ALSA 믹스 불필요 (네트워크 오디오는 dem_worker/
    // 디코더가 직접 send_audio). 채널 오디오 ring 은 overwrite 방식이라 비우지 않아도 안전.
    return;
#else
    AlsaOut alsa; alsa.open(AUDIO_SR);
    static constexpr int PERIOD=256;
    std::vector<int16_t> sbuf(PERIOD*2,0);

    while(!mix_stop.load(std::memory_order_relaxed)){
        if(!alsa.pcm){
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            alsa.open(AUDIO_SR);
            if(!alsa.pcm) continue;
        }

        for(int i=0;i<PERIOD;i++){
            float L=0,R=0;
            if(net_cli && remote_mode){
                for(int c=0;c<MAX_CHANNELS;c++){
                    bool is_muted = (local_ch_out[c]==3);
                    bool rec_on = channels[c].audio_rec_on.load(std::memory_order_relaxed);
                    if(is_muted && !rec_on){
                        float dummy; int8_t p2;
                        net_cli->audio[c].pop(dummy,p2); continue;
                    }
                    float smp=0; int8_t pan=0;
                    bool jgate=channels[c].sq_gate.load(std::memory_order_relaxed);
                    if(!net_cli->audio[c].pop(smp, pan)){
                        if(rec_on && channels[c].audio_rec_fp)
                            channels[c].maybe_rec_audio(0.f, jgate);
                        continue;
                    }
                    if(rec_on) channels[c].maybe_rec_audio(smp, jgate);
                    if(is_muted) continue;

                    int lco = local_ch_out[c];
                    if(lco==0)      { L+=smp; }
                    else if(lco==2) { R+=smp; }
                    else            { L+=smp; R+=smp; }
                }
            }
            // (예전엔 여기 else 로 "로컬 복조 채널을 직접 믹스" 하는 arm 이 있었다.
            //  GUI 는 JOIN 전용이라 오디오는 항상 net_cli 링에서 온다. HOST 는 위쪽
            //  BEWE_HEADLESS early-return 으로 이 함수에 들어오지도 않는다.)
            // EID Audio 탭 재생: 활성 시 frame 단위로 합산 (로컬 ALSA만, 브로드캐스트 X)
            if(audio_player && audio_player->active() && !audio_player->paused()){
                float pL=0, pR=0;
                if(audio_player->pop_stereo(pL, pR)){
                    L += pL; R += pR;
                }
            }
            L=L<-1.0f?-1.0f:L>1.0f?1.0f:L;
            R=R<-1.0f?-1.0f:R>1.0f?1.0f:R;
            sbuf[i*2  ]=(int16_t)(L*32767.0f);
            sbuf[i*2+1]=(int16_t)(R*32767.0f);
        }
        alsa.write(sbuf.data(),PERIOD);
    }
    alsa.close();
    bewe_log_push(0,"Mix worker exited\n");
#endif
}
