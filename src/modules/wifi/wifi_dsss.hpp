#pragma once
// ── 802.11b DSSS 1Mbps 비콘 디코더 (2.4GHz 비콘 대부분) ────────────────────
// Barker despread + DBPSK + self-sync descramble + SFD + PLCP + FCS + IE.
// 입력 = 채널 baseband 복소 (임의 SR, 내부에서 22 MSPS=2sps/chip 리샘플).
#include "wifi_meta.hpp"
#include "wifi_ofdm.hpp"   // crc32_802
#include <vector>
#include <complex>
#include <cmath>
#include <functional>
#include <cstdint>
#include <cstring>

namespace wifi_dsss {
using cf=std::complex<float>;
static const int BARKER[11]={1,-1,1,1,-1,1,1,1,-1,-1,-1};

// 채널 baseband → FCS 유효 DSSS 비콘 → on_rec(WifiRecord)
// (22 MSPS 리샘플은 wifi_ofdm::resample_frac<12> — 공용 폴리페이즈 커널)
inline void decode_buffer_dsss(const cf* xin,size_t nin,double in_sr,
                               const std::function<void(const WifiRecord&)>& on_rec,int max_pkt=400){
    std::vector<cf> rs; const cf* x; size_t N;
    if(std::fabs(in_sr-22e6)<1){ x=xin; N=nin; }
    else { rs=wifi_ofdm::resample_frac<12>(xin,nin,in_sr/22e6); x=rs.data(); N=rs.size(); }
    if(N<512) return;
    const int SPS=2, SPSYM=22;
    cf taps[22]; for(int i=0;i<22;i++) taps[i]=cf((float)BARKER[i/2],0);
    auto mf=[&](size_t n)->cf{ cf s=0; for(int i=0;i<22;i++) s+=x[n+i]*taps[i]; return s; };
    int pk=0;
    for(size_t base=0; base+300*SPSYM<N && pk<max_pkt; ){
        double e=0; for(int j=0;j<22;j++) e+=std::norm(mf(base+j));
        double pw=0; for(int j=0;j<SPSYM;j++) pw+=std::norm(x[base+j]);
        double ratio=pw>1e-9?e/22.0/(pw/SPSYM):0;
        if(ratio<4.0){ base+=SPSYM; continue; }
        int boff=0; double bmax=0;
        for(int off=0;off<SPSYM;off++){ double a=0; for(int s=0;s<32;s++){ size_t p=base+off+s*SPSYM; if(p+22<N) a+=std::abs(mf(p)); } if(a>bmax){bmax=a;boff=off;} }
        std::vector<uint8_t> sb; cf prev=0;
        for(int s=0;s<4000;s++){ size_t p=base+boff+s*SPSYM; if(p+22>=N) break; cf c=mf(p);
            if(s>0){ float d=(c*std::conj(prev)).real(); sb.push_back(d<0?1:0); } prev=c; }
        if(sb.size()<200){ base+=SPSYM; continue; }
        std::vector<uint8_t> d(sb.size());
        for(size_t n=0;n<sb.size();n++){ int a=sb[n],b4=(n>=4?sb[n-4]:0),b7=(n>=7?sb[n-7]:0); d[n]=a^b4^b7; }
        auto find_sfd=[&](int msb)->int{ for(size_t n=0;n+16<=d.size();n++){ int v=0; for(int k=0;k<16;k++) v|=d[n+k]<<(msb?(15-k):k); if(v==0xF3A0) return (int)n; } return -1; };
        int sfd=find_sfd(1); if(sfd<0) sfd=find_sfd(0);
        if(sfd<0){ base+=300*SPSYM; continue; }
        pk++;
        int hp=sfd+16; if(hp+48>(int)d.size()){ base+=300*SPSYM; continue; }
        auto gb=[&](int bp){ int v=0; for(int b=0;b<8;b++) v|=d[bp+b]<<b; return v; };
        int sig=gb(hp); int len_us=gb(hp+16)|(gb(hp+24)<<8);
        if(sig!=0x0A){ base+=300*SPSYM; continue; }       // 1Mbps DBPSK 만
        int oct=len_us/8; int mp=hp+48;
        if(oct<=24||oct>=2400||mp+oct*8>(int)d.size()){ base+=300*SPSYM; continue; }
        std::vector<uint8_t> m(oct); for(int B=0;B<oct;B++) m[B]=gb(mp+B*8);
        uint32_t fcs=(m[oct-4])|(m[oct-3]<<8)|(m[oct-2]<<16)|((uint32_t)m[oct-1]<<24);
        if(fcs!=wifi_ofdm::crc32_802(m.data(),oct-4)){ base+=300*SPSYM; continue; }
        if(m[0]!=0x80){ base+=300*SPSYM; continue; }       // beacon
        WifiRecord r{};
        snprintf(r.bssid,sizeof(r.bssid),"%02x:%02x:%02x:%02x:%02x:%02x",m[16],m[17],m[18],m[19],m[20],m[21]);
        strcpy(r.sec,"Open"); strcpy(r.phy,"b");
        int p=36;
        while(p+2<=oct-4){ int id=m[p],l=m[p+1]; if(p+2+l>oct-4)break;
            if(id==0){ int nn=l<32?l:32; if(nn){memcpy(r.ssid,&m[p+2],nn); r.ssid[nn]=0;} }
            else if(id==3&&l>=1) r.wch=m[p+2];
            else if(id==48) strcpy(r.sec,"WPA2");
            else if(id==45&&!strcmp(r.phy,"b")) strcpy(r.phy,"n");
            else if(id==191) strcpy(r.phy,"ac");
            else if(id==221&&l>=4&&m[p+2]==0&&m[p+3]==0x50&&m[p+4]==0xf2&&m[p+5]==1&&!strcmp(r.sec,"Open")) strcpy(r.sec,"WPA");
            else if(id==255&&l>=1&&m[p+2]==35) strcpy(r.phy,"ax");
            p+=2+l; }
        on_rec(r);
        base+=300*SPSYM;
    }
}

} // namespace wifi_dsss
