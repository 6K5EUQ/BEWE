# KrakenSDR DF (방향탐지) 기지 구축

BEWE 의 DF 기능은 KrakenSDR 5채널 코히런트 수신기를 쓴다. BEWE 자체는 SDR 을
직접 열지 않고, **heimdall DAQ 체인이 TCP `:5000` 으로 내주는 IQ 스트림의
소비자**다 (`src/kraken_io.cpp`). 따라서 기지마다 heimdall 을 따로 설치해야 한다.

- 최초 구축: **DGS-X (raspb2, Pi5 / Ubuntu 24.04 aarch64)** — 2026-08-02
- 이 문서는 그 실측 기록이자, 다른 기지로 옮길 때의 절차다.

```
KrakenSDR (RTL2838 x5)
   └─ heimdall DAQ chain  (rtl_daq -> rebuffer -> decimate -> delay_sync)
        ├─ iq_server.out    :5000  IQ 스트림     ─┐
        └─ hw_controller.py :5001  제어(주파수/게인)│
                                                  ▼
                                   BEWE  --sdr kraken
                                     ch0        -> 스펙트럼/워터폴/복조
                                     ch0..ch4   -> DF 엔진
```

**BEWE 는 DAQ 를 띄우지 않는다.** 이미 떠 있는 `:5000` 에 붙기만 한다. 그리고
`--sdr kraken` 은 **자동 감지 대상이 아니다** — 명시 지정해야 한다 (heimdall 이
다른 소비자를 위해 떠 있을 수 있고, 매 기동마다 `:5000` 을 두드리면 느려지므로).

---

## 0. 전제 — 전원 (여기서 제일 크게 데였다)

**KrakenSDR 은 반드시 독립 전원을 받아야 한다.** 공식 문서:

> "DC power from a 5V 2.4A capable USB-C power supply" — 통상 소비 5V 2.2A (11W)
>
> "Please note that this data port is not connected to power, so you cannot power
> the device from the data cable."

| 포트 | 방향 | 연결 대상 |
|---|---|---|
| Kraken **USB-C 전원** | 입력 | **독립 5V 2.4A+ 어댑터** (Pi 의 USB 포트 금지) |
| Kraken **USB-C 데이터** | 데이터 전용, 전원 안 흐름 | Pi 의 USB-A |

**bias-tee(노이즈 소스)를 쓰면 2.2A 위에 전류가 더 붙는다.** DF 는 위상
캘리브레이션에 노이즈 소스를 쓰므로 항상 해당된다 — **3A 급 이상**으로 잡아라.

Geekworm **X1200 UPS 의 USB-C 는 입력 전용**이라 Kraken 급전에 못 쓴다
("Power&Charge input: 5Vdc 5A via USB Type-C"). X1200 에서 먹이려면 **XH2.54
출력 커넥터**(5.1V ±5%, 각 최대 5A)를 써야 한다.

### 전원이 부족할 때의 증상 (실측)

노이즈 소스를 켜는 그 순간 USB 가 통째로 무너진다:

```
Gain change at ch: 0..4, gain 496
Noise source turned on              <- rtl_daq.c rtlsdr_set_bias_tee_gpio(dev,0,1)
cb transfer status: 1, canceling... <- 5채널 전부
rtlsdr_demod_write_reg failed with -1
rtlsdr_demod_read_reg  failed with -4
... (로그가 분당 수 GB 로 폭주, rtl_daq CPU 190%+)
```

**평상시 스트리밍은 멀쩡한데 노이즈 소스에서만 죽는 게 특징이다.** 대역이나
드라이버 문제로 오진하기 쉬우니, 아래 격리 실험으로 먼저 전원을 의심할 것.

| 실험 | 전원 부족 시 |
|---|---|
| `kraken_test -d 0 -s 2400000` (동글 1개) | 손실 0 (정상으로 보인다) |
| 동글 5개 동시 2.4 Msps | **손실 0** (여기서도 정상으로 보인다) |
| `rtl_daq.out` 단독 | 정상 (hw_controller 가 없어 노이즈 소스를 안 켠다) |
| rtl_daq + rebuffer + decimate + delay_sync (**hw_controller 제외**) | 정상 |
| 풀 체인 | **붕괴** |
| 풀 체인 + `en_noise_source_ctr = 0` | 정상 (단 캘리브 불가) |

`en_noise_source_ctr=0` 으로 우회하면 `Correlation peak dynamic range is
insufficient` (실측 15~17 dB, 최소 20 dB) 가 뜬다 — **DF 정확도가 안 나온다.
운용 우회책이 아니라 진단 수단으로만 써라.**

---

## 1. 설치 (기지 1대 기준)

전제: Ubuntu/Raspbian 64bit, `~/BEWE` 클론 및 `build-cli` 빌드 완료.
소요 시간 Pi5 기준 약 30~40분 (conda env 생성이 대부분).

### 1-1. 빌드 의존성

```bash
sudo apt update
sudo apt install -y build-essential git cmake libusb-1.0-0-dev lsof libzmq3-dev
```

### 1-2. krakenrf 커스텀 librtlsdr (**재부팅 필요**)

배포판 `librtlsdr` 로는 안 된다. Kraken 전용 드라이버가 따로 있다.

```bash
cd ~
git clone https://github.com/krakenrf/librtlsdr
cd librtlsdr
sudo cp rtl-sdr.rules /etc/udev/rules.d/rtl-sdr.rules
mkdir -p build && cd build
cmake ../ -DINSTALL_UDEV_RULES=ON
make -j$(nproc)
sudo ln -sf ~/librtlsdr/build/src/rtl_test /usr/local/bin/kraken_test

echo 'blacklist dvb_usb_rtl28xxu' | sudo tee /etc/modprobe.d/blacklist-dvb_usb_rtl28xxu.conf
sudo reboot
```

`dvb_usb_rtl28xxu` 커널 모듈이 동글을 선점하므로 블랙리스트가 필수다.
재부팅 후 `lsmod | grep dvb_usb_rtl28xxu` 가 비어야 한다.

> **드론 기지(DGS-X)는 재부팅 전에 지상 대기인지 확인할 것.** Tailscale 복귀에
> 40~60초 걸린다.

### 1-3. Ne10 (ARM64 전용)

x86_64 기지라면 이 단계 대신 KFR 을 깐다 (heimdall README 참조).

```bash
cd ~
git clone https://github.com/krakenrf/Ne10
cd Ne10 && mkdir -p build && cd build
cmake -DNE10_LINUX_TARGET_ARCH=aarch64 -DGNULINUX_PLATFORM=ON \
      -DCMAKE_C_FLAGS="-mcpu=native -Ofast -funsafe-math-optimizations" ..
make -j$(nproc)
```

### 1-4. miniforge + conda env `kraken`

heimdall 의 python 단(`delay_sync.py`, `hw_controller.py`)은 numba 를 쓴다.
ARM 에서 pip numba 가 제대로 안 붙어 conda 를 쓰는 것이 공식 권장이다.

```bash
cd ~
wget -O /tmp/miniforge.sh \
  https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
bash /tmp/miniforge.sh -b -p ~/miniforge3     # x86_64 면 파일명만 바꾼다
~/miniforge3/bin/conda config --set auto_activate_base false

export PATH="$HOME/miniforge3/bin:$PATH"
conda create -n kraken python=3.9.7 -y
conda install -n kraken -y scipy==1.9.3 numba==0.56.4 configparser pyzmq scikit-rf
```

버전을 그대로 고정할 것 — heimdall 이 그 조합에서 검증돼 있다.

### 1-5. heimdall 빌드

```bash
mkdir -p ~/krakensdr_doa && cd ~/krakensdr_doa
git clone https://github.com/krakenrf/heimdall_daq_fw.git

CORE=~/krakensdr_doa/heimdall_daq_fw/Firmware/_daq_core
cd "$CORE"
cp ~/librtlsdr/build/src/librtlsdr.a .
cp ~/librtlsdr/include/rtl-sdr.h .
cp ~/librtlsdr/include/rtl-sdr_export.h .
cp ~/Ne10/build/modules/libNE10.a .            # ARM 만

export PATH="$HOME/miniforge3/envs/kraken/bin:$PATH"
make
ls *.out    # rtl_daq / rebuffer / decimate / iq_server 4개
```

`Makefile` 의 `PIGPIO=` 줄은 주석 상태로 둔다 (Corey Koval 스위치보드 전용).

**`krakensdr_doa` 웹 UI / DoA DSP 는 설치하지 않는다.** DF 데이터 경로에 없고,
오히려 `app.py` 가 `:5001` 을 무조건 열어 BEWE 와 정면 충돌한다.

### 1-6. BEWE DF 스크립트 배치

`bewe_df_start.sh` / `bewe_df_stop.sh` / `bewe_df_run.sh` / `bewe_df/` 를
`~/krakensdr_doa/` 에 둔다. 이미 구축된 기지(DGS-X)에서 그대로 복사하면 된다:

```bash
ssh raspb2@100.123.59.3 \
  'cd ~/krakensdr_doa && tar czf - bewe_df bewe_df_start.sh bewe_df_stop.sh bewe_df_run.sh' \
  | ssh <새기지> 'mkdir -p ~/krakensdr_doa && tar xzf - -C ~/krakensdr_doa'
ssh <새기지> 'chmod +x ~/krakensdr_doa/bewe_df_*.sh'
```

**스크립트는 계정명·경로에 의존하지 않게 고쳐 뒀다** (아래 §3 참조). 그대로
복사하면 어느 기지에서든 돈다 — `KROOT` 를 손댈 필요 없다.

`bewe_df/daq_chain_config.ini` 는 stock 대비 2줄만 다르다:

| 항목 | stock | BEWE |
|---|---|---|
| `cpi_size` | 1048576 | **262144** |
| `out_data_iface_type` | shmem | **eth** |

`ctr_channel_serial_no = 1000` 은 노이즈 소스가 물려 있는 동글의 SN 이다.
**개체마다 다를 수 있으니 함부로 바꾸지 마라** — 다른 SN 을 넣으면 USB 는
안정되지만 노이즈 소스가 실제로 안 켜져 캘리브레이션이 미달한다 (§0 참조).

### 1-7. systemd 유닛 2개

기지 이름에 맞춰 `<STATION>` / 계정 / 위치번호를 바꾼다. DGS-X 는 유닛명이
`bewe-dgsx`, 나머지 기지는 `bewe-station` 이다 (bewe-fleet 스킬 §7).

**(a) DAQ 유닛** — `/etc/systemd/system/bewe-<station>-daq.service`

```ini
[Unit]
Description=KrakenSDR heimdall DAQ chain (BEWE DF)
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service
Before=shutdown.target
Conflicts=shutdown.target

[Service]
Type=simple
User=<account>
WorkingDirectory=/home/<account>/krakensdr_doa
ExecStart=/home/<account>/krakensdr_doa/bewe_df_run.sh
ExecStop=/home/<account>/krakensdr_doa/bewe_df_stop.sh
KillMode=mixed
Restart=on-failure
RestartSec=15
TimeoutStartSec=300
TimeoutStopSec=120
StandardOutput=append:/var/log/bewe-<station>-daq.log
StandardError=append:/var/log/bewe-<station>-daq.log

[Install]
WantedBy=multi-user.target
```

**(b) 기존 BEWE 유닛 수정** — 두 곳:

```ini
# [Unit] 에 추가 — DAQ 가 :5000 을 연 뒤에 떠야 한다
After=bewe-<station>-daq.service
Requires=bewe-<station>-daq.service
```

```ini
# ExecStart 에 --sdr kraken 추가 (자동감지 대상이 아니다)
ExecStart=/bin/bash -c 'exec ./BEWE --sdr kraken < <(printf "<ID>\n1\n1\n<PRESET>\n"; while :; do cat /run/bewe-<station>.cmd; done)'
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now bewe-<station>-daq.service
sudo systemctl restart bewe-<station>.service
```

### 1-8. 검증

```bash
systemctl is-active bewe-<station>-daq bewe-<station>     # 둘 다 active
ps -eo pcpu,args --sort=-pcpu | grep _daq_core | grep -v grep
#   rtl_daq / rebuffer / decimate / delay_sync / hw_controller  5개가 CPU 를 먹어야 한다
ls /dev/shm/     # decimator_in_A/B, decimator_out_A/B, delay_sync_iq_A/B, delay_sync_hwc_A/B  8개
grep -c "cb transfer" ~/krakensdr_doa/heimdall_daq_fw/Firmware/_logs/rtl_daq.log   # 0 이어야 한다
grep -a "Kraken\|SDR=" /var/log/bewe-<station>.log | tail -3
#   [Kraken] kraken5  5 ch  700.0000 MHz  2.400 MSPS  (adopted from DAQ)
#   SDR=OK ... Drops=0
```

`SDR STALL: total_ffts frozen` 이 반복되면 BEWE 는 `:5000` 에 붙었지만 프레임이
안 오는 것이다 — DAQ 체인 쪽을 본다.

---

## 2. 겪은 문제와 원인 (재발 시 바로 참조)

### 2-1. 노이즈 소스 켜는 순간 USB 붕괴 → **전원 부족**

§0 참조. 진단에 가장 오래 걸린 항목이다. 아래 오답들을 먼저 배제해 두었으니
같은 길을 다시 걷지 말 것:

| 의심했다가 실측으로 기각한 것 | 기각 근거 |
|---|---|
| USB 대역 (5ch x 2.4 Msps = 192 Mbps, USB2 한계) | 동글 5개 동시 스트림 **전부 손실 0** |
| `usbfs_memory_mb=0` | 0/1000 양쪽에서 rtl_daq 단독은 **둘 다 정상** |
| 시스템 librtlsdr 과 심볼 충돌 | `ldd` 확인 — rtl_daq 은 krakenrf `.a` 정적 링크, 충돌 없음 |
| `krakensdr_doa` 웹스택 미설치 | DF 경로에 없다. 오히려 있으면 `:5001` 을 뺏는다 |
| rebuffer 역압(파이프 블록) | `rtl_daq \| rebuffer` 단독 조합 정상 |
| 동글 개체(SN 1000) 불량 | 전원 보강 후 같은 SN 으로 정상 동작 |

**결정적 실험은 "hw_controller 를 뺀 체인"이었다.** 그것만 빼면 정상 → 노이즈
소스 제어가 범인 → bias-tee 급전 → 전원.

### 2-2. `log_level = 5` 라 heimdall 로그가 아무것도 안 보인다

`_daq_core/log.h`: `{TRACE=0, DEBUG=1, INFO=2, WARN=3, ERROR=4, FATAL=5}`.
운용 기본값 5 는 **FATAL 만** 찍는다. 그래서 `rtl_daq.log` 에 librtlsdr 의
raw stderr 만 보이고 heimdall 자신의 진단은 한 줄도 안 남는다.

진단할 땐 오버레이를 복사해 `log_level = 2` 로 바꿔 쓰고, **끝나면 반드시 5 로
되돌린다** (2 로 두면 `delay_sync.log` 가 numba DEBUG 로 폭주한다).

```bash
cp ~/krakensdr_doa/bewe_df/daq_chain_config.ini /tmp/diag.ini
sed -i 's/^log_level = 5/log_level = 2/' /tmp/diag.ini
cp /tmp/diag.ini ~/krakensdr_doa/bewe_df/daq_chain_config.ini   # 진단
# ... 끝나면 원본 복원
```

### 2-3. `usbfs_memory_mb = 0` — 커널 6.8 에서는 무제한이 아니다

`daq_start_sm.sh` 는 "libusb 한도를 풀려고" 0 을 써 넣는다. 옛 커널에선 0 =
무제한이었지만 **6.8(raspi) 에서는 0 MB 제한**이다.

실측상 이것만으로 체인이 죽지는 않았지만(단독 rtl_daq 은 0 에서도 정상),
큰 버퍼를 잡는 구성에서는 언제든 문제가 될 수 있어 `bewe_df_start.sh` 에
가드를 넣어 뒀다. **`start_sm` 이 0 을 쓰는 시점이 rtl_daq 기동보다 앞이라 한
번만 덮으면 순서가 어긋난다** — 그래서 기동이 끝날 때까지 값을 되돌리는
백그라운드 루프로 짰다.

### 2-4. systemd 로 띄우면 `delay_sync` / `hw_controller` 가 죽는다

`daq_start_sm.sh` 는 체인 5개를 **백그라운드로 띄우고 곧바로 반환**한다.
그대로 `Type=oneshot` + `RemainAfterExit=yes` 로 감싸면 ExecStart 가 끝나는
순간 systemd 가 cgroup 에 남은 프로세스를 정리해 python 2개가 죽는다. 증상은
`delay_sync.log` 의 `WARNING:shmemIface:Shared memory not exist` 반복이고,
BEWE 쪽에서는 `SDR STALL ... forcing recovery` 무한 반복으로 나타난다.

`KillMode=process` 로도 안 잡힌다. 해결책은 **상주 래퍼**(`bewe_df_run.sh`):
체인을 띄운 뒤 foreground 에 남아 핵심 프로세스를 감시하고, 사라지면 종료해
systemd 가 재기동하게 한다. `Type=simple` + `KillMode=mixed` 와 짝이다.

### 2-5. `conda: command not found`

`bewe_df_start.sh` 원본은 `conda shell.bash hook` 을 PATH 에서 찾는다.
systemd·비대화식 SSH 에는 conda 가 PATH 에 없다. 설치 경로를 직접 탐색하도록
고쳤다 (`miniforge3` / `miniconda3` / `anaconda3` / `/opt/miniforge3`).

### 2-6. 종료 후 잔여 shmem / 프로세스

체인이 비정상 종료하면 `/dev/shm` 의 8개와 `_daq_core/*` 프로세스가 남아 다음
기동을 방해한다. 깨끗이 지우는 절차:

```bash
sudo systemctl stop bewe-<station> bewe-<station>-daq
cd ~/krakensdr_doa && ./bewe_df_stop.sh
sudo pkill -f "_daq_core/"
sudo rm -f /dev/shm/decimator_* /dev/shm/delay_sync_*
```

> `pkill -f "_daq_core/"` 는 이 경우엔 안전하다 (BEWE 바이너리와 패턴이 겹치지
> 않는다). **BEWE 프로세스에는 절대 `pkill -f` 를 쓰지 마라** — bewe-fleet
> 스킬 §7 참조 (자기 자신을 죽이고 정작 대상은 못 잡는다).

### 2-7. `daq_start_sm.sh` 의 포트 게이트가 영구 행

`lsof -i:5000` 은 리스너뿐 아니라 **클라이언트 소켓도 매칭**한다. 즉 BEWE 가
접속만 하고 있어도 "포트 사용 중"으로 보고, 게이트 루프가 `daq_stop.sh` 만
무한 반복한다 (타임아웃도 상한도 없다). `bewe_df_start.sh` 가 사전에
`bewe_df_stop.sh` + `lsof` 검사로 막는다 — **DAQ 를 다시 띄우기 전에 BEWE 를
먼저 내려야 하는 이유**가 이것이다.

`Requires=` 로 묶어 뒀으므로 `systemctl restart bewe-<station>-daq` 하면
BEWE 도 함께 재기동되어 순서가 맞는다.

### 2-8. BEWE 만 재시작하면 :5000 에 못 붙는다

**`iq_server` 는 한 번에 클라이언트를 하나만 받는다** (`heimdall_client.hpp`).
BEWE 를 내리면 그 소켓이 곧바로 정리되지 않고 `CLOSE_WAIT` 로 남는 경우가 있어,
새로 뜬 BEWE 의 접속이 `SYN_SENT` 에서 영영 대기한다. 로그에는 이렇게 보인다:

```
[Kraken] no DAQ on 127.0.0.1:5000 (connect 127.0.0.1:5000: Operation now in progress)
```

DAQ 는 멀쩡히 `active` 이고 체인 5개도 다 돌고 있어 "DAQ 가 죽었나" 로 오진하기
쉽다. 소켓 상태를 보면 바로 갈린다:

```bash
sudo lsof -nP -i:5000
#   iq_server ... 127.0.0.1:5000->127.0.0.1:44608 (CLOSE_WAIT)   <- 구 세션 잔재
#   BEWE      ... 127.0.0.1:56508->127.0.0.1:5000 (SYN_SENT)     <- 새 BEWE 가 못 붙는다
```

**그래서 Kraken 기지는 BEWE 단독 재시작을 쓰지 않는다.** DAQ 를 재시작하면
`Requires=` 로 BEWE 도 같이 내려갔다 올라오고, iq_server 가 새로 뜨면서 소켓도
정리된다:

```bash
sudo systemctl restart bewe-<station>-daq     # BEWE 도 함께 재기동된다
#   sudo systemctl restart bewe-<station>     <- 이것만 하면 위 증상이 난다
```

정상 복귀는 `ESTABLISHED` 두 줄로 확인한다 (리스너 + 연결된 쌍).

(2026-08-02 v15.2.0 배포 중 실측. 배포 자체는 정상이었고 재시작 순서만 문제였다.)

---

## 3. 다른 기지로 확장할 때

DGS-X 구축 중 **스크립트를 기지 독립적으로 고쳐 뒀다.** 그대로 복사하면 된다.

| 고친 것 | 이유 |
|---|---|
| `KROOT` 하드코딩(`/home/ku/krakensdr_doa`) → `BASH_SOURCE` 파생 | 계정명이 기지마다 다르다 (`ku`/`dsa`/`raspb1`/`raspb2`) |
| conda 를 PATH 대신 설치경로 탐색 | systemd·비대화식 SSH 에서 PATH 에 없다 |
| `usbfs_memory_mb` 가드 추가 | 커널 6.8 에서 0 = 0MB 제한 |
| `bewe_df_run.sh` 신규 | systemd 상주 래퍼 (§2-4) |

### 기지별로 반드시 바꿔야 하는 것

| 항목 | 위치 |
|---|---|
| 계정명 / `WorkingDirectory` | DAQ 유닛 |
| 유닛 이름 `bewe-<station>-daq` | 파일명 + BEWE 유닛의 `After=`/`Requires=` |
| 로그 경로 `/var/log/bewe-<station>-daq.log` | DAQ 유닛 |
| `--sdr kraken` | BEWE 유닛 `ExecStart` |
| Ne10(ARM) vs KFR(x86_64) | §1-3 |
| miniforge 설치 스크립트 아키텍처 | §1-4 |

### 바꾸지 말아야 하는 것

- `bewe_df/daq_chain_config.ini` — 전 기지 동일 (md5 대조로 확인)
- `ctr_channel_serial_no = 1000` — 노이즈 소스 채널. 개체가 다르면 그때 실측
- conda 패키지 버전 (`python 3.9.7` / `scipy 1.9.3` / `numba 0.56.4`)

### DGS-1 (ku) 의 기존 설치는 구버전이다

DGS-1 에도 `~/krakensdr_doa` 가 있지만 **위 4가지 수정이 반영되기 전 버전**이다
(KROOT 하드코딩, conda PATH 의존, usbfs 가드 없음, 상주 래퍼 없음). DGS-1 에서
DF 를 다시 쓸 때는 §1-6 대로 DGS-X 것을 복사해 덮을 것.

또한 DGS-1 은 systemd DAQ 유닛이 없다 — `bewe_df_start.sh` 를 사람이 실행하는
방식이다. 자동 기동이 필요하면 §1-7 을 그대로 적용하면 된다.

---

## 4. 운용 명령 요약

```bash
# 상태
systemctl is-active bewe-<station>-daq bewe-<station>
tail -20 /var/log/bewe-<station>-daq.log
grep -a "Kraken\|SDR=" /var/log/bewe-<station>.log | tail -5

# 재기동 — 항상 DAQ 쪽을 친다. Requires= 로 BEWE 도 같이 내려갔다 올라오고,
# iq_server 가 새로 뜨면서 구 세션 소켓도 정리된다 (§2-8)
sudo systemctl restart bewe-<station>-daq

# :5000 연결 확인 (ESTABLISHED 두 줄이 정상)
sudo lsof -nP -i:5000

# DAQ 만 수동으로
cd ~/krakensdr_doa && ./bewe_df_stop.sh && ./bewe_df_start.sh

# 진단: heimdall 로그 켜기 (끝나면 반드시 되돌릴 것 — §2-2)
```

### 새 버전 배포

```bash
cd ~/BEWE && git pull --ff-only && cmake --build build-cli -j$(nproc)
./build-cli/BEWE --df-selftest            # DF DSP 회귀 (60/60)
sudo systemctl restart bewe-<station>-daq  # BEWE 단독 재시작이 아니다 (§2-8)
```

와이어 포맷이 바뀐 배포는 **Central 을 먼저** 올린다.

DGS-X 실측값 (2026-08-02, 정상 상태):

```
[Kraken] kraken5  5 ch  700.0000 MHz  2.400 MSPS  (adopted from DAQ)
CF=700.0MHz SR=2.40M Clients=0 CPU=13% RAM=10% SDR=OK IQ=OFF TX=0.0MB Drops=0
rtl_daq 14%  delay_sync 17%  decimate 4%  rebuffer 2%  hw_controller 1%
cb transfer 에러 0 / rtl_daq.log 410 bytes (기동 배너만)
```
