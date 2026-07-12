# Pi5 헤드리스 HOST — OS 레벨 튜닝 runbook

BEWE 코드 변경 없이 OS 쪽에서 얻을 수 있는 절감. 코드 최적화(v11.8.x/v11.9.x)를 먼저
배포한 뒤 적용한다 — 그래야 측정 기준선이 오염되지 않는다.

대상: DGS-1/2/3 (Pi5 헤드리스 HOST). Central(raspb2)은 부하가 작아 대상 아님.
sudo 비번은 `~/.claude/secrets.local` 참조.

효과 요약(실측 기반, 정직하게): **CPU 절감은 사실상 0.** 얻는 건 RAM 75-95MB,
SD 쓰기 감소, 전력 0.3-0.7W. CPU 를 줄이는 건 코드 최적화 쪽이지 여기가 아니다.

---

## 1. cpufreq governor (전력)

현재 `ondemand`. 배터리 운용이면 `powersave`(1.5GHz 고정)가 가장 확실하다 —
스핀 픽스 이후 총 부하가 1코어의 30% 미만이라 1.5GHz 로도 여유가 크다.

```bash
# 즉시 적용 (배터리 우선)
echo powersave | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
# 또는 균형 (AC 전원 노드)
echo schedutil | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

부팅 시 지속화:

```bash
sudo tee /etc/systemd/system/cpufreq-governor.service >/dev/null <<'EOF'
[Unit]
Description=Set cpufreq governor
After=multi-user.target
[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo powersave > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'
[Install]
WantedBy=multi-user.target
EOF
sudo systemctl enable --now cpufreq-governor.service
```

**검증(필수):** RTL-SDR 오버플로 로그(`O`) 0건, JOIN 워터폴/오디오 끊김 없음.
DVFS 램프 지연은 USB 버퍼(~100ms @2.56MSPS)와 sleep 기반 50Hz 루프가 흡수하지만,
고SR(BladeRF) 노드에서는 `schedutil` 로 두는 편이 안전하다.

기대: 0.3-0.7W. CPU 사이클 절감은 0 (설계상 당연).

---

## 2. journald 휘발화 + noatime (SD 수명)

```bash
sudo journalctl --vacuum-size=64M                     # 즉시 회수 (실측 677MB 회수됨)
sudo mkdir -p /etc/systemd/journald.conf.d
sudo tee /etc/systemd/journald.conf.d/volatile.conf >/dev/null <<'EOF'
[Journal]
Storage=volatile
RuntimeMaxUse=64M
EOF
sudo systemctl restart systemd-journald
# fstab: discard → noatime (inline discard 제거, atime 쓰기 제거)
sudo sed -i 's/\bdiscard\b/noatime/' /etc/fstab && sudo mount -o remount /
```

트레이드오프: **재부팅하면 시스템 로그가 사라진다.** BEWE 자체 로그는 screen/디스크에
별도로 남으므로 무관. 부팅 이슈를 파야 할 때만 `Storage=persistent` 로 일시 복원.

기대: 저널 쓰기 8MB/일 제거 + atime/discard 쓰기 제거. CPU 는 <0.1%.

---

## 3. 불필요 데몬 정리 (RAM)

```bash
sudo snap remove cups        # 또는 snap disable cups
sudo systemctl disable --now fwupd fwupd-refresh.timer upower lighttpd \
     motd-news.timer update-notifier-download.timer update-notifier-motd.timer
```

**절대 건드리지 말 것:** `NetworkManager`, `wpa_supplicant`, `tailscaled` —
DGS-3 업링크는 wlan0(5GHz WiFi)이고 원격 접속은 전부 Tailscale 이다. 끊으면 복구 불가.

`sysstat` 은 끄지 말 것 — sar 성능 이력이 사라져 나중 진단이 어려워진다.
`lighttpd` 는 기본 페이지만 서빙하는 잔재로 확인됐으나, 상태페이지로 쓰고 있지 않은지
한 번 확인하고 끌 것.

기대: RSS 75-95MB 해제 (fwupd 42 + cupsd 11 + cups-proxyd 7 + upowerd 8 + lighttpd 3MB).
8GB 중 6.8GB 가 free 라 **성능 영향은 0** — 순수 위생.

---

## 4. config.txt 슬림화 (선택 — 이득 작음)

```bash
# /boot/firmware/config.txt
#   dtoverlay=vc4-kms-v3d   ← 주석 처리
#   dtparam=audio=off
#   camera_auto_detect=0
#   display_auto_detect=0
sudo reboot
```

기대: RAM 실효 2-10MB, 부팅 소폭 단축. **CMA 64MB 회수는 오해** — 리눅스 CMA free
페이지는 이미 movable 할당에 재사용되고 MemAvailable 에 포함된다.

비용: **현장 HDMI 디버그 콘솔 상실** (롤백은 SSH 또는 SD 카드 마운트로만 가능).
반드시 SSH 가 살아있는 상태에서, 재기동 창에 묶어 적용하고 즉시 확인할 것.

---

## 검증 체크리스트 (적용 후 공통)

```bash
p=$(pgrep -x BEWE)
ps -eLo pcpu,comm,lwp -p $p --no-headers | sort -rn | head -5   # 스레드별 CPU
free -m                                                          # RAM
screen -S bewe -X hardcopy /tmp/hc.txt; grep -E "CF=|SDR=|room=" /tmp/hc.txt | tail -2
```

- BEWE 프로세스 CPU 가 코드 최적화 후 기대치(유휴 시 한 자릿수 %) 안인지
- `SDR=OK`, room 개설, JOIN 접속 시 워터폴/오디오/디코드 정상
- RTL 오버플로(`O`) 로그 0건
