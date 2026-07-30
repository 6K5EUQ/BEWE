#!/bin/bash
# BEWE 기지 systemd 유닛 설치 — screen 기동을 대체한다.
#
# 왜 필요한가
# -----------
# screen 으로 띄운 기지는 (a) 프로세스가 죽으면 (b) 머신이 재부팅되면 사람이 SSH 로
# 붙어 다시 띄워야 한다. /powercycle partial(프로세스 재시작)·full(머신 재부팅)은
# "죽은 뒤 누가 다시 띄우는가"가 보장돼야 성립하므로, systemd 없이 그 명령을 쓰면
# 기지가 그대로 죽는다.
#
# 사용법
#   sudo scripts/install-station-unit.sh <STATION> <TIER> <PRESET>
#   예: sudo scripts/install-station-unit.sh DGS-2 1 2
#
# 위치번호(PRESET)는 cli_host.cpp 의 presets[] 순서다:
#   1=DGS-1  2=DGS-2  3=DGS-3  4=DGS-X  5=ETC
# 프리셋을 추가/제거하면 이 숫자가 밀린다 — 그때는 이 스크립트 호출부도 같이 고칠 것.
set -euo pipefail

if [ $# -ne 3 ]; then
    echo "usage: $0 <STATION> <TIER> <PRESET>" >&2
    echo "  예: $0 DGS-2 1 2" >&2
    exit 2
fi

STATION="$1"; TIER="$2"; PRESET="$3"
STATIONLC="$(echo "$STATION" | tr '[:upper:]' '[:lower:]')"

if [ "$(id -u)" -ne 0 ]; then
    echo "install-station-unit: root 권한 필요 (sudo 로 실행)" >&2
    exit 1
fi

# 이 스크립트를 sudo 로 부른 실제 사용자가 BEWE 실행 계정이다.
RUN_USER="${SUDO_USER:-$(id -un)}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
if [ -z "$RUN_HOME" ] || [ ! -d "$RUN_HOME/BEWE/build-cli" ]; then
    echo "install-station-unit: $RUN_HOME/BEWE/build-cli 없음 - 먼저 CLI 빌드 필요" >&2
    exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE="$REPO_DIR/assets/systemd/bewe-station.service.in"
[ -f "$TEMPLATE" ] || { echo "install-station-unit: $TEMPLATE 없음" >&2; exit 1; }

FIFO="/run/bewe-${STATIONLC}.cmd"
UNIT=/etc/systemd/system/bewe-station.service

sed -e "s|@STATION@|$STATION|g" \
    -e "s|@STATIONLC@|$STATIONLC|g" \
    -e "s|@TIER@|$TIER|g" \
    -e "s|@PRESET@|$PRESET|g" \
    -e "s|@USER@|$RUN_USER|g" \
    -e "s|@HOME@|$RUN_HOME|g" \
    -e "s|@FIFO@|$FIFO|g" \
    "$TEMPLATE" > "$UNIT"

# 로그 파일을 미리 만들어 소유권을 준다 (StandardOutput=append: 는 파일이 없으면
# systemd 가 만들지만, 소유자가 root 라 나중에 사람이 지우기 번거롭다).
touch "/var/log/bewe-${STATIONLC}.log"
chown "$RUN_USER:$RUN_USER" "/var/log/bewe-${STATIONLC}.log"

# /powercycle full 은 `systemctl reboot` 을 호출한다. systemd 서비스는 로그인
# 세션이 없어 polkit 이 기본적으로 "Interactive authentication required" 로 막는다
# (2026-07-30 DGS-2 실측: 재부팅이 거부되고 기지가 죽은 채 남았다). 실행 계정에만
# 재부팅을 허용한다.
POLKIT_RULE=/etc/polkit-1/rules.d/49-bewe-reboot.rules
if [ -d /etc/polkit-1/rules.d ]; then
    cat > "$POLKIT_RULE" <<POLKIT
// BEWE /powercycle full — 실행 계정이 재부팅할 수 있게 한다.
// 무인 기지라 사람이 붙어 대화형 인증을 해 줄 수 없다.
polkit.addRule(function(action, subject) {
    if ((action.id == "org.freedesktop.login1.reboot" ||
         action.id == "org.freedesktop.login1.reboot-multiple-sessions") &&
        subject.user == "$RUN_USER") {
        return polkit.Result.YES;
    }
});
POLKIT
    echo "installed: $POLKIT_RULE (reboot for $RUN_USER)"
else
    echo "WARNING: /etc/polkit-1/rules.d 없음 — /powercycle full 이 재부팅에" >&2
    echo "         실패한다. 실패 시 프로세스 재시작으로 폴백한다." >&2
fi

systemctl daemon-reload
systemctl enable bewe-station.service

echo "installed: $UNIT"
echo "  station=$STATION tier=$TIER preset=$PRESET user=$RUN_USER"
echo "  fifo=$FIFO  log=/var/log/bewe-${STATIONLC}.log"
echo
echo "기동:  sudo systemctl start bewe-station"
echo "확인:  systemctl status bewe-station --no-pager"
echo "명령:  echo '/status' > $FIFO"
