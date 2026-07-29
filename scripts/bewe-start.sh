#!/bin/bash
# BEWE HOST (cli_host) 기동 래퍼 — stdin 을 계속 열어 둔다.
#
# 왜 필요한가
# -----------
# 기존 기동은 `printf "ID\nPW\nTIER\nSTATION\n" | ./BEWE` 였다. printf 는 4줄을
# 뱉고 곧바로 죽으므로 파이프에 쓰는 쪽이 사라지고 BEWE 의 stdin 이 EOF 가 된다.
# cli_host 는 EOF 를 g_stdin_eof 로 영구 래치하므로(EOF 파이프에 poll 을 계속 걸면
# 코어 하나를 태우기 때문), 그 뒤로는 어떤 명령도 읽지 않는다.
#
# 결과: 운용 중 명령 주입(/ch, /hist check, /chassis, /powercycle ...)이 전부
# 조용히 무시됐다. `screen -X stuff` 로 넣으면 글자는 화면에 찍히지만 BEWE 는
# 파이프만 보고 있고 그 파이프는 이미 닫힌 상태라 아무 일도 일어나지 않는다.
#
# 이 스크립트는 로그인 4줄을 흘려보낸 뒤 FIFO 를 무한히 재개방해 stdin 을 살려
# 둔다. 그러면 아무 때나 아래처럼 명령을 넣을 수 있다:
#
#     echo "/powercycle" > <FIFO>
#
# 사용법
# ------
#   bewe-start.sh <STATION> <TIER> <PRESET> <FIFO> <CLI_DIR>
#
#   STATION  로그인 ID (= 기지 이름).  예: DGS-3
#   TIER     로그인 tier.               예: 1
#   PRESET   위치번호 (cli_host.cpp presets[]). 프리셋을 추가/제거하면 바뀐다.
#   FIFO     명령 주입용 FIFO 경로.     예: /tmp/bewe-dgs3.cmd
#   CLI_DIR  cli_host 빌드 디렉토리.    예: /home/raspb1/BEWE/build-cli
#
# screen 과 함께 쓰는 예:
#   screen -dmS bewe /path/to/bewe-start.sh DGS-3 1 3 /tmp/bewe-dgs3.cmd ~/BEWE/build-cli
#
# systemd 기지(DGS-X)는 이 스크립트 없이 유닛의 ExecStart 에 같은 구조가 박혀
# 있다. 둘 중 무엇을 쓰든 stdin 배관은 동일하다 — screen/systemd 는 "누가
# 프로세스를 띄우고 살려두나"의 문제이고, FIFO 는 "stdin 에 뭘 연결하나"의
# 문제라 서로 직교한다.
set -u

if [ $# -ne 5 ]; then
    echo "usage: $0 <STATION> <TIER> <PRESET> <FIFO> <CLI_DIR>" >&2
    exit 2
fi

STATION="$1"; TIER="$2"; PRESET="$3"; FIFO="$4"; CLI_DIR="$5"

if [ ! -x "$CLI_DIR/BEWE" ]; then
    echo "bewe-start: $CLI_DIR/BEWE not found or not executable" >&2
    exit 1
fi

# FIFO 재생성. 이전 실행이 남긴 것은 지운다 (소유자/권한이 어긋날 수 있다).
rm -f "$FIFO"
if ! mkfifo -m 0600 "$FIFO"; then
    echo "bewe-start: mkfifo $FIFO failed" >&2
    exit 1
fi

cd "$CLI_DIR" || exit 1

# 로그인 4줄(ID/PW/TIER/PRESET)을 먼저 보내고, 이후 FIFO 를 무한 재개방한다.
# cat 은 writer 가 닫을 때마다 리턴하므로 while 로 다시 연다 — 이래야 명령을
# 여러 번 넣을 수 있다.
#
# 여기서 `exec ./BEWE` 를 쓰면 안 된다. 이 셸이 BEWE 로 치환되어 FIFO 를 읽어 줄
# 프로세스가 사라지고, FIFO 에 쓰는 쪽이 영원히 블록된다.
{
    printf '%s\n%s\n%s\n%s\n' "$STATION" "1" "$TIER" "$PRESET"
    while :; do
        cat "$FIFO"
    done
} | ./BEWE
