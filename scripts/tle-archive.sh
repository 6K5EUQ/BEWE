#!/usr/bin/env bash
# 일일 TLE 아카이브 — Celestrak 에서 현재 원소를 받아 LEO(비-Starlink)만 골라
# **자기 에폭 날짜로** assets/tle/archive/leo_YYYYMMDD.txt 에 남긴다.
#
# 왜 필요한가: HIST 도플러 매칭은 녹화 시점의 원소를 요구한다. SGP4 오차는 에폭에서
# 멀어질수록 커지고 역전파가 순전파보다 오히려 나쁘므로(실측 38일: 순전파 570 km,
# 역전파 1279 km), 오늘 받은 원소로 과거를 복원할 수 없다. 그날 받아 둔 것만이 답이다.
#
# 계정 불필요 (Celestrak 공개). 과거 원소 소급은 space-track gp_history 가 필요한데
# 그건 별건이다 — 이 스크립트는 "오늘부터 안 잃어버리기" 용이다.
#
# 설치:
#   crontab -e
#   17 5 * * *  /home/ku/BEWE/scripts/tle-archive.sh >> /var/log/bewe-tle.log 2>&1
set -u

BEWE_DIR="${BEWE_DIR:-$(cd "$(dirname "$0")/.." && pwd)}"
TLE_DIR="$BEWE_DIR/assets/tle"
ARCH_DIR="$TLE_DIR/archive"
TMP="$(mktemp)"
trap 'rm -f "$TMP" "$TMP.f"' EXIT

mkdir -p "$ARCH_DIR"

if ! curl --max-time 60 -fsS -A 'Mozilla/5.0' \
     'https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=tle' -o "$TMP"; then
    echo "[tle-archive] fetch failed"; exit 1
fi
if [ ! -s "$TMP" ]; then echo "[tle-archive] empty response"; exit 1; fi

# LEO(고도<2000km) 이면서 이름에 STARLINK 가 없는 것만. 동시에 에폭 중앙값을 구한다.
# 평균운동(2행 53-63열, rev/day) -> 반장축 -> 고도.
awk '
function alt(mm,   n,a){ if(mm<=0) return 1e9; n=mm*6.283185307179586/86400.0;
                         a=exp(log(398600.4418/(n*n))/3.0); return a-6378.137 }
{ L[NR%3==1?0:(NR%3==2?1:2)] = $0 }
NR%3==0 {
    name=L[0]; l1=L[1]; l2=L[2]
    if(substr(l1,1,1)!="1" || substr(l2,1,1)!="2") next
    up=toupper(name); if(index(up,"STARLINK")>0) next
    mm=substr(l2,53,11)+0
    if(alt(mm)>2000) next
    ep=substr(l1,19,14)+0            # YYDDD.DDDDDDDD
    print name > OUT; print l1 > OUT; print l2 > OUT
    eps[++k]=ep
}
END{ n=asort(eps); if(n>0) printf("%.8f\n", eps[int((n+1)/2)]) > MED }
' OUT="$TMP.f" MED="$TMP.med" "$TMP"

if [ ! -s "$TMP.f" ]; then echo "[tle-archive] no LEO records after filter"; exit 1; fi

MED="$(cat "$TMP.med" 2>/dev/null || echo 0)"
# YYDDD.ddd -> YYYYMMDD (TLE 2자리 연도: 57~99=19xx, 00~56=20xx)
DATE="$(python3 - "$MED" <<'PY'
import sys, datetime
v = float(sys.argv[1])
yy = int(v // 1000); ddd = v - yy*1000
year = 1900+yy if yy >= 57 else 2000+yy
d = datetime.date(year,1,1) + datetime.timedelta(days=ddd-1)
print(d.strftime("%Y%m%d"))
PY
)"
[ -n "$DATE" ] || { echo "[tle-archive] epoch parse failed"; exit 1; }

DST="$ARCH_DIR/leo_$DATE.txt"
if [ -e "$DST" ]; then
    echo "[tle-archive] $DATE already archived ($(($(wc -l < "$DST")/3)) sats)"
else
    mv "$TMP.f" "$DST"
    echo "[tle-archive] $DATE archived: $(($(wc -l < "$DST")/3)) sats, $(du -h "$DST"|cut -f1)"
fi

# 현재 카탈로그도 갱신해 둔다 (라이브 스캔이 쓰는 경로).
cp -f "$TMP" "$TLE_DIR/leo_tle.txt.tmp" && mv -f "$TLE_DIR/leo_tle.txt.tmp" "$TLE_DIR/leo_tle.txt"
echo "[tle-archive] leo_tle.txt refreshed ($(($(wc -l < "$TLE_DIR/leo_tle.txt")/3)) sats)"
