#!/usr/bin/env bash
# Central TLE 수집기 — 전 기지가 공유할 궤도원소를 Central 한 곳에서 모은다.
#
# 왜 Central 인가: space-track 요청 제한이 **계정당** 이라 기지마다 받으면 서로
# 잡아먹는다. 게다가 인터넷 없는 기지도 Central 만 붙으면 원소를 받을 수 있다.
#
# 두 소스:
#   Celestrak   gp.php GROUP=active   계정 불필요, **현재** 원소 (매일 오늘치)
#   space-track gp_history            계정 필요,   **과거** 원소 (소급 복구)
#
# 저장: $DB/tle/leo_YYYYMMDD.txt  (3줄 TLE, LEO 만, Starlink 제외)
#   DataBase 밑에 두는 이유는 기존 DB_DOWNLOAD_REQ(0x27) 경로가 파일명으로 그대로
#   서빙할 수 있어서다 — 와이어 프로토콜을 새로 만들 필요가 없다.
#
# 자격증명: /etc/bewe-spacetrack.conf (0600, root)
#   ST_USER=...
#   ST_PASS=...
#
# 사용:
#   central-tle-sync.sh                 오늘치 (Celestrak)
#   central-tle-sync.sh 2026-05-20      그 날짜 (space-track, 소급)
#   central-tle-sync.sh 2026-05-16 2026-07-29   구간 소급
#
# cron:
#   23 4 * * *  /home/central/BEWE/scripts/central-tle-sync.sh >> /var/log/bewe-tle.log 2>&1
set -u

DB="${BEWE_DB:-$HOME/BEWE/DataBase}"
OUT="$DB/tle"
CONF="${BEWE_ST_CONF:-/etc/bewe-spacetrack.conf}"
UA='Mozilla/5.0'
mkdir -p "$OUT"

TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT

log(){ echo "[tle-sync] $*"; }

# 3줄 TLE 스트림을 LEO·비Starlink 로 거르고 **NORAD 당 target 에폭에 가장 가까운
# 한 세트만** 남긴다. gp_history 는 하루에 위성당 10~15 세트를 주므로 이게 없으면
# 13 MB 가 그대로 쌓인다 (중복 제거 후 ~0.9 MB).
filter_dedup(){  # $1=입력 $2=출력 $3=target YYDDD.0 $4=drop_starlink(1/0) $5=leo_only(1/0)
  awk -v TARGET="$3" -v DROPSL="${4:-1}" -v LEOONLY="${5:-1}" '
    function alt(mm,  n,a){ if(mm<=0) return 1e9; n=mm*6.283185307179586/86400.0;
                            a=exp(log(398600.4418/(n*n))/3.0); return a-6378.137 }
    { B[NR%3==1?0:(NR%3==2?1:2)] = $0 }
    NR%3==0 {
      nm=B[0]; l1=B[1]; l2=B[2]
      sub(/^0 /,"",nm)
      if(substr(l1,1,1)!="1" || substr(l2,1,1)!="2") next
      if(DROPSL=="1" && index(toupper(nm),"STARLINK")>0) next
      if(LEOONLY=="1" && alt(substr(l2,53,11)+0)>2000) next
      id=substr(l1,3,5)+0
      ep=substr(l1,19,14)+0
      d=ep-TARGET; if(d<0) d=-d
      if(!(id in best) || d<best[id]){ best[id]=d; N[id]=nm; A[id]=l1; C[id]=l2 }
    }
    END{ for(k in N) print N[k] "\n" A[k] "\n" C[k] }
  ' "$1" > "$2"
}

# YYYY-MM-DD -> YYDDD.5 (그날 정오)
yyddd(){ python3 -c "
import datetime,sys
d=datetime.date.fromisoformat(sys.argv[1])
print('%02d%03d.5'%(d.year%100,(d-datetime.date(d.year,1,1)).days+1))" "$1"; }

fetch_today(){
  log "Celestrak GROUP=active ..."
  if ! curl --max-time 90 -fsS -A "$UA" \
       'https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=tle' \
       -o "$TMP/raw"; then log "celestrak fetch failed"; return 1; fi
  # gp.php 는 2줄 앞에 이름 줄이 오는 3줄 포맷 (0 접두어 없음) — 그대로 먹인다.
  local med; med="$(awk 'NR%3==2{print substr($0,19,14)+0}' "$TMP/raw" | sort -n | awk '{a[NR]=$1} END{print a[int((NR+1)/2)]}')"
  local date; date="$(python3 -c "
import datetime,sys
v=float(sys.argv[1]); yy=int(v//1000); ddd=v-yy*1000
y=1900+yy if yy>=57 else 2000+yy
print((datetime.date(y,1,1)+datetime.timedelta(days=ddd-1)).strftime('%Y%m%d'))" "$med")"
  # leo_ : LEO 페이로드, Starlink 제외 — 도플러 매칭 기본값 (작고 분리도가 좋다)
  filter_dedup "$TMP/raw" "$TMP/f" "$med" 1 1
  install_out "$TMP/f" "$date" leo
  # all_ : 전 페이로드 + Starlink, 고도 제한 없음 — 지구본과 Starlink 포함 매칭용.
  #        Celestrak GROUP=active 는 이미 활성 페이로드만이라 파편·로켓잔해는 안 들어온다.
  curl --max-time 90 -fsS -A "$UA" \
       'https://celestrak.org/NORAD/elements/gp.php?GROUP=starlink&FORMAT=tle' \
       -o "$TMP/sl" 2>/dev/null || : > "$TMP/sl"
  cat "$TMP/raw" "$TMP/sl" > "$TMP/both"
  filter_dedup "$TMP/both" "$TMP/fa" "$med" 0 0
  install_out "$TMP/fa" "$date" all
}

st_login(){
  [ -r "$CONF" ] || { log "no credentials at $CONF"; return 1; }
  set -a; . "$CONF"; set +a
  : "${ST_USER:?}" "${ST_PASS:?}"
  curl -s -c "$TMP/ck" -o /dev/null --max-time 40 \
    https://www.space-track.org/ajaxauth/login \
    --data-urlencode "identity=$ST_USER" --data-urlencode "password=$ST_PASS" \
    && [ -s "$TMP/ck" ]
}

fetch_past(){  # $1=YYYY-MM-DD
  local d="$1" nxt
  nxt="$(python3 -c "
import datetime,sys
print((datetime.date.fromisoformat(sys.argv[1])+datetime.timedelta(days=1)).isoformat())" "$d")"
  local dst="$OUT/all_${d//-/}.txt"
  [ -e "$dst" ] && { log "$d already present, skip"; return 0; }
  log "space-track gp_history $d ..."
  local q="https://www.space-track.org/basicspacedata/query/class/gp_history"
  # OBJECT_TYPE=PAYLOAD — 파편·로켓잔해는 송신을 안 하므로 후보에 있으면 오탐만
  # 늘린다. gp_history 는 안 거르면 전 카탈로그를 준다 (실측 14,116 vs 페이로드만 4,747).
  # 고도 제한을 안 걸고 페이로드 전체를 한 번만 받아 두 벌로 거른다 (요청 절약).
  q="$q/EPOCH/${d}--${nxt}/OBJECT_TYPE/PAYLOAD"
  q="$q/orderby/NORAD_CAT_ID/format/3le"
  if ! curl -s -b "$TMP/ck" --max-time 300 -o "$TMP/raw" "$q"; then
    log "$d fetch failed"; return 1; fi
  [ -s "$TMP/raw" ] || { log "$d empty"; return 1; }
  filter_dedup "$TMP/raw" "$TMP/f"  "$(yyddd "$d")" 1 1
  install_out "$TMP/f"  "${d//-/}" leo
  filter_dedup "$TMP/raw" "$TMP/fa" "$(yyddd "$d")" 0 0
  install_out "$TMP/fa" "${d//-/}" all
  sleep 4          # 요청 제한 (계정당 분 30 / 시 300) 여유
}

install_out(){  # $1=filtered $2=YYYYMMDD $3=prefix(leo|all)
  local pre="${3:-leo}"
  local n; n=$(( $(wc -l < "$1") / 3 ))
  [ "$n" -gt 0 ] || { log "$2: 0 records after filter, skipping"; return 1; }
  # **오늘 날짜를 소급 요청하면 반쪽짜리가 나온다** — gp_history 는 그 날 올라온
  # 만큼만 주므로 하루가 끝나기 전엔 수십 건뿐이다 (실측 2026-08-07: LEO 76건 vs
  # 전날 4530건). 그런 파일이 들어오면 매칭 쪽 "녹화일에 가장 가까운 날짜" 규칙이
  # 완전한 전날 것 대신 이걸 골라 정답 위성이 아예 목록에 없게 된다.
  # 같은 계열의 최근 완전본에 한참 못 미치면 버린다.
  local prev; prev=$(ls -1 "$OUT/${pre}_"*.txt 2>/dev/null | sort | tail -1)
  if [ -n "$prev" ] && [ "$prev" != "$OUT/${pre}_$2.txt" ]; then
    local pn; pn=$(( $(wc -l < "$prev") / 3 ))
    if [ "$pn" -gt 100 ] && [ "$n" -lt $(( pn / 2 )) ]; then
      log "${pre}_$2: only $n sats vs $pn in $(basename "$prev") - partial day, skipping"
      return 1
    fi
  fi
  mv -f "$1" "$OUT/${pre}_$2.txt"
  log "${pre}_$2: $n sats, $(du -h "$OUT/${pre}_$2.txt" | cut -f1)"
}

if [ $# -eq 0 ]; then
  fetch_today
elif [ $# -eq 1 ]; then
  st_login && fetch_past "$1"
else
  st_login || exit 1
  cur="$1"
  while [ "$cur" != "$(python3 -c "
import datetime,sys
print((datetime.date.fromisoformat(sys.argv[1])+datetime.timedelta(days=1)).isoformat())" "$2")" ]; do
    fetch_past "$cur"
    cur="$(python3 -c "
import datetime,sys
print((datetime.date.fromisoformat(sys.argv[1])+datetime.timedelta(days=1)).isoformat())" "$cur")"
  done
fi
