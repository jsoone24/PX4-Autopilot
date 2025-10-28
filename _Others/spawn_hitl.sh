#!/usr/bin/env bash
set -euo pipefail

# ===== Defaults (편집 가능) =====
DEFAULT_LAT="47.39855040647849"
DEFAULT_LON="8.545290332727657"
DEFAULT_RADIUS="220"      # meters
DEFAULT_ALT="0"           # meters (항상 0 권장)
DEFAULT_DEVICE="/dev/ttyACM0"
DEFAULT_BAUD="921600"
DEFAULT_RATE="250"
METERS_PER_DEGREE_LAT="111319.5"

# ===== Args =====
NUM=""
IDX=""
LAT="$DEFAULT_LAT"
LON="$DEFAULT_LON"
RADIUS="$DEFAULT_RADIUS"
ALT="$DEFAULT_ALT"
DEVICE="$DEFAULT_DEVICE"
BAUD="$DEFAULT_BAUD"
RATE="$DEFAULT_RATE"
QUIET="-q"
SITL="-s"
DRY_RUN="0"
ONLY_EXPORT="0"
EXTRA_ARGS=()

usage() {
  cat <<EOF
Usage:
  $(basename "$0") --num N --idx I [--radius M] [--lat LAT --lon LON] [--alt ALT]
                   [--device DEV] [--baud BPS] [--rate HZ] [--no-quiet] [--no-sitl]
                   [--dry-run] [--only-export] [--] [extra jmavsim args...]

Description:
  12시 방향을 1번, 시계방향으로 증가하는 인덱스 I에 해당하는 원주점에 PX4_HOME을 설정하고 jmavsim을 실행.

Examples:
  $(basename "$0") --num 14 --idx 1
  $(basename "$0") --num 14 --idx 7 --radius 300
  $(basename "$0") --num 8 --idx 3 --lat 47.39855 --lon 8.54529 --dry-run

EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --num) NUM="$2"; shift 2 ;;
    --idx) IDX="$2"; shift 2 ;;
    --radius) RADIUS="$2"; shift 2 ;;
    --lat) LAT="$2"; shift 2 ;;
    --lon) LON="$2"; shift 2 ;;
    --alt) ALT="$2"; shift 2 ;;
    --device) DEVICE="$2"; shift 2 ;;
    --baud) BAUD="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --no-quiet) QUIET=""; shift ;;
    --no-sitl) SITL=""; shift ;;
    --dry-run) DRY_RUN="1"; shift ;;
    --only-export) ONLY_EXPORT="1"; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; EXTRA_ARGS+=("$@"); break ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

# ===== Validate =====
if [[ -z "${NUM}" || -z "${IDX}" ]]; then
  echo "[ERROR] --num 와 --idx 는 필수" >&2
  usage; exit 1
fi
if ! [[ "$NUM" =~ ^[0-9]+$ && "$NUM" -ge 1 ]]; then
  echo "[ERROR] --num 은 1 이상의 정수" >&2; exit 1
fi
if ! [[ "$IDX" =~ ^[0-9]+$ && "$IDX" -ge 1 && "$IDX" -le "$NUM" ]]; then
  echo "[ERROR] --idx 는 1..$NUM 범위의 정수" >&2; exit 1
fi

# ===== Compute spawn point using awk (radians + sin/cos) =====
read CALC_LAT CALC_LON <<EOF
$(awk -v lat="$LAT" -v lon="$LON" -v num="$NUM" -v idx="$IDX" \
     -v r="$RADIUS" -v mdeg="$METERS_PER_DEGREE_LAT" '
BEGIN{
  # theta: 12시=0 rad, 시계방향 증가
  # dn=북쪽(+), de=동쪽(+) 기준
  pi=atan2(0,-1)
  theta = 2*pi*(idx-1)/num
  dn = r * cos(theta)
  de = r * sin(theta)
  dlat = dn / mdeg
  dlon = de / (mdeg * cos(lat*pi/180.0))
  printf("%.9f %.9f\n", lat + dlat, lon + dlon)
}')
EOF

# ===== Export PX4 home =====
export PX4_HOME_LAT="${CALC_LAT}"
export PX4_HOME_LON="${CALC_LON}"
export PX4_HOME_ALT="${ALT}"

echo "[INFO] Target (center):    LAT=${LAT}, LON=${LON}"
echo "[INFO] Radius (meters):    ${RADIUS}"
echo "[INFO] Points / Index:     ${NUM} / ${IDX} (12시=1, 시계방향)"
echo "[INFO] Spawned PX4_HOME:   LAT=${PX4_HOME_LAT}, LON=${PX4_HOME_LON}, ALT=${PX4_HOME_ALT}"

# ===== Optionally stop here =====
if [[ "$ONLY_EXPORT" == "1" || "$DRY_RUN" == "1" ]]; then
  echo "[INFO] DRY_RUN/ONLY_EXPORT 활성화로 jmavsim 실행은 건너뜀."
  exit 0
fi

# ===== Launch jmavsim (HITL) =====
JMAVSIM="./Tools/simulation/jmavsim/jmavsim_run.sh"
if [[ ! -x "$JMAVSIM" ]]; then
  echo "[ERROR] $JMAVSIM 실행 파일을 찾을 수 없음. PX4 소스 루트에서 실행하세요." >&2
  exit 1
fi

set -x
"$JMAVSIM" $QUIET $SITL -d "$DEVICE" -b "$BAUD" -r "$RATE" "${EXTRA_ARGS[@]}"

