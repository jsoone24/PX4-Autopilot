# File: launch_sitl.sh
# Usage: ./launch_sitl.sh <N_INSTANCES> <SIM_SPEED>
#!/usr/bin/env bash
set -e

PX4_DIR=~/ws/PX4-Autopilot
N=${1:-8}
SPEED=${2:-10}

echo "Launching $N SITL+Gazebo instances at ${SPEED}× speed..."
for ((i=0;i<N;i++)); do
  WDIR=~/ws/PX4-Autopilot/_RLModel/tmp/px4_instance_$i
  PORT=$((14540 + i * 10))
  mkdir -p "$WDIR"

  PX4_SIM_SPEED_FACTOR=${SPEED} \
  PX4_SIM_PORT_OFFSET=$((i * 10)) \
  HEADLESS=1 \
  ${PX4_DIR}/build/px4_sitl_default/bin/px4 \
    -i $i -w "$WDIR" \
    ${PX4_DIR}/etc/init.d-posix/rcS > "$WDIR/out.log" 2>&1 &

  echo "  • Instance $i @ udp://:$PORT"
done
