#!/usr/bin/env bash
# task3_KF：起 video_player + detector（不含 tracker），采集 /armors N 秒供整车标定。
# 用法（在 task3_KF 下）：./scripts/calib_run.sh [秒数，默认 30]
set -o pipefail
run_secs="${1:-30}"     # 参数最先抓：openvino setupvars.sh 会消费 $1

cd "$(dirname "$0")/.."
WS="$(pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source /opt/intel/openvino_2024.6.0/setupvars.sh >/dev/null 2>&1 || true
set -u

LOG="$WS/log"; mkdir -p "$LOG"
OUT="$LOG/armors.npz"
rm -f "$OUT"

kill_all() {
  pkill -f '[v]ideo_player_node' 2>/dev/null || true
  pkill -f '[a]rmor_detector'    2>/dev/null || true
}
trap 'kill_all' EXIT
kill_all; sleep 0.6

echo ">> 起 video_player + detector，采集 ${run_secs}s 板观测..."
ros2 launch detector video_player.launch.py > "$LOG/video_player.log" 2>&1 &
ros2 launch detector detector.launch.py     > "$LOG/detector.log"     2>&1 &
sleep 4
python3 "$WS/scripts/collect_armors.py" "$run_secs" "$OUT"
kill_all; sleep 0.5
echo ">> 标定数据在 $OUT；下一步：python3 scripts/calib_analyze.py $OUT"
