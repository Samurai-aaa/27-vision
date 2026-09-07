#!/usr/bin/env bash
# task3_KF 诊断版手动启动：与 start.sh 手动模式相同，但监控三个节点，
# 一旦某节点**自然退出**（不是 Ctrl+C），当场打印“哪个先退出 + 它日志尾部 +
# 其余节点日志尾部”，然后清理退出。用于抓“视频突然暂停、后台跟着全退”的根因。
#
# 用法（在 task3_KF 下）：
#   ./scripts/start_probe.sh     前台跑；另开终端 rqt_image_view 看 /tracker/final_img
#   出现暂停/全退后，把本终端输出整段发出来即可。Ctrl+C 正常停止全部。
set -o pipefail
# 注意：不能在 source ament setup 前 set -u，humble setup.bash 会读未绑定变量
cd "$(dirname "$0")/.."
WS="$(pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source /opt/intel/openvino_2024.6.0/setupvars.sh >/dev/null 2>&1 || true
set -u

LOG="$WS/log"; mkdir -p "$LOG"
VP_LOG="$LOG/video_player.log"; DP_LOG="$LOG/detector.log"; TP_LOG="$LOG/tracker.log"

stop_all() {
  pkill -f '[v]ideo_player_node' 2>/dev/null || true
  pkill -f '[a]rmor_detector'    2>/dev/null || true
  pkill -f '[t]racker_node'      2>/dev/null || true
}
INTERRUPTED=0
trap 'INTERRUPTED=1; echo; echo "已停止全部节点(Ctrl+C)"; stop_all' INT TERM
trap 'stop_all' EXIT
stop_all; sleep 0.6

echo ">> 启动 video_player / detector / tracker（诊断版，监控哪个先退）"
ros2 launch detector video_player.launch.py > "$VP_LOG" 2>&1 & VP=$!
ros2 launch detector detector.launch.py     > "$DP_LOG" 2>&1 & DP=$!
ros2 launch tracker   tracker.launch.py     > "$TP_LOG" 2>&1 & TP=$!
sleep 3

echo ">> 手动模式：另开终端 rqt_image_view /tracker/final_img，Ctrl+C 停止全部"
echo ">> 三 launch 进程：video_player=$VP detector=$DP tracker=$TP"

dump_tail() {  # dump_tail <名字> <文件>
  echo "----- $1 日志尾部 ($2) -----"
  tail -n 15 "$2" 2>/dev/null || echo "(无 $2)"
}

DEAD=""
while [ $INTERRUPTED -eq 0 ]; do
  for pair in "video_player $VP $VP_LOG" "detector $DP $DP_LOG" "tracker $TP $TP_LOG"; do
    set -- $pair; name="$1"; pid="$2"; pfile="$3"
    if ! kill -0 "$pid" 2>/dev/null; then
      # 进程已消失。区分：我们自己停的(INTERRUPTED/stop_all) vs 它自然退出
      if [ $INTERRUPTED -eq 0 ]; then
        echo
        echo ">>>>>>> [$name] (pid $pid) 自然退出！trap 将停止其余节点 <<<<<<<"
        dump_tail "$name" "$pfile"
        # 也顺带打印另两个的状态，判断是不是同时都退了
        for pair2 in "video_player $VP $VP_LOG" "detector $DP $DP_LOG" "tracker $TP $TP_LOG"; do
          set -- $pair2; n2="$1"; p2="$2"; f2="$3"
          if kill -0 "$p2" 2>/dev/null; then
            echo "  [$n2] (pid $p2) 仍存活"
          else
            echo "  [$n2] (pid $p2) 也已退出"
            dump_tail "$n2" "$f2"
          fi
        done
        DEAD="$name"
        break 2
      fi
    fi
  done
  sleep 0.5
done

echo
echo ">> 退出原因：$([ -n "$DEAD" ] && echo "节点 $DEAD 先退出（见上）" || echo "Ctrl+C 手动停止")"
echo ">> 三个日志文件留存：$VP_LOG / $DP_LOG / $TP_LOG"
exit 0
