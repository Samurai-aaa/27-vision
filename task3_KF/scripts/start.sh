#!/usr/bin/env bash
# task3_KF 一键录制：起 video_player + detector + 一条追踪链路，把源视频"从头到尾完整
# 播一遍"的渲染录成 avi，播完自动停全部节点。不再分手动/自动、不用手传秒数——完整一遍
# 时长自动算。选哪条追踪链路由第一个参数决定：
#
#   ./scripts/start.sh             整车 EKF（tracker_node）      → video_output/track_ekf.avi
#   ./scripts/start.sh ekf         同上（显式）
#   ./scripts/start.sh kf [CV|CA]  单板普通 KF（simple_tracker_node）
#                                    model 空 = 用 config 默认(CV) → track_kf.avi
#                                    model CV/CA                  → track_kf_CV.avi / track_kf_CA.avi
#
# 时长怎么来：完整一遍 = 源视频总帧数 / video_player 播放 fps。脚本先调
# scripts/video_full_secs.py（读 detector/config/video_player.yaml 的 video_path/fps，
# OpenCV 取总帧数）算出墙钟秒数，录制窗口 = 一遍时长 + 余量（覆盖节点启动/录制器起点的
# 时间差），到点统一收尾停节点。video_player 默认 loop:false，播完一遍节点自然退出。
#
# 想改成实时/持续联调：把 video_player.yaml 的 loop 改回 true，脚本会一直录到 Ctrl+C。
set -o pipefail
# 参数最先抓（openvino 的 setupvars.sh 会消费位置参数，source 后 $1 会变空）；
# 不能在 source ament setup 前 set -u，humble setup.bash 会读未绑定变量。
sel="${1:-ekf}"          # ekf | kf
model="${2:-}"           # kf 时可选 CV | CA（空则用 config 默认，yaml 现为 CV）
case "$sel" in
  ekf)
    TRACK="tracker tracker.launch.py"             # 整车 EKF 链路
    REC_TOPIC="/tracker/final_img"
    OUT_AVI="track_ekf.avi"
    TRACK_LOG="tracker.log"
    LABEL="整车 EKF"
    ;;
  kf)
    TRACK="tracker simple_tracker.launch.py"      # 单板普通 KF 链路（需求①② CV/CA）
    REC_TOPIC="/simple_tracker/final_img"
    TRACK_LOG="simple_tracker.log"
    if [ -n "$model" ]; then
      case "$model" in
        CV|CA) OUT_AVI="track_kf_${model}.avi" ;;
        *) echo "model 只能是 CV 或 CA（你给了 '$model'）"; exit 2 ;;
      esac
      LABEL="单板普通 KF（model=$model）"
    else
      OUT_AVI="track_kf.avi"
      LABEL="单板普通 KF（model 用 config 默认）"
    fi
    ;;
  *)
    echo "未知链路 '$sel'：用 ekf（整车EKF，默认）或 kf [CV|CA]（单板普通KF）"
    exit 2
    ;;
esac

cd "$(dirname "$0")/.."          # 切到工作区根 task3_KF
WS="$(pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source /opt/intel/openvino_2024.6.0/setupvars.sh >/dev/null 2>&1 || true
set -u

OUT="$WS/video_output"; LOG="$WS/log"; mkdir -p "$OUT" "$LOG"
OUT_AVI_PATH="$OUT/$OUT_AVI"

# 只写括号形式，避免 pkill 匹配到进程自身的命令行而自杀
# 注：[t]racker_node 同时覆盖 simple_tracker_node（两者命令行都含 tracker_node 子串）
kill_all() {
  pkill -f '[v]ideo_player_node' 2>/dev/null || true
  pkill -f '[a]rmor_detector'    2>/dev/null || true
  pkill -f '[t]racker_node'      2>/dev/null || true
  pkill -f '[r]ecord_video.py'   2>/dev/null || true
}
trap 'kill_all' EXIT
kill_all; sleep 0.5

# 自动算源视频完整播一遍的墙钟秒数
FULL_SECS="$(python3 "$WS/scripts/video_full_secs.py")" || {
  echo ">> 算源视频完整时长失败（见上面错误），中止"; exit 1; }
# 录制窗口多留几秒：录制器起点比 video_player 首帧略晚，多加余量保证录到 EOF；
# loop:false 播完即停，末尾超出的窗口只是空等，不会产生多余帧。
REC_SECS="$(python3 -c "print(round($FULL_SECS + 6, 1))")"
echo ">> [$LABEL] 源视频完整播一遍约 ${FULL_SECS}s，录制窗口 ${REC_SECS}s"

echo ">> 启动 video_player / detector / $LABEL（日志：task3_KF/log/*.log）"
ros2 launch detector video_player.launch.py     > "$LOG/video_player.log" 2>&1 &
ros2 launch detector detector.launch.py         > "$LOG/detector.log"     2>&1 &
ros2 launch $TRACK ${model:+model:=$model} > "$LOG/$TRACK_LOG" 2>&1 &

echo ">> 开始把 $REC_TOPIC 录成 $OUT_AVI_PATH（播完自动停）"
python3 "$WS/scripts/record_video.py" "$OUT_AVI_PATH" "$REC_SECS" "$REC_TOPIC"
rc=$?
if [ "$rc" -ne 0 ]; then
  echo ">> 录制被中断或失败（退出码 $rc），已收尾（未播完整一遍）"
  exit 1
fi

echo ">> 源视频已播完一遍，收尾停止节点"
ls -la "$OUT_AVI_PATH" 2>/dev/null
echo "完成：$OUT_AVI_PATH"
exit 0
