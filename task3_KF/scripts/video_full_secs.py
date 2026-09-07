#!/usr/bin/env python3
"""打印源视频在 video_player 配置下"从头到尾完整播一遍"所需的墙钟秒数（一键录制收尾用）。

读与 video_player 运行时同源的参数（install/detector/share/detector/config/video_player.yaml，
源码目录回退）里的 video_path / fps，用 OpenCV 读总帧数：
    秒数 = 总帧数 / 播放fps（fps<=0 时取视频自身帧率）
只播一遍（loop:false）时，video_player 到点自然退出，本秒数即完整一遍的录制时长下界。

用法：video_full_secs.py   → stdout 打印一个浮点（秒），信息行走 stderr；失败非零退出。
"""
import os
import sys

import cv2
import yaml


def find_params_file():
    """优先取 install 下与运行时 launch 读取的同份参数；找不到再退回源码 config。"""
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(
            get_package_share_directory('detector'), 'config', 'video_player.yaml'))
    except Exception:
        pass
    cands.append(os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'detector', 'config',
        'video_player.yaml')))
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


def main():
    cfg = find_params_file()
    if not cfg:
        print('找不到 video_player.yaml（先 colcon build，或确认源码 config 存在）', file=sys.stderr)
        sys.exit(1)
    params = yaml.safe_load(open(cfg))['video_player']['ros__parameters']
    path = os.path.expanduser(str(params['video_path']))
    if not os.path.isfile(path):
        print(f'源视频不存在: {path}', file=sys.stderr)
        sys.exit(1)
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f'打不开源视频: {path}', file=sys.stderr)
        sys.exit(1)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    src = cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.release()
    play = float(params.get('fps') or 0)
    if play <= 0:
        play = src
    if total <= 0:
        print(f'读不到源视频总帧数: {path}', file=sys.stderr)
        sys.exit(1)
    secs = total / play
    print(f'源视频 {total} 帧 @播放 {play:.1f}fps（源 {src:.1f}fps）→ 完整一遍约 {secs:.0f}s',
          file=sys.stderr)
    print('%.1f' % secs)


if __name__ == '__main__':
    main()
