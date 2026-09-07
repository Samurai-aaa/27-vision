#!/usr/bin/env python3
"""录制某条链路 /xxx/final_img 到视频文件。

用法：record_video.py <输出.avi> <时长秒数> [话题]
     话题默认 /tracker/final_img（整车 EKF）；单板普通 KF 传 /simple_tracker/final_img。
定时 watch 到点自动 release 退出；输出路径由调用方创建目录。
"""
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class Rec(Node):
    def __init__(self, out_path, secs, topic='/tracker/final_img'):
        super().__init__('record_video')
        self.br = CvBridge()
        self.out_path = out_path
        self.secs = float(secs)
        self.topic = topic
        self.t0 = time.time()
        self.w = None
        self.t_first = None
        self.frames = 0
        # 发布端为 best-effort（SensorDataQoS），这里也要用 best-effort 才收得到
        img_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Image, self.topic, self.on_img, img_qos)
        self.create_timer(0.5, self.watch)
        print(f'开始录制 → {out_path}，最长 {self.secs}s（等待 {self.topic} ...）', flush=True)

    def on_img(self, m):
        try:
            img = self.br.imgmsg_to_cv2(m, 'bgr8')
        except Exception:
            return
        if self.w is None:
            h, w = img.shape[:2]
            os.makedirs(os.path.dirname(os.path.abspath(self.out_path)), exist_ok=True)
            self.w = cv2.VideoWriter(self.out_path, cv2.VideoWriter_fourcc(*'MJPG'), 30.0, (w, h))
            if not self.w.isOpened():
                print(f'VideoWriter 打开失败: {self.out_path}', flush=True)
                self.w = None
                return
            self.t_first = time.time()
            print(f'首帧 {w}x{h}，开始写入', flush=True)
        self.w.write(img)
        self.frames += 1

    def watch(self):
        now = time.time()
        if now - self.t0 < self.secs:
            return
        if self.w is None:
            print(f'超时未收到任何 {self.topic} 帧，未生成视频', flush=True)
            os._exit(1)
        el = now - self.t_first
        self.w.release()
        eff = self.frames / el if el > 0 else 0.0
        print(f'写入 {self.frames} 帧，实测 {eff:.1f} fps（{el:.1f}s）→ {self.out_path}', flush=True)
        if self.frames == 0 and os.path.exists(self.out_path):
            os.remove(self.out_path)
        os._exit(0)


def main():
    if len(sys.argv) < 3:
        print('用法: record_video.py <输出.avi> <时长秒数> [话题(默认 /tracker/final_img)]')
        sys.exit(2)
    topic = sys.argv[3] if len(sys.argv) >= 4 else '/tracker/final_img'
    rclpy.init()
    n = Rec(sys.argv[1], sys.argv[2], topic)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        # 让 Ctrl+C 以非零码退出：调用方（start.sh）据此识别"被中断"，不误报"播完"
        print('录制被 Ctrl+C 中断', flush=True)
        raise SystemExit(130)


if __name__ == '__main__':
    main()
