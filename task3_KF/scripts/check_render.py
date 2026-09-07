#!/usr/bin/env python3
"""自动观测 N 秒（每秒采样 1 帧渲染图，不做逐帧 decode）：
统计 /tracker/final_img 渲染（白=整车预测板模型 / 绿=正在追踪的实测框）与
/tracker/target 状态，并把"追踪帧"与"TEMP_LOST 外推帧"各存一张到 task3_KF/log/ 供核对。
用法：check_render.py <秒数>
"""
import os
import sys
import time

import cv2
import rclpy
from armor_interfaces.msg import Target
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

OUT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'log'))


class Check(Node):
    def __init__(self, secs):
        super().__init__('check_render')
        os.makedirs(OUT_DIR, exist_ok=True)
        self.br = CvBridge()
        self.secs = float(secs)
        self.t0 = time.time()
        self.last_s = self.t0
        self.latest_img = None
        self.samples = self.n_green = self.n_pred = 0
        self.n_t_trk = self.n_t_pred = 0
        self.saved_green = self.saved_pred = False
        self.latest_pred = False  # 最近一帧 target 是否 TEMP_LOST 外推
        # 发布端为 best-effort（SensorDataQoS），这里也要用 best-effort 才收得到
        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.sub_img = self.create_subscription(Image, '/tracker/final_img', self.on_img, img_qos)
        self.sub_t = self.create_subscription(Target, '/tracker/target', self.on_t, 10)
        # 0.5s 高频 watchdog（比单个长周期 timer 可靠，任何 executor 都必处理）：
        # 每 tick 检查是否到观测时长；到点则 done()；未到则每 ~1s 采样一次渲染帧
        self.create_timer(0.5, self.watch)

    def on_img(self, m):
        self.latest_img = m  # QoS depth=1，只保留最新，不做 decode

    def on_t(self, m):
        if m.tracking and m.predicted:
            self.n_t_pred += 1
            self.latest_pred = True
        elif m.tracking:
            self.n_t_trk += 1
            self.latest_pred = False

    def sample(self):
        if self.latest_img is None:
            return
        try:
            img = self.br.imgmsg_to_cv2(self.latest_img, 'bgr8')
        except Exception:
            return
        self.samples += 1
        # 渲染配色：白色线=整车预测板模型；绿色框=正在追踪的实测板（TEMP_LOST 外推帧
        # 无绿框）。故以 target 外推标志判定外推帧，绿像素只在非外推帧里找（存样例图）
        if self.latest_pred:
            self.n_pred += 1
            if not self.saved_pred:
                cv2.imwrite(os.path.join(OUT_DIR, 'check_pred.png'), img)
                self.saved_pred = True
        elif cv2.inRange(img, (0, 220, 0), (90, 255, 90)).any():
            self.n_green += 1
            if not self.saved_green:
                cv2.imwrite(os.path.join(OUT_DIR, 'check_track.png'), img)
                self.saved_green = True

    def watch(self):
        now = time.time()
        if now - self.t0 >= self.secs:
            self.done()
            return
        if now - self.last_s >= 1.0:
            self.last_s = now
            self.sample()

    def done(self):
        print('--- 渲染统计 ---', flush=True)
        print(f'final_img 采样帧={self.samples}  绿(追踪实测框)={self.n_green}  外推白板帧={self.n_pred}',
              flush=True)
        print(f'target     TRACKING(实测)={self.n_t_trk}  TEMP_LOST(外推)={self.n_t_pred}', flush=True)
        ok = self.n_t_trk > 0 and self.n_t_pred > 0
        print(f'存图: {os.path.join(OUT_DIR, "check_track.png")} / check_pred.png', flush=True)
        print('结论: ' + ('OK：追踪实测帧(绿框) 与 整车外推帧(白板模型) 都出现过' if ok
                          else '注意：本段未同时观察到两种状态（目标全程没丢，或视频里一直没锁）'),
              flush=True)
        # 硬退出：不能依赖 rclpy.shutdown() 在回调里能干净结束 spin
        os._exit(0)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    n = Check(secs)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
