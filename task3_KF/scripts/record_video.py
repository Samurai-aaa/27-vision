#!/usr/bin/env python3
"""录制某条链路 /xxx/final_img 到视频文件。

用法：record_video.py <输出.avi> <时长秒数> [话题]
     话题默认 /tracker/final_img（整车 EKF）；单板普通 KF 传 /simple_tracker/final_img。
定时 watch 到点自动 release 退出；输出路径由调用方创建目录。

写盘在独立线程里做（订阅回调只入队）：MJPG 1440×1080 实测写入吞吐只有 ~35fps，
回调里直接 write 会被编码拖住，best-effort 订阅队列一溢出就丢帧，成片比源视频短一截。
"""
import os
import queue
import sys
import threading
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
        self.t_first = None
        self.got_frame = False   # 收到过任何一帧？（区分"一直没帧"的超时）
        self.frames = 0          # 订阅收到并转换成功的帧数
        self.written = 0         # 真正写进文件的帧数
        self.dropped = 0         # 队列满被丢掉的帧数
        self.w = None
        self.open_failed = False
        # 必须与 /tracker/final_img 发布端同为 reliable，否则 QoS 不匹配收不到帧。
        # 这里用过 best-effort：1440x1080x3 = 4.6MB/帧的大消息在 best-effort 下，读端
        # 只要瞬时落后就**整帧静默丢弃**（实测最后一跳仍丢 1.8%，成片 3243 帧 vs 源
        # 3301 帧）。录制是"全都要"的消费者，本来就该用 reliable。
        img_qos = QoSProfile(depth=60, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Image, self.topic, self.on_img, img_qos)

        # 写盘与订阅解耦：编码吞吐（~35fps）比输入帧率（30fps）高，队列就能把抖动吸收掉；
        # 回调里同步 write 的话，编码一慢就拖住 executor → 队列溢出丢帧
        self.q = queue.Queue(maxsize=120)   # ~4s @30fps
        self.sentinel = object()
        self.writer = threading.Thread(target=self.write_loop, daemon=True)
        self.writer.start()

        self.create_timer(0.5, self.watch)
        print(f'开始录制 → {out_path}，最长 {self.secs}s（等待 {self.topic} ...）', flush=True)

    def on_img(self, m):
        try:
            img = self.br.imgmsg_to_cv2(m, 'bgr8')
        except Exception:
            return
        if self.t_first is None:
            self.t_first = time.time()
            self.got_frame = True
        self.frames += 1
        try:
            self.q.put_nowait(img)
        except queue.Full:
            self.dropped += 1

    def write_loop(self):
        """独立写线程：首帧到达时按实际分辨率建 VideoWriter，之后循环出队写盘。"""
        while True:
            img = self.q.get()
            if img is self.sentinel:
                break
            if self.w is None:
                if self.open_failed:
                    continue
                h, w = img.shape[:2]
                os.makedirs(os.path.dirname(os.path.abspath(self.out_path)), exist_ok=True)
                self.w = cv2.VideoWriter(self.out_path, cv2.VideoWriter_fourcc(*'MJPG'), 30.0, (w, h))
                if not self.w.isOpened():
                    print(f'VideoWriter 打开失败: {self.out_path}', flush=True)
                    self.w = None
                    self.open_failed = True
                    continue
                print(f'首帧 {w}x{h}，开始写入', flush=True)
            self.w.write(img)
            self.written += 1

    def watch(self):
        now = time.time()
        # 计时基准取"首帧到达"而不是进程启动：节点启动/模型预热那十几秒不该占用录制
        # 窗口，否则首帧晚到多久、录像就整体后移多久、源视频尾部就被截掉多久。
        # 首帧还没来（t_first is None）时退回 t0，保证"一直没帧"能照常超时退出。
        ref = self.t_first if self.t_first is not None else self.t0
        if now - ref < self.secs:
            return
        if not self.got_frame:
            print(f'超时未收到任何 {self.topic} 帧，未生成视频', flush=True)
            os._exit(1)
        # 通知写线程收尾，并把队列里积压的帧写完再释放文件
        try:
            self.q.put(self.sentinel, timeout=10)
        except queue.Full:
            pass
        self.writer.join(timeout=30)
        el = now - self.t_first
        if self.w is not None and not self.writer.is_alive():
            self.w.release()
        eff = self.written / el if el > 0 else 0.0
        msg = (f'写入 {self.written} 帧（收到 {self.frames} 帧），实测 {eff:.1f} fps'
               f'（{el:.1f}s）→ {self.out_path}')
        if self.dropped:
            msg += f'，队列满丢弃 {self.dropped} 帧'
        elif self.frames > self.written:
            msg += f'，{self.frames - self.written} 帧仍在队列中未落盘'
        print(msg, flush=True)
        if self.written == 0 and os.path.exists(self.out_path):
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
