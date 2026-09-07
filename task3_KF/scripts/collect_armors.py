#!/usr/bin/env python3
"""采集 /armors 一段时间,存板观测供离线标定整车参数(armor_num / 半径 / 符号 / v_yaw)。

用法: collect_armors.py <秒数> [输出.npz 默认 log/armors.npz]
数据: 每块可跟踪板一行 {stamp(图像秒), number, type, xyz, quat(wxyz)}。
与 tracker 层 trackable() 同口径: number∈1~5/O/G(排除 Bs/Bb 固定目标)。
"""
import os
import sys
import time

import numpy as np
import rclpy
from armor_interfaces.msg import Armors
from rclpy.node import Node

OUT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'log'))


def trackable(n):
    if n in ("O", "G"):
        return True
    return len(n) == 1 and n[0] in "12345"


class Collect(Node):
    def __init__(self, secs, out):
        super().__init__('collect_armors')
        self.secs = float(secs)
        self.out = out
        self.t0 = time.time()
        self.rows = []  # (stamp, number, type, x, y, z, qw, qx, qy, qz)
        self.msg_cnt = 0
        self.create_subscription(Armors, '/armors', self.on, 10)
        self.create_timer(0.5, self.watch)
        print(f'采集 {self.secs}s /armors（目标号 1~5/O/G）→ {out}', flush=True)

    def on(self, m):
        self.msg_cnt += 1
        for a in m.armors:
            if not trackable(a.number):
                continue
            p = a.pose.position
            q = a.pose.orientation
            self.rows.append(
                (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a.number, a.type,
                 p.x, p.y, p.z, q.w, q.x, q.y, q.z))

    def watch(self):
        if time.time() - self.t0 < self.secs:
            return
        if not self.rows:
            print('未采到任何板观测，检查 video_player / detector 是否在跑', flush=True)
            os._exit(1)
        arr = np.array(self.rows, dtype=[
            ('stamp', float), ('number', 'U2'), ('type', 'U4'),
            ('x', float), ('y', float), ('z', float),
            ('qw', float), ('qx', float), ('qy', float), ('qz', float)])
        os.makedirs(os.path.dirname(self.out), exist_ok=True)
        np.savez(self.out, arr=arr)
        # 按号统计板观测行数
        from collections import Counter
        print(f'采集结束: {len(self.rows)} 条板观测, /armors {self.msg_cnt} 帧 → {self.out}', flush=True)
        print('按号统计: ' + ', '.join(f'{k}={v}' for k, v in Counter(arr['number']).items()), flush=True)
        os._exit(0)


def main():
    if len(sys.argv) < 2:
        print('用法: collect_armors.py <秒数> [输出.npz]')
        sys.exit(2)
    secs = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(OUT, 'armors.npz')
    rclpy.init()
    n = Collect(secs, out)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
