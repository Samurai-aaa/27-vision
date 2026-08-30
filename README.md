## 目录结构

分为三个task,每个task有自己的CHANGELOG.md和README.md

```
27-vision-<name>/
├── README.md                    # 本文件
├── CHANGELOG.md                 # 顶层版本记录
├── docs/                        # 任务报告（导出 PDF 放这里）
├── video_input/                 # 共享测试视频（blu.avi / red.avi / 装甲板.avi）
├── task1_armor_detection/       # 任务1：装甲板检测
│   ├── traditional_opencv/      # 传统 OpenCV 算法
│   └── rp24_nn/                 # RP24 神经网络部署
├── task2_PnP/                  # 任务2：PnP 位姿解算
│   └── rp24_nn_pnp/           # RP24 神经网络检测 + PnP 解算
└── task3_KF/
```
