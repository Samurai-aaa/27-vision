## 目录结构

分为三个task,每个task有自己的CHANGELOG.md、README.md
各个任务点的输出视频在各个目录的video_output里

```
27-vision-<name>/
├── README.md                    # 本文件
├── docs/                        # 任务报告（导出 PDF 放这里）
├── video_input/                 # 共享测试视频（blu.avi / red.avi / 装甲板.avi）
├── task1_armor_detection/       # 任务1：装甲板检测
│   ├── traditional_opencv/      # 传统 OpenCV 算法
│   └── rp24_nn/                 # RP24 神经网络部署
├── task2_PnP/                   # 任务2：PnP 位姿解算
    └── docs                     # 几种PnP解算推导
│   └── rp24_nn_pnp/             # RP24 神经网络检测 + PnP 解算
└── task3_KF/                    # 任务3：卡尔曼滤波追踪器
    └── armor_interfaces/        # 装甲板信息接口文件
    └── detector                 # 装甲板识别器
    └── tracker                  # 装甲板追踪器
```
