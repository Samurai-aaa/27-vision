# task3_KF —— 卡尔曼滤波装甲板跟踪（ROS 2 工作区）

在 task2（NN 检测 + PnP 解算）基础上，用 KF/EKF 对目标做状态估计、整车建模与预测外推。
算法核心为纯 C++（不依赖 rclcpp），节点间以话题通信。

## 1. 项目说明

| 方向 | 算法层 | 状态向量 | 对应需求 |
|---|---|---|---|
| 普通 KF（CV/CA 对比） | `tracker/simple_target` | 装甲板心：CV 6 维 / CA 9 维 | ① 状态估计+掉帧强制可视化；② 运动模型对比 |
| EKF 整车建模 | `tracker/target` | 车心+速度+朝向+转速+半径/高差（11 维，小陀螺） | ③；④⑤ |
| 跟踪状态机 | `tracker/tracker` | LOST / DETECTING / TRACKING / TEMP_LOST | ④ 多车锁定 / 掉帧外推 |

数据流：
`video_player(/image)` → `detector(/armors)` → `tracker_node(/tracker/target, /tracker/marker)`

掉帧语义（需求①）：漏检帧 → tracker 进 TEMP_LOST 纯预测外推并继续发布；图像断流 → 100ms
定时器兜底 predict，超过 `lost_time_thres` 才回 LOST。

## 2. 项目目录结构

```text
task3_KF/
├── README.md
├── armor_interfaces/              // 自定义消息包
│   └── msg/  Armor.msg / Armors.msg / Target.msg / TargetInfo.msg
├── detector/                      // NN 检测 + PnP 解算
│   ├── include/  Model/  src/
│   │   ├── detector_node.cpp      // /image → /armors（NN + solvePnP）
│   │   └── video_player_node.cpp  // 假相机：视频 → /image（上真车时删掉即可）
│   └── launch/
│       ├── video_player.launch.py  // 终端 1
│       └── detector.launch.py      // 终端 2
├── tracker/                       // KF / EKF 跟踪
│   ├── include/  src/
│   │   ├── extended_kalman_filter // 通用 KF/EKF 框架（predict/update，任意观测维度）
│   │   ├── simple_target          // 普通 KF：CV 6 维 / CA 9 维（①②）
│   │   ├── target                 // 整车 EKF：11 维（③）
│   │   ├── tracker                // 四态状态机 + 数据关联门控（④）
│   │   └── tracker_node           // /armors → /tracker/target（ROS 壳）
│   ├── config/
│   │   └── tracker.yaml           // 门控/阈值参数（调参入口）
│   └── launch/
│       └── tracker.launch.py      // 终端 3
├── build/  install/  log/         // colcon 自动生成，不提交
└── ../video_input/                // 测试视频（仓库顶层，在 task3_KF 外一层）
```

## 3. 启动测试

> 路径约定：工作区放本机 `~/Workspaces/27-vision-dev/task3_KF`，视频在其上一层
> `video_input/`。换机器只需改 `detector/launch/video_player.launch.py` 顶部的
> `default_video` 一处，其余命令照抄。

### 编译

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh   # detector 需要（含编译）
colcon build
source install/setup.bash
```

### 终端 1：启动假相机

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch detector video_player.launch.py
# 换视频：ros2 launch detector video_player.launch.py video_path:=<你的视频> fps:=30.0
```

### 终端 2：启动检测节点

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
source install/setup.bash
ros2 launch detector detector.launch.py            # 打蓝车：detect_color:=1
```

### 终端 3：启动跟踪节点

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tracker tracker.launch.py
```

### 终端 4：查看

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source install/setup.bash
# 检测标注图（2D）
ros2 run rqt_image_view rqt_image_view /armor_detector/final_img
# 原始帧
ros2 run rqt_image_view rqt_image_view /image
# 看跟踪是否在锁：/tracker/target 里 tracking=true
ros2 topic echo /tracker/target --once
```

## 4. 话题与参数

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/image` | Image | video_player | 原始帧（SensorDataQoS，可丢帧） |
| `/armors` | Armors | armor_detector | 检测结果（每帧都发，0 板也发） |
| `/armor_detector/final_img` | Image | armor_detector | 标注图（debug=true） |
| `/armor_detector/marker_array` | MarkerArray | armor_detector | 检测装甲板（3D） |
| `/tracker/target` | Target | tracker_node | 跟踪输出；TEMP_LOST 外推时也发 |
| `/tracker/marker` | MarkerArray | tracker_node | 整车估计（3D） |

`tracker/config/tracker.yaml`：`max_match_distance`（板心位置门控 m）、`max_match_yaw_diff`
（朝向门控 rad）、`tracking_thres`（TRACKING 需连续帧数）、`lost_time_thres`（TEMP_LOST
→ LOST 超时 s）。运行中动态调：`ros2 param set /tracker_node <名> <值>`。
