# task3_KF —— 卡尔曼滤波装甲板跟踪（ROS 2 工作区）

在 task2（NN 检测 + PnP 解算）基础上，用 KF/EKF 对目标做状态估计、整车建模与预测外推。
算法核心为纯 C++（不依赖 rclcpp），节点间以话题通信。

## 1. 项目说明

| 方向 | 算法层 | 状态向量 |
|---|---|---|
| EKF 整车建模 | `tracker/target` | 车心+速度+朝向+转速+半径/高差（11 维，小陀螺） |
| 跟踪状态机 | `tracker/tracker` | LOST / DETECTING / TRACKING / TEMP_LOST |
| 单板普通 KF（CV/CA 对比） | `tracker/simple_target` | 装甲板心：CV 6 维 / CA 9 维 |

数据流：
`video_player(/image)` → `detector(/armors)` → `tracker_node(/tracker/target, /tracker/marker)`

当前 tracker 链路跑**整车 EKF**（`Target`）：把整车建模成 11 维状态（车心三轴位置/速度 +
朝向/转速 + 半径/长短轴 + 高低差），一次锁定目标车，车上 4 块装甲板统一由车体状态推导。
整车几何为**绕竖直轴（相机 y）在 x-z 水平面水平公转**（sp_vision 世界系"绕竖直轴转"在
相机系下的正确表达）

图像标注：
**白色框**：整车四装甲板建模
**绿色框**：当前追踪框
**紫色虚线框**：预测框

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
│   ├── config/
│   │   └── video_player.yaml     // 假相机参数（video_path 等，换视频入口）
│   └── launch/
│       ├── video_player.launch.py  // 终端 1
│       └── detector.launch.py      // 终端 2
├── tracker/                       // EKF 跟踪（整车小陀螺）
│   ├── include/  src/
│   │   ├── extended_kalman_filter // 通用 KF/EKF 框架（predict/update，任意观测维度）
│   │   ├── simple_target          // 普通 KF：CV 6 维 / CA 9 维（①② 历史实现，保留）
│   │   ├── target                 // 整车 EKF：11 维（③ 当前链路核心）
│   │   ├── tracker                // 四态状态机 + 整车数据关联门控（④）
│   │   ├── tracker_node           // /armors → /tracker/target（ROS 壳）
│   │   └── simple_tracker_node    // ①② 单板 KF 的 ROS 壳：/simple_tracker/*（可并行对比）
│   ├── config/
│   │   ├── tracker.yaml           // 整车门控/初值（N/半径）/阈值参数（调参入口）
│   │   └── simple_tracker.yaml    // 单板 KF：model(CV/CA)、v1、门控/超时
│   └── launch/
│       ├── tracker.launch.py      // 终端 3
│       └── simple_tracker.launch.py // 并行跑 ①②（model:=CA 换模型）
├── build/  install/  log/         // colcon 自动生成，不提交
└── ../video_input/                // 测试视频（仓库顶层，在 task3_KF 外一层）
```

## 3. 启动测试

> 路径约定：工作区放本机 `~/Workspaces/27-vision-dev/task3_KF`，视频在其上一层
> `video_input/`。换机器 / 换视频只需改 `detector/config/video_player.yaml` 里的
> `video_path` 一处（支持 `~/`），其余命令照抄。launch 文件本身不含路径。

### 0：一键脚本（起三节点，把源视频完整播一遍录成视频，播完自动停）

`scripts/start.sh` 自动 source 环境、起 video_player + detector + 一条追踪链路，把源视频
**从头到尾完整播一遍**的渲染录成 avi，播完自动停止全部节点——不再分手动/自动、不用自己
记时长或传秒数。日志写 `log/*.log`。默认录**整车 EKF**；要录**单板普通 KF**

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
./scripts/start.sh            # 整车 EKF → video_output/track_ekf.avi（播完自动停）
./scripts/start.sh kf         # 单板普通 KF（model 用 config 默认 CV）→ video_output/track_kf.avi
./scripts/start.sh kf CA      # 单板 KF 匀加速模型 → video_output/track_kf_CA.avi
```

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
ros2 launch detector detector.launch.py
# 打蓝车：detect_color:=1
```

### 终端 3：启动跟踪节点

#### 整车EKF

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tracker tracker.launch.py
```

#### 单板KF

`simple_tracker_node` 跑KF，话题独立（`/simple_tracker/*`），可与上面
整车 `tracker_node` 同时开，在同一份 `/image`、`/armors` 上直接 A/B 对比两种算法。

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tracker simple_tracker.launch.py            # model 默认读 config/simple_tracker.yaml（CV）
ros2 launch tracker simple_tracker.launch.py model:=CA  # 仅本次换 CA 模型重跑（需求② 对比）
# 查看：ros2 run rqt_image_view rqt_image_view /simple_tracker/final_img
#       ros2 topic echo /simple_tracker/target --once
```

### 终端 4：查看
```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /armor_detector/final_img # 检测标注图（2D）
# tracker 渲染图：白=整车 EKF 预测的全部板（Kalman 转盘，掉帧时即外推可视化）；
#   绿=被吃进融合的实测板 NN 角点框（加粗+距离文本）；品红虚线=需求⑤ 未来外推板
ros2 run rqt_image_view rqt_image_view /tracker/final_img
# 原始帧
ros2 run rqt_image_view rqt_image_view /image
# 看跟踪是否在锁：/tracker/target 里 tracking=true / predicted=true（外推帧）
ros2 topic echo /tracker/target --once
```

## 4. 话题与参数

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/image` | Image | video_player | 原始帧（SensorDataQoS，可丢帧） |
| `/armors` | Armors | armor_detector | 检测结果（每帧都发，0 板也发） |
| `/armor_detector/final_img` | Image | armor_detector | 标注图（debug=true） |
| `/armor_detector/marker_array` | MarkerArray | armor_detector | 检测装甲板（3D） |
| `/tracker/target` | Target | tracker_node | 整车状态输出：车心位置/速度/朝向/转速/半径/板数；TEMP_LOST 外推时也发，`predicted=true` |
| `/tracker/marker` | MarkerArray | tracker_node | 整车 3D：车心球 + N 块预测装甲板 CUBE（CUBE 板面带上倾 `plate_tilt_deg`°，法线上抬，贴合真实 RM 板） |
| `/tracker/final_img` | Image | tracker_node | 渲染：白=整车 EKF 预测**全部**板（Kalman 模型转盘，掉帧/漏检时即外推可视化）；绿=本帧被 EKF 吃掉的实测板 NN 角点框（加粗+距离文本）；品红虚线=需求⑤ 未来外推板（当前状态确定性外推 `future_ms` 后，瞄准提前量预览）；青色十字=车心投影；左上 HUD=整车状态量 |
| `/simple_tracker/target` | Target | simple_tracker_node | 单板普通 KF 输出：跟踪板心位置/速度（整车字段置 0）；无实测命中的掉帧帧 `predicted=true` |
| `/simple_tracker/final_img` | Image | simple_tracker_node | 单板 KF 渲染（与整车并行）：绿=实测命中板，紫=掉帧纯预测板框，青十字=KF 板心 |

`tracker/config/tracker.yaml`：`max_match_distance` + `max_match_yaw_diff`（整车数据关联的
位置/板朝向双阈值门控）、`armor_num`/`radius_init`（整车初始化板数/半径覆盖，red.avi 实测
标定 N=4、r≈0.31 m；置 0 则按车牌默认：前哨 3 板/r0.2765、其余 4 板/r0.2）、
`tracking_thres`（TRACKING 需连续帧数）、`lost_time_thres`（TEMP_LOST → LOST 掉帧超时 s，
默认 0.3 s）、相机内参与板绘制尺寸、`show_hud`（左上整车状态 HUD 开关，默认 true）。
需求⑤ 未来外推可视化：`future_ms`（外推提前量 ms，默认 150——画"模型认为未来会转到
哪"的品红虚线板框，瞄准提前量预览；只读外推不写进滤波/消息）、`show_future`（画未来板框
开关，默认 true）。
`plate_tilt_deg`（RM 装甲板**默认上倾角** deg，默认 15：板面朝上仰、顶边略后仰——实测前向板
法线 `n_y≈−sin(上倾角)`≈−0.24。只影响板朝向/marker CUBE 朝向/绘制的板倾，板心与相位不变，
EKF 核心不动；0 = 纯竖直板）。
运行中动态调：`ros2 param set /tracker_node <名> <值>`。

`tracker/config/simple_tracker.yaml`：`model`（CV 匀速 / CA 匀加速，需求② 对比实验主调参数）、
`v1`（过程噪声强度：CV=加速度方差、CA=加加速度方差）、`max_match_distance`（同号板距 KF
预测的选板门控）、`lost_time_thres`（连续无命中 → 回 LOST 重锁车号）、渲染内参与 `show_hud`。
也可不改 yaml、启动时临时换模型：`simple_tracker.launch.py model:=CA`。
