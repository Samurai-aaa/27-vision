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
整车几何为**绕竖直 y 轴在 x-z 水平面公转**。当前没有相机—云台—世界坐标变换，
EKF 直接在相机坐标系中运行，因此相机转动仍会被滤波器视为目标运动。

图像标注：
**白色框**：整车四装甲板建模
**绿色框**：当前追踪框
**紫色虚线框**：预测框

## 2. 项目目录结构

```text
task3_KF/
├── README.md
├── config/camera.yaml             // detector/tracker 共用的相机内参
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
记时长或传秒数。日志写 `log/*.log`。默认启动**整车 EKF，不录制**；加 `--record` 开启录制。

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
./scripts/start.sh            # 整车 EKF，不录制（播完自动停）
./scripts/start.sh kf --record # 单板普通 KF（model 用 config 默认 CV）→ video_output/track_kf_<视频>.avi
./scripts/start.sh kf CA --record # 单板 KF 匀加速模型 → video_output/track_kf_CA_<视频>.avi
./scripts/start.sh ekf --no-record   # 整车 EKF，只播放和追踪，不录制
./scripts/start.sh kf CV --no-record # 单板 CV，只播放和追踪，不录制
./scripts/start.sh ekf --record      # 显式开启录制
./scripts/start.sh --help            # 查看命令行选项
```

`--record` / `--no-record` 可放在链路参数前后。不录制时不会启动录制器或等待其订阅，
源视频播完后自动停止节点；配置 `loop: true` 时持续播放，按 Ctrl+C 停止。

输出名 = `track_<链路>[_<model>]_<源视频>.avi`，`<源视频>` 取自 `video_path` 的文件名
（`video_path: .../red.avi` → `track_ekf_red.avi`）。所以**同链路 + 同模型 + 同视频**
重跑会覆盖同一个文件（就是刷新结果），而**换模型（CV/CA）或换视频**各占一个文件、
互不覆盖——已经录好的结果不会被另一份配置的录制顶掉。

> ⚠️ 换了 `video_path` 要同步改 `detector/config/detector.yaml` 的 `detect_color`
> （0=打红车 1=打蓝车），修改后重启节点即可。设错的话
> detector 会把该颜色的板**整块过滤**：`/armors` 照常发布但一条不含板 → tracker 一直
> LOST，`/tracker/final_img` 只会显示 `LOST no lock`，不会出现跟踪框。

### 编译

```bash
cd ~/Workspaces/27-vision-dev/task3_KF
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 会让 `install/<包>/share/<包>/config/` 指向源码配置。首次这样编译后，
修改 `config/camera.yaml`、`detector/config/*.yaml` 或 `tracker/config/*.yaml` 只需重启对应
节点，不再需要编译。相机内参只维护 `config/camera.yaml` 这一处。

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
| `/image` | Image | video_player | 原始帧（**reliable，不丢帧**——见下方 QoS 说明） |
| `/armors` | Armors | armor_detector | 检测结果（每帧都发，0 板也发） |
| `/armor_detector/final_img` | Image | armor_detector | 标注图（debug=true） |
| `/armor_detector/marker_array` | MarkerArray | armor_detector | 检测装甲板（3D） |
| `/tracker/target` | Target | tracker_node | 整车状态输出：车心位置/速度/朝向/转速/半径/板数；TEMP_LOST 外推时也发，`predicted=true` |
| `/tracker/marker` | MarkerArray | tracker_node | 整车 3D：车心球 + N 块预测装甲板 CUBE（CUBE 板面带上倾 `plate_tilt_deg`°，法线上抬，贴合真实 RM 板） |
| `/tracker/final_img` | Image | tracker_node | 渲染：白=整车 EKF 预测**全部**板（Kalman 模型转盘，掉帧/漏检时即外推可视化）；绿=本帧被 EKF 吃掉的实测板 NN 角点框（加粗+距离文本）；品红虚线=需求⑤ 未来外推板（当前状态确定性外推 `future_ms` 后，瞄准提前量预览）；青色十字=车心投影；左上 HUD=整车状态量。未锁定时也发（原始帧 + `LOST no lock`，`publish_when_lost`），全程不断流 |
| `/simple_tracker/target` | Target | simple_tracker_node | 单板普通 KF 输出：跟踪板心位置/速度（整车字段置 0）；无实测命中的掉帧帧 `predicted=true` |
| `/simple_tracker/final_img` | Image | simple_tracker_node | 单板 KF 渲染（与整车并行）：绿=实测命中板，紫=掉帧纯预测板框，青十字=KF 板心。只在已锁上时发（`!st_` 直接返回），所以 blu.avi 这类多车稀疏视频里帧数会明显少于源视频 |

### QoS：为什么 `/image` 用 reliable

`/image` 单帧 1440×1080×3 = **4.6MB**。这条链上所有话题（`/image`、`/armors`、
`/tracker/final_img`、`/simple_tracker/final_img` 的发布与订阅）都用 **reliable**。

原因是实测出来的：用 best-effort 时，3301 帧的源视频只有 ~3020 帧能送到订阅者手里
（丢 8%），`/armors`、渲染、录像全按这个缩水的帧率走，**成片只有 101s，比源视频短 9 秒**。
当时逐项排查过：detector 单帧回调只占 9.7ms（不是算力）、订阅深度 5→60 无变化（不是缓存）、
内核 UDP 的 `RcvbufErrors` 全程为 0（不是丢包，4.6MB 走的是 Fast DDS 共享内存通道）——
真因是 best-effort 语义本身：大消息下读端只要瞬时落后，整帧就被静默丢弃。换 reliable 后
3301/3301 全收到，成片与源视频等长。

> ⚠️ **代价**：reliable 写端在历史写满时会阻塞 `publish`。所以**录制期间不要用 rqt 看
> `/image` 或 `/tracker/final_img`**——如果 rqt 跟不上，它的落后会反压拖慢整个播放（表现为
> 播放/录制变慢）。录制完再看不影响。`/armor_detector/final_img` 仍是 best-effort（纯调试
> 观看用途），看它不会波及算法链。

`tracker/config/tracker.yaml`：`max_match_distance` + `max_match_yaw_diff`（整车数据关联的
位置/板朝向双阈值门控）、`armor_num`/`radius_init`（整车初始化板数/半径覆盖，red.avi 实测
标定 N=4、r≈0.31 m；置 0 则按车牌默认：前哨 3 板/r0.2765、其余 4 板/r0.2）、
`tracking_thres`（DETECTING → TRACKING 需要的累计命中帧数）、**`max_miss_frames`**（漏检
容忍窗口：连续 N 帧无命中才回 LOST，默认 15。多车/远距离/弱光下 detector 会成片丢板，
逐帧判定会让画面在 LOST 上反复断流；设 1 即退回"一漏就回 LOST"）、`lost_time_thres`
（`/armors` 断流时的时间兜底，默认 1.5 s；流还活着时由 `max_miss_frames` 按帧判定）、
**`publish_when_lost`**（未锁定时也发原始帧 + `LOST no lock` 标注，默认 true，保证
`/tracker/final_img` 全程不断流；只影响出图，不伪造跟踪框）、相机内参与板绘制尺寸、
`show_hud`（左上整车状态 HUD 开关，默认 true）。
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


## 稳定性修正（2026-09-14）

相机坐标为 **x右、y下、z前**。EKF 视线观测使用 `yaw=atan2(x,z)`、
`pitch=atan2(y,hypot(x,z))`（向下为正），与车体板法线相位
`atan2(n_z,n_x)` 是两个不同的角度定义；整车仍在 x-z 平面绕竖直 y 轴转动。

整车按图像采集时间推进，渲染按同一时间戳取缓存图；重复/乱序观测丢弃。
回放时若时间戳倒退，请重启 tracker。断流后 timer 仅发布副本预测状态，
不把未来框叠在旧采集图上。正常空检测帧仍通过 TEMP_LOST 续跟并绘图。

新增启动参数：`tracker.yaml` 的 `max_reproj_error`（四角点误差和，默认 12 px）；
`detector.yaml` 的 `confidence_threshold`（0.35）和 `nms_threshold`（0.45）。
本轮参数均在节点启动时读取；使用 `colcon build --symlink-install` 安装后，修改源码 YAML
并重启对应节点即可生效。
几何范围、创新门限和噪声仍需实车视频标定。本轮修改清单见 CHANGELOG 顶部。

回归检查：编译后运行 `ctest --test-dir build/tracker --output-on-failure`。


## 第二轮稳定性修正

当前检测候选阈值为 0.35；整车以 ≥0.65 的可靠同号候选建轨，低分/颜色不确定/
模糊车号仅可在已确认轨迹附近有限续跟。主板连续缺失 3 帧才移交，其他板仍可更新整车。
Armor 消息已扩展，需重启整条链路并 source 新 install 环境。

task3 已统一使用系统 OpenCV 4.5.4，与 ROS cv_bridge 一致；不必卸载 /usr/local 的 4.7。
具体参数、改动对应问题、视频抽查结果和 OpenCV 重建方法见
[稳定性第二轮报告](docs/stability_report_2026-09-14.md)。
