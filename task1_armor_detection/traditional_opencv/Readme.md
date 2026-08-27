# traditional_opencv · 传统 OpenCV 装甲板检测

## 介绍

本项目只实现传统视觉识别灯条，不要求实现数字识别，pnp解算以及EKF追踪等功能。

基于传统 OpenCV 的装甲板识别，纯图像处理算法：

- **灯条检测**：灰度二值化 → 轮廓查找 → minAreaRect → 按长宽比/角度筛选出灯条
- **颜色区分**：按 BGR 通道和判断灯条颜色，支持 红 / 蓝 / 红蓝都检
- **装甲板配对**：两根灯条按 长度比 + 中心距（区分大/小装甲板）+ 角度 + 高度差 判据配对
- **置信度去重**：每帧多组配对按共享灯条去重，避免同一装甲板被重复检测

## 目录结构

```
traditional_opencv/
├── armor_detector/
│   ├── CMakeLists.txt
│   ├── include/armor_detector/
│   │   ├── armor.hpp          # Light / Armor 数据结构
│   │   └── detector.hpp       # Detector 类、参数结构体
│   └── src/
│       ├── main.cpp           # 入口：读参数、逐帧检测、显示
│       └── detector.cpp       # 检测实现（灯条筛选/配对/去重）
├── config/
│   └── detector_params.txt    # 全部可调参数（key=value）
├── video_output/              # 预留输出目录
└── Readme.md

> 测试视频统一放在 task1 共享目录 `../video_input/`（blu.avi / red.avi / 装甲板.avi，不入 git）
```

## 编译

依赖：OpenCV 4（`pkg-config --cflags --libs opencv4`）。CMake 构建：

```bash
cmake -S armor_detector -B armor_detector/build
cmake --build armor_detector/build
```

产物在 `armor_detector/build/armor_detector`。

## 运行

所有参数默认从 `config/detector_params.txt` 读取，命令行参数可覆盖。**视频路径默认取配置里的 `video_path`**。

程序会把**标注后的画面保存为 AVI 视频**，输出路径优先级：命令行第 3 个参数 > 配置里的 `video_output` > 自动在输入视频名后加 `_out.avi`。

```bash
# 1. 直接运行（视频/颜色/阈值等全部用 config 里的值，输出到 config 的 video_output）
./armor_detector/build/armor_detector

# 2. 指定视频（覆盖配置里的 video_path，输出自动为 video_input/xxx_out.avi）
./armor_detector/build/armor_detector video_input/red.avi
./armor_detector/build/armor_detector video_input/blu.avi
./armor_detector/build/armor_detector video_input/装甲板.avi

# 3. 指定视频 + 指定配置文件
./armor_detector/build/armor_detector video_input/red.avi config/detector_params.txt

# 4. 指定视频 + 配置文件 + 输出路径（任意位置，需先确保目录存在）
./armor_detector/build/armor_detector video_input/red.avi config/detector_params.txt video_output/red_out.avi
```

窗口打开后按 **q** 退出。

## Debug 模式

加 `--debug` 参数（位置随意），会额外打开一张**二值化中间图窗口**，并在终端**逐帧打印**检测统计：

```bash
./armor_detector/build/armor_detector --debug
./armor_detector/build/armor_detector video_input/red.avi --debug
```

终端输出示例：

```
frame 3: lights=2 armors=1 [SMALL conf=0.78]
frame 4: lights=2 armors=1 [SMALL conf=0.78]
```

`lights` 为检测到的灯条数，`armors` 为去重后装甲板数，`SMALL/BIG` 为大小装甲板，`conf` 为置信度（0~1）。用 debug 观察配对情况时，可一边跑一边改 `config/detector_params.txt` 里的阈值（保存后重开程序生效）。

## 参数配置

参数、视频路径等配置在 `config/detector_params.txt`：

| 参数 | 默认 | 含义 |
|---|---|---|
| `video_path` | `video_input/装甲板.avi` | 测试视频路径（相对项目根目录） |
| `detect_color` | `2` | 检测颜色：`0`=红 `1`=蓝 `2`=红蓝都检 |
| `binary_thres` | `120` | 灰度二值化阈值（0~255），越小越易把暗色误判为灯条 |
| `light_min_ratio` / `light_max_ratio` | `0.1` / `0.4` | 灯条短边/长边比例范围，细长灯条应远小于 1 |
| `light_max_angle` | `40.0` | 灯条与竖直方向最大夹角（度） |
| `armor_min_light_ratio` | `0.7` | 配对两灯条长度比（短/长）下限 |
| `armor_min_small_center_distance` / `armor_max_small_center_distance` | `0.5` / `2.5` | 小装甲板灯条中心距（以平均灯长为单位）区间 |
| `armor_min_large_center_distance` / `armor_max_large_center_distance` | `2.5` / `3.5` | 大装甲板中心距区间 |
| `armor_max_angle` | `10.0` | 两灯条中心连线与水平方向最大夹角（度） |
| `armor_max_center_height_diff` | `0.5` | 两灯条中心高度差上限（以平均灯长为单位），太大直接取消配对 |
| `video_output` | 空（自动加 `_out.avi`） | 标注视频输出路径（相对运行时目录），空则自动生成，命令行第 3 个参数可覆盖 |
