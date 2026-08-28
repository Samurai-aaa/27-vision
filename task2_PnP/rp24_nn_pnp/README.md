# 部署深圳大学神经网络模型（RP24 + PnP）

## 项目结构

本项目继承 task1 的 `rp24_nn` 原有项目结构（OpenVINO 部署 RP24 装甲板检测网络），
在此基础上增加了 **PnP 位姿解算模块**，并对神经网络输出到装甲板的数据做了封装（`armor`）。

```
rp24_nn_pnp/
├── CMakeLists.txt            # CMake 构建（OpenCV + OpenVINO）
├── Model/                    # 模型文件 0526.onnx（相对路径，须在本目录下运行）
├── include/
│   ├── OpenvinoInfer.h       # 检测后端：Object 定义 + OpenvinoInfer 类
│   ├── armor.hpp             # Armor 结构 + Object→Armor 转换
│   └── solver.hpp            # Solver：6-DOF PnP 解算 + 重投影
├── src/
│   ├── main.cpp              # 视频循环、绘制、文本标注
│   ├── OpenvinoInfer.cpp     # OpenVINO 推理 + 后处理（过滤 / 独热 / NMS）
│   ├── armor.cpp             # armorFromObject：神经网络数据 → 装甲板
│   └── solver.cpp            # solvePnP / distance / yaw / projectPoint / drawZAxis
└── video_output/             # 标注视频输出目录
```

## 模块职责

| 模块 | 职责 |
| --- | --- |
| `OpenvinoInfer` | 封装 OpenVINO 推理（NHWC→RGB 归一化预处理）、后处理：置信度阈值、颜色/类别独热解码、NMS。结果写入 `tmp_objects`（`Object`：`rect` / `landmarks[8]` / `label` / `prob` / `color` / `length` / `width` / `ratio`） |
| `armor` | `armorFromObject(obj, sx, sy)`：把 640×640 图上的 landmarks 各向异性映射回原图、按 ratio 判定大小装甲板、透传 label/color/prob |
| `Solver` | 6-DOF PnP：`solvePose`（`SOLVEPNP_IPPE`，平面目标）、`distance`（tvec 模长 mm→m）、`yaw`（板面法线在相机 XZ 平面的偏角）、`projectPoint`、`drawZAxis` |

### 坐标与数据约定

- **landmarks 顺序**：`TL, BL, BR, TR`（左上逆时针），已实测确认与代码假设一致
- **armor 3D 模型点**（mm，z=0 平面）：小装甲板半宽 65 / 半高 27.5；大装甲板半宽 115 / 半高 27.5
- **相机内参**（1440×1080 真实标定值，无畸变）：
  ```
  fx=2556.2545862166521  fy=2553.5331992802749
  cx=705.83803766013978  cy=584.62889512335437
  ```
- **模型 color 输出**：`0=蓝 1=红`（`detect_color` 参数按此约定滤波）
- **NN 输入** 640×640，各向异性映射回原图（`sx=frame.cols/640, sy=frame.rows/640`）

## 输出

1. 利用神经网络识别装甲板，并在画面标注，格式：`装甲板类型+置信度`
   （label：G,1,2,3,4,5,O,Bs,Bb；红框=红车，蓝框=蓝车）
2. 装甲板的 **PnP 位姿**：黄色文本标注 `距离(m) + yaw(°)`，绿色线段为装甲板
   局部坐标系 `(0,0,0)→(0,0,50)`（mm）在图像上的**重投影**（z 轴方向）

## 运行指令

### 1. 环境准备

每次打开新终端先加载 OpenVINO 环境变量（含运行时库路径）：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
```

### 2. 编译

```bash
cmake -S . -B build
cmake --build build
```

产物在 `build/rp_detect`。> 修改源码后需重新 `cmake --build build`。

### 3. 运行

> 模型路径 `Model/0526.onnx` 是相对路径，**必须在 `rp24_nn_pnp/` 目录下运行**。

```bash
./build/rp_detect <输入视频路径> [detect_color] [输出视频路径]
```

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| 输入视频路径 | 必填，待检测的视频文件（测试视频统一在项目顶层 `../../video_input/`） | — |
| detect_color | 检测颜色：`0`=保留红(滤蓝)，`1`=保留蓝(滤红) | `0` |
| 输出视频路径 | 标注结果写入该 AVI 文件 | `<输入名>_rp_out.avi` |

示例：

```bash
# 检测红色（red.avi 为红色装甲板测试视频）
./build/rp_detect ../../video_input/red.avi 0 video_output/red_out.avi
# 检测蓝色
./build/rp_detect ../../video_input/blu.avi 1
```

## 已知问题与说明

- **`detect_color` 约定**：模型 color 输出 `0=蓝 1=红`，`detect_color=0` 保留红、`1` 保留蓝，
  与多数开源代码约定相反，已按实测（red.avi 装甲板输出 `1`）修正。
- **远距离小目标检测质量**：当装甲板很小很远（如 red.avi 中 ~3.5~3.87m 处，映射回原图仅 ~80px 宽），
  NN 回归的 4 角点会偏离标准装甲板比例。实测两目标 ratio 为 `0.64`（竖长条，疑似灯条误检）和 `1.58`
  （标准小装甲板应 ≈2.36）。几何失配时 `solvePnP`（IPPE）会解出**翻转退化姿态**
  （yaw 111°/-158° 无物理意义）。
- **缓解方向**：① 形状校验过滤 ratio 明显异常的检测；② 固定 roll=0/pitch=15° 做 4-DOF 约束解算；
  ③ EKF / 滑动平均时间平滑。
- **各向异性映射**：640×640 与 1440×1080 纵横比不同，landmarks 各向异性回映射会偏离针孔模型，
  叠加一定系统误差。
