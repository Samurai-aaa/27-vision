# 部署深圳大学神经网络模型

## 项目结构
本项目主要由开源神经网络构成，我添加了main函数，并且对源码进行小幅改动（ai检查得出）：
OpenvinoInfer.h — 修复 OpenvinoInfer(string model_path, string device) 单文件构造的预处理配置。

| 原项目 | 现在的改动 |
| --- | --- |
| 输入 shape | `{1,1,640,640}`（与模型不符） | 按模型真实输入 `[1,3,640,640]`，user tensor 为 `{1,640,640,3}` u8 NHWC BGR |
| 归一化 | 无（缺 scale） | `convert_element_type(f32)` → `convert_color(RGB)` → `scale(1/255)` |

- 原因：0526/0708 模型真实输入是 `[1,3,640,640]`，原配置把输入数据解读错误且缺归一化，实测会导致检测幻觉（对纯噪声图"检出"1.5 万个目标）。修复方式就是复用原项目多文件构造里已经写好的正确预处理（那段代码本来就是对的，只是单文件构造没复用）。

## 输出
除了利用神经网络识别灯条和装甲板之外，还通过增加曝光度，在黑暗环境下识别出装甲板数字，并在画面标注，格式为：装甲板类型+置信度（label： G,1,2,3,4,5,O,Bs,Bb）

## 运行指令

### 1. 环境准备

每次打开新终端先加载 OpenVINO 环境变量（含运行时库路径）：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
```

### 2. 编译

本目录提供 `CMakeLists.txt`（推荐），也保留了原来的 g++ 直接编译方式。**以下命令都在仓库根目录执行**，先 `cd` 进本模块并加载 OpenVINO 环境：

```bash
cd task1_armor_detection/rp24_nn                # 后续命令均在本目录下运行
source /opt/intel/openvino_2024.6.0/setupvars.sh   # 换新终端需重新 source
```

**方式一：CMake**

```bash
cmake -S . -B build
cmake --build build -j
```

产物在 `build/rp_detect`。

**方式二：g++（原始方式）**

```bash
g++ -o rp_detect main.cpp OpenvinoInfer.cpp \
    -I/opt/intel/openvino_2024.6.0/runtime/include \
    -L/opt/intel/openvino_2024.6.0/runtime/lib/intel64 \
    -lopenvino $(pkg-config --cflags --libs opencv4)
```

产物在**当前目录** `./rp_detect`（方式一产物在 `build/`，两者路径不同，别混用）。

> 注意：`libopenvino.so` 位于 `runtime/lib/intel64/` 子目录，`-L` 路径必须带 `intel64`，否则链接报 `找不到 -lopenvino`。修改置信度等参数后需重新编译。

### 3. 运行

```bash
./build/rp_detect <输入视频路径> [detect_color] [输出视频路径]
```

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| 输入视频路径 | 必填，待检测的视频文件（测试视频统一在项目顶层 `../../video_input/`） | — |
| detect_color | 检测颜色：`0`=保留红(滤蓝)，`1`=保留蓝(滤红) | `0` |
| 输出视频路径 | 标注结果写入该 AVI 文件 | `<输入名>_rp_out.avi`（与输入同目录） |

> **必须在 `task1_armor_detection/rp24_nn` 目录下运行**：模型路径在代码里写死为 `Model/0526.onnx`，按**当前工作目录**解析，换目录执行会报找不到模型。
> 有 `DISPLAY` 时弹窗显示，按 **ESC** 退出；无显示环境自动只写文件。

示例：

```bash
cd task1_armor_detection/rp24_nn     # 模型路径依赖 CWD，须在本模块目录下运行

# 检测红色（red.avi 为红色装甲板测试视频，位于项目顶层 ../../video_input/）
./build/rp_detect ../../video_input/red.avi 0 video_output/red_out.avi
# 检测蓝色（blu.avi 为蓝色装甲板测试视频）
./build/rp_detect ../../video_input/blu.avi 1

# 不指定输出路径：自动写到输入视频同目录 ../../video_input/<输入名>_rp_out.avi
./build/rp_detect ../../video_input/装甲板.avi 0
```