# task1_armor_detection · 装甲板检测

RoboMaster 装甲板检测，包含两个子方案，实现同一套检测目标：

- **检测灯条及其角点**
- **区分装甲板颜色**（红 / 蓝）
- **区分不同装甲板**（灯条不错误匹配）
- `rp24_nn/` 额外**识别装甲板数字**（G/1-5/O/Bs/Bb）

## 子方案

| 子目录 | 方案 | 说明 |
|---|---|---|
| [traditional_opencv/](traditional_opencv/) | 传统 OpenCV 图像处理 | 灰度二值化 → 轮廓 → minAreaRect 提取灯条，按长宽比/角度/颜色筛选，双灯条四重判据配对，共享灯条去重 |
| [rp24_nn/](rp24_nn/) | 深大 RP24 开源神经网络 | 魔改 YOLOv5 + MobileNetV3，OpenVINO CPU 推理，输出 4 关键点 + 颜色 + 数字 |

## 测试视频与运行效果

- **测试视频**：统一放在 `video_input/`（blu.avi / red.avi / 装甲板.avi，不入 git）。两个子方案运行时统一用 `../video_input/` 访问
- **运行效果视频**：
  - 传统算法：`traditional_opencv/video_output/`
  - 神经网络：`rp24_nn/video_output/`

## 编译运行

环境与步骤请直接看子方案各自的 README（编译命令、参数含义、Debug 模式都在里面）：

- 传统算法编译运行：`traditional_opencv/Readme.md`
- 神经网络编译运行：`rp24_nn/README.md`

## 变更记录

见本目录 `CHANGELOG.md`。
