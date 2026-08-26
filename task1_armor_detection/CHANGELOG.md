# 更新日志

本目录所有值得记录的变更都会记录在此文件中，格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [1.1.0] - 2026-08-26

### Changed 变更

+ 项目结构重组：由 `level3` 顶层仓库迁移至 `27-vision-<name>/task1_armor_detection/`
  + `traditional_opencv/`：原 `project1`（传统 OpenCV 装甲板检测）
  + `rp24_nn/`：原 `project1_nn`（RP24 神经网络部署）
  + 两个子方案合并为任务1下的并列模块，各自独立编译

### Added 新增

+ `rp24_nn/CMakeLists.txt`：将原来的 g++ 命令行编译方式改为 CMake（`find_package(OpenVINO)`）

## [1.0.0] - 2026-08-25

首次实现，仓库由两个独立技术栈的装甲板检测方案组成。

### Feature 新增

+ `traditional_opencv`（原 project1）：
  + 灯条检测：灰度二值化 + 轮廓查找 + minAreaRect，按长宽比/角度筛选灯条
  + 颜色区分：按 BGR 通道和判断灯条颜色，支持红 / 蓝 / 红蓝都检（`detect_color`）
  + 装甲板配对：长度比 + 中心距（区分大/小装甲板）+ 角度 + 高度差四重判据
  + 置信度去重：按共享灯条去重，避免同一装甲板被重复检测
  + 全部参数集中在 `config/detector_params.txt`，命令行可覆盖视频路径
  + `--debug` 模式：额外显示二值化中间图 + 逐帧打印检测统计
+ `rp24_nn`（原 project1_nn）：
  + 部署深大 RP24 开源神经网络（魔改 YOLOv5 + MobileNetV3）
  + 输入 640×640 BGR，输出 4 关键点 + 颜色 + 数字（9 类 G/1-5/O/Bs/Bb）
  + OpenVINO 2024.6.0 CPU 推理
