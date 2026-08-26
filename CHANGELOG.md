# 更新日志

本仓库所有值得记录的变更都会记录在此文件中，格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [1.1.0] - 2026-08-26

### Changed 变更

+ 仓库重组为夏令营提交格式：由单层 `level3` 仓库迁移为 `27-vision-<name>/`
  + 原 `project1`（传统算法）+ 原 `project1_nn`（神经网络）合并入 `task1_armor_detection/`
  + 新增 `task2_xxx/`、`task3_xxx/` 占位目录，待任务确定后补充

### Added 新增

+ 顶层 `README.md`（任务清单 + 目录导航）与 `CHANGELOG.md`
+ `task1_armor_detection/` 下补充任务级 `README.md` / `CHANGELOG.md`
+ `rp24_nn/CMakeLists.txt`（原为 g++ 命令行编译）

## [1.0.0] - 2026-08-25

首次提交，仓库由两个独立技术栈的装甲板检测方案组成。

### Feature 新增

+ 新建 `project1`：基于传统 OpenCV 的装甲板识别
  + 灯条检测：灰度二值化 + 轮廓查找 + minAreaRect，按长宽比/角度筛选灯条
  + 颜色区分：按 BGR 通道和判断灯条颜色，支持红 / 蓝 / 红蓝都检（`detect_color`）
  + 装甲板配对：长度比 + 中心距（区分大/小装甲板）+ 角度 + 高度差四重判据
  + 置信度去重：按共享灯条去重，避免同一装甲板被重复检测
  + 全部参数集中在 `config/detector_params.txt`，命令行可覆盖视频路径
  + `--debug` 模式：额外显示二值化中间图 + 逐帧打印检测统计
+ 新建 `project1_nn`：部署深大 RP24 开源神经网络检测
  + 魔改 YOLOv5 + MobileNetV3 模型，输入 640×640 BGR
  + 输出 4 关键点（可直接喂 solvePnP）+ 颜色 + 数字识别（9 类 G/1-5/O/Bs/Bb）
  + OpenVINO 2024.6.0 CPU 推理
