# 更新日志

## [1.0.0] - 2026-08-31

### Feature 新增

- interfaces消息包

### Changed 变更

- 继承task2,并将detector改写成ros2通信形式

### Fixed 修复


## [1.0.0] - 2026-09-01

### Feature 新增

- video_player_node节点，成功改写并跑通

### Changed 变更

### Fixed 修复


## [1.0.0] - 2026-09-03

### Feature 新增

- 完成tracker节点并跑通

### Changed 变更

### Fixed 修复


## [1.0.0] - 2026-09-03

### Feature 新增

### Changed 变更

- 增加了launch,优化了终端启动

### Fixed 修复


## [1.0.0] - 2026-09-05

### Feature 新增

- 新增一些脚本文件，方便快捷开启检测
- 录制EF算法cv、ca运动模型的检测视频

### Changed 变更

### Fixed 修复


## [1.0.0] - 2026-09-05

### Feature 新增

- tracker_node 接入整车 EKF（Target，11 维），单车 4 块装甲板统一建模
- record_ekf.sh：一键录制整车渲染视频（track_ekf.avi）

### Changed 变更

- Tracker 由单板普通 KF（SimpleTarget）切换为整车 EKF，双阈值门控（位置+板朝向）匹配
- tracker_node 整车字段填充（车心位置/速度/朝向/转速/半径/板数）+ 车心球 & N 板 marker
- config/tracker.yaml 换整车参数

### Fixed 修复


## [1.0.0] - 2026-09-05

### Feature 新增

- **整车几何重构为"绕竖直轴水平公转"**（sp_vision 世界系水平面在相机系下的正确表达）：
  板心 `车心+r·(cosφ,0,sinφ)`、公转在相机 x-z 水平面，板面竖直、法线径向朝外；板朝向
  观测口径改 `atan2(n_z,n_x)`（法线在 x-z 水平面的方位角，前向板 sinφ<0）。
  修复：掉帧橙框原本"继续向右直线飞"——旧模型把公转轨道放在像平面 x-y、yaw 双稳态不可观；
  现随整车转到侧向投影收窄、翻过车背消失，下一块另一侧露出（绕背）

### Changed 变更

- target 整车模型：同帧全部双阈值命中板**逐块喂 EKF 融合**（sp_vision 同款），不再只喂最优单板
- R 自适应按**掠射角**（板法线与视线夹角）调板朝向噪声，距离噪声单独随距离（log 模型）
- /tracker/marker 板 CUBE 朝向按竖直板面重建 `R=[t|u|n̂]`（宽向水平切向/高向竖直/法线径向朝外）

### Fixed 修复

- 修正 `atan2(n_y,n_x)` 双稳态不可观问题导致的整车 yaw 无法收敛 / 外推框直线右飞


## [1.1.0] - 2026-09-06

### Feature 新增

- simple_tracker_node 单板普通 KF
- 目标锁定口径与整车一致：锁定"车号"，其后只喂同号板、忽略其它号；无同号命中即纯外推
- `config/simple_tracker.yaml`（model/v1/门控/超时/内参）+ `simple_tracker.launch.py`
  （支持 `model:=CV|CA` 临时换模型）；渲染：绿=实测命中板，紫=掉帧纯预测板框，青十字=KF 板心


## [1.2.0] - 2026-09-06

### Changed 变更

- 一键脚本改为"把源视频完整播一遍录成 avi、播完自动停"
- 两追踪节点左上 HUD 字体加大

### Fixed 修复


## [1.3.0] - 2026-09-07

### Feature 新增

- 整车 EKF 未来外推可视化：渲染叠第三层**品红虚线板框**=
  `future_ms`（默认 150ms）后"模型认为车会转到哪"（瞄准提前量预览），白=现在模型转盘、
  绿=实测板，三层同画对照
- `config/tracker.yaml` 新增 `future_ms`（提前量 ms，默认 150.0）/`show_future`（开关）

### Changed 变更