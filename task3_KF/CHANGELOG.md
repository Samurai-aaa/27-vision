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
- 整车板建模补上 RM 装甲板默认**上倾 15°**（`plate_tilt_deg`，默认 15）：板绘制/marker CUBE
  板面绕宽轴转 −tilt 上仰（实测前向板法线 n_y≈−0.24≈−sin15° 佐证）；板心与相位不受，EKF 核心不动
- `/tracker/final_img` 品红未来外推虚线框线宽 1→2（更醒目）

### Changed 变更

- 首帧法线核对日志/注释从「前向板应 n_y≈0」改为「n_y≈−sin(plate_tilt_deg°)」——原假设被实测 n_y≈−0.24 推翻（板上倾所致，非纯竖直）


## [1.4.0] - 2026-09-13

### Feature 新增

- `detector/config/detector.yaml`：`detect_color`（敌方颜色 0=打红车 1=打蓝车）/`device`/`debug`
  从 launch 里的硬编码挪进 config；`detector.launch.py` 改读 yaml，CLI 仍可覆盖
  （`detect_color:=1 device:=GPU debug:=false`）。改完需 `colcon build --packages-select detector`
- 一键录制输出名带上源视频维度：`track_<链路>[_<model>]_<源视频>.avi`
  （新增 `video_full_secs.py --stem` 提供视频名）。同链路 + 同模型 + 同视频重跑即覆盖刷新；
  换模型（CV/CA）或换视频各占一个文件

### Changed 变更

- `start.sh` 输出名规则、头部注释与 README 同步更新

### Fixed 修复

- 录制产物被另一份配置的录制顶掉：此前输出名不含视频维度，换视频后一次失败录制会
  覆盖上一份视频的成品，且 `video_output/` 不入 git 无从恢复


## [1.5.0] - 2026-09-14
### 变更

- 采用强候选优先匹配
- 检测候选阈值 0.65→0.35；Armor 消息增加目标分数、类别分差与颜色不确定标记，未知颜色只作弱候选。
- 整车高质量建轨、强候选优先关联；弱候选使用半宽门控和 4 倍 R，最多 8 帧窗口，禁止弱观测初始化/转正及无限续命。
- 主板连续缺失 3 帧才换板；其他板命中仍算整车有效更新，等待期间显示原主板预测框。
- 新增弱候选、车号容错、主板换板回归与手动 ROS 测试。编译、算法、ROS、ldd 及反向版本配置检查通过。



## 2026-09-14 — 跟踪稳定性修正

### 变更

- 相机系保持 x右/y下/z前；视线角改为 yaw=atan2(x,z)、pitch=atan2(y,hypot(x,z))（向下为正），同步修正解析雅可比，避免光轴附近奇异。车体法线相位 atan2(n_z,n_x) 不变。
- 普通车几何初始方差 r/l/dz 改为 0.0004/0.0004/0.0001 m²，降低初始化速度不确定性。两组半径均检查 0.12–0.4 m，高差检查 ±0.15 m；异常更新回退到预测，不再单独夹断 r
- 同帧观测采用位置/朝向联合代价的贪心一对一关联，显式传模型板号给 EKF；同一观测/模型板不重复使用。
- EKF 使用更新前创新与协方差计算 NIS，以 LDLT 求解替代显式求逆；整车启用 18.47 创新门控，保留 Joseph 协方差更新；删除无真值的伪 NEES 计算。
- 断流超过 200 ms 后从 TRACKING/DETECTING 也可进入 TEMP_LOST；timer 使用状态副本发布预测，超时释放目标，不污染后续观测时间线。停流不在旧图上绘制未来框。
- PnP 用正深度和外法线与位置点积为负筛选，再取最小重投影误差；无合法解不输出。尚未加入双解的时序联合选择。
- 检测 NMS 改用与初筛一致的目标置信度；开放 confidence_threshold/nms_threshold，当前默认 0.35/0.45，修正 NMS 索引边界。

### 新增
- 增加 stability_test：相机角雅可比、光轴、创新拒绝、重复观测、遮挡恢复、倒退时间及 CV/CA 回归。
- 增加状态/协方差有限性、深度、速度、转速检查；tracker 消费 reproj_err 并拒绝超过 max_reproj_error（默认四点误差和 12 px）的观测，剩余观测按误差增大 R。
