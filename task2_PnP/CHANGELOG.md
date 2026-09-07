# 更新日志

## [1.0.0] - 2026-08-27

### Feature 新增

- 新建PnP

### Changed 变更

### Fixed 修复


## [1.0.1] - 2026-08-28

### Feature 新增

- 新增armor.hpp用于封装甲板信息

- 新增armor.cpp用于从神经网络获取装甲板数据并进行转换处理

### Changed 变更

### Fixed 修复


## [1.0.1] - 2026-08-30

### Feature 新增

- 新增solvePnPGeneric功能，并可通过输入指令选择两种模式

### Changed 变更

### Fixed 修复


## [1.0.2] - 2026-09-07

### Feature 新增

- solvePnPGeneric 把 IPPE **双解都画出来对照**：未选中的另一解（镜像假设）写进
  `Armor.rvec_alt/tvec_alt`，main 用橙色画另一解 z 轴（绿轴=选中解，橙轴=另一解）；
  单解路径两者置空，不影响原行为

### Changed 变更

- `Solver::drawZAxis` 拆成"按 Armor 画"与"按任意 rvec/tvec+颜色画"两个重载，
  供画另一解复用同一套投影

### Fixed 修复


## [1.1.0] - 2026-09-07

### Feature 新增

- 支持**多种 OpenCV 内置 PnP**，终端 `pnp_method` 选择（顺序即索引）：
  `0`=双解（`solvePnPGeneric`-IPPE，绿=选中解，橙=未选中另一解）→ 默认
  `1`=单解（`solvePnP`-IPPE）→ `2`=SQPNP → `3`=EPNP → `4`=ITERATIVE →
  `5`=P3P → `6`=AP3P；越界参数报错退出
- 统一 `Solver::solvePose` 按方法分派：0 走 `solvePnPGeneric`，1~6 走新增的
  `solveSingle(Armor&, int flag)`（校验/清另一解/solvePnP/填指标共用）

### Changed 变更

- **pnp_method 编号重排**：原 0=单解/1=双解 → 现 0=双解/1=单解（与原 0/1 相反，
  旧调用需对调）。原因：把「默认 0」留给最稳的双解模式
- 结束统计新增 `PnP 成功 n (占检出%)`，各内核横向对照可量化
- 画面**左上角第二行标注当前 PnP 方法 tag**（录制后看视频即可知用的哪个方法，
  黄字粗体在 info 行下方）；随后录制 SQPNP/EPNP/ITERATIVE/P3P/AP3P 五个完整
  对比视频到 `video_output/red_*.avi`（双解/单解两个 IPPE 沿用既有产物，未重录）

### Fixed 修复


## [1.2.0] - 2026-09-07

### Feature 新增

- 解算后新增 **yaw 重投影校验 + 绕竖轴遍历精修**（参照 sp_vision `optimize_yaw`）：
  把当前姿态下装甲板 3D 模型 4 角点重投影回图像，与 NN 检测 4 角点逐点算 L2 像素
  距离之和，作为「yaw 解算是否准确」的量化指标（写 `Armor.yaw_refine_err0`）
- 原 yaw 保留到 `Armor.yaw_raw` 供对照；是否触发/更新记 `yaw_refined`，精修后误差写
  `yaw_refine_err1`（未触发/无改善时 == err0）

### Changed 变更

### Fixed 修复

- 单解路径（1~6）在解出前先把 `rvec_alt/tvec_alt` 置空，避免对象复用残留上一次的
  橙色另一解