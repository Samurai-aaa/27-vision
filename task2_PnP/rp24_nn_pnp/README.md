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
│   ├── solver.hpp            # Solver：6-DOF PnP 解算 + 重投影
│   └── yawstudy.hpp          # yaw 一维优化研究模块（yawstudy::Sample/代价/优化器）
├── src/
│   ├── main.cpp              # 视频循环、绘制、文本标注（rp_detect）
│   ├── OpenvinoInfer.cpp     # OpenVINO 推理 + 后处理（过滤 / 独热 / NMS）
│   ├── armor.cpp             # armorFromObject：神经网络数据 → 装甲板
│   ├── solver.cpp            # solvePnP / distance / yaw / projectPoint / drawZAxis
│   ├── yaw_study_main.cpp    # rp_yaw_study 主流程（视频采集 + A/B/C 研究编排）
│   └── yawstudy.cpp          # yawstudy 模块实现（算法 + OpenCV 绘图）
├── video_output/             # 标注视频输出目录
└── yaw_study_out/            # rp_yaw_study 图输出目录（运行后生成）
```

## 模块职责

| 模块 | 职责 |
| --- | --- |
| `OpenvinoInfer` | 封装 OpenVINO 推理（NHWC→RGB 归一化预处理）、后处理：置信度阈值、颜色/类别独热解码、NMS。结果写入 `tmp_objects`（`Object`：`rect` / `landmarks[8]` / `label` / `prob` / `color` / `length` / `width` / `ratio`） |
| `armor` | `armorFromObject(obj, sx, sy)`：把 640×640 图上的 landmarks 各向异性映射回原图、按 ratio 判定大小装甲板、透传 label/color/prob |
| `Solver` | 6-DOF PnP 7 种解算方式，统一走 `solvePose` 分派：0=双解（`solvePnPGeneric`-IPPE，IPPE 两解中选优）、1=单解（`solvePnP`-IPPE）、2=SQPNP、3=EPNP、4=ITERATIVE、5=P3P、6=AP3P（1~6 走 `solveSingle` 单解）；另提供 `distance`（tvec 模长 mm→m）、`yaw`（板面法线在相机 XZ 平面的偏角）、`projectPoint`、`drawZAxis`。`solvePnPGeneric` 会把未选中的另一解写进 `Armor.rvec_alt/tvec_alt`，供上层用另一颜色画出来对照。解出成功统一收口到 `postProcessYaw`：记录 `yaw_raw`，做 **yaw 重投影校验/绕竖轴遍历精修**（见下方小节） |
| `yawstudy`（rp_yaw_study） | yaw 一维优化**研究工具**的算法层（不参与 `rp_detect`）：以一块已解算的板为一个 `Sample`，提供"固定 xyz、绕相机竖轴旋转 Δ"的重投影代价（`costSq/costL1`）、代价曲线扫描/局部极小、一维优化器集合（`opt1d`：grid/golden/brent/lm/prod）、以及曲线画图。见下方"yaw 一维优化研究"一节 |

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
3. 双解模式（`pnp_method=0`）把 IPPE **双解都画出来**：绿轴 = 选中的解
   （板面朝相机 + 重投影误差最小），**橙轴 = 未选中的另一解**（镜像假设）——
   平面目标前后两解投影到几乎同一组角点，只有 z 轴方向能区分，橙色用于对照双解歧义；
   其余模式（1~6）为各内核 `solvePnP` 单解，只画绿轴，无橙轴
4. 画面左上角 info 行下方有**黄字标题标注当前方法 tag**（如 `SQPNP`/`EPNP`/
   `ITERATIVE`/`P3P`/`AP3P`/`IPPE 单解`/`IPPE 双解`），录制后看视频即知用的哪个方法
5. 解算后对每块做 **yaw 重投影校验**：误差超出阈值才触发绕竖轴遍历择优（不改变正常
   帧的 yaw，见下节）；正常帧行为与 1.1.0 一致，多画/少画无变化

## yaw 重投影校验与绕竖轴精修

PnP 解出的 pose 只能保证 4 个 3D 角点投影到检测角点，yaw 本身是否可靠需要二次校验。
对每块成功解出的装甲板，`postProcessYaw` 做：

1. **校验**：把当前姿态的 3D 板 4 角点重投影回图像，与 NN 检测 4 角点逐点算 L2 像素
   距离之和 `err0`（写 `Armor.yaw_refine_err0`）。误差和 ≤ 阈值（默认 6px）→ 认为 yaw
   解算贴合图像，不动，直接返回。
2. **遍历择优**（误差过大时）：把整块板绕**相机竖轴（y）**在解算 yaw 附近
   `±search_range_deg`（默认 15°）按 `search_step_deg`（默认 0.5°）逐度重投影
   （`R ← Ry(Δ)·R`，板心 `tvec` 不动），取误差和最小者。**只有严格减小**才更新
   `rvec/yaw`（`yaw_refined=true`），否则保持原解。
3. **留档对照**：PnP 原始 yaw 存 `Armor.yaw_raw`，精修前后误差存 `err0/err1`，末尾
   统计打印触发率与误差变化。

配置走 `YawRefineConfig`（见 `include/solver.hpp`），在 `Solver::setYawRefineConfig`
可整体开关或调参；默认 `enable=true, thresh_px=6, search_range_deg=15, search_step_deg=0.5`。

**验证（red.avi，双解 IPPE，4707 块 PnP 全成功）**：全部块重投影误差和均值 1.53px、
p90 2.34、p99 2.90、max 3.79px，均低于 6px 阈值 → 精修 0 触发，即 IPPE 的 yaw 对图像
已贴合、无需扫描。该兜底主要用于初始姿态投影本身偏大的情况（如 EPNP 对共面点退化，
同数据下 err0 均值约 67px、触发率 ~94%，经绕竖轴扫描可降一部分）；IPPE 类内核几乎不会
触发。注意：镜像双解投影到几乎同一组角点，纯角点重投影误差分辨不了"选错镜像解"。

## yaw 一维优化研究（rp_yaw_study）

研究需求④：**在 PnP 基础上，固定双解筛选后得到的 xyz，把 yaw 作为唯一优化变量，
构造重投影误差最小化问题**——考察代价曲线形状、IPPE 双解能否仅靠代价分辨、按曲线
特点选优化方法、以及近正对时优化结果是否在正负 yaw 间跳变。全部 C++ 自实现 +
OpenCV 画图，不依赖 python。

### 坐标与参数化（关键前提）

- 板模型 **+z 朝外**：板正对相机时法线指向相机(−z)，`R22<0`、裸 `yaw≈±180°`。
  判断"板是否朝前/正对"要看 **离正对转角 `ψ = wrapDeg(atan2(nx, −nz))`**
  （正对≈0），而不是裸 yaw（`yawstudy::headPsiDeg`）。
- 一维参数化：`R ← Ry(Δ)·R_sel`（绕**相机竖轴 y** 旋转），板心 `tvec` 与高度不动；
  **Δ=0 即 IPPE 选中姿态**。代价 `costSq=Σ(dx²+dy²)`(px²) 与 `costL1=Σ‖角点差‖`(px)
  都以 Δ(度) 为自变量。

### 模块拆分

| 文件 | 内容 |
| --- | --- |
| [include/yawstudy.hpp](include/yawstudy.hpp) | 模块接口：`Sample`（一块板 + 双解姿态）、角度工具、`costSq/costL1`、`Curve`/`scanCurve`/`localMinima`、`opt1d` 优化器、`saveCurveFigure` |
| [src/yawstudy.cpp](src/yawstudy.cpp) | 上述实现（手写投影/优化器/OpenCV 绘图），不依赖 OpenVINO/Solver，可脱离工具复用 |
| [src/yaw_study_main.cpp](src/yaw_study_main.cpp) | 主流程：视频采集（OpenVINO 检测 + DUAL_IPPE）、选代表样本、跑 A/B/C 三组研究并出图 |

### 运行与产物

```bash
cmake --build build                # 生成 build/rp_yaw_study
./build/rp_yaw_study ../../video_input/red.avi 0 yaw_study_out
#        <视频>  <detect_color=0> <输出目录> [max_frames=0=全部]
```

运行约 25s，在输出目录生成并打印：

- `curve_rep{i}_psi….png` —— **区域 A**：5 个代表样本（离正对 ~0/15/30/50/63°）的完整
  代价曲线三面板图（Σ² log / ΣL1 log / 近 Δ=0 线性），黑竖线=局部极小，绿点=选中解、
  橙点=另一候选在固定 t 下的代价；
- `bench.png` + 终端表格 —— **区域 B**：5 种方法逐块耗时/精度对比；
- `near0_refine_psi.png` + 统计 —— **区域 C**：近正对板的精修是否跨 0 跳变。

### 结论（red.avi 全量，4707 块、全双解）

1. **代价曲线形状**：完整 ±180° 上"单深谷 + 至多一个很浅的远谷"，深谷恒 1 个——
   接近单峰但**非严格单峰**；近正对时两候选谷间距 <1°、代价近退化（0.19 vs 0.27 px²）。
2. **双解能否只靠代价分辨**：**不能**。镜像歧义活在 6-DOF 完整姿态空间（两解各自投影
   都 <2px），但在**固定 t 的 yaw-only 切片上镜像候选落在 10~221 px² 的高坡**，选不中；
   该曲线只会把选中支选为全局极小，天然"挡住"镜像，不会帮你在两候选间选真值。
3. **方法选型**：golden 因假设单峰而**锁错远谷**（平均偏 29.7°、RMSE 1.85px，不可用）；
   grid0.5 / brent / LM(from0) / coarse1°+Brent 都收敛到同一谷（相对 0.2° 真值差 0.016°）。
   每块耗时：grid0.2 0.06ms / grid0.5 0.024 / brent 0.0017 / **LM(from0) 0.0007** /
   **coarse1°+Brent 0.013** ms。曲线单深谷 + 起点已在谷 ⇒ **生产建议 coarse1°+Brent**
   （粗扫全局稳健）或 LM 从 PnP 起点起步（2 次求值即可）。
4. **近正对跳变**：717 块 |ψ|≤8 的板，精修平均 |Δ|=0.06°、跨 0 仅 1%、大跳镜像 0%——
   因为近正对时双候选本来就都贴正对（谷间距 <1°），换谷也只造成 <1° 抖动，**不会翻到
   "背对"那支**。即 yaw 一维精修不会放大 IPPE 双解歧义。

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

产物在 `build/rp_detect`（部署主程序）与 `build/rp_yaw_study`（yaw 一维研究小工具，见
上节）。> 修改源码后需重新 `cmake --build build`。

### 3. 运行

> 模型路径 `Model/0526.onnx` 是相对路径，**必须在 `rp24_nn_pnp/` 目录下运行**。

```bash
./build/rp_detect <输入视频路径> [detect_color] [输出视频路径] [pnp_method]
```

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| 输入视频路径 | 必填，待检测的视频文件（测试视频统一在项目顶层 `../../video_input/`） | — |
| detect_color | 检测颜色：`0`=保留红(滤蓝)，`1`=保留蓝(滤红) | `0` |
| 输出视频路径 | 标注结果写入该 AVI 文件 | `<输入名>_rp_out.avi` |
| pnp_method | PnP 解算方式：`0`=双解（solvePnPGeneric-IPPE，绿=选中解，橙=未选中另一解）；`1`=单解（solvePnP-IPPE）；`2`=SQPNP；`3`=EPNP；`4`=ITERATIVE；`5`=P3P；`6`=AP3P（1~6 为 `solvePnP` 单解） | `0` |

示例：

```bash
# 检测红色（默认双解：绿=选中解，橙=另一解）
./build/rp_detect ../../video_input/red.avi 0 video_output/red_dual.avi 0
# 检测红色（单解 IPPE）
./build/rp_detect ../../video_input/red.avi 0 video_output/red_single.avi 1
# 换内核：SQPNP / EPNP / ITERATIVE / P3P / AP3P（终端第 4 个参数选择）
./build/rp_detect ../../video_input/red.avi 0 video_output/red_sqpn.avi 2
./build/rp_detect ../../video_input/red.avi 0 video_output/red_epnp.avi 3
./build/rp_detect ../../video_input/red.avi 0 video_output/red_iter.avi 4
./build/rp_detect ../../video_input/red.avi 0 video_output/red_p3p.avi  5
./build/rp_detect ../../video_input/red.avi 0 video_output/red_ap3p.avi 6
# 检测蓝色
./build/rp_detect ../../video_input/blu.avi 1
```
