#ifndef YAWSTUDY_HPP
#define YAWSTUDY_HPP

// yawstudy：yaw 一维优化研究模块（rp_yaw_study 小工具的核心算法层）。
//
// 研究问题（对应需求④）：在 PnP(双解 IPPE)基础上固定 xyz，把 yaw 作为唯一优化
// 变量，构造"重投影误差最小化"一维问题，研究代价曲线形状/双解歧义/方法选型/
// 近正对跳变。本模块与视频采集/OpenVINO/Solver 无关，只操作"一块板"的数学模型，
// 因此可脱离 rp_yaw_study 单独复用。
//
// 一维参数化：姿态只绕相机竖轴 y 额外旋转 Δ（R←Ry(Δ)·R_sel，板心 tvec 与高度
// 不变）。Δ=0 即 IPPE 选中的姿态。costSq/costL1 都以 Δ(度) 为自变量。
//
// 坐标约定：模型 +z 朝外，板正对相机时法线指向相机(-z_cam)，R22<0、裸 yaw≈±180。
// 所以"板是否朝前/正对"要看 psi = wrapDeg(atan2(nx,-nz))（离正对转角，正对≈0），
// 而不是裸 yaw。

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "armor.hpp"   // task2::Armor / ArmorType

namespace task2 {
namespace yawstudy {

// ---- 相机内参（与 solver.cpp 默认一致：1440x1080 无畸变真实标定值） ----
inline constexpr double kCamFx = 2556.2545862166521;
inline constexpr double kCamFy = 2553.5331992802749;
inline constexpr double kCamCx = 705.83803766013978;
inline constexpr double kCamCy = 584.62889512335437;

// 一块装甲板的研究样本：检测角点 + 双解 IPPE 的选中解/另一解 + 板 3D 模型。
struct Sample {
    double hw = 0, hh = 0;     // 模型半宽/半高（mm）
    double mx[4], my[4];       // 模型角点板坐标（mm）TL,BL,BR,TR（z=0 平面）
    double qx[4], qy[4];       // NN 检测角点（原图像素），与模型角点同顺序
    double r0[9];              // 选中解旋转（行主序）
    double t0[3];              // 选中解平移（mm）
    double rA[9];              // 另一解（镜像假设）旋转；hasAlt=false 时无效
    double tA[3];              // 另一解平移
    bool hasAlt = false;       // 是否有未选中的另一解（solvePnPGeneric 双解）
    double yaw0 = 0, yawA = 0; // 法线在相机 XZ 平面方位角（度，模型 +z 朝外）
    double psi0 = 0, psiA = 0; // 离"正对相机"转角（度）：psi=atan2(nx,-nz)，正对≈0
    double dist = 0;           // 板心距离（m）
    double rmse0 = 0, rmseA = 0; // 两解在各自旋转(R)但固定选中平移 t0 下的重投影 RMSE(px)
    int frame = 0;             // 来源帧号（调试用）
};

// 由一块"已解算"的装甲板（Solver::solvePose 成功）构造样本。INVALID 类型返回 false。
bool makeSample(const Armor& a, int frame, Sample& out);

// ---- 角度 / 姿态工具 ----
// 归一化到 (-180,180]
double wrapDeg(double deg);
// 法线方位角 yaw = atan2(R(0,2), R(2,2))（模型 +z 朝外）
double normalAzimuthDeg(const double R[9]);
// 离正对转角 psi = wrapDeg(atan2(R(0,2), -R(2,2)))：正对相机时≈0
double headPsiDeg(const double R[9]);
// R = Ry(deltaDeg) · R_sel（绕相机竖轴 y 的增量旋转），输出 9 元行主序
void rotateYaw(const Sample& s, double deltaDeg, double Rout[9]);
// 用位姿 R + s.t0 投影 4 角点，得 8 维残差(px) e={u0-qx0,v0-qy0,...}；角点跑到相机后返回 false
bool evalResiduals(const Sample& s, const double R[9], double e[8]);

// ---- 代价函数（Δ=0 即 IPPE 选中姿态） ----
// Σ_k[(u-qx)²+(v-qy)²]（px²），无导数但可数值求 Jacobian
double costSq(const Sample& s, double deltaDeg);
// Σ_k‖(u-qx,v-qy)‖（px），逐角点 L2 和，对离群角点更稳
double costL1(const Sample& s, double deltaDeg);

// ---- 代价曲线 ----
struct Curve {
    std::vector<double> xs;   // Δ（度）
    std::vector<double> sq;   // costSq
    std::vector<double> l1;   // costL1
};
Curve scanCurve(const Sample& s, double lo, double hi, double stepDeg);
// 等间距序列 y 的局部极小下标（排除 1e300 的无效点）
std::vector<int> localMinima(const std::vector<double>& y);

// ---- 一维优化器：变量为 Δ(度)，Δ=0 即 IPPE 选中姿态 ----
struct OptResult {
    double delta = 0;   // 最优 Δ
    int    evals = 0;   // 代价求值次数（各方法口径见实现）
    double bestF = 0;   // costSq 在最优 Δ 的值
};
namespace opt1d {
    // 枚举网格扫：基准"真值"（代价函数可非单峰，网格最稳）
    OptResult grid(const Sample& s, double lo, double hi, double stepDeg);
    // 黄金分割：假定单峰。实测在"单深谷+浅远谷"上会锁错谷，勿用于全局
    OptResult golden(const Sample& s, double lo, double hi);
    // Brent 无导数：假定单峰且给三点点括 (lo, mid, hi)
    OptResult brent(const Sample& s, double lo, double mid, double hi);
    // 阻尼高斯牛顿(LM)：数值 Jacobian，8 残差。从 PnP 起点(Δ≈0)起步收敛快
    OptResult lm(const Sample& s, double startDeg);
    // 生产推荐：1° 粗扫定主谷 + Brent 抛光（全局稳健且便宜）
    OptResult prod(const Sample& s);
}

// ---- 渲染 ----
// 画某代表样本的完整代价曲线图并保存为 outdir/curve_rep{idx}_psi{...}.png：
// 三面板：costSq(log) / costL1(log) / costSq 近 Δ=0 线性（黑竖线标局部极小）；
// 图上标注两个 IPPE 候选：绿=选中解(Δ=0)、橙=另一解(Δ=dAlt)。
// dAlt/costAltSq：另一候选在 Δ 域的坐标与固定 t 代价（无另一解时 costAltSq≥1e20）。
void saveCurveFigure(const Sample& s, int idx, const std::string& outdir,
                     const Curve& c, double dAlt, double costAltSq, double zoomRad);

}  // namespace yawstudy
}  // namespace task2

#endif  // YAWSTUDY_HPP
