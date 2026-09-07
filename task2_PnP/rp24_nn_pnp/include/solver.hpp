#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <opencv2/core.hpp>
#include <vector>

#include "armor.hpp"

namespace task2 {

// PnP 解算方式。枚举顺序即 main.cpp 里 pnp_method 的终端索引：
//   0 DUAL_IPPE  双解(solvePnPGeneric)：IPPE 两解里选"板面朝相机+重投影误差最小"，
//              未选中的另一解（镜像假设）留 armor.rvec_alt/tvec_alt，供橙色画出来
//   1 IPPE       单解（solvePnP）
//   2 SQPNP      全局最优、任意点构型≥3
//   3 EPNP       快速 n 点（≥4）
//   4 ITERATIVE  LM 迭代精修（平面无初值可能落镜像支，适合配 guess 当精修器）
//   5 P3P        恰 4 点（4 点校验后唯一）
//   6 AP3P       恰 4 点（4 点校验后唯一）
enum class PnPMethod {
    DUAL_IPPE, IPPE, SQPNP, EPNP, ITERATIVE, P3P, AP3P,
};

// yaw 重投影精修配置（参照 sp_vision optimize_yaw：解算后校验，误差过大则在
// yaw 范围内遍历重投影，取 4 角点像素误差和最小者更新）
struct YawRefineConfig {
    bool   enable        = true;   // 是否做精修（false = 保持 PnP 原始 yaw）
    double thresh_px     = 6.0;    // 当前重投影 4 角点误差和(px)超过该值才触发遍历
    double search_range_deg = 15.0;  // yaw 遍历半范围（绕竖轴，单位度）
    double search_step_deg  = 0.5;   // yaw 遍历步长（度）
};

class Solver {
public:
    Solver();   // 默认用真实相机内参（1440x1080 标定值）
    Solver(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs = cv::Mat());

    void setMethod(PnPMethod method);
    PnPMethod method() const;

    // yaw 重投影精修配置（见 YawRefineConfig）
    void setYawRefineConfig(const YawRefineConfig& cfg);
    const YawRefineConfig& yawRefineConfig() const;

    // 统一入口：按当前方法（method_）解一帧装甲板位姿，
    // 结果写进 armor.rvec / armor.tvec / distance / yaw
    bool solvePose(Armor& armor);

    // 显式用 solvePnPGeneric：在 IPPE 双解中选"板面朝相机 + 重投影误差最小"的解
    bool solvePnPGeneric(Armor& armor);

    // 把装甲板坐标系下一点 (x,y,z)（单位 mm）投影回图像
    cv::Point2f projectPoint(const cv::Mat& rvec, const cv::Mat& tvec,
                             const cv::Point3f& obj_pt) const;

    // 投影 z 轴：连 (0,0,0) 和 (0,0,50) 并画到 img 上（画 armor 选中的解）
    void drawZAxis(cv::Mat& img, const Armor& armor,
                   const cv::Scalar& color = cv::Scalar(0, 255, 0)) const;

    // 画任意一组 rvec/tvec 的 z 轴（重载）：用于把 solvePnPGeneric 未选中的
    // 另一解也用别的颜色画出来（如橙 = 镜像假设）
    void drawZAxis(cv::Mat& img, const cv::Mat& rvec, const cv::Mat& tvec,
                   const cv::Scalar& color = cv::Scalar(0, 255, 0)) const;

private:
    cv::Mat camera_matrix_, dist_coeffs_;
    PnPMethod method_ = PnPMethod::DUAL_IPPE;   // 与 main 默认 pnp_method=0 一致
    YawRefineConfig yaw_refine_;

    // solvePnP 单解共用：按给定 OpenCV flag 解一帧（校验/清另一解/解/填指标）
    bool solveSingle(Armor& armor, int flag);

    // 由 rvec/tvec 填 distance / yaw（solvePose 与 solvePnPGeneric 共用）
    void fillPoseMetrics(Armor& armor) const;

    // 解完的后处理：记录 yaw_raw，若重投影误差过大则绕竖轴遍历择优（yaw 精修）
    void postProcessYaw(Armor& armor);

    // 把当前模型 4 角点按 rvec/tvec 重投影，与 NN 检测角点逐点算 L2 距离和(px)
    double reprojErr(const Armor& armor, const cv::Mat& R, const cv::Mat& tvec) const;

    static const std::vector<cv::Point3f>& modelSmall();  // 半宽65 半高27.5 (mm)
    static const std::vector<cv::Point3f>& modelBig();    // 半宽115 半高27.5 (mm)
};

}  // namespace task2

#endif  // SOLVER_HPP