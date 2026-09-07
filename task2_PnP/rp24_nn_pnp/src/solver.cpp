#include "solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cfloat>
#include <cmath>

namespace task2 {

Solver::Solver()
    : camera_matrix_((cv::Mat_<double>(3, 3) <<
        2556.2545862166521, 0, 705.83803766013978,
        0, 2553.5331992802749, 584.62889512335437,
        0, 0, 1)),
      dist_coeffs_() {}

Solver::Solver(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    : camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs) {}

void Solver::setMethod(PnPMethod method) { method_ = method; }
PnPMethod Solver::method() const { return method_; }

void Solver::setYawRefineConfig(const YawRefineConfig& cfg) { yaw_refine_ = cfg; }
const YawRefineConfig& Solver::yawRefineConfig() const { return yaw_refine_; }

// 统一入口：按当前方法（method_）分派。0=双解走 solvePnPGeneric，
// 其余各内核走 solvePnP 单解（solveSingle）。任一解出 pose 后统一做
// postProcessYaw（记录 yaw_raw + 重投影误差大时绕竖轴遍历择优）。
bool Solver::solvePose(Armor& armor) {
    bool ok = false;
    switch (method_) {
        case PnPMethod::DUAL_IPPE: ok = solvePnPGeneric(armor); break;
        case PnPMethod::IPPE:      ok = solveSingle(armor, cv::SOLVEPNP_IPPE); break;
        case PnPMethod::SQPNP:     ok = solveSingle(armor, cv::SOLVEPNP_SQPNP); break;
        case PnPMethod::EPNP:      ok = solveSingle(armor, cv::SOLVEPNP_EPNP); break;
        case PnPMethod::ITERATIVE: ok = solveSingle(armor, cv::SOLVEPNP_ITERATIVE); break;
        case PnPMethod::P3P:       ok = solveSingle(armor, cv::SOLVEPNP_P3P); break;
        case PnPMethod::AP3P:      ok = solveSingle(armor, cv::SOLVEPNP_AP3P); break;
    }
    if (ok) postProcessYaw(armor);
    return ok;
}

// solvePnP 单解共用：校验 → 清另一解 → 按 flag 解 → 填指标。
// 单解只有一解，rvec_alt/tvec_alt 置空（防止对象复用残留上一次的橙轴）。
bool Solver::solveSingle(Armor& armor, int flag) {
    if (armor.type == ArmorType::INVALID || armor.corners.size() != 4) return false;
    armor.rvec_alt.release();
    armor.tvec_alt.release();
    const auto& model = armor.type == ArmorType::BIG ? modelBig() : modelSmall();
    bool ok = cv::solvePnP(model, armor.corners, camera_matrix_, dist_coeffs_,
                           armor.rvec, armor.tvec, false, flag);
    if (!ok) return false;
    fillPoseMetrics(armor);
    return true;
}

// 把 armor 对应 3D 模型（大/小板）按 R/tvec 重投影，与 NN 检测 4 角点
// 逐点算 L2 像素距离之和（px）——sp_vision optimize_yaw 同款误差指标。
double Solver::reprojErr(const Armor& armor, const cv::Mat& R, const cv::Mat& tvec) const {
    if (armor.type == ArmorType::INVALID || armor.corners.size() != 4) return DBL_MAX;
    const auto& model = armor.type == ArmorType::BIG ? modelBig() : modelSmall();
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    std::vector<cv::Point2f> proj;
    cv::projectPoints(model, rvec, tvec, camera_matrix_, dist_coeffs_, proj);
    double err = 0;
    for (int i = 0; i < 4; i++) err += cv::norm(proj[i] - armor.corners[i]);
    return err;
}

// 解算后处理（每个成功解出 pose 的装甲板都走这里）：
// 1) yaw_raw 保存 PnP 原始 yaw；
// 2) 若当前姿态的重投影 4 角点误差和超过阈值（yaw 解算不准），则把整块板绕
//    竖直轴（相机 y，即板绕自身竖直轴转）在当前 yaw 附近按步长遍历，每次重投影
//    角点找与 NN 检测最贴合（误差最小）的姿态；有切实改善才更新 armor.rvec/yaw。
//    板心(tvec)不变 —— 转的是板的朝向，不是位置。参照 sp_vision optimize_yaw。
void Solver::postProcessYaw(Armor& armor) {
    armor.yaw_refined = false;
    armor.yaw_raw = armor.yaw;                     // 保留 PnP 原始 yaw 供对照
    armor.yaw_refine_err0 = armor.yaw_refine_err1 = 0.0;
    if (!yaw_refine_.enable) return;
    if (armor.type == ArmorType::INVALID || armor.corners.size() != 4) return;

    cv::Mat R;
    cv::Rodrigues(armor.rvec, R);
    const double err0 = reprojErr(armor, R, armor.tvec);
    armor.yaw_refine_err0 = err0;
    if (err0 <= yaw_refine_.thresh_px) {           // 误差在阈值内 => 认为解算准确，不遍历
        armor.yaw_refine_err1 = err0;
        return;
    }

    // 绕竖直轴遍历：Rc = Ry(delta) * R，tvec 不动（旋转轴过板心）
    const double range = yaw_refine_.search_range_deg * CV_PI / 180.0;
    const double step  = yaw_refine_.search_step_deg  * CV_PI / 180.0;
    cv::Mat best_R = R.clone();
    double best_err = err0;
    for (double delta = -range; delta <= range + step * 1e-3; delta += step) {
        double c = std::cos(delta), s = std::sin(delta);
        cv::Mat Ry = (cv::Mat_<double>(3, 3) << c, 0, s,  0, 1, 0,  -s, 0, c);
        cv::Mat Rc = Ry * R;
        double e = reprojErr(armor, Rc, armor.tvec);
        if (e < best_err) { best_err = e; best_R = Rc; }
    }

    // 只有严格更小才更新，避免纯噪声导致 yaw 抖动
    armor.yaw_refine_err1 = best_err;
    if (best_err < err0 - 1e-6) {
        cv::Rodrigues(best_R, armor.rvec);
        fillPoseMetrics(armor);   // 用新姿态重填 yaw/distance（tvec 没变，distance 不变）
        armor.yaw_refined = true;
    }
}

// solvePnPGeneric：IPPE 平面目标固定返回 2 个解（真解 + 镜像解），
// 在其中选"板面法线朝相机 + 重投影误差最小"的那个，避免翻转退化姿态。
bool Solver::solvePnPGeneric(Armor& armor) {
    if (armor.type == ArmorType::INVALID || armor.corners.size() != 4) return false;
    const auto& model = armor.type == ArmorType::BIG ? modelBig() : modelSmall();

    std::vector<cv::Mat> rvecs, tvecs;
    int n = cv::solvePnPGeneric(model, armor.corners, camera_matrix_, dist_coeffs_,
                                rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    if (n <= 0) return false;

    // 选解：优先选"板面朝相机 + 重投影误差最小"的解；
    // 若所有解都朝后（几何失配导致 IPPE 退化），回退选重投影误差最小的解，
    // 保证上层能画出 z 轴（此时姿态可能翻转，是否可信由上层判断）。
    int best = 0;
    double best_err = DBL_MAX;
    bool have_facing = false;
    int fallback = 0;
    double fallback_err = DBL_MAX;
    for (int k = 0; k < n; k++) {
        cv::Mat R;
        cv::Rodrigues(rvecs[k], R);

        std::vector<cv::Point2f> proj;
        cv::projectPoints(model, rvecs[k], tvecs[k], camera_matrix_, dist_coeffs_, proj);
        double err = 0;
        for (int i = 0; i < 4; i++)
            err += cv::norm(proj[i] - armor.corners[i]);  // 像素误差

        if (err < fallback_err) { fallback_err = err; fallback = k; }
        if (R.at<double>(2, 2) <= 0) continue;  // 镜像解，板面朝后，仅作回退备选
        have_facing = true;
        if (err < best_err) { best_err = err; best = k; }
    }
    if (!have_facing) best = fallback;  // 全朝后：回退最小误差解，保证有输出

    armor.rvec = rvecs[best];
    armor.tvec = tvecs[best];

    // 另一解：取未被选中的那一个（IPPE 平面目标固定 2 解），供上层用别的
    // 颜色画出来对照；n==1 时无另一解，置空
    if (n > 1) {
        int alt = (best == 0) ? 1 : 0;
        armor.rvec_alt = rvecs[alt];
        armor.tvec_alt = tvecs[alt];
    } else {
        armor.rvec_alt.release();
        armor.tvec_alt.release();
    }

    fillPoseMetrics(armor);
    return true;
}

// 由 rvec/tvec 填 distance / yaw（两个入口共用）
void Solver::fillPoseMetrics(Armor& armor) const {
    // 距离 = tvec 模长（mm → m）
    armor.distance = cv::norm(armor.tvec) / 1000.0;

    // yaw = 装甲板 z 轴在相机 XZ 平面相对相机 z 轴的偏角
    // R 第三列就是装甲板 z 轴在相机系的方向
    cv::Mat R;
    cv::Rodrigues(armor.rvec, R);
    armor.yaw = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2)) * 180.0 / CV_PI;
}

// 把装甲板坐标系一点（mm）投影回图像
cv::Point2f Solver::projectPoint(const cv::Mat& rvec, const cv::Mat& tvec,
                                 const cv::Point3f& obj_pt) const {
    std::vector<cv::Point2f> out;
    cv::projectPoints(std::vector<cv::Point3f>{obj_pt}, rvec, tvec,
                      camera_matrix_, dist_coeffs_, out);
    return out[0];
}

// 画z轴：投影 (0,0,0) 和 (0,0,50) 连线（画 armor 选中的解）
void Solver::drawZAxis(cv::Mat& img, const Armor& armor,
                       const cv::Scalar& color) const {
    drawZAxis(img, armor.rvec, armor.tvec, color);
}

// 画任意一组 rvec/tvec 的 z 轴：画"另一解"时直接把它的位姿传进来
void Solver::drawZAxis(cv::Mat& img, const cv::Mat& rvec, const cv::Mat& tvec,
                       const cv::Scalar& color) const {
    auto p0 = projectPoint(rvec, tvec, {0, 0, 0});
    auto p1 = projectPoint(rvec, tvec, {0, 0, 50});
    cv::line(img, p0, p1, color, 2);
    cv::circle(img, p0, 3, color, -1);
    cv::circle(img, p1, 3, color, -1);
}

const std::vector<cv::Point3f>& Solver::modelSmall() {
    static const std::vector<cv::Point3f> pts{
        {-65, 27.5, 0}, {-65, -27.5, 0}, {65, -27.5, 0}, {65, 27.5, 0}};
    return pts;
}

const std::vector<cv::Point3f>& Solver::modelBig() {
    static const std::vector<cv::Point3f> pts{
        {-115, 27.5, 0}, {-115, -27.5, 0}, {115, -27.5, 0}, {115, 27.5, 0}};
    return pts;
}

}  // namespace task2
