#include "solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cfloat>
#include <cmath>

namespace task3 {

Solver::Solver()
    : camera_matrix_((cv::Mat_<double>(3, 3) <<
        2556.2545862166521, 0, 705.83803766013978,
        0, 2553.5331992802749, 584.62889512335437,
        0, 0, 1)),
      dist_coeffs_() {}

Solver::Solver(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    : camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs) {}

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
    armor.reproj_err = have_facing ? best_err : fallback_err;
    fillPoseMetrics(armor);
    return true;
}

// 由 rvec/tvec 填 distance / yaw
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

// 画z轴：投影 (0,0,0) 和 (0,0,50) 连线
void Solver::drawZAxis(cv::Mat& img, const Armor& armor,
                       const cv::Scalar& color) const {
    auto p0 = projectPoint(armor.rvec, armor.tvec, {0, 0, 0});
    auto p1 = projectPoint(armor.rvec, armor.tvec, {0, 0, 50});
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

}  // namespace task3
