#include "solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

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

bool Solver::solvePose(Armor& armor) {
    if (armor.type == ArmorType::INVALID || armor.corners.size() != 4) return false;
    const auto& model = armor.type == ArmorType::BIG ? modelBig() : modelSmall();
    cv::solvePnP(model, armor.corners, camera_matrix_, dist_coeffs_,
                 armor.rvec, armor.tvec, false, cv::SOLVEPNP_IPPE);

    // 距离 = tvec 模长（mm → m）
    armor.distance = cv::norm(armor.tvec) / 1000.0;

    // yaw = 装甲板 z 轴在相机 XZ 平面相对相机 z 轴的偏角
    // R 第三列就是装甲板 z 轴在相机系的方向
    cv::Mat R;
    cv::Rodrigues(armor.rvec, R);
    armor.yaw = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2)) * 180.0 / CV_PI;

    return true;
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

}
