#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <opencv2/core.hpp>
#include <vector>

#include "armor.hpp"

namespace task3 {

class Solver {
public:
    Solver(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs = cv::Mat());

    // 显式用 solvePnPGeneric：在 IPPE 双解中选"板面朝相机 + 重投影误差最小"的解
    bool solvePnPGeneric(Armor& armor);

    // 把装甲板坐标系下一点 (x,y,z)（单位 mm）投影回图像
    cv::Point2f projectPoint(const cv::Mat& rvec, const cv::Mat& tvec,
                             const cv::Point3f& obj_pt) const;

    // 投影 z 轴：连 (0,0,0) 和 (0,0,50) 并画到 img 上
    void drawZAxis(cv::Mat& img, const Armor& armor,
                   const cv::Scalar& color = cv::Scalar(0, 255, 0)) const;

private:
    cv::Mat camera_matrix_, dist_coeffs_;

    void fillPoseMetrics(Armor& armor) const;

    static const std::vector<cv::Point3f>& modelSmall();  // 半宽65 半高27.5 (mm)
    static const std::vector<cv::Point3f>& modelBig();    // 半宽115 半高27.5 (mm)
};

}  // namespace task3

#endif  // SOLVER_HPP
