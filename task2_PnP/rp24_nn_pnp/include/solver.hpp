#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <opencv2/core.hpp>
#include <vector>

#include "armor.hpp"

namespace task2 {

class Solver {
public:
    Solver();   // 默认用真实相机内参（1440x1080 标定值）
    Solver(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs = cv::Mat());

    // 对一帧装甲板做 6-DOF PnP，结果写进 armor.rvec / armor.tvec / distance / yaw
    bool solvePose(Armor& armor);

    // 把装甲板坐标系下一点 (x,y,z)（单位 mm）投影回图像
    cv::Point2f projectPoint(const cv::Mat& rvec, const cv::Mat& tvec,
                             const cv::Point3f& obj_pt) const;

    // 投影 z 轴：连 (0,0,0) 和 (0,0,50) 并画到 img 上
    void drawZAxis(cv::Mat& img, const Armor& armor,
                   const cv::Scalar& color = cv::Scalar(0, 255, 0)) const;

private:
    cv::Mat camera_matrix_, dist_coeffs_;
    static const std::vector<cv::Point3f>& modelSmall();  // 半宽65 半高27.5 (mm)
    static const std::vector<cv::Point3f>& modelBig();    // 半宽115 半高27.5 (mm)
};

}  // namespace task2

#endif  // SOLVER_HPP