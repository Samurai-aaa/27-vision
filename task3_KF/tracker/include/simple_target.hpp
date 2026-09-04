#ifndef TRACKER__SIMPLE_TARGET_HPP_
#define TRACKER__SIMPLE_TARGET_HPP_

#include <chrono>

#include <Eigen/Dense>

#include "extended_kalman_filter.hpp"
#include "target.hpp"  // ObservedArmor

namespace task3
{

// 需求①②：普通 KF（线性）状态估计，CV / CA 两种运动模型对比。
// 状态直接描述所跟踪的那块装甲板，不建模整车旋转；观测就是 PnP 的 xyz。
// 复用 ExtendedKalmanFilter 框架：predict/update 不传非线性钩子即退化为普通 KF。
class SimpleTarget
{
public:
  enum class Model { CV, CA };

  // v1：过程噪声强度——CV 里是加速度方差，CA 里是加加速度（jerk）方差。
  // 量级参照：直线板心的加速度 std ~1 m/s² 起步（v1=1 即 σa=1）。
  // 注意不要照搬整车 Target 的 100：那是给小陀螺车心用的，单板直线用会速度发散。
  // 需求②对比实验的主调参数：变速机动大 → 调大 v1，匀速 → 调小
  SimpleTarget(
    const ObservedArmor & armor, Model model, std::chrono::steady_clock::time_point t,
    double v1 = 1.0);

  // 每帧先调：推状态。漏检帧只调它 → 输出纯预测位置（需求①掉帧构造）
  void predict(std::chrono::steady_clock::time_point t);
  // 固定步长外推（预测虚线框 / 误差评估用）
  void predict(double dt);

  // z = PnP 的 xyz 直接观测（线性修正：不传 h，默认 z = H·x）
  void update(const ObservedArmor & armor);

  Eigen::VectorXd ekf_x() const;
  const ExtendedKalmanFilter & ekf() const;
  Eigen::Vector3d armor_xyz() const;  // 估计的板中心
  bool diverged() const;

private:
  Model model_;
  double v1_;
  Eigen::MatrixXd H_;  // 位置抽取（3×dim），armor_xyz = H_ · x
  ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;
};

}  // namespace task3

#endif  // TRACKER__SIMPLE_TARGET_HPP_
