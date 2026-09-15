#ifndef TRACKER__MATH_TOOLS_HPP_
#define TRACKER__MATH_TOOLS_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>

namespace task3
{

// 角度归一化到 (-pi, pi]
inline double limit_rad(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

// 两个时间戳的间隔，秒（double）
inline double delta_time(
  const std::chrono::steady_clock::time_point & t1, const std::chrono::steady_clock::time_point & t2)
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
}

// 相机系 x右、y下、z前：水平角绕竖直 y 轴，俯仰角向下为正。
// 这是视线角；车体板法线相位仍为 atan2(n_z,n_x)，二者基准轴不同。
inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  const double x = xyz.x(), y = xyz.y(), z = xyz.z();
  return {std::atan2(x, z), std::atan2(y, std::hypot(x, z)), xyz.norm()};
}

inline Eigen::Matrix3d xyz2ypd_jacobian(const Eigen::Vector3d & xyz)
{
  const double x = xyz.x(), y = xyz.y(), z = xyz.z();
  const double s = std::max(1e-12, x*x + z*z);
  const double h = std::sqrt(s);
  const double d2 = std::max(1e-12, s + y*y);
  const double d = std::sqrt(d2);
  Eigen::Matrix3d J;
  J << z/s, 0, -x/s,
       -x*y/(h*d2), h/d2, -z*y/(h*d2),
       x/d, y/d, z/d;
  return J;
}

}  // namespace task3

#endif  // TRACKER__MATH_TOOLS_HPP_
