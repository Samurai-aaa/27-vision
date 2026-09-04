#ifndef TRACKER__MATH_TOOLS_HPP_
#define TRACKER__MATH_TOOLS_HPP_

#include <chrono>
#include <cmath>

#include <Eigen/Dense>

namespace task3
{

// 角度归一化到 (-pi, pi]
inline double limit_rad(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// 两个时间戳的间隔，秒（double）
inline double delta_time(
  const std::chrono::steady_clock::time_point & t1, const std::chrono::steady_clock::time_point & t2)
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
}

// 直角坐标 → 球坐标 [方位角, 俯仰角, 距离]
inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];
  auto yaw = std::atan2(y, x);
  auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
  auto distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}

// xyz2ypd 对 xyz 的雅可比（3×3），链式法则里 xyz→ypd 的一段
inline Eigen::Matrix3d xyz2ypd_jacobian(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];
  auto s = x * x + y * y;          // 水平距离平方
  auto d = x * x + y * y + z * z;  // 距离平方

  Eigen::Matrix3d J;
  // clang-format off
  //       ∂/∂x                ∂/∂y                ∂/∂z
  J <<    -y / s,              x / s,                       0,
    -(x * z) / ((z * z / s + 1) * std::pow(s, 1.5)),
    -(y * z) / ((z * z / s + 1) * std::pow(s, 1.5)),
        1 / ((z * z / s + 1) * std::pow(s, 0.5)),
     x / std::pow(d, 0.5), y / std::pow(d, 0.5), z / std::pow(d, 0.5);
  // clang-format on
  return J;
}

}  // namespace task3

#endif  // TRACKER__MATH_TOOLS_HPP_
