#ifndef TARGET_HPP
#define TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "extended_kalman_filter.hpp"

namespace task3 {

// tracker 的观测输入：node 层从 armor_interfaces::msg::Armor 翻译而来。
// 不 include 任何 ROS 消息/detector 头，target 层保持零 ROS 依赖。
struct ObservedArmor {
  std::string number;     // "1"~"5" / "outpost"
    Eigen::Vector3d xyz;    // 板中心，相机系，单位 m
    double yaw;             // 板朝向角（板法线的水平投影），rad
};

class Target
{
public:
  std::string number;  // "1"~"5" / "outpost"（前哨站特判用）
  bool jumped;         // 本次匹配到的不是主板（id != 0）
  int last_id;    // debug only

  Target() = default;

  // 从第一块实测装甲板初始化整车状态（车心由板位置反推）
  // t 为该帧时间戳：t_ 从它起步，否则第一次 predict(t) 的 dt 会从时钟纪元算起
  Target(
    const ObservedArmor & armor, std::chrono::steady_clock::time_point t, double radius,
    int armor_num, Eigen::VectorXd P0_dig);

  // 合成数据：直接给真值状态、P0=0（EKF 收敛性验证用）
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  // 状态往前推 dt 秒（CV/CA 模型切换在此实现）
  void predict(double dt);
  // 用一块观测修正状态（内部先做装甲板匹配）
  void update(const ObservedArmor & armor);

  // 查询接口（node 层转 Target.msg / 可视化用）
  Eigen::VectorXd ekf_x() const;
  void set_x(const Eigen::VectorXd & x);  // 状态回写（tracker 层 r 限幅用）
  const ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;  // 每块板 [x,y,z,朝向角]

  bool diverged() const;    // r/r+l 越界 → 滤波发散，上层应重置
  bool convergened();       // 观测次数足够且未发散 → 状态可信

  bool isinit = false;

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;  // 上次预测的时间戳（算 dt 用）

  void update_ypda(const ObservedArmor & armor, int id);  // yaw pitch distance angle 观测

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace task3

#endif  // TARGET_HPP
