#ifndef TARGET_HPP
#define TARGET_HPP

#include <Eigen/Dense>
#include <array>
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
  double yaw;             // 板法线在相机 x-z 水平面的方位角 atan2(n_z,n_x)（rad），
                          //   前向可见板 ∈(-π,0)；整车绕竖直轴公转的第 id 板相位 = x[6]+id·2π/N
  // NN 四角点原图像素（TL,BL,BR,TR，x,y 交错 0..7）。算法层数据关联不用它，
  // 只是随观测带着，node 画"贴合实测框"时直接取（掉帧帧无此观测）。
  std::array<float, 8> corners_px = {};  // 全 0 = 无效/未带
  // 板局部坐标 → 相机系的旋转（板局部 x=宽、y=高、z=板法线朝外），由 pose 四元数转来。
  // 掉帧外推画"保持真实朝向的预测板"用（宽=rot.col(0)、高=rot.col(1)）
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
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
  // 用一块观测修正状态（内部先做装甲板匹配）。返回该观测关联到的整车模型板号
  // （0~N-1）。tracker 层拿它做"正在追踪板"的相位连续性锚点：绿框绑在同一块板上，
  // 换板瞬间不再因"全局最近距离"在相邻板之间抖/回跳。
  int update(const ObservedArmor & armor);

  // 查询接口（node 层转 Target.msg / 可视化用）
  Eigen::VectorXd ekf_x() const;
  void set_x(const Eigen::VectorXd & x);  // 状态回写（tracker 层 r 限幅用）
  const ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;  // 每块板 [x,y,z,朝向角]

  // 需求⑤ 只读未来外推：把当前状态按确定性转移推到 tau 秒后，返回那时 N 块预测板
  // [x,y,z,朝向角]。**不改 EKF 状态/时间戳**（渲染插一次 predict 会污染下一帧 dt），
  // 纯给渲染层画"150ms 后未来板框"（瞄准提前量预览）用。
  std::vector<Eigen::Vector4d> armor_xyza_list_at(double tau) const;

  // 整车状态直接查询（node 层填 Target.msg / marker / 掉帧重建用）
  Eigen::Vector3d center() const;      // 车心位置 (x[0], x[2], x[4])，相机系 m
  Eigen::Vector3d velocity() const;    // 车心速度 (x[1], x[3], x[5])，m/s
  Eigen::Vector3d armor_xyz(int id) const;  // 第 id 块板的预测板心（= h_armor_xyz 口径）
  int armor_num() const;               // 建模的板上块数（初始化时按车牌/配置选定）
  int update_count() const { return update_count_; }  // 累计参与滤波修正的观测板数（HUD 用）

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
