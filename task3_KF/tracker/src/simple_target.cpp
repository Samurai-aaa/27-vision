#include "simple_target.hpp"

#include <cmath>

#include "math_tools.hpp"

namespace task3
{

SimpleTarget::SimpleTarget(
  const ObservedArmor & armor, Model model, std::chrono::steady_clock::time_point t, double v1)
: model_(model), v1_(v1), t_(t)
{
  // 状态交错排列：x/y/z 每轴一组 [p, v(, a)]，F/Q 按轴分块复制同一小块
  const int n_axis = (model_ == Model::CA) ? 3 : 2;  // 每轴状态数：CA 三阶，CV 二阶
  const int dim = n_axis * 3;
  const int pos[3] = {0, n_axis, 2 * n_axis};  // 各轴起始下标

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dim);
  for (int i = 0; i < 3; i++) x0[pos[i]] = armor.xyz[i];

  // 初值不确定性：位置较信（PnP 米级 1m²），速度/加速度完全未知
  Eigen::VectorXd P0_dig = Eigen::VectorXd::Zero(dim);
  for (int i = 0; i < 3; i++) {
    P0_dig[pos[i]] = 1.0;
    P0_dig[pos[i] + 1] = 100.0;
    if (n_axis == 3) P0_dig[pos[i] + 2] = 1000.0;
  }

  ekf_ = ExtendedKalmanFilter(x0, P0_dig.asDiagonal());

  // H：从状态里抽 [x, y, z]
  H_ = Eigen::MatrixXd::Zero(3, dim);
  for (int i = 0; i < 3; i++) H_(i, pos[i]) = 1.0;
}

void SimpleTarget::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void SimpleTarget::predict(double dt)
{
  const int n_axis = (model_ == Model::CA) ? 3 : 2;
  const int dim = n_axis * 3;

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(dim, dim);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim, dim);

  for (int i = 0; i < 3; i++) {  // x/y/z 各轴独立，同一小块复制三份
    auto B = F.block(n_axis * i, n_axis * i, n_axis, n_axis);
    auto Qb = Q.block(n_axis * i, n_axis * i, n_axis, n_axis);
    if (n_axis == 2) {
      // CV：匀速，加速度当白噪声
      // clang-format off
      B  << 1, dt,
            0, 1;
      Qb << dt* dt* dt* dt / 4, dt* dt* dt / 2,
            dt* dt* dt / 2,     dt* dt;
      // clang-format on
    } else {
      // CA：匀加速，加加速度（jerk）当白噪声
      double t2 = dt * dt, t3 = t2 * dt, t4 = t3 * dt, t5 = t4 * dt;
      // clang-format off
      B  << 1, dt, t2 / 2,
            0, 1,  dt,
            0, 0,  1;
      Qb << t5 / 20, t4 / 8, t3 / 6,
            t4 / 8,  t3 / 3, t2 / 2,
            t3 / 6,  t2 / 2, dt;
      // clang-format on
    }
  }

  ekf_.predict(F, Q * v1_);  // 不传 f：默认 x = F·x，普通 KF 的线性预测
}

void SimpleTarget::update(const ObservedArmor & armor)
{
  // PnP 特性：横向误差小、深度误差大，R 起步常值对角、之后按距离调
  Eigen::VectorXd R_dig{{1e-2, 1e-2, 1e-1}};

  // 不传 h / z_subtract：默认 z = H·x、残差 = z − h(x)，普通 KF 的线性修正
  Eigen::VectorXd z = armor.xyz;
  ekf_.update(z, H_, R_dig.asDiagonal());
}

Eigen::VectorXd SimpleTarget::ekf_x() const { return ekf_.x; }

const ExtendedKalmanFilter & SimpleTarget::ekf() const { return ekf_; }

Eigen::Vector3d SimpleTarget::armor_xyz() const { return H_ * ekf_.x; }

Eigen::Vector3d SimpleTarget::velocity() const
{
  // 状态交错：每轴 [p, v(, a)]，速度恒在每轴 +1 下标
  const int n_axis = (model_ == Model::CA) ? 3 : 2;
  Eigen::Vector3d v;
  for (int i = 0; i < 3; i++) v[i] = ekf_.x[i * n_axis + 1];
  return v;
}

bool SimpleTarget::diverged() const
{
  return !ekf_.x.allFinite() || ekf_.x.norm() > 50.0;
}

}  // namespace task3
