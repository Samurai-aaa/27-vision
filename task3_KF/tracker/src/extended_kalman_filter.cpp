#include "extended_kalman_filter.hpp"

#include <numeric>
#include <cmath>

namespace task3 {
ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
    : x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
    data["residual_yaw"] = 0.0;
    data["residual_pitch"] = 0.0;
    data["residual_distance"] = 0.0;
    data["residual_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nees"] = 0.0;
    data["nis_fail"] = 0.0;
    data["nees_fail"] = 0.0;
    data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
      return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
    P = F * P * F.transpose() + Q;
    x = f(x);
    return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    update_accepted = false;
    const Eigen::VectorXd residual = z_subtract(z, h(x));
    const Eigen::MatrixXd S = H * P * H.transpose() + R;
    if (!residual.allFinite() || !S.allFinite()) return x;
    const Eigen::LDLT<Eigen::MatrixXd> solve(S);
    if (solve.info() != Eigen::Success || !solve.isPositive() ||
        (solve.vectorD().array() <= 0).any()) return x;
    last_nis = residual.dot(solve.solve(residual));
    data["nis"] = last_nis;
    const bool rejected = !std::isfinite(last_nis) || last_nis > innovation_gate;
    data["nis_fail"] = rejected ? 1.0 : 0.0;
    recent_nis_failures.push_back(rejected ? 1 : 0);
    if (recent_nis_failures.size() > window_size) recent_nis_failures.pop_front();
    data["recent_nis_failures"] = static_cast<double>(
      std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0)) /
      recent_nis_failures.size();
    if (rejected) return x;
    const Eigen::MatrixXd K = solve.solve(H * P).transpose();
    const Eigen::VectorXd next_x = x_add(x, K * residual);
    const Eigen::MatrixXd A = I - K * H;
    Eigen::MatrixXd next_P = A * P * A.transpose() + K * R * K.transpose();
    next_P = (0.5 * (next_P + next_P.transpose())).eval();
    if (!next_x.allFinite() || !next_P.allFinite()) return x;
    x = next_x;
    P = next_P;
    update_accepted = true;
    if (residual.size() > 0) data["residual_yaw"] = residual[0];
    if (residual.size() > 1) data["residual_pitch"] = residual[1];
    if (residual.size() > 2) data["residual_distance"] = residual[2];
    if (residual.size() > 3) data["residual_angle"] = residual[3];
    // 无真值不能计算 NEES；不再将前后估计差冒充 NEES。

    return x;
}

}   // namespace task3