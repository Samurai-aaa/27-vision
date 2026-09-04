#include "target.hpp"

#include <cmath>

#include "math_tools.hpp"

namespace task3 {

namespace {

// 角度安全加法：EKF 所有 "状态 + 增量"（x = x + K·innovation）都走它，
// 和的角度分量归一化，防止状态角漂出 ±pi（例如 3.1 + 0.1 直接折回 -3.08 附近）
Eigen::VectorXd angle_safe_add(const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
  Eigen::VectorXd c = a + b;
  c[6] = limit_rad(c[6]);
  return c;
}

}  // namespace

Target::Target(
    const ObservedArmor & armor, std::chrono::steady_clock::time_point t, double radius,
    int armor_num, Eigen::VectorXd P0_dig)
    : number(armor.number),
    jumped(false),
    last_id(0),
    armor_num_(armor_num),
    switch_count_(0),
    update_count_(0),
    is_switch_(false),
    is_converged_(false),
    t_(t)
{
    auto r = radius;
    const Eigen::Vector3d & xyz = armor.xyz;  // 板中心（相机系）
    double a = armor.yaw;                     // 板朝向角 = 车体朝向

    // 旋转中心的坐标：板装在车体对角上，板平面距旋转轴的水平距离为 r，
    // 板法线水平方向即车体朝向 → 从板中心沿朝向前进 r 即车心
    auto center_x = xyz[0] + r * std::cos(a);
    auto center_y = xyz[1] + r * std::sin(a);
    auto center_z = xyz[2];  // 车心与所观测的板同高（板间高度差归 h 维管）

    Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, a, 0, r, 0, 0}};
    Eigen::MatrixXd P0 = P0_dig.asDiagonal();

    ekf_ = ExtendedKalmanFilter(x0, P0, angle_safe_add);
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
    Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
    Eigen::VectorXd P0_dig = Eigen::VectorXd::Zero(11);  // 状态视为精确已知
    Eigen::MatrixXd P0 = P0_dig.asDiagonal();

    ekf_ = ExtendedKalmanFilter(x0, P0, angle_safe_add);
}

// 时间戳版预测：算出与上次的时间间隔 dt，推状态，再记录本帧时间。
// node 层每个图像帧调一次 this->predict(img_stamp)。
void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
    // 状态转移矩阵
    // clang-format off
    Eigen::MatrixXd F{
        {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
    };
    double v1, v2;
    if (number == "O") {  // "O" = outpost（detector 的 kLabels[6]）
        v1 = 10;   // 前哨站加速度方差
        v2 = 0.1;  // 前哨站角加速度方差
    } else {
        v1 = 100;  // 加速度方差
        v2 = 400;  // 角加速度方差
    }
    auto a = dt * dt * dt * dt / 4;
    auto b = dt * dt * dt / 2;
    auto c = dt * dt;
    // 预测过程噪声偏差的方差
    // clang-format off
    Eigen::MatrixXd Q{
        {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
        {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
        {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
        {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
        {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
        {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
        {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
        {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
        {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
        {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
        {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
    };
    // clang-format on

    // 防止夹角求和出现异常值：预测后的朝向角归一化到 ±pi
    auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
        Eigen::VectorXd x_prior = F * x;
        x_prior[6] = limit_rad(x_prior[6]);
        return x_prior;
    };

    // 前哨站转速特判：收敛后按规则固定转速 0.8pi（2.51 rad/s）钳制
    if (convergened() && number == "O" && std::abs(ekf_.x[7]) > 2)
        ekf_.x[7] = ekf_.x[7] > 0 ? 2.51 : -2.51;

    ekf_.predict(F, Q, f);
}

void Target::update(const ObservedArmor & armor)
{
    // 数据关联：观测对应的模型里的块板
    int id = 0;
    double min_angle_error = 1e10;
    const std::vector<Eigen::Vector4d> xyza_list = armor_xyza_list();

    std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
    for (int i = 0; i < armor_num_; i++) xyza_i_list.push_back({xyza_list[i], i});

    // 按预测距离升序：近板离相机近、观测质量高，优先参与匹配
    std::sort(
        xyza_i_list.begin(), xyza_i_list.end(),
        [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
            return a.first.head(3).norm() < b.first.head(3).norm();
        });

    double obs_yaw_line = std::atan2(armor.xyz[1], armor.xyz[0]);  // 观测方位角

    // 最近 3 块里选 "|Δ板朝向| + |Δ方位|" 综合误差最小者
    int n_cand = std::min(3, armor_num_);  // 防止 2 板车（平衡步兵）越界
    for (int i = 0; i < n_cand; i++) {
        const auto & xyza = xyza_i_list[i].first;
        auto angle_error = std::abs(limit_rad(armor.yaw - xyza[3])) +
                           std::abs(limit_rad(obs_yaw_line - std::atan2(xyza[1], xyza[0])));
        if (angle_error < min_angle_error) {
            id = xyza_i_list[i].second;
            min_angle_error = angle_error;
        }
    }

    // 记录匹配结果（tracker 层和 debug 用）
    if (id != 0) jumped = true;
    is_switch_ = (id != last_id);
    if (is_switch_) switch_count_++;
    last_id = id;
    update_count_++;

    update_ypda(armor, id);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

void Target::set_x(const Eigen::VectorXd & x) { ekf_.x = x; }

const ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
    std::vector<Eigen::Vector4d> list;
    for (int i = 0; i < armor_num_; i++) {
        auto angle = limit_rad(ekf_.x[6] + i * 2 * M_PI / armor_num_);
        Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
        list.push_back({xyz[0], xyz[1], xyz[2], angle});
    }
    return list;
}

bool Target::diverged() const
{
    auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
    auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;
    return !(r_ok && l_ok);
}

bool Target::convergened()
{
    int need = (number == "O") ? 10 : 3;  // 前哨转速慢，收敛要更多观测
    if (update_count_ > need && !diverged()) is_converged_ = true;
    return is_converged_;
}

void Target::update_ypda(const ObservedArmor & armor, int id)
{
    // 观测雅可比（当前状态处线性化）
    Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

    // R 自适应：板法线偏离视线（delta_angle）越大 → 斜视角下 PnP 朝向越不可信；
    // 距离越远 → 距离观测噪声越大（log 模型）
    auto center_yaw = std::atan2(armor.xyz[1], armor.xyz[0]);
    auto delta_angle = limit_rad(armor.yaw - center_yaw);
    Eigen::Vector3d ypd_obs = xyz2ypd(armor.xyz);
    Eigen::VectorXd R_dig{
        {4e-3, 4e-3,
         log(std::abs(delta_angle) + 1) + 1,
         log(std::abs(ypd_obs[2]) + 1) / 200 + 9e-2}};
    Eigen::MatrixXd R = R_dig.asDiagonal();

    // h: 整车状态 → 观测空间 [方位, 俯仰, 距离, 板朝向角]
    auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
        Eigen::Vector3d xyz = h_armor_xyz(x, id);
        Eigen::Vector3d ypd = xyz2ypd(xyz);
        auto angle = limit_rad(x[6] + id * 2 * M_PI / armor_num_);
        return {ypd[0], ypd[1], ypd[2], angle};
    };

    // 残差的角度分量解卷绕（方位/俯仰/板朝向是角度，距离不是）
    auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
        Eigen::VectorXd c = a - b;
        c[0] = limit_rad(c[0]);
        c[1] = limit_rad(c[1]);
        c[3] = limit_rad(c[3]);
        return c;
    };

    Eigen::VectorXd z{{ypd_obs[0], ypd_obs[1], ypd_obs[2], armor.yaw}};

    ekf_.update(z, H, R, h, z_subtract);
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
    auto angle = limit_rad(x[6] + id * 2 * M_PI / armor_num_);
    // 4 板车对角交替长短轴/高度差（id 1、3 用 r+l 和 z+h）；2/3 板车只有一组
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

    auto r = (use_l_h) ? x[8] + x[9] : x[8];
    auto armor_x = x[0] - r * std::cos(angle);
    auto armor_y = x[2] - r * std::sin(angle);
    auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

    return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
    auto angle = limit_rad(x[6] + id * 2 * M_PI / armor_num_);
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

    auto r = (use_l_h) ? x[8] + x[9] : x[8];
    auto dx_da = r * std::sin(angle);
    auto dy_da = -r * std::cos(angle);

    auto dx_dr = -std::cos(angle);
    auto dy_dr = -std::sin(angle);
    auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
    auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

    auto dz_dh = (use_l_h) ? 1.0 : 0.0;

    // clang-format off
    // 状态 → 板 [x,y,z,angle] 的雅可比（4×11）
    Eigen::MatrixXd H_armor_xyza{
        {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
        {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
        {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
        {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
    };

    Eigen::Vector3d armor_xyz = h_armor_xyz(x, id);
    Eigen::MatrixXd H_armor_ypd = xyz2ypd_jacobian(armor_xyz);
    // 板 [x,y,z,angle] → 观测 [y,p,d,angle]（角度行直接透传）
    Eigen::MatrixXd H_armor_ypda{
        {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
        {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
        {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
        {                0,                 0,                 0, 1}
    };
    // clang-format on

    // 链式法则：∂ypd/∂状态 = ∂ypd/∂板xyz × ∂板xyz/∂状态
    return H_armor_ypda * H_armor_xyza;
}

}  // namespace task3
