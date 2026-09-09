#include "target.hpp"

#include <algorithm>
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
    const Eigen::Vector3d & xyz = armor.xyz;  // 板中心（相机系 x右 y下 z前）
    double a = armor.yaw;  // 板法线在相机 x-z 水平面的方位角 atan2(n_z,n_x)（前向 ∈(-π,0)）

    // 车心反推：整车绕相机 y（竖直）轴在 x-z 平面水平公转，板心
    //   p = C + r·(cos a, 0, sin a)   （法线径向朝外 = (cos a,0,sin a)；
    //    前向板 sin a<0 → 板比车心更近相机）→  C = p − r·(cos a, 0, sin a)
    auto center_x = xyz[0] - r * std::cos(a);
    auto center_y = xyz[1];  // 首板即与车心同高（板间高度差归 dz 维管）
    auto center_z = xyz[2] - r * std::sin(a);

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

int Target::update(const ObservedArmor & armor)
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

    // 最近 3 块里选 "|Δ板法线方位角|" 最小者。新几何板只绕竖直轴公转，块间靠相位
    // φ 区分（邻板差 2π/N），而各板同高 → 屏幕方位 atan2(y,x) 无区分度，故只比 φ。
    int n_cand = std::min(3, armor_num_);  // 防止 2 板车（平衡步兵）越界
    for (int i = 0; i < n_cand; i++) {
        const auto & xyza = xyza_i_list[i].first;
        auto angle_error = std::abs(limit_rad(armor.yaw - xyza[3]));
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
    return id;
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

void Target::set_x(const Eigen::VectorXd & x) { ekf_.x = x; }

const ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

Eigen::Vector3d Target::center() const { return {ekf_.x[0], ekf_.x[2], ekf_.x[4]}; }

Eigen::Vector3d Target::velocity() const { return {ekf_.x[1], ekf_.x[3], ekf_.x[5]}; }

Eigen::Vector3d Target::armor_xyz(int id) const { return h_armor_xyz(ekf_.x, id); }

int Target::armor_num() const { return armor_num_; }

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

// 只读未来外推。转移口径与 predict(double) 的状态矩阵 F 完全一致
// （位置 += 速度·τ、yaw += v_yaw·τ 并归一化；r/l/dz 在 τ 窗口内视为不变），但只
// 作用在副本上、不写 ekf_ 与 t_——渲染层若直接插一次 predict 会推进真实状态，
// 下一帧 predict(now) 的 dt 就把这 τ 多算一遍，滤波被污染。τ 内不建模新观测/过程
// 噪声：纯"当前估计的速度/转速把目标带到哪"的瞄准提前量预览。
std::vector<Eigen::Vector4d> Target::armor_xyza_list_at(double tau) const
{
    Eigen::VectorXd xf = ekf_.x;
    xf[0] += xf[1] * tau;  // cx += vx·τ
    xf[2] += xf[3] * tau;  // cy += vy·τ
    xf[4] += xf[5] * tau;  // cz += vz·τ
    xf[6] = limit_rad(xf[6] + xf[7] * tau);  // 公转相位推进并归一化

    std::vector<Eigen::Vector4d> list;
    for (int i = 0; i < armor_num_; i++) {
        auto angle = limit_rad(xf[6] + i * 2 * M_PI / armor_num_);
        Eigen::Vector3d xyz = h_armor_xyz(xf, i);
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

    // R 自适应（按掠射角 delta）：板面正对相机时 PnP 法线方位最可信，越偏切线越不可
    // 信。delta = 板法线 (cos yaw,0,sin yaw) 与指向相机的视线 −p 的夹角（用预测板心 p）。
    // R 分块顺序与观测 z=[方位,俯仰,距离,板角] 对齐：距离噪声随距离增大(log)，板角
    // 噪声随掠射增大(log(delta+1))。
    Eigen::Vector3d p_plate = h_armor_xyz(ekf_.x, id);
    double cos_yaw = std::cos(armor.yaw), sin_yaw = std::sin(armor.yaw);
    double cos_delta = -1.0 / std::max(1e-6, p_plate.norm()) * (cos_yaw * p_plate[0] +
                                                                sin_yaw * p_plate[2]);
    double delta_angle = std::acos(std::clamp(cos_delta, -1.0, 1.0));
    Eigen::Vector3d ypd_obs = xyz2ypd(armor.xyz);
    Eigen::VectorXd R_dig{
        {4e-3, 4e-3,
         log(std::abs(ypd_obs[2]) + 1) / 200 + 9e-2,
         log(delta_angle + 1) + 1}};
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
    // 整车绕相机 y（竖直）轴在 x-z 水平面公转：板心 = 车心 + r·(cosφ, 0, sinφ)，
    // 板面竖直、法线径向朝外 n̂=(cosφ,0,sinφ)；前向板 sinφ<0（n̂ 的 z 分量指相机）。
    // 高差沿相机 y：dz>0 = 板比车心更高 → 相机 y 向下故相减。id 1、3 用长半径 r+l 并
    // 抬 dz（4 板车对角两组对角板异高），2/3 板车只有一组。
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

    auto r = (use_l_h) ? x[8] + x[9] : x[8];
    auto armor_x = x[0] + r * std::cos(angle);
    auto armor_y = x[2] - ((use_l_h) ? x[10] : 0.0);
    auto armor_z = x[4] + r * std::sin(angle);

    return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
    auto angle = limit_rad(x[6] + id * 2 * M_PI / armor_num_);
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

    // 与 h_armor_xyz 对应：p=(x0+r cosφ, x2−dz, x4+r sinφ)，r=(use? x8+x9 : x8)
    auto r = (use_l_h) ? x[8] + x[9] : x[8];
    auto dx_da = -r * std::sin(angle);              // ∂px/∂yaw（相位）
    auto dz_da = r * std::cos(angle);               // ∂pz/∂yaw

    auto dx_dr = std::cos(angle);                   // ∂px/∂r
    auto dz_dr = std::sin(angle);                   // ∂pz/∂r
    auto dx_dl = (use_l_h) ? std::cos(angle) : 0.0; // ∂px/∂l（id1/3 长半径）
    auto dz_dl = (use_l_h) ? std::sin(angle) : 0.0; // ∂pz/∂l

    auto dy_dh = (use_l_h) ? -1.0 : 0.0;            // ∂py/∂dz（相机 y 向下，dz>0=更高）

    // clang-format off
    // 状态 → 板 [x,y,z,angle] 的雅可比（4×11）。板绕竖直轴公转：yaw 相位只驱动
    // x/z 分量，竖直 y 只由车心 y(x2) 与 dz 高度偏置贡献。
    Eigen::MatrixXd H_armor_xyza{
        {1, 0, 0, 0, 0, 0,  dx_da, 0, dx_dr, dx_dl,      0},
        {0, 0, 1, 0, 0, 0,      0, 0,     0,     0,  dy_dh},
        {0, 0, 0, 0, 1, 0,  dz_da, 0, dz_dr, dz_dl,      0},
        {0, 0, 0, 0, 0, 0,      1, 0,     0,     0,      0}
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
