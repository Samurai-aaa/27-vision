#include "tracker.hpp"

#include <algorithm>

#include "math_tools.hpp"

namespace task3
{

Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: state(State::LOST),
  tracked_number(""),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff),
  detect_count_(0)
{
}

bool Tracker::init(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  // 离相机最近的板观测质量最高，作为整车初值的来源
  const ObservedArmor * closest = &armors.front();
  for (const auto & a : armors)
    if (a.xyz.norm() < closest->xyz.norm()) closest = &a;

  // 根据车牌号优化初始化参数（sp 同款）
  int armor_num;
  double radius;
  Eigen::VectorXd P0_dig;
  if (closest->number == "O") {
    // 前哨站：三块板同半径同高，无长短轴概念 → l/h 分量协方差置 0 钉死
    armor_num = 3;
    radius = 0.2765;
    P0_dig = (Eigen::VectorXd(11) << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0).finished();
  } else {
    // 普通车：4 板。大装甲 2 板车型（平衡步兵）暂不支持，视频里没有
    armor_num = 4;
    radius = 0.2;
    P0_dig = (Eigen::VectorXd(11) << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1).finished();
  }

  tracked_number = closest->number;
  target = Target(*closest, t, radius, armor_num, P0_dig);

  state = State::DETECTING;
  detect_count_ = 0;
  return true;
}

void Tracker::update(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t)
{
  // 每帧先整车预测。TEMP_LOST / 未匹配时没有修正，预测值就是输出
  // ——需求 1 的"掉帧/漏检时强制构造可视化"发生在这一步
  target.predict(t);

  // 门控：同号观测板 与 模型预测出的各板（armor_xyza_list）找最近配对，
  // 位置 + 朝向双阈值同时满足才算匹配（rm 的 max_match_distance/max_match_yaw_diff）
  bool matched = false;
  const ObservedArmor * matched_armor = nullptr;
  double min_position_diff = 1e10, yaw_diff = 1e10;
  const auto xyza_list = target.armor_xyza_list();

  for (const auto & a : armors) {
    if (a.number != tracked_number) continue;  // 只考虑锁定的同号车
    for (const auto & xyza : xyza_list) {
      auto position_diff = (a.xyz - xyza.head(3)).norm();
      if (position_diff < min_position_diff) {
        min_position_diff = position_diff;
        yaw_diff = std::abs(limit_rad(a.yaw - xyza[3]));
        matched_armor = &a;
      }
    }
  }

  if (matched_armor != nullptr && min_position_diff < max_match_distance_ &&
      yaw_diff < max_match_yaw_diff_) {
    matched = true;
    target.update(*matched_armor);  // 内部自会做整车多板数据关联
  }

  // r 限幅：半径是弱观测维度，噪声会把它拉飞（rm 同款 0.12~0.4 m）
  auto x = target.ekf_x();
  if (x[8] < 0.12 || x[8] > 0.4) {
    x[8] = std::clamp(x[8], 0.12, 0.4);
    target.set_x(x);
  }

  // 状态机转移（rm 同款）
  switch (state) {
    case State::DETECTING:
      if (matched) {
        if (++detect_count_ > tracking_thres) {
          detect_count_ = 0;
          state = State::TRACKING;
        }
      } else {
        detect_count_ = 0;
        state = State::LOST;
      }
      break;
    case State::TRACKING:
      if (!matched) {
        state = State::TEMP_LOST;
        temp_lost_time_ = t;
      }
      break;
    case State::TEMP_LOST:
      if (matched) {
        state = State::TRACKING;
      } else if (delta_time(t, temp_lost_time_) > lost_time_thres) {
        state = State::LOST;
      }
      break;
    default:  // LOST：由调用方（node 层）发现后转 init
      break;
  }

  // 发散保护：r 或 r+l 跑出物理范围 → 丢弃整车模型，下帧重新 init
  if (state != State::LOST && target.diverged()) state = State::LOST;
}

}  // namespace task3
