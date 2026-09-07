#include "tracker.hpp"

#include <algorithm>
#include <cmath>

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

  // 离相机最近的板观测质量最高，作为整车初值（车心由该板位置反推）的来源
  const ObservedArmor * closest = &armors.front();
  for (const auto & a : armors)
    if (a.xyz.norm() < closest->xyz.norm()) closest = &a;

  // 板数/初始半径/初始协方差：默认按车牌号选择；node 可给非 0 覆盖值（实测标定）
  int armor_num;
  double radius;
  Eigen::VectorXd P0_dig(11);
  if (closest->number == "O") {
    // 前哨站：三块板同半径同高，无长短轴概念 → l/h 分量协方差置 0 钉死
    armor_num = 3;
    radius = 0.2765;
    P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
  } else {
    // 普通车（步兵/英雄/哨兵）：4 板
    armor_num = 4;
    radius = 0.2;
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
  }
  if (armor_num_override > 0) armor_num = armor_num_override;
  if (radius_override > 0.0) radius = radius_override;

  tracked_number = closest->number;
  target = Target(*closest, t, radius, armor_num, P0_dig);
  matched_armor = *closest;  // 锁定帧即命中该板（node 画贴合框）
  last_obs = *closest;       // 最近命中观测（含朝向，掉帧外推用）
  primary_id_ = 0;           // 初始化板被 Target 建为模型板 0（车心由它反推）

  state = State::DETECTING;
  detect_count_ = 0;
  return true;
}

void Tracker::update(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t)
{
  if (!target) return;  // 防御：LOST 下 node 层会先 init，不应走到这里

  matched_armor.reset();  // 每帧先清：本帧是否命中由下面关联决定

  // 每帧先整车预测。TEMP_LOST / 未匹配时没有修正，预测值就是输出
  // ——需求 1 的"掉帧/漏检时强制构造可视化"发生在这一步
  target->predict(t);

  // 数据关联门控：同号观测板 vs 整车预测出的各板（armor_xyza_list）。
  // 每块观测板找"离它最近的预测板"，位置+朝向双阈值同时满足即命中；命中块全部
  // 喂 target->update()（同帧多板融合，sp_vision 同款：整车转到两板同时可见时一起
  // 修正 yaw/r，可观测性更好）。update() 返回该观测关联到的模型板号 mid，用来做
  // 主命中板（绿框"正在追踪"）的相位连续性。
  struct Hit {
    const ObservedArmor * obs;
    int mid;      // EKF 给这块观测关联的整车模型板号（0~N-1）
    double pd;    // 到最近预测板的位置误差，m
  };
  std::vector<Hit> hits;
  const auto xyza_list = target->armor_xyza_list();
  for (const auto & a : armors) {
    if (a.number != tracked_number) continue;  // 只考虑锁定的同号车
    // 该观测板离最近的预测板多远 / yaw 差多少
    double pd_nearest = 1e10, yaw_diff = 0.0;
    for (const auto & xyza : xyza_list) {
      const double pd = (a.xyz - xyza.head(3)).norm();
      if (pd < pd_nearest) {
        pd_nearest = pd;
        yaw_diff = std::abs(limit_rad(a.yaw - xyza[3]));
      }
    }
    if (pd_nearest < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      int mid = target->update(a);  // 命中：该板参与整车修正（内部再自关联选 id）
      hits.push_back({&a, mid, pd_nearest});
    }
  }

  // 主命中板选择（matched_armor，node 画绿框+挂文本）：
  //   只要 primary_id_（上一帧"正在追踪"的模型板）本帧还有观测命中，就继续跟它——
  //   即使另一块可见板此刻更近也不换（否则换板瞬间绿框会在相邻两板间抖/提前跳）；
  //   只有它转出视野（本帧已无观测能关联到它）才移交给"当前最近命中板"（退路），
  //   并把它设成新的 primary_id_——交棒随整车旋转单调推进，不会跳隔块/回跳。
  const Hit * nearest = nullptr;   // 退路：当前跟踪板消失后的接管板
  const Hit * same_id = nullptr;   // 延续：关联到 primary_id_ 的命中板
  for (const auto & h : hits) {
    if (nearest == nullptr || h.pd < nearest->pd) nearest = &h;
    if (h.mid == primary_id_ && (same_id == nullptr || h.pd < same_id->pd)) same_id = &h;
  }
  const Hit * chosen = (same_id != nullptr) ? same_id : nearest;
  if (chosen != nullptr) {
    if (same_id == nullptr) primary_id_ = chosen->mid;  // 旧板转走，移交新可见板
    matched_armor = *chosen->obs;  // node 据此画"贴合实测框"（绿实线）
    last_obs = *chosen->obs;       // 刷新"最近朝向"（掉帧外推保持目标真实朝向）
  }

  // r 限幅：半径是弱观测维度，噪声会把它拉飞（rm 同款 0.12~0.4 m）
  auto x = target->ekf_x();
  if (x[8] < 0.12 || x[8] > 0.4) {
    x[8] = std::clamp(x[8], 0.12, 0.4);
    target->set_x(x);
  }

  // 状态机转移（rm 同款）
  const bool matched = matched_armor.has_value();
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

  // 发散保护 & 清理：r/r+l 跑出物理范围 → 回 LOST；回 LOST 即丢整车模型，
  // 下帧观测重新 init
  if (state != State::LOST && target->diverged()) state = State::LOST;
  if (state == State::LOST) {
    target.reset();
    matched_armor.reset();
    last_obs.reset();
  }
}

}  // namespace task3
