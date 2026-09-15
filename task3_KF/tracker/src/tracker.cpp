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

void Tracker::reset()
{
  state = State::LOST;
  tracked_number.clear();
  target.reset();
  matched_armor.reset();
  last_obs.reset();
  detect_count_ = 0;
  miss_count_ = 0;
  weak_count_ = 0;
  primary_id_ = 0;
  primary_miss_count_ = 0;
}

bool Tracker::init(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  // 离相机最近的板观测质量最高，作为整车初值（车心由该板位置反推）的来源
  const ObservedArmor * closest = nullptr;
  for (const auto & a : armors) {
    if (a.confidence < high_confidence || a.color_uncertain || a.class_margin < min_class_margin) continue;
    if (closest == nullptr || a.xyz.norm() < closest->xyz.norm()) closest = &a;
  }
  if (!closest) return false;

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
    P0_dig << 0.1, 4, 0.1, 4, 0.1, 4, 0.4, 25, 0.0004, 0.0004, 0.0001;
  }
  if (armor_num_override > 0) armor_num = armor_num_override;
  if (radius_override > 0.0) radius = radius_override;

  tracked_number = closest->number;
  target = Target(*closest, t, radius, armor_num, P0_dig);
  matched_armor = *closest;  // 锁定帧即命中该板（node 画贴合框）
  last_obs = *closest;       // 最近命中观测（含朝向，掉帧外推用）
  primary_miss_count_ = 0;
  primary_id_ = 0;           // 初始化板被 Target 建为模型板 0（车心由它反推）

  state = State::DETECTING;
  detect_count_ = 0;
  miss_count_ = 0;
  weak_count_ = 0;
  return true;
}

void Tracker::update(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t)
{
  if (!target) return;  // 防御：LOST 下 node 层会先 init，不应走到这里

  matched_armor.reset();  // 每帧先清：本帧是否命中由下面关联决定

  // 每帧先整车预测。TEMP_LOST / 未匹配时没有修正，预测值就是输出
  // ——需求 1 的"掉帧/漏检时强制构造可视化"发生在这一步
  target->predict(t);

  // 同号观测与预测板联合位置/朝向门控，按代价贪心一对一分配。
  struct Hit {
    const ObservedArmor * obs;
    int mid;      // EKF 给这块观测关联的整车模型板号（0~N-1）
    double pd;    // 到最近预测板的位置误差，m
  };
  std::vector<Hit> hits;
  const auto xyza_list = target->armor_xyza_list();
  struct Candidate { size_t obs; int mid; double cost; double distance; bool weak; };
  std::vector<Candidate> candidates;
  for (size_t i = 0; i < armors.size(); ++i) {
    const auto & a = armors[i];
    const bool same_number = a.number == tracked_number;
    const bool weak = a.confidence < high_confidence || a.color_uncertain ||
                      a.class_margin < min_class_margin || !same_number;
    if (a.confidence < low_confidence) continue;
    // 可信的异号板永不接纳；模糊车号只能在已确认轨迹附近短暂续跟。
    if (!same_number && a.class_margin >= min_class_margin) continue;
    if (weak && (state == State::DETECTING || weak_count_ >= max_weak_frames)) continue;
    for (size_t j = 0; j < xyza_list.size(); ++j) {
      const double pd = (a.xyz - xyza_list[j].head(3)).norm();
      const double yd = std::abs(limit_rad(a.yaw - xyza_list[j][3]));
      if (pd < max_match_distance_ * (weak ? 0.5 : 1.0) &&
          yd < max_match_yaw_diff_ * (weak ? 0.5 : 1.0))
        candidates.push_back({i, static_cast<int>(j),
          std::pow(pd/max_match_distance_, 2) + std::pow(yd/max_match_yaw_diff_, 2), pd, weak});
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    if (a.weak != b.weak) return !a.weak;  // 先关联高质量候选
    if (a.cost != b.cost) return a.cost < b.cost;
    if (a.mid != b.mid) return a.mid < b.mid;
    return a.obs < b.obs;
  });
  std::vector<bool> used_obs(armors.size(), false), used_id(xyza_list.size(), false);
  bool strong_hit = false;
  for (const auto & c : candidates) {
    if (used_obs[c.obs] || used_id[c.mid]) continue;
    // 先固定分配，避免被拒绝的观测转而尝试错误板号。
    used_obs[c.obs] = true;
    used_id[c.mid] = true;
    auto observation = armors[c.obs];
    observation.noise_scale = c.weak ? 4.0 : 1.0;
    if (target->update(observation, c.mid) >= 0) {
      hits.push_back({&armors[c.obs], c.mid, c.distance});
      strong_hit = strong_hit || !c.weak;
    }
  }

  // 弱观测不能无限续命；只有同号、高质量命中才能重置窗口。
  weak_count_ = strong_hit ? 0 : weak_count_ + 1;

  // 主命中板选择（matched_armor，node 画绿框+挂文本）：
  //   只要 primary_id_（上一帧"正在追踪"的模型板）本帧还有观测命中，就继续跟它——
  //   即使另一块可见板此刻更近也不换（否则换板瞬间绿框会在相邻两板间抖/提前跳）；
  //   连续 primary_switch_frames 帧无主板观测才移交给其他命中板，
  //   短暂遮挡时保留主板身份；其他板仍可修正整车。
  const Hit * nearest = nullptr;   // 退路：当前跟踪板消失后的接管板
  const Hit * same_id = nullptr;   // 延续：关联到 primary_id_ 的命中板
  for (const auto & h : hits) {
    if (nearest == nullptr || h.pd < nearest->pd) nearest = &h;
    if (h.mid == primary_id_ && (same_id == nullptr || h.pd < same_id->pd)) same_id = &h;
  }
  primary_miss_count_ = same_id ? 0 : primary_miss_count_ + 1;
  const Hit * chosen = same_id ? same_id :
    (primary_miss_count_ >= primary_switch_frames ? nearest : nullptr);
  if (chosen != nullptr) {
    if (same_id == nullptr) {
      primary_id_ = chosen->mid;
      primary_miss_count_ = 0;
    }  // 旧板转走，移交新可见板
    matched_armor = *chosen->obs;  // node 据此画"贴合实测框"（绿实线）
    last_obs = *chosen->obs;       // 刷新"最近朝向"（掉帧外推保持目标真实朝向）
  }

  // 状态机转移（rm 同款 + 漏检容忍窗口）
  const bool matched = !hits.empty();  // 其他板有效更新也算整车命中，主板暂缺不失锁
  // 连续无命中帧计数：命中即清零。状态机据此把"逐帧判定"放宽成"允许连续 N 帧漏检"，
  // 稀疏检出（多车/远距离/弱光）时不至于一漏就回 LOST、画面跟着断流
  miss_count_ = matched ? 0 : miss_count_ + 1;
  switch (state) {
    case State::DETECTING:
      if (matched) {
        if (++detect_count_ > tracking_thres) {
          detect_count_ = 0;
          state = State::TRACKING;
        }
      } else if (miss_count_ > max_miss_frames) {
        // 注意：容忍窗口内的漏检**不清零** detect_count_——断续命中也要能攒够转正帧数，
        // 否则稀疏检出下永远停在 DETECTING
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
      } else if (miss_count_ > max_miss_frames ||
                 delta_time(t, temp_lost_time_) > lost_time_thres) {
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
    reset();
  }
}

}  // namespace task3
