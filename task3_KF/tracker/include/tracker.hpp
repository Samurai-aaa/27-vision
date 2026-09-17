#ifndef TRACKER__TRACKER_HPP_
#define TRACKER__TRACKER_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "target.hpp"

namespace task3
{

// 四态：DETECTING 连续匹配若干帧才转正，TRACKING 稳定跟踪，
// TEMP_LOST 漏检期间纯预测续命，超时才回 LOST
enum class State
{
  LOST,
  DETECTING,
  TRACKING,
  TEMP_LOST,
};

class Tracker
{
public:
  // 整车 EKF 门控参数：位置门控 max_match_distance（m）+ 板朝向门控
  // max_match_yaw_diff（rad），同时满足才算匹配命中（rm 同款双阈值）
  Tracker(double max_match_distance, double max_match_yaw_diff);

  // LOST 态下用这批观测锁定目标：选最近板建"整车 EKF"（Target），进入 DETECTING。
  // 板数/初始半径/初始协方差按车牌号自动选择（前哨 3 板、其余 4 板）；若 node 通过
  // armor_num_override / radius_override 给了非 0 覆盖值则优先用覆盖值（实测标定）
  bool init(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t);
  void reset();

  // 非 LOST 态每帧调用：先整车预测 → 同号观测板与整车预测板集合做"位置+朝向"双阈值
  // 门控 → 命中才修正 EKF → r 限幅 → 状态机转移 → 发散保护
  void update(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t);

  State state;
  std::string tracked_number;           // 锁定的车牌号（"1"~"5" / "O" / "G"）
  std::optional<Target> target;         // 整车 EKF（LOST 态下为空，丢弃等重新 init）

  // 本帧"正在追踪"的观测板（带 NN 角点）；失配/掉帧帧为空。node 渲染据此二选一：
  // 有值 → 连 NN 角点画"贴合实测框"（绿实线）；空（TEMP_LOST/外推）→ 整车预测板。
  // 取角度窗口选出的 primary_id_ 对应观测；其他板命中仍可更新整车。
  std::optional<ObservedArmor> matched_armor;
  // 最近一次命中的观测（含板朝向 rot）。TEMP_LOST 掉帧期间不回清（直到回 LOST）。
  // node 掉帧画外推板时用它保持目标真实朝向，不随整车自转相位摆动
  std::optional<ObservedArmor> last_obs;

  // 每帧按预测模型选主板（TEMP_LOST 也更新）；-1 表示没有满足瞄准窗口的板。
  int primary_id() const { return primary_id_; }

  // node 层可调参数
  double high_confidence = 0.65;
  Target::AdaptiveQOptions adaptive_q;
  double low_confidence = 0.35;
  double min_class_margin = 1.0;
  int max_weak_frames = 8;
  double aim_coming_angle = 60.0 * 3.141592653589793 / 180.0;
  double aim_leaving_angle = 20.0 * 3.141592653589793 / 180.0;
  double aim_spin_speed = 2.0;  // rad/s，使用转速 x[7] 区分普通运动/小陀螺
  int tracking_thres = 5;  // DETECTING → TRACKING 需要的累计命中帧数（容忍窗口内的漏检不清零）
  // 连续漏检帧数上限，与 TEMP_LOST 时间上限共同限制遮挡预测。
  int max_miss_frames = 120;
  // TEMP_LOST 持续超过该秒数 → LOST；node 同时处理消息断流。
  double lost_time_thres = 1.5;
  // 非 0 则覆盖按车牌号选择的板数/初始半径（0 = 走默认 O→3板/r0.2765，其余→4板/r0.2）
  int armor_num_override = 0;
  double radius_override = 0.0;

private:
  double max_match_distance_;  // 位置门控，m
  double max_match_yaw_diff_;  // 板朝向门控，rad
  int detect_count_;
  int weak_count_ = 0;
  int miss_count_ = 0;         // 当前连续无命中帧数：命中即清零，未命中累加（容忍窗口用）
  std::chrono::steady_clock::time_point temp_lost_time_;  // 进入 TEMP_LOST 的时刻
  // 主板选择独立于观测是否缺失，普通运动通过 aim_lock_id_ 保持窗口内连续性。
  int primary_id_ = 0;
  int aim_lock_id_ = -1;
  bool observed_other_plate_ = false;
};

}  // namespace task3

#endif  // TRACKER__TRACKER_HPP_
