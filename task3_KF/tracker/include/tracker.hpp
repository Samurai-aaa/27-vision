#ifndef TRACKER__TRACKER_HPP_
#define TRACKER__TRACKER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "target.hpp"

namespace task3
{

// rm_vision 四态：DETECTING 连续匹配若干帧才转正，TRACKING 稳定跟踪，
// TEMP_LOST 漏检期间纯预测续命（需求 1 掉帧构造发生在这里），超时才回 LOST
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
  explicit Tracker(double max_match_distance, double max_match_yaw_diff);

  // LOST 态下用这批观测锁定目标：选最近板建整车 EKF，进入 DETECTING。
  // 板数/初始半径/初始协方差按车牌号自动选择（前哨 3 板，其余 4 板）
  bool init(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t);

  // 非 LOST 态每帧调用：先整车预测 → 同号板与预测板集合做"位置+朝向"双阈值门控
  // → 匹配成功才修正 EKF → 状态机转移 → 发散保护
  void update(const std::vector<ObservedArmor> & armors, std::chrono::steady_clock::time_point t);

  State state;
  std::string tracked_number;  // 锁定的车牌号（"1"~"5" / "O"）
  Target target;               // 整车 EKF（LOST 态下无意义）

  // node 层可调参数
  int tracking_thres = 5;        // DETECTING → TRACKING 需要的连续匹配帧数
  double lost_time_thres = 0.3;  // TEMP_LOST 持续超过该秒数 → LOST

private:
  double max_match_distance_;  // 位置门控，m
  double max_match_yaw_diff_;  // 朝向门控，rad
  int detect_count_;
  std::chrono::steady_clock::time_point temp_lost_time_;  // 进入 TEMP_LOST 的时刻
};

}  // namespace task3

#endif  // TRACKER__TRACKER_HPP_
