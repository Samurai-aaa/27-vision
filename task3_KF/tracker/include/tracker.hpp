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
  // 多块命中时取 primary_id_ 那一块（相位连续性，见 update），不是全局最近那块——
  // 保证绿框跟在同一块板上直到它转出视野才交棒，换板瞬间不抖/不跳隔块。
  std::optional<ObservedArmor> matched_armor;
  // 最近一次命中的观测（含板朝向 rot）。TEMP_LOST 掉帧期间不回清（直到回 LOST）。
  // node 掉帧画外推板时用它保持目标真实朝向，不随整车自转相位摆动
  std::optional<ObservedArmor> last_obs;

  // 当前"正在追踪"的整车模型板号（0~N-1）：绿框在跟的那块。TEMP_LOST 掉帧期间不更新，
  // node 层据此用 EKF 整车公转外推续画"正在追踪的检测框"（橙色），与 TRACKING 的绿框
  // 保持同一追踪对象（simple_tracker 掉帧紫框的整车版）。
  int primary_id() const { return primary_id_; }

  // node 层可调参数
  double high_confidence = 0.65;
  double low_confidence = 0.35;
  double min_class_margin = 1.0;
  int max_weak_frames = 8;
  int primary_switch_frames = 3;
  int tracking_thres = 5;  // DETECTING → TRACKING 需要的累计命中帧数（容忍窗口内的漏检不清零）
  // 漏检容忍窗口：连续 max_miss_frames 帧无命中才回 LOST。多车并存 / 远距离 / 弱光下
  // detector 会成片丢板（blu.avi 实测有板帧只占 56%，最差一段仅 9%），逐帧判定会让
  // 目标在 LOST 上反复断流、画面时有时无。稠密检测时等价于原来的逐帧行为。
  int max_miss_frames = 15;
  // TEMP_LOST 持续超过该秒数 → LOST。只在 /armors 断流（update 不再被调）时兜底：
  // 流还活着时由 max_miss_frames 按帧判定，所以这里要给得比 N 帧对应的时长宽
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
  // "正在追踪"的整车模型板号（0~N-1）：主命中板（绿框）的相位连续性锚点。一块板
  // 还检得到就一直是它；它转出视野后才移交给下一可见板，随旋转单调推进。
  int primary_id_ = 0;
  int primary_miss_count_ = 0;
};

}  // namespace task3

#endif  // TRACKER__TRACKER_HPP_
