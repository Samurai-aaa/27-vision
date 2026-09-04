#ifndef TRACKER__TRACKER_NODE_HPP_
#define TRACKER__TRACKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <armor_interfaces/msg/armors.hpp>
#include <armor_interfaces/msg/target.hpp>

#include "tracker.hpp"

namespace task3
{

// ROS 壳：订阅 detector 的 /armors，把装甲板翻译成 ObservedArmor 喂给算法层 Tracker，
// 发布整车估计 Target.msg + 可视化 MarkerArray。
// 本文件只做"消息 ↔ 算法层"的翻译与时钟驱动，不掺滤波逻辑。
class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onArmors(const armor_interfaces::msg::Armors::SharedPtr msg);

  // 掉帧外推定时器：TEMP_LOST 期间即使 /armors 停流，也周期推进状态并发布
  // predicted=true 的整车估计（需求① 掉帧/漏检时的强制构造入口）
  void onTimer();

  // 填 Target.msg：把 tracker_ 的整车 EKF 状态按交错下标展开
  void fillTargetMsg(armor_interfaces::msg::Target & msg, bool tracking);
  void publishMarker(const armor_interfaces::msg::Target & msg);

  rclcpp::Subscription<armor_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Publisher<armor_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr predict_timer_;

  std::unique_ptr<Tracker> tracker_;
  std::string frame_id_ = "camera_optical_frame";

  // 停流超时判断：/armors 停流时 Tracker::update 不再被调，TEMP_LOST 的超时
  // 由 timer 用这两个成员补判
  bool was_temp_lost_ = false;
  std::chrono::steady_clock::time_point temp_lost_since_;

  // 参数
  double max_match_distance_;
  double max_match_yaw_diff_;
};

}  // namespace task3

#endif  // TRACKER__TRACKER_NODE_HPP_
