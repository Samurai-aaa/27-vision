#include "tracker_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "target.hpp"

namespace task3
{

namespace
{

// 从四元数提"板朝向角"（占位定义，见 ctor 注释）：取旋转矩阵第三列
// （solvePnP 装甲板坐标系的 z 轴 = 板法线）在水平面(x-y)的方位。
// 整车小陀螺模型的旋转平面、朝向角基准是否与相机系完全一致，
// 待阶段三坐标统一时校准——当前主要保证有值可用。
double armorYawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
  Eigen::Matrix3d R = eq.toRotationMatrix();
  return std::atan2(R(1, 2), R(0, 2));
}

// 该 number 是否为可跟踪目标："1"~"5" 步兵/英雄、前哨 "O"、哨兵 "G"。
// detector 的 kLabels：G=哨兵（会移动/自转，整车 EKF 的主要目标）、
// Bs/Bb（能量机关/基地，固定不可击打）不锁。
bool trackable(const std::string & n)
{
  if (n == "O" || n == "G") return true;
  return n.size() == 1 && n[0] >= '1' && n[0] <= '5';
}

std::string stateName(State s)
{
  switch (s) {
    case State::LOST: return "LOST";
    case State::DETECTING: return "DETECTING";
    case State::TRACKING: return "TRACKING";
    case State::TEMP_LOST: return "TEMP_LOST";
  }
  return "?";
}

}  // namespace

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: Node("tracker_node", options)
{
  max_match_distance_ = declare_parameter<double>("max_match_distance", 0.2);
  max_match_yaw_diff_ = declare_parameter<double>("max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance_, max_match_yaw_diff_);
  tracker_->tracking_thres = declare_parameter<int>("tracking_thres", 5);
  tracker_->lost_time_thres = declare_parameter<double>("lost_time_thres", 0.3);

  armors_sub_ = create_subscription<armor_interfaces::msg::Armors>(
    "/armors", rclcpp::QoS(10), [this](const armor_interfaces::msg::Armors::SharedPtr msg) {
      onArmors(msg);
    });
  target_pub_ = create_publisher<armor_interfaces::msg::Target>("/tracker/target", rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/tracker/marker", rclcpp::QoS(10));

  // 掉帧外推：TEMP_LOST 时 100ms tick 一次 predict 并发布 predicted=true
  predict_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this]() { onTimer(); });

  RCLCPP_INFO(get_logger(), "TrackerNode 就绪: max_match_distance=%.2f max_match_yaw_diff=%.2f",
              max_match_distance_, max_match_yaw_diff_);
}

void TrackerNode::onArmors(const armor_interfaces::msg::Armors::SharedPtr msg)
{
  // 掉帧期间到达的空 /armors 也要推进状态机：predict 后进入 TEMP_LOST
  std::vector<ObservedArmor> obs;
  obs.reserve(msg->armors.size());
  for (const auto & m : msg->armors) {
    if (!trackable(m.number)) continue;
    ObservedArmor a;
    a.number = m.number;
    a.xyz << m.pose.position.x, m.pose.position.y, m.pose.position.z;  // 相机系 m
    a.yaw = armorYawFromQuat(m.pose.orientation);
    obs.push_back(a);
  }

  auto now = std::chrono::steady_clock::now();
  if (tracker_->state == State::LOST) {
    if (!tracker_->init(obs, now)) return;  // 无可锁目标，保持 LOST 等下一帧
  } else {
    tracker_->update(obs, now);
  }

  // 记录进入 TEMP_LOST 的时刻，供停流时 timer 判超时
  bool now_temp = tracker_->state == State::TEMP_LOST;
  if (now_temp && !was_temp_lost_) temp_lost_since_ = now;
  was_temp_lost_ = now_temp;

  frame_id_ = msg->header.frame_id;
  armor_interfaces::msg::Target target_msg;
  target_msg.header.stamp = msg->header.stamp;
  target_msg.header.frame_id = frame_id_;
  bool tracking = tracker_->state == State::TRACKING || tracker_->state == State::TEMP_LOST;
  fillTargetMsg(target_msg, tracking);
  target_pub_->publish(target_msg);
  publishMarker(target_msg);

  RCLCPP_DEBUG(
    get_logger(), "[%s] id=%s 观测=%zu", stateName(tracker_->state).c_str(),
    tracker_->tracked_number.c_str(), obs.size());
}

void TrackerNode::onTimer()
{
  // 只兜底"掉帧停流"情形：TEMP_LOST 期间持续 predict 外推，
  // 让漏检帧也能输出强制构造的整车估计（需求①）
  if (tracker_->state != State::TEMP_LOST) return;

  auto now = std::chrono::steady_clock::now();

  // 停流时 Tracker::update 不再被调，TEMP_LOST → LOST 的超时由这里补判
  if (now - temp_lost_since_ > std::chrono::duration<double>(tracker_->lost_time_thres)) {
    tracker_->state = State::LOST;
    was_temp_lost_ = false;
    armor_interfaces::msg::Target target_msg;  // 清空 marker 用 tracking=false
    target_msg.tracking = false;
    target_pub_->publish(target_msg);
    publishMarker(target_msg);
    RCLCPP_INFO(get_logger(), "TEMP_LOST 超时，回到 LOST");
    return;
  }

  tracker_->target.predict(now);

  armor_interfaces::msg::Target target_msg;
  target_msg.header.stamp = rclcpp::Clock().now();
  target_msg.header.frame_id = frame_id_;
  fillTargetMsg(target_msg, true);
  target_pub_->publish(target_msg);
  publishMarker(target_msg);
}

void TrackerNode::fillTargetMsg(armor_interfaces::msg::Target & msg, bool tracking)
{
  msg.tracking = tracking;
  msg.id = tracker_->tracked_number;
  if (!tracking) return;  // LOST 态 target 可能未构造，勿读 ekf_x()
  const auto x = tracker_->target.ekf_x();
  msg.armors_num = (tracker_->tracked_number == "O") ? 3 : 4;

  msg.position.x = x[0];
  msg.velocity.x = x[1];
  msg.position.y = x[2];
  msg.velocity.y = x[3];
  msg.position.z = x[4];
  msg.velocity.z = x[5];
  msg.yaw = x[6];
  msg.v_yaw = x[7];
  msg.radius_1 = x[8];
  // 4 板车有长短轴 r + l；前哨 3 板同半径，radius_2 无意义
  msg.radius_2 = (msg.armors_num == 4) ? x[8] + x[9] : x[8];
  msg.dz = x[10];
}

void TrackerNode::publishMarker(const armor_interfaces::msg::Target & msg)
{
  visualization_msgs::msg::MarkerArray arr;
  if (!msg.tracking) {
    marker_pub_->publish(arr);  // 空数组=清除
    return;
  }

  const auto x = tracker_->target.ekf_x();
  visualization_msgs::msg::Marker center;
  center.header.stamp = msg.header.stamp;
  center.header.frame_id = msg.header.frame_id;
  center.ns = "center";
  center.type = visualization_msgs::msg::Marker::SPHERE;
  center.action = visualization_msgs::msg::Marker::ADD;
  center.scale.x = center.scale.y = center.scale.z = 0.1;
  center.color.r = 0.0f; center.color.g = 1.0f; center.color.b = 0.0f; center.color.a = 1.0f;
  center.pose.position.x = x[0];
  center.pose.position.y = x[2];
  center.pose.position.z = x[4];
  arr.markers.push_back(center);

  // 四块预测板（整车的装甲板集合可视化，需求④）
  const auto xyza = tracker_->target.armor_xyza_list();
  for (size_t i = 0; i < xyza.size(); i++) {
    visualization_msgs::msg::Marker armor;
    armor.header.stamp = msg.header.stamp;
    armor.header.frame_id = msg.header.frame_id;
    armor.ns = "armors";
    armor.id = static_cast<int>(i);
    armor.type = visualization_msgs::msg::Marker::CUBE;
    armor.action = visualization_msgs::msg::Marker::ADD;
    armor.scale.x = 0.2; armor.scale.y = 0.05; armor.scale.z = 0.13;
    armor.color.b = 1.0f; armor.color.a = 0.6f;
    armor.pose.position.x = xyza[i][0];
    armor.pose.position.y = xyza[i][1];
    armor.pose.position.z = xyza[i][2];
    // 朝向占位：绕相机 z 转 xyza[3]，精确基准待坐标统一
    double y = xyza[i][3];
    armor.pose.orientation.w = std::cos(y / 2);
    armor.pose.orientation.z = std::sin(y / 2);
    arr.markers.push_back(armor);
  }

  marker_pub_->publish(arr);
}

}  // namespace task3

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<task3::TrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
