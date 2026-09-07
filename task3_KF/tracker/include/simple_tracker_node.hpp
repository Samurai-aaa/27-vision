#ifndef TRACKER__SIMPLE_TRACKER_NODE_HPP_
#define TRACKER__SIMPLE_TRACKER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <armor_interfaces/msg/armors.hpp>
#include <armor_interfaces/msg/target.hpp>

#include "simple_target.hpp"

namespace task3
{

// ROS 壳：把 /armors 翻译成 ObservedArmor，喂给需求①② 的历史单板普通 KF（SimpleTarget，
// CV 6 维 / CA 9 维）。与整车 tracker_node 并行、互不抢话题：
//   /simple_tracker/target      — 估计的板心位置/速度（Target.msg，整车字段置 0）
//   /simple_tracker/final_img   — 渲染（绿=本帧实测命中板，紫=掉帧/无同号板时的纯预测框）
// 目标锁定口径与整车 EKF 一致：锁定"车号"，其后只喂同号板、忽略其它号。
// 本文件只做"消息 ↔ 算法层"的翻译与时钟驱动，不掺滤波逻辑。
class SimpleTrackerNode : public rclcpp::Node
{
public:
  explicit SimpleTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onArmors(const armor_interfaces::msg::Armors::SharedPtr msg);
  // 缓存最近一帧 /image 作为渲染底图
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);
  // 掉帧全停流定时器：/armors 长时间不来时周期 predict 外推并发布 predicted=true
  void onTimer();

  // ObservedArmor 翻译：只保留可跟踪号（1-5/O/G），算法层只认 number/xyz，角点与姿态随带
  std::vector<ObservedArmor> buildObs(const armor_interfaces::msg::Armors::SharedPtr msg);

  // LOST 时取"离相机最近的可跟踪板"初始化滤波器并锁定该号
  bool tryInit(const std::vector<ObservedArmor> & obs, std::chrono::steady_clock::time_point now);
  // 同号候选里找"离 KF 预测最近"的板喂 update，过近超门控则视为漏检（纯外推）
  const ObservedArmor * pick(const std::vector<ObservedArmor> & obs);
  void reset();

  void publishAndRender(std::chrono::steady_clock::time_point now);
  void renderAndPublish();
  void drawHud(cv::Mat & out);

  rclcpp::Subscription<armor_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<armor_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr final_img_pub_;
  rclcpp::TimerBase::SharedPtr predict_timer_;

  // 算法层
  std::unique_ptr<SimpleTarget> st_;
  SimpleTarget::Model model_ = SimpleTarget::Model::CV;
  double v1_ = 1.0;               // 过程噪声强度（CV=加速度方差，CA=加加速度方差）
  std::string tracked_number_;    // 锁定的目标车号（只喂同号板）
  bool matched_now_ = false;      // 本帧是否实际吃到同号板 update（false = 纯预测/外推帧）
  ObservedArmor last_matched_;    // 最近一次实际命中的板（画预测框的姿态 / 掉帧外推参照）

  // 超时/停流判断
  std::chrono::steady_clock::time_point last_match_time_{};  // 上次真正命中 update 的时刻
  std::chrono::steady_clock::time_point last_frame_time_{};  // 上次收到 /armors 的时刻
  double max_match_distance_ = 0.2;   // 同号板距 KF 预测的最近位置门控，m（防串到邻板）
  double lost_time_thres_ = 0.3;      // 连续无命中超过该秒数 → 回 LOST 重新锁

  // 渲染底图与相机参数
  cv::Mat latest_img_;
  std_msgs::msg::Header latest_img_header_;
  bool has_img_ = false;
  double fx_, fy_, cx_, cy_;
  double armor_width_, armor_height_;
  bool show_hud_;
};

}  // namespace task3

#endif  // TRACKER__SIMPLE_TRACKER_NODE_HPP_
