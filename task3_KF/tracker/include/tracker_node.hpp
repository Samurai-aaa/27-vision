#ifndef TRACKER__TRACKER_NODE_HPP_
#define TRACKER__TRACKER_NODE_HPP_

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <armor_interfaces/msg/armors.hpp>
#include <armor_interfaces/msg/target.hpp>

#include "tracker.hpp"

namespace task3
{

// ROS 壳：订阅 detector 的 /armors，把装甲板翻译成 ObservedArmor 喂给算法层 Tracker
//（整车 EKF），发布整车状态估计 Target.msg + 可视化 MarkerArray。
// 本文件只做"消息 ↔ 算法层"的翻译与时钟驱动，不掺滤波逻辑。
class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onArmors(const armor_interfaces::msg::Armors::SharedPtr msg);
  void processArmors(const armor_interfaces::msg::Armors::SharedPtr msg);

  // 缓存最近一帧 /image 作为渲染底图；detector 预热期间（/armors 还没来）由它兜底出图
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);

  // 掉帧外推定时器：TEMP_LOST 期间即使 /armors 停流，也周期推进整车预测并发布
  // predicted=true 的估计（需求① 掉帧/漏检时的强制构造入口）
  void onTimer();

  // 填 Target.msg：整车状态 → 车心位置/速度 + 朝向/转速 + 半径/高差/板数
  void fillTargetMsg(armor_interfaces::msg::Target & msg, bool tracking);
  // 整车 marker：车心球 + N 块预测装甲板 CUBE
  void publishMarker(const armor_interfaces::msg::Target & msg);

  // 需求④ 左上 HUD：整车各状态量实时文本（车心/速度/朝向/转速/半径/板数/匹配…），
  // Kalman 可视化。show_hud_=true 时画。
  void drawHud(cv::Mat & out);

  // —— 需求① 的 2D 可视化：画被跟踪板框（rqt_image_view 看）——
  // 两层叠加：白色线=整车 EKF 预测的**全部**装甲板（Kalman 模型转盘，掉帧外推也画，
  // 即需求① 强行绘制）；绿色框=本帧真正被 EKF 吃掉的"正在追踪板"（matched_armor 的
  // NN 角点贴合框 + 距离文本）；另画整车车心青色十字 + 左上 HUD（show_hud_）。
  // 未锁定（LOST）时按 publish_when_lost_ 决定是发原始帧（标 LOST）还是不发
  void renderAndPublish();

  rclcpp::Subscription<armor_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<armor_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr final_img_pub_;
  rclcpp::TimerBase::SharedPtr predict_timer_;

  std::unique_ptr<Tracker> tracker_;
  std::string frame_id_ = "camera_optical_frame";

  // 停流超时判断：/armors 停流时 Tracker::update 不再被调，TEMP_LOST 的超时
  // 由 timer 用这两个成员补判
  bool was_temp_lost_ = false;
  std::chrono::steady_clock::time_point temp_lost_since_;

  // 渲染底图（最近一帧 /image）
  cv::Mat latest_img_;
  std_msgs::msg::Header latest_img_header_;
  bool has_img_ = false;
  int64_t observation_stamp_ns_ = -1;
  int64_t rendered_stamp_ns_ = -1;
  double max_reproj_error_ = 12.0;

  // 最近一次收到 /armors 的时刻（默认构造 = 纪元，故首帧判定即为"超时未收到"）。
  // onImage 靠它判断 detector 是否在供数：只有断供时才由 /image 兜底渲染，否则同一源帧
  // 会被 onImage + onArmors 渲染两次 → 录制帧率高于源视频 → 成片变慢动作
  std::chrono::steady_clock::time_point last_armors_time_{};

  // 参数
  double max_match_distance_;   // 整车门控：位置门控，m
  double max_match_yaw_diff_;   // 整车门控：板朝向门控，rad
  int armor_num_;               // 非 0 覆盖整车建模板数（实测标定，0=按车牌默认）
  double radius_init_;          // 非 0 覆盖初始半径（实测标定，m）
  double fx_, fy_, cx_, cy_;             // 相机内参（与 detector 标定一致）
  double armor_width_, armor_height_;    // 绘制用的装甲板世界尺寸（m）
  double plate_tilt_deg_ = 15.0;         // RM 装甲板默认上倾角（deg，实测前向板法线 n_y≈-sin15°）
  bool show_hud_;                        // 左上 HUD 开关（需求④ 可视化）
  double future_ms_ = 150.0;             // 需求⑤ 未来外推提前量 ms（白=现在，品红虚线=未来）
  bool show_future_ = true;              // 需求⑤ 画未来板框开关（纯可视化，不进滤波/消息）
  // LOST（含从未锁上）时也发原始帧 + LOST 标注，保证 /tracker/final_img 全程不断流。
  // 关闭则退回旧行为：未锁定时不发布任何帧（rqt 定格、录制器空等超时）
  bool publish_when_lost_ = true;
  bool sign_logged_ = false;             // 法线符号调试日志只打一次
};

}  // namespace task3

#endif  // TRACKER__TRACKER_NODE_HPP_
