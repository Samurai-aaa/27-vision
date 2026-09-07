#include "tracker_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace task3
{

namespace
{

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

// 四元数 → 板朝向角 yaw = atan2(R(2,2), R(0,2))：板法线（板局部 z 轴，重建为 R.col(2)）
// 在相机 x-z 水平面（相机 x 右、z 前）的方位角，前向可见板 ∈(-π,0)。与 target 整车
// 模型"绕竖直轴在 x-z 水平面公转"的观测口径一致。
// 旧版 atan2(R(1,2),R(0,2)) 把法线投到相机像平面 x-y（竖直面）→ 旋转中双稳态(≈0/≈-π)
// 不可观，已弃用。
double armorYawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
  Eigen::Matrix3d R = eq.toRotationMatrix();
  return std::atan2(R(2, 2), R(0, 2));
}

}  // namespace

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: Node("tracker_node", options)
{
  max_match_distance_ = declare_parameter<double>("max_match_distance", 0.2);
  max_match_yaw_diff_ = declare_parameter<double>("max_match_yaw_diff", 1.0);
  armor_num_ = declare_parameter<int>("armor_num", 0);       // 0 = 按车牌默认（哨兵/步兵 4 板）
  radius_init_ = declare_parameter<double>("radius_init", 0.0);  // 0 = 按车牌默认
  tracker_ = std::make_unique<Tracker>(max_match_distance_, max_match_yaw_diff_);
  tracker_->tracking_thres = declare_parameter<int>("tracking_thres", 5);
  tracker_->lost_time_thres = declare_parameter<double>("lost_time_thres", 0.3);
  tracker_->armor_num_override = armor_num_;
  tracker_->radius_override = radius_init_;

  // 渲染用相机内参（与 detector 的 solver 标定一致，1440×1080）
  fx_ = declare_parameter<double>("fx", 2556.2545862166521);
  fy_ = declare_parameter<double>("fy", 2553.5331992802749);
  cx_ = declare_parameter<double>("cx", 705.83803766013978);
  cy_ = declare_parameter<double>("cy", 584.62889512335437);
  armor_width_ = declare_parameter<double>("armor_width", 0.13);     // 绘制板宽（小装甲）
  armor_height_ = declare_parameter<double>("armor_height", 0.055);  // 板高
  show_hud_ = declare_parameter<bool>("show_hud", true);             // 左上 HUD（需求④）
  // 需求⑤：未来状态外推可视化（只画框，不写进滤波/消息）——渲染层多叠一层
  // "current + future_ms 后"的预测板虚线框，作瞄准提前量预览。可运行时动态调。
  future_ms_ = declare_parameter<double>("future_ms", 150.0);        // 提前量 ms
  show_future_ = declare_parameter<bool>("show_future", true);       // 画未来板框开关

  armors_sub_ = create_subscription<armor_interfaces::msg::Armors>(
    "/armors", rclcpp::QoS(10), [this](const armor_interfaces::msg::Armors::SharedPtr msg) {
      onArmors(msg);
    });
  target_pub_ = create_publisher<armor_interfaces::msg::Target>("/tracker/target", rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/tracker/marker", rclcpp::QoS(10));

  // 渲染底图：缓存最近一帧 /image，把整车预测框画上去发 /tracker/final_img
  // （需求① 掉帧/漏检时"强行绘制"，用 rqt_image_view 看）
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      onImage(msg);
    });
  // 渲染图用 SensorDataQoS（best-effort）：纯观看用途，宁可丢帧也绝不能因慢的
  // rqt/viewer 反压而阻塞整车跟踪/渲染（reliable + 深度 10 会在慢消费者下写满
  // 队列，publish 卡死 → 画面"中途冻住"）
  final_img_pub_ =
    create_publisher<sensor_msgs::msg::Image>("/tracker/final_img", rclcpp::SensorDataQoS());

  // 掉帧外推：TEMP_LOST 时 100ms tick 一次 predict 并发布 predicted=true
  predict_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this]() { onTimer(); });

  RCLCPP_INFO(
    get_logger(),
    "TrackerNode 就绪(整车EKF): max_match_distance=%.2f max_match_yaw_diff=%.2f "
    "armor_num=%d radius_init=%.3f",
    max_match_distance_, max_match_yaw_diff_, armor_num_, radius_init_);
}

void TrackerNode::onArmors(const armor_interfaces::msg::Armors::SharedPtr msg)
{
  // 掉帧期间到达的空 /armors 也要推进状态机：predict 后进入 TEMP_LOST
  std::vector<ObservedArmor> obs;
  obs.reserve(msg->armors.size());
  for (const auto & m : msg->armors) {
    if (!trackable(m.number)) continue;
    // 首帧首个可跟踪板打一次法线三分量（板局部 z 轴在相机系的重建），供核对
    // "前向板 n_z<0、n_y≈0（法线基本水平指相机）"这一整套几何的关键假设
    if (!sign_logged_) {
      sign_logged_ = true;
      const Eigen::Quaterniond qeq(
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z);
      const Eigen::Matrix3d R0 = qeq.toRotationMatrix();
      RCLCPP_INFO(
        get_logger(), "板法线观测 相机系(n_x,n_y,n_z)=(%+.3f,%+.3f,%+.3f)  前向板应 n_z<0、n_y≈0",
        R0(0, 2), R0(1, 2), R0(2, 2));
    }
    ObservedArmor a;
    a.number = m.number;
    a.xyz << m.pose.position.x, m.pose.position.y, m.pose.position.z;  // 相机系 m
    a.yaw = armorYawFromQuat(m.pose.orientation);  // 板法线在 x-z 水平面方位角（整车观测口径）
    a.corners_px = m.corners_px;  // NN 四角点（detector 新版本带），画贴合框用
    a.rot = Eigen::Quaterniond(m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y,
                               m.pose.orientation.z)
              .toRotationMatrix();
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

  renderAndPublish();  // 需求①：整车预测板框画到图上（TEMP_LOST 时即"强行绘制"）

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
    tracker_->target.reset();  // 回 LOST：丢弃整车滤波器，等下一帧观测重新 init
    tracker_->matched_armor.reset();
    tracker_->last_obs.reset();
    was_temp_lost_ = false;
    armor_interfaces::msg::Target target_msg;  // 清空 marker 用 tracking=false
    target_msg.tracking = false;
    target_pub_->publish(target_msg);
    publishMarker(target_msg);
    RCLCPP_INFO(get_logger(), "TEMP_LOST 超时，回到 LOST");
    return;
  }

  tracker_->target->predict(now);

  armor_interfaces::msg::Target target_msg;
  target_msg.header.stamp = rclcpp::Clock().now();
  target_msg.header.frame_id = frame_id_;
  fillTargetMsg(target_msg, true);
  target_pub_->publish(target_msg);
  publishMarker(target_msg);
  renderAndPublish();  // /armors 断流时的整车外推也要在图上画出来
}

void TrackerNode::fillTargetMsg(armor_interfaces::msg::Target & msg, bool tracking)
{
  msg.tracking = tracking;
  msg.id = tracker_->tracked_number;
  msg.predicted = (tracker_->state == State::TEMP_LOST);  // 掉帧/漏检外推构造帧
  if (!tracking || !tracker_->target) {  // LOST 态 target 已释放，勿读
    msg.armors_num = 0;
    return;
  }

  // 整车状态输出：车心位置/速度 + 朝向/转速 + 半径/高差/板数
  const auto & x = tracker_->target->ekf_x();
  const Eigen::Vector3d c = tracker_->target->center();
  const Eigen::Vector3d v = tracker_->target->velocity();
  msg.position.x = c[0];
  msg.position.y = c[1];
  msg.position.z = c[2];
  msg.velocity.x = v[0];
  msg.velocity.y = v[1];
  msg.velocity.z = v[2];
  msg.yaw = x[6];        // 车体朝向（= 主轴位板朝向角）
  msg.v_yaw = x[7];      // 自转角速度
  msg.radius_1 = x[8];   // 板心到旋转轴距离
  msg.armors_num = tracker_->target->armor_num();
  // 4 板车有长短轴 r+l；前哨 3 板同半径，radius_2 = radius_1
  msg.radius_2 = (msg.armors_num == 4) ? x[8] + x[9] : x[8];
  msg.dz = x[10];        // 对角/高低差板的高度偏移（4 板车才生效）
}

void TrackerNode::publishMarker(const armor_interfaces::msg::Target & msg)
{
  visualization_msgs::msg::MarkerArray arr;
  if (!msg.tracking || !tracker_->target) {
    marker_pub_->publish(arr);  // 空数组=清除
    return;
  }

  const auto & x = tracker_->target->ekf_x();

  // 车心：绿色球（整车估计的中心，位置/速度都在它身上）
  visualization_msgs::msg::Marker center;
  center.header.stamp = msg.header.stamp;
  center.header.frame_id = msg.header.frame_id;
  center.ns = "center";
  center.type = visualization_msgs::msg::Marker::SPHERE;
  center.action = visualization_msgs::msg::Marker::ADD;
  center.scale.x = center.scale.y = center.scale.z = 0.08;
  center.color.r = 0.0f; center.color.g = 1.0f; center.color.b = 0.0f; center.color.a = 1.0f;
  center.pose.position.x = x[0];
  center.pose.position.y = x[2];
  center.pose.position.z = x[4];
  arr.markers.push_back(center);

  // N 块预测装甲板（整车建模的装甲板集合可视化）。新几何：整车绕竖直轴在相机 x-z
  // 水平面公转、板面竖直、法线径向朝外。用 CUBE 局部坐标架表达板朝向 ——
  //   局部 x = 宽向 t  =(−sinφ, 0, cosφ)（水平切向，随旋转扫过 x-z 圆环）
  //   局部 y = 高向 u  =(0, -1, 0)       （竖直，相机 y 向下故顶在上为 -y）
  //   局部 z = 法线 n̂  =(cosφ, 0, sinφ)  （径向朝外，前向板 sinφ<0）
  // scale：x=板宽、y=板高、z=板厚。
  const auto xyza_list = tracker_->target->armor_xyza_list();
  for (size_t i = 0; i < xyza_list.size(); i++) {
    visualization_msgs::msg::Marker plate;
    plate.header.stamp = msg.header.stamp;
    plate.header.frame_id = msg.header.frame_id;
    plate.ns = "armors";
    plate.id = static_cast<int>(i);
    plate.type = visualization_msgs::msg::Marker::CUBE;
    plate.action = visualization_msgs::msg::Marker::ADD;
    plate.scale.x = armor_width_;            // 板宽沿局部 x（水平切向）
    plate.scale.y = armor_height_;           // 板高沿局部 y（竖直）
    plate.scale.z = 0.03;                    // 板厚（法线向，薄片）
    plate.color.b = 1.0f; plate.color.a = 0.6f;
    plate.pose.position.x = xyza_list[i][0];
    plate.pose.position.y = xyza_list[i][1];
    plate.pose.position.z = xyza_list[i][2];
    // R = [t | u | n̂]，右旋正交 → 转四元数
    const double phi = xyza_list[i][3];
    const double cf = std::cos(phi), sf = std::sin(phi);
    Eigen::Matrix3d Rm;
    Rm.col(0) = Eigen::Vector3d{-sf, 0, cf};
    Rm.col(1) = Eigen::Vector3d{0, -1, 0};
    Rm.col(2) = Eigen::Vector3d{cf, 0, sf};
    const Eigen::Quaterniond q(Rm);
    plate.pose.orientation.w = q.w();
    plate.pose.orientation.x = q.x();
    plate.pose.orientation.y = q.y();
    plate.pose.orientation.z = q.z();
    arr.markers.push_back(plate);
  }

  marker_pub_->publish(arr);
}

void TrackerNode::onImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_img_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
  latest_img_header_ = msg->header;
  has_img_ = true;
}

void TrackerNode::renderAndPublish()
{
  if (!has_img_ || tracker_->state == State::LOST) return;
  if (tracker_->tracked_number.empty()) return;

  cv::Mat out = latest_img_.clone();

  // 画什么，三层叠加：
  // ① 白色——整车 EKF 预测的**全部**装甲板（Kalman 模型框）：只要整车 target 在跟，就
  //    把 armor_xyza_list 各板按水平公转方向反投影成白色竖直矩形，整车转盘一目了然；
  //    掉帧/漏检（TEMP_LOST）时无实测框，白框即整车外推可视化（需求① 强行绘制，随
  //    自转相位绕车背收窄/冒出，不再单独画橙虚线外推框）。
  // ② 绿色——本帧真正被整车 EKF 数据关联吃掉的"正在追踪的装甲板"（matched_armor）：
  //    NN 四角点连"贴合实测框"（绿粗线 + 距离文本），与白框对照看模型对真值的贴合。
  // ③ 品红虚线——需求⑤ 未来外推板框：把当前估计按确定性转移外推 future_ms_ 毫秒后
  //    那一帧的整车预测板（armor_xyza_list_at，只读不碰滤波）。白=现在模型在哪，
  //    品红虚线=150ms 后模型认为会转到哪（瞄准提前量预览）。
  const bool matched = tracker_->matched_armor.has_value();

  // 相机系 3D 点 → 图像像素（z<0.02 视为跑到相机背后，返回 false 不画）
  const auto to_px = [&](const Eigen::Vector3d & p, cv::Point & pt) -> bool {
    if (p[2] < 0.02) return false;
    pt = cv::Point(cvRound(fx_ * p[0] / p[2] + cx_), cvRound(fy_ * p[1] / p[2] + cy_));
    return true;
  };
  // 板心 c、相位 phi 的竖直板矩形四角 → 图像。板宽向 t=(−sinφ,0,cosφ)（水平切向）、
  // 高向 u=(0,−1,0)（竖直，相机 y 向下故顶在 −y）。任一角跑到相机后则不画。
  const auto plate_quad = [&](const Eigen::Vector3d & c, double phi, cv::Point q[4]) -> bool {
    if (c[2] < 0.05) return false;
    const double cf = std::cos(phi), sf = std::sin(phi);
    const Eigen::Vector3d t{-sf, 0, cf};
    const Eigen::Vector3d u{0, -1, 0};
    const double hw = armor_width_ / 2.0, hh = armor_height_ / 2.0;
    const Eigen::Vector3d cor[4] = {
      c + hw * t + hh * u, c + hw * t - hh * u,
      c - hw * t - hh * u, c - hw * t + hh * u};
    for (int k = 0; k < 4; k++)
      if (!to_px(cor[k], q[k])) return false;
    return true;
  };

  // ① 整车预测板（白色线条，四个装甲板的整车转盘模型）
  if (tracker_->target) {
    const auto xyza_list = tracker_->target->armor_xyza_list();
    for (const auto & xyza : xyza_list) {
      cv::Point q[4];
      if (!plate_quad(xyza.head(3), xyza[3], q)) continue;
      for (int k = 0; k < 4; k++)
        cv::line(out, q[k], q[(k + 1) % 4], cv::Scalar(255, 255, 255), 1);
    }
  }

  // ③ 需求⑤ 未来外推板框（品红虚线）：整车 EKF 外推 future_ms_ 后那一帧的预测板。
  //    画法与 ① 同构（全板、竖直矩形），但线画虚线 + 品红，一眼区分"现在/未来"两圈
  //    转盘；只在最近一块未来板上挂一次 "t+xxxms" 标注。掉帧/实测帧都叠这层。
  if (tracker_->target && show_future_) {
    // 沿每条边按"实/空各 ~6px"分段（画虚线，与 ①/② 的实线语义区分）
    const auto dashed_edge = [&](cv::Point a, cv::Point b, const cv::Scalar & col) {
      const cv::Point d = b - a;
      const double len = std::sqrt(double(d.x) * d.x + double(d.y) * d.y);
      if (len < 4.0) { cv::line(out, a, b, col, 1); return; }
      const int n = std::max(1, int(len / 12.0));
      for (int k = 0; k < n; k++) {
        const double t0 = double(2 * k) / (2 * n), t1 = double(2 * k + 1) / (2 * n);
        cv::line(out, cv::Point(cvRound(a.x + t0 * d.x), cvRound(a.y + t0 * d.y)),
                 cv::Point(cvRound(a.x + t1 * d.x), cvRound(a.y + t1 * d.y)), col, 1);
      }
    };
    const auto fut = tracker_->target->armor_xyza_list_at(future_ms_ / 1000.0);
    double best_d = 1e18;  // 最近一块未来板（挂时间标注）
    cv::Point best_p;
    bool any = false;
    const cv::Scalar fut_col(255, 0, 255);  // BGR 品红
    for (const auto & xyza : fut) {
      cv::Point q[4];
      if (!plate_quad(xyza.head(3), xyza[3], q)) continue;
      for (int k = 0; k < 4; k++) dashed_edge(q[k], q[(k + 1) % 4], fut_col);
      const double d = xyza.head(3).norm();
      if (d < best_d) { best_d = d; best_p = q[0]; any = true; }
    }
    if (any) {
      char ft[32];
      snprintf(ft, sizeof(ft), "t+%dms", cvRound(future_ms_));
      cv::putText(out, ft, cv::Point(best_p.x, best_p.y - 6), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                  fut_col, 1);
    }
  }

  // ② 正在追踪的实测板（绿色粗框 + 距离文本）
  if (matched) {
    const ObservedArmor & mm = *tracker_->matched_armor;
    const bool has_corners = std::any_of(mm.corners_px.begin(), mm.corners_px.end(),
                                         [](float v) { return v != 0.0f; });
    cv::Point q[4];
    bool ok = false;
    if (has_corners) {
      for (int k = 0; k < 4; k++)
        q[k] = cv::Point(cvRound(mm.corners_px[2 * k]), cvRound(mm.corners_px[2 * k + 1]));
      ok = true;
    } else {
      // 兜底（detector 没带角点时）：用实测 xyz + 姿态 R 投影竖直板矩形
      const Eigen::Matrix3d & R = mm.rot;
      const Eigen::Vector3d w = R.col(0), hgt = R.col(1);
      const double hw = armor_width_ / 2.0, hh = armor_height_ / 2.0;
      const Eigen::Vector3d cor[4] = {
        mm.xyz + hw * w + hh * hgt, mm.xyz + hw * w - hh * hgt,
        mm.xyz - hw * w - hh * hgt, mm.xyz - hw * w + hh * hgt};
      ok = true;
      for (int k = 0; k < 4; k++)
        if (!to_px(cor[k], q[k])) { ok = false; break; }
    }
    if (ok) {
      for (int k = 0; k < 4; k++)
        cv::line(out, q[k], q[(k + 1) % 4], cv::Scalar(0, 255, 0), 3);
      char text[96];
      snprintf(text, sizeof(text), "id=%s d=%.2fm", tracker_->tracked_number.c_str(),
               mm.xyz.norm());
      cv::putText(out, text, cv::Point(q[0].x, q[0].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 0), 2);
    }
  }

  // —— 整车车心投影十字：Kalman 估计的旋转中心（需求④ "画出整车状态"）——
  // 车心不随单板圆周抖，随整车平移/转向平滑移动；TEMP_LOST 外推时落在预测处。
  if (tracker_->target) {
    const Eigen::Vector3d cc = tracker_->target->center();
    cv::Point pc;
    if (to_px(cc, pc)) {
      const int rr = 10;
      cv::line(out, cv::Point(pc.x - rr, pc.y), cv::Point(pc.x + rr, pc.y),
               cv::Scalar(255, 255, 0), 2);
      cv::line(out, cv::Point(pc.x, pc.y - rr), cv::Point(pc.x, pc.y + rr),
               cv::Scalar(255, 255, 0), 2);
    }
  }

  if (show_hud_) drawHud(out);  // 需求④：左上整车状态 HUD

  final_img_pub_->publish(*cv_bridge::CvImage(latest_img_header_, "bgr8", out).toImageMsg());
}

void TrackerNode::drawHud(cv::Mat & out)
{
  if (!tracker_->target) return;  // LOST 已早退，这里只防御

  // 需求④ 左上整车状态 HUD：把 Target/EKF 状态量实时可视化（Kalman 窗口）。
  // 状态口径：pos=车心位置（相机系 m）、vel=车心速度、yaw=板 0 公转相位（法线
  // 水平方位）、r=短半径 / r2=r+l(长半径 4 板车)、N=板数、dz=id1/3 板高度偏置。
  const auto & x = tracker_->target->ekf_x();
  const Eigen::Vector3d c = tracker_->target->center();
  const Eigen::Vector3d v = tracker_->target->velocity();
  const int n = tracker_->target->armor_num();
  const bool matched = tracker_->matched_armor.has_value();

  // 左上半透明底衬（读白字）
  const cv::Rect hud(10, 10, 520, 235);
  cv::Mat roi = out(hud);
  cv::addWeighted(roi, 0.45, cv::Mat::zeros(roi.size(), CV_8UC3), 0.0, 30.0, roi);

  char line[160];
  int y = 42;
  const auto put = [&](const char * s, const cv::Scalar & col) {
    cv::putText(out, s, cv::Point(18, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, col, 2);
    y += 32;
  };

  // 行1 标题色随状态：绿=本帧有实测板命中 / 亮灰=纯整车外推(无绿实测框可画)
  snprintf(line, sizeof(line), "id=%s  %s%s", tracker_->tracked_number.c_str(),
           stateName(tracker_->state).c_str(),
           tracker_->state == State::TEMP_LOST ? "  (外推)" : "");
  put(line, matched ? cv::Scalar(0, 255, 0) : cv::Scalar(235, 235, 235));

  snprintf(line, sizeof(line), "pos(%+.2f,%+.2f,%+.2f) d=%.2f", c[0], c[1], c[2], c.norm());
  put(line, cv::Scalar(255, 255, 255));
  snprintf(line, sizeof(line), "vel(%+.2f,%+.2f,%+.2f)", v[0], v[1], v[2]);
  put(line, cv::Scalar(255, 255, 255));
  snprintf(line, sizeof(line), "yaw %+.2f  v_yaw %+.2f", x[6], x[7]);
  put(line, cv::Scalar(255, 255, 255));
  if (n == 4)
    snprintf(line, sizeof(line), "r=%.3f  N=%d  r2=%.3f  dz=%+.3f", x[8], n, x[8] + x[9], x[10]);
  else
    snprintf(line, sizeof(line), "r=%.3f  N=%d  dz=%+.3f", x[8], n, x[10]);
  put(line, cv::Scalar(255, 255, 255));
  snprintf(line, sizeof(line), "match=%s  last_id=%d  upd=%d", matched ? "Y" : "N",
           tracker_->target->last_id, tracker_->target->update_count());
  put(line, cv::Scalar(255, 255, 255));
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
