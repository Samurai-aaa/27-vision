#include "simple_tracker_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>

#include "math_tools.hpp"

namespace task3
{

namespace
{

// 该 number 是否为可跟踪目标："1"~"5" 步兵/英雄、前哨 "O"、哨兵 "G"（与整车一致）。
bool trackable(const std::string & n)
{
  if (n == "O" || n == "G") return true;
  return n.size() == 1 && n[0] >= '1' && n[0] <= '5';
}

// HUD / 日志用：本步状态文本（LOST / 本帧有实测命中 / 无命中纯预测外推）
const char * stepState(bool locked, bool matched)
{
  if (!locked) return "LOST";
  return matched ? "LOCKED" : "EXTRAP";
}

const char * modelName(SimpleTarget::Model m)
{
  return m == SimpleTarget::Model::CV ? "CV" : "CA";
}

}  // namespace

SimpleTrackerNode::SimpleTrackerNode(const rclcpp::NodeOptions & options)
: Node("simple_tracker_node", options)
{
  // 运动模型与过程噪声（需求② 对比实验主调参数）
  const std::string model_str = declare_parameter<std::string>("model", "CV");
  model_ = (model_str == "CA") ? SimpleTarget::Model::CA : SimpleTarget::Model::CV;
  v1_ = declare_parameter<double>("v1", 1.0);

  // 数据关联/状态机
  max_match_distance_ = declare_parameter<double>("max_match_distance", 0.2);
  lost_time_thres_ = declare_parameter<double>("lost_time_thres", 0.3);

  // 渲染用相机内参（与 detector 的 solver 标定一致，1440×1080）
  fx_ = declare_parameter<double>("fx", 2556.2545862166521);
  fy_ = declare_parameter<double>("fy", 2553.5331992802749);
  cx_ = declare_parameter<double>("cx", 705.83803766013978);
  cy_ = declare_parameter<double>("cy", 584.62889512335437);
  armor_width_ = declare_parameter<double>("armor_width", 0.13);
  armor_height_ = declare_parameter<double>("armor_height", 0.055);
  show_hud_ = declare_parameter<bool>("show_hud", true);

  armors_sub_ = create_subscription<armor_interfaces::msg::Armors>(
    "/armors", rclcpp::QoS(10), [this](const armor_interfaces::msg::Armors::SharedPtr msg) {
      onArmors(msg);
    });
  target_pub_ = create_publisher<armor_interfaces::msg::Target>(
    "/simple_tracker/target", rclcpp::QoS(10));
  // 渲染图用 SensorDataQoS（best-effort）：纯观看，绝不被慢 rqt/viewer 反压堵死
  final_img_pub_ =
    create_publisher<sensor_msgs::msg::Image>("/simple_tracker/final_img", rclcpp::SensorDataQoS());

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      onImage(msg);
    });

  // 掉帧全停流的兜底：/armors 长时间不来 → 周期 predict 外推（需求① 掉帧强制构造）
  predict_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { onTimer(); });

  RCLCPP_INFO(get_logger(), "SimpleTrackerNode 就绪(单板普通KF): model=%s v1=%.2f", modelName(model_),
              v1_);
}

void SimpleTrackerNode::onArmors(const armor_interfaces::msg::Armors::SharedPtr msg)
{
  const auto now = std::chrono::steady_clock::now();
  last_frame_time_ = now;
  latest_img_header_ = msg->header;  // 渲染/发布的参考帧头

  const std::vector<ObservedArmor> obs = buildObs(msg);

  if (st_) {
    // 锁定中：每帧先 predict（含无同号板的帧，漏检即纯外推，输出即预测）
    st_->predict(now);
    matched_now_ = false;
    if (const ObservedArmor * cand = pick(obs)) {
      st_->update(*cand);
      matched_now_ = true;
      last_matched_ = *cand;
      last_match_time_ = now;
    }
    // 连续无命中超时 → 回 LOST，等本帧重锁（可能换板/换号）
    if (!matched_now_ &&
        std::chrono::duration<double>(now - last_match_time_).count() > lost_time_thres_) {
      reset();
    }
  }

  // LOST 则尝试从本帧锁一台车号
  if (!st_) {
    if (tryInit(obs, now)) matched_now_ = true;
  }

  publishAndRender(now);

  RCLCPP_DEBUG(get_logger(), "[%s] id=%s 观测=%zu", stepState(st_ != nullptr, matched_now_),
               tracked_number_.c_str(), obs.size());
}

void SimpleTrackerNode::onTimer()
{
  // 只在 /armors 全停流时兜底外推；正常帧（含空帧）由 onArmors 驱动
  if (!st_) return;
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_frame_time_).count() < 0.2) return;

  // 停流超过无命中超时 → 回 LOST 清空（下次 /armors 恢复时重锁）
  if (std::chrono::duration<double>(now - last_match_time_).count() > lost_time_thres_) {
    reset();
    publishAndRender(now);
    RCLCPP_INFO(get_logger(), "无命中超时，回到 LOST");
    return;
  }

  st_->predict(now);
  matched_now_ = false;  // 纯外推帧
  publishAndRender(now);
}

std::vector<ObservedArmor> SimpleTrackerNode::buildObs(
  const armor_interfaces::msg::Armors::SharedPtr msg)
{
  std::vector<ObservedArmor> obs;
  obs.reserve(msg->armors.size());
  for (const auto & m : msg->armors) {
    if (!trackable(m.number)) continue;
    ObservedArmor a;
    a.number = m.number;
    a.xyz << m.pose.position.x, m.pose.position.y, m.pose.position.z;  // 相机系，m
    a.yaw = 0.0;  // 单板 KF 不观测朝向，此字段不用
    a.corners_px = m.corners_px;  // NN 四角点（有观测帧画贴合框用）
    a.rot = Eigen::Quaterniond(m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y,
                               m.pose.orientation.z)
              .toRotationMatrix();  // 外推帧画预测板框的姿态来源
    obs.push_back(a);
  }
  return obs;
}

bool SimpleTrackerNode::tryInit(
  const std::vector<ObservedArmor> & obs, std::chrono::steady_clock::time_point now)
{
  if (obs.empty()) return false;

  // 离相机最近的可跟踪板作为初值（PnP 质量最高）
  const ObservedArmor * closest = &obs.front();
  for (const auto & a : obs)
    if (a.xyz.norm() < closest->xyz.norm()) closest = &a;

  st_ = std::make_unique<SimpleTarget>(*closest, model_, now, v1_);
  tracked_number_ = closest->number;  // 锁定该车号，其后只喂同号板
  matched_now_ = true;
  last_matched_ = *closest;
  last_match_time_ = now;
  RCLCPP_INFO(get_logger(), "锁定车号 %s (%s)", tracked_number_.c_str(), modelName(model_));
  return true;
}

const ObservedArmor * SimpleTrackerNode::pick(const std::vector<ObservedArmor> & obs)
{
  if (!st_ || obs.empty()) return nullptr;

  // 同号候选里取"离 KF 预测板心最近"的一块；超过门控视为串板/换板，宁可漏检外推
  const ObservedArmor * best = nullptr;
  double best_d = max_match_distance_;
  const Eigen::Vector3d pred = st_->armor_xyz();
  for (const auto & a : obs) {
    if (a.number != tracked_number_) continue;  // 只喂锁定车号的板
    const double d = (a.xyz - pred).norm();
    if (d < best_d) {
      best_d = d;
      best = &a;
    }
  }
  return best;
}

void SimpleTrackerNode::reset()
{
  st_.reset();
  tracked_number_.clear();
  matched_now_ = false;
}

void SimpleTrackerNode::onImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_img_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
  latest_img_header_ = msg->header;
  has_img_ = true;
}

void SimpleTrackerNode::publishAndRender(std::chrono::steady_clock::time_point now)
{
  armor_interfaces::msg::Target tmsg;
  tmsg.header = latest_img_header_;
  if (st_) {
    // 单板普通 KF 输出：位置 = 估计板心，速度 = KF 抽出的速度；整车字段(yaw/v_yaw/radius)无意义置 0
    tmsg.tracking = true;
    tmsg.id = tracked_number_;
    tmsg.predicted = !matched_now_;  // 本帧无实测命中 → 掉帧/漏检纯预测帧（需求①）
    const Eigen::Vector3d p = st_->armor_xyz();
    const Eigen::Vector3d v = st_->velocity();
    tmsg.position.x = p[0];
    tmsg.position.y = p[1];
    tmsg.position.z = p[2];
    tmsg.velocity.x = v[0];
    tmsg.velocity.y = v[1];
    tmsg.velocity.z = v[2];
    tmsg.armors_num = 1;
  } else {
    tmsg.tracking = false;  // LOST：清空
    tmsg.id = "";
    tmsg.predicted = false;
    tmsg.armors_num = 0;
  }
  target_pub_->publish(tmsg);

  if (st_) renderAndPublish();
}

void SimpleTrackerNode::renderAndPublish()
{
  if (!has_img_) return;
  if (!st_) return;

  cv::Mat out = latest_img_.clone();

  const auto to_px = [&](const Eigen::Vector3d & p, cv::Point & pt) -> bool {
    if (p[2] < 0.02) return false;
    pt = cv::Point(cvRound(fx_ * p[0] / p[2] + cx_), cvRound(fy_ * p[1] / p[2] + cy_));
    return true;
  };
  // 已知板中心/姿态(R：宽=col0、高=col1)投影竖直板矩形四角；任一角跑到相机后则不画
  const auto plate_quad = [&](const Eigen::Vector3d & c, const Eigen::Matrix3d & R,
                              cv::Point q[4]) -> bool {
    if (c[2] < 0.05) return false;
    const Eigen::Vector3d w = R.col(0), hgt = R.col(1);
    const double hw = armor_width_ / 2.0, hh = armor_height_ / 2.0;
    const Eigen::Vector3d cor[4] = {
      c + hw * w + hh * hgt, c + hw * w - hh * hgt, c - hw * w - hh * hgt, c - hw * w + hh * hgt};
    for (int k = 0; k < 4; k++)
      if (!to_px(cor[k], q[k])) return false;
    return true;
  };

  // ① 无命中（掉帧/漏检）：画"纯预测板框"（紫）——用 KF 外推的板心 + 最近一次命中板的
  //    姿态反投影。这是需求① 的"掉帧强制构造可视化"：看不到实测板时仍画出 KF 认为板在哪。
  const bool matched = matched_now_;
  if (!matched) {
    cv::Point q[4];
    if (plate_quad(st_->armor_xyz(), last_matched_.rot, q)) {
      for (int k = 0; k < 4; k++)
        cv::line(out, q[k], q[(k + 1) % 4], cv::Scalar(255, 0, 255), 2);
      const Eigen::Vector3d p = st_->armor_xyz();
      char text[96];
      snprintf(text, sizeof(text), "%s id=%s 外推 d=%.2fm", modelName(model_),
               tracked_number_.c_str(), p.norm());
      cv::putText(out, text, cv::Point(q[0].x, q[0].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(255, 0, 255), 2);
    }
  }

  // ② 本帧实际命中的板：NN 四角点连"贴合实测框"（绿），无角点时用实测姿态反投影兜底
  if (matched) {
    const ObservedArmor & mm = last_matched_;
    const bool has_corners = std::any_of(mm.corners_px.begin(), mm.corners_px.end(),
                                         [](float v) { return v != 0.0f; });
    cv::Point q[4];
    bool ok = false;
    if (has_corners) {
      for (int k = 0; k < 4; k++)
        q[k] = cv::Point(cvRound(mm.corners_px[2 * k]), cvRound(mm.corners_px[2 * k + 1]));
      ok = true;
    } else {
      ok = plate_quad(mm.xyz, mm.rot, q);
    }
    if (ok) {
      for (int k = 0; k < 4; k++)
        cv::line(out, q[k], q[(k + 1) % 4], cv::Scalar(0, 255, 0), 3);
      char text[96];
      snprintf(text, sizeof(text), "%s id=%s d=%.2fm", modelName(model_),
               tracked_number_.c_str(), mm.xyz.norm());
      cv::putText(out, text, cv::Point(q[0].x, q[0].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 0), 2);
    }
  }

  // KF 估计板心投影十字（青）：有观测/外推都画，直观看到滤波器认为的板心
  {
    cv::Point pc;
    if (to_px(st_->armor_xyz(), pc)) {
      const int rr = 8;
      cv::line(out, cv::Point(pc.x - rr, pc.y), cv::Point(pc.x + rr, pc.y), cv::Scalar(255, 255, 0),
               2);
      cv::line(out, cv::Point(pc.x, pc.y - rr), cv::Point(pc.x, pc.y + rr), cv::Scalar(255, 255, 0),
               2);
    }
  }

  if (show_hud_) drawHud(out);

  final_img_pub_->publish(*cv_bridge::CvImage(latest_img_header_, "bgr8", out).toImageMsg());
}

void SimpleTrackerNode::drawHud(cv::Mat & out)
{
  if (!st_) return;

  const Eigen::Vector3d p = st_->armor_xyz();
  const Eigen::Vector3d v = st_->velocity();

  const cv::Rect hud(10, 10, 500, 175);
  cv::Mat roi = out(hud);
  cv::addWeighted(roi, 0.45, cv::Mat::zeros(roi.size(), CV_8UC3), 0.0, 30.0, roi);

  char line[128];
  int y = 42;
  const auto put = [&](const char * s, const cv::Scalar & col) {
    cv::putText(out, s, cv::Point(18, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, col, 2);
    y += 32;
  };

  // 行1 状态色：本帧有实测命中绿 / 纯外推紫
  snprintf(line, sizeof(line), "id=%s  %s%s", tracked_number_.c_str(),
           stepState(true, matched_now_), matched_now_ ? "" : "  (外推)");
  put(line, matched_now_ ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 255));

  snprintf(line, sizeof(line), "%sKF pos(%+.2f,%+.2f,%+.2f) d=%.2f", modelName(model_), p[0], p[1],
           p[2], p.norm());
  put(line, cv::Scalar(255, 255, 255));
  snprintf(line, sizeof(line), "vel(%+.2f,%+.2f,%+.2f)", v[0], v[1], v[2]);
  put(line, cv::Scalar(255, 255, 255));
  snprintf(line, sizeof(line), "max_match=%.2f  lost=%.2fs", max_match_distance_, lost_time_thres_);
  put(line, cv::Scalar(180, 180, 180));
}

}  // namespace task3

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<task3::SimpleTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
