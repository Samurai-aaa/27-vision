#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/quaternion.hpp>

// STD
#include <cstdio>
#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// project
#include "armor.hpp"
#include "detector_node.hpp"

namespace task3 {

// label 0~8: G,1,2,3,4,5,O,Bs,Bb
static const char* kLabels[] = {"G", "1", "2", "3", "4", "5", "O", "Bs", "Bb"};

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options) {
    RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

    detect_color_ = declare_parameter<int>("detect_color", 0);

    const double fx = declare_parameter<double>("fx", 0.0);
    const double fy = declare_parameter<double>("fy", 0.0);
    const double cx = declare_parameter<double>("cx", 0.0);
    const double cy = declare_parameter<double>("cy", 0.0);
    if (fx <= 0.0 || fy <= 0.0) {
        throw std::invalid_argument("camera fx/fy must be positive; check config/camera.yaml");
    }
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);

    infer_ = initInfer();
    infer_->conf_threshold = declare_parameter<double>("confidence_threshold", 0.35);
    infer_->nms_threshold = declare_parameter<double>("nms_threshold", 0.45);
    solver_ = std::make_unique<Solver>(camera_matrix);

    // debug 开关 + 运行中切换
    bool debug = declare_parameter<bool>("debug", true);
    if (debug) createDebugPublishers();
    debug_param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto & p : params) {
                if (p.get_name() == "debug") {
                    p.as_bool() ? createDebugPublishers() : destroyDebugPublishers();
                }
            }
            return result;
        });

    armors_pub_ = create_publisher<armor_interfaces::msg::Armors>("/armors", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/armor_detector/marker_array", 10);
    // /image 必须与 video_player 发布端同为 reliable，否则 QoS 不匹配收不到帧。
    // 原来这里是 SensorDataQoS（best-effort），正是全链路丢 8% 的来源，详见
    // video_player_node.cpp 里的实测说明。depth 20 ≈ 0.67s 的历史，足够吸收调度抖动。
    rclcpp::QoS img_qos = rclcpp::SensorDataQoS();
    img_qos.keep_last(20).reliable();
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/image", img_qos,
        std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));

};

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    // 零拷贝取图：原实现 toCvCopy 会把整帧（1440x1080x3 = 4.6MB）memcpy 一遍，紧接着又
    // resize 到 640 丢给模型——这次拷贝对推理没有任何贡献。这里改为在消息缓冲区上直接建
    // 只读视图。注：本回调实测只占 9.7ms（推理 9.1 + resize 0.6），不是瓶颈（链路丢帧的
    // 真因是 best-effort QoS，见 video_player_node.cpp）；省掉这 4.6MB 拷贝只是去掉
    // 每秒 140MB 的无谓搬运，不是"修好了丢帧"。
    // 只在编码/步长不是紧凑 bgr8 时回退到 toCvCopy（保持原语义）。
    cv::Mat frame;
    if (img_msg->encoding == "bgr8" &&
        img_msg->step == static_cast<uint32_t>(img_msg->width) * 3u) {
        frame = cv::Mat(static_cast<int>(img_msg->height), static_cast<int>(img_msg->width),
                        CV_8UC3, const_cast<unsigned char *>(img_msg->data.data()));
    } else {
        frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    }
    if (frame.empty()) return;

    cv::Mat img640;
    cv::resize(frame, img640, cv::Size(640, 640));
    infer_->infer(img640, detect_color_);

    float sx = (float)frame.cols / 640.0f;
    float sy = (float)frame.rows / 640.0f;
    cv::Point2f img_center(frame.cols / 2.0f, frame.rows / 2.0f);

    armors_msg_.armors.clear();                    // 成员消息复用，必须先 clear
    armors_msg_.header.stamp = img_msg->header.stamp;   // 继承图像时间戳，勿用 now()
    armors_msg_.header.frame_id = img_msg->header.frame_id;

    std::vector<Armor> armors;                     // 内部 Armor（画图/marker 用）
    for (const Object & obj : infer_->tmp_objects) {
        Armor a = armorFromObject(obj, sx, sy);
        if (!solver_->solvePnPGeneric(a)) continue;
        armors.push_back(a);

        armor_interfaces::msg::Armor m;
        m.confidence = obj.prob;
        m.class_margin = obj.class_margin;
        m.color_uncertain = obj.color_uncertain;
        m.number = (a.label >= 0 && a.label < 9) ? kLabels[a.label] : "?";
        m.type = (a.type == ArmorType::BIG) ? "big" : "small";
        m.distance_to_image_center =
            (float)cv::norm(0.5f * (a.corners[0] + a.corners[2]) - img_center);
        m.pose.position.x = a.tvec.at<double>(0) / 1000.0;   // mm → m
        m.pose.position.y = a.tvec.at<double>(1) / 1000.0;
        m.pose.position.z = a.tvec.at<double>(2) / 1000.0;
        cv::Mat R;
        cv::Rodrigues(a.rvec, R);
        cv::Quatd q = cv::Quatd::createFromRotMat(R);
        m.pose.orientation.w = q.w;  m.pose.orientation.x = q.x;
        m.pose.orientation.y = q.y;  m.pose.orientation.z = q.z;
        m.reproj_err = a.reproj_err;
        for (int k = 0; k < 4 && k < static_cast<int>(a.corners.size()); k++) {
          m.corners_px[2 * k] = a.corners[k].x;
          m.corners_px[2 * k + 1] = a.corners[k].y;
        }
        armors_msg_.armors.push_back(m);
    }
    armors_pub_->publish(armors_msg_);

    if (marker_pub_) publishMarkers(armors);

    // debug 标注图：四点框 + 角点 + 类别/置信度 + 距离/yaw/重投影误差 + z 轴
    if (final_img_pub_) {
        // frame 现在可能是消息缓冲区的只读视图，标注要画在自己的副本上
        cv::Mat canvas = frame.clone();
        for (const Armor & a : armors) {
            cv::Scalar c = (a.color == 1) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);  // 1=红 0=蓝
            for (int k = 0; k < 4; k++)
                cv::line(canvas, a.corners[k], a.corners[(k + 1) % 4], c, 2);
            for (const auto & p : a.corners) cv::circle(canvas, p, 3, c, -1);

            const char * label = (a.label >= 0 && a.label < 9) ? kLabels[a.label] : "?";
            char text[64];
            snprintf(text, sizeof(text), "%s %.2f", label, a.prob);
            cv::putText(canvas, text, cv::Point2f(a.corners[0].x, a.corners[0].y - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, c, 2);

            char txt[64];
            snprintf(txt, sizeof(txt), "%.2fm yaw=%.1f err=%.0f",
                     a.distance, a.yaw, a.reproj_err);
            cv::putText(canvas, txt, cv::Point2f(a.corners[0].x, a.corners[0].y - 28),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            solver_->drawZAxis(canvas, a);
        }
        final_img_pub_->publish(*cv_bridge::CvImage(armors_msg_.header, "bgr8", canvas).toImageMsg());
    }
};

std::unique_ptr<OpenvinoInfer> DetectorNode::initInfer() {
    std::string model_default =
        ament_index_cpp::get_package_share_directory("detector") + "/Model/0526.onnx";
    std::string model_path = declare_parameter<std::string>("model_path", model_default);
    std::string device     = declare_parameter<std::string>("device", "CPU");
    return std::make_unique<OpenvinoInfer>(model_path, device);
}

void DetectorNode::createDebugPublishers() {
    // 调试标注图用 SensorDataQoS（best-effort）：纯观看用途，宁可丢帧也绝不能
    // 让慢的 rqt/viewer 反压阻塞推理链（reliable + 深度 10 会在慢消费者下写满队列
    // 使 imageCallback 卡在 publish，表现为画面"中途冻住"）
    final_img_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/armor_detector/final_img", rclcpp::SensorDataQoS());
};

void DetectorNode::destroyDebugPublishers() {
    final_img_pub_.reset();
};

void DetectorNode::publishMarkers(const std::vector<Armor> & armors) {
    marker_array_.markers.clear();
    for (size_t i = 0; i < armors.size(); i++) {
        armor_marker_.header.stamp = armors_msg_.header.stamp;
        armor_marker_.header.frame_id = armors_msg_.header.frame_id;
        armor_marker_.ns = "armor";  armor_marker_.id = i;
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        armor_marker_.pose = armors_msg_.armors[i].pose;
        armor_marker_.scale.x = 0.13;  armor_marker_.scale.y = 0.025;  armor_marker_.scale.z = 0.055;
        armor_marker_.color.a = 0.5;  armor_marker_.color.g = 1.0;
        marker_array_.markers.push_back(armor_marker_);
    }
    marker_pub_->publish(marker_array_);
}

}   // namespace task3

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<task3::DetectorNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
