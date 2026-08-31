#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/quaternion.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
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
    frame_id_     = declare_parameter<std::string>("frame_id", "camera_optical_frame");

    infer_  = initInfer();
    solver_ = std::make_unique<Solver>();

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
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/image", rclcpp::SensorDataQoS(),
        std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));

};

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    cv::Mat frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    if (frame.empty()) return;

    cv::Mat img640;
    cv::resize(frame, img640, cv::Size(640, 640));
    infer_->infer(img640, detect_color_);

    float sx = (float)frame.cols / 640.0f;
    float sy = (float)frame.rows / 640.0f;
    cv::Point2f img_center(frame.cols / 2.0f, frame.rows / 2.0f);

    armors_msg_.armors.clear();                    // 成员消息复用，必须先 clear
    armors_msg_.header.stamp = img_msg->header.stamp;   // 继承图像时间戳，勿用 now()
    armors_msg_.header.frame_id = frame_id_;

    std::vector<Armor> armors;                     // 内部 Armor（画图/marker 用）
    for (const Object & obj : infer_->tmp_objects) {
        Armor a = armorFromObject(obj, sx, sy);
        if (!solver_->solvePnPGeneric(a)) continue;
        armors.push_back(a);

        armor_interfaces::msg::Armor m;
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
        armors_msg_.armors.push_back(m);
    }
    armors_pub_->publish(armors_msg_);

    if (marker_pub_) publishMarkers(armors);
    if (final_img_pub_) { /* 画框/角点/z轴/文本 → CvImage → final_img_pub_->publish() */ }
};

std::unique_ptr<OpenvinoInfer> DetectorNode::initInfer() {
    std::string model_default =
        ament_index_cpp::get_package_share_directory("detector") + "/Model/0526.onnx";
    std::string model_path = declare_parameter<std::string>("model_path", model_default);
    std::string device     = declare_parameter<std::string>("device", "CPU");
    return std::make_unique<OpenvinoInfer>(model_path, device);
}

void DetectorNode::createDebugPublishers() {
    final_img_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/armor_detector/final_img", 10);
};

void DetectorNode::destroyDebugPublishers() {
    final_img_pub_.reset();
};

void DetectorNode::publishMarkers(const std::vector<Armor> & armors) {
    marker_array_.markers.clear();
    for (size_t i = 0; i < armors.size(); i++) {
        armor_marker_.header.stamp = armors_msg_.header.stamp;
        armor_marker_.header.frame_id = frame_id_;
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
