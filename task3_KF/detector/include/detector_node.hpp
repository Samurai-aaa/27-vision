#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP

// STD
#include <memory>
#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// project
#include "armor.hpp"
#include "OpenvinoInfer.h"
#include "solver.hpp"

// messages
#include <armor_interfaces/msg/armor.hpp>
#include <armor_interfaces/msg/armors.hpp>


namespace task3 {

class DetectorNode : public rclcpp::Node {
public:
    explicit DetectorNode(const rclcpp::NodeOptions & options);

private:
    // /image 回调：NN 检测 → PnP → 填 Armors 并发布
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    // 读模型路径/设备参数并构造推理器
    std::unique_ptr<OpenvinoInfer> initInfer();

    // debug 参数开关：控制标注图 / marker 的发布（运行中 ros2 param set 即时切换）
    void createDebugPublishers();
    void destroyDebugPublishers();
    void publishMarkers(const std::vector<Armor> & armors);

    // NN 推理 + PnP 解算
    std::unique_ptr<OpenvinoInfer> infer_;
    std::unique_ptr<Solver> solver_;

    // 检测结果发布
    armor_interfaces::msg::Armors armors_msg_;
    rclcpp::Publisher<armor_interfaces::msg::Armors>::SharedPtr armors_pub_;

    // debug：标注图（binary_img / number_img 是传统灯条流程的中间图，NN 检测没有）
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr final_img_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr debug_param_cb_;

    // RViz 3D 标记（装甲板位姿；tracker 阶段扩展为整车状态）
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // 图像订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // 参数缓存
    std::string frame_id_;
    int detect_color_ = 0;
};

}  // namespace task3

#endif  // DETECTOR_NODE_HPP
