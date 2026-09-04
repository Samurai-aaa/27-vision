// 联测用"假相机"：读视频文件按 fps 发布 /image，接口与真实相机驱动一致。
// 上真车时删掉本文件即可，detector 只认 /image，不受影响。
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

namespace task3 {

class VideoPlayerNode : public rclcpp::Node {
public:
    VideoPlayerNode() : Node("video_player") {
        // 参数
        video_path_ = declare_parameter<std::string>("video_path", "");
        fps_ = declare_parameter<double>("fps", 30.0);  // <=0 时自动取视频自身帧率
        loop_ = declare_parameter<bool>("loop", true);
        frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_frame");

        cap_.open(video_path_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", video_path_.c_str());
            throw std::runtime_error("Failed to open video file");
        }

        if (fps_ <= 0.0) fps_ = cap_.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(this->get_logger(), "Playing %s @ %.1f fps (loop=%s)",
                    video_path_.c_str(), fps_, loop_ ? "true" : "false");

        // SensorDataQoS：与 detector 订阅端一致，推理跟不上时自然丢帧（正好测 KF 掉帧逻辑）
        img_pub_ = create_publisher<sensor_msgs::msg::Image>("/image", rclcpp::SensorDataQoS());

        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));
        timer_ = create_wall_timer(period, std::bind(&VideoPlayerNode::timerCallback, this));
    }

private:
    void timerCallback() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            if (loop_) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);  // 本帧跳过，下个周期从头播
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Video finished, shutting down.");
            rclcpp::shutdown();
            return;
        }

        // 时间戳从这打：detector 转发、tracker 算 dt 都以 ROS 时钟为准
        img_msg_.header.stamp = now();
        img_msg_.header.frame_id = frame_id_;
        cv_bridge::CvImage(img_msg_.header, "bgr8", frame).toImageMsg(img_msg_);
        img_pub_->publish(img_msg_);
    }

    // 参数
    std::string video_path_;
    double fps_;
    bool loop_;
    std::string frame_id_;

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Image img_msg_;  // 复用消息，避免每帧分配
};

}  // namespace task3

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<task3::VideoPlayerNode>());
    rclcpp::shutdown();
    return 0;
}
