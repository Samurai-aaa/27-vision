// 联测用"假相机"：读视频文件按 fps 发布 /image，接口与真实相机驱动一致。
// 上真车时删掉本文件即可，detector 只认 /image，不受影响。
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <chrono>
#include <cstdlib>
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

        // 路径支持 ~ 开头（config yaml 里写成 ~/ 更通用），展开成 $HOME 再打开
        if (!video_path_.empty() && video_path_.front() == '~') {
            if (const char * home = std::getenv("HOME")) {
                video_path_ = std::string(home) + video_path_.substr(1);
            }
        }

        cap_.open(video_path_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", video_path_.c_str());
            throw std::runtime_error("Failed to open video file");
        }

        if (fps_ <= 0.0) fps_ = cap_.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(this->get_logger(), "Playing %s @ %.1f fps (loop=%s)",
                    video_path_.c_str(), fps_, loop_ ? "true" : "false");

        // /image 用 reliable —— 这是整条链路上唯一能保证"一帧不丢"的开关。
        // 实测（blu.avi，3301 帧 @30Hz，1440x1080x3 = 4.6MB/帧）：
        //   best-effort 下被动订阅者只收到 ~3020 帧（丢 8%），detector 实际只拿到 27.5Hz，
        //   /armors、渲染、录像全部按这个缩水的帧率走 → 成片比源视频短 8%。
        //   丢帧既不来自算力（detector 单帧回调仅 9.7ms，只占 33ms 周期的 29%），
        //   也不来自队列深度（depth 5→60 无变化），更不是内核丢包（UDP 的 RcvbufErrors
        //   全程为 0，4.6MB 走的是 Fast DDS 共享内存通道）—— 是 best-effort 语义本身：
        //   读端只要瞬时落后，整帧就被丢弃。改 reliable 后实测 3301/3301 全收到，
        //   发布端仍是 30.00Hz（没有被反压拖慢）。
        // 代价：reliable 写端在历史写满时会阻塞 publish，所以**用 rqt 看 /image 且跟不上
        // 会反压拖慢整个播放**。链路自身的消费者（detector/tracker 回调都在 10ms 量级）
        // 不存在这个问题；纯观看请看 /tracker/final_img（仍是 best-effort，慢 viewer
        // 不会波及算法链）。
        rclcpp::QoS img_qos(rclcpp::KeepLast(20));
        img_qos.reliable().durability_volatile();
        img_pub_ = create_publisher<sensor_msgs::msg::Image>("/image", img_qos);

        // 周期用纳秒精度：原来 (int)(1000.0/30.0) = 33ms，实际按 30.30fps 播，比源视频快 1%
        // （3301 帧的片子 108.96s 就播完，而不是 110.03s），录出来的时长也跟着偏短
        auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / fps_));
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
