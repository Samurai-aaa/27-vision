// 装甲板识别主程序（纯 OpenCV）：读视频 -> 检测 -> 绘制 -> 显示（不保存视频）
// 用法: ./armor_detector [video_path] [config_path] [--debug]
//   所有参数（含视频路径）默认从 config/detector_params.txt 读取，命令行可覆盖
#include <opencv2/opencv.hpp>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"

namespace {
std::map<std::string, std::string> loadParams(const std::string & path)
{
    std::map<std::string, std::string> params;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[main] 警告: 无法打开配置文件 " << path << "，使用默认参数\n";
        return params;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) key.pop_back();
        while (!key.empty() && (key.front() == ' ' || key.front() == '\t')) key.erase(key.begin());

        std::string value = line.substr(eq + 1);
        while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) value.pop_back();
        while (!value.empty() && (value.front() == ' ' || value.front() == '\t'))
            value.erase(value.begin());
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
            value = value.substr(1, value.size() - 2);   // 去掉双引号
        }
        params[key] = value;
    }
    return params;
}

// 从配置取数字参数（key 不存在或解析失败时返回默认值）
double getNum(const std::map<std::string, std::string> & p, const std::string & key, double def)
{
    auto it = p.find(key);
    if (it == p.end()) return def;
    try {
        return std::stod(it->second);
    } catch (...) {
        return def;
    }
}
}  // namespace

int main(int argc, char ** argv)
{
    // 1. 解析命令行：--debug 开关；其余参数依次为 视频路径、配置文件路径
    std::string video_path;
    std::string config_path = "config/detector_params.txt";
    bool debug = false;
    std::vector<std::string> args;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--debug") {
            debug = true;
        } else {
            args.push_back(argv[i]);
        }
    }
    if (args.size() >= 1) video_path = args[0];
    if (args.size() >= 2) config_path = args[1];

    // 2. 加载配置：视频路径（命令行没给时用配置里的）、检测颜色、二值化阈值、灯条/装甲板参数
    auto p = loadParams(config_path);
    if (video_path.empty()) {
        video_path = p.count("video_path") ? p["video_path"] : "video_input/装甲板.avi";
    }
    const int binary_thres = static_cast<int>(getNum(p, "binary_thres", 160));

    project1::Detector::LightParams light_params{
      getNum(p, "light_min_ratio", 0.1),
      getNum(p, "light_max_ratio", 0.4),
      getNum(p, "light_max_angle", 40.0)};
    project1::Detector::ArmorParams armor_params{
      getNum(p, "armor_min_light_ratio", 0.7),
      getNum(p, "armor_min_small_center_distance", 0.8),
      getNum(p, "armor_max_small_center_distance", 3.2),
      getNum(p, "armor_min_large_center_distance", 3.2),
      getNum(p, "armor_max_large_center_distance", 5.5),
      getNum(p, "armor_max_angle", 35.0),
      getNum(p, "armor_max_center_height_diff", 0.5)};

    // 检测目标颜色：配置文件里 detect_color=0/1/2 对应 红/蓝/红蓝都检测
    const int color_val = static_cast<int>(getNum(p, "detect_color", 2));
    project1::Color detect_color =
      color_val == 0 ? project1::Color::RED : color_val == 1 ? project1::Color::BLUE
                                                             : project1::Color::NONE;

    // 3. 打开视频
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "[main] 无法打开视频: " << video_path << "\n";
        return -1;
    }
    const double fps = cap.get(cv::CAP_PROP_FPS);   // 视频帧率，用于按原速播放

    project1::Detector detector(binary_thres, detect_color, light_params, armor_params);
    std::cout << "[main] 视频 " << video_path << " | 二值化阈值 " << binary_thres
              << " | 检测颜色 " << color_val << " (0红 1蓝 2红蓝) | 参数文件 " << config_path
              << "\n";

    // 4. 逐帧检测并显示（按 q 退出；--debug 额外显示二值图并打印统计）
    cv::Mat frame;
    int frame_count = 0;
    while (cap.read(frame)) {
        auto armors = detector.detect(frame);
        detector.drawResults(frame);

        cv::putText(
          frame, "frame " + std::to_string(frame_count) + "  armors " +
                   std::to_string(armors.size()),
          cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

        if (debug) {
            cv::imshow("binary (debug)", detector.binary_img);
            printf(
              "frame %d: lights=%zu armors=%zu", frame_count, detector.lights().size(),
              armors.size());
            for (const auto & a : armors) {
                printf(
                  " [%s conf=%.2f]", a.type == project1::ArmorType::BIG ? "BIG" : "SMALL",
                  a.confidence);
            }
            printf("\n");
        }

        frame_count++;
        cv::imshow("armor_detector", frame);
        int delay = fps > 0 ? static_cast<int>(1000.0 / fps) : 33;
        if (cv::waitKey(delay) == 'q') break;
    }

    std::cout << "[main] 处理完成，共 " << frame_count << " 帧\n";
    cv::destroyAllWindows();
    return 0;
}
