// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"

namespace project1{

Detector::Detector(
    int bin_thres, Color color, const LightParams& l, const ArmorParams& a)
    : binary_thres(bin_thres), detect_color(color), l(l), a(a)
    {    
    }

std::vector<Armor> Detector::detect(const cv::Mat& input) {
    binary_img = preprocessImage(input);
    lights_ = findLights(input, binary_img);
    armors_ = matchLights(lights_);

    return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat& input) {
    cv::Mat img_Gray;
    // 注意：OpenCV 默认读入的是 BGR 图像，必须用 COLOR_BGR2GRAY；
    // 若误用 COLOR_RGB2GRAY，红色通道权重会从 0.299 变成 0.114，红方灯条变暗、容易漏检
    cv::cvtColor(input, img_Gray, cv::COLOR_BGR2GRAY);

    cv::Mat img_Binary;
    cv::threshold(img_Gray, img_Binary, binary_thres, 255, cv::THRESH_BINARY);

    return img_Binary;
}

std::vector<Light> Detector::findLights(const cv::Mat& rgb_img, const cv::Mat& binary_img)
{
    // 1. 从二值图提取所有轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Light> lights;

    for (const auto & contour : contours) {
        // minAreaRect 计算最小外接旋转矩形至少需要 5 个点
        if (contour.size() < 5) continue;

        // 2. 每个轮廓拟合出旋转矩形，构造灯条结构体
        auto r_rect = cv::minAreaRect(contour);
        auto light = Light(r_rect);

        // 3. 几何筛选（长宽比 + 倾角）
        if (!isLight(light)) continue;

        // 4. 颜色判断：用原图在灯条包围盒内统计 R/B 通道亮度
        //    灯条包围盒可能越出图像边界，先校验再取 ROI，避免 at() 越界崩溃
        auto rect = light.boundingRect();
        if (
          0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 0 <= rect.y &&
          0 <= rect.height && rect.y + rect.height <= rgb_img.rows) {
            int sum_r = 0, sum_b = 0;
            auto roi = rgb_img(rect);

            // 遍历 ROI，只累加真正在轮廓内部的点
            for (int i = 0; i < roi.rows; i++) {
                for (int j = 0; j < roi.cols; j++) {
                    if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                        sum_r += roi.at<cv::Vec3b>(i, j)[2];   // BGR：索引 2 是 R 通道
                        sum_b += roi.at<cv::Vec3b>(i, j)[0];   // BGR：索引 0 是 B 通道
                    }
                }
            }
            light.color = sum_r > sum_b ? Color::RED : Color::BLUE;
            lights.emplace_back(light);
        }
    }

    return lights;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light>& lights)
{
    std::vector<Armor> armors;

    // 遍历所有灯条两两组合（j 从 i+1 开始，避免重复配对）
    for (size_t i = 0; i < lights.size(); i++) {
        for (size_t j = i + 1; j < lights.size(); j++) {
            const auto & light_1 = lights[i];
            const auto & light_2 = lights[j];

            // 目标颜色过滤：detect_color 为 NONE 时红蓝都参与配对（区分颜色）
            if (detect_color != Color::NONE &&
                (light_1.color != detect_color || light_2.color != detect_color)) {
                continue;
            }

            // 两灯条之间不能含有其他灯条，防止把相邻装甲板的灯条错配成一块
            if (iscontainLights(light_1, light_2, lights)) continue;

            // 几何判据 + 类型判定 + 置信度，通过才构成装甲板
            float confidence = 0;
            auto type = isArmor(light_1, light_2, confidence);
            if (type != ArmorType::INVALID) {
                auto armor = Armor(light_1, light_2);
                armor.type = type;
                armor.confidence = confidence;
                armors.emplace_back(armor);
            }
        }
    }

    // 重叠去重：共用灯条的装甲板（相邻灯条对不同配对产生的矛盾结果），
    // 保留置信度高的，删掉置信度低的，防止一条灯条同时属于多个装甲板导致错误匹配
    for (size_t i = 0; i < armors.size(); i++) {
        for (size_t j = i + 1; j < armors.size(); j++) {
            if (shareLight(armors[i], armors[j])) {
                if (armors[i].confidence >= armors[j].confidence) {
                    armors[j].confidence = -1.0f;   // 标记删除置信度低者
                } else {
                    armors[i].confidence = -1.0f;
                }
            }
        }
    }
    armors.erase(
      std::remove_if(
        armors.begin(), armors.end(), [](const Armor & a) { return a.confidence < 0.0f; }),
      armors.end());

    return armors;
}

bool Detector::isLight(const Light& light)
{
    // 灯条短边 / 长边比例：细长灯条应远小于 1
    float ratio = light.width / light.length;
    bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

    // 灯条与垂直方向的夹角
    bool angle_ok = light.tilt_angle < l.max_angle;

    return ratio_ok && angle_ok;
}

bool Detector::iscontainLights(
  const Light& l1, const Light& l2, const std::vector<Light>& lights)
{
    // 取两条灯条四个角点的包围盒
    auto points = std::vector<cv::Point2f>{l1.top, l1.bottom, l2.top, l2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    // 若包围盒内还含有第三条灯条，说明中间隔了别的灯条，视为错误匹配
    for (const auto & test_light : lights) {
        if (test_light.center == l1.center || test_light.center == l2.center) continue;
        if (
          bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
          bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

ArmorType Detector::isArmor(const Light& l1, const Light& l2, float& confidence)
{
    // 判据1：两灯条长度比（短 / 长），两根灯条应长短接近
    float light_length_ratio =
      l1.length < l2.length ? l1.length / l2.length : l2.length / l1.length;
    bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

    // 判据2：中心距（以平均灯长为单位），分小/大装甲两个区间
    float avg_light_length = (l1.length + l2.length) / 2;
    float center_distance = cv::norm(l1.center - l2.center) / avg_light_length;
    bool center_distance_ok =
      (a.min_small_center_distance <= center_distance && center_distance < a.max_small_center_distance) ||
      (a.min_large_center_distance <= center_distance && center_distance < a.max_large_center_distance);

    // 判据3：两灯条中心连线与水平方向的夹角（用 atan2 避免 x 分量除零）
    cv::Point2f diff = l1.center - l2.center;
    float angle = std::atan2(std::abs(diff.y), std::abs(diff.x)) * 180.0 / CV_PI;
    bool angle_ok = angle < a.max_angle;

    // 判据4：两灯条中心高度差（y 坐标差）不能过大，防止上下错位的灯条错误配对
    // 角度判据只看 dy/dx 的比例，dx 很大时 dy 也会被放行；这里用平均灯长归一化补一个绝对高度约束
    bool height_ok = std::abs(diff.y) / avg_light_length < a.max_center_height_diff;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok && height_ok;
    if (!is_armor) {
        confidence = 0.0f;
        return ArmorType::INVALID;
    }

    // 中心距大于大装甲区间下限则判为大装甲板，否则小装甲板
    ArmorType type = center_distance > a.min_large_center_distance ? ArmorType::BIG : ArmorType::SMALL;

    // 置信度：三个几何量越接近理想值越高，范围约 (0.4, 1.0]
    //   - 长度比越接近 1 越高（权重 0.6）
    //   - 中心连线越接近水平越高（权重 0.2）
    //   - 中心距越接近所在区间中点越高（权重 0.2）
    float dist_ideal = (type == ArmorType::BIG)
                         ? (a.min_large_center_distance + a.max_large_center_distance) / 2
                         : (a.min_small_center_distance + a.max_small_center_distance) / 2;
    float dist_half_width = (type == ArmorType::BIG)
                              ? (a.max_large_center_distance - a.min_large_center_distance) / 2
                              : (a.max_small_center_distance - a.min_small_center_distance) / 2;
    float dist_score = 1.0f - std::abs(center_distance - dist_ideal) / dist_half_width;

    confidence =
      0.6f * light_length_ratio + 0.2f * (1.0f - angle / a.max_angle) + 0.2f * dist_score;
    return type;
}

bool Detector::shareLight(const Armor& a1, const Armor& a2)
{
    // 两块装甲板是否共用灯条（同一根灯条的 center 是同一浮点拷贝，== 成立）
    return a1.left_light.center == a2.left_light.center ||
           a1.left_light.center == a2.right_light.center ||
           a1.right_light.center == a2.left_light.center ||
           a1.right_light.center == a2.right_light.center;
}

void Detector::drawResults(cv::Mat& img)
{
    // 灯条：上下角点画圆，中心连线按颜色着色（红=红、蓝=蓝）
    for (const auto & light : lights_) {
        cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
        cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
        auto line_color = light.color == Color::RED ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::line(img, light.top, light.bottom, line_color, 1);
    }

    // 装甲板：画交叉线 + 类型/颜色标注
    for (const auto & armor : armors_) {
        cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
        cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
    }
}

}