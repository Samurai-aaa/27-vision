#ifndef ARMOR_DETECTOR__ARMOR_HPP
#define ARMOR_DETECTOR__ARMOR_HPP

#include <opencv2/core.hpp>

#include <algorithm>
#include <string>

namespace project1 {

enum class Color{
    RED = 0,
    BLUE = 1,
    NONE = 2,
};

enum class ArmorType{
    SMALL,
    BIG,
    INVALID
};

struct Light : public cv::RotatedRect {
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) * 180.0 / CV_PI;
    }

    Color color;
    cv::Point2f top, bottom;
    double length, width;
    float tilt_angle;
};

struct Armor {
    Armor() = default;
    Armor(const Light& l1, const Light& l2){
        if (l1.center.x < l2.center.x){
            left_light = l1, right_light = l2;
        }else{
            left_light = l2, right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2;

    }

    // 灯条
    Light left_light, right_light;
    cv::Point2f center;
    ArmorType type;

    // 数字识别字段（预留，本次不实现）
    cv::Mat number_img;
    std::string number;
    float confidence;
    std::string classification_result;
};

}  // namespace project1

#endif  // ARMOR_DETECTOR__ARMOR_HPP
