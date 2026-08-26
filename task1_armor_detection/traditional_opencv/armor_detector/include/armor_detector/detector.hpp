#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace project1 {

class Detector{
public:
    struct LightParams
    {   
        // 宽/高
        double min_ratio;
        double max_ratio;
        // 与纵向夹角
        double max_angle;
    };
    
    struct ArmorParams
    {
        double min_light_ratio;
        // 灯条对距离
        double min_small_center_distance;
        double max_small_center_distance;
        double min_large_center_distance;
        double max_large_center_distance;
        // 最大角度
        double max_angle;
        // 两根灯条中心高度差上限（以平均灯长为单位），过大说明上下错位，取消匹配
        double max_center_height_diff;
    };
    Detector(int bin_thres, Color detect_color, const LightParams& l, const ArmorParams& a);

    std::vector<Armor> detect(const cv::Mat& input);

    cv::Mat preprocessImage(const cv::Mat& input);
    std::vector<Light> findLights(const cv::Mat& rbg_img, const cv::Mat& binary_img);
    std::vector<Armor> matchLights(const std::vector<Light>& lights);

    // 调试用：debug 模式下访问检测出的灯条
    const std::vector<Light>& lights() const { return lights_; }
    void drawResults(cv::Mat& img);

    int binary_thres;
    Color detect_color;
    LightParams l;
    ArmorParams a;

    // Debug
    cv::Mat binary_img;

private:
    bool isLight(const Light& possible_light);
    bool iscontainLights(const Light& l1, const Light& l2, const std::vector<Light>& lights);
    ArmorType isArmor(const Light& l1, const Light& l2, float& confidence);
    bool shareLight(const Armor& a1, const Armor& a2);

    std::vector<Light> lights_;
    std::vector<Armor> armors_;
};

}   // namespace project1

#endif  // ARMOR_DETECTOR__DETECTOR_HPP