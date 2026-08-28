#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/core.hpp>

#include <algorithm>
#include <vector>
#include <string>

namespace task2 {
    
struct Object;

enum class ArmorType{
    SMALL,
    BIG,
    INVALID
};

struct Armor {
    Armor() = default;

    // 输入：4 个角点（原图像素坐标），顺序 TL,BL,BR,TR
    std::vector<cv::Point2f> corners;

    // 由角点几何判定（或外部指定），决定 PnP 选哪组 3D 模型
    ArmorType type = ArmorType::INVALID;

    // PnP 输出（solver 填充）
    cv::Mat rvec;   // 旋转向量
    cv::Mat tvec;   // 平移向量（mm）

    double distance = 0.0;   // m
    double yaw = 0.0;        // 度

    // 来自 NN 的信息，调试用
    int label = -1;    // G,1-5,O,Bs,Bb
    int color = 0;     // 0 红 1 蓝
    float prob = 0.0f;
};

// Object（NN 输出）→ Armor：sx/sy = 原图尺寸 / 640，由调用方传入
Armor armorFromObject(const Object& obj, float sx, float sy);

}  // namespace task2

#endif  // ARMOR_HPP
