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

    // solvePnPGeneric 的另一解（IPPE 镜像假设），供"把双解都画出来"调试：
    // rvec/tvec = 选中的最优解，rvec_alt/tvec_alt = 未被选中的那一个；
    // 单解路径 / 无另一解时为空 Mat（empty() 为真）
    cv::Mat rvec_alt;   // 另一解旋转向量
    cv::Mat tvec_alt;   // 另一解平移向量（mm）

    double distance = 0.0;   // m
    double yaw = 0.0;        // 度（重投影精修后的 yaw；未触发精修时 == yaw_raw）
    double yaw_raw = 0.0;    // 度（PnP 直接解出的原始 yaw，精修前保留，对照用）
    bool yaw_refined = false;   // 本帧是否触发过"遍历重投影择优"并更新

    // yaw 精修前后的重投影 4 角点误差和（px），量化 yaw 解算是否准确
    double yaw_refine_err0 = 0.0;  // 精修前
    double yaw_refine_err1 = 0.0;  // 精修后（未触发/无改善时 == err0）

    // 来自 NN 的信息，调试用
    int label = -1;    // G,1-5,O,Bs,Bb
    int color = 0;     // 0 红 1 蓝
    float prob = 0.0f;
};

// Object（NN 输出）→ Armor：sx/sy = 原图尺寸 / 640，由调用方传入
Armor armorFromObject(const Object& obj, float sx, float sy);

}  // namespace task2

#endif  // ARMOR_HPP
