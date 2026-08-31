#include "armor.hpp"

#include "OpenvinoInfer.h"   // 用 task2::Object 的完整定义

namespace task3 {

Armor armorFromObject(const Object& obj, float sx, float sy) {
    Armor a;

    // 1. landmarks 是 640x640 图上的坐标，各向异性映射回原图
    //    sx = frame.cols / 640, sy = frame.rows / 640，由调用方传入
    for (int k = 0; k < 4; k++)
        a.corners.emplace_back(obj.landmarks[2 * k] * sx, obj.landmarks[2 * k + 1] * sy);

    // 2. type 判定：用 NN 算好的 ratio（= 宽 / 灯条高，小≈2.36，大≈4.18）
    //    阈值 3.0 卡中间。注意各向异性会压缩 ratio 约 25%（原图 1440x1080 时），
    //    3.0 仍能分开小/大，若换分辨率或误判就改用 corners 几何宽高比
    a.type = obj.ratio > 3.0 ? ArmorType::BIG : ArmorType::SMALL;

    // 3. NN 信息透传（solver 只需要用 rvec/tvec，这些留给调试）
    a.label = obj.label;
    a.color = obj.color;
    a.prob = obj.prob;

    return a;
}

}  // namespace task3
