// 深大 RP24 装甲板检测 · 最小视频验证程序
// 用法: ./rp_detect <视频路径> [detect_color=0] [输出视频路径] [pnp_method=0]
//   detect_color: 0=保留红(滤蓝)  1=保留蓝(滤红)
//   pnp_method:  0=双解(solvePnPGeneric-IPPE)  1=单解(solvePnP-IPPE)
//               2=SQPNP  3=EPNP  4=ITERATIVE  5=P3P  6=AP3P
//   双解时绿轴=选中的解、橙轴=未选中的另一解(镜像假设)，两种假设都画出来对照
#include "OpenvinoInfer.h"
#include "solver.hpp"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <algorithm>
#include <vector>

using namespace cv;
using namespace std;
using namespace task2;

// label 0~8 对应 README: G,1,2,3,4,5,O,Bs,Bb
static const char* LABELS[] = {"G", "1", "2", "3", "4", "5", "O", "Bs", "Bb"};

// pnp_method 选择表：索引即终端参数值（与 PnPMethod 枚举顺序一致）。
// name 用于启动打印/usage；tag 是画在画面左上角的"视频标题"（当前方法）。
struct MethodEntry { const char* name; const char* tag; PnPMethod m; };
static const MethodEntry METHODS[] = {
    {"双解 solvePnPGeneric-IPPE (绿=选中解, 橙=另一解)", "IPPE 双解", PnPMethod::DUAL_IPPE},
    {"单解 solvePnP-IPPE",                              "IPPE 单解", PnPMethod::IPPE},
    {"solvePnP-SQPNP",                                   "SQPNP",   PnPMethod::SQPNP},
    {"solvePnP-EPNP",                                    "EPNP",    PnPMethod::EPNP},
    {"solvePnP-ITERATIVE",                               "ITERATIVE", PnPMethod::ITERATIVE},
    {"solvePnP-P3P",                                     "P3P",     PnPMethod::P3P},
    {"solvePnP-AP3P",                                    "AP3P",    PnPMethod::AP3P},
};
static const int N_METHODS = (int)(sizeof(METHODS) / sizeof(METHODS[0]));

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "用法: %s <视频路径> [detect_color=0] [输出视频路径] [pnp_method=0]\n", argv[0]);
        fprintf(stderr, "  detect_color: 0=保留红(滤蓝)  1=保留蓝(滤红)\n");
        fprintf(stderr, "  pnp_method:  0=双解(solvePnPGeneric-IPPE)\n");
        for (int i = 1; i < N_METHODS; i++)
            fprintf(stderr, "              %d=%s\n", i, METHODS[i].name);
        return -1;
    }
    string video_path = argv[1];
    int detect_color = argc > 2 ? atoi(argv[2]) : 0;
    int pnp_method = argc > 4 ? atoi(argv[4]) : 0;
    if (pnp_method < 0 || pnp_method >= N_METHODS) {
        fprintf(stderr, "pnp_method=%d 越界(有效 0~%d)。用法见上。\n", pnp_method, N_METHODS - 1);
        return -1;
    }
    string out_path = argc > 3
        ? argv[3]
        : video_path.substr(0, video_path.find_last_of('.')) + "_rp_out.avi";
    bool has_display = getenv("DISPLAY") != nullptr;

    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        fprintf(stderr, "无法打开视频: %s\n", video_path.c_str());
        return -1;
    }
    int frame_w = (int)cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_h = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
    double fps_in = cap.get(CAP_PROP_FPS);
    printf("视频 %s  %dx%d  %.1ffps  输入640x640, detect_color=%d, pnp_method=%s\n",
           video_path.c_str(), frame_w, frame_h, fps_in, detect_color,
           METHODS[pnp_method].name);

    VideoWriter writer;
    writer.open(out_path, VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_in, Size(frame_w, frame_h));
    if (writer.isOpened())
        printf("标注视频写入: %s\n", out_path.c_str());
    else
        fprintf(stderr, "警告: 无法写入输出视频 %s\n", out_path.c_str());

    OpenvinoInfer infer("Model/0526.onnx", "CPU");

    Mat frame, img_640;
    int i = 0, n_det_total = 0, n_solved_total = 0, n_yaw_refined_total = 0;
    double acc_ref_e0 = 0, acc_ref_e1 = 0, acc_dyaw = 0;  // 精修帧误差/yaw 变化累计
    double acc_e1_all = 0;                                // 全部解出块精修后误差累计
    std::vector<double> e0_all;                           // 全部解出块精修前误差(px)分布
    double acc_ms = 0;
    int64 t0 = getTickCount();

    Solver solver;
    solver.setMethod(METHODS[pnp_method].m);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 输入必须 640x640 BGR（infer 内部不 resize）
        resize(frame, img_640, Size(640, 640));

        int64 ta = getTickCount();
        infer.infer(img_640, detect_color);
        int64 tb = getTickCount();
        acc_ms += (tb - ta) / getTickFrequency() * 1000.0;
        n_det_total += (int)infer.tmp_objects.size();

        // 各向异性映射回原图: 宽/高分别按原图比例缩放
        float sx = (float)frame.cols / 640.0f;
        float sy = (float)frame.rows / 640.0f;

        for (const Object& obj : infer.tmp_objects) {
            Point2f pts[4];
            for (int k = 0; k < 4; k++)
                pts[k] = Point2f(obj.landmarks[2 * k] * sx, obj.landmarks[2 * k + 1] * sy);
            Scalar c = (obj.color == 1) ? Scalar(0, 0, 255) : Scalar(255, 0, 0);  // 1=红 0=蓝
            for (int k = 0; k < 4; k++) line(frame, pts[k], pts[(k + 1) % 4], c, 2);
            for (int k = 0; k < 4; k++) circle(frame, pts[k], 3, c, -1);
            const char* label = (obj.label >= 0 && obj.label < 9) ? LABELS[obj.label] : "?";
            char text[64];
            snprintf(text, sizeof(text), "%s %.2f", label, obj.prob);
            putText(frame, text, Point2f(pts[0].x, pts[0].y - 8), FONT_HERSHEY_SIMPLEX, 0.7, c, 2);

            Armor a = armorFromObject(obj, sx, sy);
            if (solver.solvePose(a)) {
                n_solved_total++;
                acc_e1_all += a.yaw_refine_err1;       // 全部解出块精修后误差累计
                e0_all.push_back(a.yaw_refine_err0);   // 全部解出块精修前误差，供分布统计
                if (a.yaw_refined) {
                    n_yaw_refined_total++;
                    acc_ref_e0 += a.yaw_refine_err0;
                    acc_ref_e1 += a.yaw_refine_err1;
                    acc_dyaw   += std::fabs(a.yaw - a.yaw_raw);
                }
                // 先画另一解（solvePnPGeneric 未选中的镜像假设，橙色），再画选中的解（绿）
                if (!a.rvec_alt.empty())
                    solver.drawZAxis(frame, a.rvec_alt, a.tvec_alt, Scalar(0, 165, 255));
                solver.drawZAxis(frame, a);
                char txt[64];
                snprintf(txt, sizeof(txt), "%.2fm yaw=%.1f", a.distance, a.yaw);
                putText(frame, txt, Point2f(pts[0].x, pts[0].y - 28),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
            }
        }

        char info[128];
        snprintf(info, sizeof(info), "frame %d  dets=%zu  avg=%.1fms", i + 1,
                 infer.tmp_objects.size(), (i > 0) ? acc_ms / (i + 1) : 0);
        putText(frame, info, Point(10, 28), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
        // 视频标题：左上角第二行标注当前 PnP 方法（录制后看视频即可知用哪个方法）
        putText(frame, METHODS[pnp_method].tag, Point(10, 60), FONT_HERSHEY_SIMPLEX,
                1.0, Scalar(0, 255, 255), 2);

        if (writer.isOpened()) writer.write(frame);
        if (has_display) {
            imshow("RP24 Detection", frame);
            if (waitKey(1) == 27) break;  // ESC
        }
        i++;
    }

    double total_s = (getTickCount() - t0) / getTickFrequency();
    printf("处理 %d 帧, 共 %.2fs, 平均 %.1f ms/帧, 检测目标总数 %d, PnP 成功 %d (%.0f%%), "
           "yaw 遍历精修触发 %d (%.1f%%)\n",
           i, total_s, i ? total_s / i * 1000 : 0, n_det_total, n_solved_total,
           n_det_total ? 100.0 * n_solved_total / n_det_total : 0.0,
           n_yaw_refined_total,
           n_solved_total ? 100.0 * n_yaw_refined_total / n_solved_total : 0.0);
    if (n_yaw_refined_total > 0)
        printf("精修统计: 均值 重投影误差 %.2f→%.2f px(每块 4 角点误差和), yaw 修正 |Δ|=%.2f°\n",
               acc_ref_e0 / n_yaw_refined_total, acc_ref_e1 / n_yaw_refined_total,
               acc_dyaw / n_yaw_refined_total);
    if (!e0_all.empty()) {
        std::sort(e0_all.begin(), e0_all.end());
        auto pct = [&](double q) { return e0_all[(size_t)(q * (e0_all.size() - 1))]; };
        double mean_e0 = 0;
        for (double e : e0_all) mean_e0 += e;
        mean_e0 /= e0_all.size();
        double mean_e1 = n_solved_total ? acc_e1_all / n_solved_total : 0.0;
        printf("全部解出块重投影误差(每块4角点误差和): 均值=%.2f px, 精修后均值=%.2f px, "
               "分布 p50=%.2f p90=%.2f p99=%.2f max=%.2f px\n",
               mean_e0, mean_e1, pct(0.50), pct(0.90), pct(0.99), e0_all.back());
    }
    cap.release();
    writer.release();
    return 0;
}
