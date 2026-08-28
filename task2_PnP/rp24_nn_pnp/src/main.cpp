// 深大 RP24 装甲板检测 · 最小视频验证程序
// 用法: ./rp_detect <视频路径> [detect_color=0] [输出视频路径]
//   detect_color: 0=保留红(滤蓝)  1=保留蓝(滤红)
#include "OpenvinoInfer.h"
#include "solver.hpp"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace cv;
using namespace std;
using namespace task2;

// label 0~8 对应 README: G,1,2,3,4,5,O,Bs,Bb
static const char* LABELS[] = {"G", "1", "2", "3", "4", "5", "O", "Bs", "Bb"};

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "用法: %s <视频路径> [detect_color=0] [输出视频路径]\n", argv[0]);
        fprintf(stderr, "  detect_color: 0=保留红(滤蓝)  1=保留蓝(滤红)\n");
        return -1;
    }
    string video_path = argv[1];
    int detect_color = argc > 2 ? atoi(argv[2]) : 0;
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
    printf("视频 %s  %dx%d  %.1ffps  输入640x640, detect_color=%d\n",
           video_path.c_str(), frame_w, frame_h, fps_in, detect_color);

    VideoWriter writer;
    writer.open(out_path, VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_in, Size(frame_w, frame_h));
    if (writer.isOpened())
        printf("标注视频写入: %s\n", out_path.c_str());
    else
        fprintf(stderr, "警告: 无法写入输出视频 %s\n", out_path.c_str());

    OpenvinoInfer infer("Model/0526.onnx", "CPU");

    Mat frame, img_640;
    int i = 0, n_det_total = 0;
    double acc_ms = 0;
    int64 t0 = getTickCount();

    Solver solver;

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

        if (writer.isOpened()) writer.write(frame);
        if (has_display) {
            imshow("RP24 Detection", frame);
            if (waitKey(1) == 27) break;  // ESC
        }
        i++;
    }

    double total_s = (getTickCount() - t0) / getTickFrequency();
    printf("处理 %d 帧, 共 %.2fs, 平均 %.1f ms/帧, 检测目标总数 %d\n",
           i, total_s, i ? total_s / i * 1000 : 0, n_det_total);
    cap.release();
    writer.release();
    return 0;
}
