// rp_yaw_study：yaw 一维优化研究小工具（主流程入口）。
//
// 需求④：在 PnP(双解 IPPE)基础上固定 xyz，把 yaw 作为唯一优化变量，构造重投影
// 误差最小化问题。本程序对整段视频逐块采样，跑 4 个研究：
//   A. 画代价曲线：完整 yaw 范围是否单峰/多个局部极小；IPPE 双候选在曲线上的
//      位置与代价（固定 t），分析能否仅靠代价函数分辨双解
//   B. 方法选型 + 耗时：grid/golden/brent/LM/粗扫+brent 对比（需≥3 种）
//   C. 近正对（|psi|≤8）精修会不会在正负 psi 间跳变
// 算法层在 yawstudy 模块（include/yawstudy.hpp + src/yawstudy.cpp）。
//
// 用法: ./rp_yaw_study <视频路径> [detect_color=0] [输出目录=yaw_study_out] [max_frames=0=全部]
//   (Model/0526.onnx 相对路径,须在 rp24_nn_pnp/ 目录下运行)
#include "OpenvinoInfer.h"
#include "solver.hpp"
#include "yawstudy.hpp"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
using namespace task2;
using namespace task2::yawstudy;

static void makeDir(const string& p) {
    string cmd = "mkdir -p " + p;
    system(cmd.c_str());
}

// "正面候选"：离正对转角 |psi|<=75（板面朝相机）且解算干净。
// 模型 +z 朝外、正对相机时法线指向相机(-z)，故用 psi（不是裸 yaw）判断朝向。
static bool validSel(const Sample& s) {
    return s.hasAlt && s.rmse0 < 3 && s.dist > 0.5 && s.dist < 9 && std::fabs(s.psi0) <= 75;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "用法: %s <视频路径> [detect_color=0] [输出目录=yaw_study_out] [max_frames=0=全部]\n", argv[0]);
        return -1;
    }
    string video_path = argv[1];
    int detect_color = argc > 2 ? atoi(argv[2]) : 0;
    string outdir = argc > 3 ? argv[3] : "yaw_study_out";
    int max_frames = argc > 4 ? atoi(argv[4]) : 0;
    makeDir(outdir);

    VideoCapture cap(video_path);
    if (!cap.isOpened()) { fprintf(stderr, "无法打开视频: %s\n", video_path.c_str()); return -1; }
    int fw = (int)cap.get(CAP_PROP_FRAME_WIDTH), fh = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
    printf("视频 %s %dx%d\n", video_path.c_str(), fw, fh);

    OpenvinoInfer infer("Model/0526.onnx", "CPU");
    Solver solver;
    solver.setMethod(PnPMethod::DUAL_IPPE);
    YawRefineConfig rc = solver.yawRefineConfig();  // 关掉 1.2.0 后处理,取原始 PnP yaw
    rc.enable = false;
    solver.setYawRefineConfig(rc);

    // ---- 采集：每块板(双解 IPPE 选中解) → 一个 Sample ----
    std::vector<Sample> S;
    S.reserve(5000);
    Mat frame, img640;
    int fidx = 0;
    float sx = (float)fw / 640.0f, sy = (float)fh / 640.0f;
    while (true) {
        cap >> frame;
        if (frame.empty() || (max_frames > 0 && fidx >= max_frames)) break;
        resize(frame, img640, Size(640, 640));
        infer.infer(img640, detect_color);
        for (const Object& obj : infer.tmp_objects) {
            Armor a = armorFromObject(obj, sx, sy);
            if (a.type == ArmorType::INVALID) continue;
            if (!solver.solvePose(a)) continue;
            Sample s;
            if (!makeSample(a, fidx, s)) continue;
            S.push_back(s);
        }
        fidx++;
    }
    cap.release();

    // ---- 朝向诊断：两解的 R22(板法线 z) 符号 —— 印证"选中解是否物理朝前" ----
    int nAlt = 0; for (auto& s : S) if (s.hasAlt) nAlt++;
    int cSelP = 0, cAltP = 0, cBothP = 0, cBothN = 0;
    for (auto& s : S) {
        bool sp = s.r0[8] > 0, ap = s.hasAlt && s.rA[8] > 0;
        if (sp) cSelP++; if (ap) cAltP++;
        if (sp && ap) cBothP++;
        if (!sp && !ap) cBothN++;
    }
    printf("采样 %d 个板(双解 %d) | R22>0: 选中解 %d, 另一解 %d | 两解都>0 %d, 都<0 %d\n",
           (int)S.size(), nAlt, cSelP, cAltP, cBothP, cBothN);
    if (S.empty()) { fprintf(stderr, "无样本\n"); return -1; }
    {   // 调试:打印最"对正"(|R22| 最接近 1)样本的双解姿态,核对 IPPE 解的朝向结构
        int bi = -1; double bv = -1;
        for (size_t i = 0; i < S.size(); i++) { double v = std::fabs(S[i].r0[8]); if (v > bv) { bv = v; bi = (int)i; } }
        if (bi >= 0 && S[bi].hasAlt) {
            const Sample& p = S[bi];
            printf("[dbg] frame %d yaw0=%.1f | R0:\n", p.frame, p.yaw0);
            for (int r = 0; r < 3; r++) printf("    %.3f %.3f %.3f\n", p.r0[3*r], p.r0[3*r+1], p.r0[3*r+2]);
            printf("    t0=(%.0f,%.0f,%.0f)mm q0=(%.0f,%.0f)\n", p.t0[0], p.t0[1], p.t0[2], p.qx[0], p.qy[0]);
            printf("    yawA=%.1f | RA:\n", p.yawA);
            for (int r = 0; r < 3; r++) printf("    %.3f %.3f %.3f\n", p.rA[3*r], p.rA[3*r+1], p.rA[3*r+2]);
        }
    }
    int nFront = 0, nHead = 0;
    for (auto& s : S) { if (std::fabs(s.psi0) <= 75) nFront++; if (std::fabs(s.psi0) <= 8) nHead++; }
    printf("正面候选(|psi|<=75) %d 个 | 近正对(|psi|<=8) %d 个\n", nFront, nHead);

    // ---- 代表样本：最近正对 / 转过 ~15/30/50° / 重投影最差 ----
    std::vector<int> reps;
    auto add = [&](int i) { if (i >= 0 && find(reps.begin(), reps.end(), i) == reps.end()) reps.push_back(i); };
    { int b = -1; double mn = 1e300;
      for (size_t i = 0; i < S.size(); i++) if (validSel(S[i]) && std::fabs(S[i].psi0) < mn) { mn = std::fabs(S[i].psi0); b = (int)i; }
      add(b); }
    for (double tg : { 15.0, 30.0, 50.0 }) {
        int b = -1; double bd = 1e300;
        for (size_t i = 0; i < S.size(); i++) if (validSel(S[i])) {
            double sc = std::fabs(std::fabs(S[i].psi0) - tg);
            if (sc < bd) { bd = sc; b = (int)i; } }
        add(b);
    }
    { int b = -1; double bw = -1;
      for (size_t i = 0; i < S.size(); i++) if (validSel(S[i]) && S[i].rmse0 > bw) { bw = S[i].rmse0; b = (int)i; }
      add(b); }

    // ---- A. 完整范围代价曲线：单峰?/局部极小?/双候选在曲线上的位置与代价 ----
    printf("\n===== A. 完整 yaw 范围代价曲线(-180..180 @0.5deg) =====\n");
    for (int idx = 0; idx < (int)reps.size(); idx++) {
        const Sample& s = S[reps[idx]];
        Curve c = scanCurve(s, -180, 180, 0.5);
        auto lm = localMinima(c.sq);
        double fMin = 1e300; for (double v : c.sq) if (v < fMin) fMin = v;
        int nDeep = 0; for (int i : lm) if (c.sq[i] < fMin * 10) nDeep++;
        // 另一候选在 Δ 域的位置 = 两解方位差; 该处固定 t 的代价是否也低?
        double dAlt = s.hasAlt ? wrapDeg(s.yawA - s.yaw0) : 0;
        double cAlt = s.hasAlt ? costSq(s, dAlt) : 1e300;
        double zoomRad = std::fabs(s.psi0) < 12 ? 30 : 45;
        printf("rep%d: yaw0=%+.2f (正对偏差 psi0=%+.1f, mirror psiA=%+.1f) dist=%.2fm rmse0=%.2fpx | "
               "局部极小 %d 个(<10*fmin 深谷 %d) | mirror@Δ=%+.1f 固定t cost=%.3g (mirror 自身rmse=%.2fpx)\n",
               idx, s.yaw0, s.psi0, s.psiA, s.dist, s.rmse0, (int)lm.size(), nDeep,
               dAlt, cAlt, s.rmseA);
        saveCurveFigure(s, idx, outdir, c, dAlt, cAlt, zoomRad);
    }

    // ---- B. 优化方法选型 + 耗时对比（域 Δ∈[-90,90]，200 块） ----
    printf("\n===== B. 优化方法耗时对比(域 Δ∈[-90,90],逐块) =====\n");
    std::vector<Sample> bench;
    { int cnt = 0; for (auto& s : S) if (validSel(s) && s.dist > 0.8 && cnt < 200) { bench.push_back(s); cnt++; } }
    struct Mres { std::string name; double ev, ms, dDiff, fBest, rmse; };
    std::vector<Mres> mr;

    auto run = [&](const char* nm, std::function<OptResult(const Sample&)> fn) {
        double t0 = cv::getTickCount();
        double sumE = 0, sumD = 0, sumF = 0;
        for (auto& s : bench) { OptResult r = fn(s); sumE += r.evals; sumD += std::fabs(r.delta); sumF += r.bestF; }
        Mres m; m.name = nm;
        m.ev = sumE / bench.size(); m.dDiff = sumD / bench.size();
        m.fBest = sumF / bench.size(); m.rmse = std::sqrt(m.fBest / 4);
        m.ms = (cv::getTickCount() - t0) / cv::getTickFrequency() * 1000.0 / bench.size();
        mr.push_back(m);
        printf("  %-13s 平均 %5.1f 次求值 | %.4f ms/块 | RMSE %.3f px | 平均|Δopt|=%.3f°\n",
               nm, m.ev, m.ms, m.rmse, m.dDiff);
    };
    // 真值（0.2° 网格）先算好，只用于报告各方法偏差
    std::vector<double> gridRefD(bench.size());
    { double sumE = 0; double t0 = cv::getTickCount();
      for (size_t i = 0; i < bench.size(); i++) { OptResult r = opt1d::grid(bench[i], -90, 90, 0.2); gridRefD[i] = r.delta; sumE += r.evals; }
      printf("  [参考] grid0.2 平均 %.1f 次求值 | %.4f ms/块\n", sumE / bench.size(),
             (cv::getTickCount() - t0) / cv::getTickFrequency() * 1000.0 / bench.size()); }

    run("grid0.5",       [](const Sample& s) { return opt1d::grid(s, -90, 90, 0.5); });
    run("golden",        [](const Sample& s) { return opt1d::golden(s, -90, 90); });
    run("brent",         [](const Sample& s) { return opt1d::brent(s, -90, 0, 90); });
    run("LM(from0)",     [](const Sample& s) { return opt1d::lm(s, 0.0); });
    run("coarse1+Brent", [](const Sample& s) { return opt1d::prod(s); });
    // 各方法与 grid0.2 真值逐块差
    printf("  相对 grid0.2 真值的平均 |Δopt-Δgrid|:\n");
    std::vector<std::function<double(const Sample&)>> disp;
    disp.push_back([](const Sample& s) { return opt1d::grid(s, -90, 90, 0.5).delta; });
    disp.push_back([](const Sample& s) { return opt1d::golden(s, -90, 90).delta; });
    disp.push_back([](const Sample& s) { return opt1d::brent(s, -90, 0, 90).delta; });
    disp.push_back([](const Sample& s) { return opt1d::lm(s, 0.0).delta; });
    disp.push_back([](const Sample& s) { return opt1d::prod(s).delta; });
    const char* names[] = { "grid0.5", "golden", "brent", "LM(from0)", "coarse1+Brent" };
    for (int j = 0; j < 5; j++) {
        double sum = 0;
        for (size_t i = 0; i < bench.size(); i++) sum += std::fabs(disp[j](bench[i]) - gridRefD[i]);
        printf("    %-13s |Δ|=%.4f°\n", names[j], sum / bench.size());
    }

    // ---- C. 近正对(|psi0|<=8)精修跳变：会不会在正负 psi 间跳 ----
    printf("\n===== C. yaw≈0(近正对)精修跳变分析(prodRefine) =====\n");
    const double bins[4][2] = { {0.5,1.5},{1.5,3},{3,6},{6,12} };
    struct BinSt { int n=0,nNear=0,nCross=0,nBig=0; double sumA=0,sumKap=0,sumR0=0,sumRo=0; };
    BinSt st[4];
    std::vector<double> fxv, fyv;
    for (const Sample& s : S) {
        if (!s.hasAlt || s.rmse0 > 2.5 || std::fabs(s.psi0) > 8 || s.dist < 0.5 || s.dist > 12) continue;
        int bin = 3;
        for (int b = 0; b < 4; b++) if (s.dist >= bins[b][0] && s.dist < bins[b][1]) { bin = b; break; }
        OptResult r = opt1d::prod(s);           // Δ:相对 PnP 选中解绕相机竖轴的附加转角
        double psiOpt = wrapDeg(s.psi0 + r.delta);  // 精修后的离正对转角
        double k0 = costSq(s, 0), kp = costSq(s, 1.0), km = costSq(s, -1.0);
        BinSt& B = st[bin];
        B.n++; B.sumA += std::fabs(r.delta); B.sumKap += (kp + km - 2 * k0);
        B.sumR0 += s.rmse0; B.sumRo += std::sqrt(r.bestF / 4);
        // 判据: near=仍贴近平正对; cross=符号相对 PnP 翻转(正负跳变); big=大跳到远镜像
        if (std::fabs(psiOpt) <= 2.0) B.nNear++;
        if ((psiOpt < 0) != (s.psi0 < 0)) B.nCross++;
        if (std::fabs(psiOpt) > 8.0) B.nBig++;
        fxv.push_back(s.dist); fyv.push_back(psiOpt);
    }
    int totN=0,totNear=0,totX=0,totB=0;
    for (int b = 0; b < 4; b++) { totN += st[b].n; totNear += st[b].nNear; totX += st[b].nCross; totB += st[b].nBig; }
    for (int b = 0; b < 4; b++) {
        if (st[b].n == 0) continue;
        printf("dist %.1f-%.1fm: n=%d | 平均|Δ|=%.2f° | 精修后仍近0(|psi|<=2) %.0f%% | "
               "跨0(正负跳变) %.0f%% | 大跳到镜像(|psi|>8) %.0f%% | 曲率κ=%.3g px²/°² | RMSE %.2f->%.2f px\n",
               bins[b][0], bins[b][1], st[b].n, st[b].sumA / st[b].n,
               100.0 * st[b].nNear / st[b].n, 100.0 * st[b].nCross / st[b].n, 100.0 * st[b].nBig / st[b].n,
               st[b].sumKap / st[b].n, st[b].sumR0 / st[b].n, st[b].sumRo / st[b].n);
    }
    if (totN)
        printf("合计 n=%d | 仍近0 %.0f%% | 跨0跳变 %.0f%% | 大跳镜像 %.0f%%\n", totN,
               100.0 * totNear / totN, 100.0 * totX / totN, 100.0 * totB / totN);
    // 近0散点图:y=精修后离正对转角 psiOpt(绿>0,蓝<0,红=出 ±8° 视口的大跳)
    {
        Mat fig = Mat::zeros(Size(1200, 620), CV_8UC3);
        fig.setTo(Scalar(255, 255, 255));
        cv::putText(fig, "near-frontal (|psi0|<8): refined head-deviation psi_opt vs dist  (grn>0 blu<0 red=out-of-range)",
                    Point(24, 34), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
        int x0 = 90, x1 = 1140, y0 = 80, y1 = 580;
        double pmax = 8.0;
        int mid = (y0 + y1) / 2;
        cv::line(fig, Point(x0, mid), Point(x1, mid), Scalar(0, 0, 0), 1);
        cv::putText(fig, "dist(m)", Point(x1 - 60, y1 + 24), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,0), 1);
        cv::putText(fig, "psi_opt(deg)", Point(8, mid - 4), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,0), 1);
        for (size_t i = 0; i < fxv.size(); i++) {
            int xx = x0 + (int)((fxv[i] - 0.3) / 12.0 * (x1 - x0));
            double p = std::max(-pmax, std::min(pmax, fyv[i]));
            int yy = mid - (int)(p / pmax * (y1 - y0) / 2);
            Scalar c = (std::fabs(fyv[i]) > pmax) ? Scalar(0, 0, 255)
                     : ((fyv[i] < 0) ? Scalar(60, 60, 220) : Scalar(60, 220, 60));
            cv::circle(fig, Point(xx, yy), 3, c, -1);
        }
        imwrite(outdir + "/near0_refine_psi.png", fig);
        printf("散点图: %s/near0_refine_psi.png (绿=psiOpt>0 蓝<0, 红=|psiOpt|>8 出界)\n", outdir.c_str());
    }

    // bench 柱状图
    {
        Mat fig = Mat::zeros(Size(1500, 620), CV_8UC3);
        fig.setTo(Scalar(255, 255, 255));
        int x0 = 60, x1 = 1460, yTop1 = 80, yBot1 = 300, yTop2 = 380, yBot2 = 580;
        cv::putText(fig, "mean evals/block (log)", Point(24, 34), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,0), 2);
        double maxE = 1; for (auto& m : mr) maxE = std::max(maxE, m.ev);
        double maxMs = 1e-4; for (auto& m : mr) maxMs = std::max(maxMs, m.ms);
        int n = (int)mr.size();
        for (int i = 0; i < n; i++) {
            double w = (x1 - x0) / n * 0.6;
            double xc = x0 + (x1 - x0) / n * (i + 0.5);
            double h1 = (yBot1 - yTop1) * std::log10(1 + mr[i].ev) / std::log10(1 + maxE);
            cv::rectangle(fig, Point((int)(xc - w/2), (int)(yBot1 - h1)), Point((int)(xc + w/2), (int)yBot1), Scalar(100,100,255), -1);
            cv::putText(fig, mr[i].name, Point((int)(xc - w/2) - 5, yBot1 + 18), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0,0,0), 1);
            char tt[32]; snprintf(tt, sizeof(tt), "%.0f", mr[i].ev);
            cv::putText(fig, tt, Point((int)(xc - w/2), (int)(yBot1 - h1 - 20)), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,0), 1);
            double h2 = (yBot2 - yTop2) * mr[i].ms / maxMs;
            cv::rectangle(fig, Point((int)(xc - w/2), (int)(yBot2 - h2)), Point((int)(xc + w/2), (int)yBot2), Scalar(120,255,120), -1);
            char tm[32]; snprintf(tm, sizeof(tm), "%.4fms", mr[i].ms);
            cv::putText(fig, tm, Point((int)(xc - w/2), (int)(yBot2 - h2 - 6)), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0,0,0), 1);
        }
        cv::putText(fig, "mean ms/block", Point(24, 370), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,0), 2);
        imwrite(outdir + "/bench.png", fig);
        printf("柱状图: %s/bench.png\n", outdir.c_str());
    }
    printf("\n===== 完成 =====\n");
    return 0;
}
