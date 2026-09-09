// yawstudy 模块实现（接口见 include/yawstudy.hpp）。
//
// 一块板重投影误差的一维代价曲线用"手写投影"算，不经过 Solver，保证每步只改
// yaw、tvec 固定，方便反复扫描/求值。所有代价/优化函数对 Δ(度) 操作，Δ=0 即
// IPPE 选中姿态。

#include "yawstudy.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace task2 {
namespace yawstudy {

// ================= 角度 / 姿态工具 =================

double wrapDeg(double a) {
    double r = std::fmod(a + 180.0, 360.0);
    if (r < 0) r += 360.0;
    return r - 180.0;
}

double normalAzimuthDeg(const double R[9]) {
    return std::atan2(R[2], R[8]) * 180.0 / CV_PI;  // R(0,2), R(2,2)
}

double headPsiDeg(const double R[9]) {
    // 模型 +z 朝外时，正对板的法线指向相机(-z)，psi=atan2(nx,-nz) 正对≈0
    return wrapDeg(std::atan2(R[2], -R[8]) * 180.0 / CV_PI);
}

void rotateYaw(const Sample& s, double deltaDeg, double Rout[9]) {
    double d = deltaDeg * CV_PI / 180.0;
    double c = std::cos(d), sn = std::sin(d);
    // Ry*R：行0 = c*row0+s*row2；行1 = row1；行2 = -s*row0+c*row2
    Rout[0] = c * s.r0[0] + sn * s.r0[6]; Rout[1] = c * s.r0[1] + sn * s.r0[7]; Rout[2] = c * s.r0[2] + sn * s.r0[8];
    Rout[3] = s.r0[3]; Rout[4] = s.r0[4]; Rout[5] = s.r0[5];
    Rout[6] = -sn * s.r0[0] + c * s.r0[6]; Rout[7] = -sn * s.r0[1] + c * s.r0[7]; Rout[8] = -sn * s.r0[2] + c * s.r0[8];
}

bool evalResiduals(const Sample& s, const double R[9], double e[8]) {
    for (int k = 0; k < 4; k++) {
        double X = R[0] * s.mx[k] + R[1] * s.my[k] + s.t0[0];
        double Y = R[3] * s.mx[k] + R[4] * s.my[k] + s.t0[1];
        double Z = R[6] * s.mx[k] + R[7] * s.my[k] + s.t0[2];
        if (Z <= 1e-6) return false;  // 角点跑到相机后，投影无意义
        double u = kCamFx * X / Z + kCamCx, v = kCamFy * Y / Z + kCamCy;
        e[2 * k] = u - s.qx[k];
        e[2 * k + 1] = v - s.qy[k];
    }
    return true;
}

// ================= 样本构造 =================

bool makeSample(const Armor& a, int frame, Sample& out) {
    Sample s;
    if (a.type == ArmorType::BIG) { s.hw = 115; s.hh = 27.5; }
    else if (a.type == ArmorType::SMALL) { s.hw = 65; s.hh = 27.5; }
    else return false;  // INVALID：没有可用的 3D 模型

    // 3D 模型角点（z=0 平面，TL,BL,BR,TR）
    s.mx[0] = -s.hw; s.my[0] = s.hh; s.mx[1] = -s.hw; s.my[1] = -s.hh;
    s.mx[2] = s.hw;  s.my[2] = -s.hh; s.mx[3] = s.hw; s.my[3] = s.hh;
    // 2D 检测角点
    for (int k = 0; k < 4; k++) { s.qx[k] = a.corners[k].x; s.qy[k] = a.corners[k].y; }

    // 选中解：旋转向量 → 旋转矩阵
    cv::Mat Rm;
    cv::Rodrigues(a.rvec, Rm);
    for (int i = 0; i < 9; i++) s.r0[i] = Rm.at<double>(i / 3, i % 3);
    for (int i = 0; i < 3; i++) s.t0[i] = a.tvec.at<double>(i);
    s.yaw0 = normalAzimuthDeg(s.r0);
    s.psi0 = headPsiDeg(s.r0);

    // 另一解（solvePnPGeneric 未选中的镜像假设），仅双解时存在
    if (!a.rvec_alt.empty()) {
        s.hasAlt = true;
        cv::Mat Ra;
        cv::Rodrigues(a.rvec_alt, Ra);
        for (int i = 0; i < 9; i++) s.rA[i] = Ra.at<double>(i / 3, i % 3);
        for (int i = 0; i < 3; i++) s.tA[i] = a.tvec_alt.at<double>(i);
        s.yawA = normalAzimuthDeg(s.rA);
        s.psiA = headPsiDeg(s.rA);
    }

    s.dist = a.distance;
    s.frame = frame;

    // 两解各自的重投影 RMSE（旋转用各自的 r，平移固定为选中解 t0）
    double sq = 0, sqA = 0;
    { double R[9], e[8];
      for (int i = 0; i < 9; i++) R[i] = s.r0[i];
      if (evalResiduals(s, R, e)) for (int i = 0; i < 8; i++) sq += e[i] * e[i];
      if (s.hasAlt && evalResiduals(s, s.rA, e)) for (int i = 0; i < 8; i++) sqA += e[i] * e[i]; }
    s.rmse0 = std::sqrt(sq / 4);
    s.rmseA = std::sqrt(sqA / 4);

    out = s;
    return true;
}

// ================= 代价函数 =================

double costSq(const Sample& s, double deltaDeg) {
    double R[9], e[8];
    rotateYaw(s, deltaDeg, R);
    if (!evalResiduals(s, R, e)) return 1e300;  // 有角点在相机后 → 大数罚
    double sq = 0;
    for (int i = 0; i < 8; i++) sq += e[i] * e[i];
    return sq;
}

double costL1(const Sample& s, double deltaDeg) {
    double R[9], e[8];
    rotateYaw(s, deltaDeg, R);
    if (!evalResiduals(s, R, e)) return 1e300;
    double l1 = 0;
    for (int k = 0; k < 4; k++) l1 += std::sqrt(e[2*k]*e[2*k] + e[2*k+1]*e[2*k+1]);
    return l1;
}

// ================= 代价曲线 / 局部极小 =================

Curve scanCurve(const Sample& s, double lo, double hi, double stepDeg) {
    Curve c;
    for (double d = lo; d <= hi + stepDeg * 0.5; d += stepDeg) {
        c.xs.push_back(d);
        c.sq.push_back(costSq(s, d));
        c.l1.push_back(costL1(s, d));
    }
    return c;
}

std::vector<int> localMinima(const std::vector<double>& y) {
    std::vector<int> out;
    for (size_t i = 1; i + 1 < y.size(); i++)
        if (y[i] <= y[i - 1] && y[i] <= y[i + 1] && y[i] < 1e20) out.push_back((int)i);
    return out;
}

// ================= 一维优化器 =================

namespace opt1d {

OptResult grid(const Sample& s, double lo, double hi, double stepDeg) {
    OptResult r;
    r.bestF = 1e300;
    r.delta = lo;
    for (double d = lo; d <= hi + stepDeg * 0.5; d += stepDeg) {
        double f = costSq(s, d);
        r.evals++;
        if (f < r.bestF) { r.bestF = f; r.delta = d; }
    }
    return r;
}

OptResult golden(const Sample& s, double lo, double hi) {
    // 假定单峰。注意：yaw-only 代价在"单深谷+浅远谷"上非严格单峰，会锁错谷
    const double gr = (std::sqrt(5.0) - 1.0) / 2.0;
    const double tolDeg = 1e-4;
    double a = lo, b = hi;
    double x1 = b - gr * (b - a), x2 = a + gr * (b - a);
    double f1 = costSq(s, x1), f2 = costSq(s, x2);
    int iters = 2;
    while (std::fabs(b - a) > tolDeg) {
        if (f1 < f2) { b = x2; x2 = x1; f2 = f1; x1 = b - gr * (b - a); f1 = costSq(s, x1); }
        else         { a = x1; x1 = x2; f1 = f2; x2 = a + gr * (b - a); f2 = costSq(s, x2); }
        iters++;
    }
    OptResult r;
    r.delta = 0.5 * (a + b);
    r.evals = iters;
    r.bestF = costSq(s, r.delta);
    return r;
}

OptResult brent(const Sample& s, double lo, double mid, double hi) {
    // 假定单峰（三点点括 ax<bx<cx）。NR 风格 Brent 无导数最小化。
    const double CGOLD = 0.3819660112501051, ZEPS = 1e-10;
    const double tolDeg = 1e-5;
    auto f = [&s](double x) { return costSq(s, x); };
    double a = std::min(lo, hi), b = std::max(lo, hi);
    double x = mid, w = mid, v = mid;
    double fw = f(w), fv = fw, fx = fw;
    double e = 0.0, d = 0.0;
    int iters = 1;
    for (int i = 0; i < 60; i++) {
        double xm = 0.5 * (a + b);
        double tol1 = tolDeg * std::fabs(x) + ZEPS;
        double tol2 = 2 * tol1;
        if (std::fabs(x - xm) <= tol2 - 0.5 * (b - a)) break;
        double u = 0, fu = 0;
        if (std::fabs(e) > tol1) {
            double r = (x - w) * (fx - fv), q = (x - v) * (fx - fw);
            double p = (x - v) * q - (x - w) * r;
            q = 2 * (q - r);
            if (q > 0) p = -p;
            q = std::fabs(q);
            double etemp = e; e = d;
            if (std::fabs(p) >= std::fabs(0.5 * q * etemp) || p <= q * (a - x) || p >= q * (b - x))
                d = CGOLD * (e = (x >= xm ? a - x : b - x));
            else { d = p / q; u = x + d; if (u - a < tol2 || b - u < tol2) d = (xm - x >= 0) ? tol1 : -tol1; }
        } else d = CGOLD * (e = (x >= xm ? a - x : b - x));
        u = (std::fabs(d) >= tol1) ? x + d : x + (d >= 0 ? tol1 : -tol1);
        fu = f(u); iters++;
        if (fu <= fx) {
            if (u >= x) a = x; else b = x;
            v = w; fv = fw; w = x; fw = fx; x = u; fx = fu;
        } else {
            if (u < x) a = u; else b = u;
            if (fu <= fw || w == x) { v = w; fv = fw; w = u; fw = fu; }
            else if (fu <= fv || v == x || v == w) { v = u; fv = fu; }
        }
    }
    OptResult r;
    r.delta = x;
    r.evals = iters;
    r.bestF = costSq(s, x);
    return r;
}

OptResult lm(const Sample& s, double startDeg) {
    // 阻尼高斯牛顿：残差 8 维、数值雅可比（步长 1e-3°），起点 startDeg
    const double hd = 1e-3;
    double d = startDeg;
    double lam = 1e-3;
    int iters = 0;
    for (int it = 0; it < 40; it++) {
        double R[9], e[8], e1[8], e2[8];
        rotateYaw(s, d, R);
        if (!evalResiduals(s, R, e)) break;
        double dn0 = d + hd, dn1 = d - hd, R1[9], R2[9];
        rotateYaw(s, dn0, R1); rotateYaw(s, dn1, R2);
        bool ok1 = evalResiduals(s, R1, e1), ok2 = evalResiduals(s, R2, e2);
        if (!ok1 || !ok2) break;
        double g = 0, H = 0;
        for (int i = 0; i < 8; i++) {
            double J = (e1[i] - e2[i]) / (2 * hd);
            g += J * e[i];
            H += J * J;
        }
        double f0 = 0;
        for (int i = 0; i < 8; i++) f0 += e[i] * e[i];
        double step = g / (H + lam);
        double f1 = costSq(s, d - step);
        iters++;
        if (f1 <= f0) {
            d -= step; lam *= 0.6;
            if (std::fabs(step) < 1e-5 || (f0 - f1) < 1e-9 * (1.0 + f0)) break;
        } else {
            lam *= 5.0;
            if (lam > 1e8) break;
        }
    }
    OptResult r;
    r.delta = d;
    r.evals = iters;
    r.bestF = costSq(s, d);
    return r;
}

OptResult prod(const Sample& s) {
    // 1° 粗扫定主谷（对"单深谷+浅远谷"稳健），再 Brent 抛光
    OptResult g = grid(s, -90, 90, 1.0);
    OptResult b = brent(s, g.delta - 6, g.delta, g.delta + 6);
    b.evals += g.evals;   // 求值总数 = 粗扫 + 抛光
    return b;
}

}  // namespace opt1d

// ================= 渲染 =================
// 单面板：画一/多根曲线 + 竖线标记；x 轴 Δ(度)，y 可选 log10(1+v)
static cv::Mat drawPanel(int W, int H, const std::string& title,
                         double xmin, double xmax, double ymin, double ymax, bool logy,
                         const std::vector<std::vector<double>>& xs,
                         const std::vector<std::vector<double>>& ys,
                         const std::vector<cv::Scalar>& cols,
                         const std::vector<std::vector<double>>& xmarks) {
    using namespace cv;
    Mat img = Mat::zeros(Size(W, H), CV_8UC3);
    img.setTo(Scalar(255, 255, 255));
    int ml = 110, mr = 24, mt = 44, mb = 52;
    auto mpx = [&](double v) { return ml + (int)std::lround((v - xmin) / (xmax - xmin) * (W - ml - mr)); };
    auto mpy = [&](double v) {
        double yv = logy ? std::log10(1.0 + std::max(v, 0.0)) : v;
        return mt + (int)std::lround((1.0 - (yv - ymin) / (ymax - ymin)) * (H - mt - mb)); };
    cv::putText(img, title, Point(ml, 26), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 0, 0), 1, LINE_AA);
    cv::rectangle(img, Point(ml, mt), Point(W - mr, H - mb), Scalar(0, 0, 0), 1);
    for (int i = 0; i <= 4; i++) {
        double v = ymin + (ymax - ymin) * i / 4.0;
        double dv = logy ? std::pow(10.0, v) - 1.0 : v;
        int yy = mt + (int)((1.0 - i / 4.0) * (H - mt - mb));
        cv::line(img, Point(ml, yy), Point(W - mr, yy), Scalar(225, 225, 225), 1);
        char t[48]; snprintf(t, sizeof(t), logy ? "%.0e" : "%.2f", dv);
        cv::putText(img, t, Point(4, yy + 4), FONT_HERSHEY_SIMPLEX, 0.42, Scalar(60, 60, 60), 1, LINE_AA);
    }
    for (double v = std::ceil(xmin / 30.0) * 30.0; v <= xmax; v += 30.0) {
        int xx = mpx(v);
        cv::line(img, Point(xx, mt), Point(xx, H - mb), Scalar(230, 230, 230), 1);
        char t[32]; snprintf(t, sizeof(t), "%.0f", v);
        cv::putText(img, t, Point(xx - 9, H - mb + 18), FONT_HERSHEY_SIMPLEX, 0.42, Scalar(0, 0, 0), 1, LINE_AA);
    }
    cv::putText(img, "delta_deg (0 = IPPE selected yaw)", Point(ml, H - 12),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1, LINE_AA);
    for (size_t j = 0; j < ys.size(); j++) {
        std::vector<Point> pts;
        for (size_t i = 0; i < ys[j].size(); i++) {
            double v = ys[j][i];
            if (v >= 1e20) continue;
            pts.emplace_back(mpx(xs[j][i]), mpy(v));
        }
        for (size_t i = 1; i < pts.size(); i++)
            cv::line(img, pts[i - 1], pts[i], cols[j], j == 0 ? 2 : 1, LINE_AA);
    }
    for (auto& mk : xmarks) {
        if (mk.empty()) continue;
        double xv = mk[0], yv = mk.size() > 1 ? mk[1] : ymin;
        Scalar col = mk.size() > 2 ? Scalar(mk[2], mk[3], mk[4]) : Scalar(0, 255, 0);
        int xx = mpx(xv);
        cv::line(img, Point(xx, mt), Point(xx, H - mb), col, 1);
        int yy = mpy(std::min(yv, 1e14));
        cv::circle(img, Point(xx, yy), 5, col, -1, LINE_AA);
    }
    return img;
}

// 单张完整图：三面板（costSq log / costL1 log / costSq 近 0 线性 + 局部极小竖线）
void saveCurveFigure(const Sample& s, int idx, const std::string& outdir,
                     const Curve& c, double dAlt, double costAltSq, double zoomRad) {
    using namespace cv;
    const int PW = 1500, PH = 320;
    const std::vector<double>& xs = c.xs;
    const std::vector<double>& ys = c.sq;
    const std::vector<double>& yl = c.l1;

    int i0 = 0;
    for (size_t i = 0; i < xs.size(); i++) if (xs[i] >= -0.25 && xs[i] <= 0.25) i0 = (int)i;

    auto maxFinite = [](const std::vector<double>& v) { double m = 1; for (double x : v) if (x < 1e20) m = std::max(m, x); return m; };
    auto minFinite = [](const std::vector<double>& v) { double m = 1e300; for (double x : v) if (x < 1e20) m = std::min(m, x); return m; };

    double ymaxY = std::max(maxFinite(ys), costAltSq);
    double ymaxL = maxFinite(yl);
    double yminY = std::max(0.0, std::log10(1 + minFinite(ys)) - 1.2);
    double ymaxYl = std::log10(1 + ymaxY) * 1.02;
    double ymaxLl = std::log10(1 + ymaxL) * 1.02;

    std::vector<std::vector<double>> Xs{ xs }, YsY{ ys }, YsL{ yl };
    std::vector<Scalar> cols{ Scalar(20, 20, 220), Scalar(200, 60, 60) };

    // 标记：绿 = 选中解(Δ=0)，橙 = 另一候选(Δ=dAlt)
    std::vector<std::vector<double>> mk1;
    mk1.push_back({ 0, ys[i0], 0, 200, 0 });
    if (costAltSq < 1e20) mk1.push_back({ dAlt, costAltSq, 0, 140, 255 });

    char t0[200];
    snprintf(t0, sizeof(t0), "rep%d  yaw0=%.2f  psi0=%.2f  dist=%.2fm  %s", idx, s.yaw0, s.psi0, s.dist, s.hw > 100 ? "BIG" : "SMALL");
    Mat p1 = drawPanel(PW, PH, std::string(t0) + "  |  cost=sum(dx^2+dy^2), log", -180, 180, yminY, ymaxYl, true, Xs, YsY, { cols[0] }, mk1);
    Mat p2 = drawPanel(PW, PH, "cost=sum(L2 per corner), log", -180, 180, 0, ymaxLl, true, Xs, YsL, { cols[1] }, {});
    // 近 0 放大线性 + 局部极小位置竖线
    std::vector<std::vector<double>> mk3;
    { auto lm = localMinima(ys);
      for (int i : lm)
          if (xs[i] >= -zoomRad && xs[i] <= zoomRad) mk3.push_back({ xs[i], ys[i], 0, 0, 0 }); }
    double loC = minFinite(ys), hiC = maxFinite(ys);
    Mat p3 = drawPanel(PW, PH, "zoom near 0, cost linear; black vlines = local minima", -zoomRad, zoomRad,
                       0, std::min(hiC * 1.05, loC * 40 + 20), false, Xs, YsY, { cols[0] }, mk3);

    Mat fig = Mat::zeros(Size(PW, 3 * PH + 90), CV_8UC3);
    fig.setTo(Scalar(255, 255, 255));
    p1.copyTo(fig(Rect(0, 0, PW, PH)));
    p2.copyTo(fig(Rect(0, PH, PW, PH)));
    p3.copyTo(fig(Rect(0, 2 * PH, PW, PH)));
    char fname[256];
    snprintf(fname, sizeof(fname), "%s/curve_rep%d_psi%+.1f_d%.2fm.png", outdir.c_str(), idx, s.psi0, s.dist);
    imwrite(fname, fig);
    printf("   saved %s\n", fname);
}

}  // namespace yawstudy
}  // namespace task2
