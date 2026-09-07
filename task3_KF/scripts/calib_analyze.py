#!/usr/bin/env python3
"""离线标定整车 EKF 初值:装甲板数 N / 旋转半径 r / 车心符号 s / v_yaw / dz。

输入为 collect_armors.py 产出的 .npz。坐标系与 tracker 层一致:
yaw = atan2(R(1,2), R(0,2))（板法线水平投影角）,板心相机系 m。
输出:
  armor_num   板上圆周分布块数 N（2/3/4）
  r_est        板心到旋转轴水平距离(m)（多板同帧锚 + 单板轨迹圆拟合交叉）
  flip_sign    是否需把 target.cpp 的 车心 = 板心 + r·[cos,sin](yaw) 翻成 −r
               （>0 物理=板法线朝外 → 车在板法线负侧 → 现代码反号需翻 = True）
  v_yaw_est    自转角速度 (rad/s)
  dz_est       同帧两板高度差（4 板长轴板/对面板才有意义）
用法: python3 scripts/calib_analyze.py <npz>
"""
import sys
import numpy as np

LIM = lambda a: (a + np.pi) % (2 * np.pi) - np.pi  # 与 math_tools limit_rad 一致


def quat_to_yaw_nz(q):
    """输入列 (w,x,y,z)，返回板法线水平角 yaw 与法线 z 分量（朝相机判定）。"""
    w, x, y, z = q
    r02 = 2 * (x * z + w * y)      # R(0,2)
    r12 = 2 * (y * z - w * x)      # R(1,2)
    r22 = 1 - 2 * (x * x + y * y)  # R(2,2)
    return np.arctan2(r12, r02), r22


def circle_fit(points):
    """Kasa 代数圆拟合。points:(n,2) 水平(x,y)。返回 (cx,cy,r,rmse)。"""
    pts = np.asarray(points, float)
    if len(pts) < 5:
        return None
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack([x, y, np.ones_like(x)])
    b = -(x * x + y * y)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = -sol[0] / 2, -sol[1] / 2
    r2 = (sol[0] ** 2 + sol[1] ** 2) / 4 - sol[2]
    if r2 <= 0 or not np.isfinite(r2):
        return None
    r = np.sqrt(r2)
    res = np.hypot(x - cx, y - cy) - r
    return (cx, cy, r, float(np.sqrt(np.mean(res ** 2))))


def main():
    if len(sys.argv) < 2:
        print('用法: python3 scripts/calib_analyze.py <collect 产出的 .npz>')
        sys.exit(2)
    d = np.load(sys.argv[1])
    arr = d['arr']
    # 主目标 = 观测最多号（red.avi 里为哨兵 G）
    from collections import Counter
    counts = Counter(arr['number'])
    main_num = counts.most_common(1)[0][0]
    print(f'== 按号观测条数 == {dict(counts)}')
    print(f'== 主目标号 = {main_num}')

    rows = []
    for r in arr:
        if r['number'] != main_num:
            continue
        yaw, nz = quat_to_yaw_nz((r['qw'], r['qx'], r['qy'], r['qz']))
        if nz <= 0:      # 只留法线朝相机的板（同 detector 选板口径）
            continue
        rows.append((r['stamp'], r['type'], r['x'], r['y'], r['z'], yaw))
    if len(rows) < 20:
        print('主目标有效观测太少，无法标定')
        sys.exit(1)
    stamps = np.array([r[0] for r in rows])
    X = np.array([[r[2], r[3], r[4]] for r in rows])   # 板心 xyz
    YAW = np.array([r[5] for r in rows])
    TYPES = [r[1] for r in rows]

    # —— 每帧可见板数直方图 ——
    uniq, cnt = np.unique(stamps, return_counts=True)
    print(f'== 采样 {len(uniq)} 帧，每帧 {main_num} 板数直方图(计数):', flush=True)
    for k in sorted(set(cnt)):
        print(f'   {k} 板/帧: {(cnt == k).sum()} 帧', flush=True)
    multi_ok = (cnt >= 2).sum()

    # —— ② 多板同帧锚：同帧两板求 r / 符号 / N ——
    n_hat = np.column_stack([np.cos(YAW), np.sin(YAW)])
    r_ij, s_ij, dpsi = [], [], []
    for s in uniq:
        m = stamps == s
        if m.sum() < 2:
            continue
        idx = np.where(m)[0]
        for a in range(len(idx)):
            for b in range(a + 1, len(idx)):
                i, j = idx[a], idx[b]
                n1, n2 = n_hat[i], n_hat[j]
                ang = abs(LIM(np.arctan2(n2[1], n2[0]) - np.arctan2(n1[1], n1[0])))
                if ang < 0.5 or ang > np.pi - 0.5:   # 夹角 30°~150° 才数值稳
                    continue
                dn = n1 - n2
                dvec = X[i, :2] - X[j, :2]
                rv = np.linalg.norm(dvec) / np.linalg.norm(dn)
                sv = np.sign(np.dot(dvec, dn))
                if 0.01 < rv < 0.5:
                    r_ij.append(rv)
                    s_ij.append(sv)
                dpsi.append(ang)
    print(f'== 多板同帧锚 == 合法对 {len(r_ij)}（另合法夹角 {len(dpsi)}）')
    if r_ij:
        r_med, r_std = float(np.median(r_ij)), float(np.std(r_ij))
        print(f'   r 多板锚: median={r_med:.4f}±{r_std:.4f} m   符号 s 均值={np.mean(s_ij):+.2f}（'
              f'正{s_ij.count(1)}/负{s_ij.count(-1)}）')
        dpsi_a = np.array(dpsi)
        if len(dpsi_a):
            peak = float(np.median(dpsi_a))
            cand = [round(2 * np.pi / peak) for peak in (dpsi_a[abs(dpsi_a - peak) < 0.35] or [peak])]
            from collections import Counter as C2
            print(f'   同帧两板夹角(rad): median={peak:.3f} → N 候选 {C2(cand).most_common(3)}')

    # —— ③ 单板轨迹圆拟合 + 符号判据(每帧取离相机最近板作"可见板") ——
    best = []          # 每帧最近板的水平位置 (x,y)
    best_yaw = []      # 对应的板法线水平角 yaw
    for s in uniq:
        m = stamps == s
        if m.sum() == 0:
            continue
        grp = np.where(m)[0]
        i = grp[np.argmin(np.linalg.norm(X[grp], axis=1))]
        best.append(X[i, :2])
        best_yaw.append(YAW[i])
    cf = circle_fit(best)
    print('== 单板轨迹圆拟合 ==', flush=True)
    if cf:
        cx, cy, r_c, rmse = cf
        print(f'   圆心({cx:.3f},{cy:.3f}) r={r_c:.4f} m  RMSE={rmse:.4f} m'
              f'（RMSE<0.02 才可信，覆盖角度大则代表整个旋转圆周）', flush=True)
        # 符号判据：圆心(≈车心)→板的径向单位向量 u 与板法线 n̂ 点积。
        # 物理上朝相机板法线朝外，车心在板后 → 径向朝外与法线同向 → u·n̂≈+1。
        best = np.asarray(best, float)
        u = best - np.array([[cx, cy]])
        u = u / np.linalg.norm(u, axis=1, keepdims=True)
        n_for_best = np.column_stack([np.cos(best_yaw), np.sin(best_yaw)])
        dot = np.sum(u * n_for_best, axis=1)
        frac_pos = float((dot > 0).mean())
        print(f'   u·n̂（径向 vs 法线）: 均值={dot.mean():+.2f}  >0 占比={frac_pos:.0%}', flush=True)
        # 现代码 target.cpp: 车心 = 板心 + r·n̂(同径向) → u·n̂≈+1 时车心被推到板外侧(相机侧)，
        # 与真实"车在板后"相反 → 需把 +r 翻成 −r。
        print(f'   判定: ' + ('u·n̂≈+1 主导 → target.cpp 的 车心=板心+r·n̂ 反号需翻(flip_sign=True)'
                             if frac_pos > 0.7 else
                             ('u·n̂≈-1 主导 → 车心=板心+r·n̂ 与物理一致,不用翻号'
                              if frac_pos < 0.3 else '符号不明确，需人工复核')))
    else:
        print('   拟合失败（点太少/退化）', flush=True)

    # —— v_yaw：相邻帧最近板 yaw 差 / dt（去掉 >1.2rad 的板间跳变） ——
    tseq = np.sort(uniq)
    dv, dtl = [], []
    for s in tseq:
        m = stamps == s
        i = np.where(m)[0][np.argmin(np.linalg.norm(X[m], axis=1))]
        if 'prev' not in locals():
            prev = (s, YAW[i])
            continue
        dt = s - prev[0]
        if 0 < dt < 0.15:
            dy = LIM(YAW[i] - prev[1])
            if abs(dy) < 1.2:
                dv.append(dy / dt)
                dtl.append(dt)
        prev = (s, YAW[i])
    if dv:
        dv = np.array(dv)
        print(f'== v_yaw == median={np.median(abs(dv)):.2f} rad/s  std={np.std(dv):.2f}'
              f'（正旋转 {np.mean(np.array(dv) > 0):.0%}）')

    # —— dz：同帧两板 z 差 ——
    dzs = []
    for s in uniq:
        m = stamps == s
        if m.sum() < 2:
            continue
        zs = X[m, 2]
        for a in range(len(zs)):
            for b in range(a + 1, len(zs)):
                dzs.append(abs(zs[a] - zs[b]))
    if dzs:
        dzs = np.array(dzs)
        print(f'== 同帧两板 |Δz| == median={np.median(dzs)*1000:.1f} mm')

    # 汇总
    print('\n==== 标定小结（写进 tracker/config/tracker.yaml） ====')
    print(f'  目标号: "{main_num}"')
    if multi_ok == 0:
        print(f'  同帧多板帧: 0（只有单板可见 → N 无法由数据定，默认按 4 板；r 用圆拟合）')
    if cf:
        print(f'  radius_init ≈ {r_c:.3f} m（圆拟合）')
    if r_ij and cf:
        print(f'  radius_init 建议 = median({r_med:.3f}, 圆拟合 {r_c:.3f}) → {(r_med+r_c)/2:.3f} m')


if __name__ == '__main__':
    main()
