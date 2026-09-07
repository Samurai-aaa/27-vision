#!/usr/bin/env python3
"""离线回放新几何（绕相机 y 竖直轴、x-z 水平公转）整车 EKF，验证数值成立后再写 C++。

数据：log/armors.npz（/armors 实录：number/type/x,y,z/qw,qx,qy,qz）。
复刻 target.cpp 新设计：
  状态 x = [cx,vx,cy,vy,cz,vz,yaw,v_yaw,r,l,dz]（11 维，交错，index0/2/4=cx,cy,cz）
  板心 p_id = ( cx + r_i cosφ,  cy + (id1/3? dz:0),  cz + r_i sinφ ),  φ=wrap(yaw+id·2π/N)
  观测 z = [ypd(板心)[方位,俯仰,距离], 板法线水平方位 φ_obs=atan2(n_z,n_x)]
  ypd = (atan2(y,x), atan2(z,√(x²+y²)), √(x²+y²+z²))   —— 与 math_tools 相同
  H 用数值中心差分（回放只验概念，不复制 C++ 解析 H）
门控同 tracker：同号、位置差<0.2、|Δφ|<1.0；命中即 update（同帧多板逐块喂）。
仅用于验证：本脚本不改任何运行代码。

用法：python3 scripts/replay_horizontal.py [npz路径]
"""
import sys
import numpy as np

M2PI = 2 * np.pi


def wrap(a):
    return (a + np.pi) % M2PI - np.pi


def rotv(q, v):
    w, x, y, z = q
    qv = np.array([x, y, z])
    c = np.cross(qv, v)
    return v + 2 * w * c + 2 * np.cross(qv, c)


def ypd(p):
    x, y, z = p
    return np.array([np.arctan2(y, x), np.arctan2(z, np.hypot(x, y)), np.linalg.norm(p)])


def h_plate(x, n, id_):
    phi = wrap(x[6] + id_ * M2PI / n)
    use = (n == 4) and (id_ in (1, 3))
    r = x[8] + x[9] if use else x[8]
    return np.array([x[0] + r * np.cos(phi),
                     x[2] + (x[10] if use else 0.0),
                     x[4] + r * np.sin(phi)])


def obs_h(x, n, id_):
    p = h_plate(x, n, id_)
    phi = wrap(x[6] + id_ * M2PI / n)
    y = ypd(p)
    return np.array([y[0], y[1], y[2], phi])


def F_state(dt):
    F = np.eye(11)
    for i in range(0, 6, 2):
        F[i, i + 1] = dt
    F[6, 7] = dt
    return F


def Q_state(dt, v1=100.0, v2=400.0):
    a = dt ** 4 / 4; b = dt ** 3 / 2; c = dt ** 2
    Q = np.zeros((11, 11))
    for (i, v) in ((0, v1), (2, v1), (4, v1), (6, v2)):
        Q[i:i + 2, i:i + 2] = [[a * v, b * v], [b * v, c * v]]
    return Q


def numer_H(x, n, id_, eps=1e-6):
    base = obs_h(x, n, id_)
    H = np.zeros((4, 11))
    for j in range(11):
        xp = x.copy(); xm = x.copy()
        xp[j] += eps; xm[j] -= eps
        H[:, j] = (obs_h(xp, n, id_) - obs_h(xm, n, id_)) / (2 * eps)
    return H


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'log/armors.npz'
    d = np.load(path, allow_pickle=True)['arr']

    def obsvec(r):
        xyz = np.array([r['x'], r['y'], r['z']])
        n = rotv(np.array([r['qw'], r['qx'], r['qy'], r['qz']]), np.array([0., 0, 1]))
        return dict(stamp=r['stamp'], num=r['number'], xyz=xyz,
                    phi=float(np.arctan2(n[2], n[0])), nz=float(n[2]), ny=float(n[1]))

    rows = [obsvec(r) for r in d if r['number'] == 'G']
    if not rows:
        print('无 G 板数据'); return
    rows.sort(key=lambda a: a['stamp'])
    t0 = rows[0]['stamp']
    # 分帧（同 stamp = 同帧多板）
    frames = []
    for a in rows:
        if frames and abs(frames[-1][-1]['stamp'] - a['stamp']) < 1e-6:
            frames[-1].append(a)
        else:
            frames.append([a])

    N = 4
    r_init = 0.31
    P0 = np.array([1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1])

    # init：取首帧里离相机最近的板，按新公式反推车心
    first = min(frames[0], key=lambda a: np.linalg.norm(a['xyz']))
    a0 = first['phi']
    c0 = np.array([first['xyz'][0] - r_init * np.cos(a0),
                   first['xyz'][1],
                   first['xyz'][2] - r_init * np.sin(a0)])
    x = np.array([c0[0], 0, c0[1], 0, c0[2], 0, a0, 0, r_init, 0, 0])
    P = np.diag(P0)

    t_prev = first['stamp']
    yaw_t = []; vyaw_t = []; r_t = []; res_phi = []; res_pos = []
    anchors = []       # 每帧若双板，两板锚车心 vs EKF 车心
    last_id = 0; n_switch = 0; n_upd = 0
    consec_lost = 0; lost_segs = 0; matched_frames = 0
    est_centers = []
    det_los = []; nz_ok = 0; tot = 0

    def update_one(a_obs):
        nonlocal last_id, n_switch, n_upd
        # 内部关联：最近 3 块(按距离) 里 min |Δφ|
        plates = []
        for i in range(N):
            p = h_plate(x, N, i)
            plates.append((np.linalg.norm(a_obs['xyz'] - p), wrap(x[6] + i * M2PI / N), i))
        plates.sort(key=lambda t: t[0])
        best = min(plates[:3], key=lambda t: abs(wrap(a_obs['phi'] - t[1])))
        id_ = best[2]
        if id_ != last_id:
            n_switch += 1
        last_id = id_
        # R（掠射角自适应）
        p = h_plate(x, N, id_)
        na = np.array([np.cos(a_obs['phi']), 0, np.sin(a_obs['phi'])])
        tocam = -p / np.linalg.norm(p)
        cosd = np.clip(np.dot(na, tocam), -1.0, 1.0)
        delta = np.arccos(cosd)
        dist = np.linalg.norm(a_obs['xyz'])
        R = np.diag([4e-3, 4e-3, np.log(dist + 1) / 200 + 9e-2, np.log(delta + 1) + 1])
        z = np.array([ypd(a_obs['xyz'])[0], ypd(a_obs['xyz'])[1], ypd(a_obs['xyz'])[2], a_obs['phi']])
        H = numer_H(x, N, id_)
        h = obs_h(x, N, id_)
        innov = z - h
        for k in (0, 1, 3):
            innov[k] = wrap(innov[k])
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.solve(S, np.eye(4))
        I = np.eye(11)
        P[:] = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T
        x[:] += K @ innov
        x[6] = wrap(x[6])
        n_upd += 1
        res_phi.append(abs(wrap(z[3] - h[3])))
        res_pos.append(np.linalg.norm(a_obs['xyz'] - p))
        # r 限幅（tracker 同款）
        if x[8] < 0.12 or x[8] > 0.4:
            x[8] = np.clip(x[8], 0.12, 0.4)

    # 主循环：按帧 predict(dt)，同帧同号板逐块 gate+update（sp_vision 多板融合）
    for fi, fr in enumerate(frames):
        a0f = fr[0]
        dt = a0f['stamp'] - t_prev
        t_prev = a0f['stamp']
        # predict
        F = F_state(dt)
        x[:] = F @ x
        x[6] = wrap(x[6])
        P = F @ P @ F.T + Q_state(dt)
        # gate+update
        fed = []
        for a in fr:
            tot += 1
            if a['nz'] < 0:
                nz_ok += 1
            best_d = None; best_phi_d = 1e9; best_i = None
            for i in range(N):
                pd = np.linalg.norm(a['xyz'] - h_plate(x, N, i))
                if best_d is None or pd < best_d:
                    best_d = pd; best_i = i
            pdiff = best_d
            phidiff = abs(wrap(a['phi'] - wrap(x[6] + best_i * M2PI / N)))
            if pdiff < 0.2 and phidiff < 1.0:
                fed.append(a)
        if fed:
            matched_frames += 1
            consec_lost = 0
            # 同帧多板逐块喂
            for a in sorted(fed, key=lambda a: np.linalg.norm(a['xyz'])):
                update_one(a)
        else:
            consec_lost += 1
        yaw_t.append(x[6]); vyaw_t.append(x[7]); r_t.append(x[8])
        est_centers.append(x[[0, 2, 4]].copy())
        # 双板锚：两板各自 C=p−r·u，比较中位
        if len(fr) >= 2:
            for a in fr:
                u = np.array([np.cos(a['phi']), 0, np.sin(a['phi'])])
                anchors.append(a['xyz'] - r_init * u)
            det_los.append(True)
        else:
            det_los.append(False)

    yaw_t = np.array(yaw_t); vyaw_t = np.array(vyaw_t); r_t = np.array(r_t)
    est_centers = np.array(est_centers)
    anchors = np.array(anchors)
    # 锚与 EKF 车心偏差（锚帧取同帧均值后比）
    anchor_errs = []
    k = 0
    for fi, (fr, est) in enumerate(zip(frames, est_centers)):
        if len(fr) >= 2:
            a_m = np.mean([fr[i]['xyz'] - r_init * np.array([np.cos(fr[i]['phi']), 0, np.sin(fr[i]['phi'])])
                           for i in range(len(fr))], axis=0)
            anchor_errs.append(np.linalg.norm(a_m - est))
    anchor_errs = np.array(anchor_errs)

    uyaw = np.unwrap(yaw_t)
    r_med_after = np.median(r_t[matched_frames // 2:]) if matched_frames > 1 else np.nan
    print('=== 回放统计 (帧=%d 命中=%d) ===' % (len(frames), matched_frames))
    print('板法线 nz<0 占比: %.4f   ny 中位 |%.3f|' % (nz_ok / max(1, tot),
                                                        np.median([np.abs(a['ny']) for a in rows])))
    print('yaw: 首=%+.2f 末=%+.2f  全段摆动 %.2f rad  (unwrap)' % (uyaw[0], uyaw[-1], uyaw[-1] - uyaw[0]))
    print('v_yaw: 中位 %+.3f  p90 %.3f  max|.| %.3f' % (np.median(vyaw_t), np.percentile(np.abs(vyaw_t), 90),
                                                         np.max(np.abs(vyaw_t))))
    print('r: p10=%.3f 中位=%.3f  p90=%.3f  (后半中位 %.3f)' % (
        np.percentile(r_t, 10), np.median(r_t), np.percentile(r_t, 90), r_med_after))
    print('EKF车心 vs 双板锚偏差: 中位 %.3f m  p90 %.3f' % (
        np.median(anchor_errs) if len(anchor_errs) else np.nan,
        np.percentile(anchor_errs, 90) if len(anchor_errs) else np.nan))
    print('更新次数=%d  id切换=%d  last_id=%d  板角残差中位 %.4f rad  板心残差中位 %.3f m' % (
        n_upd, n_switch, last_id, np.median(res_phi), np.median(res_pos)))
    # 连续无命中段统计（掉帧/漏检）
    print('连续无命中帧: max=%d (%.2fs)' % (
        consec_lost, np.max(consec_lost) * 0.023))
    ok = (np.abs(np.median(vyaw_t)) < 3 and np.percentile(np.abs(vyaw_t), 90) < 8
          and 0.25 < r_med_after < 0.4 and np.median(res_phi) < 0.1
          and np.median(anchor_errs) < 0.15)
    print('结论: ' + ('OK：几何/可观测性/关联数值成立' if ok else '需调整（见上指标）'))


if __name__ == '__main__':
    main()
