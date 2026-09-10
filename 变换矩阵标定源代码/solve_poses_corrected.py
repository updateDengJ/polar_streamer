"""
Compute precise rigid (rotation + translation only) transforms of the four
quadrant tiles relative to the top-left (lt) tile, using SIFT correspondences
and a global rigid bundle adjustment that closes the 2x2 loop.

Tiles: lt (ref), rt, lb, rb.  Each 2048 x 2448.
Model per tile k:   p_ref = R(theta_k) @ p_k + t_k     (t_k = [tx, ty])
lt is fixed at identity.

Output: H_k = [[cos,-sin,tx],[sin,cos,ty],[0,0,1]] mapping tile-k pixel
coords to the lt reference frame.
"""
import cv2
import numpy as np

TILES = {
    "lt": "corrected_only/lt_corrected.bmp",
    "rt": "corrected_only/rt_corrected.bmp",
    "lb": "corrected_only/lb_corrected.bmp",
    "rb": "corrected_only/rb_corrected.bmp",
}

# Adjacent pairs that share overlap (a, b): we match features between them.
# 四条边全部参与全局平差。rt-rb 曾因原始图上竖向重叠太薄、匹配不可靠而被排除；
# 但在矫正后的图上 rt-rb 有大量稳定匹配(实测 ~78 对)，故加回，让 rb 在
# lb(左邻) 和 rt(上邻) 之间取折中，四条缝一起最优。
PAIRS = [("lt", "rt"), ("lt", "lb"), ("lb", "rb"), ("rt", "rb")]
CHECK_PAIRS = []

RATIO = 0.75
RANSAC_THRESH = 4.0     # px, for the pairwise rigid RANSAC


def load_gray(path):
    img = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(path)
    return img


def sift_matches(g1, g2):
    sift = cv2.SIFT_create()
    k1, d1 = sift.detectAndCompute(g1, None)
    k2, d2 = sift.detectAndCompute(g2, None)
    bf = cv2.BFMatcher(cv2.NORM_L2)
    knn = bf.knnMatch(d1, d2, k=2)
    good = []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < RATIO * n.distance:
            good.append(m)
    p1 = np.float64([k1[m.queryIdx].pt for m in good])
    p2 = np.float64([k2[m.trainIdx].pt for m in good])
    return p1, p2


def rigid_from_pts(src, dst):
    """Least-squares rigid (no scale) mapping src -> dst. Returns (R, t)."""
    sc = src - src.mean(0)
    dc = dst - dst.mean(0)
    U, _, Vt = np.linalg.svd(sc.T @ dc)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = dst.mean(0) - R @ src.mean(0)
    return R, t


def ransac_rigid(src, dst, thresh, iters=5000, seed=0):
    """RANSAC on a pure rigid model. Returns inlier boolean mask."""
    rng = np.random.default_rng(seed)
    n = len(src)
    best_mask = None
    best_cnt = 0
    for _ in range(iters):
        idx = rng.choice(n, 2, replace=False)
        R, t = rigid_from_pts(src[idx], dst[idx])
        pred = (R @ src.T).T + t
        d = np.linalg.norm(pred - dst, axis=1)
        mask = d < thresh
        c = int(mask.sum())
        if c > best_cnt:
            best_cnt = c
            best_mask = mask
    # refit on inliers and re-threshold once
    R, t = rigid_from_pts(src[best_mask], dst[best_mask])
    pred = (R @ src.T).T + t
    d = np.linalg.norm(pred - dst, axis=1)
    return d < thresh


def build_correspondences():
    grays = {k: load_gray(v) for k, v in TILES.items()}
    edges = []
    for a, b in PAIRS:
        # match b's features to a: gives p_a (in a coords) <-> p_b (in b coords)
        pa, pb = sift_matches(grays[a], grays[b])
        if len(pa) < 3:
            print(f"[warn] {a}-{b}: too few matches ({len(pa)})")
            continue
        mask = ransac_rigid(pb, pa, RANSAC_THRESH)  # map b->a for direction sanity
        pa_in, pb_in = pa[mask], pb[mask]
        R, t = rigid_from_pts(pb_in, pa_in)
        ang = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        pred = (R @ pb_in.T).T + t
        rms = float(np.sqrt(np.mean(np.sum((pred - pa_in) ** 2, 1))))
        xspread = pa_in[:, 0].max() - pa_in[:, 0].min()
        yspread = pa_in[:, 1].max() - pa_in[:, 1].min()
        print(f"[pair] {a}-{b}: matches={len(pa)} inliers={int(mask.sum())} "
              f"angle(b->a)={ang:+.4f}deg t=({t[0]:.1f},{t[1]:.1f}) rms={rms:.3f}px "
              f"spread=({xspread:.0f}x{yspread:.0f})")
        edges.append((a, b, pa_in, pb_in))
    return edges


# ---------- global rigid bundle adjustment ----------
# params: for each non-ref tile, [theta, tx, ty]. Order fixed below.
FREE = ["rt", "lb", "rb"]  # lt is reference (identity)


def pose(params, name):
    if name == "lt":
        return 0.0, np.zeros(2)
    i = FREE.index(name)
    th = params[3 * i]
    t = params[3 * i + 1:3 * i + 3]
    return th, t


def R_of(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])


def dR_of(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[-s, -c], [c, -s]])


def residuals_and_J(params, edges):
    rows = []
    Jrows = []
    m = len(params)
    for a, b, pa, pb in edges:
        tha, ta = pose(params, a)
        thb, tb = pose(params, b)
        Ra, Rb = R_of(tha), R_of(thb)
        # residual per point: Ra pa + ta - (Rb pb + tb)   (2-vector)
        res = (Ra @ pa.T).T + ta - ((Rb @ pb.T).T + tb)
        for k in range(len(pa)):
            for comp in range(2):  # x, y component
                rows.append(res[k, comp])
                J = np.zeros(m)
                if a in FREE:
                    ia = FREE.index(a)
                    dRa = dR_of(tha)
                    J[3 * ia] = (dRa @ pa[k])[comp]      # d/dtheta_a
                    J[3 * ia + 1 + comp] = 1.0            # d/dt_a
                if b in FREE:
                    ib = FREE.index(b)
                    dRb = dR_of(thb)
                    J[3 * ib] = -(dRb @ pb[k])[comp]
                    J[3 * ib + 1 + comp] = -1.0
                Jrows.append(J)
    return np.array(rows), np.array(Jrows)


def residuals_only(params, edges):
    rows = []
    for a, b, pa, pb in edges:
        tha, ta = pose(params, a)
        thb, tb = pose(params, b)
        res = (R_of(tha) @ pa.T).T + ta - ((R_of(thb) @ pb.T).T + tb)
        rows.extend(res.ravel())
    return np.array(rows)


def huber_weights(r, delta):
    """Per-point Huber weights (applied to both x,y comps of a point)."""
    # r is flat [x0,y0,x1,y1,...]; compute per-point norm
    rp = r.reshape(-1, 2)
    n = np.linalg.norm(rp, axis=1)
    w = np.ones_like(n)
    big = n > delta
    w[big] = delta / n[big]
    return np.repeat(w, 2)  # back to flat per-component


def initial_guess(edges):
    """Chain estimate to seed the optimizer."""
    p = np.zeros(9)
    # helper: pairwise b->a rigid
    def pair_ba(a, b):
        for ea, eb, pa, pb in edges:
            if (ea, eb) == (a, b):
                R, t = rigid_from_pts(pb, pa)
                return np.arctan2(R[1, 0], R[0, 0]), t
        return None
    # rt: from lt-rt (rt->lt)
    g = pair_ba("lt", "rt")
    if g: p[0], p[1:3] = g[0], g[1]
    # lb: from lt-lb (lb->lt)
    g = pair_ba("lt", "lb")
    if g: p[3], p[4:6] = g[0], g[1]
    # rb: via lb  (rb->lb then lb->lt)
    glb = pair_ba("lb", "rb")
    if glb:
        th_rb_lb, t_rb_lb = glb
        th_lb, t_lb = p[3], p[4:6]
        Rlb = R_of(th_lb)
        p[6] = th_lb + th_rb_lb
        p[7:9] = Rlb @ t_rb_lb + t_lb
    return p


def optimize(edges, iters=200):
    p = initial_guess(edges)
    delta = 3.0          # Huber threshold in px
    lam = 1e-3           # LM damping
    r = residuals_only(p, edges)
    w = huber_weights(r, delta)
    cost = float(np.sum(w * r ** 2))
    for it in range(iters):
        _, J = residuals_and_J(p, edges)
        w = huber_weights(r, delta)
        Jw = J * w[:, None]
        H = Jw.T @ J
        g = Jw.T @ r
        for _try in range(30):
            A = H + lam * np.diag(np.diag(H) + 1e-12)
            try:
                dp = np.linalg.solve(A, -g)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            p_new = p + dp
            r_new = residuals_only(p_new, edges)
            w_new = huber_weights(r_new, delta)
            cost_new = float(np.sum(w_new * r_new ** 2))
            if cost_new < cost:
                p, r, cost = p_new, r_new, cost_new
                lam = max(lam * 0.5, 1e-12)
                break
            lam *= 4
        else:
            break
        if np.linalg.norm(dp) < 1e-11:
            print(f"[opt] converged at iter {it}")
            break
    rms = float(np.sqrt(np.mean(r ** 2)))
    print(f"[opt] done, robust rms={rms:.5f}px")
    return p


def H_matrix(th, t):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, t[0]], [s, c, t[1]], [0, 0, 1.0]])


def main():
    edges = build_correspondences()
    p = optimize(edges)

    print("\n================ Transforms to lt (top-left) reference ================")
    results = {}
    for name in ["lt", "rt", "lb", "rb"]:
        th, t = pose(p, name)
        H = H_matrix(th, t)
        results[name] = H
        print(f"\n# {name} -> lt   (rotation {np.degrees(th):+.5f} deg, "
              f"tx={t[0]:.4f}, ty={t[1]:.4f})")
        with np.printoptions(precision=8, suppress=True):
            print(H)

    # final per-edge RMS check
    r = residuals_only(p, edges)
    print(f"\n[global] solve residual RMS = {np.sqrt(np.mean(r**2)):.5f} px "
          f"over {len(r)//2} correspondences (edges used: {[a+'-'+b for a,b,_,_ in edges]})")

    # loop-closure diagnostic on the excluded rt-rb edge
    grays = {k: load_gray(v) for k, v in TILES.items()}
    for a, b in CHECK_PAIRS:
        pa, pb = sift_matches(grays[a], grays[b])
        if len(pa) < 3:
            continue
        mask = ransac_rigid(pb, pa, RANSAC_THRESH)
        pa_in, pb_in = pa[mask], pb[mask]
        # predict pa from pb using solved poses: p_lt = H_a^-1?  compare in lt frame
        Ha, Hb = results[a], results[b]
        pb_lt = (Hb[:2, :2] @ pb_in.T).T + Hb[:2, 2]
        pa_lt = (Ha[:2, :2] @ pa_in.T).T + Ha[:2, 2]
        d = np.linalg.norm(pa_lt - pb_lt, axis=1)
        print(f"[check] {a}-{b} loop-closure over {int(mask.sum())} matches: "
              f"median={np.median(d):.2f}px mean={d.mean():.2f}px "
              f"(large/scattered => this thin edge is unreliable, as expected)")

    # 保存为可手动编辑的 txt：每张图一段，'名字:' 后跟 3 行 3 列矩阵。
    # 以 '#' 开头的行是注释，读取时忽略。
    with open("poses_to_lt_corrected.txt", "w", encoding="utf-8") as f:
        f.write("# 各图 -> 左上(lt) 的变换矩阵 (p_lt = H @ p_tile)\n")
        f.write("# 每段: '名字:' 后跟 3 行，每行 3 个数（空格分隔）。\n")
        f.write("# 手动微调平移: 改每张矩阵第 1 行第 3 个数(tx, 右移为正)、\n")
        f.write("#              第 2 行第 3 个数(ty, 下移为正)。\n\n")
        for name in ["lt", "rt", "lb", "rb"]:
            H = results[name]
            f.write(f"{name}:\n")
            for row in H:
                f.write("  " + " ".join(f"{v:.10f}" for v in row) + "\n")
            f.write("\n")
    print("\nsaved matrices -> poses_to_lt_corrected.txt")


if __name__ == "__main__":
    main()