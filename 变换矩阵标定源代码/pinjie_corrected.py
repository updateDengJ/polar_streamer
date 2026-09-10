"""
四宫格拼接（矫正后图版本）。
矩阵从 solve_poses_corrected.py 生成的 poses_to_lt_corrected.txt 读取(可手动编辑)，
拼的是 corrected_only/ 里矫正后的四张图。
依赖: pip install numpy opencv-python
"""
import cv2
import numpy as np

# ============== 输入 / 输出 配置 ==============
TILES = {
    "lt": "corrected_only/lt_corrected.bmp",
    "rt": "corrected_only/rt_corrected.bmp",
    "lb": "corrected_only/lb_corrected.bmp",
    "rb": "corrected_only/rb_corrected.bmp",
}
POSES = "poses_to_lt_corrected.txt"          # solve_poses_corrected.py 的输出(可手动编辑)
OUTPUT = r"panorama_corrected.bmp"           # 输出全景图

BLEND = "average"   # "average"(重叠区取平均, 接缝较柔) 或 "overwrite"(后画的覆盖前面)
# =============================================


def load(path):
    img = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"读不到图片: {path}")
    return img


def warped_corners(h, w, H):
    """把一张 h x w 图的四角用 H 变换后的坐标返回 (4,2)。"""
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float64)
    pts = np.concatenate([corners, np.ones((4, 1))], axis=1).T  # 3x4
    out = H @ pts
    out /= out[2:3, :]
    return out[:2, :].T


def load_poses(path):
    """读取 txt 矩阵文件。格式：每段 '名字:' 后跟 3 行 3 列；'#' 开头为注释。"""
    poses = {}
    name = None
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            if s.endswith(":"):                 # 新的一段开始
                if name is not None and rows:
                    poses[name] = np.array(rows, dtype=np.float64)
                name = s[:-1].strip()
                rows = []
            else:
                rows.append([float(v) for v in s.split()])
    if name is not None and rows:               # 收尾最后一段
        poses[name] = np.array(rows, dtype=np.float64)
    for k in ("lt", "rt", "lb", "rb"):
        if k not in poses or poses[k].shape != (3, 3):
            raise ValueError(f"矩阵文件缺少或格式错误: {k}")
    return poses


def main():
    poses = load_poses(POSES)
    items = [(k, load(p), poses[k]) for k, p in TILES.items()]

    # ---- 计算总画布范围 ----
    all_pts = []
    for _, img, H in items:
        h, w = img.shape[:2]
        all_pts.append(warped_corners(h, w, H))
    all_pts = np.concatenate(all_pts, axis=0)
    xmin, ymin = np.floor(all_pts.min(axis=0)).astype(int)
    xmax, ymax = np.ceil(all_pts.max(axis=0)).astype(int)
    W = xmax - xmin
    H_ = ymax - ymin
    print(f"[画布] 尺寸 = {W} x {H_}, 偏移 = ({-xmin}, {-ymin})")

    # 平移矩阵: 把最小坐标挪到 (0,0)
    T = np.array([[1, 0, -xmin],
                  [0, 1, -ymin],
                  [0, 0, 1]], dtype=np.float64)

    # ---- 逐张 warp + 融合 ----
    acc = np.zeros((H_, W, 3), dtype=np.float32)   # 累加颜色
    cnt = np.zeros((H_, W), dtype=np.float32)      # 累加权重(覆盖次数)
    canvas = np.zeros((H_, W, 3), dtype=np.uint8)  # overwrite 模式用

    for name, img, H in items:
        Hf = T @ H
        warp = cv2.warpPerspective(img, Hf, (W, H_))
        # 掩码用「实际有内容」判定：矫正图四角的黑三角(值~0)必须排除，
        # 否则它们会以 0 值混进平均，把拼缝处的画面拉黑。
        content = cv2.warpPerspective(
            (img.max(axis=2) > 5).astype(np.uint8) if img.ndim == 3 else (img > 5).astype(np.uint8),
            Hf, (W, H_))
        m = content > 0
        if BLEND == "average":
            acc[m] += warp[m].astype(np.float32)
            cnt[m] += 1.0
        else:  # overwrite
            canvas[m] = warp[m]
        print(f"  已合成 {name}")

    if BLEND == "average":
        cnt[cnt == 0] = 1.0
        canvas = (acc / cnt[..., None]).clip(0, 255).astype(np.uint8)

    ok, buf = cv2.imencode(".bmp", canvas)
    buf.tofile(OUTPUT)
    print(f"[完成] 全景图已保存: {OUTPUT}")


if __name__ == "__main__":
    main()