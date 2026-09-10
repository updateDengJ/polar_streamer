"""
只做「透视矫正」，不拼接。用于查看四张图各自矫正后的样子。

原理：同一相机绕中心朝四方向偏转 (yaw=±4.375, pitch=±6.61) 拍摄，
每张图用 infinite homography  H = K · R · K^-1  掰正回同一虚拟平面。
矫正只依赖相机参数，与图像内容无关（不需要特征点匹配）。

参考 ../2026.7.29/correct_and_stitch.py，但去掉了平移微调和融合，
只输出矫正结果，方便观察 H = K·R·K⁻¹ 的效果。
依赖: pip install numpy opencv-python
"""
import cv2
import numpy as np
import math
import os

# ============== 配置 ==============
TILES = {"lt": "lt.bmp", "rt": "rt.bmp", "lb": "lb.bmp", "rb": "rb.bmp"}
OUT_DIR = "corrected_only"              # 矫正结果输出目录

YAW_HALF, PITCH_HALF = 4.375, 6.61      # 半偏转角(度)
F, PIXEL_SIZE = 35.0, 0.00345           # 焦距(mm), 像元(mm)
IMG_W, IMG_H = 2048, 2448               # 单图尺寸
CANVAS_W, CANVAS_H = 4000, 5000         # 矫正后画布(留大一点防止切边)

# 四张图各自的偏转角 (yaw, pitch)
ANGLES = {
    "lt": (-YAW_HALF,  PITCH_HALF),     # 左上：往左上转
    "rt": ( YAW_HALF,  PITCH_HALF),     # 右上：往右上转
    "lb": (-YAW_HALF, -PITCH_HALF),     # 左下：往左下转
    "rb": ( YAW_HALF, -PITCH_HALF),     # 右下：往右下转
}
# ==================================


def create_rotation_matrix(yaw_deg, pitch_deg):
    """构造把相机偏转「转正」用的 3D 旋转矩阵。"""
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(pitch), -np.sin(pitch)],
                   [0, np.sin(pitch),  np.cos(pitch)]])
    Ry = np.array([[np.cos(yaw), 0, np.sin(yaw)],
                   [0, 1, 0],
                   [-np.sin(yaw), 0, np.cos(yaw)]])
    return Ry @ Rx


def load(path):
    img = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(path)
    return img


def save(path, img):
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img)
    if not ok:
        raise RuntimeError(f"编码失败: {path}")
    buf.tofile(path)


def crop_black(img):
    """裁掉四周的黑边，只保留有内容的区域（单独查看用）。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ys, xs = np.where(gray > 5)
    if len(xs) == 0:
        return img
    return img[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def main():
    print("========== 只做透视矫正（不拼接）==========\n")
    os.makedirs(OUT_DIR, exist_ok=True)

    # 相机内参 K
    f_px = F / PIXEL_SIZE
    cx, cy = IMG_W / 2.0, IMG_H / 2.0
    K = np.array([[f_px, 0, cx], [0, f_px, cy], [0, 0, 1]])
    K_inv = np.linalg.inv(K)
    # 把矫正结果挪到大画布中心，避免跑出画面
    T_shift = np.array([[1, 0, CANVAS_W / 2.0 - cx],
                        [0, 1, CANVAS_H / 2.0 - cy],
                        [0, 0, 1]])
    print(f"焦距 f_px = {f_px:.1f} 像素")

    for name, path in TILES.items():
        img = load(path)
        yaw, pitch = ANGLES[name]

        # 矫正矩阵 H = T · (K · R · K⁻¹)
        R = create_rotation_matrix(yaw, pitch)
        H = T_shift @ (K @ R @ K_inv)

        warped = cv2.warpPerspective(img, H, (CANVAS_W, CANVAS_H))
        warped = crop_black(warped)   # 裁掉大画布的黑边，单独看更清楚

        out_full = os.path.join(OUT_DIR, f"{name}_corrected.bmp")
        save(out_full, warped)
        print(f"✅ {name}: yaw={yaw:+.3f}° pitch={pitch:+.3f}° -> {out_full}")

    print(f"\n完成。矫正结果在 {OUT_DIR}\\ 目录下：")
    print("  *_corrected.bmp        完整矫正图")


if __name__ == "__main__":
    main()