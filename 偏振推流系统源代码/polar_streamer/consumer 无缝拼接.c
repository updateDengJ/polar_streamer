// ==========================================
// 文件名: consumer.c
// 功能:
//   1. 四相机偏振图像实时拼接推流
//   2. 使用离线标定得到的刚性/仿射矩阵作为固定初始位姿
//   3. 初始化阶段预计算每个相机的逆向映射表和融合权重
//   4. 实时每帧只执行: 查表 warp + 亮度补偿 + mask 融合 + 固定 crop
//   5. 最终输出固定为 3600x4800，crop 区域在初始化时自动选择，尽量避开边缘黑块
//
// 说明:
//   - 这是纯 C 版本，不依赖 OpenCV。
//   - 你可以整体覆盖原来的 consumer.c。
//   - 仍保留原项目的 g_sync_box / g_cam_params / GStreamer 推流结构。
//   - 相机索引沿用你原来的约定:
//       0: 左下 BL/LD
//       1: 左上 TL/LU
//       2: 右下 BR/RD
//       3: 右上 TR/RU
// ==========================================

#include "consumer.h"
#include "config.h"
#include "ezsdk_core.h"
#include "remote_ctrl.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <omp.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// ==================== 输出尺寸配置 ====================
// 内部拼接画布: 保留足够空间承载四幅图的完整变换结果。
// 最终推流画面: 从内部画布中裁出 3600x4800，尽量避开黑边。
#define CANVAS_W 4096
#define CANVAS_H 4896
#define FINAL_W  3600
#define FINAL_H  4800

#define CANVAS_PIXELS ((size_t)CANVAS_W * (size_t)CANVAS_H)
#define FINAL_PIXELS  ((size_t)FINAL_W  * (size_t)FINAL_H)

// 亮度补偿采样步长。越小越准但更耗时；8 对 10fps 一般比较稳。
#define GAIN_SAMPLE_STEP 8

// crop 自动搜索步长。初始化时只跑一次，8 像素粒度已经够用。
#define CROP_SEARCH_STEP 8

// 几何动态参数变化超过这个阈值，才重建查表缓存。
// 如果 UDP 微调每帧都在变，这个阈值可以适当调大，避免频繁重建。
#define PARAM_EPS_T     0.01f
#define PARAM_EPS_THETA 0.00005f

// 距离权重使用的最大距离。只在初始化/重建缓存时计算，不进入每帧热路径。
#define DIST_INF 30000

static GstClockTime frame_duration = GST_SECOND / 10;  // 10fps: 100ms per frame
static GstClockTime current_pts = GST_SECOND;          // 从 1s 起步，避免部分播放器首帧时间戳异常

// ==================== 小工具函数 ====================

static inline uint8_t clamp_u8_float(float v) {
    if (v <= 0.0f) return 0;
    if (v >= 255.0f) return 255;
    return (uint8_t)(v + 0.5f);
}

static inline double get_time_diff_ms(struct timespec start, struct timespec end) {
    return (end.tv_sec - start.tv_sec) * 1000.0 +
           (end.tv_nsec - start.tv_nsec) / 1000000.0;
}

// 快速近似 atan2f，保留你原来代码里的优化版本。
static inline float fast_atan2f(float y, float x) {
    if (x == 0.0f) {
        return (y > 0.0f) ? (float)M_PI_2 : (y < 0.0f) ? -(float)M_PI_2 : 0.0f;
    }

    float abs_x = fabsf(x);
    float abs_y = fabsf(y);
    float a = (abs_x > abs_y) ? abs_y / abs_x : abs_x / abs_y;
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;

    if (abs_y > abs_x) r = (float)M_PI_2 - r;
    if (x < 0.0f) r = (float)M_PI - r;
    if (y < 0.0f) r = -r;
    return r;
}

// ==================== 3x3 矩阵定义与运算 ====================

typedef struct {
    double m[3][3];
} Mat3x3d;

static Mat3x3d mat3_identity(void) {
    Mat3x3d H = {{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    }};
    return H;
}

static Mat3x3d mat3_mul(Mat3x3d A, Mat3x3d B) {
    Mat3x3d C = {{{0}}};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            C.m[r][c] = A.m[r][0] * B.m[0][c] +
                        A.m[r][1] * B.m[1][c] +
                        A.m[r][2] * B.m[2][c];
        }
    }
    return C;
}

static Mat3x3d mat3_delta_from_udp(float tx, float ty, float theta) {
    // UDP 动态微调被解释为在 LU 统一坐标系下做的小刚体修正。
    double c = cos((double)theta);
    double s = sin((double)theta);
    Mat3x3d D = {{
        { c, -s, (double)tx},
        { s,  c, (double)ty},
        {0.0, 0.0, 1.0}
    }};
    return D;
}

static int mat3_inverse(Mat3x3d H, Mat3x3d* invH) {
    double a = H.m[0][0], b = H.m[0][1], c = H.m[0][2];
    double d = H.m[1][0], e = H.m[1][1], f = H.m[1][2];
    double g = H.m[2][0], h = H.m[2][1], i = H.m[2][2];

    double A =  (e * i - f * h);
    double B = -(d * i - f * g);
    double C =  (d * h - e * g);
    double D = -(b * i - c * h);
    double E =  (a * i - c * g);
    double F = -(a * h - b * g);
    double G =  (b * f - c * e);
    double Hc = -(a * f - c * d);
    double I =  (a * e - b * d);

    double det = a * A + b * B + c * C;
    if (fabs(det) < 1e-12) {
        return 0;
    }

    double inv_det = 1.0 / det;
    invH->m[0][0] = A * inv_det;
    invH->m[0][1] = D * inv_det;
    invH->m[0][2] = G * inv_det;
    invH->m[1][0] = B * inv_det;
    invH->m[1][1] = E * inv_det;
    invH->m[1][2] = Hc * inv_det;
    invH->m[2][0] = C * inv_det;
    invH->m[2][1] = F * inv_det;
    invH->m[2][2] = I * inv_det;
    return 1;
}

static inline int apply_homography(Mat3x3d H, double x, double y, double* ox, double* oy) {
    double w = H.m[2][0] * x + H.m[2][1] * y + H.m[2][2];
    if (fabs(w) < 1e-12) {
        return 0;
    }
    *ox = (H.m[0][0] * x + H.m[0][1] * y + H.m[0][2]) / w;
    *oy = (H.m[1][0] * x + H.m[1][1] * y + H.m[1][2]) / w;
    return 1;
}

// ==================== 离线初始矩阵 ====================

typedef struct {
    Mat3x3d H_LU_to_LU;
    Mat3x3d H_LD_to_LU;
    Mat3x3d H_RU_to_LU;
    Mat3x3d H_RD_to_LU;
} KnownHomographies;

static KnownHomographies get_known_homographies(void) {
    // 注意: 这里是标准 C 的二维数组初始化，不能写成 Mat H = {{{9 个数}}}。
    // 你前面 make 出现 excess elements warning，就是因为初始化层级少了一层。

    Mat3x3d H_LU_to_LU = {{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    }};

    // 右上 -> 左上
    Mat3x3d H_RU_to_LU = {{
        { 9.99934833e-01, -1.14161731e-02, 1.60728974e+03},
        { 1.14161731e-02,  9.99934833e-01, -3.84487442e+00},
        { 0.00000000e+00,  0.00000000e+00, 1.00000000e+00}
    }};

    // 右下 -> 左下
    Mat3x3d H_RD_to_LD = {{
        { 9.99997653e-01,  2.16637303e-03, 1.62378124e+03},
        {-2.16637303e-03,  9.99997653e-01, 2.98957637e+01},
        { 0.00000000e+00,  0.00000000e+00, 1.00000000e+00}
    }};

    // 左下 -> 左上
    Mat3x3d H_LD_to_LU = {{
        { 9.99650504e-01, -2.64361617e-02, 3.70244524e+01},
        { 2.64361617e-02,  9.99650504e-01, 2.31909710e+03},
        { 0.00000000e+00,  0.00000000e+00, 1.00000000e+00}
    }};

    // 右下 -> 右上
    Mat3x3d H_RD_to_RU = {{
        { 9.99999346e-01, -1.14375712e-03, 7.25192141e+01},
        { 1.14375712e-03,  9.99999346e-01, 2.39846605e+03},
        { 0.00000000e+00,  0.00000000e+00, 1.00000000e+00}
    }};

    // 右下到左上有两条路径:
    //   路径 A: RD -> RU -> LU
    //   路径 B: RD -> LD -> LU
    // 为了减少单一路径误差，取两条路径的平移/旋转近似平均。
    Mat3x3d H_RD_to_LU_via_right  = mat3_mul(H_RU_to_LU, H_RD_to_RU);
    Mat3x3d H_RD_to_LU_via_bottom = mat3_mul(H_LD_to_LU, H_RD_to_LD);

    Mat3x3d H_RD_to_LU = mat3_identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            H_RD_to_LU.m[r][c] =
                0.5 * (H_RD_to_LU_via_right.m[r][c] + H_RD_to_LU_via_bottom.m[r][c]);
        }
    }

    // 如果右下角仍然肉眼感觉偏移，可以只微调这两个值。
    // 负数含义: 往左/往上移动右下图。
    H_RD_to_LU.m[0][2] -= 10.0;  // 右下图左移 10 像素
    H_RD_to_LU.m[1][2] -= 5.0;   // 右下图上移 5 像素

    KnownHomographies kh;
    kh.H_LU_to_LU = H_LU_to_LU;
    kh.H_LD_to_LU = H_LD_to_LU;
    kh.H_RU_to_LU = H_RU_to_LU;
    kh.H_RD_to_LU = H_RD_to_LU;
    return kh;
}

// ==================== 查表缓存结构 ====================

typedef struct {
    // 对内部画布每个像素，记录它来自源图哪个 2x2 偏振块的左上角索引。
    // -1 表示该相机不覆盖这个画布像素。
    int32_t* src_idx;

    // RAW 模式下，记录每个画布像素应该取 2x2 偏振块中的哪一个值:
    // 0=i90, 1=i45, 2=i135, 3=i0。
    uint8_t* raw_sel;

    // 该相机在内部画布上的有效区域 mask。
    uint8_t* mask;

    // 该相机与当前已融合画面重叠的区域。
    uint8_t* overlap;

    // 预计算融合权重，范围 0~255。
    // 0   表示完全用旧画面
    // 255 表示完全用当前相机
    uint8_t* weight_new;
} CameraWarpCache;

typedef struct {
    int initialized;
    int single_w;
    int single_h;
    int crop_x;
    int crop_y;

    CameraWarpCache cam[CAM_COUNT];
    uint8_t* union_mask;

    CameraDynamicParams cached_params[CAM_COUNT];

    // 每个相机的亮度补偿做时间平滑，避免曝光变化或统计波动导致画面突然跳亮/跳暗。
    float smooth_gain[CAM_COUNT];
    float smooth_bias[CAM_COUNT];
    uint8_t smooth_valid[CAM_COUNT];
} RuntimeStitchCache;

// ==================== 内存申请与释放 ====================

static int alloc_camera_cache(CameraWarpCache* c) {
    c->src_idx = (int32_t*)malloc(CANVAS_PIXELS * sizeof(int32_t));
    c->raw_sel = (uint8_t*)malloc(CANVAS_PIXELS);
    c->mask = (uint8_t*)malloc(CANVAS_PIXELS);
    c->overlap = (uint8_t*)malloc(CANVAS_PIXELS);
    c->weight_new = (uint8_t*)malloc(CANVAS_PIXELS);

    if (!c->src_idx || !c->raw_sel || !c->mask || !c->overlap || !c->weight_new) {
        return 0;
    }
    return 1;
}

static void free_camera_cache(CameraWarpCache* c) {
    free(c->src_idx);
    free(c->raw_sel);
    free(c->mask);
    free(c->overlap);
    free(c->weight_new);

    c->src_idx = NULL;
    c->raw_sel = NULL;
    c->mask = NULL;
    c->overlap = NULL;
    c->weight_new = NULL;
}

static int alloc_runtime_cache(RuntimeStitchCache* st) {
    memset(st, 0, sizeof(*st));

    for (int i = 0; i < CAM_COUNT; ++i) {
        if (!alloc_camera_cache(&st->cam[i])) {
            return 0;
        }
    }

    st->union_mask = (uint8_t*)malloc(CANVAS_PIXELS);
    if (!st->union_mask) {
        return 0;
    }

    return 1;
}

static void free_runtime_cache(RuntimeStitchCache* st) {
    if (!st) return;

    for (int i = 0; i < CAM_COUNT; ++i) {
        free_camera_cache(&st->cam[i]);
    }

    free(st->union_mask);
    st->union_mask = NULL;
    st->initialized = 0;
}

// ==================== 偏振物理量计算 ====================

static inline uint8_t sample_polar_value(uint8_t* src,
                                         int src_idx,
                                         int src_w,
                                         uint8_t raw_sel,
                                         uint8_t mode) {
    // 源图 2x2 偏振排列沿用原代码:
    //   [i90,  i45]
    //   [i135, i0 ]
    float i90  = (float)src[src_idx];
    float i45  = (float)src[src_idx + 1];
    float i135 = (float)src[src_idx + src_w];
    float i0   = (float)src[src_idx + src_w + 1];

    if (mode == MODE_RAW) {
        switch (raw_sel) {
            case 0: return (uint8_t)i90;
            case 1: return (uint8_t)i45;
            case 2: return (uint8_t)i135;
            default: return (uint8_t)i0;
        }
    }

    float S0 = 0.25f * (i0 + i45 + i90 + i135);
    float S1 = i0 - i90;
    float S2 = i45 - i135;

    if (mode == MODE_AOLP) {
        if (fabsf(S1) < 1e-6f && fabsf(S2) < 1e-6f) {
            return 128;
        }
        float aolp = 0.5f * fast_atan2f(S2, S1);
        float wrapped = fmodf(aolp, (float)M_PI);
        if (wrapped < 0.0f) wrapped += (float)M_PI;
        return clamp_u8_float((wrapped / (float)M_PI) * 255.0f);
    }

    if (mode == MODE_DOLP) {
        float mag = sqrtf(S1 * S1 + S2 * S2);
        return clamp_u8_float((mag / (S0 + 1e-6f)) * 255.0f);
    }

    // 默认 S0 强度图。
    return clamp_u8_float(S0);
}

// ==================== 几何预计算 ====================

static int rectified_xy_to_source_block(double rx,
                                        double ry,
                                        int src_w,
                                        int src_h,
                                        float physical_theta,
                                        int* src_idx,
                                        uint8_t* raw_sel) {
    // 这里把“离线拼接坐标系中的相机图像坐标”反解回原始相机 RAW 坐标。
    // 原代码中左侧相机 base_theta = +90 度，右侧相机 base_theta = -90 度。
    double c = cos((double)physical_theta);
    double s = sin((double)physical_theta);

    double cx = (double)src_w * 0.5;
    double cy = (double)src_h * 0.5;

    // 旋转后外接矩形尺寸，与原 process_polar_dynamic_warp_gray 一致。
    double out_w = fabs(c) * (double)src_w + fabs(s) * (double)src_h;
    double out_h = fabs(s) * (double)src_w + fabs(c) * (double)src_h;
    double dst_cx = out_w * 0.5;
    double dst_cy = out_h * 0.5;

    // 逆旋转: rectified 坐标 -> 原始源图坐标。
    double dx = rx - dst_cx;
    double dy = ry - dst_cy;
    double sx_f =  c * dx + s * dy + cx;
    double sy_f = -s * dx + c * dy + cy;

    int sx = ((int)floor(sx_f)) & ~1;
    int sy = ((int)floor(sy_f)) & ~1;

    if (sx < 0 || sx + 1 >= src_w || sy < 0 || sy + 1 >= src_h) {
        return 0;
    }

    *src_idx = sy * src_w + sx;

    // RAW 模式下保留原代码左右相机 2x2 重排逻辑。
    // 画布像素奇偶决定取 2x2 中哪个偏振分量。
    // 左相机 physical_theta > 0:
    //   dst TL/TR/BL/BR = i135/i90/i0/i45
    // 右相机 physical_theta < 0:
    //   dst TL/TR/BL/BR = i45/i0/i90/i135
    // 这里 raw_sel 的具体赋值在 build_camera_warp_map 中结合画布像素奇偶完成。
    *raw_sel = 0;
    return 1;
}

static uint8_t raw_selector_for_canvas_pixel(int x, int y, float physical_theta) {
    int px = x & 1;
    int py = y & 1;

    if (physical_theta > 0.0f) {
        if (py == 0 && px == 0) return 2;  // i135
        if (py == 0 && px == 1) return 0;  // i90
        if (py == 1 && px == 0) return 3;  // i0
        return 1;                          // i45
    }

    if (py == 0 && px == 0) return 1;      // i45
    if (py == 0 && px == 1) return 3;      // i0
    if (py == 1 && px == 0) return 0;      // i90
    return 2;                              // i135
}

static void build_camera_warp_map(CameraWarpCache* cache,
                                  int src_w,
                                  int src_h,
                                  Mat3x3d H_cam_to_LU,
                                  float physical_theta,
                                  CameraDynamicParams* params) {
    Mat3x3d delta = mat3_delta_from_udp(params->tx, params->ty, params->theta_rad);
    Mat3x3d H_eff = mat3_mul(delta, H_cam_to_LU);
    Mat3x3d invH;

    if (!mat3_inverse(H_eff, &invH)) {
        fprintf(stderr, "Error: homography inverse failed, camera map disabled.\n");
        memset(cache->src_idx, 0xFF, CANVAS_PIXELS * sizeof(int32_t));
        memset(cache->raw_sel, 0, CANVAS_PIXELS);
        memset(cache->mask, 0, CANVAS_PIXELS);
        return;
    }

    #pragma omp parallel for schedule(static)
    for (int y = 0; y < CANVAS_H; ++y) {
        size_t row = (size_t)y * (size_t)CANVAS_W;
        for (int x = 0; x < CANVAS_W; ++x) {
            double rx = 0.0;
            double ry = 0.0;
            int src_idx = -1;
            uint8_t sel = 0;

            size_t p = row + (size_t)x;

            if (!apply_homography(invH, (double)x, (double)y, &rx, &ry)) {
                cache->src_idx[p] = -1;
                cache->raw_sel[p] = 0;
                cache->mask[p] = 0;
                continue;
            }

            if (!rectified_xy_to_source_block(rx, ry, src_w, src_h,
                                              physical_theta, &src_idx, &sel)) {
                cache->src_idx[p] = -1;
                cache->raw_sel[p] = 0;
                cache->mask[p] = 0;
                continue;
            }

            cache->src_idx[p] = src_idx;
            cache->raw_sel[p] = raw_selector_for_canvas_pixel(x, y, physical_theta);
            cache->mask[p] = 255;
        }
    }
}

static int params_changed_for_cache(RuntimeStitchCache* st, CameraDynamicParams* params) {
    if (!st->initialized) {
        return 1;
    }

    for (int i = 0; i < CAM_COUNT; ++i) {
        float dtx = fabsf(params[i].tx - st->cached_params[i].tx);
        float dty = fabsf(params[i].ty - st->cached_params[i].ty);
        float dtheta = fabsf(params[i].theta_rad - st->cached_params[i].theta_rad);

        if (dtx > PARAM_EPS_T || dty > PARAM_EPS_T || dtheta > PARAM_EPS_THETA) {
            return 1;
        }
    }

    return 0;
}

// ==================== 融合权重预计算 ====================

static void compute_chamfer_distance_u16(const uint8_t* mask, uint16_t* dist) {
    // 纯 C 近似 distanceTransform:
    //   mask > 0 的有效区域内，计算到无效区域边界的近似距离。
    //   mask == 0 的位置距离为 0。
    //
    // OpenCV 的 distanceTransform 更精确，但实时项目是纯 C。
    // 这里用两遍 4 邻域距离，初始化阶段计算一次，效果足够用于 feather 权重。
    #pragma omp parallel for schedule(static)
    for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
        dist[p] = mask[p] ? DIST_INF : 0;
    }

    for (int y = 0; y < CANVAS_H; ++y) {
        for (int x = 0; x < CANVAS_W; ++x) {
            size_t p = (size_t)y * (size_t)CANVAS_W + (size_t)x;
            uint16_t d = dist[p];
            if (x > 0) {
                uint16_t v = (uint16_t)(dist[p - 1] + 1);
                if (v < d) d = v;
            }
            if (y > 0) {
                uint16_t v = (uint16_t)(dist[p - CANVAS_W] + 1);
                if (v < d) d = v;
            }
            dist[p] = d;
        }
    }

    for (int y = CANVAS_H - 1; y >= 0; --y) {
        for (int x = CANVAS_W - 1; x >= 0; --x) {
            size_t p = (size_t)y * (size_t)CANVAS_W + (size_t)x;
            uint16_t d = dist[p];
            if (x + 1 < CANVAS_W) {
                uint16_t v = (uint16_t)(dist[p + 1] + 1);
                if (v < d) d = v;
            }
            if (y + 1 < CANVAS_H) {
                uint16_t v = (uint16_t)(dist[p + CANVAS_W] + 1);
                if (v < d) d = v;
            }
            dist[p] = d;
        }
    }
}

static void build_simple_blend_weights(RuntimeStitchCache* st) {
    memset(st->union_mask, 0, CANVAS_PIXELS);
    for (int i = 0; i < CAM_COUNT; ++i) {
        memset(st->cam[i].overlap, 0, CANVAS_PIXELS);
        memset(st->cam[i].weight_new, 0, CANVAS_PIXELS);
    }

    // 融合顺序:
    //   1. 左上作为基准
    //   2. 左下和左上做上下融合
    //   3. 右上和左上做左右融合
    //   4. 右下最后进入，同时照顾右侧上下接缝和底部横向接缝
    int order[CAM_COUNT] = {1, 0, 3, 2};

    for (int oi = 0; oi < CAM_COUNT; ++oi) {
        int ci = order[oi];
        CameraWarpCache* c = &st->cam[ci];

        if (oi == 0) {
            // 第一幅基准图，完全使用当前图。
            #pragma omp parallel for schedule(static)
            for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
                if (c->mask[p]) {
                    c->weight_new[p] = 255;
                    st->union_mask[p] = 255;
                }
            }
            continue;
        }

        // 先标出真实重叠区。后面的权重只在真实 overlap 里变化，
        // 这就是它和旧版“全画布 x/y 粗略渐变”的本质区别。
        #pragma omp parallel for schedule(static)
        for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
            if (!c->mask[p]) {
                c->weight_new[p] = 0;
                c->overlap[p] = 0;
            } else if (!st->union_mask[p]) {
                c->weight_new[p] = 255;
                c->overlap[p] = 0;
            } else {
                c->weight_new[p] = 128;
                c->overlap[p] = 255;
            }
        }

        if (ci == 0) {
            // LD: 上下拼接。对每一列，只在该列的真实重叠段内从上到下渐变。
            #pragma omp parallel for schedule(static)
            for (int x = 0; x < CANVAS_W; ++x) {
                int first = -1;
                int last = -1;
                for (int y = 0; y < CANVAS_H; ++y) {
                    size_t p = (size_t)y * (size_t)CANVAS_W + (size_t)x;
                    if (c->overlap[p]) {
                        if (first < 0) first = y;
                        last = y;
                    }
                }

                if (first < 0) continue;
                int len = last - first;
                for (int y = first; y <= last; ++y) {
                    size_t p = (size_t)y * (size_t)CANVAS_W + (size_t)x;
                    if (!c->overlap[p]) continue;
                    int w = (len <= 0) ? 128 : (int)((double)(y - first) * 255.0 / (double)len);
                    if (w < 0) w = 0;
                    if (w > 255) w = 255;
                    c->weight_new[p] = (uint8_t)w;
                }
            }
        } else if (ci == 3) {
            // RU: 左右拼接。对每一行，只在该行的真实重叠段内从左到右渐变。
            #pragma omp parallel for schedule(static)
            for (int y = 0; y < CANVAS_H; ++y) {
                int first = -1;
                int last = -1;
                size_t row = (size_t)y * (size_t)CANVAS_W;
                for (int x = 0; x < CANVAS_W; ++x) {
                    size_t p = row + (size_t)x;
                    if (c->overlap[p]) {
                        if (first < 0) first = x;
                        last = x;
                    }
                }

                if (first < 0) continue;
                int len = last - first;
                for (int x = first; x <= last; ++x) {
                    size_t p = row + (size_t)x;
                    if (!c->overlap[p]) continue;
                    int w = (len <= 0) ? 128 : (int)((double)(x - first) * 255.0 / (double)len);
                    if (w < 0) w = 0;
                    if (w > 255) w = 255;
                    c->weight_new[p] = (uint8_t)w;
                }
            }
        } else if (ci == 2) {
            // RD: 角落图最容易露接缝，使用接近 pinjie.cpp 的距离权重。
            // new_weight = dist_new / (dist_old + dist_new)
            uint16_t* dist_old = (uint16_t*)malloc(CANVAS_PIXELS * sizeof(uint16_t));
            uint16_t* dist_new = (uint16_t*)malloc(CANVAS_PIXELS * sizeof(uint16_t));

            if (dist_old && dist_new) {
                compute_chamfer_distance_u16(st->union_mask, dist_old);
                compute_chamfer_distance_u16(c->mask, dist_new);

                #pragma omp parallel for schedule(static)
                for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
                    if (!c->overlap[p]) continue;

                    int d_old = (int)dist_old[p];
                    int d_new = (int)dist_new[p];
                    int denom = d_old + d_new;
                    int w = (denom <= 0) ? 128 : (d_new * 255) / denom;

                    if (w < 0) w = 0;
                    if (w > 255) w = 255;
                    c->weight_new[p] = (uint8_t)w;
                }
            } else {
                fprintf(stderr, "Warning: distance weight allocation failed, RD uses 50/50 overlap blend.\n");
            }

            free(dist_old);
            free(dist_new);
        }

        // 更新全局覆盖 mask。
        #pragma omp parallel for schedule(static)
        for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
            if (c->mask[p]) {
                st->union_mask[p] = 255;
            }
        }
    }
}

static void find_best_crop_once(RuntimeStitchCache* st) {
    // 从 union mask 里找一个 3600x4800 的固定窗口。
    // 目标不是“保留最多画面”这么简单，而是优先让四条边尽量不是黑块，
    // 因为黑块最容易出现在最终画面边缘。
    int best_x = 0;
    int best_y = 0;
    long long best_score = -1;

    int max_x = CANVAS_W - FINAL_W;
    int max_y = CANVAS_H - FINAL_H;

    if (max_x < 0 || max_y < 0) {
        st->crop_x = 0;
        st->crop_y = 0;
        return;
    }

    for (int y0 = 0; y0 <= max_y; y0 += CROP_SEARCH_STEP) {
        for (int x0 = 0; x0 <= max_x; x0 += CROP_SEARCH_STEP) {
            long long score = 0;

            // 采样窗口内部与边缘。边缘权重大，尽量避开明显黑边。
            for (int yy = 0; yy < FINAL_H; yy += 16) {
                int y = y0 + yy;
                const uint8_t* row = st->union_mask + (size_t)y * (size_t)CANVAS_W + (size_t)x0;
                for (int xx = 0; xx < FINAL_W; xx += 16) {
                    int edge_bonus = (xx < 64 || yy < 64 || xx > FINAL_W - 65 || yy > FINAL_H - 65) ? 6 : 1;
                    score += row[xx] ? edge_bonus : 0;
                }
            }

            if (score > best_score) {
                best_score = score;
                best_x = x0;
                best_y = y0;
            }
        }
    }

    st->crop_x = best_x;
    st->crop_y = best_y;
    printf("[StitchCache] fixed crop: x=%d, y=%d, size=%dx%d\n",
           st->crop_x, st->crop_y, FINAL_W, FINAL_H);
}

static int init_or_rebuild_runtime_cache(RuntimeStitchCache* st,
                                         int single_w,
                                         int single_h,
                                         CameraDynamicParams* params) {
    if (!st->union_mask) {
        if (!alloc_runtime_cache(st)) {
            fprintf(stderr, "Error: failed to allocate runtime stitch cache.\n");
            return 0;
        }
    }

    KnownHomographies kh = get_known_homographies();

    st->single_w = single_w;
    st->single_h = single_h;

    // 相机索引:
    //   1: LU/TL, 左上，物理旋转 +90
    //   0: LD/BL, 左下，物理旋转 +90
    //   3: RU/TR, 右上，物理旋转 -90
    //   2: RD/BR, 右下，物理旋转 -90
    build_camera_warp_map(&st->cam[1], single_w, single_h, kh.H_LU_to_LU,  (float)M_PI_2,  &params[1]);
    build_camera_warp_map(&st->cam[0], single_w, single_h, kh.H_LD_to_LU,  (float)M_PI_2,  &params[0]);
    build_camera_warp_map(&st->cam[3], single_w, single_h, kh.H_RU_to_LU, -(float)M_PI_2,  &params[3]);
    build_camera_warp_map(&st->cam[2], single_w, single_h, kh.H_RD_to_LU, -(float)M_PI_2,  &params[2]);

    build_simple_blend_weights(st);
    find_best_crop_once(st);

    memcpy(st->cached_params, params, sizeof(CameraDynamicParams) * CAM_COUNT);
    st->initialized = 1;
    return 1;
}

// ==================== 每帧查表渲染 ====================

static void render_camera_by_map(uint8_t* src,
                                 uint8_t* cam_y,
                                 CameraWarpCache* cache,
                                 int src_w,
                                 uint8_t mode) {
    #pragma omp parallel for schedule(static)
    for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
        int src_idx = cache->src_idx[p];
        if (src_idx < 0) {
            cam_y[p] = 0;
            continue;
        }

        cam_y[p] = sample_polar_value(src, src_idx, src_w, cache->raw_sel[p], mode);
    }
}

static void estimate_gain_bias(const uint8_t* base_y,
                               const uint8_t* cam_y,
                               const uint8_t* valid_mask,
                               const uint8_t* overlap_mask,
                               float* gain,
                               float* bias) {
    // 尽量复刻 pinjie.cpp 的亮度补偿:
    //   gain = std_old / std_new
    //   bias = mean_old - gain * mean_new
    //
    // 但实时灰度偏振图比离线 RGB 更敏感，所以这里额外做两件事:
    //   1. 只统计真实 overlap 里的有效像素。
    //   2. 排除黑边、过曝点、极暗点，避免 bias 被黑块拉偏导致灰蒙蒙。
    double sum_b = 0.0;
    double sum_c = 0.0;
    double sum_b2 = 0.0;
    double sum_c2 = 0.0;
    int n = 0;

    for (int y = 0; y < CANVAS_H; y += GAIN_SAMPLE_STEP) {
        size_t row = (size_t)y * (size_t)CANVAS_W;
        for (int x = 0; x < CANVAS_W; x += GAIN_SAMPLE_STEP) {
            size_t p = row + (size_t)x;
            if (!valid_mask[p] || !overlap_mask[p]) {
                continue;
            }

            int b = (int)base_y[p];
            int c = (int)cam_y[p];

            // 严格排除黑边和过曝点。
            // 如果阈值太低，黑边会把 bias 往正方向拉，导致整幅图灰。
            if (b <= 12 || c <= 12 || b >= 245 || c >= 245) {
                continue;
            }

            sum_b += (double)b;
            sum_c += (double)c;
            sum_b2 += (double)b * (double)b;
            sum_c2 += (double)c * (double)c;
            ++n;
        }
    }

    if (n < 64) {
        *gain = 1.0f;
        *bias = 0.0f;
        return;
    }

    double mean_b = sum_b / (double)n;
    double mean_c = sum_c / (double)n;
    double var_b = sum_b2 / (double)n - mean_b * mean_b;
    double var_c = sum_c2 / (double)n - mean_c * mean_c;

    if (var_b < 1.0 || var_c < 1.0) {
        *gain = 1.0f;
        *bias = 0.0f;
        return;
    }

    double std_b = sqrt(var_b);
    double std_c = sqrt(var_c);
    double g = std_b / std_c;
    double b0 = mean_b - g * mean_c;

    // 防止偶发错误重叠导致补偿过猛。
    // 这里比 pinjie.cpp 略收紧，兼顾“消缝”和“不灰”。
    if (g < 0.65) g = 0.65;
    if (g > 1.55) g = 1.55;
    if (b0 < -28.0) b0 = -28.0;
    if (b0 >  28.0) b0 =  28.0;

    *gain = (float)g;
    *bias = (float)b0;
}

static void blend_camera_into_canvas(uint8_t* pano_y,
                                     uint8_t* pano_mask,
                                     const uint8_t* cam_y,
                                     CameraWarpCache* cache,
                                     RuntimeStitchCache* st,
                                     int cam_idx) {
    float gain = 1.0f;
    float bias = 0.0f;

    estimate_gain_bias(pano_y, cam_y, cache->mask, cache->overlap, &gain, &bias);

    // 亮度跳变的主要来源:
    //   每帧 overlap 内容不同、曝光突然变化、黑边/运动目标进入统计区域，
    //   都会让本帧估计出的 gain/bias 抖动。
    //
    // 这里做两层保护:
    //   1. 限制单帧最大变化量，防止突然跳。
    //   2. 指数平滑，避免帧间闪烁。
    if (!st->smooth_valid[cam_idx]) {
        st->smooth_gain[cam_idx] = gain;
        st->smooth_bias[cam_idx] = bias;
        st->smooth_valid[cam_idx] = 1;
    } else {
        float prev_gain = st->smooth_gain[cam_idx];
        float prev_bias = st->smooth_bias[cam_idx];

        float dg = gain - prev_gain;
        float db = bias - prev_bias;

        if (dg > 0.06f) dg = 0.06f;
        if (dg < -0.06f) dg = -0.06f;
        if (db > 3.0f) db = 3.0f;
        if (db < -3.0f) db = -3.0f;

        float limited_gain = prev_gain + dg;
        float limited_bias = prev_bias + db;

        const float alpha = 0.18f;
        st->smooth_gain[cam_idx] = prev_gain * (1.0f - alpha) + limited_gain * alpha;
        st->smooth_bias[cam_idx] = prev_bias * (1.0f - alpha) + limited_bias * alpha;

        gain = st->smooth_gain[cam_idx];
        bias = st->smooth_bias[cam_idx];
    }

    #pragma omp parallel for schedule(static)
    for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
        if (!cache->mask[p]) {
            continue;
        }

        // 效果优先版本:
        //   尽量复刻 pinjie.cpp，对新图全部有效区域应用 gain + bias。
        //   这样接缝两侧不会出现“重叠区一套亮度、非重叠区另一套亮度”的断层。
        //   灰蒙蒙问题主要靠 estimate_gain_bias 里的黑边/过曝排除和限幅来控制。
        float corrected_f = (float)cam_y[p] * gain + bias;
        uint8_t corrected = clamp_u8_float(corrected_f);

        if (!pano_mask[p]) {
            pano_y[p] = corrected;
            pano_mask[p] = 255;
            continue;
        }

        uint8_t w = cache->weight_new[p];
        uint16_t old_part = (uint16_t)pano_y[p] * (uint16_t)(255 - w);
        uint16_t new_part = (uint16_t)corrected * (uint16_t)w;
        pano_y[p] = (uint8_t)((old_part + new_part + 127) / 255);
        pano_mask[p] = 255;
    }
}

static void crop_canvas_to_nv12(uint8_t* pano_y,
                                uint8_t* nv12,
                                int crop_x,
                                int crop_y) {
    // Y 平面: 从内部画布固定裁剪出 3600x4800。
    for (int y = 0; y < FINAL_H; ++y) {
        const uint8_t* src_row = pano_y + (size_t)(crop_y + y) * (size_t)CANVAS_W + (size_t)crop_x;
        uint8_t* dst_row = nv12 + (size_t)y * (size_t)FINAL_W;
        memcpy(dst_row, src_row, FINAL_W);
    }

    // UV 平面: 灰度图推流，UV 固定 128。
    memset(nv12 + FINAL_PIXELS, 128, FINAL_PIXELS / 2);
}

// ==================== 主拼接推流线程 ====================

void* StitchingThreadFunc(void* arg) {
    (void)arg;

    HIMAGE local_images[CAM_COUNT] = {NULL};
    GstElement* pipeline = NULL;
    GstElement* appsrc = NULL;
    bool is_initialized = false;
    int single_w = 0;
    int single_h = 0;

    RuntimeStitchCache stitch_cache;
    memset(&stitch_cache, 0, sizeof(stitch_cache));

    uint8_t* pano_y = NULL;
    uint8_t* pano_mask = NULL;
    uint8_t* cam_y[CAM_COUNT] = {NULL};

    struct timespec fps_start_time;
    struct timespec current_time;
    struct timespec process_start;
    struct timespec process_end;
    int frame_count = 0;
    double total_process_ms = 0.0;

    while (g_running) {
        pthread_mutex_lock(&g_sync_box.lock);
        while (g_sync_box.ready_mask != 15 && g_running) {
            pthread_cond_wait(&g_sync_box.cond, &g_sync_box.lock);
        }

        if (!g_running) {
            pthread_mutex_unlock(&g_sync_box.lock);
            break;
        }

        for (int i = 0; i < CAM_COUNT; ++i) {
            local_images[i] = g_sync_box.images[i];
            g_sync_box.images[i] = NULL;
        }
        g_sync_box.ready_mask = 0;
        pthread_mutex_unlock(&g_sync_box.lock);

        CameraDynamicParams local_params[CAM_COUNT];
        pthread_mutex_lock(&g_params_lock);
        memcpy(local_params, g_cam_params, sizeof(CameraDynamicParams) * CAM_COUNT);
        pthread_mutex_unlock(&g_params_lock);

        if (!is_initialized) {
            ImageInfo info = g_Image_GetImageInfo(local_images[0]);
            single_w = info.width;
            single_h = info.height;

            printf("Initializing stitch stream: single=%dx%d, canvas=%dx%d, output=%dx%d\n",
                   single_w, single_h, CANVAS_W, CANVAS_H, FINAL_W, FINAL_H);

            pano_y = (uint8_t*)malloc(CANVAS_PIXELS);
            pano_mask = (uint8_t*)malloc(CANVAS_PIXELS);
            for (int i = 0; i < CAM_COUNT; ++i) {
                cam_y[i] = (uint8_t*)malloc(CANVAS_PIXELS);
            }

            if (!pano_y || !pano_mask || !cam_y[0] || !cam_y[1] || !cam_y[2] || !cam_y[3]) {
                fprintf(stderr, "Error: failed to allocate frame buffers.\n");
                goto cleanup_frame;
            }

            if (!init_or_rebuild_runtime_cache(&stitch_cache, single_w, single_h, local_params)) {
                fprintf(stderr, "Error: failed to initialize stitch cache.\n");
                goto cleanup_frame;
            }

            char gst_pipeline_str[1024];
            snprintf(gst_pipeline_str, sizeof(gst_pipeline_str),
                "appsrc name=mysrc is-live=true format=TIME do-timestamp=true block=false "
                "caps=video/x-raw,format=NV12,width=%d,height=%d,framerate=10/1 ! "
                "queue name=input_queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
                "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
                "nvv4l2h265enc bitrate=1000000 control-rate=1 maxperf-enable=1 insert-sps-pps=true idrinterval=10 ! "
                "h265parse config-interval=-1 ! "
                "rtph265pay pt=96 mtu=1400 ! "
                "queue name=udp_queue max-size-buffers=50 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
                "udpsink host=192.168.100.10 port=5000 sync=false async=true "
                "buffer-size=2097152 max-bitrate=0 qos-dscp=0",
                FINAL_W, FINAL_H);

            GError* error = NULL;
            pipeline = gst_parse_launch(gst_pipeline_str, &error);
            if (error || !pipeline) {
                fprintf(stderr, "GStreamer Pipeline Error: %s\n",
                        error ? error->message : "unknown error");
                if (error) g_error_free(error);
                goto cleanup_frame;
            }

            appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "mysrc");
            if (!appsrc) {
                fprintf(stderr, "Error: failed to get appsrc.\n");
                goto cleanup_frame;
            }

            g_object_set(appsrc, "block", FALSE, NULL);
            gst_element_set_state(pipeline, GST_STATE_PLAYING);

            clock_gettime(CLOCK_MONOTONIC, &fps_start_time);
            current_pts = GST_SECOND;
            is_initialized = true;
        }

        clock_gettime(CLOCK_MONOTONIC, &process_start);

        // 如果 UDP 几何微调发生变化，需要重建逆向映射表和融合权重。
        // 注意: 模式 MODE_S0/MODE_RAW/MODE_AOLP/MODE_DOLP 改变不需要重建查表。
        if (params_changed_for_cache(&stitch_cache, local_params)) {
            printf("[StitchCache] geometry params changed, rebuilding maps...\n");
            if (!init_or_rebuild_runtime_cache(&stitch_cache, single_w, single_h, local_params)) {
                fprintf(stderr, "Error: failed to rebuild stitch cache.\n");
                goto cleanup_frame;
            }
        }

        int nv12_size = FINAL_W * FINAL_H * 3 / 2;
        GstBuffer* buffer = gst_buffer_new_allocate(NULL, nv12_size, NULL);
        if (!buffer) {
            goto cleanup_frame;
        }

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
            gst_buffer_unref(buffer);
            goto cleanup_frame;
        }

        uint8_t* dst_nv12 = map.data;

        // 当前帧清空内部拼接画布。最终黑边问题主要靠固定 crop 避免。
        memset(pano_y, 0, CANVAS_PIXELS);
        memset(pano_mask, 0, CANVAS_PIXELS);

        // 获取图像指针。索引沿用你原来的代码:
        //   0: 左下, 1: 左上, 2: 右下, 3: 右上
        uint8_t* src_BL = (uint8_t*)g_Image_GetImageBuff(local_images[0]);
        uint8_t* src_TL = (uint8_t*)g_Image_GetImageBuff(local_images[1]);
        uint8_t* src_BR = (uint8_t*)g_Image_GetImageBuff(local_images[2]);
        uint8_t* src_TR = (uint8_t*)g_Image_GetImageBuff(local_images[3]);

        // 1. 查表 warp，把四路相机分别渲染到统一 LU 坐标系内部画布。
        render_camera_by_map(src_TL, cam_y[1], &stitch_cache.cam[1], single_w, local_params[1].mode);
        render_camera_by_map(src_BL, cam_y[0], &stitch_cache.cam[0], single_w, local_params[0].mode);
        render_camera_by_map(src_TR, cam_y[3], &stitch_cache.cam[3], single_w, local_params[3].mode);
        render_camera_by_map(src_BR, cam_y[2], &stitch_cache.cam[2], single_w, local_params[2].mode);

        // 2. 按固定顺序融合。每加入一幅图，先在重叠区估计亮度补偿，再做 feather。
        blend_camera_into_canvas(pano_y, pano_mask, cam_y[1], &stitch_cache.cam[1], &stitch_cache, 1);
        blend_camera_into_canvas(pano_y, pano_mask, cam_y[0], &stitch_cache.cam[0], &stitch_cache, 0);
        blend_camera_into_canvas(pano_y, pano_mask, cam_y[3], &stitch_cache.cam[3], &stitch_cache, 3);
        blend_camera_into_canvas(pano_y, pano_mask, cam_y[2], &stitch_cache.cam[2], &stitch_cache, 2);

        // 3. 从内部画布固定裁剪到 3600x4800，并写成 NV12。
        crop_canvas_to_nv12(pano_y, dst_nv12, stitch_cache.crop_x, stitch_cache.crop_y);

        gst_buffer_unmap(buffer, &map);

        GST_BUFFER_PTS(buffer) = current_pts;
        GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DURATION(buffer) = frame_duration;
        current_pts += frame_duration;

        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        if (ret != GST_FLOW_OK) {
            gst_buffer_unref(buffer);
        }

        clock_gettime(CLOCK_MONOTONIC, &process_end);
        total_process_ms += get_time_diff_ms(process_start, process_end);

        if (++frame_count == 10) {
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double elapsed = get_time_diff_ms(fps_start_time, current_time);
            printf("[PolarStream] Output:%dx%d | FPS:%.2f | Latency:%.2fms | crop=(%d,%d)\n",
                   FINAL_W, FINAL_H,
                   (10.0 / elapsed) * 1000.0,
                   total_process_ms / 10.0,
                   stitch_cache.crop_x,
                   stitch_cache.crop_y);

            total_process_ms = 0.0;
            frame_count = 0;
            fps_start_time = current_time;
        }

cleanup_frame:
        for (int i = 0; i < CAM_COUNT; ++i) {
            if (local_images[i]) {
                g_Image_Recycle(local_images[i]);
                local_images[i] = NULL;
            }
        }
    }

    if (appsrc) {
        gst_object_unref(appsrc);
    }

    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }

    free_runtime_cache(&stitch_cache);

    free(pano_y);
    free(pano_mask);
    for (int i = 0; i < CAM_COUNT; ++i) {
        free(cam_y[i]);
    }

    printf("StitchingThreadFunc exiting\n");
    return NULL;
}
