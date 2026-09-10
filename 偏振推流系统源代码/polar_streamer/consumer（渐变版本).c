// ==========================================
// 文件名: consumer.c
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
#include <sys/time.h>
#include <inttypes.h>

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
#define CANVAS_W 4096
#define CANVAS_H 4896
#define FINAL_W  3600
#define FINAL_H  4800

#define CANVAS_PIXELS ((size_t)CANVAS_W * (size_t)CANVAS_H)
#define FINAL_PIXELS  ((size_t)FINAL_W  * (size_t)FINAL_H)

#define GAIN_SAMPLE_STEP 8
#define CROP_SEARCH_STEP 8

#define PARAM_EPS_T     0.01f
#define PARAM_EPS_THETA 0.00005f

#define DIST_INF 30000

#define POLAR_DIFF_MIN    (-255)
#define POLAR_DIFF_MAX    (255)
#define POLAR_DIFF_RANGE  (511)
#define POLAR_DIFF_OFFSET (255)

#define AOLP_MIN_S0_SUM 30
#define AOLP_MIN_DOLP   30

#define STOKES_BLUR_RADIUS 1

static uint8_t  g_aolp_lut[POLAR_DIFF_RANGE][POLAR_DIFF_RANGE];
static uint32_t g_mag_lut_q8[POLAR_DIFF_RANGE][POLAR_DIFF_RANGE];
static uint8_t  g_polar_lut_ready = 0;

static GstClockTime frame_duration = GST_SECOND / 10;

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

static void init_polar_luts_once(void) {
    if (g_polar_lut_ready) return;

    for (int s1 = POLAR_DIFF_MIN; s1 <= POLAR_DIFF_MAX; ++s1) {
        for (int s2 = POLAR_DIFF_MIN; s2 <= POLAR_DIFF_MAX; ++s2) {
            int i1 = s1 + POLAR_DIFF_OFFSET;
            int i2 = s2 + POLAR_DIFF_OFFSET;

            float mag = sqrtf((float)(s1 * s1 + s2 * s2));
            g_mag_lut_q8[i1][i2] = (uint32_t)(mag * 256.0f + 0.5f);

            if (s1 == 0 && s2 == 0) {
                g_aolp_lut[i1][i2] = 128;
            } else {
                float aolp = 0.5f * fast_atan2f((float)s2, (float)s1);
                if (aolp < 0.0f) {
                    aolp += (float)M_PI;
                }
                g_aolp_lut[i1][i2] = clamp_u8_float((aolp / (float)M_PI) * 255.0f);
            }
        }
    }
    g_polar_lut_ready = 1;
}

// ---------------------------------------------------------
// 🔥 核心新增：将 64位序列号和时间戳 编码为 NV12 图像顶部像素
// ---------------------------------------------------------
static void encode_metadata_to_pixels(uint8_t* y_plane, uint32_t width, uint64_t seq, uint64_t ts_us) {
    // 共 64 位。1位占 4个像素宽、2个像素高。
    // 第 0-255 列存 Seq，第 256-511 列存 Timestamp。
    for (int i = 0; i < 64; ++i) {
        uint8_t bit_seq = (seq & (1ULL << i)) ? 255 : 0;
        uint8_t bit_ts  = (ts_us & (1ULL << i)) ? 255 : 0;
        
        for (int j = 0; j < 4; ++j) {
            // 第 0 行 和 第 1 行 (冗余存储，抵抗 H265 压缩模糊)
            y_plane[i*4 + j] = bit_seq;                               // 行0, Seq
            y_plane[width + i*4 + j] = bit_seq;                       // 行1, Seq
            
            y_plane[256 + i*4 + j] = bit_ts;                          // 行0, Time
            y_plane[width + 256 + i*4 + j] = bit_ts;                  // 行1, Time
        }
    }
}
// ---------------------------------------------------------

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
    if (fabs(w) < 1e-12) return 0;
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

static Mat3x3d mat3_from_affine_params(const AffineMatrixParams* p) {
    Mat3x3d H = {{
        {(double)p->v[0], (double)p->v[1], (double)p->v[2]},
        {(double)p->v[3], (double)p->v[4], (double)p->v[5]},
        {0.0, 0.0, 1.0}
    }};
    return H;
}

static KnownHomographies get_known_homographies(void) {
    KnownHomographies kh;

    pthread_mutex_lock(&g_params_lock);
    kh.H_LD_to_LU = mat3_from_affine_params(&g_affine_params[0]);
    kh.H_LU_to_LU = mat3_from_affine_params(&g_affine_params[1]);
    kh.H_RD_to_LU = mat3_from_affine_params(&g_affine_params[2]);
    kh.H_RU_to_LU = mat3_from_affine_params(&g_affine_params[3]);
    pthread_mutex_unlock(&g_params_lock);

    return kh;
}

// ==================== 查表缓存结构 ====================

typedef struct {
    int32_t* src_idx;
    uint8_t* raw_sel;
    uint8_t* mask;
    uint8_t* overlap;
    uint8_t* weight_new;
} CameraWarpCache;

typedef struct {
    uint8_t* i0;
    uint8_t* i45;
    uint8_t* i90;
    uint8_t* i135;
    uint8_t* s0;
    uint8_t* aolp;
    uint8_t* dolp;
} PolarPlaneCache;

typedef struct {
    int initialized;
    int single_w;
    int single_h;
    int plane_w;
    int plane_h;
    int crop_x;
    int crop_y;

    CameraWarpCache cam[CAM_COUNT];
    PolarPlaneCache planes[CAM_COUNT];
    uint8_t* union_mask;
    Mat3x3d cached_inv_h[CAM_COUNT];

    CameraDynamicParams cached_params[CAM_COUNT];
    AffineMatrixParams cached_affine[CAM_COUNT];

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
    if (!c->src_idx || !c->raw_sel || !c->mask || !c->overlap || !c->weight_new) return 0;
    return 1;
}

static int alloc_polar_plane_cache(PolarPlaneCache* p, int plane_pixels) {
    p->i0 = (uint8_t*)malloc((size_t)plane_pixels);
    p->i45 = (uint8_t*)malloc((size_t)plane_pixels);
    p->i90 = (uint8_t*)malloc((size_t)plane_pixels);
    p->i135 = (uint8_t*)malloc((size_t)plane_pixels);
    p->s0 = (uint8_t*)malloc((size_t)plane_pixels);
    p->aolp = (uint8_t*)malloc((size_t)plane_pixels);
    p->dolp = (uint8_t*)malloc((size_t)plane_pixels);
    if (!p->i0 || !p->i45 || !p->i90 || !p->i135 || !p->s0 || !p->aolp || !p->dolp) return 0;
    return 1;
}

static void free_camera_cache(CameraWarpCache* c) {
    free(c->src_idx); free(c->raw_sel); free(c->mask); free(c->overlap); free(c->weight_new);
    c->src_idx = NULL; c->raw_sel = NULL; c->mask = NULL; c->overlap = NULL; c->weight_new = NULL;
}

static void free_polar_plane_cache(PolarPlaneCache* p) {
    free(p->i0); free(p->i45); free(p->i90); free(p->i135); free(p->s0); free(p->aolp); free(p->dolp);
    p->i0 = NULL; p->i45 = NULL; p->i90 = NULL; p->i135 = NULL; p->s0 = NULL; p->aolp = NULL; p->dolp = NULL;
}

static int alloc_runtime_cache(RuntimeStitchCache* st) {
    memset(st, 0, sizeof(*st));
    for (int i = 0; i < CAM_COUNT; ++i) {
        if (!alloc_camera_cache(&st->cam[i])) return 0;
    }
    st->union_mask = (uint8_t*)malloc(CANVAS_PIXELS);
    if (!st->union_mask) return 0;
    return 1;
}

static void free_runtime_cache(RuntimeStitchCache* st) {
    if (!st) return;
    for (int i = 0; i < CAM_COUNT; ++i) {
        free_camera_cache(&st->cam[i]);
        free_polar_plane_cache(&st->planes[i]);
    }
    free(st->union_mask);
    st->union_mask = NULL;
    st->initialized = 0;
}

static int ensure_polar_plane_caches(RuntimeStitchCache* st, int single_w, int single_h) {
    int plane_w = single_w / 2;
    int plane_h = single_h / 2;
    int plane_pixels = plane_w * plane_h;

    if (st->plane_w == plane_w && st->plane_h == plane_h &&
        st->planes[0].i0 && st->planes[1].i0 && st->planes[2].i0 && st->planes[3].i0) {
        return 1;
    }

    for (int i = 0; i < CAM_COUNT; ++i) free_polar_plane_cache(&st->planes[i]);
    for (int i = 0; i < CAM_COUNT; ++i) {
        if (!alloc_polar_plane_cache(&st->planes[i], plane_pixels)) return 0;
    }

    st->plane_w = plane_w;
    st->plane_h = plane_h;
    return 1;
}

// ==================== 偏振物理量计算 ====================

static inline void sample_blurred_stokes_3x3(uint8_t* src,
                                             int src_idx,
                                             int src_w,
                                             int src_h,
                                             int* s0_sum,
                                             int* s1,
                                             int* s2) {
    int center_y = src_idx / src_w;
    int center_x = src_idx - center_y * src_w;

    int sum_i90 = 0, sum_i45 = 0, sum_i135 = 0, sum_i0 = 0, cnt = 0;

    for (int by = -STOKES_BLUR_RADIUS; by <= STOKES_BLUR_RADIUS; ++by) {
        int sy = center_y + by * 2;
        if (sy < 0 || sy + 1 >= src_h) continue;

        for (int bx = -STOKES_BLUR_RADIUS; bx <= STOKES_BLUR_RADIUS; ++bx) {
            int sx = center_x + bx * 2;
            if (sx < 0 || sx + 1 >= src_w) continue;

            int p = sy * src_w + sx;
            sum_i90  += (int)src[p];
            sum_i45  += (int)src[p + 1];
            sum_i135 += (int)src[p + src_w];
            sum_i0   += (int)src[p + src_w + 1];
            ++cnt;
        }
    }

    if (cnt <= 0) {
        *s0_sum = 0; *s1 = 0; *s2 = 0;
        return;
    }

    int avg_i90  = (sum_i90  + cnt / 2) / cnt;
    int avg_i45  = (sum_i45  + cnt / 2) / cnt;
    int avg_i135 = (sum_i135 + cnt / 2) / cnt;
    int avg_i0   = (sum_i0   + cnt / 2) / cnt;

    *s0_sum = avg_i0 + avg_i45 + avg_i90 + avg_i135;
    *s1 = avg_i0 - avg_i90;
    *s2 = avg_i45 - avg_i135;
}

static inline uint8_t sample_polar_value(uint8_t* src,
                                         int src_idx,
                                         int src_w,
                                         int src_h,
                                         uint8_t raw_sel,
                                         uint8_t mode) {
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

    if (mode == MODE_AOLP) {
        int s0_sum = 0, s1 = 0, s2 = 0;
        sample_blurred_stokes_3x3(src, src_idx, src_w, src_h, &s0_sum, &s1, &s2);
        if (s0_sum <= AOLP_MIN_S0_SUM) return 128;

        uint32_t mag_q8 = g_mag_lut_q8[s1 + POLAR_DIFF_OFFSET][s2 + POLAR_DIFF_OFFSET];
        int dolp_val = (int)((mag_q8 * 1020u) / ((uint32_t)s0_sum * 256u));

        if (dolp_val < AOLP_MIN_DOLP) return 128;
        return g_aolp_lut[s1 + POLAR_DIFF_OFFSET][s2 + POLAR_DIFF_OFFSET];
    }

    if (mode == MODE_DOLP) {
        int s0_sum = 0, s1 = 0, s2 = 0;
        sample_blurred_stokes_3x3(src, src_idx, src_w, src_h, &s0_sum, &s1, &s2);
        if (s0_sum <= AOLP_MIN_S0_SUM) return 0;
        uint32_t mag_q8 = g_mag_lut_q8[s1 + POLAR_DIFF_OFFSET][s2 + POLAR_DIFF_OFFSET];
        int out = (int)((mag_q8 * 1020u) / ((uint32_t)s0_sum * 256u));
        if (out > 255) out = 255;
        if (out < 0) out = 0;
        return (uint8_t)out;
    }

    float S0 = 0.25f * (i0 + i45 + i90 + i135);
    return clamp_u8_float(S0);
}

static void precompute_polar_planes(uint8_t* src,
                                    int src_w,
                                    int src_h,
                                    PolarPlaneCache* planes,
                                    uint8_t mode) {
    int plane_w = src_w / 2;
    int plane_h = src_h / 2;

    #pragma omp parallel for schedule(static)
    for (int y = 0; y < plane_h; ++y) {
        int sy = y * 2;
        size_t plane_row = (size_t)y * (size_t)plane_w;
        size_t src_row_top = (size_t)sy * (size_t)src_w;
        size_t src_row_bottom = src_row_top + (size_t)src_w;

        for (int x = 0; x < plane_w; ++x) {
            int sx = x * 2;
            size_t p = plane_row + (size_t)x;
            uint8_t i90 = src[src_row_top + (size_t)sx];
            uint8_t i45 = src[src_row_top + (size_t)sx + 1];
            uint8_t i135 = src[src_row_bottom + (size_t)sx];
            uint8_t i0 = src[src_row_bottom + (size_t)sx + 1];

            if (mode == MODE_I90)  { planes->i90[p] = i90; continue; }
            if (mode == MODE_I45)  { planes->i45[p] = i45; continue; }
            if (mode == MODE_I135) { planes->i135[p] = i135; continue; }
            if (mode == MODE_I0)   { planes->i0[p] = i0; continue; }
            if (mode == MODE_S0)   { planes->s0[p] = (uint8_t)(((int)i0 + (int)i45 + (int)i90 + (int)i135 + 2) >> 2); continue; }

            int sum_i90 = 0, sum_i45 = 0, sum_i135 = 0, sum_i0 = 0, cnt = 0;

            for (int by = -STOKES_BLUR_RADIUS; by <= STOKES_BLUR_RADIUS; ++by) {
                int py = y + by;
                if (py < 0 || py >= plane_h) continue;
                size_t nbr_src_row_top = (size_t)(py * 2) * (size_t)src_w;
                size_t nbr_src_row_bottom = nbr_src_row_top + (size_t)src_w;

                for (int bx = -STOKES_BLUR_RADIUS; bx <= STOKES_BLUR_RADIUS; ++bx) {
                    int px = x + bx;
                    if (px < 0 || px >= plane_w) continue;
                    size_t qx = (size_t)(px * 2);
                    sum_i90  += (int)src[nbr_src_row_top + qx];
                    sum_i45  += (int)src[nbr_src_row_top + qx + 1];
                    sum_i135 += (int)src[nbr_src_row_bottom + qx];
                    sum_i0   += (int)src[nbr_src_row_bottom + qx + 1];
                    ++cnt;
                }
            }

            if (cnt <= 0) {
                if (mode == MODE_AOLP) planes->aolp[p] = 128;
                else planes->dolp[p] = 0;
                continue;
            }

            int avg_i90 = (sum_i90 + cnt / 2) / cnt;
            int avg_i45 = (sum_i45 + cnt / 2) / cnt;
            int avg_i135 = (sum_i135 + cnt / 2) / cnt;
            int avg_i0 = (sum_i0 + cnt / 2) / cnt;

            int s0_sum = avg_i0 + avg_i45 + avg_i90 + avg_i135;
            int s1 = avg_i0 - avg_i90;
            int s2 = avg_i45 - avg_i135;

            if (mode == MODE_AOLP) {
                if (s0_sum <= AOLP_MIN_S0_SUM) { planes->aolp[p] = 128; continue; }
                uint32_t mag_q8 = g_mag_lut_q8[s1 + POLAR_DIFF_OFFSET][s2 + POLAR_DIFF_OFFSET];
                int dolp_val = (int)((mag_q8 * 1020u) / ((uint32_t)s0_sum * 256u));
                if (dolp_val < AOLP_MIN_DOLP) { planes->aolp[p] = 128; continue; }
                planes->aolp[p] = g_aolp_lut[s1 + POLAR_DIFF_OFFSET][s2 + POLAR_DIFF_OFFSET];
                continue;
            }

            if (s0_sum <= AOLP_MIN_S0_SUM) { planes->dolp[p] = 0; continue; }

            uint32_t mag_q8 = g_mag_lut_q8[s1 + POLAR_DIFF_OFFSET][s2 + POLAR_DIFF_OFFSET];
            int out = (int)((mag_q8 * 1020u) / ((uint32_t)s0_sum * 256u));
            if (out > 255) out = 255;
            if (out < 0) out = 0;
            planes->dolp[p] = (uint8_t)out;
        }
    }
}

// ==================== 几何预计算 ====================

static int rectified_xy_to_source_block(double rx, double ry, int src_w, int src_h,
                                        float physical_theta, int* src_idx, uint8_t* raw_sel) {
    double c = cos((double)physical_theta);
    double s = sin((double)physical_theta);

    double cx = (double)src_w * 0.5;
    double cy = (double)src_h * 0.5;

    double out_w = fabs(c) * (double)src_w + fabs(s) * (double)src_h;
    double out_h = fabs(s) * (double)src_w + fabs(c) * (double)src_h;
    double dst_cx = out_w * 0.5;
    double dst_cy = out_h * 0.5;

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
    *raw_sel = 0;
    return 1;
}

static uint8_t raw_selector_for_canvas_pixel(int x, int y, float physical_theta) {
    int px = x & 1;
    int py = y & 1;

    if (physical_theta > 0.0f) {
        if (py == 0 && px == 0) return 2;
        if (py == 0 && px == 1) return 0;
        if (py == 1 && px == 0) return 3;
        return 1;
    }

    if (py == 0 && px == 0) return 1;
    if (py == 0 && px == 1) return 3;
    if (py == 1 && px == 0) return 0;
    return 2;
}

static void build_camera_warp_map(CameraWarpCache* cache, int src_w, int src_h,
                                  Mat3x3d H_cam_to_LU, float physical_theta,
                                  CameraDynamicParams* params, Mat3x3d* invH_out) {
    Mat3x3d delta = mat3_delta_from_udp(params->tx, params->ty, params->theta_rad);
    Mat3x3d H_eff = mat3_mul(delta, H_cam_to_LU);
    Mat3x3d invH;

    if (!mat3_inverse(H_eff, &invH)) {
        fprintf(stderr, "Error: homography inverse failed, camera map disabled.\n");
        memset(cache->src_idx, 0xFF, CANVAS_PIXELS * sizeof(int32_t));
        memset(cache->raw_sel, 0, CANVAS_PIXELS);
        memset(cache->mask, 0, CANVAS_PIXELS);
        if (invH_out) *invH_out = mat3_identity();
        return;
    }

    if (invH_out) *invH_out = invH;

    #pragma omp parallel for schedule(static)
    for (int y = 0; y < CANVAS_H; ++y) {
        size_t row = (size_t)y * (size_t)CANVAS_W;
        for (int x = 0; x < CANVAS_W; ++x) {
            double rx = 0.0, ry = 0.0;
            int src_idx = -1;
            uint8_t sel = 0;
            size_t p = row + (size_t)x;

            if (!apply_homography(invH, (double)x, (double)y, &rx, &ry)) {
                cache->src_idx[p] = -1; cache->raw_sel[p] = 0; cache->mask[p] = 0;
                continue;
            }

            if (!rectified_xy_to_source_block(rx, ry, src_w, src_h, physical_theta, &src_idx, &sel)) {
                cache->src_idx[p] = -1; cache->raw_sel[p] = 0; cache->mask[p] = 0;
                continue;
            }

            cache->src_idx[p] = src_idx;
            cache->raw_sel[p] = raw_selector_for_canvas_pixel(x, y, physical_theta);
            cache->mask[p] = 255;
        }
    }
}

static int params_changed_for_cache(RuntimeStitchCache* st, CameraDynamicParams* params) {
    if (!st->initialized) return 1;

    for (int i = 0; i < CAM_COUNT; ++i) {
        float dtx = fabsf(params[i].tx - st->cached_params[i].tx);
        float dty = fabsf(params[i].ty - st->cached_params[i].ty);
        float dtheta = fabsf(params[i].theta_rad - st->cached_params[i].theta_rad);
        if (dtx > PARAM_EPS_T || dty > PARAM_EPS_T || dtheta > PARAM_EPS_THETA) return 1;
    }

    pthread_mutex_lock(&g_params_lock);
    for (int i = 0; i < CAM_COUNT; ++i) {
        for (int j = 0; j < 6; ++j) {
            float da = fabsf(g_affine_params[i].v[j] - st->cached_affine[i].v[j]);
            if (da > 1e-6f) { pthread_mutex_unlock(&g_params_lock); return 1; }
        }
    }
    pthread_mutex_unlock(&g_params_lock);

    return 0;
}

// ==================== 融合权重预计算 ====================

static void compute_chamfer_distance_u16(const uint8_t* mask, uint16_t* dist) {
    #pragma omp parallel for schedule(static)
    for (size_t p = 0; p < CANVAS_PIXELS; ++p) dist[p] = mask[p] ? DIST_INF : 0;

    for (int y = 0; y < CANVAS_H; ++y) {
        for (int x = 0; x < CANVAS_W; ++x) {
            size_t p = (size_t)y * (size_t)CANVAS_W + (size_t)x;
            uint16_t d = dist[p];
            if (x > 0) { uint16_t v = (uint16_t)(dist[p - 1] + 1); if (v < d) d = v; }
            if (y > 0) { uint16_t v = (uint16_t)(dist[p - CANVAS_W] + 1); if (v < d) d = v; }
            dist[p] = d;
        }
    }

    for (int y = CANVAS_H - 1; y >= 0; --y) {
        for (int x = CANVAS_W - 1; x >= 0; --x) {
            size_t p = (size_t)y * (size_t)CANVAS_W + (size_t)x;
            uint16_t d = dist[p];
            if (x + 1 < CANVAS_W) { uint16_t v = (uint16_t)(dist[p + 1] + 1); if (v < d) d = v; }
            if (y + 1 < CANVAS_H) { uint16_t v = (uint16_t)(dist[p + CANVAS_W] + 1); if (v < d) d = v; }
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

    int order[CAM_COUNT] = {1, 0, 3, 2};

    for (int oi = 0; oi < CAM_COUNT; ++oi) {
        int ci = order[oi];
        CameraWarpCache* c = &st->cam[ci];

        if (oi == 0) {
            #pragma omp parallel for schedule(static)
            for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
                if (c->mask[p]) { c->weight_new[p] = 255; st->union_mask[p] = 255; }
            }
            continue;
        }

        #pragma omp parallel for schedule(static)
        for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
            if (!c->mask[p]) {
                c->weight_new[p] = 0; c->overlap[p] = 0;
            } else if (!st->union_mask[p]) {
                c->weight_new[p] = 255; c->overlap[p] = 0;
            } else {
                c->weight_new[p] = 128; c->overlap[p] = 255;
            }
        }

        if (ci == 0 || ci == 2) {
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
                    if (w < 0) w = 0; if (w > 255) w = 255;
                    c->weight_new[p] = (uint8_t)w;
                }
            }
            free(dist_old); free(dist_new);
        } else if (ci == 3) {
            #pragma omp parallel for schedule(static)
            for (int y = 0; y < CANVAS_H; ++y) {
                int first = -1, last = -1;
                size_t row = (size_t)y * (size_t)CANVAS_W;
                for (int x = 0; x < CANVAS_W; ++x) {
                    size_t p = row + (size_t)x;
                    if (c->overlap[p]) { if (first < 0) first = x; last = x; }
                }
                if (first < 0) continue;
                int len = last - first;
                for (int x = first; x <= last; ++x) {
                    size_t p = row + (size_t)x;
                    if (!c->overlap[p]) continue;
                    int w = (len <= 0) ? 128 : (int)((double)(x - first) * 255.0 / (double)len);
                    if (w < 0) w = 0; if (w > 255) w = 255;
                    c->weight_new[p] = (uint8_t)w;
                }
            }
        }

        #pragma omp parallel for schedule(static)
        for (size_t p = 0; p < CANVAS_PIXELS; ++p) {
            if (c->mask[p]) st->union_mask[p] = 255;
        }
    }
}

static void find_best_crop_once(RuntimeStitchCache* st) {
    int best_x = 0; int best_y = 0;
    long long best_score = -1;
    int max_x = CANVAS_W - FINAL_W;
    int max_y = CANVAS_H - FINAL_H;

    if (max_x < 0 || max_y < 0) { st->crop_x = 0; st->crop_y = 0; return; }

    for (int y0 = 0; y0 <= max_y; y0 += CROP_SEARCH_STEP) {
        for (int x0 = 0; x0 <= max_x; x0 += CROP_SEARCH_STEP) {
            long long score = 0;
            for (int yy = 0; yy < FINAL_H; yy += 16) {
                int y = y0 + yy;
                const uint8_t* row = st->union_mask + (size_t)y * (size_t)CANVAS_W + (size_t)x0;
                for (int xx = 0; xx < FINAL_W; xx += 16) {
                    int edge_bonus = (xx < 64 || yy < 64 || xx > FINAL_W - 65 || yy > FINAL_H - 65) ? 6 : 1;
                    score += row[xx] ? edge_bonus : 0;
                }
            }
            if (score > best_score) { best_score = score; best_x = x0; best_y = y0; }
        }
    }

    st->crop_x = best_x - 10;
    st->crop_y = best_y - 50;

    if (st->crop_x < 0) st->crop_x = 0;
    if (st->crop_x > CANVAS_W - FINAL_W) st->crop_x = CANVAS_W - FINAL_W;
    if (st->crop_y < 0) st->crop_y = 0;
    if (st->crop_y > CANVAS_H - FINAL_H) st->crop_y = CANVAS_H - FINAL_H;

    printf("[StitchCache] fixed crop: x=%d, y=%d, size=%dx%d\n", st->crop_x, st->crop_y, FINAL_W, FINAL_H);
}

static int init_or_rebuild_runtime_cache(RuntimeStitchCache* st, int single_w, int single_h, CameraDynamicParams* params) {
    if (!st->union_mask) {
        if (!alloc_runtime_cache(st)) return 0;
    }
    if (!ensure_polar_plane_caches(st, single_w, single_h)) return 0;

    KnownHomographies kh = get_known_homographies();
    st->single_w = single_w; st->single_h = single_h;

    build_camera_warp_map(&st->cam[1], single_w, single_h, kh.H_LU_to_LU,  (float)M_PI_2, &params[1], &st->cached_inv_h[1]);
    build_camera_warp_map(&st->cam[0], single_w, single_h, kh.H_LD_to_LU,  (float)M_PI_2, &params[0], &st->cached_inv_h[0]);
    build_camera_warp_map(&st->cam[3], single_w, single_h, kh.H_RU_to_LU, -(float)M_PI_2, &params[3], &st->cached_inv_h[3]);
    build_camera_warp_map(&st->cam[2], single_w, single_h, kh.H_RD_to_LU, -(float)M_PI_2, &params[2], &st->cached_inv_h[2]);

    build_simple_blend_weights(st);
    find_best_crop_once(st);

    memcpy(st->cached_params, params, sizeof(CameraDynamicParams) * CAM_COUNT);
    pthread_mutex_lock(&g_params_lock);
    memcpy(st->cached_affine, g_affine_params, sizeof(AffineMatrixParams) * CAM_COUNT);
    pthread_mutex_unlock(&g_params_lock);
    st->initialized = 1;
    return 1;
}

// ==================== 每帧查表渲染 ====================

static void render_camera_by_map(uint8_t* src, uint8_t* cam_y, CameraWarpCache* cache,
                                 int src_w, int src_h, uint8_t mode, int crop_x, int crop_y) {
    #pragma omp parallel for schedule(static)
    for (int y = crop_y; y < crop_y + FINAL_H; ++y) {
        size_t row = (size_t)y * (size_t)CANVAS_W;
        for (int x = crop_x; x < crop_x + FINAL_W; ++x) {
            size_t p = row + (size_t)x;
            int src_idx = cache->src_idx[p];
            if (src_idx < 0) { cam_y[p] = 0; continue; }
            cam_y[p] = sample_polar_value(src, src_idx, src_w, src_h, cache->raw_sel[p], mode);
        }
    }
}

static void render_camera_by_map_polar_plane(PolarPlaneCache* planes, uint8_t* cam_y,
                                             CameraWarpCache* cache, RuntimeStitchCache* st, uint8_t mode) {
    const uint8_t* plane = planes->i0;

    switch (mode) {
        case MODE_S0:   plane = planes->s0; break;
        case MODE_AOLP: plane = planes->aolp; break;
        case MODE_DOLP: plane = planes->dolp; break;
        case MODE_I45:  plane = planes->i45; break;
        case MODE_I90:  plane = planes->i90; break;
        case MODE_I135: plane = planes->i135; break;
        case MODE_I0:
        default:        plane = planes->i0; break;
    }

    int plane_w = st->plane_w;
    int plane_h = st->plane_h;

    #pragma omp parallel for schedule(static)
    for (int y = st->crop_y; y < st->crop_y + FINAL_H; ++y) {
        size_t row = (size_t)y * (size_t)CANVAS_W;
        for (int x = st->crop_x; x < st->crop_x + FINAL_W; ++x) {
            size_t p = row + (size_t)x;
            int src_idx = cache->src_idx[p];
            if (src_idx < 0) { cam_y[p] = 0; continue; }

            int sy = src_idx / st->single_w;
            int sx = src_idx - sy * st->single_w;
            int px = sx >> 1;
            int py = sy >> 1;

            if (px < 0 || py < 0 || px >= plane_w || py >= plane_h) {
                cam_y[p] = 0; continue;
            }

            cam_y[p] = plane[(size_t)py * (size_t)plane_w + (size_t)px];
        }
    }
}

static void blend_camera_into_canvas(uint8_t* pano_y, uint8_t* pano_mask, const uint8_t* cam_y,
                                     CameraWarpCache* cache, RuntimeStitchCache* st, int cam_idx, uint8_t mode) {
    float gain = 1.0f;
    float bias = 0.0f;

    #pragma omp parallel for schedule(static)
    for (int y = st->crop_y; y < st->crop_y + FINAL_H; ++y) {
        size_t row = (size_t)y * (size_t)CANVAS_W;
        for (int x = st->crop_x; x < st->crop_x + FINAL_W; ++x) {
            size_t p = row + (size_t)x;
            if (!cache->mask[p]) continue;

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
}

static void crop_canvas_to_nv12(uint8_t* pano_y, uint8_t* nv12, int crop_x, int crop_y) {
    for (int y = 0; y < FINAL_H; ++y) {
        const uint8_t* src_row = pano_y + (size_t)(crop_y + y) * (size_t)CANVAS_W + (size_t)crop_x;
        uint8_t* dst_row = nv12 + (size_t)y * (size_t)FINAL_W;
        memcpy(dst_row, src_row, FINAL_W);
    }
    // 初始化 UV 平面为灰色
    memset(nv12 + FINAL_PIXELS, 128, FINAL_PIXELS / 2);
}

// ==================== 主拼接推流线程 ====================

void* StitchingThreadFunc(void* arg) {
    (void)arg;

    HIMAGE local_images[CAM_COUNT] = {NULL};
    GstElement* pipeline = NULL;
    GstElement* appsrc = NULL;
    GstElement* encoder = NULL;
    bool is_initialized = false;
    int single_w = 0, single_h = 0;

    RuntimeStitchCache stitch_cache;
    memset(&stitch_cache, 0, sizeof(stitch_cache));

    uint8_t* pano_y = NULL; uint8_t* pano_mask = NULL;
    uint8_t* cam_y[CAM_COUNT] = {NULL};

    struct timespec fps_start_time, current_time, process_start, process_end;
    int frame_count = 0;
    double total_process_ms = 0.0;
    uint16_t last_bitrate_mbps = 0;

    while (g_running) {
        pthread_mutex_lock(&g_sync_box.lock);
        while (g_sync_box.ready_mask != 15 && g_running) {
            pthread_cond_wait(&g_sync_box.cond, &g_sync_box.lock);
        }
        if (!g_running) { pthread_mutex_unlock(&g_sync_box.lock); break; }

        for (int i = 0; i < CAM_COUNT; ++i) {
            local_images[i] = g_sync_box.images[i];
            g_sync_box.images[i] = NULL;
        }
        g_sync_box.ready_mask = 0;
        
        // 捕获当前的全局序列号和微秒时间戳
        uint64_t current_frame_seq = g_sync_box.target_seq;
        uint64_t current_timestamp_us = g_sync_box.capture_timestamp_us;
        pthread_mutex_unlock(&g_sync_box.lock);

        CameraDynamicParams local_params[CAM_COUNT];
        uint16_t target_bitrate_mbps = 0;

        pthread_mutex_lock(&g_params_lock);
        memcpy(local_params, g_cam_params, sizeof(CameraDynamicParams) * CAM_COUNT);
        target_bitrate_mbps = g_current_bitrate_mbps;
        pthread_mutex_unlock(&g_params_lock);

        if (target_bitrate_mbps == 0) target_bitrate_mbps = 4;

        if (!is_initialized) {
            ImageInfo info = g_Image_GetImageInfo(local_images[0]);
            single_w = info.width; single_h = info.height;

            pano_y = (uint8_t*)malloc(CANVAS_PIXELS);
            pano_mask = (uint8_t*)malloc(CANVAS_PIXELS);
            for (int i = 0; i < CAM_COUNT; ++i) cam_y[i] = (uint8_t*)malloc(CANVAS_PIXELS);

            init_polar_luts_once();
            if (!init_or_rebuild_runtime_cache(&stitch_cache, single_w, single_h, local_params)) goto cleanup_frame;

            uint32_t init_bitrate_bps = (uint32_t)target_bitrate_mbps * 1000u * 1000u;

            // do-timestamp=true 交由 GStreamer 内部保持时钟平稳
            char gst_pipeline_str[1024];
            snprintf(gst_pipeline_str, sizeof(gst_pipeline_str),
                "appsrc name=mysrc is-live=true format=TIME do-timestamp=true block=false "
                "caps=video/x-raw,format=NV12,width=%d,height=%d,framerate=10/1 ! "
                "queue name=input_queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
                "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
                "nvv4l2h265enc name=enc bitrate=%u control-rate=1 maxperf-enable=1 insert-sps-pps=true idrinterval=10 ! "
                "h265parse config-interval=1 ! "
                "rtph265pay pt=96 mtu=1400 ! "
                "udpsink host=192.168.1.23 port=2000 sync=false async=true max-bitrate=0 qos-dscp=0",
                FINAL_W, FINAL_H, init_bitrate_bps);

            GError* error = NULL;
            pipeline = gst_parse_launch(gst_pipeline_str, &error);
            if (error || !pipeline) goto cleanup_frame;

            appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "mysrc");
            encoder = gst_bin_get_by_name(GST_BIN(pipeline), "enc");

            last_bitrate_mbps = target_bitrate_mbps;
            g_object_set(appsrc, "block", FALSE, NULL);
            gst_element_set_state(pipeline, GST_STATE_PLAYING);
            clock_gettime(CLOCK_MONOTONIC, &fps_start_time);
            is_initialized = true;
        }

        if (encoder && target_bitrate_mbps != last_bitrate_mbps) {
            uint32_t target_bitrate_bps = (uint32_t)target_bitrate_mbps * 1000u * 1000u;
            g_object_set(G_OBJECT(encoder), "bitrate", target_bitrate_bps, NULL);
            last_bitrate_mbps = target_bitrate_mbps;
        }

        clock_gettime(CLOCK_MONOTONIC, &process_start);
        if (params_changed_for_cache(&stitch_cache, local_params)) {
            init_or_rebuild_runtime_cache(&stitch_cache, single_w, single_h, local_params);
        }

        GstBuffer* buffer = gst_buffer_new_allocate(NULL, FINAL_W * FINAL_H * 3 / 2, NULL);
        GstMapInfo map;
        if (!buffer || !gst_buffer_map(buffer, &map, GST_MAP_WRITE)) goto cleanup_frame;

        uint8_t* dst_nv12 = map.data;

        for (int y = stitch_cache.crop_y; y < stitch_cache.crop_y + FINAL_H; ++y) {
            size_t p = (size_t)y * (size_t)CANVAS_W + (size_t)stitch_cache.crop_x;
            memset(pano_y + p, 0, FINAL_W);
            memset(pano_mask + p, 0, FINAL_W);
        }

        uint8_t* src_BL = (uint8_t*)g_Image_GetImageBuff(local_images[0]);
        uint8_t* src_TL = (uint8_t*)g_Image_GetImageBuff(local_images[1]);
        uint8_t* src_BR = (uint8_t*)g_Image_GetImageBuff(local_images[2]);
        uint8_t* src_TR = (uint8_t*)g_Image_GetImageBuff(local_images[3]);

        // 渲染 TL
        if (local_params[1].mode == MODE_I0 || local_params[1].mode == MODE_I45 ||
            local_params[1].mode == MODE_I90 || local_params[1].mode == MODE_I135 ||
            local_params[1].mode == MODE_S0 || local_params[1].mode == MODE_AOLP ||
            local_params[1].mode == MODE_DOLP) {
            precompute_polar_planes(src_TL, single_w, single_h, &stitch_cache.planes[1], local_params[1].mode);
            render_camera_by_map_polar_plane(&stitch_cache.planes[1], cam_y[1], &stitch_cache.cam[1], &stitch_cache, local_params[1].mode);
        } else {
            render_camera_by_map(src_TL, cam_y[1], &stitch_cache.cam[1], single_w, single_h, local_params[1].mode, stitch_cache.crop_x, stitch_cache.crop_y);
        }

        // 渲染 BL
        if (local_params[0].mode == MODE_I0 || local_params[0].mode == MODE_I45 ||
            local_params[0].mode == MODE_I90 || local_params[0].mode == MODE_I135 ||
            local_params[0].mode == MODE_S0 || local_params[0].mode == MODE_AOLP ||
            local_params[0].mode == MODE_DOLP) {
            precompute_polar_planes(src_BL, single_w, single_h, &stitch_cache.planes[0], local_params[0].mode);
            render_camera_by_map_polar_plane(&stitch_cache.planes[0], cam_y[0], &stitch_cache.cam[0], &stitch_cache, local_params[0].mode);
        } else {
            render_camera_by_map(src_BL, cam_y[0], &stitch_cache.cam[0], single_w, single_h, local_params[0].mode, stitch_cache.crop_x, stitch_cache.crop_y);
        }

        // 渲染 TR
        if (local_params[3].mode == MODE_I0 || local_params[3].mode == MODE_I45 ||
            local_params[3].mode == MODE_I90 || local_params[3].mode == MODE_I135 ||
            local_params[3].mode == MODE_S0 || local_params[3].mode == MODE_AOLP ||
            local_params[3].mode == MODE_DOLP) {
            precompute_polar_planes(src_TR, single_w, single_h, &stitch_cache.planes[3], local_params[3].mode);
            render_camera_by_map_polar_plane(&stitch_cache.planes[3], cam_y[3], &stitch_cache.cam[3], &stitch_cache, local_params[3].mode);
        } else {
            render_camera_by_map(src_TR, cam_y[3], &stitch_cache.cam[3], single_w, single_h, local_params[3].mode, stitch_cache.crop_x, stitch_cache.crop_y);
        }

        // 渲染 BR
        if (local_params[2].mode == MODE_I0 || local_params[2].mode == MODE_I45 ||
            local_params[2].mode == MODE_I90 || local_params[2].mode == MODE_I135 ||
            local_params[2].mode == MODE_S0 || local_params[2].mode == MODE_AOLP ||
            local_params[2].mode == MODE_DOLP) {
            precompute_polar_planes(src_BR, single_w, single_h, &stitch_cache.planes[2], local_params[2].mode);
            render_camera_by_map_polar_plane(&stitch_cache.planes[2], cam_y[2], &stitch_cache.cam[2], &stitch_cache, local_params[2].mode);
        } else {
            render_camera_by_map(src_BR, cam_y[2], &stitch_cache.cam[2], single_w, single_h, local_params[2].mode, stitch_cache.crop_x, stitch_cache.crop_y);
        }

        for (int y = stitch_cache.crop_y; y < stitch_cache.crop_y + FINAL_H; ++y) {
            size_t p = (size_t)y * (size_t)CANVAS_W + (size_t)stitch_cache.crop_x;
            memset(pano_mask + p, 0, FINAL_W);
        }

        blend_camera_into_canvas(pano_y, pano_mask, cam_y[1], &stitch_cache.cam[1], &stitch_cache, 1, local_params[1].mode);
        blend_camera_into_canvas(pano_y, pano_mask, cam_y[0], &stitch_cache.cam[0], &stitch_cache, 0, local_params[0].mode);
        blend_camera_into_canvas(pano_y, pano_mask, cam_y[3], &stitch_cache.cam[3], &stitch_cache, 3, local_params[3].mode);
        blend_camera_into_canvas(pano_y, pano_mask, cam_y[2], &stitch_cache.cam[2], &stitch_cache, 2, local_params[2].mode);

        // 裁剪并准备 NV12 数据
        crop_canvas_to_nv12(pano_y, dst_nv12, stitch_cache.crop_x, stitch_cache.crop_y);

        // 🔥 将物理绝对元数据打入像素的最顶端边缘！
        encode_metadata_to_pixels(dst_nv12, FINAL_W, current_frame_seq, current_timestamp_us);

        gst_buffer_unmap(buffer, &map);

        // Push 进流
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        if (ret != GST_FLOW_OK) gst_buffer_unref(buffer);
        
        clock_gettime(CLOCK_MONOTONIC, &process_end);
        total_process_ms += get_time_diff_ms(process_start, process_end);

        if (++frame_count == 10) {
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double elapsed = get_time_diff_ms(fps_start_time, current_time);
            printf("[PolarStream] Output:%dx%d | FPS:%.2f | Latency:%.2fms | Seq:%" PRIu64 " | Timestamp:%" PRIu64 " us\n",
                   FINAL_W, FINAL_H,
                   (10.0 / elapsed) * 1000.0,
                   total_process_ms / 10.0,
                   current_frame_seq,
                   current_timestamp_us);
            total_process_ms = 0.0; frame_count = 0; fps_start_time = current_time;
        }

cleanup_frame:
        for (int i = 0; i < CAM_COUNT; ++i) if (local_images[i]) g_Image_Recycle(local_images[i]);
    }

    if (encoder) gst_object_unref(encoder);
    if (appsrc) gst_object_unref(appsrc);
    if (pipeline) { gst_element_set_state(pipeline, GST_STATE_NULL); gst_object_unref(pipeline); }
    free_runtime_cache(&stitch_cache); free(pano_y); free(pano_mask);
    for (int i = 0; i < CAM_COUNT; ++i) free(cam_y[i]);
    return NULL;
}
