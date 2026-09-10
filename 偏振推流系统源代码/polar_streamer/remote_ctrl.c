// ==========================================
// 文件名: remote_ctrl.c
// 说明:
//   1. 通过 UDP 接收上位机控制命令
//   2. 普通命令按单相机处理: 图像模式 / 曝光 / 增益 / 平移 / 旋转
//   3. 触发模式只由 HostCmdPacket.trigger_mode 控制
//   4. 支持动态设置编码码率 bitrate_mbps
//   5. 支持按相机下发六参数仿射矩阵，并写入配置文件持久化
//   6. 支持将 tx/ty/rotation 微调量一并写入配置文件，重启后恢复
// ==========================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/select.h>
#include <math.h>
#include <sys/time.h>

#include "config.h"
#include "remote_ctrl.h"
#include "camera.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define AFFINE_CONFIG_FILE "affine_params.cfg"
#define AFFINE_EPS 1e-6f

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

// ==========================================
// 1. 实例化全局共享变量
// ==========================================
CameraDynamicParams g_cam_params[CAM_COUNT] = {
    {0.0f, 0.0f, 0.0f, MODE_S0},   // 改为 MODE_S0
    {0.0f, 0.0f, 0.0f, MODE_S0},   // 改为 MODE_S0
    {0.0f, 0.0f, 0.0f, MODE_S0},   // 改为 MODE_S0
    {0.0f, 0.0f, 0.0f, MODE_S0}    // 改为 MODE_S0
};

// 定义全局数组 - 与 Python 脚本完全一致
AffineMatrixParams g_affine_params[CAM_COUNT] = {
    // cam1 左下 -> LB
    {{ 0.9999926579f,  -0.0038319781f,  -25.6904124439f,
      0.0038319781f,   0.9999926579f,   2400.0057161729f }},
    
    // cam2 左上 -> LT (单位阵)
    {{ 1.0f,            0.0f,            0.0f,
       0.0f,            1.0f,            0.0f }},
    
    // cam3 右下 -> RB
    {{ 0.9997886759f,   0.0205573222f,   1676.2736934993f,
      -0.0205573222f,   0.9997886759f,   2440.5989928649f }},
    
    // cam4 右上 -> RT
    {{ 0.9999446622f,   0.0105201041f,   1679.2916020805f,
      -0.0105201041f,   0.9999446622f,   4.7720838628f }}
};

pthread_mutex_t g_params_lock = PTHREAD_MUTEX_INITIALIZER;

uint8_t  g_current_trigger_mode = TRIGGER_MODE_SOFT_CONTINUOUS;
uint16_t g_current_bitrate_mbps = 20;
uint8_t  g_current_expected_gray[CAM_COUNT] = {120, 120, 120, 120};

static uint8_t  g_current_gain[CAM_COUNT] = {0};
static uint32_t g_current_exposure[CAM_COUNT] = {0};
static uint8_t  g_current_exp_type[CAM_COUNT] = {0};  // 0=手动, 1=自动

static void init_default_affine_params(AffineMatrixParams mats[CAM_COUNT]) {
    // 直接使用全局变量的值作为默认值（与 Python 脚本保持一致）
    for (int i = 0; i < CAM_COUNT; ++i) {
        mats[i].v[0] = g_affine_params[i].v[0];
        mats[i].v[1] = g_affine_params[i].v[1];
        mats[i].v[2] = g_affine_params[i].v[2];
        mats[i].v[3] = g_affine_params[i].v[3];
        mats[i].v[4] = g_affine_params[i].v[4];
        mats[i].v[5] = g_affine_params[i].v[5];
    }
}

static void save_geometry_config_to_file_locked(void) {
    FILE* fp = fopen(AFFINE_CONFIG_FILE, "w");
    if (!fp) {
        fprintf(stderr, "[Remote] failed to open %s for writing.\n", AFFINE_CONFIG_FILE);
        return;
    }

    fprintf(fp, "# cam_id a b c d e f tx ty rotation_deg\n");
    for (int i = 0; i < CAM_COUNT; ++i) {
        float rotation_deg = g_cam_params[i].theta_rad * 180.0f / (float)M_PI;
        fprintf(fp, "%d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",
                i + 1,
                g_affine_params[i].v[0], g_affine_params[i].v[1], g_affine_params[i].v[2],
                g_affine_params[i].v[3], g_affine_params[i].v[4], g_affine_params[i].v[5],
                g_cam_params[i].tx, g_cam_params[i].ty, rotation_deg);
    }

    fclose(fp);
}

static void load_geometry_config_from_file_locked(void) {
    init_default_affine_params(g_affine_params);
    for (int i = 0; i < CAM_COUNT; ++i) {
        g_cam_params[i].tx = 0.0f;
        g_cam_params[i].ty = 0.0f;
        g_cam_params[i].theta_rad = 0.0f;
    }

    FILE* fp = fopen(AFFINE_CONFIG_FILE, "r");
    if (!fp) {
        save_geometry_config_to_file_locked();
        return;
    }

    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }

        int cam_id = 0;
        float a, b, c, d, e, f;
        float tx = 0.0f, ty = 0.0f, rotation_deg = 0.0f;
        int parsed = sscanf(line, "%d %f %f %f %f %f %f %f %f %f",
                            &cam_id, &a, &b, &c, &d, &e, &f, &tx, &ty, &rotation_deg);
        if (parsed == 7 || parsed == 10) {
            if (cam_id >= 1 && cam_id <= CAM_COUNT) {
                int idx = cam_id - 1;
                g_affine_params[idx].v[0] = a;
                g_affine_params[idx].v[1] = b;
                g_affine_params[idx].v[2] = c;
                g_affine_params[idx].v[3] = d;
                g_affine_params[idx].v[4] = e;
                g_affine_params[idx].v[5] = f;
                if (parsed == 10) {
                    g_cam_params[idx].tx = tx;
                    g_cam_params[idx].ty = ty;
                    g_cam_params[idx].theta_rad = rotation_deg * (float)M_PI / 180.0f;
                }
            }
        }
    }

    fclose(fp);
}

// ==========================================
// 2. ACK 填充
// ==========================================
static void fill_ack_packet(CamInfoPacket* ack, uint8_t cam_id) {
    int cam_idx = (int)cam_id - 1;
    if (ack == NULL || cam_idx < 0 || cam_idx >= CAM_COUNT) {
        return;
    }

    memset(ack, 0, sizeof(CamInfoPacket));
    ack->header = 0xAA55;
    ack->length = (uint16_t)(sizeof(CamInfoPacket) - 4);
    ack->cam_id = cam_id;

    pthread_mutex_lock(&g_params_lock);
    ack->image_type = g_cam_params[cam_idx].mode;
    ack->exp_type = g_current_exp_type[cam_idx];
    ack->gain = g_current_gain[cam_idx];
    ack->exposure_us = g_current_exposure[cam_idx];
    ack->trans_x = (int16_t)g_cam_params[cam_idx].tx;
    ack->trans_y = (int16_t)g_cam_params[cam_idx].ty;
    ack->rotation = (uint16_t)(g_cam_params[cam_idx].theta_rad * 180.0f / M_PI * 10.0f);
    ack->trigger_mode = g_current_trigger_mode;
    ack->bitrate_mbps = g_current_bitrate_mbps;
    ack->expected_gray_value = g_current_expected_gray[cam_idx];
    memcpy(ack->affine_params, g_affine_params[cam_idx].v, sizeof(ack->affine_params));
    pthread_mutex_unlock(&g_params_lock);

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    ack->timestamp_ms = (uint64_t)tv_now.tv_sec * 1000ULL + (uint64_t)(tv_now.tv_usec / 1000);
    ack->crc16 = 0;
}

// ==========================================
// 3. 处理全局触发模式切换（仅看 trigger_mode 字段）
// ==========================================
static int handle_trigger_mode(uint8_t trigger_mode_field) {
    CameraTriggerMode target_mode;

    if (trigger_mode_field == TRIGGER_MODE_SOFT_CONTINUOUS) {
        target_mode = TRIGGER_SOFT_CONTINUOUS;
    } else if (trigger_mode_field == TRIGGER_MODE_HARD_LINE) {
        target_mode = TRIGGER_HARD_LINE;
    } else {
        return 0;
    }

    pthread_mutex_lock(&g_params_lock);
    uint8_t current_mode = g_current_trigger_mode;
    pthread_mutex_unlock(&g_params_lock);

    if (current_mode == trigger_mode_field) {
        return 0;
    }

    printf("[Remote] Trigger request from field: %u\n", trigger_mode_field);

    int switch_ret = SwitchAllCameraTriggerMode(target_mode);
    if (switch_ret == 0) {
        pthread_mutex_lock(&g_params_lock);
        g_current_trigger_mode = trigger_mode_field;
        pthread_mutex_unlock(&g_params_lock);
    }

    printf("[Remote] Trigger switch result: mode=%s ret=%d\n",
           target_mode == TRIGGER_HARD_LINE ? "HARD" : "SOFT/FREE-RUN",
           switch_ret);

    return switch_ret;
}

// ==========================================
// 4. UDP 监听与控制线程
// ==========================================
void* CommandThreadFunc(void* arg) {
    int port = *((int*)arg);
    free(arg);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("[Remote] socket create failed");
        return NULL;
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)port);

    if (bind(sockfd, (const struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[Remote] bind failed");
        close(sockfd);
        return NULL;
    }

    printf("[Remote] command server started on port %d\n", port);
    printf("   - gain/exposure: absolute set\n");
    printf("   - tx/ty/rotation: incremental accumulate\n");
    printf("   - trigger_mode: 0=free-run, 1=hard trigger, 255=no change\n");
    printf("   - bitrate_mbps: >0 means update bitrate\n");
    printf("   - affine_params[6]: used when command=0x30, and persisted to %s\n", AFFINE_CONFIG_FILE);
    printf("   - tx/ty/rotation: also persisted to %s\n", AFFINE_CONFIG_FILE);

    char buf[1024];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    struct timeval tv;

    while (g_running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);

        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select(sockfd + 1, &readfds, NULL, NULL, &tv);
        if (!(ret > 0 && FD_ISSET(sockfd, &readfds))) {
            continue;
        }

        int n = recvfrom(sockfd, buf, sizeof(buf), 0,
                         (struct sockaddr*)&client_addr, &client_len);
        if (n <= 0) {
            continue;
        }

        if (n != (int)sizeof(HostCmdPacket)) {
            printf("[Remote] invalid packet length: %d (expected %zu)\n", n, sizeof(HostCmdPacket));
            continue;
        }

        HostCmdPacket* cmd = (HostCmdPacket*)buf;
        int cam_idx = (int)cmd->cam_id - 1;

        if (cam_idx < 0 || cam_idx >= CAM_COUNT) {
            printf("[Remote] invalid cam_id: %u\n", cmd->cam_id);
            continue;
        }

        int trigger_ret = handle_trigger_mode(cmd->trigger_mode);

        pthread_mutex_lock(&g_params_lock);

        if (cmd->bitrate_mbps > 0 && cmd->bitrate_mbps != g_current_bitrate_mbps) {
            printf("[Remote] Bitrate change: %u -> %u Mbps\n",
                   g_current_bitrate_mbps, cmd->bitrate_mbps);
            g_current_bitrate_mbps = cmd->bitrate_mbps;
        }

        if (cmd->command == CMD_SET_AFFINE) {
            memcpy(g_affine_params[cam_idx].v, cmd->affine_params, sizeof(g_affine_params[cam_idx].v));
            save_geometry_config_to_file_locked();

            printf("[Remote] cam %d affine updated:\n", cam_idx + 1);
            printf("   [%.9g %.9g %.9g]\n", g_affine_params[cam_idx].v[0], g_affine_params[cam_idx].v[1], g_affine_params[cam_idx].v[2]);
            printf("   [%.9g %.9g %.9g]\n", g_affine_params[cam_idx].v[3], g_affine_params[cam_idx].v[4], g_affine_params[cam_idx].v[5]);
        }

        float old_tx = g_cam_params[cam_idx].tx;
        float old_ty = g_cam_params[cam_idx].ty;
        float old_rot_deg = g_cam_params[cam_idx].theta_rad * 180.0f / M_PI;

        g_cam_params[cam_idx].tx += (float)cmd->trans_x;
        g_cam_params[cam_idx].ty += (float)cmd->trans_y;

        float delta_theta_deg = cmd->rotation / 10.0f;
        float delta_theta_rad = delta_theta_deg * (float)(M_PI / 180.0f);
        g_cam_params[cam_idx].theta_rad += delta_theta_rad;
        g_cam_params[cam_idx].theta_rad =
            fmodf(g_cam_params[cam_idx].theta_rad, 2.0f * (float)M_PI);
        if (g_cam_params[cam_idx].theta_rad < 0.0f) {
            g_cam_params[cam_idx].theta_rad += 2.0f * (float)M_PI;
        }

        if (cmd->trans_x != 0 || cmd->trans_y != 0 || cmd->rotation != 0) {
            save_geometry_config_to_file_locked();
        }

        if (cmd->command == CMD_AUTO_EXP) {
            g_current_exp_type[cam_idx] = 1;
            g_current_gain[cam_idx] = cmd->gain;
            g_current_exposure[cam_idx] = cmd->exposure_us;
            for (int i = 0; i < CAM_COUNT; ++i) {
                g_current_expected_gray[i] = cmd->expected_gray_value;
            }
        } else if (cmd->command == CMD_MANUAL_EXP) {
            for (int i = 0; i < CAM_COUNT; ++i) {
                g_current_exp_type[i] = 0;
                g_current_gain[i] = cmd->gain;
                g_current_exposure[i] = cmd->exposure_us;
            }
        }

        if (cmd->command == MODE_RAW  ||
            cmd->command == MODE_AOLP ||
            cmd->command == MODE_DOLP ||
            cmd->command == MODE_S0   ||
            cmd->command == MODE_I0   ||
            cmd->command == MODE_I45  ||
            cmd->command == MODE_I90  ||
            cmd->command == MODE_I135) {
            for (int i = 0; i < CAM_COUNT; ++i) {
                g_cam_params[i].mode = cmd->command;
            }
        }

        float new_rot_deg = g_cam_params[cam_idx].theta_rad * 180.0f / M_PI;
        printf("[Remote] cam %d update:\n", cam_idx + 1);
        printf("   translate: (%.1f, %.1f) + (%.1f, %.1f) = (%.1f, %.1f)\n",
               old_tx, old_ty, (float)cmd->trans_x, (float)cmd->trans_y,
               g_cam_params[cam_idx].tx, g_cam_params[cam_idx].ty);
        printf("   rotate: %.1f + %.1f = %.1f deg\n",
               old_rot_deg, delta_theta_deg, new_rot_deg);

        pthread_mutex_unlock(&g_params_lock);

        if (cmd->command == CMD_AUTO_EXP) {
            UpdateCameraHardwareParams(cam_idx,
                                       cmd->command,
                                       cmd->exposure_us,
                                       cmd->gain,
                                       g_current_expected_gray[cam_idx]);
        } else if (cmd->command == CMD_MANUAL_EXP) {
            for (int i = 0; i < CAM_COUNT; ++i) {
                UpdateCameraHardwareParams(i,
                                           cmd->command,
                                           cmd->exposure_us,
                                           cmd->gain,
                                           g_current_expected_gray[i]);
            }
        }

        CamInfoPacket ack;
        fill_ack_packet(&ack, cmd->cam_id);

        sendto(sockfd, &ack, sizeof(ack), 0,
               (struct sockaddr*)&client_addr, client_len);

        printf("[Remote] ACK sent: cam=%u mode=%u exp=%u gain=%u trig=%u bitrate=%u gray=%u affine=[%.6g %.6g %.6g %.6g %.6g %.6g]\n",
               ack.cam_id, ack.image_type, ack.exposure_us, ack.gain,
               ack.trigger_mode, ack.bitrate_mbps, ack.expected_gray_value,
               ack.affine_params[0], ack.affine_params[1], ack.affine_params[2],
               ack.affine_params[3], ack.affine_params[4], ack.affine_params[5]);

        (void)trigger_ret;
    }

    close(sockfd);
    return NULL;
}

// ==========================================
// 5. 启动函数
// ==========================================
int start_remote_server(int port) {
    pthread_t tid;
    int* p_port = (int*)malloc(sizeof(int));
    if (p_port == NULL) {
        fprintf(stderr, "[Remote] failed to allocate port arg.\n");
        return -1;
    }

    pthread_mutex_lock(&g_params_lock);
    load_geometry_config_from_file_locked();
    pthread_mutex_unlock(&g_params_lock);

    *p_port = port;

    if (pthread_create(&tid, NULL, CommandThreadFunc, p_port) != 0) {
        fprintf(stderr, "[Remote] failed to create command thread.\n");
        free(p_port);
        return -1;
    }

    pthread_detach(tid);
    return 0;
}
