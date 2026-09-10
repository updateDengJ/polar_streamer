// ==========================================
// 文件名: config.h
// 描  述: 全局配置、宏定义与跨模块共享变量声明
// ==========================================
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include "EzSDK_C.h"

#define CAM_COUNT 4
#define BUFFER_COUNT 3
#define MAX_LINE_LENGTH 20480

extern const int TARGET_PORT;
extern const char* TARGET_SNS[CAM_COUNT];

extern volatile sig_atomic_t g_running;

extern HDEVICE g_h_devs[CAM_COUNT];
extern HSUBDEVICE g_h_subdevs[CAM_COUNT];
extern int g_cam_indices[CAM_COUNT];

typedef struct {
    HIMAGE images[CAM_COUNT];
    uint64_t block_ids[CAM_COUNT];
    int ready_mask;
    uint64_t target_seq;           // 当前正在同步收集的全局目标序号
    uint64_t capture_timestamp_us; // 当前帧组合的微秒级系统时间戳
    pthread_mutex_t lock;
    pthread_cond_t cond;
} FrameSyncBox;

extern FrameSyncBox g_sync_box;

typedef enum {
    MODE_RAW  = 0x01,
    MODE_AOLP = 0x02,
    MODE_DOLP = 0x03,
    MODE_S0   = 0x10,
    MODE_I0   = 0x11,
    MODE_I45  = 0x12,
    MODE_I90  = 0x13,
    MODE_I135 = 0x14
} ProcessingMode;

#define CMD_TRANSFORM_ONLY 0x00
#define CMD_AUTO_EXP       0x04
#define CMD_MANUAL_EXP     0x05
#define CMD_SET_AFFINE     0x30

typedef enum {
    TRIGGER_MODE_SOFT_CONTINUOUS = 0,
    TRIGGER_MODE_HARD_LINE       = 1,
    TRIGGER_MODE_NO_CHANGE       = 255
} TriggerMode;

#pragma pack(push, 1)

typedef struct {
    uint16_t header;
    uint16_t length;
    uint8_t  cam_id;
    uint8_t  command;
    uint8_t  gain;
    uint32_t exposure_us;
    int16_t  trans_x;
    int16_t  trans_y;
    uint16_t rotation;
    uint8_t  trigger_mode;
    uint16_t bitrate_mbps;
    uint8_t  expected_gray_value;
    float    affine_params[6];
    uint16_t crc16;
} HostCmdPacket;

typedef struct {
    uint16_t header;
    uint16_t length;
    uint64_t timestamp_ms;
    uint8_t  cam_id;
    uint8_t  image_type;
    uint8_t  exp_type;
    uint8_t  gain;
    uint32_t exposure_us;
    int16_t  trans_x;
    int16_t  trans_y;
    uint16_t rotation;
    uint8_t  trigger_mode;
    uint16_t bitrate_mbps;
    uint8_t  expected_gray_value;
    float    affine_params[6];
    uint16_t crc16;
} CamInfoPacket;

#pragma pack(pop)

typedef struct {
    float tx;
    float ty;
    float theta_rad;
    uint8_t mode;
} CameraDynamicParams;

typedef struct {
    float v[6];
} AffineMatrixParams;

extern CameraDynamicParams g_cam_params[CAM_COUNT];
extern AffineMatrixParams g_affine_params[CAM_COUNT];
extern pthread_mutex_t g_params_lock;

extern uint8_t  g_current_trigger_mode;
extern uint16_t g_current_bitrate_mbps;
extern uint8_t  g_current_expected_gray[CAM_COUNT];

static inline void sleep_ms(unsigned int ms) {
    usleep(ms * 1000);
}

#endif // CONFIG_H
