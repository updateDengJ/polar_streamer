// ==========================================
// 文件名: camera.c
// 描  述: 相机生命周期管理、码流回调与硬件实时调参
// ==========================================
#include "camera.h"
#include "config.h"
#include "ezsdk_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>

static pthread_mutex_t g_trigger_switch_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_auto_exposure_sync_lock = PTHREAD_MUTEX_INITIALIZER;

static CameraTriggerMode s_camera_trigger_mode = TRIGGER_SOFT_CONTINUOUS;

// ==================== [核心修改区] ====================
// 10FPS 周期为 100ms。设定 40ms 为同一帧的时间宽容度
#define SYNC_TOLERANCE_US 40000 

// 全局软件主序号生成器 (代替了原有的独立计数器)
static uint64_t s_global_software_seq = 0;
// ======================================================

static volatile int s_master_auto_sync_enabled = 0;
static int s_master_auto_cam_idx = 0;  // 左上 TL 作为自动曝光主相机
static uint8_t s_auto_fixed_gain_raw = 0;
static uint8_t s_auto_expected_gray = 120;
static double s_last_synced_exposure_us = -1.0;
static const double s_exposure_sync_threshold_us = 100.0;

void MyStreamCallBack(HIMAGE h_img, HSUBDEVICE h_dev, unsigned int streamIndex, unsigned int event, void* context);

static void ClearFrameSyncBox(void) {
    pthread_mutex_lock(&g_sync_box.lock);
    for (int i = 0; i < CAM_COUNT; ++i) {
        if (g_sync_box.images[i] != NULL) {
            g_Image_Recycle(g_sync_box.images[i]);
            g_sync_box.images[i] = NULL;
        }
        g_sync_box.block_ids[i] = 0;
    }
    g_sync_box.ready_mask = 0;
    g_sync_box.target_seq = 0;
    g_sync_box.capture_timestamp_us = 0;
    s_global_software_seq = 0; // 重置全局序号
    pthread_mutex_unlock(&g_sync_box.lock);
}

static int SetSoftContinuousMode(HSUBDEVICE h_sub, int cam_idx) {
    int r1 = g_SetNodeValue_Int(h_sub, "AcquisitionMode", 2);
    int r2 = g_SetNodeValue_Int(h_sub, "TriggerSelector", 1);
    int r3 = g_SetNodeValue_Int(h_sub, "TriggerMode", 0);
    int r4 = g_SetNodeValue_Int(h_sub, "TriggerSource", 0);
    int r5 = g_SetNodeValue_Int(h_sub, "TriggerActivation", 1);
    int r6 = g_SetNodeValue_Int(h_sub, "AcquisitionFrameRateMode", 1);
    int r7 = g_SetNodeValue_Double(h_sub, "AcquisitionFrameRate", 10.0);
    if (r1 || r2 || r3 || r4 || r5 || r6 || r7) return -1;
    return 0;
}

static int SetHardTriggerMode(HSUBDEVICE h_sub, int cam_idx) {
    int r1 = g_SetNodeValue_Int(h_sub, "AcquisitionMode", 2);
    int r2 = g_SetNodeValue_Int(h_sub, "TriggerSelector", 1);
    int r3 = g_SetNodeValue_Int(h_sub, "TriggerMode", 1);
    int r4 = g_SetNodeValue_Int(h_sub, "TriggerSource", 1);
    int r5 = g_SetNodeValue_Int(h_sub, "TriggerActivation", 1);
    int r6 = g_SetNodeValue_Int(h_sub, "LineMode", 0);
    int r7 = g_SetNodeValue_Int(h_sub, "AcquisitionFrameRateMode", 0);
    if (r1 || r2 || r3 || r4 || r5 || r6|| r7) return -1;
    return 0;
}

int SwitchAllCameraTriggerMode(CameraTriggerMode mode) {
    int ret = 0;
    if (mode != TRIGGER_SOFT_CONTINUOUS && mode != TRIGGER_HARD_LINE) return -1;

    pthread_mutex_lock(&g_trigger_switch_lock);
    if (mode == s_camera_trigger_mode) {
        pthread_mutex_unlock(&g_trigger_switch_lock);
        return 0;
    }

    for (int i = 0; i < CAM_COUNT; ++i) {
        if (g_h_subdevs[i]) g_SubDevice_StopStreamByIndex(g_h_subdevs[i], 0);
    }
    ClearFrameSyncBox();

    for (int i = 0; i < CAM_COUNT; ++i) {
        if (!g_h_subdevs[i]) continue;
        int r = (mode == TRIGGER_HARD_LINE) ? SetHardTriggerMode(g_h_subdevs[i], i) : SetSoftContinuousMode(g_h_subdevs[i], i);
        if (r != 0) ret = -1;
    }

    for (int i = 0; i < CAM_COUNT; ++i) {
        if (g_h_subdevs[i]) {
            int r = g_SubDevice_StartStreamByIndex(g_h_subdevs[i], 0, MyStreamCallBack, &g_cam_indices[i], BUFFER_COUNT);
            if (r != 0) ret = -1;
        }
    }

    if (ret == 0) {
        s_camera_trigger_mode = mode;
        pthread_mutex_lock(&g_params_lock);
        g_current_trigger_mode = (mode == TRIGGER_HARD_LINE) ? TRIGGER_MODE_HARD_LINE : TRIGGER_MODE_SOFT_CONTINUOUS;
        pthread_mutex_unlock(&g_params_lock);
    }
    pthread_mutex_unlock(&g_trigger_switch_lock);
    return ret;
}

static int SetNodeFromNodesTxT(HSUBDEVICE h_subdev) {
    char line[1024]; char nodeName[128]; char nodeValue[1024];
    FILE* fp = fopen("nodes.txt", "r");
    if (!fp) return -1;
    while (fgets(line, sizeof(line), fp)) {
        size_t len = strlen(line);
        if (len > 0 && line[len - 1] == '\n') line[len - 1] = '\0';
        if (len > 1 && line[len - 2] == '\r') line[len - 2] = '\0';
        char* colonPos = strchr(line, ':');
        if (colonPos) {
            *colonPos = '\0';
            strncpy(nodeName, line, sizeof(nodeName)-1);
            strncpy(nodeValue, colonPos+1, sizeof(nodeValue)-1);
            NodeType type;
            if (g_GetNodeType(h_subdev, nodeName, &type) != 0) continue;
            switch(type) {
                case nodeTypeINTEGER: g_SetNodeValue_Int(h_subdev, nodeName, strtoll(nodeValue, NULL, 10)); break;
                case nodeTypeFLOAT: g_SetNodeValue_Double(h_subdev, nodeName, strtod(nodeValue, NULL)); break;
                case nodeTypeBOOLEAN: g_SetNodeValue_Bool(h_subdev, nodeName, (strcmp(nodeValue, "true") == 0)); break;
                case nodeTypeCOMMAND: g_SetNodeValue_Command(h_subdev, nodeName); break;
                case nodeTypeENUMERATION: {
                    char* usPos = strchr(nodeValue, '_');
                    if(usPos) { *usPos='\0'; g_SetNodeValue_Int(h_subdev, nodeName, strtoll(nodeValue, NULL, 10)); }
                    break;
                }
                case nodeTypeSTRING: g_SetNodeValue_Chars(h_subdev, nodeName, nodeValue); break;
                default: break;
            }
        }
    }
    fclose(fp); return 0;
}

static int ConfigureMasterAutoExposure(HSUBDEVICE h_sub, int cam_idx, uint8_t gain_raw, uint8_t expected_gray_value) {
    g_SetNodeValue_Int(h_sub, "ExposureAuto", 1);
    g_SetNodeValue_Int(h_sub, "GainAuto", 0);
    g_SetNodeValue_Double(h_sub, "Gain", (double)gain_raw / 10.0);
    g_SetNodeValue_Int(h_sub, "ExpectedGrayValue", (int64_t)expected_gray_value);
    return 0;
}

static int ConfigureSlaveAutoFollow(HSUBDEVICE h_sub, int cam_idx, uint8_t gain_raw, uint8_t expected_gray_value) {
    g_SetNodeValue_Int(h_sub, "ExposureAuto", 0);
    g_SetNodeValue_Int(h_sub, "GainAuto", 0);
    g_SetNodeValue_Double(h_sub, "Gain", (double)gain_raw / 10.0);
    g_SetNodeValue_Int(h_sub, "ExpectedGrayValue", (int64_t)expected_gray_value);
    return 0;
}

static void SyncSlaveExposureFromMaster(void) {
    if (!s_master_auto_sync_enabled || !g_GetNodeValue_Double || s_master_auto_cam_idx < 0 || s_master_auto_cam_idx >= CAM_COUNT) return;
    HSUBDEVICE h_master = g_h_subdevs[s_master_auto_cam_idx];
    if (!h_master) return;

    pthread_mutex_lock(&g_auto_exposure_sync_lock);
    double master_exp_us = 0.0;
    if (g_GetNodeValue_Double(h_master, "ExposureTime", &master_exp_us) != 0 || master_exp_us <= 0.0) {
        pthread_mutex_unlock(&g_auto_exposure_sync_lock); return;
    }

    if (s_last_synced_exposure_us > 0.0 && fabs(master_exp_us - s_last_synced_exposure_us) < s_exposure_sync_threshold_us) {
        pthread_mutex_unlock(&g_auto_exposure_sync_lock); return;
    }

    for (int i = 0; i < CAM_COUNT; ++i) {
        if (i == s_master_auto_cam_idx || !g_h_subdevs[i]) continue;
        g_SetNodeValue_Double(g_h_subdevs[i], "ExposureTime", master_exp_us);
    }
    s_last_synced_exposure_us = master_exp_us;
    pthread_mutex_unlock(&g_auto_exposure_sync_lock);
}

void UpdateCameraHardwareParams(int cam_idx, uint8_t command, uint32_t exp_us, uint8_t gain, uint8_t expected_gray_value) {
    if (cam_idx < 0 || cam_idx >= CAM_COUNT || !g_h_subdevs[cam_idx]) return;
    HSUBDEVICE h_sub = g_h_subdevs[cam_idx];

    if (command == CMD_MANUAL_EXP) {
        s_master_auto_sync_enabled = 0;
        g_SetNodeValue_Int(h_sub, "ExposureAuto", 0);
        g_SetNodeValue_Int(h_sub, "GainAuto", 0);
        g_SetNodeValue_Double(h_sub, "ExposureTime", (double)exp_us);
        g_SetNodeValue_Double(h_sub, "Gain", (double)gain / 10.0);
    } else if (command == CMD_AUTO_EXP) {
        s_auto_fixed_gain_raw = gain;
        s_auto_expected_gray = expected_gray_value;
        if (cam_idx == s_master_auto_cam_idx) {
            if (ConfigureMasterAutoExposure(h_sub, cam_idx, gain, expected_gray_value) == 0) {
                s_master_auto_sync_enabled = 1; s_last_synced_exposure_us = -1.0;
            }
        } else {
            ConfigureSlaveAutoFollow(h_sub, cam_idx, gain, expected_gray_value);
        }
    }
}

// ==================== [核心修改区] ====================
void MyStreamCallBack(HIMAGE h_img,
                      HSUBDEVICE h_dev,
                      unsigned int streamIndex,
                      unsigned int event,
                      void* context)
{
    int cam_index = *(int*)context;
    ImageInfo imageInfo = g_Image_GetImageInfo(h_img);

    // 自动曝光同步
    if (s_master_auto_sync_enabled &&
        cam_index == s_master_auto_cam_idx)
    {
        SyncSlaveExposureFromMaster();
    }

    pthread_mutex_lock(&g_sync_box.lock);

    // 如果这一位置已经有旧图，直接释放
    if (g_sync_box.images[cam_index] != NULL)
    {
        g_Image_Recycle(g_sync_box.images[cam_index]);
    }

    // 保存最新图像
    g_sync_box.images[cam_index] = h_img;

    // 保存SDK原始BlockID（仅用于调试）
    g_sync_box.block_ids[cam_index] = imageInfo.blockId;

    // 标记该路已经到达
    g_sync_box.ready_mask |= (1 << cam_index);

    // 四路齐了，通知consumer线程
    if (g_sync_box.ready_mask == 0x0F)
    {
        pthread_cond_signal(&g_sync_box.cond);
    }

    pthread_mutex_unlock(&g_sync_box.lock);
}
// ======================================================

int ConnectAndStartCameras(HMANAGER* h_mgr_out) {
    HMANAGER h_mgr = g_Manager_Create(INTERFACE_TYPE_MISC);
    if (!h_mgr) return -1;
    *h_mgr_out = h_mgr;
    int connected_count = 0;

    for (int i = 0; i < 10 && connected_count < CAM_COUNT; i++) {
        HDEVICE h_dev = g_Manager_GetDeviceByIndex(h_mgr, i, INTERFACE_TYPE_MISC);
        if (!h_dev) continue;
        int sub_count = g_Device_GetSubDeviceCount(h_dev);
        bool dev_has_target = false;

        for (int j = 0; j < sub_count; j++) {
            HSUBDEVICE h_sub = g_Device_GetSubDeviceByIndex(h_dev, j);
            if (!h_sub) continue;
            if (g_SubDevice_Connect(h_sub) == 0) {
                char name[256] = {0};
                g_SubDevice_GetSubDeviceName(h_sub, name, sizeof(name));
                bool matched = false;
                for (int k = 0; k < CAM_COUNT; k++) {
                    if (g_h_subdevs[k] == NULL && strstr(name, TARGET_SNS[k]) != NULL) {
                        g_h_subdevs[k] = h_sub; g_h_devs[k] = h_dev;
                        connected_count++; matched = true; dev_has_target = true;
                        SetNodeFromNodesTxT(h_sub); break;
                    }
                }
                if (!matched) { g_SubDevice_DisConnect(h_sub); g_SubDevice_Destory(h_sub); }
            }
        }
        if (!dev_has_target) g_Device_Destory(h_dev);
    }

    if (connected_count < CAM_COUNT) return -1;

    for (int i = 0; i < CAM_COUNT; i++) {
        g_SubDevice_OpenStreamByIndex(g_h_subdevs[i], 0, BUFFER_COUNT);
        g_SubDevice_StartStreamByIndex(g_h_subdevs[i], 0, MyStreamCallBack, &g_cam_indices[i], BUFFER_COUNT);
    }

    uint8_t default_gain_raw = 10;
    uint8_t default_expected_gray = 120;
    UpdateCameraHardwareParams(s_master_auto_cam_idx, CMD_AUTO_EXP, 0, default_gain_raw, default_expected_gray);
    for (int i = 0; i < CAM_COUNT; ++i) {
        if (i == s_master_auto_cam_idx) continue;
        UpdateCameraHardwareParams(i, CMD_AUTO_EXP, 0, default_gain_raw, default_expected_gray);
    }
    return 0;
}

void StopAndDisconnectCameras(void) {
    s_master_auto_sync_enabled = 0;
    for (int i = 0; i < CAM_COUNT; i++) {
        if (g_h_subdevs[i]) {
            g_SubDevice_StopStreamByIndex(g_h_subdevs[i], 0);
            g_SubDevice_CloseStreamByIndex(g_h_subdevs[i], 0);
            g_SubDevice_DisConnect(g_h_subdevs[i]);
            g_SubDevice_Destory(g_h_subdevs[i]);
        }
    }
    for (int i = 0; i < CAM_COUNT; i++) {
        if (g_h_devs[i]) {
            bool is_duplicate = false;
            for (int j = 0; j < i; j++) if (g_h_devs[i] == g_h_devs[j]) is_duplicate = true;
            if (!is_duplicate) g_Device_Destory(g_h_devs[i]);
        }
    }
}
