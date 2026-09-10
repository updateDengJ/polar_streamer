// ==========================================
// 文件名: camera.h
// 描  述: 相机连接、配置下发与数据流回调
// ==========================================
#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include "EzSDK_C.h"

typedef enum {
    TRIGGER_SOFT_CONTINUOUS = 0,
    TRIGGER_HARD_LINE = 1
} CameraTriggerMode;

int ConnectAndStartCameras(HMANAGER* h_mgr_out);
void UpdateCameraHardwareParams(int cam_idx,
                                uint8_t command,
                                uint32_t exp_us,
                                uint8_t gain,
                                uint8_t expected_gray_value);
int SwitchAllCameraTriggerMode(CameraTriggerMode mode);
void StopAndDisconnectCameras(void);

#endif // CAMERA_H
