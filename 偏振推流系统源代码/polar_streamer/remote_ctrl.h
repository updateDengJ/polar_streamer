#ifndef REMOTE_CTRL_H
#define REMOTE_CTRL_H

#include "config.h"

int start_remote_server(int port);
void UpdateCameraHardwareParams(int cam_idx,
                                uint8_t command,
                                uint32_t exp_us,
                                uint8_t gain,
                                uint8_t expected_gray_value);

#endif // REMOTE_CTRL_H
