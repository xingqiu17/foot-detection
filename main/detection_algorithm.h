#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "detection_types.h"   

#ifdef __cplusplus
extern "C" {
#endif


struct SitActionParams {
    
  // pitch 门禁（相对起始）
    float UP_PITCH_DELTA_MIN;
    float HIGH_PITCH_MIN;
    float BACK_PITCH_DELTA_MAX;

    // yaw 稳定
    float YAW_DELTA_MAX;

    // 角速度（deg/s）
    float GYRO_MOVE_TH;
    float GYRO_IDLE_TH;

    // 防抖
    int need_up_cnt;
    int need_idle_cnt;
    int need_down_cnt;

    // 超时
    uint32_t UP_TIMEOUT_MS;
    uint32_t HIGH_IDLE_TIMEOUT_MS;
    uint32_t DOWN_TIMEOUT_MS;

};

extern const SitActionParams SIT_LIFT_PARAMS;
extern const SitActionParams SIT_ANKLE_PARAMS;


bool step_update(const detection_data_t* det, int* step_total);
bool sit_update(const detection_data* det,uint8_t sport_flag);
void step_reset(void);
#ifdef __cplusplus
}
#endif
