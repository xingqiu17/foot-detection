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


struct StandActionParams {
  
  //加速度门禁（决定是否动作开始）
  float up_th;    // g
  float down_th;    // g
  float land_th;    // g
  float near0_th;   // g

  // 防抖连续次数
  int need_up_cnt;
  int need_down_cnt;
  int need_land_cnt;
  int need_near0_cnt;

  // ===== 即时响应（20Hz下约9帧）=====
  uint32_t UP_TIMEOUT_MS;
  uint32_t DOWN_TIMEOUT_MS;

  // ===== 线速度门禁（判断方向和速度）=====
  float VZ_UP_TH;   // m/s
  float VZ_DOWN_TH;   // m/s（向下用 -VZ_DOWN_TH）

  float Z_MIN_LIFT;  // m
  float Z_BACK_TH;  // m

  float PITCH_FLAT_DEG;

};

extern const StandActionParams STAND_STEP_PARAMS;
extern const StandActionParams STAND_HIGH_PARAMS;


bool step_update(const detection_data_t* det, int* step_total,uint8_t sport_flag);
bool sit_update(const detection_data* det,uint8_t sport_flag);
void sit_reset(void);
void step_reset(void);
#ifdef __cplusplus
}
#endif
