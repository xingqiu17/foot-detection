#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "MPU6050.h"   // Quaternion, VectorInt16, VectorFloat 等在这里

#ifdef __cplusplus
extern "C" {
#endif

typedef struct detection_data {
    float acc_x, acc_y, acc_z;        // raw accel (g) - 你现在已有
    float gyr_x, gyr_y, gyr_z;        // gyro (dps)
    float yaw, pitch, roll;          // deg
    VectorFloat gravity;             // DMP gravity
    Quaternion q;                    // DMP quaternion

    // ✅ 世界坐标线加速度（已去重力）——你算法需要的
    float lin_wx, lin_wy, lin_wz;     // unit: g

    uint32_t tick;                   // 时间戳
    bool have_dmp;                   // 是否有效DMP数据
} detection_data_t;

#ifdef __cplusplus
}
#endif
