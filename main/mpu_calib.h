#pragma once

#include <stdint.h>
#include "esp_err.h"

#include "MPU6050.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t ax_off, ay_off, az_off;
    int16_t gx_off, gy_off, gz_off;
} mpu_offsets_t;

/**
 * @brief 初始化 NVS
 */
esp_err_t mpu_calib_nvs_init(void);

/**
 * @brief 从 NVS 读取 offset
 */
esp_err_t mpu_calib_offsets_load(mpu_offsets_t *o);

/**
 * @brief 把 offset 写入 NVS（commit）
 */
esp_err_t mpu_calib_offsets_save(const mpu_offsets_t *o);

/**
 * @brief 把 offset 应用到 MPU6050（写入寄存器）
 */
void mpu_calib_offsets_apply(MPU6050 &mpu, const mpu_offsets_t *o);

/**
 * @brief 从 MPU6050 读出当前 offset（从寄存器读出来）
 */
void mpu_calib_offsets_read_back(MPU6050 &mpu, mpu_offsets_t *o);

/**
 * @brief 一键流程：
 *        - 若 NVS 有 offset：读取并 apply（无需校准）
 *        - 若 NVS 没有：执行 CalibrateGyro/Accel -> 读回 offset -> 保存到 NVS -> apply
 *
 * @param mpu  已经 initialize + testConnection 成功的对象
 * @param loops 校准 Loops（建议 5~8，越大越慢越稳）
 * @param force_calibrate 强制重新校准（忽略 NVS）
 */
esp_err_t mpu_calib_apply_or_calibrate(MPU6050 &mpu, uint8_t loops, bool force_calibrate);

/**
 * @brief 删除已保存的 offset（下次开机将重新校准）
 */
esp_err_t mpu_calib_offsets_erase(void);

#ifdef __cplusplus
}
#endif
