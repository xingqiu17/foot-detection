#include "mpu_calib.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu_calib";
static const char *NVS_NS = "mpu6050";

esp_err_t mpu_calib_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t mpu_calib_offsets_load(mpu_offsets_t *o)
{
    if (!o) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    err = nvs_get_i16(h, "ax", &o->ax_off); if (err != ESP_OK) { nvs_close(h); return err; }
    err = nvs_get_i16(h, "ay", &o->ay_off); if (err != ESP_OK) { nvs_close(h); return err; }
    err = nvs_get_i16(h, "az", &o->az_off); if (err != ESP_OK) { nvs_close(h); return err; }

    err = nvs_get_i16(h, "gx", &o->gx_off); if (err != ESP_OK) { nvs_close(h); return err; }
    err = nvs_get_i16(h, "gy", &o->gy_off); if (err != ESP_OK) { nvs_close(h); return err; }
    err = nvs_get_i16(h, "gz", &o->gz_off); if (err != ESP_OK) { nvs_close(h); return err; }

    nvs_close(h);
    return ESP_OK;
}

esp_err_t mpu_calib_offsets_save(const mpu_offsets_t *o)
{
    if (!o) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    ESP_ERROR_CHECK(nvs_set_i16(h, "ax", o->ax_off));
    ESP_ERROR_CHECK(nvs_set_i16(h, "ay", o->ay_off));
    ESP_ERROR_CHECK(nvs_set_i16(h, "az", o->az_off));

    ESP_ERROR_CHECK(nvs_set_i16(h, "gx", o->gx_off));
    ESP_ERROR_CHECK(nvs_set_i16(h, "gy", o->gy_off));
    ESP_ERROR_CHECK(nvs_set_i16(h, "gz", o->gz_off));

    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void mpu_calib_offsets_apply(MPU6050 &mpu, const mpu_offsets_t *o)
{
    if (!o) return;

    mpu.setXAccelOffset(o->ax_off);
    mpu.setYAccelOffset(o->ay_off);
    mpu.setZAccelOffset(o->az_off);

    mpu.setXGyroOffset(o->gx_off);
    mpu.setYGyroOffset(o->gy_off);
    mpu.setZGyroOffset(o->gz_off);
}

void mpu_calib_offsets_read_back(MPU6050 &mpu, mpu_offsets_t *o)
{
    if (!o) return;

    o->ax_off = mpu.getXAccelOffset();
    o->ay_off = mpu.getYAccelOffset();
    o->az_off = mpu.getZAccelOffset();

    o->gx_off = mpu.getXGyroOffset();
    o->gy_off = mpu.getYGyroOffset();
    o->gz_off = mpu.getZGyroOffset();
}

esp_err_t mpu_calib_offsets_erase(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    // 直接擦 namespace 更省事
    err = nvs_erase_all(h);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t mpu_calib_apply_or_calibrate(MPU6050 &mpu, uint8_t loops, bool force_calibrate)
{
    if (loops == 0) loops = 1;

    // 先确保 NVS OK
    esp_err_t nvs_ret = mpu_calib_nvs_init();
    if (nvs_ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(nvs_ret));
        return nvs_ret;
    }

    mpu_offsets_t off{};
    esp_err_t load_ret = force_calibrate ? ESP_ERR_NVS_NOT_FOUND : mpu_calib_offsets_load(&off);

    if (load_ret == ESP_OK) {
        ESP_LOGI(TAG, "Offsets found in NVS, apply directly.");
        mpu_calib_offsets_apply(mpu, &off);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "No offsets in NVS (%s) or force_calibrate=1. Start calibration...",
             esp_err_to_name(load_ret));

    // 校准前让设备“静止一下”，减少你手抖带来的偏差
    vTaskDelay(pdMS_TO_TICKS(200));

    // 关键：你的 CalibrateGyro/Accel 内部会把 offset 写到 MPU6050 寄存器
    mpu.CalibrateGyro(loops);
    mpu.CalibrateAccel(loops);

    // 把写进去的 offset 读出来存 NVS
    mpu_calib_offsets_read_back(mpu, &off);

    ESP_LOGI(TAG, "Calibrated offsets: A[%d,%d,%d] G[%d,%d,%d]",
             off.ax_off, off.ay_off, off.az_off,
             off.gx_off, off.gy_off, off.gz_off);

    esp_err_t save_ret = mpu_calib_offsets_save(&off);
    if (save_ret != ESP_OK) {
        ESP_LOGE(TAG, "Save offsets to NVS failed: %s", esp_err_to_name(save_ret));
        return save_ret;
    }

    // 再 apply 一次（其实校准已经写过了，这里只是保证一致）
    mpu_calib_offsets_apply(mpu, &off);

    ESP_LOGI(TAG, "Offsets saved to NVS and applied.");
    return ESP_OK;
}
