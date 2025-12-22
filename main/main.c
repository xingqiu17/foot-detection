/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 9      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "main";
static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{

    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS_1);

    ESP_ERROR_CHECK(mpu6050 ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS));

    ESP_ERROR_CHECK(mpu6050_wake_up(mpu6050));

}


static void mpu6050_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(50); // 20Hz；
    while (1) {
        mpu6050_acce_value_t acce;
        mpu6050_gyro_value_t gyro;
        mpu6050_temp_value_t temp;

        esp_err_t r1 = mpu6050_get_acce(mpu6050, &acce);
        esp_err_t r2 = mpu6050_get_gyro(mpu6050, &gyro);
        esp_err_t r3 = mpu6050_get_temp(mpu6050, &temp);

        if (r1 == ESP_OK && r2 == ESP_OK && r3 == ESP_OK) {
            ESP_LOGI(TAG,
                     "acce[%.2f, %.2f, %.2f] gyro[%.2f, %.2f, %.2f] temp=%.2f",
                     acce.acce_x, acce.acce_y, acce.acce_z,
                     gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
                     temp.temp);
        } else {
            ESP_LOGW(TAG, "read fail: acce=%s gyro=%s temp=%s",
                     esp_err_to_name(r1), esp_err_to_name(r2), esp_err_to_name(r3));
        }

        vTaskDelay(period);
    }
}


//TEST_CASE("Sensor mpu6050 test", "[mpu6050][iot][sensor]")
void app_main(void)
{
    esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    i2c_sensor_mpu6050_init();

    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid));
    
    ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050, &acce));

    ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050, &gyro));

    ESP_ERROR_CHECK(mpu6050_get_temp(mpu6050, &temp));
   
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);


    
    // mpu6050_delete(mpu6050);
    // ret = i2c_driver_delete(I2C_MASTER_NUM);
    // ESP_LOGE(TAG, "ret=%d (%s)", ret, esp_err_to_name(ret));
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
}
