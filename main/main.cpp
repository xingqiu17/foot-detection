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
#include "MPU6050.h"  // 如果你要DMP才换成 MotionApps20 头


#define I2C_MASTER_SCL_IO 9      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define MPU6050_ADDR         0x69   // MPU6050_I2C_ADDRESS_1 一般就是 0x68

static MPU6050 mpu(MPU6050_ADDR);

static const char *TAG = "main";


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
 * @brief i2c device initialization
 */
static void i2c_sensor_mpu6050_init(void)
{

    i2c_bus_init();
    mpu.initialize();
    if (!mpu.testConnection()) {
        ESP_LOGE(TAG, "MPU6050 testConnection failed");
        abort();
    }

}


static void mpu6050_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(50); // 20Hz；
    while (1) {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        // 读加速度+陀螺仪（替代 mpu6050_get_acce + mpu6050_get_gyro）
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // 温度（替代 mpu6050_get_temp）
        int16_t tRaw = mpu.getTemperature();
        float tempC = (float)tRaw / 340.0f + 36.53f;

        // ⚠️ 你之前打印的是“工程化单位(float)”
        // 这里我们做同样的换算：±4g => 8192 LSB/g；±500dps => 65.5 LSB/(deg/s)
        // 如果你改了量程，这两个系数也要跟着改
        float acc_x = (float)ax / 8192.0f;
        float acc_y = (float)ay / 8192.0f;
        float acc_z = (float)az / 8192.0f;

        float gyr_x = (float)gx / 65.5f;
        float gyr_y = (float)gy / 65.5f;
        float gyr_z = (float)gz / 65.5f;

        ESP_LOGI(TAG,
                 "acce[g]=[%.2f, %.2f, %.2f] gyro[dps]=[%.2f, %.2f, %.2f] temp=%.2fC",
                 acc_x, acc_y, acc_z,
                 gyr_x, gyr_y, gyr_z,
                 tempC);

        vTaskDelay(period);
    }
}


//TEST_CASE("Sensor mpu6050 test", "[mpu6050][iot][sensor]")
extern "C" void app_main(void)
{


    i2c_sensor_mpu6050_init();

    uint8_t id = mpu.getDeviceID(); // 替代 mpu6050_get_deviceid
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", id);
    
   
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);


    
    // mpu6050_delete(mpu6050);
    // ret = i2c_driver_delete(I2C_MASTER_NUM);
    // ESP_LOGE(TAG, "ret=%d (%s)", ret, esp_err_to_name(ret));
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
}
