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
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "MPU6050.h"  
#include "MPU6050_6Axis_MotionApps20.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mpu_calib.h"
#include <math.h>


//I2C及MPU相关变量
#define I2C_MASTER_SCL_IO 9      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define MPU6050_ADDR             MPU6050_ADDRESS_AD0_HIGH 


// 当前动作状态（互斥，5选1）
#define EVT_MODE_IDLE   (1<<0)
#define EVT_MODE_ACT1   (1<<1)
#define EVT_MODE_ACT2   (1<<2)
#define EVT_MODE_STEP   (1<<3)   // 踏步
#define EVT_MODE_ACT4   (1<<4)

// 动作切换通知（瞬时事件）
#define EVT_MODE_CHANGED (1<<8)

// 方便用的 mask
#define EVT_MODE_MASK (EVT_MODE_IDLE|EVT_MODE_ACT1|EVT_MODE_ACT2|EVT_MODE_STEP|EVT_MODE_ACT4)



//dmp相关变量
static bool g_dmp_ready = false;
static uint16_t g_dmp_packet_size = 0;

static QueueHandle_t g_det_q = NULL;     //动作识别队列句柄
static EventGroupHandle_t g_evt = NULL;  //动作类型事件句柄


//动作识别所用数据
struct detection_data{
    float acc_x,acc_y,acc_z;
    float gyr_x, gyr_y, gyr_z;
    float yaw,pitch,roll;
    Quaternion q;

    uint32_t tick; //时间戳
    bool have_dmp;
  
};


static MPU6050 mpu(MPU6050_ADDR);

static const char *TAG = "main";

//系数换算工具
static float accel_lsb_per_g_from_afs(uint8_t afs_sel) {
    // AFS_SEL: 0=±2g,1=±4g,2=±8g,3=±16g
    switch (afs_sel & 0x03) {
        case 0: return 16384.0f;
        case 1: return  8192.0f;
        case 2: return  4096.0f;
        case 3: return  2048.0f;
        default:return 16384.0f;
    }
}

static float gyro_lsb_per_dps_from_fs(uint8_t fs_sel) {
    // FS_SEL: 0=±250,1=±500,2=±1000,3=±2000 (deg/s)
    switch (fs_sel & 0x03) {
        case 0: return 131.0f;
        case 1: return  65.5f;
        case 2: return  32.8f;
        case 3: return  16.4f;
        default:return 131.0f;
    }
}


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

static void dump_accel_gyro_cfg(uint8_t *out_afs, uint8_t *out_fs) {
    uint8_t aconf=0, gconf=0;
    mpu.ReadRegister(0x1C, &aconf, 1);
    mpu.ReadRegister(0x1B, &gconf, 1);

    uint8_t afs = (aconf >> 3) & 0x03;
    uint8_t fs  = (gconf >> 3) & 0x03;

    if (out_afs) *out_afs = afs;
    if (out_fs)  *out_fs  = fs;

    ESP_LOGI(TAG, "ACCEL_CONFIG=0x%02X (AFS_SEL=%u)  GYRO_CONFIG=0x%02X (FS_SEL=%u)",
        aconf, afs, gconf, fs
    );
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


// -------------------- DMP 初始化 --------------------
static void mpu_dmp_init_or_die(void) {
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        ESP_LOGE(TAG, "dmpInitialize failed, code=%u", devStatus);
        abort();
    }

    // 暂不开启 FIFO + DMP，防止与校准的交叉初始化
    mpu.setDMPEnabled(false);
    mpu.setFIFOEnabled(false);
    mpu.resetFIFO();

    g_dmp_packet_size = mpu.dmpGetFIFOPacketSize();
    g_dmp_ready = true;

    ESP_LOGI(TAG, "DMP ready. packetSize=%u", (unsigned)g_dmp_packet_size);
}



//动作切换接口函数
static void set_sport_mode(int mode /*0 idle, 1..4动作*/) {
    EventBits_t setBit = EVT_MODE_IDLE;
    switch (mode) {
        case 1: setBit = EVT_MODE_ACT1; break;
        case 2: setBit = EVT_MODE_ACT2; break;
        case 3: setBit = EVT_MODE_STEP; break; // 踏步
        case 4: setBit = EVT_MODE_ACT4; break;
        default:setBit = EVT_MODE_IDLE; break;
    }

    // 1) 清掉旧模式位
    xEventGroupClearBits(g_evt, EVT_MODE_MASK);

    // 2) 设置新模式位 + 发出“模式改变”事件
    xEventGroupSetBits(g_evt, setBit | EVT_MODE_CHANGED);
}


// -------------------- MPU读数据任务 --------------------
static void mpu_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(50); // 20Hz 
    uint8_t fifoBuffer[64]; // 42字节包够用，留点余量

   
    // 读一次当前档位（避免写死换算系数）
    uint8_t afs=0, fs=0;
    dump_accel_gyro_cfg(&afs, &fs);
    float acc_lsb_per_g = accel_lsb_per_g_from_afs(afs);
    float gyr_lsb_per_dps = gyro_lsb_per_dps_from_fs(fs);

    while (1) {
        detection_data det = {};
        // -------- 1) DMP优先：从FIFO拿姿态 --------
        VectorFloat gravity;
        float ypr[3] = {0};

        bool have_dmp = false;


        if (g_dmp_ready) {
            uint16_t fifoCount = mpu.getFIFOCount();

            // FIFO overflow（MPU6050 FIFO最大1024字节）
            if (fifoCount >= 1024) {
                mpu.resetFIFO();
                ESP_LOGW(TAG, "FIFO overflow -> resetFIFO");
            } else if (fifoCount >= g_dmp_packet_size) {
                // 只取一包（你也可以while取到只剩<packetSize）
                mpu.getFIFOBytes(fifoBuffer, g_dmp_packet_size);

                mpu.dmpGetQuaternion(&det.q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &det.q);
                mpu.dmpGetYawPitchRoll(ypr, &det.q, &gravity);

                have_dmp = true;
            }
        }
        det.have_dmp = have_dmp;

        // -------- 2) RAW：寄存器读 accel/gyro/temp --------
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        int16_t tRaw = mpu.getTemperature();
        float tempC = (float)tRaw / 340.0f + 36.53f;

        det.acc_x = (float)ax / acc_lsb_per_g;
        det.acc_y = (float)ay / acc_lsb_per_g;
        det.acc_z = (float)az / acc_lsb_per_g;

        det.gyr_x = (float)gx / gyr_lsb_per_dps;
        det.gyr_y = (float)gy / gyr_lsb_per_dps;
        det.gyr_z = (float)gz / gyr_lsb_per_dps;

        // -------- 输出：姿态(DMP) + raw惯导 --------
        if (have_dmp) {
            // ypr是弧度，转角度方便看
            det.yaw   = ypr[0] * 180.0f / (float)M_PI;
            det.pitch = ypr[1] * 180.0f / (float)M_PI;
            det.roll  = ypr[2] * 180.0f / (float)M_PI;

            ESP_LOGI(TAG,
                "Q[wxyz]=[%.4f %.4f %.4f %.4f] YPR[deg]=[%.1f %.1f %.1f] | "
                "A[g]=[%.2f %.2f %.2f] G[dps]=[%.2f %.2f %.2f] T=%.2fC",
                det.q.w, det.q.x, det.q.y, det.q.z,
                det.yaw, det.pitch, det.roll,
                det.acc_x, det.acc_y, det.acc_z,
                det.gyr_x, det.gyr_y, det.gyr_z,
                tempC
            );
        } else {
            ESP_LOGI(TAG,
                "DMP(not ready/no packet) | A[g]=[%.2f %.2f %.2f] G[dps]=[%.2f %.2f %.2f] T=%.2fC",
                det.acc_x, det.acc_y, det.acc_z,
                det.gyr_x, det.gyr_y, det.gyr_z,
                tempC
            );
            det.yaw = det.pitch = det.roll = NAN;
        }

 
        det.tick = (uint32_t)xTaskGetTickCount();
        xQueueOverwrite(g_det_q, &det);
        

        vTaskDelay(period);
    }
}



/*----------------------------动作识别任务----------------------------------*/
static void detect_task(void *arg) {
    detection_data det;
    EventBits_t bits;

    int current_mode = 0; // 0 idle, 1..4

    while (1) {

        // ① 如果发生模式切换：读取并清掉“切换事件”
        bits = xEventGroupWaitBits(
            g_evt,
            EVT_MODE_CHANGED,     // 等这个bit
            pdTRUE,               // 自动清掉 EVT_MODE_CHANGED
            pdFALSE,
            0                     // 0=不阻塞（只检查一下）
        );

        if (bits & EVT_MODE_CHANGED) {
            EventBits_t modeBits = xEventGroupGetBits(g_evt) & EVT_MODE_MASK;
            if      (modeBits & EVT_MODE_ACT1) current_mode = 1;
            else if (modeBits & EVT_MODE_ACT2) current_mode = 2;
            else if (modeBits & EVT_MODE_STEP) current_mode = 3; // 踏步
            else if (modeBits & EVT_MODE_ACT4) current_mode = 4;
            else                               current_mode = 0;

            // 模式切换时常见操作：清零计数、重置状态机等
            // step_reset();
            ESP_LOGI(TAG, "Mode changed -> %d", current_mode);
        }

        // ② 等待一帧最新数据（Queue）
        if (xQueueReceive(g_det_q, &det, portMAX_DELAY) != pdTRUE) continue;
        if (!det.have_dmp) continue; // 你如果不想处理无DMP帧就跳过

        // ③ 根据模式选择算法
        switch (current_mode) {
            case 0: // IDLE 等待
                // 什么都不做，或做低频检测
                break;

            case 1:
                // action1_update(&det);
                break;

            case 2:
                // action2_update(&det);
                break;

            case 3:
                // === 踏步（第三个动作）示例 ===
                // step_detect_update(&det);
                // 例如简单打印一下：
                // ESP_LOGI(TAG, "STEP mode: az=%.2f", det.acc_z);
                break;

            case 4:
                // action4_update(&det);
                break;
        }
    }
}


extern "C" void app_main(void)
{

    ESP_ERROR_CHECK(nvs_flash_init());


    //Queue句柄创建
    g_det_q = xQueueCreate(1,sizeof(detection_data));
    if (!g_det_q) { ESP_LOGE(TAG, "queue create failed"); abort(); }

    //事件传输句柄创建
    g_evt = xEventGroupCreate();
    if (!g_evt) { ESP_LOGE(TAG, "event group create failed"); abort(); }

    // 默认等待状态
    xEventGroupSetBits(g_evt, EVT_MODE_IDLE);


    i2c_sensor_mpu6050_init();

    uint8_t id = mpu.getDeviceID();
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", id);


    
    //DMP初始化
    mpu_dmp_init_or_die();

    // 传感器校准：    true： 强制执行校准   false：NVS中有存储数据则直接使用校准数据，不进行校准
    ESP_ERROR_CHECK(mpu_calib_apply_or_calibrate(mpu, 6 /*loops*/, false /*force*/));
    
    //启动DMP
    mpu.resetFIFO();
    mpu.setFIFOEnabled(true);
    mpu.setDMPEnabled(true);


    xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 6, NULL);//读数据任务优先级稍微高一些

    xTaskCreate(detect_task, "detect_task", 4096, NULL, 5, NULL);
    

}
