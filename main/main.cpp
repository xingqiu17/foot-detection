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

#include "detection_types.h"
#include "detection_algorithm.h"

#include "esp_wifi.h"
#include "espnow.h"
#include "espnow_utils.h"
#include "esp_mac.h"
#include "espnow_sr.h"


 

//I2C及MPU相关变量
#define I2C_MASTER_SCL_IO 3      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 2      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define MPU6050_ADDR             MPU6050_ADDRESS_AD0_LOW 


// 当前动作状态（互斥，5选1）
#define EVT_MODE_IDLE   (1<<0)
#define EVT_MODE_SIT_ANKLE   (1<<1)
#define EVT_MODE_SIT_LIFT   (1<<2)
#define EVT_MODE_STEP   (1<<3)   // 踏步
#define EVT_MODE_HIGH   (1<<4)   //站姿高抬腿

// 动作切换通知（瞬时事件）
#define EVT_MODE_CHANGED (1<<8)

// 方便用的 mask
#define EVT_MODE_MASK (EVT_MODE_IDLE|EVT_MODE_SIT_ANKLE|EVT_MODE_SIT_LIFT|EVT_MODE_STEP|EVT_MODE_HIGH)

uint8_t master_mac[6]={0};
static bool master_mac_valid = false;


//dmp相关变量
static bool g_dmp_ready = false;
static uint16_t g_dmp_packet_size = 0;

static QueueHandle_t g_det_q = NULL;     //动作识别队列句柄
static EventGroupHandle_t g_evt = NULL;  //动作类型事件句柄


//任务控制句柄
static TaskHandle_t mpu_task_handle    = NULL;
static TaskHandle_t detect_task_handle = NULL;

static void mpu_task(void *arg);
static void detect_task(void *arg);


static slave_state_t slave_state = SLAVE_IDLE;
static slave_state_t last_state = SLAVE_IDLE;

static const char *TAG = "main";

static constexpr TickType_t HEARTBEAT_INTERVAL_TICKS = pdMS_TO_TICKS(2000);
static constexpr uint8_t HEARTBEAT_MISS_MAX = 3;
static constexpr TickType_t WAIT_MASTER_CONFIRM_TIMEOUT_TICKS = pdMS_TO_TICKS(3000);
static constexpr wifi_ps_type_t POWER_OFF_WIFI_PS = WIFI_PS_MIN_MODEM;


uint32_t seq = 0;

enum {
    EXERCISE_MODE_SIT_ANKLE = 1,
    EXERCISE_MODE_SIT_LIFT = 2,
    EXERCISE_MODE_STEP = 3,
    EXERCISE_MODE_HIGH = 4,
};


static void send_exercise_data(uint32_t mode)
{
    if (!master_mac_valid) {
        ESP_LOGW(TAG, "Skip EXERCISE_DATA: master mac not ready");
        return;
    }

    espnow_frame_head_t frame_head{};
    frame_head.retransmit_count = 5;
    frame_head.broadcast = false;

    esp_now_data exercise_data = {
        .type = EXERCISE_DATA,
        .seq  = seq++,
        .data = mode
    };

    esp_err_t err = espnow_send(ESPNOW_DATA_TYPE_DATA,
        master_mac,
        &exercise_data,
        sizeof(exercise_data),
        &frame_head,
        pdMS_TO_TICKS(200));

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "EXERCISE_DATA send failed: %s", esp_err_to_name(err));
    }
}

static esp_err_t send_keep_alive(void)
{
    if (!master_mac_valid) {
        ESP_LOGW(TAG, "Skip KEEP_ALIVE: master mac not ready");
        return ESP_ERR_INVALID_STATE;
    }

    espnow_frame_head_t frame_head{};
    frame_head.retransmit_count = 3;
    frame_head.broadcast = false;

    esp_now_data keep_alive = {
        .type = KEEP_ALIVE,
        .seq  = seq++,
        .data = 0
    };

    esp_err_t err = espnow_send(ESPNOW_DATA_TYPE_DATA,
        master_mac,
        &keep_alive,
        sizeof(keep_alive),
        &frame_head,
        pdMS_TO_TICKS(150));

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "KEEP_ALIVE send failed: %s", esp_err_to_name(err));
    }

    return err;
}





static MPU6050 mpu(MPU6050_ADDR);
static bool g_sensor_stack_initialized = false;

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


uint8_t g_master_mac[6];//主设备地址



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


// -------------------- WIFI 初始化 --------------------
static void app_wifi_init()
{
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}



//动作切换接口函数
static void set_sport_mode(int mode /*0 idle, 1..4动作*/) {
    EventBits_t setBit = EVT_MODE_IDLE;
    switch (mode) {
        case 1: setBit = EVT_MODE_SIT_ANKLE; break;
        case 2: setBit = EVT_MODE_SIT_LIFT; break;
        case 3: setBit = EVT_MODE_STEP; break; // 踏步
        case 4: setBit = EVT_MODE_HIGH; break;
        default:setBit = EVT_MODE_IDLE; break;
    }

    // 1) 清掉旧模式位
    xEventGroupClearBits(g_evt, EVT_MODE_MASK);

    // 2) 设置新模式位 + 发出“模式改变”事件
    xEventGroupSetBits(g_evt, setBit | EVT_MODE_CHANGED);
}


//保存主设备地址
esp_err_t nvs_save_master_mac(const uint8_t *mac)
{
    nvs_handle_t nvs;
    esp_err_t err;

    err = nvs_open("pairing", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs, "master_mac", mac, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return err;
    }

    err = nvs_commit(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Master MAC saved to NVS");
    }
   
    nvs_close(nvs);
    return err;
}


// -------------------- 低功耗策略相关 --------------------


static esp_err_t send_power_manage_reply(const uint8_t *dst_mac, uint32_t data)
{
    if (!dst_mac) {
        return ESP_ERR_INVALID_ARG;
    }

    espnow_frame_head_t frame_head{};
    frame_head.retransmit_count = 5;
    frame_head.broadcast = false;

    esp_now_data power_reply = {
        .type = POWER_MANAGE,
        .seq = seq++,
        .data = data
    };

    return espnow_send(ESPNOW_DATA_TYPE_DATA,
        dst_mac,
        &power_reply,
        sizeof(power_reply),
        &frame_head,
        pdMS_TO_TICKS(200));
}

static esp_err_t init_sensor_stack_once(void)
{
    if (g_sensor_stack_initialized) {
        return ESP_OK;
    }

    g_det_q = xQueueCreate(1, sizeof(detection_data));
    if (!g_det_q) {
        ESP_LOGE(TAG, "queue create failed");
        return ESP_FAIL;
    }

    g_evt = xEventGroupCreate();
    if (!g_evt) {
        ESP_LOGE(TAG, "event group create failed");
        return ESP_FAIL;
    }

    xEventGroupSetBits(g_evt, EVT_MODE_IDLE);

    i2c_sensor_mpu6050_init();

    uint8_t id = mpu.getDeviceID();
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", id);

    mpu_dmp_init_or_die();
    ESP_ERROR_CHECK(mpu_calib_apply_or_calibrate(mpu, 6 /*loops*/, false /*force*/));

    mpu.resetFIFO();
    mpu.setFIFOEnabled(true);
    mpu.setDMPEnabled(true);

    xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 6, &mpu_task_handle);
    vTaskSuspend(mpu_task_handle);

    xTaskCreate(detect_task, "detect_task", 4096, NULL, 5, &detect_task_handle);
    vTaskSuspend(detect_task_handle);

    g_sensor_stack_initialized = true;
    return ESP_OK;
}

static void enter_low_power_mode(void)
{
    if (mpu_task_handle) {
        vTaskSuspend(mpu_task_handle);
    }
    if (detect_task_handle) {
        vTaskSuspend(detect_task_handle);
    }

    if (g_sensor_stack_initialized) {
        mpu.setDMPEnabled(false);
        mpu.setFIFOEnabled(false);
        mpu.setSleepEnabled(true);
    }

    set_sport_mode(0);
    slave_set_powered_on(false);
    slave_clear_power_owner();

    if (master_mac_valid) {
        espnow_del_peer(master_mac);
    }
    slave_clear_pairing_lock();
    memset(master_mac, 0, sizeof(master_mac));
    master_mac_valid = false;

    esp_wifi_set_ps(POWER_OFF_WIFI_PS);
    ESP_LOGI(TAG, "Enter low power mode: only keep ESPNOW wake condition");
}

static esp_err_t power_on_init_from_request(const uint8_t *owner_mac)
{
    if (!owner_mac) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = init_sensor_stack_once();
    if (err != ESP_OK) {
        return err;
    }

    memcpy(master_mac, owner_mac, sizeof(master_mac));
    master_mac_valid = true;

    slave_set_power_owner(owner_mac);
    slave_set_powered_on(true);
    slave_clear_pairing_lock();

    espnow_add_peer(master_mac, NULL);

    mpu.setSleepEnabled(false);
    mpu.resetFIFO();
    mpu.setFIFOEnabled(true);
    mpu.setDMPEnabled(true);
    set_sport_mode(0);

    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "Power on request accepted from %02X:%02X:%02X:%02X:%02X:%02X",
        owner_mac[0], owner_mac[1], owner_mac[2], owner_mac[3], owner_mac[4], owner_mac[5]);

    return ESP_OK;
}



// -------------------- MPU读数据任务 --------------------
static void mpu_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(50); // 20Hz
    uint8_t fifoBuffer[64]; // DMP包42字节，留余量

    // 读一次当前档位（避免写死换算系数）
    uint8_t afs=0, fs=0;
    dump_accel_gyro_cfg(&afs, &fs);
    float acc_lsb_per_g = accel_lsb_per_g_from_afs(afs);
    float gyr_lsb_per_dps = gyro_lsb_per_dps_from_fs(fs);

    // DMP的加速度是原始计数（LSB），这里给你准备两个版本：
    // 1) RAW(寄存器) 的 g 单位（你现有）
    // 2) DMP( FIFO ) 的 world linear accel（推荐做世界系）
    VectorInt16 aa, aaReal, aaWorld;
    int log_cnt = 0;

    while (1) {
        detection_data det = {};

        float ypr[3] = {0};
        bool have_dmp = false;

        if (g_dmp_ready) {
            uint16_t fifoCount = mpu.getFIFOCount();

            if (fifoCount >= 1024) {
                mpu.resetFIFO();
                ESP_LOGW(TAG, "FIFO overflow -> resetFIFO");
            } else if (fifoCount >= g_dmp_packet_size) {
                mpu.getFIFOBytes(fifoBuffer, g_dmp_packet_size);

                // 1) 四元数
                mpu.dmpGetQuaternion(&det.q, fifoBuffer);

                // 2) 重力（传感器坐标系）
                mpu.dmpGetGravity(&det.gravity, &det.q);

                // 3) YPR（注意这里要传 det.gravity）
                mpu.dmpGetYawPitchRoll(ypr, &det.q, &det.gravity);

                // 4) （推荐）用FIFO里的加速度做“去重力 + 世界坐标系”
                //    这样 accel 与 quaternion 同步，结果更靠谱
                mpu.dmpGetAccel(&aa, fifoBuffer);                                // 原始加速度(计数)
                mpu.dmpGetLinearAccel(&aaReal, &aa, &det.gravity);              // 去重力(传感器系)
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &det.q);        // 世界系线加速度(计数)

                have_dmp = true;
            }
        }

        det.have_dmp = have_dmp;

        // -------- 2) RAW：寄存器读 accel/gyro/temp（你现有逻辑保留）--------
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        int16_t tRaw = mpu.getTemperature();
        float tempC = (float)tRaw / 340.0f + 36.53f;

        det.acc_x = -(float)ax / acc_lsb_per_g;
        det.acc_y = -(float)ay / acc_lsb_per_g;
        det.acc_z = (float)az / acc_lsb_per_g;

        det.gyr_x = -(float)gx / gyr_lsb_per_dps;
        det.gyr_y = -(float)gy / gyr_lsb_per_dps;
        det.gyr_z = (float)gz / gyr_lsb_per_dps;

        

        if (have_dmp) {
            det.yaw   = ypr[0] * 180.0f / (float)M_PI;
            det.pitch = -ypr[2] * 180.0f / (float)M_PI;
            det.roll  = -ypr[1] * 180.0f / (float)M_PI;

            // 世界坐标系线加速度（单位换成 g）
            // aaWorld 是“线加速度”，不含重力
            float ax_w_g = -(float)aaWorld.x / acc_lsb_per_g;
            float ay_w_g = -(float)aaWorld.y / acc_lsb_per_g;
            float az_w_g = (float)aaWorld.z / acc_lsb_per_g;

            det.lin_wx = -(float)aaWorld.x / acc_lsb_per_g;
            det.lin_wy = -(float)aaWorld.y / acc_lsb_per_g;
            det.lin_wz = (float)aaWorld.z / acc_lsb_per_g;
            
            // log_cnt++;
            // if(log_cnt >=3){
            //     log_cnt=0;
            
            //     ESP_LOGI(TAG,
            //         "YPR=[%.1f %.1f %.1f] | ACC=[%.2f %.2f %.2f] | "
            //         "ACC_WORLD[g]=[%.2f %.2f %.2f] | gyro=[%.2f %.2f %.2f] T=%.2fC",
            //         det.yaw, det.pitch, det.roll,
            //         det.acc_x, det.acc_y, det.acc_z,
            //         ax_w_g, ay_w_g, az_w_g,
            //         det.gyr_x, det.gyr_y, det.gyr_z,
            //         tempC
            //     );
            // }
        } else {
            // ESP_LOGI(TAG,
            //     "DMP(no packet) | A[g]=[%.2f %.2f %.2f] G[dps]=[%.2f %.2f %.2f] T=%.2fC",
            //     det.acc_x, det.acc_y, det.acc_z,
            //     det.gyr_x, det.gyr_y, det.gyr_z,
            //     tempC
            // );
            det.yaw = det.pitch = det.roll = NAN;
            det.lin_wx = det.lin_wy = det.lin_wz = NAN;
        }

       

        det.tick = (uint32_t)xTaskGetTickCount();
        xQueueOverwrite(g_det_q, &det);

        vTaskDelay(period);
    }
}



//espnow发送测试任务
static void espnow_tx_task(void *arg)
{
    

    const char payload[] = "4565189";

    while (true) {
        

        ESP_LOGI("ESPNOW_TX", "send ok");

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 秒
    }
}



/*----------------------------动作识别任务----------------------------------*/
static void detect_task(void *arg) {
    detection_data det;
    EventBits_t bits;

    int current_mode = 0;   // 0 idle, 1..4
    int step_total = 0;     // ✅ 踏步计数

    while (1) {

        // ① 如果发生模式切换：读取并清掉“切换事件”
        bits = xEventGroupWaitBits(
            g_evt,
            EVT_MODE_CHANGED,
            pdTRUE,     // 自动清掉 EVT_MODE_CHANGED
            pdFALSE,
            0           // 不阻塞
        );

        if (bits & EVT_MODE_CHANGED) {
            EventBits_t modeBits = xEventGroupGetBits(g_evt) & EVT_MODE_MASK;
            int new_mode = 0;

            if      (modeBits & EVT_MODE_SIT_ANKLE) new_mode = 1;
            else if (modeBits & EVT_MODE_SIT_LIFT) new_mode = 2;
            else if (modeBits & EVT_MODE_STEP) new_mode = 3;
            else if (modeBits & EVT_MODE_HIGH) new_mode = 4;
            else                               new_mode = 0;

            if (new_mode != current_mode) {
                current_mode = new_mode;
                switch(current_mode){
                    case 0: ESP_LOGI(TAG,"IDLE MODE");break;
                    case 1: ESP_LOGI(TAG,"SIT_ANKLE MODE");break;
                    case 2: ESP_LOGI(TAG,"SIT_LIFT MODE");break;
                    case 3: ESP_LOGI(TAG,"STEP MODE");break;
                    case 4: ESP_LOGI(TAG,"HIGH MODE");break;
                }
                // 模式切换时重置各动作状态机，避免基线和防抖计数残留。
                if (current_mode == 3 || current_mode == 4) {
                    step_total = 0;
                }
                step_reset();
                sit_reset();

                ESP_LOGI(TAG, "Mode changed -> %d", current_mode);
            }
        }

        // ② 等待一帧最新数据（Queue）
        if (xQueueReceive(g_det_q, &det, portMAX_DELAY) != pdTRUE) continue;
        if (!det.have_dmp) continue;

        // ③ 根据模式选择算法
        switch (current_mode) {
            case 0:
                break;

            case 1:{
                
                bool new_sit_ankle = sit_update(&det,0);
                if (new_sit_ankle) {
                    ESP_LOGI(TAG, "SIT_ANKLE COMPLETE");
                    send_exercise_data(EXERCISE_MODE_SIT_ANKLE);
                }
            } break;

            case 2:{
                bool new_sit_lift = sit_update(&det,1);
                if (new_sit_lift) {
                    ESP_LOGI(TAG, "SIT_LIFT COMPLETE");
                    send_exercise_data(EXERCISE_MODE_SIT_LIFT);
                }
            } break;
               

            case 3: {
                bool new_step = step_update(&det, &step_total,0);
                if (new_step) {
                    ESP_LOGI(TAG, "STEP ++  total=%d  lin_wz=%.2f", step_total, det.lin_wz);
                    send_exercise_data(EXERCISE_MODE_STEP);
                }
            } break;

            case 4: {
                bool new_step = step_update(&det, &step_total,1);
                if (new_step) {
                    ESP_LOGI(TAG, "STEP ++  total=%d  lin_wz=%.2f", step_total, det.lin_wz);
                    send_exercise_data(EXERCISE_MODE_HIGH);
                }
            } break;
        }
    }
}



void slave_main_task(void *arg)
{
    slave_evt_msg_t evt;
    bool heartbeat_waiting_ack = false;
    uint8_t heartbeat_miss_count = 0;
    TickType_t next_heartbeat_tick = 0;
    TickType_t wait_master_confirm_deadline = 0;

    while (1) {
        TickType_t wait_ticks = portMAX_DELAY;
        if (slave_state == SLAVE_WAIT_MAIN_CONFIRM) {
            TickType_t now = xTaskGetTickCount();
            if (wait_master_confirm_deadline == 0) {
                wait_master_confirm_deadline = now + WAIT_MASTER_CONFIRM_TIMEOUT_TICKS;
            }
            wait_ticks = (now >= wait_master_confirm_deadline) ? 0 : (wait_master_confirm_deadline - now);
        } else if (slave_state == SLAVE_READY || slave_state == SLAVE_RUNNING) {
            TickType_t now = xTaskGetTickCount();
            if (next_heartbeat_tick == 0) {
                next_heartbeat_tick = now + HEARTBEAT_INTERVAL_TICKS;
            }
            wait_ticks = (now >= next_heartbeat_tick) ? 0 : (next_heartbeat_tick - now);
        }

        if (xQueueReceive(slave_evt_queue, &evt, wait_ticks)) {

            if (evt.event == EVT_POWER_ON_REQ) {
                if (slave_is_powered_on()) {
                    ESP_LOGI(TAG, "Ignore power-on event because device is already on");
                    continue;
                }

                if (power_on_init_from_request(evt.master_mac) != ESP_OK) {
                    ESP_LOGE(TAG, "Power-on init failed, keep low power mode");
                    continue;
                }

                slave_state = SLAVE_IDLE;
                last_state = SLAVE_IDLE;
                heartbeat_waiting_ack = false;
                heartbeat_miss_count = 0;
                next_heartbeat_tick = 0;
                wait_master_confirm_deadline = 0;
                continue;
            }

            if (!slave_is_powered_on()) {
                ESP_LOGI(TAG, "Ignore event %d while powered off", evt.event);
                continue;
            }

            if (evt.event == EVT_POWER_OFF_REQ) {
                esp_err_t reply_err = send_power_manage_reply(evt.master_mac, 2);
                if (reply_err != ESP_OK) {
                    ESP_LOGW(TAG, "POWER_OFF reply failed: %s", esp_err_to_name(reply_err));
                }

                enter_low_power_mode();
                slave_state = SLAVE_IDLE;
                last_state = SLAVE_IDLE;
                heartbeat_waiting_ack = false;
                heartbeat_miss_count = 0;
                next_heartbeat_tick = 0;
                wait_master_confirm_deadline = 0;
                continue;
            }

            if (evt.event == EVT_HEARTBEAT_ACK) {
                if (master_mac_valid && memcmp(evt.master_mac, master_mac, 6) == 0) {
                    heartbeat_waiting_ack = false;
                    heartbeat_miss_count = 0;
                } else {
                    ESP_LOGW(TAG, "Ignore heartbeat ACK from unknown peer");
                    continue;
                }
            }

            // 1. 状态切换
            last_state = slave_state;

            if (last_state == SLAVE_WAIT_MAIN_CONFIRM && evt.event == EVT_RECEIVE_MASTER_ACK) {
                if (!master_mac_valid || memcmp(evt.master_mac, master_mac, 6) != 0) {
                    ESP_LOGW(TAG, "Ignore master confirm from unknown peer");
                    continue;
                }
            }

            slave_state = slave_state_machine(slave_state, evt.event);

            if ((slave_state == SLAVE_READY || slave_state == SLAVE_RUNNING)
                && !(last_state == SLAVE_READY || last_state == SLAVE_RUNNING)) {
                heartbeat_waiting_ack = false;
                heartbeat_miss_count = 0;
                next_heartbeat_tick = xTaskGetTickCount() + HEARTBEAT_INTERVAL_TICKS;
            }

            if (slave_state == SLAVE_WAIT_MAIN_CONFIRM && last_state != SLAVE_WAIT_MAIN_CONFIRM) {
                wait_master_confirm_deadline = xTaskGetTickCount() + WAIT_MASTER_CONFIRM_TIMEOUT_TICKS;
            } else if (slave_state != SLAVE_WAIT_MAIN_CONFIRM) {
                wait_master_confirm_deadline = 0;
            }

            if (!(slave_state == SLAVE_READY || slave_state == SLAVE_RUNNING)) {
                heartbeat_waiting_ack = false;
                heartbeat_miss_count = 0;
                next_heartbeat_tick = 0;
            }

            //从SLAVE_RUNNING状态离开后，继续暂停传感器和识别任务
            if (last_state == SLAVE_RUNNING && slave_state != SLAVE_RUNNING) {
                if (mpu_task_handle) {
                    vTaskSuspend(mpu_task_handle);
                }
                if (detect_task_handle) {
                    vTaskSuspend(detect_task_handle);
                }
            }

            // 2. 行为（和状态配合）
            switch (slave_state) {

                case SLAVE_IDLE:{
                    // 空状态
                    // 等待 CONNECTION_REQUEST / RESET 等事件
                    if (last_state != SLAVE_IDLE) {
                        if (master_mac_valid) {
                            espnow_del_peer(master_mac);
                        }
                        slave_clear_pairing_lock();
                        memset(master_mac, 0, sizeof(master_mac));
                        master_mac_valid = false;
                        ESP_LOGW(TAG, "Master disconnected, return to prepare-connection state");
                    }
                }break;

                case SLAVE_WAIT_MAIN_CONFIRM:{
                    if (evt.event == EVT_RECEIVE_REQ && last_state == SLAVE_IDLE) {
                        ESP_LOGI(TAG,"Wait Main Confirm");
                        if(!master_mac_valid){
                            memcpy(master_mac, evt.master_mac, 6);
                            master_mac_valid = true;
                            ESP_LOGI(TAG,"Set Master Mac :%d %d: %d %d: %d %d",
                                master_mac[0],master_mac[1],master_mac[2],master_mac[3],master_mac[4],master_mac[5]);
                        }

                        slave_set_pairing_lock(master_mac);

                        espnow_add_peer(master_mac, NULL);
                        espnow_frame_head_t frame_head{};
                        frame_head.retransmit_count = 5;
                        frame_head.broadcast = false;

                        esp_now_data slave_confirm = {
                            .type = CONNECTION_SLAVE_CONFIRM,
                            .seq  = seq++,
                            .data = 0
                        };

                        espnow_send(ESPNOW_DATA_TYPE_DATA,
                            master_mac,
                            &slave_confirm,
                            sizeof(slave_confirm),
                            &frame_head,
                            portMAX_DELAY);
                    } else if (evt.event == EVT_RECEIVE_REQ) {
                        ESP_LOGI(TAG, "Ignore broadcast request while waiting master confirm");
                    }
                }break;

                //Ready状态负责保存mac地址。该状态标志着主从设备连接成功，等待工作
                case SLAVE_READY:{
                    if (last_state != SLAVE_READY) {
                        ESP_LOGI(TAG,"SLAVE_READY");
                    }

                    if (last_state != SLAVE_READY && master_mac_valid) {
                        nvs_save_master_mac(master_mac);
                        espnow_frame_head_t frame_head{};
                        frame_head.retransmit_count = 5;
                        frame_head.broadcast = false;

                        esp_now_data status_confirm = {
                            .type = STATUS_CONFIRM,
                            .seq  = seq++,
                            .data = 666
                        };

                        espnow_send(ESPNOW_DATA_TYPE_DATA,
                            master_mac,
                            &status_confirm,
                            sizeof(status_confirm),
                            &frame_head,
                            portMAX_DELAY);
                    }
                    
                }break;

                case SLAVE_RUNNING:{
                    // 正常工作
                    if(last_state != SLAVE_RUNNING){
                        if (mpu_task_handle) {
                            vTaskResume(mpu_task_handle);
                        }
                        if (detect_task_handle) {
                            vTaskResume(detect_task_handle);
                        }
                        set_sport_mode(evt.data);//根据接收回调内容设置模式
                        
                    }



                }break;

                default:{
                    ESP_LOGW(TAG,
                        "Unhandled slave_state=%d (event=%d)",
                        slave_state,
                        evt.event);
                }break;
            }
        }

        if (slave_state == SLAVE_WAIT_MAIN_CONFIRM && wait_master_confirm_deadline != 0) {
            TickType_t now = xTaskGetTickCount();
            if (now >= wait_master_confirm_deadline) {
                last_state = slave_state;
                slave_state = slave_state_machine(slave_state, EVT_WAIT_MASTER_ACK_TIMEOUT);
                wait_master_confirm_deadline = 0;

                if (master_mac_valid) {
                    espnow_del_peer(master_mac);
                }
                slave_clear_pairing_lock();
                memset(master_mac, 0, sizeof(master_mac));
                master_mac_valid = false;

                ESP_LOGW(TAG, "Wait master confirm timeout, back to SLAVE_IDLE");
                continue;
            }
        }

        if (slave_state == SLAVE_READY || slave_state == SLAVE_RUNNING) {
            TickType_t now = xTaskGetTickCount();
            if (next_heartbeat_tick != 0 && now >= next_heartbeat_tick) {
                if (heartbeat_waiting_ack) {
                    heartbeat_miss_count++;
                    ESP_LOGW(TAG, "Heartbeat ACK timeout, miss_count=%u", heartbeat_miss_count);
                }

                if (heartbeat_miss_count >= HEARTBEAT_MISS_MAX) {
                    last_state = slave_state;
                    slave_state = slave_state_machine(slave_state, EVT_MASTER_LOST);

                    if (last_state == SLAVE_RUNNING && slave_state != SLAVE_RUNNING) {
                        if (mpu_task_handle) {
                            vTaskSuspend(mpu_task_handle);
                        }
                        if (detect_task_handle) {
                            vTaskSuspend(detect_task_handle);
                        }
                    }

                    if (master_mac_valid) {
                        espnow_del_peer(master_mac);
                    }
                    slave_clear_pairing_lock();
                    memset(master_mac, 0, sizeof(master_mac));
                    master_mac_valid = false;

                    heartbeat_waiting_ack = false;
                    heartbeat_miss_count = 0;
                    next_heartbeat_tick = 0;

                    ESP_LOGE(TAG, "Master link lost after %u heartbeat misses", HEARTBEAT_MISS_MAX);
                    continue;
                }

                if (send_keep_alive() == ESP_OK) {
                    heartbeat_waiting_ack = true;
                } else {
                    heartbeat_waiting_ack = true;
                }

                next_heartbeat_tick = now + HEARTBEAT_INTERVAL_TICKS;
            }
        }
    }
}



extern "C" void app_main(void)
{
    //NVS初始化
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    //wifi配置初始化
    app_wifi_init();

    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_init(&espnow_config);

    slave_evt_queue = xQueueCreate(8, sizeof(slave_evt_msg_t));
    if (!slave_evt_queue) {
        ESP_LOGE(TAG, "slave event queue create failed");
        abort();
    }

    slave_set_powered_on(false);
    slave_clear_power_owner();
    slave_state = SLAVE_IDLE;
    last_state = SLAVE_IDLE;
    ESP_ERROR_CHECK(esp_wifi_set_ps(POWER_OFF_WIFI_PS));
    ESP_LOGI(TAG, "Boot in low power mode, waiting POWER_MANAGE(data=1) broadcast");

    vTaskDelay(pdMS_TO_TICKS(1000));
    //set_sport_mode(1);


    xTaskCreate(slave_main_task,"slave_main",4096,NULL,4,NULL);

    
    espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, slave_receive_handle);
}
