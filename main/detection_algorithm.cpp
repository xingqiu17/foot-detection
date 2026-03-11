#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


//抬腿状态机
#define SIT_LIFT_IDLE        0
#define SIT_LIFT_UP          1
#define SIT_LIFT_HIGH_IDLE   2
#define SIT_LIFT_DOWN        3

//踏步状态机
#define STEP_IDLE 0
#define STEP_UP   1
#define STEP_DOWN 2

#include "detection_algorithm.h"

static const char* TAG_STEP = "step";

/*
 * det->lin_wz 单位：g（加速度），不是速度！
 * 本算法把 lin_wz(g) -> a(m/s^2) 积分得到 vz(m/s) 与 z(m)（短窗口内可用）
 * 用 vz/z 做“方向门禁/位移门禁”，避免“上升减速必负加速度 -> 误进DOWN”的致命问题
 */

//状态
static int   dev_state = 0;

static bool g_state_enter_flag = false;
static bool g_state_loop_flag = false;
static uint32_t g_state_enter_tick = 0;
static uint32_t g_last_step_tick = 0;
static uint32_t g_state_loop_tick = 0;

// 防抖计数
static int g_up_cnt = 0;
static int g_down_cnt = 0;
static int g_idle_cnt = 0;
static int g_land_cnt = 0;

// 观测量
static bool  g_saw_landed = false;
static float g_peak_up_az = 0.0f;
static float g_peak_down_az = 0.0f;

// 速度/位移（世界系z）
static float    g_vz = 0.0f;         // m/s
static float    g_z  = 0.0f;         // m
static float    g_max_z = 0.0f;      // m（UP阶段抬脚最大高度估计：这里会改成相对高度）
static uint32_t g_last_tick = 0;
static float    g_az_bias = 0.0f;    // g（lin_wz 零偏，静止时在线估计）
static bool     g_az_bias_inited = false;

// A方案：相对回位的参考零点（进入UP时的z）
static float    g_z_ref0 = 0.0f;     // m：本次步态的相对零点（进入UP时的z）

// pitch 基线（平放基线）
static float g_pitch0 = 0.0f;
static bool  g_pitch0_inited = false;
static bool  g_pitch_reacq_active = false;
static uint32_t g_pitch_reacq_start_tick = 0;
static float g_pitch_candidate = 0.0f;
static int   g_pitch_candidate_cnt = 0;
static float g_pitch_last_good = 0.0f;
static int   g_pitch_idle_stable_cnt = 0;
static uint32_t g_step_rearm_until_tick = 0;
static float g_idle_az_ref = 0.0f;
static float g_idle_vz_ref = 0.0f;
static bool  g_idle_ref_inited = false;
static int   g_idle_ref_stable_cnt = 0;

static inline void step_enter_idle_rearm(uint32_t tick, const char* reason)
{
    // 回到 IDLE 时清掉积分残差，避免下一步识别依赖“先停顿几百毫秒”。
    g_vz = 0.0f;
    g_z = 0.0f;
    g_z_ref0 = 0.0f;
    g_max_z = 0.0f;
    g_idle_cnt = 0;
    g_up_cnt = 0;
    g_down_cnt = 0;
    g_land_cnt = 0;
    g_saw_landed = false;
    g_step_rearm_until_tick = tick + pdMS_TO_TICKS(180);
    ESP_LOGI(TAG_STEP, "IDLE REARM | reason=%s", reason ? reason : "unknown");
}

// sit 基线（用于 sit_reset）
static float g_sit_pitch0 = 0.0f;
static float g_sit_roll0  = 0.0f;
static float g_sit_yaw0   = 0.0f;
static bool  g_sit_base_ok = false;

static inline const char* st_name(int s){
    return (s==STEP_IDLE)?"IDLE":(s==STEP_UP)?"UP":"DOWN";
}

void step_reset(void)
{
    dev_state = STEP_IDLE;
    g_state_enter_tick = 0;
    g_last_step_tick = 0;

    g_up_cnt = g_down_cnt = g_idle_cnt = g_land_cnt = 0;
    g_saw_landed = false;
    g_peak_up_az = 0.0f;
    g_peak_down_az = 0.0f;

    g_vz = 0.0f;
    g_z  = 0.0f;
    g_max_z = 0.0f;
    g_last_tick = 0;
    g_az_bias = 0.0f;
    g_az_bias_inited = false;

    g_z_ref0 = 0.0f;

    g_pitch0 = 0.0f;
    g_pitch0_inited = false;
    g_pitch_reacq_active = false;
    g_pitch_reacq_start_tick = 0;
    g_pitch_candidate = 0.0f;
    g_pitch_candidate_cnt = 0;
    g_pitch_last_good = 0.0f;
    g_pitch_idle_stable_cnt = 0;
    g_step_rearm_until_tick = 0;
    g_idle_az_ref = 0.0f;
    g_idle_vz_ref = 0.0f;
    g_idle_ref_inited = false;
    g_idle_ref_stable_cnt = 0;


}


//坐姿抬腿参数
const SitActionParams SIT_LIFT_PARAMS = {
      // pitch 门禁（相对起始）
    5.0f,     //  UP_PITCH_DELTA_MIN  
    30.0f,     //  HIGH_PITCH_MIN;
    10.0f,     //BACK_PITCH_DELTA_MAX

    // yaw 稳定
    15.0f,     //YAW_DELTA_MAX

    // 角速度（deg/s）
    10.0f,     //GYRO_MOVE_TH
    10.0f,     //GYRO_IDLE_TH

    // 防抖
    5,     //need_up_cnt
    5,     //need_idle_cnt
    5,     //need_down_cnt

    // 超时
    1200,     //UP_TIMEOUT_MS
    4000,     //HIGH_IDLE_TIMEOUT_MS
    1200     //DOWN_TIMEOUT_MS
};

//坐姿绷脚参数
const SitActionParams SIT_ANKLE_PARAMS = {
      // pitch 门禁（相对起始）
    5.0f,     //  UP_PITCH_DELTA_MIN  
    15.0f,     //  HIGH_PITCH_MIN;
    10.0f,     //BACK_PITCH_DELTA_MAX

    // yaw 稳定
    20.0f,     //YAW_DELTA_MAX

    // 角速度（deg/s）
    20.0f,     //GYRO_MOVE_TH
    10.0f,     //GYRO_IDLE_TH

    // 防抖
    5,     //need_up_cnt
    5,     //need_idle_cnt
    5,     //need_down_cnt

    // 超时
    1200,     //UP_TIMEOUT_MS
    4000,     //HIGH_IDLE_TIMEOUT_MS
    1200     //DOWN_TIMEOUT_MS
};


bool sit_update(const detection_data* det,uint8_t sport_flag)
{
    if (!det || !det->have_dmp) return false;

    SitActionParams SitParams = {};

    //参数选择
    if(!sport_flag){SitParams = SIT_ANKLE_PARAMS;}
    else{SitParams = SIT_LIFT_PARAMS;}

    const uint32_t up_timeout        =pdMS_TO_TICKS(SitParams.UP_TIMEOUT_MS);
    const uint32_t idle_timeout = pdMS_TO_TICKS(SitParams.HIGH_IDLE_TIMEOUT_MS);
    const uint32_t down_timeout      = pdMS_TO_TICKS(SitParams.DOWN_TIMEOUT_MS);

    /* ================= 当前帧 ================= */

    const float pitch = det->pitch;
    const float roll  = det->roll;
    const float yaw   = det->yaw;
    const float gyr_x = det->gyr_x;
    const float gyr_y = det->gyr_y;

    /* ================= 基线 ================= */

    if (!g_sit_base_ok) {
        g_sit_pitch0 = pitch;
        g_sit_roll0  = roll;
        g_sit_yaw0   = yaw;
        g_sit_base_ok = true;
    } else if (dev_state == SIT_LIFT_IDLE && fabsf(gyr_x) < SitParams.GYRO_IDLE_TH && fabsf(gyr_y) < SitParams.GYRO_IDLE_TH) {
        g_sit_pitch0 = 0.90f * g_sit_pitch0 + 0.10f * pitch;
        g_sit_roll0  = 0.90f * g_sit_roll0  + 0.10f * roll;
        g_sit_yaw0   = 0.90f * g_sit_yaw0   + 0.10f * yaw;
    }

    const float dpitch = pitch - g_sit_pitch0;
    const float droll  = roll  - g_sit_roll0;
    const float dyaw   = yaw   - g_sit_yaw0;

    const float abs_gyr_x = fabsf(gyr_x);
    const float abs_gyr_y = fabsf(gyr_y);

    // 踢腿（sport_flag=1）更容易出现轴耦合：左/右脚安装时，pitch 与 roll 可能互换或符号翻转。
    const float angle_for_up = sport_flag ? fmaxf(fabsf(dpitch), fabsf(droll)) : dpitch;
    const float angle_for_hold = sport_flag ? fmaxf(fabsf(dpitch), fabsf(droll)) : dpitch;
    const float angle_for_idle = sport_flag ? fmaxf(fabsf(dpitch), fabsf(droll)) : fabsf(dpitch);
    const float gyr_for_move = sport_flag ? fmaxf(abs_gyr_x, abs_gyr_y) : abs_gyr_x;
    const float gyr_for_idle = sport_flag ? fmaxf(abs_gyr_x, abs_gyr_y) : abs_gyr_x;

    /* ================= 门禁 ================= */

    const bool up_gate =
        (angle_for_up > SitParams.UP_PITCH_DELTA_MIN) &&
        (gyr_for_move > SitParams.GYRO_MOVE_TH) &&
        (fabsf(dyaw) < SitParams.YAW_DELTA_MAX);

    const bool high_idle_gate =
        (angle_for_hold > SitParams.HIGH_PITCH_MIN) &&
        (gyr_for_idle < SitParams.GYRO_IDLE_TH);

    const float down_pitch_hys = sport_flag ? 10.0f : 5.0f;
    const float down_gyr_th = sport_flag ? SitParams.GYRO_MOVE_TH : (SitParams.GYRO_MOVE_TH * 0.7f);

    const bool down_gate =
        (angle_for_hold < SitParams.HIGH_PITCH_MIN - down_pitch_hys) &&
        (gyr_for_move > down_gyr_th);

    const bool idle_gate =
        (angle_for_idle < SitParams.BACK_PITCH_DELTA_MAX) &&
        (gyr_for_idle < SitParams.GYRO_IDLE_TH);

    /* ================= 状态机 ================= */

    switch (dev_state) {

    case SIT_LIFT_IDLE: {
        if(!g_state_enter_flag){
            g_state_enter_tick = det->tick;
            g_state_enter_flag = true;
        }
        if(g_up_cnt){
            if(g_up_cnt==1){g_state_enter_tick = det->tick;}
            if (det->tick - g_state_enter_tick > up_timeout) {
                ESP_LOGW("SIT_LIFT", "TIMEOUT  IDLE -> UP");
                dev_state = SIT_LIFT_IDLE;
                g_up_cnt =0;
                g_state_enter_flag = false;
                break;
            }
        }
        
        if (up_gate) {
            
            ESP_LOGI("SIT_LIFT", "IDLE -> UP: %d/%d",g_up_cnt,SitParams.need_up_cnt);
            if (++g_up_cnt >= SitParams.need_up_cnt) {  
                g_state_enter_flag = false;
                dev_state = SIT_LIFT_UP;
                g_up_cnt =0;    
                ESP_LOGI("SIT_LIFT", "STATE CHANGE:IDLE -> UP");
            }
        }
    } break;

    case SIT_LIFT_UP: {
        if(!g_state_enter_flag){
                g_state_enter_tick = det->tick;
                g_state_enter_flag = true;
        }
        if (det->tick - g_state_enter_tick > up_timeout) {
            ESP_LOGW("SIT_LIFT", "TIMEOUT UP -> HIGH_IDLE");
            dev_state = SIT_LIFT_IDLE;
            g_idle_cnt =0;
            g_state_enter_flag = false;
            break;
        }
        if (high_idle_gate) {   
            ESP_LOGI("SIT_LIFT", "UP -> HIGH_IDLE: %d/%d",g_idle_cnt,SitParams.need_idle_cnt);
            if (++g_idle_cnt >= SitParams.need_idle_cnt) {
                g_state_enter_flag = false;
                dev_state = SIT_LIFT_HIGH_IDLE;
                g_idle_cnt = 0;
                ESP_LOGI("SIT_LIFT", "STATE CHANGE:UP -> HIGH_IDLE");
            }
        } else {
            g_idle_cnt = 0;
        }
    } break;

    case SIT_LIFT_HIGH_IDLE: {
        if(!g_state_enter_flag){
                g_state_enter_tick = det->tick;
                g_state_enter_flag = true;
        }
        if (det->tick - g_state_enter_tick > idle_timeout) {
            ESP_LOGW("SIT_LIFT", "TIMEOUT HIGH_IDLE -> DOWN");
            dev_state = SIT_LIFT_IDLE;
            g_down_cnt = 0;
            g_state_enter_flag = false;
            break;
        }
        

        if (down_gate) {
            
            ESP_LOGI("SIT_LIFT", "HIGH_IDLE -> DOWN: %d/%d",g_down_cnt,SitParams.need_down_cnt);
            if (++g_down_cnt >= SitParams.need_down_cnt) {
                dev_state = SIT_LIFT_DOWN;
                g_state_enter_flag = false;
                g_down_cnt = 0;
                ESP_LOGI("SIT_LIFT", "STATE CHANGE:HIGH_IDLE -> DOWN");
            }
        } else {
            g_down_cnt = 0;
        }
    } break;

    case SIT_LIFT_DOWN: {
        if(!g_state_enter_flag){
            g_state_enter_tick = det->tick;
            g_state_enter_flag = true;
        }
        if (det->tick - g_state_enter_tick > down_timeout) {
            ESP_LOGW("SIT_LIFT", "TIMEOUT DOWN -> IDLE");
            dev_state = SIT_LIFT_IDLE;
            g_idle_cnt = 0;
            g_state_enter_flag = false;
            break;
        }
        
        if (idle_gate) {
            ESP_LOGI("SIT_LIFT", "DOWN -> IDLE: %d/%d",g_idle_cnt,SitParams.need_idle_cnt);
            if (++g_idle_cnt >= SitParams.need_idle_cnt) {
                dev_state = SIT_LIFT_IDLE;
                g_idle_cnt = 0;
                g_state_enter_flag = false;
                return true;
            }
        } else {
            g_idle_cnt = 0;
        }
    } break;

    default:
        dev_state = SIT_LIFT_IDLE;
        break;
    }

    return false;
}

void sit_reset(void)
{
    dev_state = SIT_LIFT_IDLE;
    g_state_enter_flag = false;
    g_state_loop_flag = false;
    g_state_enter_tick = 0;
    g_state_loop_tick = 0;

    g_up_cnt = 0;
    g_down_cnt = 0;
    g_idle_cnt = 0;

    g_sit_pitch0 = 0.0f;
    g_sit_roll0  = 0.0f;
    g_sit_yaw0   = 0.0f;
    g_sit_base_ok = false;
}

//站姿踏步参数
const StandActionParams STAND_STEP_PARAMS = {
    //加速度门禁（决定是否动作开始）
  0.02f,     //up_th  g
  0.02f,     //down_th  g
  -0.10f,    //land_th  g
  0.005f,    //near0_th g

  // 防抖连续次数
  2, //need_up_cnt
  2, //need_down_cnt
  1, //need_land_cnt
  2, //need_near0_cnt

  // ===== 即时响应（20Hz下约9帧）=====
  650, //UP_TIMEOUT_MS
  650, //DOWN_TIMEOUT_MS

  // ===== 线速度门禁（判断方向和速度）=====
  0.03f,   // VZ_UP_TH   m/s
  0.03f,   // VZ_DOWN_TH   m/s（向下用 -VZ_DOWN_TH）

  0.006f,  //  Z_MIN_LIFT m
  0.010f,  //  Z_BACK_TH  m

  12.0f,  //PITCH_FLAT_DEG
};

//站姿高抬腿参数
const StandActionParams STAND_HIGH_PARAMS = {
    //加速度门禁（决定是否动作开始）
  0.02f,     //up_th  g
  0.02f,     //down_th  g
  -0.10f,    //land_th  g
  0.005f,    //near0_th g

  // 防抖连续次数
  2, //need_up_cnt
  2, //need_down_cnt
  1, //need_land_cnt
  2, //need_near0_cnt

  // ===== 即时响应（20Hz下约9帧）=====
  450, //UP_TIMEOUT_MS
  450, //DOWN_TIMEOUT_MS

  // ===== 线速度门禁（判断方向和速度）=====
  0.03f,   // VZ_UP_TH   m/s
  0.03f,   // VZ_DOWN_TH   m/s（向下用 -VZ_DOWN_TH）

  0.012f,  //  Z_MIN_LIFT m
  0.010f,  //  Z_BACK_TH  m

  12.0f,  //PITCH_FLAT_DEG
};



//踏步检测
bool step_update(const detection_data* det, int* step_total,uint8_t sport_flag)
{
    if (!det || !det->have_dmp) return false;
    if (isnan(det->lin_wz)) return false;

    StandActionParams StandParams = {};
     //参数选择
    if(!sport_flag){StandParams = STAND_STEP_PARAMS;}
    else{StandParams = STAND_HIGH_PARAMS;}

    //         ===== 阈值=====
    const float up_th    = +StandParams.up_th;      // g
    const float down_th  = -StandParams.down_th;    // g
    const float land_th  = StandParams.land_th;     // g
    const float near0_th = StandParams.near0_th;    // g
    const uint32_t min_interval = pdMS_TO_TICKS(250);

    // 防抖连续次数
    const int need_up_cnt    = StandParams.need_up_cnt;
    const int need_down_cnt  = StandParams.need_down_cnt;
    const int need_land_cnt  = StandParams.need_land_cnt;
    const int need_near0_cnt = StandParams.need_near0_cnt;

    // ===== 即时响应（20Hz下约9帧）=====
    const uint32_t UP_TIMEOUT_MS   = StandParams.UP_TIMEOUT_MS;
    const uint32_t DOWN_TIMEOUT_MS = StandParams.DOWN_TIMEOUT_MS;
    const uint32_t up_timeout   = pdMS_TO_TICKS(UP_TIMEOUT_MS);
    const uint32_t down_timeout = pdMS_TO_TICKS(DOWN_TIMEOUT_MS);

    // ===== 门禁=====
    const float VZ_UP_TH   = StandParams.VZ_UP_TH;      // m/s
    const float VZ_DOWN_TH = StandParams.VZ_DOWN_TH;    // m/s（向下用 -VZ_DOWN_TH）

    const float Z_MIN_LIFT = StandParams.Z_MIN_LIFT;    // m
    const float Z_BACK_TH  = StandParams.Z_BACK_TH;     // m

    const float PITCH_FLAT_DEG = StandParams.PITCH_FLAT_DEG;

    // ===== 当前帧数据 =====
    const float az_raw_g = det->lin_wz;   // g
    float pitch = det->pitch;
    if (isnan(pitch)) pitch = 0.0f;

    if (!g_pitch0_inited) {
        g_pitch0 = pitch;
        g_pitch0_inited = true;
        ESP_LOGI(TAG_STEP, "pitch baseline init: pitch0=%.2f deg", g_pitch0);
    }

    const float dpitch = pitch - g_pitch0;
    const bool foot_flat = (fabsf(dpitch) <= PITCH_FLAT_DEG);

    // 仅在平放且角速度较小阶段追踪零偏，避免把真实抬脚加速度学进去。
    const bool bias_track_ok = foot_flat && (fabsf(det->gyr_x) < 35.0f) && (fabsf(det->gyr_y) < 35.0f);
    if (!g_az_bias_inited) {
        g_az_bias = az_raw_g;
        g_az_bias_inited = true;
    } else if (dev_state == STEP_IDLE && bias_track_ok) {
        const float alpha = (fabsf(az_raw_g - g_az_bias) < 0.05f) ? 0.03f : 0.01f;
        g_az_bias += alpha * (az_raw_g - g_az_bias);
    }

    float az_g = az_raw_g - g_az_bias;
    if (fabsf(az_g) < 0.006f) az_g = 0.0f;

    // 基线策略：仅在“连续静稳的IDLE窗口”里重建，避免漏检时把动作姿态学进基线。
    const bool still_for_baseline =
        (dev_state == STEP_IDLE) &&
        (fabsf(det->gyr_x) < 10.0f) &&
        (fabsf(det->gyr_y) < 10.0f) &&
        (fabsf(det->gyr_z) < 15.0f) &&
        (fabsf(az_g) < (near0_th * 1.6f)) &&
        (fabsf(g_vz) < 0.08f) &&
        foot_flat;

    if (still_for_baseline) g_pitch_idle_stable_cnt++;
    else g_pitch_idle_stable_cnt = 0;

    if (dev_state == STEP_IDLE && g_pitch0_inited && !g_pitch_reacq_active && g_pitch_idle_stable_cnt >= 4) {
        g_pitch_reacq_active = true;
        g_pitch_reacq_start_tick = det->tick;
        g_pitch_candidate = pitch;
        g_pitch_candidate_cnt = 1;
        g_pitch_last_good = g_pitch0;
        // ESP_LOGI(TAG_STEP,
        //     "BASELINE REACQ START | stable_cnt=%d pitch0_old=%.2f",
        //     g_pitch_idle_stable_cnt, g_pitch_last_good);
    }

    if (dev_state == STEP_IDLE && g_pitch0_inited && g_pitch_reacq_active) {
        if (still_for_baseline) {
            if (g_pitch_candidate_cnt == 0) g_pitch_candidate = pitch;
            else g_pitch_candidate = 0.75f * g_pitch_candidate + 0.25f * pitch;
            g_pitch_candidate_cnt++;

            if (g_pitch_candidate_cnt >= 3) {
                const float base_delta = fabsf(g_pitch_candidate - g_pitch_last_good);
                if (base_delta <= 8.0f) {
                    g_pitch0 = g_pitch_candidate;
                    ESP_LOGI(TAG_STEP,
                        "BASELINE REACQ OK | pitch0=%.2f(old=%.2f) delta=%.2f cnt=%d",
                        g_pitch0, g_pitch_last_good, base_delta, g_pitch_candidate_cnt);
                } else {
                    // ESP_LOGW(TAG_STEP,
                    //     "BASELINE REACQ REJECT | candidate=%.2f old=%.2f delta=%.2f > 8.0 (keep old)",
                    //     g_pitch_candidate, g_pitch_last_good, base_delta);
                }
                g_pitch_reacq_active = false;
            }
        } else {
            g_pitch_candidate_cnt = 0;
        }

        if (g_pitch_reacq_active && (det->tick - g_pitch_reacq_start_tick >= pdMS_TO_TICKS(400))) {
            g_pitch0 = g_pitch_last_good;
            g_pitch_reacq_active = false;
            ESP_LOGW(TAG_STEP,
                "BASELINE REACQ TIMEOUT | keep old pitch0=%.2f window=400ms",
                g_pitch0);
        }
    }

    // 非重建窗口下做轻微慢跟踪，处理长时间温漂/绑带微位移。
    if (dev_state == STEP_IDLE && g_pitch0_inited && !g_pitch_reacq_active && still_for_baseline) {
        if (fabsf(pitch - g_pitch0) < 6.0f) {
            g_pitch0 = 0.992f * g_pitch0 + 0.008f * pitch;
        }
    }

    // ===== near0 计数（用于收尾、漂移抑制）=====
    if (fabsf(az_g) < near0_th) g_idle_cnt++;
    else g_idle_cnt = 0;

    // ===== 速度/位移积分 =====
    const float G0 = 9.80665f; // m/s^2 per g
    if (g_last_tick == 0) g_last_tick = det->tick;
    float dt_s = (float)(det->tick - g_last_tick) * (portTICK_PERIOD_MS / 1000.0f);
    g_last_tick = det->tick;
    if (dt_s < 0.0001f || dt_s > 0.20f) dt_s = 0.05f;

    float a_ms2 = az_g * G0;
    g_vz += a_ms2 * dt_s;
    g_z  += g_vz * dt_s;

    // rearm窗口内加强收敛，快速把上一动作残余速度/位移清零。
    if (dev_state == STEP_IDLE && det->tick < g_step_rearm_until_tick) {
        g_vz *= 0.55f;
        g_z  *= 0.60f;
        if (fabsf(g_vz) < 0.03f) g_vz = 0.0f;
        if (fabsf(g_z)  < 0.004f) g_z = 0.0f;
    }

    // 漂移抑制：near0时把 vz/z 拉回（短窗口够用）
    if (g_idle_cnt >= need_near0_cnt) {
        g_vz *= 0.70f;
        g_z  *= 0.80f;
        if (fabsf(g_vz) < 0.02f) g_vz = 0.0f;
        if (fabsf(g_z)  < 0.002f) g_z = 0.0f;
    } else {
        g_vz *= 0.99f;
        g_z  *= 0.99f;
    }

    // IDLE 参考值：用当前静止/近平放段的 az 与 vz 作为“零动态基线”，避免静止偏置误触发。
    const bool idle_ref_track_ok =
        (dev_state == STEP_IDLE) &&
        foot_flat &&
        (fabsf(det->gyr_x) < 25.0f) &&
        (fabsf(det->gyr_y) < 25.0f) &&
        (fabsf(det->gyr_z) < 35.0f) &&
        (g_up_cnt == 0);

    if (idle_ref_track_ok) {
        if (!g_idle_ref_inited) {
            g_idle_az_ref = az_g;
            g_idle_vz_ref = g_vz;
            g_idle_ref_inited = true;
            g_idle_ref_stable_cnt = 1;
        } else {
            const float ref_alpha = (g_idle_ref_stable_cnt < 4) ? 0.20f : 0.05f;
            g_idle_az_ref += ref_alpha * (az_g - g_idle_az_ref);
            g_idle_vz_ref += ref_alpha * (g_vz - g_idle_vz_ref);
            if (g_idle_ref_stable_cnt < 20) g_idle_ref_stable_cnt++;
        }
    }

    const float az_up = az_g - g_idle_az_ref;
    const float vz_up = g_vz - g_idle_vz_ref;

    // A方案：相对位移（以进入UP时的z为零点）
    const float z_rel = g_z - g_z_ref0;

    // ===== 组合门禁（关键修复点：连续计数只对 gate 生效）=====
    const bool up_gate =
        g_idle_ref_inited &&
        (g_idle_ref_stable_cnt >= 2) &&
        (az_up > up_th) &&
        (vz_up > VZ_UP_TH) &&
        foot_flat;

    switch (dev_state) {

    case STEP_IDLE: {
        // IDLE等待：清理一些状态
        g_saw_landed = false;
        g_down_cnt = 0;
        g_land_cnt = 0;
        g_peak_up_az = 0.0f;
        g_peak_down_az = 0.0f;
        g_max_z = 0.0f;

        if (up_gate) {
            g_up_cnt++;
            ESP_LOGI(TAG_STEP,
                "IDLE: up_cnt=%d/%d | gate=1 az=%.4f(ref=%.4f,dyn=%.4f raw=%.4f,bias=%.4f)>%.4f vz=%.3f(ref=%.3f,dyn=%.3f)>%.2f dpitch=%.2f flat=%d",
                g_up_cnt, need_up_cnt,
                az_g, g_idle_az_ref, az_up, az_raw_g, g_az_bias, up_th,
                g_vz, g_idle_vz_ref, vz_up, VZ_UP_TH,
                dpitch, (int)foot_flat);

            if (g_up_cnt >= need_up_cnt) {
                dev_state = STEP_UP;
                g_state_enter_tick = det->tick;
                g_up_cnt = 0;

                g_peak_up_az = az_g;
                g_max_z = 0.0f;

                // ✅A方案：进入UP时记录相对零点
                g_z_ref0 = g_z;

                // 进入UP时，把 DOWN 相关计数清掉
                g_down_cnt = 0;
                g_land_cnt = 0;
                g_saw_landed = false;

                ESP_LOGI(TAG_STEP,
                    "STATE CHANGE: IDLE -> UP | reason: up_gate consec | az=%.4f vz=%.3f z=%.3f(z_ref0=%.3f)",
                    az_g, g_vz, g_z, g_z_ref0);
            }
        } else {
            if (g_up_cnt) {
                ESP_LOGD(TAG_STEP,
                    "IDLE: up_cnt reset | gate=0 az_dyn=%.4f vz_dyn=%.3f az=%.4f vz=%.3f dpitch=%.2f flat=%d",
                    az_up, vz_up, az_g, g_vz, dpitch, (int)foot_flat);
            }
            g_up_cnt = 0;
        }
    } break;

    case STEP_UP: {
        uint32_t dur = det->tick - g_state_enter_tick;

        if (az_g > g_peak_up_az) g_peak_up_az = az_g;

        // A方案：max_z 记录相对高度（而不是绝对z）
        if (z_rel > g_max_z) g_max_z = z_rel;

        // UP 超时：没形成有效抬脚就回IDLE
        if (dur >= up_timeout) {
            ESP_LOGW(TAG_STEP,
                "UP TIMEOUT -> IDLE (NO COUNT) | dur~%ums max_z_rel=%.3f(need>=%.3f) vz=%.3f az=%.4f",
                UP_TIMEOUT_MS, g_max_z, Z_MIN_LIFT, g_vz, az_g);

            dev_state = STEP_IDLE;
            g_state_enter_tick = 0;
            step_enter_idle_rearm(det->tick, "UP_TIMEOUT");
            break;
        }

        // UP->DOWN 门禁：保留硬门禁，同时增加“软门禁”避免过严导致超时。
        const bool down_gate_hard = (az_g < down_th) && (g_vz < -VZ_DOWN_TH);
        const bool down_gate_soft =
            (g_max_z >= (Z_MIN_LIFT * 0.85f)) &&
            (g_vz < -(VZ_DOWN_TH * 0.55f)) &&
            (az_g < (up_th * 0.5f));
        const bool down_gate = down_gate_hard || down_gate_soft;

        // down_cnt 只对 down_gate 连续计数，否则清零
        if (down_gate) {
            g_down_cnt++;
            ESP_LOGI(TAG_STEP,
                "UP: down_cnt=%d/%d | gate=1(hard=%d soft=%d) az=%.4f<%.4f vz=%.3f<-%.2f max_z_rel=%.3f",
                g_down_cnt, need_down_cnt, (int)down_gate_hard, (int)down_gate_soft,
                az_g, down_th, g_vz, VZ_DOWN_TH, g_max_z);

            if (g_down_cnt >= need_down_cnt) {

                // 还要保证抬脚高度足够（剔除脚尖脚跟抖动）
                if (g_max_z >= Z_MIN_LIFT) {
                    dev_state = STEP_DOWN;
                    g_state_enter_tick = det->tick;

                    g_down_cnt = 0;
                    g_peak_down_az = az_g;

                    g_land_cnt = 0;
                    g_saw_landed = false;

                    ESP_LOGI(TAG_STEP,
                        "STATE CHANGE: UP -> DOWN | reason: down_gate consec + max_z_rel>=%.3f | vz=%.3f max_z_rel=%.3f",
                        Z_MIN_LIFT, g_vz, g_max_z);
                } else {
                    ESP_LOGW(TAG_STEP,
                        "UP: reject DOWN | reason: lift too small max_z_rel=%.3f < %.3f",
                        g_max_z, Z_MIN_LIFT);
                    // ✅ 关键：reject 时清零，避免后面“一帧达标就秒切”
                    g_down_cnt = 0;
                }
            }
        } else {
            if (g_down_cnt) {
                ESP_LOGD(TAG_STEP,
                    "UP: down_cnt reset | gate=0 az=%.4f vz=%.3f",
                    az_g, g_vz);
            }
            g_down_cnt = 0;
        }

    } break;

    case STEP_DOWN: {
        uint32_t dur = det->tick - g_state_enter_tick;
        if (az_g < g_peak_down_az) g_peak_down_az = az_g;

        // 硬落地冲击（可用但不作为唯一）
        if (az_g < land_th) {
            g_land_cnt++;
            if (g_land_cnt >= need_land_cnt && !g_saw_landed) {
                g_saw_landed = true;
                ESP_LOGI(TAG_STEP,
                    "DOWN: HARD LANDED | az=%.4f<%.4f vz=%.3f z=%.3f z_rel=%.3f(z_ref0=%.3f)",
                    az_g, land_th, g_vz, g_z, z_rel, g_z_ref0);
            }
        } else {
            g_land_cnt = 0;
        }

        // 落地后若加速度已接近静止，快速抑制积分残差，避免最后阶段被 vz/z 漂移卡死。
        if (g_saw_landed && (fabsf(az_g) < (near0_th * 2.5f))) {
            g_vz *= 0.82f;
            g_z  *= 0.96f;
        }

        // 软落地：优先严格判定；临近超时时允许轻微放宽，避免积分残差导致漏计。
        const bool back_pos = (fabsf(z_rel) < Z_BACK_TH);
        const float z_back_soft_th = fmaxf(Z_BACK_TH * 1.40f, g_max_z * 1.25f + 0.002f);
        const bool back_pos_soft = (fabsf(z_rel) < z_back_soft_th);
        // 抗积分漂移：当 near-timeout 且已明显静稳时，按本步抬高量放宽回位阈值。
        const float z_back_drift_th = fmaxf(z_back_soft_th, g_max_z * 2.20f + 0.002f);
        const bool back_pos_drift = (fabsf(z_rel) < z_back_drift_th);
        const float z_back_timeout_th = fmaxf(z_back_drift_th, 0.060f);
        const bool back_pos_timeout = (fabsf(z_rel) < z_back_timeout_th);

        const bool vz_stop  = (fabsf(g_vz) < 0.15f);
        const bool vz_stop_landed = (fabsf(g_vz) < 0.30f);
        const bool foot_flat_relaxed = (fabsf(dpitch) <= (PITCH_FLAT_DEG + 3.0f));

        const bool near0_ok = (g_idle_cnt >= need_near0_cnt);
        const bool near0_by_cnt_soft =
            ((need_near0_cnt > 1) && (g_idle_cnt >= (need_near0_cnt - 1)) && (fabsf(az_g) < (near0_th * 1.8f)));
        const bool near0_by_abs_soft = (fabsf(az_g) <= (near0_th * 2.4f));
        const bool near0_ok_soft = near0_ok || near0_by_cnt_soft || near0_by_abs_soft;

        const bool near_timeout = (dur + pdMS_TO_TICKS(80) >= down_timeout);

        bool finish_ok = false;
        const char* finish_reason = "";

        if (foot_flat && near0_ok && back_pos && vz_stop) {
            finish_ok = true;
            finish_reason = "SOFT_FINISH(vz~0+z_rel~0+near0+flat)";
        }
        if (!finish_ok && near_timeout && foot_flat && near0_ok_soft && back_pos_soft && vz_stop) {
            finish_ok = true;
            finish_reason = "SOFT_FINISH_RELAXED(near-timeout)";
        }
        if (!finish_ok && near_timeout && foot_flat && near0_ok && back_pos_drift && vz_stop) {
            finish_ok = true;
            finish_reason = "SOFT_FINISH_ANTIDRIFT(near-timeout)";
        }
        if (!finish_ok && near_timeout && foot_flat && near0_by_abs_soft && back_pos_timeout && vz_stop) {
            finish_ok = true;
            finish_reason = "SOFT_FINISH_TIMEOUT_CAP(near-timeout)";
        }
        // 硬落地也需要 near0+flat 才算完成
        if (!finish_ok && g_saw_landed && foot_flat && near0_ok_soft) {
            finish_ok = true;
            finish_reason = "HARD_LAND+near0_soft+flat";
        }
        // near-timeout 且已识别落地时，允许更宽松的 flat/vz 收尾，避免最后一帧漏计。
        if (!finish_ok && near_timeout && g_saw_landed && near0_ok && back_pos_timeout && foot_flat_relaxed && vz_stop_landed) {
            finish_ok = true;
            finish_reason = "HARD_LAND_TIMEOUT_RELAXED";
        }

        // ===== 你之前加的诊断日志：保持风格，补齐 z_rel 信息 =====
        ESP_LOGI(TAG_STEP,
            "DOWN: frame | dur=%u/%u ticks | az=%.4f g vz=%.3f m/s z=%.3f m z_rel=%.3f (z_ref0=%.3f) | near0_cnt=%d (need %d) | land_cnt=%d (need %d) saw_landed=%d | dpitch=%.2f flat=%d | peak_down=%.4f",
            (unsigned)dur, (unsigned)down_timeout,
            az_g, g_vz, g_z, z_rel, g_z_ref0,
            g_idle_cnt, need_near0_cnt,
            g_land_cnt, need_land_cnt, (int)g_saw_landed,
            dpitch, (int)foot_flat, g_peak_down_az);

        ESP_LOGI(TAG_STEP,
            "DOWN: gates | near0_ok=%d near0_soft=%d(cnt=%d abs=%d, %d/%d) back_pos=%d back_soft=%d back_drift=%d back_timeout=%d(|z_rel|=%.4f < %.4f/%.4f/%.4f/%.4f) vz_stop=%d landed_vz=%d(|vz|=%.4f < 0.15/0.30) flat=%d flat_relaxed=%d(dpitch=%.2f <= %.1f/%.1f) near_timeout=%d saw_landed=%d | finish_ok=%d(%s)",
            (int)near0_ok, (int)near0_ok_soft, (int)near0_by_cnt_soft, (int)near0_by_abs_soft, g_idle_cnt, need_near0_cnt,
            (int)back_pos, (int)back_pos_soft, (int)back_pos_drift, (int)back_pos_timeout, fabsf(z_rel), Z_BACK_TH, z_back_soft_th, z_back_drift_th, z_back_timeout_th,
            (int)vz_stop, (int)vz_stop_landed, fabsf(g_vz),
            (int)foot_flat, (int)foot_flat_relaxed, dpitch, PITCH_FLAT_DEG, (PITCH_FLAT_DEG + 3.0f),
            (int)near_timeout,
            (int)g_saw_landed,
            (int)finish_ok, finish_reason[0]?finish_reason:"NONE");

        if (finish_ok) {
            uint32_t dt = det->tick - g_last_step_tick;
            if (dt >= min_interval) {
                g_last_step_tick = det->tick;
                if (step_total) (*step_total)++;

                ESP_LOGI(TAG_STEP,
                    "COUNT SUCCESS | %s | dt=%u(min=%u) az=%.4f vz=%.3f z=%.3f z_rel=%.3f max_z_rel=%.3f dpitch=%.2f peak_up=%.4f peak_down=%.4f",
                    finish_reason, (unsigned)dt, (unsigned)min_interval,
                    az_g, g_vz, g_z, z_rel, g_max_z, dpitch, g_peak_up_az, g_peak_down_az);

                dev_state = STEP_IDLE;
                g_state_enter_tick = 0;
                step_enter_idle_rearm(det->tick, "COUNT_SUCCESS");
                return true;
            } else {
                ESP_LOGW(TAG_STEP,
                    "COUNT FAILED | too fast dt=%u < min=%u | %s",
                    (unsigned)dt, (unsigned)min_interval, finish_reason);

                dev_state = STEP_IDLE;
                g_state_enter_tick = 0;
                step_enter_idle_rearm(det->tick, "COUNT_TOO_FAST");
                break;
            }
        }

        // near-timeout 诊断（可选：只在快超时前几帧打）
        if (dur + pdMS_TO_TICKS(50) >= down_timeout) {
            ESP_LOGW(TAG_STEP,
                "DOWN: near-timeout DIAG | strict: near0=%d back=%d | soft: near0=%d(cnt=%d abs=%d) back=%d drift_back=%d timeout_back=%d | vz_stop=%d landed_vz=%d flat=%d flat_relaxed=%d hard_ok=%d soft_ok=%d drift_ok=%d timeout_ok=%d land_timeout_ok=%d | az=%.4f vz=%.3f z=%.3f z_rel=%.3f near0_cnt=%d saw_landed=%d dpitch=%.2f",
                (int)near0_ok, (int)back_pos,
                (int)near0_ok_soft, (int)near0_by_cnt_soft, (int)near0_by_abs_soft, (int)back_pos_soft, (int)back_pos_drift, (int)back_pos_timeout,
                (int)vz_stop, (int)vz_stop_landed, (int)foot_flat, (int)foot_flat_relaxed,
                (int)(g_saw_landed && foot_flat && near0_ok_soft),
                (int)(foot_flat && near0_ok_soft && back_pos_soft && vz_stop),
                (int)(foot_flat && near0_ok && back_pos_drift && vz_stop),
                (int)(foot_flat && near0_by_abs_soft && back_pos_timeout && vz_stop),
                (int)(g_saw_landed && near0_ok && back_pos_timeout && foot_flat_relaxed && vz_stop_landed),
                az_g, g_vz, g_z, z_rel, g_idle_cnt, (int)g_saw_landed, dpitch);
        }

        // DOWN 超时：不计步回IDLE（避免卡死）
        if (dur >= down_timeout) {
            ESP_LOGW(TAG_STEP,
                "DOWN TIMEOUT -> IDLE (NO COUNT) | dur~%ums | az=%.4f vz=%.3f z=%.3f z_rel=%.3f z_ref0=%.3f max_z_rel=%.3f near0_cnt=%d dpitch=%.2f flat=%d saw_landed=%d",
                DOWN_TIMEOUT_MS,
                az_g, g_vz, g_z, z_rel, g_z_ref0, g_max_z,
                g_idle_cnt, dpitch, (int)foot_flat, (int)g_saw_landed);

            dev_state = STEP_IDLE;
            g_state_enter_tick = 0;
            step_enter_idle_rearm(det->tick, "DOWN_TIMEOUT");
            break;
        }
    } break;

    default:
        dev_state = STEP_IDLE;
        g_state_enter_tick = 0;
        break;
    }

    return false;
}
