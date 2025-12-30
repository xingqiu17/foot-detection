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

// A方案：相对回位的参考零点（进入UP时的z）
static float    g_z_ref0 = 0.0f;     // m：本次步态的相对零点（进入UP时的z）

// pitch 基线（平放基线）
static float g_pitch0 = 0.0f;
static bool  g_pitch0_inited = false;

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

    g_z_ref0 = 0.0f;


}


//坐姿抬腿参数
const SitActionParams SIT_LIFT_PARAMS = {
      // pitch 门禁（相对起始）
    10.0f,     //  UP_PITCH_DELTA_MIN  
    60.0f,     //  HIGH_PITCH_MIN;
    10.0f,     //BACK_PITCH_DELTA_MAX

    // yaw 稳定
    10.0f,     //YAW_DELTA_MAX

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

//坐姿抬腿参数
const SitActionParams SIT_ANKLE_PARAMS = {
      // pitch 门禁（相对起始）
    5.0f,     //  UP_PITCH_DELTA_MIN  
    25.0f,     //  HIGH_PITCH_MIN;
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


    /* ================= 参数 ================= */



    const uint32_t up_timeout        =pdMS_TO_TICKS(SitParams.UP_TIMEOUT_MS);
    const uint32_t idle_timeout = pdMS_TO_TICKS(SitParams.HIGH_IDLE_TIMEOUT_MS);
    const uint32_t down_timeout      = pdMS_TO_TICKS(SitParams.DOWN_TIMEOUT_MS);

    /* ================= 当前帧 ================= */

    const float pitch = det->pitch;
    const float yaw   = det->yaw;
    const float gyr = det->gyr_x;

    /* ================= 基线 ================= */

    static float pitch0 = 0.0f;
    static float yaw0   = 0.0f;
    static bool  base_ok = false;

    if (!base_ok && dev_state == SIT_LIFT_IDLE) {
        pitch0 = pitch;
        yaw0   = yaw;
        base_ok = true;
    }

    const float dpitch = pitch - pitch0;
    const float dyaw   = yaw   - yaw0;

    const float abs_gyr = fabsf(gyr);

    /* ================= 门禁 ================= */

    const bool up_gate =
        (dpitch > SitParams.UP_PITCH_DELTA_MIN) &&
        (abs_gyr > SitParams.GYRO_MOVE_TH) &&
        (fabsf(dyaw) < SitParams.YAW_DELTA_MAX);

    const bool high_idle_gate =
        (dpitch > SitParams.HIGH_PITCH_MIN) &&
        (abs_gyr < SitParams.GYRO_IDLE_TH);

    const bool down_gate =
        (dpitch < SitParams.HIGH_PITCH_MIN - 10.0f) &&
        (abs_gyr > SitParams.GYRO_MOVE_TH);

    const bool idle_gate =
        (fabsf(dpitch) < SitParams.BACK_PITCH_DELTA_MAX) &&
        (abs_gyr < SitParams.GYRO_IDLE_TH);

    /* ================= 状态机 ================= */

    switch (dev_state) {

    case SIT_LIFT_IDLE: {
        if(!g_state_enter_flag){
            g_state_enter_tick = det->tick;
            g_state_enter_flag = true;
        }
        if(g_up_cnt){
            if (det->tick - g_state_enter_tick > up_timeout) {
                ESP_LOGW("SIT_LIFT", "TIMEOUT  UP -> HIGH_IDLE");
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
        if (det->tick - g_state_enter_tick > idle_timeout) {
            ESP_LOGW("SIT_LIFT", "TIMEOUT  UP -> HIGH_IDLE");
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


//踏步检测
bool step_update(const detection_data* det, int* step_total)
{
    if (!det || !det->have_dmp) return false;
    if (isnan(det->lin_wz)) return false;

    //         ===== 阈值=====
    const float up_th    = +0.02f;    // g
    const float down_th  = -0.02f;    // g
    const float land_th  = -0.10f;    // g
    const float near0_th =  0.005f;   // g
    const uint32_t min_interval = pdMS_TO_TICKS(250);

    // 防抖连续次数
    const int need_up_cnt    = 2;
    const int need_down_cnt  = 2;
    const int need_land_cnt  = 1;
    const int need_near0_cnt = 2;

    // ===== 即时响应（20Hz下约9帧）=====
    const uint32_t UP_TIMEOUT_MS   = 450;
    const uint32_t DOWN_TIMEOUT_MS = 450;
    const uint32_t up_timeout   = pdMS_TO_TICKS(UP_TIMEOUT_MS);
    const uint32_t down_timeout = pdMS_TO_TICKS(DOWN_TIMEOUT_MS);

    // ===== 门禁=====
    const float VZ_UP_TH   = 0.03f;   // m/s
    const float VZ_DOWN_TH = 0.03f;   // m/s（向下用 -VZ_DOWN_TH）

    const float Z_MIN_LIFT = 0.012f;  // m
    const float Z_BACK_TH  = 0.010f;  // m

    const float PITCH_FLAT_DEG = 12.0f;

    // ===== 当前帧数据 =====
    const float az_g = det->lin_wz;   // g
    float pitch = det->pitch;
    if (isnan(pitch)) pitch = 0.0f;

    if (!g_pitch0_inited) {
        g_pitch0 = pitch;
        g_pitch0_inited = true;
        ESP_LOGI(TAG_STEP, "pitch baseline init: pitch0=%.2f deg", g_pitch0);
    }

    const float dpitch = pitch - g_pitch0;
    const bool foot_flat = (fabsf(dpitch) <= PITCH_FLAT_DEG);

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

    // 漂移抑制：near0时把 vz/z 拉回（短窗口够用）
    if (g_idle_cnt >= need_near0_cnt) {
        g_vz *= 0.85f;
        g_z  *= 0.85f;
        if (fabsf(g_vz) < 0.02f) g_vz = 0.0f;
        if (fabsf(g_z)  < 0.002f) g_z = 0.0f;
    } else {
        g_vz *= 0.99f;
        g_z  *= 0.99f;
    }

    // A方案：相对位移（以进入UP时的z为零点）
    const float z_rel = g_z - g_z_ref0;

    // ===== 组合门禁（关键修复点：连续计数只对 gate 生效）=====
    const bool up_gate   = (az_g > up_th)   && (g_vz >  VZ_UP_TH)   && foot_flat;
    const bool down_gate = (az_g < down_th) && (g_vz < -VZ_DOWN_TH);

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
                "IDLE: up_cnt=%d/%d | gate=1 az=%.4f>%.4f vz=%.3f>%.2f dpitch=%.2f flat=%d",
                g_up_cnt, need_up_cnt, az_g, up_th, g_vz, VZ_UP_TH, dpitch, (int)foot_flat);

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
                    "IDLE: up_cnt reset | gate=0 az=%.4f vz=%.3f dpitch=%.2f flat=%d",
                    az_g, g_vz, dpitch, (int)foot_flat);
            }
            g_up_cnt = 0;
        }
    } break;

    case STEP_UP: {
        uint32_t dur = det->tick - g_state_enter_tick;

        if (az_g > g_peak_up_az) g_peak_up_az = az_g;

        // ✅A方案：max_z 记录相对高度（而不是绝对z）
        if (z_rel > g_max_z) g_max_z = z_rel;

        // UP 超时：没形成有效抬脚就回IDLE
        if (dur >= up_timeout) {
            ESP_LOGW(TAG_STEP,
                "UP TIMEOUT -> IDLE (NO COUNT) | dur~%ums max_z_rel=%.3f(need>=%.3f) vz=%.3f az=%.4f",
                UP_TIMEOUT_MS, g_max_z, Z_MIN_LIFT, g_vz, az_g);

            dev_state = STEP_IDLE;
            g_state_enter_tick = 0;
            g_down_cnt = 0;
            g_land_cnt = 0;
            g_saw_landed = false;
            break;
        }

        // ✅ 修复：down_cnt 只对 down_gate 连续计数，否则清零
        if (down_gate) {
            g_down_cnt++;
            ESP_LOGI(TAG_STEP,
                "UP: down_cnt=%d/%d | gate=1 az=%.4f<%.4f vz=%.3f<-%.2f max_z_rel=%.3f",
                g_down_cnt, need_down_cnt, az_g, down_th, g_vz, VZ_DOWN_TH, g_max_z);

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

        // 软落地：vz接近0 + z回到接近0 + near0稳定 + 平放
        // ✅A方案：回位用 z_rel
        const bool back_pos = (fabsf(z_rel) < Z_BACK_TH);
        const bool vz_stop  = (fabsf(g_vz) < 0.12f);
        const bool near0_ok = (g_idle_cnt >= need_near0_cnt);

        bool finish_ok = false;
        const char* finish_reason = "";

        if (foot_flat && near0_ok && back_pos && vz_stop) {
            finish_ok = true;
            finish_reason = "SOFT_FINISH(vz~0+z_rel~0+near0+flat)";
        }
        // 硬落地也需要 near0+flat 才算完成
        if (g_saw_landed && foot_flat && near0_ok) {
            finish_ok = true;
            finish_reason = "HARD_LAND+near0+flat";
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
            "DOWN: gates | near0_ok=%d(%d/%d) back_pos=%d(|z_rel|=%.4f < %.4f) vz_stop=%d(|vz|=%.4f < 0.12) flat=%d(dpitch=%.2f <= %.1f) saw_landed=%d | finish_ok=%d(%s)",
            (int)near0_ok, g_idle_cnt, need_near0_cnt,
            (int)back_pos, fabsf(z_rel), Z_BACK_TH,
            (int)vz_stop, fabsf(g_vz),
            (int)foot_flat, dpitch, PITCH_FLAT_DEG,
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
                g_up_cnt = g_down_cnt = g_land_cnt = 0;
                g_saw_landed = false;
                return true;
            } else {
                ESP_LOGW(TAG_STEP,
                    "COUNT FAILED | too fast dt=%u < min=%u | %s",
                    (unsigned)dt, (unsigned)min_interval, finish_reason);

                dev_state = STEP_IDLE;
                g_state_enter_tick = 0;
                g_up_cnt = g_down_cnt = g_land_cnt = 0;
                g_saw_landed = false;
                break;
            }
        }

        // near-timeout 诊断（可选：只在快超时前几帧打）
        if (dur + pdMS_TO_TICKS(50) >= down_timeout) {
            ESP_LOGW(TAG_STEP,
                "DOWN: near-timeout DIAG | missing: near0_ok=%d back_pos=%d vz_stop=%d flat=%d hard_ok=%d soft_ok=%d | az=%.4f vz=%.3f z=%.3f z_rel=%.3f near0_cnt=%d saw_landed=%d dpitch=%.2f",
                (int)near0_ok, (int)back_pos, (int)vz_stop, (int)foot_flat,
                (int)(g_saw_landed && foot_flat && near0_ok),
                (int)(foot_flat && near0_ok && back_pos && vz_stop),
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
            g_up_cnt = g_down_cnt = g_land_cnt = 0;
            g_saw_landed = false;
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
