#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

//踏步状态机（保持不变）
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

static int      g_st = STEP_IDLE;
static uint32_t g_state_enter_tick = 0;
static uint32_t g_last_step_tick = 0;

// 防抖计数
static int g_up_cnt = 0;
static int g_down_cnt = 0;
static int g_near0_cnt = 0;
static int g_land_cnt = 0;

// 观测量
static bool  g_saw_landed = false;
static float g_peak_up_az = 0.0f;
static float g_peak_down_az = 0.0f;

// 速度/位移（世界系z）
static float    g_vz = 0.0f;         // m/s
static float    g_z  = 0.0f;         // m
static float    g_max_z = 0.0f;      // m（UP阶段抬脚最大高度估计）
static uint32_t g_last_tick = 0;

// pitch 基线（平放基线）
static float g_pitch0 = 0.0f;
static bool  g_pitch0_inited = false;

static inline const char* st_name(int s){
    return (s==STEP_IDLE)?"IDLE":(s==STEP_UP)?"UP":"DOWN";
}

void step_reset(void)
{
    g_st = STEP_IDLE;
    g_state_enter_tick = 0;
    g_last_step_tick = 0;

    g_up_cnt = g_down_cnt = g_near0_cnt = g_land_cnt = 0;
    g_saw_landed = false;
    g_peak_up_az = 0.0f;
    g_peak_down_az = 0.0f;

    g_vz = 0.0f;
    g_z  = 0.0f;
    g_max_z = 0.0f;
    g_last_tick = 0;

    // pitch 基线不强制清，你也可以清掉：
    // g_pitch0_inited = false;
}

void step_set_pitch_baseline(float pitch_deg)
{
    g_pitch0 = pitch_deg;
    g_pitch0_inited = true;
    ESP_LOGI(TAG_STEP, "pitch baseline set: pitch0=%.2f deg", g_pitch0);
}

bool step_update(const detection_data* det, int* step_total)
{
    if (!det || !det->have_dmp) return false;
    if (isnan(det->lin_wz)) return false;

    // ===== 你现有阈值（完全不动）=====
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

    // ===== 门禁（不是你那4个阈值）=====
    const float VZ_UP_TH   = 0.03f;   // m/s
    const float VZ_DOWN_TH = 0.03f;   // m/s（向下用 -VZ_DOWN_TH）

    const float Z_MIN_LIFT = 0.012f;  // m
    const float Z_BACK_TH  = 0.006f;  // m

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
    if (fabsf(az_g) < near0_th) g_near0_cnt++;
    else g_near0_cnt = 0;

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
    if (g_near0_cnt >= need_near0_cnt) {
        g_vz *= 0.85f;
        g_z  *= 0.85f;
        if (fabsf(g_vz) < 0.02f) g_vz = 0.0f;
        if (fabsf(g_z)  < 0.002f) g_z = 0.0f;
    } else {
        g_vz *= 0.99f;
        g_z  *= 0.99f;
    }

    // ===== 组合门禁（关键修复点：连续计数只对 gate 生效）=====
    const bool up_gate   = (az_g > up_th)   && (g_vz >  VZ_UP_TH)   && foot_flat;
    const bool down_gate = (az_g < down_th) && (g_vz < -VZ_DOWN_TH);

    switch (g_st) {

    case STEP_IDLE: {
        // IDLE等待：清理一些状态
        g_saw_landed = false;
        g_down_cnt = 0;
        g_land_cnt = 0;
        g_peak_up_az = 0.0f;
        g_peak_down_az = 0.0f;
        g_max_z = 0.0f;

        // ✅ 修复：up_cnt 只对 up_gate 连续计数，否则清零
        if (up_gate) {
            g_up_cnt++;
            ESP_LOGI(TAG_STEP,
                "IDLE: up_cnt=%d/%d | gate=1 az=%.4f>%.4f vz=%.3f>%.2f dpitch=%.2f flat=%d",
                g_up_cnt, need_up_cnt, az_g, up_th, g_vz, VZ_UP_TH, dpitch, (int)foot_flat);

            if (g_up_cnt >= need_up_cnt) {
                g_st = STEP_UP;
                g_state_enter_tick = det->tick;
                g_up_cnt = 0;

                g_peak_up_az = az_g;
                g_max_z = 0.0f;

                // 进入UP时，把 DOWN 相关计数清掉
                g_down_cnt = 0;
                g_land_cnt = 0;
                g_saw_landed = false;

                ESP_LOGI(TAG_STEP,
                    "STATE CHANGE: IDLE -> UP | reason: up_gate consec | az=%.4f vz=%.3f z=%.3f",
                    az_g, g_vz, g_z);
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
        if (g_z > g_max_z) g_max_z = g_z;

        // UP 超时：没形成有效抬脚就回IDLE
        if (dur >= up_timeout) {
            ESP_LOGW(TAG_STEP,
                "UP TIMEOUT -> IDLE (NO COUNT) | dur~%ums max_z=%.3f(need>=%.3f) vz=%.3f az=%.4f",
                UP_TIMEOUT_MS, g_max_z, Z_MIN_LIFT, g_vz, az_g);

            g_st = STEP_IDLE;
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
                "UP: down_cnt=%d/%d | gate=1 az=%.4f<%.4f vz=%.3f<-%.2f max_z=%.3f",
                g_down_cnt, need_down_cnt, az_g, down_th, g_vz, VZ_DOWN_TH, g_max_z);

            if (g_down_cnt >= need_down_cnt) {

                // 还要保证抬脚高度足够（剔除脚尖脚跟抖动）
                if (g_max_z >= Z_MIN_LIFT) {
                    g_st = STEP_DOWN;
                    g_state_enter_tick = det->tick;

                    g_down_cnt = 0;
                    g_peak_down_az = az_g;

                    g_land_cnt = 0;
                    g_saw_landed = false;

                    ESP_LOGI(TAG_STEP,
                        "STATE CHANGE: UP -> DOWN | reason: down_gate consec + max_z>=%.3f | vz=%.3f max_z=%.3f",
                        Z_MIN_LIFT, g_vz, g_max_z);
                } else {
                    ESP_LOGW(TAG_STEP,
                        "UP: reject DOWN | reason: lift too small max_z=%.3f < %.3f",
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
                    "DOWN: HARD LANDED | az=%.4f<%.4f vz=%.3f z=%.3f",
                    az_g, land_th, g_vz, g_z);
            }
        } else {
            g_land_cnt = 0;
        }

        // 软落地：vz接近0 + z回到接近0 + near0稳定 + 平放
        const bool back_pos = (fabsf(g_z) < Z_BACK_TH);
        const bool vz_stop  = (fabsf(g_vz) < 0.12f);
        const bool near0_ok = (g_near0_cnt >= need_near0_cnt);

        bool finish_ok = false;
        const char* finish_reason = "";

        if (foot_flat && near0_ok && back_pos && vz_stop) {
            finish_ok = true;
            finish_reason = "SOFT_FINISH(vz~0+z~0+near0+flat)";
        }
        // 硬落地也需要 near0+flat 才算完成
        if (g_saw_landed && foot_flat && near0_ok) {
            finish_ok = true;
            finish_reason = "HARD_LAND+near0+flat";
        }

        if (finish_ok) {
            uint32_t dt = det->tick - g_last_step_tick;
            if (dt >= min_interval) {
                g_last_step_tick = det->tick;
                if (step_total) (*step_total)++;

                ESP_LOGI(TAG_STEP,
                    "COUNT SUCCESS | %s | dt=%u(min=%u) az=%.4f vz=%.3f z=%.3f max_z=%.3f dpitch=%.2f peak_up=%.4f peak_down=%.4f",
                    finish_reason, (unsigned)dt, (unsigned)min_interval,
                    az_g, g_vz, g_z, g_max_z, dpitch, g_peak_up_az, g_peak_down_az);

                g_st = STEP_IDLE;
                g_state_enter_tick = 0;
                g_up_cnt = g_down_cnt = g_land_cnt = 0;
                g_saw_landed = false;
                return true;
            } else {
                ESP_LOGW(TAG_STEP,
                    "COUNT FAILED | too fast dt=%u < min=%u | %s",
                    (unsigned)dt, (unsigned)min_interval, finish_reason);

                g_st = STEP_IDLE;
                g_state_enter_tick = 0;
                g_up_cnt = g_down_cnt = g_land_cnt = 0;
                g_saw_landed = false;
                break;
            }
        }

        // DOWN 超时：不计步回IDLE（避免卡死）
        if (dur >= down_timeout) {
            ESP_LOGW(TAG_STEP,
                "DOWN TIMEOUT -> IDLE (NO COUNT) | dur~%ums | az=%.4f vz=%.3f z=%.3f max_z=%.3f near0_cnt=%d dpitch=%.2f flat=%d saw_landed=%d",
                DOWN_TIMEOUT_MS,
                az_g, g_vz, g_z, g_max_z, g_near0_cnt, dpitch, (int)foot_flat, (int)g_saw_landed);

            g_st = STEP_IDLE;
            g_state_enter_tick = 0;
            g_up_cnt = g_down_cnt = g_land_cnt = 0;
            g_saw_landed = false;
            break;
        }
    } break;

    default:
        g_st = STEP_IDLE;
        g_state_enter_tick = 0;
        break;
    }

    return false;
}
