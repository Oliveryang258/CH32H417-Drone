#ifndef __BSP_HEIGHT_H
#define __BSP_HEIGHT_H

#include <stdint.h>
#include "shared_data.h"

/* === 通用参数 === */
#define RC_STICK_MAX        120.0f
#define RC_STICK_DEADBAND   5
#define STICK_THROTTLE       (g_shared_sensor.rc_pitch)
#define RC_SW_WAIT                   0U
#define RC_SW_HEIGHT_HOLD            1U
#define RC_SW_FLY                    2U
#define PID_DT               0.006667f  /* 1 / 150Hz */
#define THR_TEST_MAX_US     1550U
#define THR_MAX_US          THR_TEST_MAX_US
#define PWM_SAFE_MAX_US     1750U

/* === V307 标签定义 === */
#define V307_TAG_IMG_HEAD    0xBBU
#define V307_TAG_BATT_LOW    0xCCU
#define V307_TAG_OVERCURRENT 0xDDU
#define V307_BATT_HOLD_MS    500U
#define V307_OVERCURRENT_HOLD_MS 500U
#define V307_OVERCURRENT_BUZZ_PERIOD_MS 80U
#define V307_OVERCURRENT_BUZZ_ON_MS     35U

/* === 高度估计/控制常量 === */
#define HEIGHT_TOF_MIN_MM            50U
#define HEIGHT_TOF_MAX_MM            4000U
#define HEIGHT_TOF_I_FREEZE_MS       100U
#define HEIGHT_TOF_TIMEOUT_MS        200U
#define HEIGHT_SENSOR_RECOVERY_MS    500U
#define HEIGHT_SOURCE_DT_MIN_S       0.010f
#define HEIGHT_SOURCE_DT_MAX_S       0.100f
#define HEIGHT_TILT_COS_MIN          0.819152f  /* cos(35°) */
#define HEIGHT_LPF_CUTOFF_HZ         5.0f
#define HEIGHT_VZ_LPF_CUTOFF_HZ      3.0f
#define HEIGHT_JUMP_BASE_M           0.12f
#define HEIGHT_JUMP_MAX_VZ_MPS       1.5f
#define HEIGHT_READY_FRAMES          3U
#define HEIGHT_ENTRY_MIN_M           0.15f
#define HEIGHT_LOW_STICK             (-80)
#define HEIGHT_ENTRY_STICK_MAX       100
#define MANUAL_TAKEOVER_CENTER_MIN   (-100)
#define MANUAL_TAKEOVER_CENTER_MAX   100
#define HEIGHT_ENTRY_BLEND_MS        500U
#define HEIGHT_ENTRY_WAIT_MS         3000U
#define HEIGHT_FALLBACK_BLEND_MS     300U
#define HEIGHT_PI_DT                 (3.0f * PID_DT)
#define HEIGHT_POS_DT                (6.0f * PID_DT)
#define HEIGHT_I_LIMIT_US            10.0f
#define HEIGHT_ENTRY_VZ_MAX_MPS       0.60f
#define HEIGHT_VZ_CMD_ACCEL_MPS2      0.50f

#define HEIGHT_DIAG_NEW_FRAME        0x01U
#define HEIGHT_DIAG_VALID            0x02U
#define HEIGHT_DIAG_TIMEOUT          0x04U
#define HEIGHT_DIAG_TILT_REJECT      0x08U
#define HEIGHT_DIAG_JUMP_REJECT      0x10U
#define HEIGHT_DIAG_ACTIVE           0x20U
#define HEIGHT_DIAG_SAT_HIGH         0x40U
#define HEIGHT_DIAG_SAT_LOW          0x80U
#define HEIGHT_DIAG_ENTRY_REJECTED   0x0200U
#define HEIGHT_DIAG_SENSOR_HOLD      0x0400U
#define HEIGHT_DIAG_ENTRY_PENDING    0x0800U
#define HEIGHT_DIAG_MANUAL_REMAP     0x1000U

#define HEIGHT_GUARD_LOW_MM       250U
#define HEIGHT_GUARD_HIGH_MM      350U
#define HEIGHT_GUARD_SOFTSTOP_MM  500U
#define HEIGHT_GUARD_HOLD_MS      200U
#define HEIGHT_GUARD_TOF_STALE_MS 250U

/* === 高度蜂鸣事件类型 === */
typedef enum
{
    HEIGHT_BUZZ_NONE = 0,
    HEIGHT_BUZZ_REQUEST,
    HEIGHT_BUZZ_ACTIVE,
    HEIGHT_BUZZ_REJECTED,
    HEIGHT_BUZZ_SENSOR_FAIL
} HeightBuzzEvent_t;

/* === 高度模式枚举 === */
typedef enum
{
    HEIGHT_MODE_OFF = 0,
    HEIGHT_MODE_ACTIVE,
    HEIGHT_MODE_SENSOR_HOLD,
    HEIGHT_MODE_DEGRADED
} HeightMode_t;

/* === 高度估计器结构体 === */
typedef struct
{
    uint32_t seen_source_mark;
    uint32_t accepted_source_mark;
    uint32_t candidate_source_mark;
    uint32_t last_seen_local_ms;
    uint32_t last_accepted_local_ms;
    uint16_t raw_mm;
    uint16_t candidate_raw_mm;
    float source_dt_ms;
    float height_comp_m;
    float height_filt_m;
    float last_height_comp_m;
    float last_height_filt_m;
    float vz_raw_mps;
    float vz_filt_mps;
    uint8_t initialized;
    uint8_t valid;
    uint8_t good_frames;
    uint8_t freeze_integrator;
    uint8_t diag_flags;
    uint8_t candidate_state;
    uint8_t candidate_valid;
    uint8_t candidate_ready;
} HeightEstimator_t;

/* === 共享状态（定义于 bsp_height.c） === */
extern HeightEstimator_t s_height_est;
extern HeightMode_t s_height_mode;
extern uint8_t  s_height_cycle;
extern float    s_height_target_m;
extern float    s_height_target_vz_mps;
extern float    s_height_correction_us;
extern float    s_height_hover_base_us;
extern uint8_t  s_height_entry_rejected;
extern uint8_t  s_height_entry_pending;
extern uint32_t s_height_entry_wait_start_ms;
extern uint32_t s_height_transition_start_ms;
extern uint8_t  s_manual_takeover_active;
extern int16_t  s_manual_takeover_stick;
extern float    s_manual_takeover_collective_us;

/* === 可调参数（定义于 main.c） === */
extern volatile uint8_t  g_height_hold_enable;
extern volatile float    g_height_pos_kp;
extern volatile float    g_height_vel_kp;
extern volatile float    g_height_vel_ki;
extern volatile float    g_hover_throttle_us;
extern volatile float    g_height_corr_limit_us;
extern volatile float    g_height_vz_up_max_mps;
extern volatile float    g_height_vz_down_max_mps;
extern volatile float    g_height_stick_rate_mps;
extern volatile uint8_t  g_height_guard_enable;

/* === 从 main.c 共享的变量（非 static） === */
extern volatile uint32_t g_sys_tick;
extern float thr_base;
extern volatile uint8_t  g_test_motor;
extern volatile uint8_t  g_test_ramp_active;
extern volatile float    g_thr_override;
extern uint8_t soft_stop_active;

/* === 高度保护状态（定义于 bsp_height.c） === */
extern float    height_guard_cap_us;
extern uint16_t height_guard_high_ms;
extern uint32_t height_guard_seen_tof_tick;
extern uint32_t height_guard_seen_local_ms;

/* === 工具函数（定义于 main.c，供高度模块使用） === */
float clampf(float v, float lo, float hi);
float stick_norm(int16_t stick);
float wrap_angle_deg(float a);

/* === 高度控制 API === */
uint8_t V307_AlarmPoll(uint32_t now_ms);
uint8_t Height_ReadTofSnapshot(uint32_t *mark,
                               uint16_t *distance_mm,
                               uint8_t *state,
                               uint8_t *valid);
void    HeightEstimator_Update(uint32_t now_ms);
uint8_t Height_SwitchRequest(void);
void    HeightControl_Reset(void);
void    HeightControl_StartDegraded(uint32_t now_ms, uint8_t block_reentry);
void    HeightControl_StartManualTakeover(uint32_t now_ms);
void    HeightControl_StartSensorHold(uint32_t now_ms);
void    HeightControl_ResumeSensorHold(uint32_t now_ms);
float   HeightControl_StickVzCommand(void);
float   HeightControl_SlewVz(float current, float target);
void    HeightControl_PositionLoop(void);
void    HeightControl_VelocityLoop(void);
uint8_t HeightControl_EntryReady(void);
void    HeightControl_EnterActive(uint32_t now_ms);
uint8_t HeightControl_Update(float manual_target_us, uint32_t now_ms,
                              float *collective_target_us);
float   ManualTakeover_Target(int16_t stick, float normal_target_us);
void    HeightControl_ApplyHeadroom(float out_roll_mix, float out_pitch_mix,
                                    float out_yaw_mix, float *collective_us);
uint16_t HeightControl_DiagFlags(void);

#endif /* __BSP_HEIGHT_H */
