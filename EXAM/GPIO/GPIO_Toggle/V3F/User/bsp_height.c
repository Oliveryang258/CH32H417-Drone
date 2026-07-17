#include "bsp_height.h"
#include "shared_data.h"
#include "bsp_pwm.h"
#include "bsp_comunicate.h"
#include "bsp_led_buzz.h"
#include <math.h>

/* ================================================================
 *  高度保护状态（extern 导出，由 main.c 高度保护块访问）
 * ================================================================ */
volatile uint8_t g_height_guard_enable = 0U;
float    height_guard_cap_us = 0.0f;
uint16_t height_guard_high_ms = 0U;
uint32_t height_guard_seen_tof_tick = 0UL;
uint32_t height_guard_seen_local_ms = 0UL;

/* ================================================================
 *  高度蜂鸣器状态
 * ================================================================ */
static volatile uint8_t s_height_buzz_event = HEIGHT_BUZZ_NONE;
static volatile uint8_t s_height_buzz_seq = 0U;
static volatile uint32_t s_height_buzz_start_ms = 0UL;

static void HeightBuzz_Queue(HeightBuzzEvent_t event, uint32_t now_ms)
{
    s_height_buzz_event = (uint8_t)event;
    s_height_buzz_start_ms = now_ms;
    __asm__ volatile("fence rw, rw" ::: "memory");
    s_height_buzz_seq++;
}

static uint8_t HeightBuzz_Pattern(uint32_t now_ms)
{
    static uint8_t last_seq = 0U;
    static uint8_t event = HEIGHT_BUZZ_NONE;
    static uint32_t start_ms = 0UL;
    uint8_t seq = s_height_buzz_seq;
    uint32_t elapsed_ms;

    if (seq != last_seq) {
        __asm__ volatile("fence rw, rw" ::: "memory");
        event = s_height_buzz_event;
        start_ms = s_height_buzz_start_ms;
        last_seq = seq;
    }

    elapsed_ms = now_ms - start_ms;
    switch ((HeightBuzzEvent_t)event) {
        case HEIGHT_BUZZ_REQUEST:
            if (elapsed_ms < 80U) return 1U;
            break;
        case HEIGHT_BUZZ_ACTIVE:
            if (elapsed_ms < 80U ||
                (elapsed_ms >= 160U && elapsed_ms < 240U)) return 1U;
            if (elapsed_ms < 240U) return 0U;
            break;
        case HEIGHT_BUZZ_REJECTED:
            if (elapsed_ms < 450U) return 1U;
            break;
        case HEIGHT_BUZZ_SENSOR_FAIL:
            if ((elapsed_ms < 60U) ||
                (elapsed_ms >= 120U && elapsed_ms < 180U) ||
                (elapsed_ms >= 240U && elapsed_ms < 300U) ||
                (elapsed_ms >= 360U && elapsed_ms < 420U)) return 1U;
            if (elapsed_ms < 420U) return 0U;
            break;
        case HEIGHT_BUZZ_NONE:
        default:
            break;
    }

    event = HEIGHT_BUZZ_NONE;
    return 0U;
}

uint8_t V307_AlarmPoll(uint32_t now_ms)
{
    static enum { LP_IDLE, LP_IMG_X, LP_IMG_Y, LP_IMG_TAIL } s_state = LP_IDLE;
    static uint32_t s_last_batt_ms = 0U;
    static uint32_t s_last_overcurrent_ms = 0U;
    static uint8_t  s_seen_batt   = 0U;
    static uint8_t  s_seen_overcurrent = 0U;
    static uint8_t  s_buzz_on     = 0U;
    uint8_t b, batt_alarm, overcurrent, buzz_on, alarm_flags;

    while (COMM_RxRead(&b)) {
        switch (s_state) {
            case LP_IDLE:
                if      (b == V307_TAG_IMG_HEAD)  { s_state = LP_IMG_X; }
                else if (b == V307_TAG_BATT_LOW)  { s_last_batt_ms = now_ms; s_seen_batt = 1U; }
                else if (b == V307_TAG_OVERCURRENT) { s_last_overcurrent_ms = now_ms; s_seen_overcurrent = 1U; }
                /* 其他标签 (0xAA/0xAB/0x00) 忽略 */
                break;
            case LP_IMG_X:    s_state = LP_IMG_Y;    break;
            case LP_IMG_Y:    s_state = LP_IMG_TAIL; break;
            case LP_IMG_TAIL: s_state = LP_IDLE;     break;
        }
    }

    batt_alarm = (s_seen_batt && (now_ms - s_last_batt_ms) <= V307_BATT_HOLD_MS) ? 1U : 0U;
    overcurrent = (s_seen_overcurrent &&
                   (now_ms - s_last_overcurrent_ms) <= V307_OVERCURRENT_HOLD_MS) ? 1U : 0U;

    if (overcurrent) {
        buzz_on = ((now_ms % V307_OVERCURRENT_BUZZ_PERIOD_MS) < V307_OVERCURRENT_BUZZ_ON_MS) ? 1U : 0U;
    } else if (batt_alarm) {
        buzz_on = 1U;
    } else {
        buzz_on = HeightBuzz_Pattern(now_ms);
    }

    if (buzz_on != s_buzz_on) {
        s_buzz_on = buzz_on;
        BUZZ_Control(buzz_on);
    }

    alarm_flags = 0U;
    if (batt_alarm)   alarm_flags |= SHARED_ALARM_BATT_LOW;
    if (overcurrent)  alarm_flags |= SHARED_ALARM_OVERCURRENT;
    g_shared_sensor.alarm_flags = alarm_flags;

    return alarm_flags;
}

/* ================================================================
 *  Height estimator and control state
 * ================================================================ */
HeightEstimator_t s_height_est = {0};
HeightMode_t s_height_mode = HEIGHT_MODE_OFF;
static uint8_t s_height_request_prev = 0U;
static uint8_t s_height_reentry_block = 0U;
uint8_t s_height_cycle = 0U;
static uint8_t s_height_sat_high = 0U;
static uint8_t s_height_sat_low = 0U;
uint32_t s_height_transition_start_ms = 0UL;
static float s_height_transition_from_us = (float)PWM_MIN_PULSE_US;
float s_height_target_m = 0.0f;
float s_height_target_vz_mps = 0.0f;
static float s_height_vz_error_mps = 0.0f;
static float s_height_p_us = 0.0f;
static float s_height_i_us = 0.0f;
float s_height_correction_us = 0.0f;
float s_height_hover_base_us = 1400.0f;
static float s_height_sensor_hold_us = (float)PWM_MIN_PULSE_US;
uint8_t s_height_entry_rejected = 0U;
uint8_t s_height_entry_pending = 0U;
uint32_t s_height_entry_wait_start_ms = 0UL;
static int16_t s_height_stick_center = 0;
static uint8_t s_height_stick_active_prev = 0U;
static uint8_t s_height_entry_vz_capture_active = 0U;
uint8_t s_manual_takeover_active = 0U;
static uint8_t s_manual_takeover_first_sample = 0U;
int16_t s_manual_takeover_stick = 0;
float s_manual_takeover_collective_us = (float)PWM_MIN_PULSE_US;

/* ================================================================
 *  手动接管辅助函数（static，高度模块内部使用）
 * ================================================================ */
static void ManualTakeover_Reset(void)
{
    s_manual_takeover_active = 0U;
    s_manual_takeover_first_sample = 0U;
    s_manual_takeover_stick = 0;
    s_manual_takeover_collective_us = (float)PWM_MIN_PULSE_US;
}

static void ManualTakeover_Capture(void)
{
    /* 为双向操作预留物理行程。即使飞行员在端点附近退出定高，
     * 第一次采样仍然连续，后续手动刻度通过现有的 slew 缓慢爬向目标端点。 */
    s_manual_takeover_stick = (int16_t)clampf((float)STICK_THROTTLE,
                                              (float)MANUAL_TAKEOVER_CENTER_MIN,
                                              (float)MANUAL_TAKEOVER_CENTER_MAX);
    s_manual_takeover_collective_us = clampf(thr_base,
                                              (float)PWM_MIN_PULSE_US,
                                              (float)THR_MAX_US);
    s_manual_takeover_active = 1U;
    s_manual_takeover_first_sample = 1U;
}

float ManualTakeover_Target(int16_t stick, float normal_target_us)
{
    float stick_f = clampf((float)stick, -RC_STICK_MAX, RC_STICK_MAX);
    float center_f = (float)s_manual_takeover_stick;
    float target_us;

    if (s_manual_takeover_active == 0U) {
        return normal_target_us;
    }

    if (s_manual_takeover_first_sample != 0U) {
        s_manual_takeover_first_sample = 0U;
        return s_manual_takeover_collective_us;
    }

    if (stick_f > center_f + (float)RC_STICK_DEADBAND) {
        float range = RC_STICK_MAX - center_f - (float)RC_STICK_DEADBAND;
        float ratio = (range > 0.5f) ?
            ((stick_f - center_f - (float)RC_STICK_DEADBAND) / range) : 0.0f;
        target_us = s_manual_takeover_collective_us +
            ratio * ((float)THR_MAX_US - s_manual_takeover_collective_us);
    } else if (stick_f < center_f - (float)RC_STICK_DEADBAND) {
        float range = center_f + RC_STICK_MAX - (float)RC_STICK_DEADBAND;
        float ratio = (range > 0.5f) ?
            ((center_f - stick_f - (float)RC_STICK_DEADBAND) / range) : 0.0f;
        target_us = s_manual_takeover_collective_us -
            ratio * (s_manual_takeover_collective_us - (float)PWM_MIN_PULSE_US);
    } else {
        target_us = s_manual_takeover_collective_us;
    }

    return clampf(target_us, (float)PWM_MIN_PULSE_US, (float)THR_MAX_US);
}

/* ================================================================
 *  高度估计器 (Height Estimator)
 * ================================================================
 *
 * 数据流：
 *   TOF 斜距 → 双采样确认去毛刺 → 倾斜补偿(cos_roll*cos_pitch) → 垂直高度
 *   → 一阶 LPF(α 自适应采样间隔) → height_filt_m
 *   → 微分(相邻帧) → vz_raw_mps → LPF → vz_filt_mps
 *
 * 异常拒绝：
 *   - 倾斜角 > 85° → 拒绝（cos < HEIGHT_TILT_COS_MIN）
 *   - 相邻帧跳变 > HEIGHT_JUMP_BASE_M + HEIGHT_JUMP_MAX_VZ_MPS * dt → 拒绝
 *   - 采样间隔 < 10ms 或 > 500ms → 拒绝
 *   - 超时 > HEIGHT_TOF_TIMEOUT_MS → valid=0, 高度环退化为 degraded
 *   - 冻结 > HEIGHT_TOF_I_FREEZE_MS → 暂停高度 I 项积分
 *
 * 输出（供高度控制使用）：
 *   s_height_est.height_filt_m — 低通滤波后垂直高度 (m)
 *   s_height_est.vz_filt_mps   — 低通滤波后垂直速度 (m/s)
 *   s_height_est.valid         — 数据有效标志
 *   s_height_est.freeze_integrator — 冻结高度 I 项
 */
uint8_t Height_ReadTofSnapshot(uint32_t *mark,
                                uint16_t *distance_mm,
                                uint8_t *state,
                                uint8_t *valid)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < 3U; attempt++) {
        uint32_t mark_begin = g_shared_sensor.tof_update_tick;
        uint32_t mark_end;
        uint16_t d;
        uint8_t st;
        uint8_t ok;

        if (mark_begin == 0UL) {
            return 0U;
        }

        __asm__ volatile("fence rw, rw" ::: "memory");
        d = g_shared_sensor.tof_distance_mm;
        st = g_shared_sensor.tof_state;
        ok = g_shared_sensor.tof_valid;
        __asm__ volatile("fence rw, rw" ::: "memory");
        mark_end = g_shared_sensor.tof_update_tick;

        if (mark_begin == mark_end) {
            *mark = mark_begin;
            *distance_mm = d;
            *state = st;
            *valid = ok;
            return 1U;
        }
    }

    return 0U;
}

void HeightEstimator_Update(uint32_t now_ms)
{
    uint32_t source_mark;
    uint16_t distance_mm;
    uint8_t source_state;
    uint8_t source_valid;
    uint8_t snapshot_confirmed = 0U;

    s_height_est.diag_flags &= (uint8_t)~HEIGHT_DIAG_NEW_FRAME;

    if (Height_ReadTofSnapshot(&source_mark, &distance_mm,
                               &source_state, &source_valid) &&
        source_mark != s_height_est.seen_source_mark) {
        /* tof_update_tick 紧挨打包字段，在固定共享 ABI 中并非自然对齐。
         * 要求连续两个 150Hz tick 读取到完全一致的快照才确认消费。 */
        if (s_height_est.candidate_ready != 0U &&
            source_mark == s_height_est.candidate_source_mark &&
            distance_mm == s_height_est.candidate_raw_mm &&
            source_state == s_height_est.candidate_state &&
            source_valid == s_height_est.candidate_valid) {
            snapshot_confirmed = 1U;
            s_height_est.candidate_ready = 0U;
        } else {
            s_height_est.candidate_source_mark = source_mark;
            s_height_est.candidate_raw_mm = distance_mm;
            s_height_est.candidate_state = source_state;
            s_height_est.candidate_valid = source_valid;
            s_height_est.candidate_ready = 1U;
        }
    }

    if (snapshot_confirmed != 0U) {
        float roll_deg = g_shared_sensor.roll;
        float pitch_deg = g_shared_sensor.pitch;
        float roll_r;
        float pitch_r;
        float tilt_cos;
        float raw_m;
        float height_comp_m;
        float dt_s = 0.0f;
        uint8_t sample_ok;

        s_height_est.seen_source_mark = source_mark;
        s_height_est.last_seen_local_ms = now_ms;
        s_height_est.raw_mm = distance_mm;
        s_height_est.diag_flags |= HEIGHT_DIAG_NEW_FRAME;

        sample_ok = (source_valid != 0U &&
                     source_state == 0U &&
                     distance_mm >= HEIGHT_TOF_MIN_MM &&
                     distance_mm <= HEIGHT_TOF_MAX_MM) ? 1U : 0U;

        if (!(roll_deg == roll_deg) || !(pitch_deg == pitch_deg) ||
            fabsf(roll_deg) > 85.0f || fabsf(pitch_deg) > 85.0f) {
            sample_ok = 0U;
            s_height_est.diag_flags |= HEIGHT_DIAG_TILT_REJECT;
        }

        roll_r = roll_deg * 0.017453293f;
        pitch_r = pitch_deg * 0.017453293f;
        tilt_cos = cosf(roll_r) * cosf(pitch_r);
        if (tilt_cos < HEIGHT_TILT_COS_MIN) {
            sample_ok = 0U;
            s_height_est.diag_flags |= HEIGHT_DIAG_TILT_REJECT;
        }

        raw_m = (float)distance_mm * 0.001f;
        height_comp_m = raw_m * tilt_cos;

        if (sample_ok && s_height_est.initialized != 0U) {
            uint32_t source_delta_ms = source_mark - s_height_est.accepted_source_mark;
            dt_s = (float)source_delta_ms * 0.001f;
            s_height_est.source_dt_ms = (float)source_delta_ms;

            if (dt_s < HEIGHT_SOURCE_DT_MIN_S ||
                dt_s > HEIGHT_SOURCE_DT_MAX_S) {
                sample_ok = 0U;
            } else {
                float jump_limit_m = HEIGHT_JUMP_BASE_M + HEIGHT_JUMP_MAX_VZ_MPS * dt_s;
                if (fabsf(height_comp_m - s_height_est.last_height_comp_m) > jump_limit_m) {
                    sample_ok = 0U;
                    s_height_est.diag_flags |= HEIGHT_DIAG_JUMP_REJECT;
                }
            }
        }

        if (sample_ok) {
            if (s_height_est.initialized == 0U) {
                s_height_est.height_comp_m = height_comp_m;
                s_height_est.height_filt_m = height_comp_m;
                s_height_est.last_height_comp_m = height_comp_m;
                s_height_est.last_height_filt_m = height_comp_m;
                s_height_est.vz_raw_mps = 0.0f;
                s_height_est.vz_filt_mps = 0.0f;
                s_height_est.source_dt_ms = 0.0f;
                s_height_est.good_frames = 1U;
                s_height_est.initialized = 1U;
            } else {
                const float height_tau = 1.0f / (6.283185307f * HEIGHT_LPF_CUTOFF_HZ);
                const float vz_tau = 1.0f / (6.283185307f * HEIGHT_VZ_LPF_CUTOFF_HZ);
                float height_alpha = dt_s / (height_tau + dt_s);
                float vz_alpha = dt_s / (vz_tau + dt_s);

                s_height_est.height_comp_m = height_comp_m;
                s_height_est.height_filt_m += height_alpha *
                    (height_comp_m - s_height_est.height_filt_m);
                s_height_est.vz_raw_mps =
                    (s_height_est.height_filt_m - s_height_est.last_height_filt_m) / dt_s;
                s_height_est.vz_filt_mps += vz_alpha *
                    (s_height_est.vz_raw_mps - s_height_est.vz_filt_mps);
                s_height_est.last_height_comp_m = height_comp_m;
                s_height_est.last_height_filt_m = s_height_est.height_filt_m;
                if (s_height_est.good_frames < 255U) {
                    s_height_est.good_frames++;
                }
            }

            s_height_est.accepted_source_mark = source_mark;
            s_height_est.last_accepted_local_ms = now_ms;
            s_height_est.valid = 1U;
            s_height_est.freeze_integrator = 0U;
            s_height_est.diag_flags &= (uint8_t)~(HEIGHT_DIAG_TIMEOUT |
                                                   HEIGHT_DIAG_TILT_REJECT |
                                                   HEIGHT_DIAG_JUMP_REJECT);
        } else {
            s_height_est.good_frames = 0U;
            s_height_est.freeze_integrator = 1U;
        }
    }

    if (s_height_est.initialized != 0U &&
        (now_ms - s_height_est.last_accepted_local_ms) > HEIGHT_TOF_I_FREEZE_MS) {
        s_height_est.freeze_integrator = 1U;
    }

    if (s_height_est.initialized == 0U ||
        (now_ms - s_height_est.last_accepted_local_ms) > HEIGHT_TOF_TIMEOUT_MS) {
        s_height_est.valid = 0U;
        s_height_est.good_frames = 0U;
        s_height_est.freeze_integrator = 1U;
        s_height_est.diag_flags |= HEIGHT_DIAG_TIMEOUT;
        s_height_est.diag_flags &= (uint8_t)~HEIGHT_DIAG_VALID;
        if ((now_ms - s_height_est.last_accepted_local_ms) > HEIGHT_TOF_TIMEOUT_MS) {
            s_height_est.initialized = 0U;
        }
    } else {
        s_height_est.diag_flags |= HEIGHT_DIAG_VALID;
        s_height_est.diag_flags &= (uint8_t)~HEIGHT_DIAG_TIMEOUT;
    }
}

/* ================================================================
 *  Height control
 * ================================================================ */

/* 模式触发沿刻意与油门解耦。解锁期间一直保持高位的开关需要先释放再重新置位，
 * 才能触发定高进入，防止解锁后意外触发。 */
uint8_t Height_SwitchRequest(void)
{
    return (g_height_hold_enable != 0U &&
            g_shared_sensor.rc_link_ok == 1U &&
            g_shared_sensor.rc_sw == RC_SW_HEIGHT_HOLD) ? 1U : 0U;
}

void HeightControl_Reset(void)
{
    uint8_t request = Height_SwitchRequest();

    s_height_mode = HEIGHT_MODE_OFF;
    s_height_request_prev = request;
    s_height_reentry_block = request;
    s_height_cycle = 0U;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
    s_height_transition_start_ms = 0UL;
    s_height_transition_from_us = (float)PWM_MIN_PULSE_US;
    s_height_target_m = s_height_est.height_filt_m;
    s_height_target_vz_mps = 0.0f;
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_hover_base_us = clampf(g_hover_throttle_us,
                                    (float)PWM_MIN_PULSE_US,
                                    (float)THR_MAX_US);
    s_height_sensor_hold_us = (float)PWM_MIN_PULSE_US;
    s_height_entry_rejected = 0U;
    s_height_entry_pending = 0U;
    s_height_entry_wait_start_ms = 0UL;
    s_height_stick_center = 0;
    s_height_stick_active_prev = 0U;
    s_height_entry_vz_capture_active = 0U;
    ManualTakeover_Reset();
}

void HeightControl_StartDegraded(uint32_t now_ms, uint8_t block_reentry)
{
    s_height_mode = HEIGHT_MODE_DEGRADED;
    s_height_transition_start_ms = now_ms;
    s_height_transition_from_us = thr_base;
    s_height_target_vz_mps = 0.0f;
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
    s_height_entry_pending = 0U;
    s_height_stick_active_prev = 0U;
    s_height_entry_vz_capture_active = 0U;
    if (block_reentry != 0U) {
        s_height_reentry_block = 1U;
    }
}

void HeightControl_StartManualTakeover(uint32_t now_ms)
{
    ManualTakeover_Capture();
    HeightControl_StartDegraded(now_ms, 0U);
    /* 重映射后手动目标在此瞬间等于 thr_base，因此无需 blend 延迟，
     * 飞行员可立即获得操控权限。 */
    s_height_transition_start_ms = now_ms - HEIGHT_FALLBACK_BLEND_MS;
}

/* 定高激活期间刻意忽略手控油门。传感器短暂丢失时不悄无声息地切回未知的
 * 手控值，而是保持最后的总距输出，直到正常退回或超时。 */
void HeightControl_StartSensorHold(uint32_t now_ms)
{
    s_height_mode = HEIGHT_MODE_SENSOR_HOLD;
    s_height_reentry_block = 1U;
    s_height_cycle = 0U;
    s_height_transition_start_ms = now_ms;
    s_height_sensor_hold_us = clampf(thr_base,
                                     (float)PWM_MIN_PULSE_US,
                                     (float)THR_MAX_US);
    s_height_target_vz_mps = 0.0f;
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
    HeightBuzz_Queue(HEIGHT_BUZZ_SENSOR_FAIL, now_ms);
}

void HeightControl_ResumeSensorHold(uint32_t now_ms)
{
    float resume_collective = clampf(thr_base,
                                     (float)PWM_MIN_PULSE_US,
                                     (float)THR_MAX_US);

    s_height_mode = HEIGHT_MODE_ACTIVE;
    s_height_reentry_block = 0U;
    s_height_cycle = 5U;
    s_height_transition_start_ms = now_ms;
    s_height_transition_from_us = resume_collective;
    s_height_sensor_hold_us = resume_collective;
    s_height_hover_base_us = resume_collective;
    s_height_target_m = s_height_est.height_filt_m;
    s_height_target_vz_mps = clampf(s_height_est.vz_filt_mps,
                                    -HEIGHT_ENTRY_VZ_MAX_MPS,
                                     HEIGHT_ENTRY_VZ_MAX_MPS);
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
    s_height_entry_rejected = 0U;
    s_height_entry_pending = 0U;
    s_height_stick_center = (int16_t)clampf((float)STICK_THROTTLE,
                                           -RC_STICK_MAX, RC_STICK_MAX);
    s_height_stick_active_prev = 0U;
    s_height_entry_vz_capture_active = 1U;
    HeightBuzz_Queue(HEIGHT_BUZZ_ACTIVE, now_ms);
}

float HeightControl_StickVzCommand(void)
{
    int16_t delta = STICK_THROTTLE - s_height_stick_center;
    float normalized;

    if (delta > -RC_STICK_DEADBAND && delta < RC_STICK_DEADBAND) {
        return 0.0f;
    }

    if (delta > 0) {
        float range = RC_STICK_MAX - (float)s_height_stick_center -
                      (float)RC_STICK_DEADBAND;
        if (range < 1.0f) range = 1.0f;
        normalized = ((float)delta - (float)RC_STICK_DEADBAND) / range;
    } else {
        float range = RC_STICK_MAX + (float)s_height_stick_center -
                      (float)RC_STICK_DEADBAND;
        if (range < 1.0f) range = 1.0f;
        normalized = ((float)delta + (float)RC_STICK_DEADBAND) / range;
    }

    normalized = clampf(normalized, -1.0f, 1.0f);
    return clampf(normalized * g_height_stick_rate_mps,
                  -g_height_vz_down_max_mps,
                   g_height_vz_up_max_mps);
}

float HeightControl_SlewVz(float current, float target)
{
    float max_step = HEIGHT_VZ_CMD_ACCEL_MPS2 * HEIGHT_POS_DT;
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

/*
 * HeightControl_PositionLoop — 高度位置环 @ 25Hz
 * ================================================================
 * 输入：s_height_est.height_filt_m — 当前滤波高度 (m)
 * 步骤：
 *   a) 摇杆偏离死区 → 映射为目标垂直速度 pilot_vz_mps，
 *      锁当前高度为目标高度（松开即悬停）
 *   b) 摇杆居中 + 刚进入 active → bumpless 过渡，捕获当前 Vz 作为起始
 *   c) 摇杆居中 + 稳态 → 高度误差 P 控：
 *      desired_vz = kp_pos * (target_height - current_height)
 *      clip 到 [vz_down_max, vz_up_max]
 *   d) 加速度限幅 (HEIGHT_VZ_CMD_ACCEL_MPS2)，防指令突变
 * 输出：s_height_target_vz_mps — 速度环期望 (m/s)
 */
void HeightControl_PositionLoop(void)
{
    float pilot_vz_mps = HeightControl_StickVzCommand();
    float desired_vz_mps;

    if (fabsf(pilot_vz_mps) > 0.001f) {
        /* 飞行员操作垂直运动时不累积远处的高度目标。
         * 当前高度即为松开摇杆后要锁定的悬停点。 */
        s_height_target_m = s_height_est.height_filt_m;
        s_height_entry_vz_capture_active = 0U;
        s_height_stick_active_prev = 1U;
        desired_vz_mps = pilot_vz_mps;
    } else if (s_height_entry_vz_capture_active != 0U) {
        /* 无扰进入：从实测垂直速度出发，逐步刹车减速到零，
         * 而非瞬间要求零速度，避免油门阶跃。 */
        s_height_target_m = s_height_est.height_filt_m;
        desired_vz_mps = 0.0f;
        if (fabsf(s_height_target_vz_mps) <=
            HEIGHT_VZ_CMD_ACCEL_MPS2 * HEIGHT_POS_DT) {
            s_height_entry_vz_capture_active = 0U;
            s_height_target_m = s_height_est.height_filt_m;
        }
    } else {
        float height_error_m;
        if (s_height_stick_active_prev != 0U) {
            s_height_target_m = s_height_est.height_filt_m;
            s_height_stick_active_prev = 0U;
        }
        height_error_m = s_height_target_m - s_height_est.height_filt_m;
        desired_vz_mps = clampf(g_height_pos_kp * height_error_m,
                                -g_height_vz_down_max_mps,
                                 g_height_vz_up_max_mps);
    }

    s_height_target_vz_mps = HeightControl_SlewVz(s_height_target_vz_mps,
                                                  desired_vz_mps);
}

/*
 * HeightControl_VelocityLoop — 高度速度环 @ 50Hz
 * ================================================================
 * 输入：
 *   s_height_target_vz_mps — 位置环输出的速度期望 (m/s)
 *   s_height_est.vz_filt_mps — 滤波后的垂直速度 (m/s)
 * 步骤：
 *   a) error = target_vz - measured_vz
 *   b) P 项 = kp * error（油门修正 us）
 *   c) I 项 += ki * error * dt，限幅 ±HEIGHT_I_LIMIT_US
 *      - 积分冻结条件：I 饱和(高低限幅)、TOF 冻结期、entry blend 期
 *   d) 总修正 = clampf(P + I, ±corr_limit)
 * 输出：
 *   s_height_correction_us — 油门修正量 (us)，加到 hover_base 上
 */
void HeightControl_VelocityLoop(void)
{
    float i_candidate;
    float output_candidate;
    float limit = clampf(g_height_corr_limit_us, 0.0f, 30.0f);
    uint8_t allow_integrator = (s_height_est.freeze_integrator == 0U) ? 1U : 0U;

    s_height_vz_error_mps = s_height_target_vz_mps - s_height_est.vz_filt_mps;
    s_height_p_us = g_height_vel_kp * s_height_vz_error_mps;
    i_candidate = clampf(s_height_i_us +
                         g_height_vel_ki * s_height_vz_error_mps * HEIGHT_PI_DT,
                         -HEIGHT_I_LIMIT_US, HEIGHT_I_LIMIT_US);

    if ((s_height_sat_high != 0U && s_height_vz_error_mps > 0.0f) ||
        (s_height_sat_low != 0U && s_height_vz_error_mps < 0.0f)) {
        allow_integrator = 0U;
    }
    if ((g_sys_tick - s_height_transition_start_ms) < HEIGHT_ENTRY_BLEND_MS) {
        allow_integrator = 0U;
    }

    output_candidate = s_height_p_us + i_candidate;
    if ((output_candidate > limit && s_height_vz_error_mps > 0.0f) ||
        (output_candidate < -limit && s_height_vz_error_mps < 0.0f)) {
        allow_integrator = 0U;
    }

    if (allow_integrator != 0U) {
        s_height_i_us = i_candidate;
    }
    s_height_correction_us = clampf(s_height_p_us + s_height_i_us,
                                    -limit, limit);
}

uint8_t HeightControl_EntryReady(void)
{
    float correction_limit = clampf(g_height_corr_limit_us, 0.0f, 30.0f);
    float capture_lo = (float)PWM_MIN_PULSE_US + correction_limit;
    float capture_hi = (float)THR_MAX_US - correction_limit;
    float min_safe_height_m = HEIGHT_ENTRY_MIN_M;

    if (s_height_est.vz_filt_mps < 0.0f) {
        /* 需要足够的地面余量以便在配置的指令加速度下将下降速度刹车归零 */
        float down_vz_mps = -s_height_est.vz_filt_mps;
        min_safe_height_m += (down_vz_mps * down_vz_mps) /
                             (2.0f * HEIGHT_VZ_CMD_ACCEL_MPS2);
    }

    return (s_height_reentry_block == 0U &&
            STICK_THROTTLE > HEIGHT_LOW_STICK &&
            STICK_THROTTLE <= HEIGHT_ENTRY_STICK_MAX &&
            s_height_est.valid != 0U &&
            s_height_est.good_frames >= HEIGHT_READY_FRAMES &&
            s_height_est.height_filt_m >= min_safe_height_m &&
            fabsf(s_height_est.vz_filt_mps) <= HEIGHT_ENTRY_VZ_MAX_MPS &&
            thr_base >= capture_lo &&
            thr_base <= capture_hi) ? 1U : 0U;
}

void HeightControl_EnterActive(uint32_t now_ms)
{
    s_height_mode = HEIGHT_MODE_ACTIVE;
    ManualTakeover_Reset();
    /* 同时捕获当前总距和 RC 油门作为中立点。
     * 不同电池、不同手控悬停油门不会在接管瞬间产生油门阶跃。 */
    s_height_hover_base_us = thr_base;
    s_height_stick_center = (int16_t)clampf((float)STICK_THROTTLE,
                                           -RC_STICK_MAX, RC_STICK_MAX);
    s_height_target_m = s_height_est.height_filt_m;
    s_height_target_vz_mps = clampf(s_height_est.vz_filt_mps,
                                    -HEIGHT_ENTRY_VZ_MAX_MPS,
                                     HEIGHT_ENTRY_VZ_MAX_MPS);
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
    s_height_cycle = 5U; /* 下一 tick 先跑 25Hz 位置环再跑 50Hz PI 速度环 */
    s_height_transition_start_ms = now_ms;
    s_height_transition_from_us = thr_base;
    s_height_entry_rejected = 0U;
    s_height_entry_pending = 0U;
    s_height_stick_active_prev = 0U;
    s_height_entry_vz_capture_active = 1U;
    HeightBuzz_Queue(HEIGHT_BUZZ_ACTIVE, now_ms);
}

/*
 * HeightControl_Update — 高度控制主入口，每 tick 调用
 * ============================================================================
 * 输入：
 *   manual_target_us — 手控/缓升/油门按键决定的油门目标 (us)
 *   now_ms           — 系统时间戳 (ms)
 * 输出：
 *   *collective_target_us — 高度控制输出的油门目标 (us)，覆盖 manual
 *   return 1U 时高度环接管油门，return 0U 时不接管
 *
 * 模式状态机：
 *   HEIGHT_MODE_OFF          → 手动油门，不接管
 *   HEIGHT_MODE_ACTIVE       → 高度保持，位置环+速度环串级控制
 *     - 摇杆偏离中心 → 命令垂直速度 → 松开后锁当前高度
 *     - 传感器失效 → HEIGHT_MODE_SENSOR_HOLD
 *     - 开关切回 Wait → HEIGHT_MODE_DEGRADED（手动接管）
 *   HEIGHT_MODE_SENSOR_HOLD  → 传感器短时失效，保持当前油门不变
 *     - N 秒内恢复 → 自动回到 ACTIVE
 *     - 超时或切开关 → HEIGHT_MODE_DEGRADED
 *   HEIGHT_MODE_DEGRADED     → 手动接管过渡，blend 回手动油门
 *
 * 进入条件（EntryReady）：
 *   - 开关切到 HeightHold 且未 reentry_block
 *   - 当前高度 > HEIGHT_ENTRY_MIN_M（考虑下降余量）
 *   - 垂直速度 |vz| ≤ HEIGHT_ENTRY_VZ_MAX_MPS
 *   - 油门在 [PWM_MIN+capture, THR_MAX-capture] 内
 *
 * 频率分频：
 *   - 位置环 (PositionLoop)  @ 150/6 = 25Hz  (s_height_cycle%6==0)
 *   - 速度环 (VelocityLoop)  @ 150/3 = 50Hz  (s_height_cycle%3==0)
 */
uint8_t HeightControl_Update(float manual_target_us,
                              uint32_t now_ms,
                              float *collective_target_us)
{
    uint8_t request = Height_SwitchRequest();
    uint8_t rising = (request != 0U && s_height_request_prev == 0U) ? 1U : 0U;

    if (request == 0U) {
        s_height_reentry_block = 0U;
        s_height_entry_rejected = 0U;
        s_height_entry_pending = 0U;
    }

    if (g_test_motor != 0U || g_test_ramp_active != 0U ||
        g_thr_override > 1.0f || soft_stop_active != 0U) {
        HeightControl_Reset();
        *collective_target_us = manual_target_us;
        return 0U;
    }

    if (s_height_mode == HEIGHT_MODE_ACTIVE && s_height_est.valid == 0U) {
        HeightControl_StartSensorHold(now_ms);
    } else if (s_height_mode == HEIGHT_MODE_ACTIVE && request == 0U) {
        HeightControl_StartManualTakeover(now_ms);
    }

    if (s_height_mode == HEIGHT_MODE_SENSOR_HOLD) {
        uint32_t hold_elapsed_ms = now_ms - s_height_transition_start_ms;

        if (request == 0U || STICK_THROTTLE <= HEIGHT_LOW_STICK) {
            HeightControl_StartManualTakeover(now_ms);
        } else if (hold_elapsed_ms <= HEIGHT_SENSOR_RECOVERY_MS &&
                   s_height_est.valid != 0U &&
                   s_height_est.good_frames >= HEIGHT_READY_FRAMES) {
            HeightControl_ResumeSensorHold(now_ms);
        }
    }

    if (s_height_mode == HEIGHT_MODE_OFF && rising != 0U &&
        s_height_reentry_block == 0U) {
        /* 等待期间保持手控油门权限。接管只会在这个短时、显式的开关请求
         * 窗口内触发，不会悄无声息地切换。 */
        s_height_entry_pending = 1U;
        s_height_entry_wait_start_ms = now_ms;
        s_height_entry_rejected = 0U;
        HeightBuzz_Queue(HEIGHT_BUZZ_REQUEST, now_ms);
    }

    if (s_height_mode == HEIGHT_MODE_OFF && request != 0U &&
        s_height_entry_pending != 0U) {
        if (HeightControl_EntryReady() != 0U) {
            HeightControl_EnterActive(now_ms);
        } else if ((now_ms - s_height_entry_wait_start_ms) >=
                   HEIGHT_ENTRY_WAIT_MS) {
            /* 超时的请求不会在之后自动接管，需要显式 Fly→Hover 开关循环 */
            s_height_entry_pending = 0U;
            s_height_reentry_block = 1U;
            s_height_entry_rejected = 1U;
            HeightBuzz_Queue(HEIGHT_BUZZ_REJECTED, now_ms);
        }
    }

    /* 上面可能在同一次 ISR tick 内激活定高。在降级交接前重新评估手动目标，
     * 确保降级后的第一个采样值恰好等于当前实际输出的总距。 */
    manual_target_us = ManualTakeover_Target(STICK_THROTTLE, manual_target_us);

    if (s_height_mode == HEIGHT_MODE_ACTIVE) {
        float controller_collective_us;
        float blend;
        uint32_t elapsed_ms;

        s_height_cycle++;
        if (s_height_cycle >= 6U) {
            s_height_cycle = 0U;
        }
        if (s_height_cycle == 0U) {
            HeightControl_PositionLoop();
        }
        if ((s_height_cycle % 3U) == 0U) {
            HeightControl_VelocityLoop();
        }

        controller_collective_us = clampf(s_height_hover_base_us + s_height_correction_us,
                                          (float)PWM_MIN_PULSE_US,
                                          (float)THR_MAX_US);
        elapsed_ms = now_ms - s_height_transition_start_ms;
        blend = (elapsed_ms >= HEIGHT_ENTRY_BLEND_MS) ? 1.0f :
                ((float)elapsed_ms / (float)HEIGHT_ENTRY_BLEND_MS);
        *collective_target_us = s_height_transition_from_us +
            blend * (controller_collective_us - s_height_transition_from_us);
        s_height_request_prev = request;
        return 1U;
    }

    if (s_height_mode == HEIGHT_MODE_SENSOR_HOLD) {
        *collective_target_us = s_height_sensor_hold_us;
        s_height_request_prev = request;
        return 1U;
    }

    if (s_height_mode == HEIGHT_MODE_DEGRADED) {
        uint32_t elapsed_ms = now_ms - s_height_transition_start_ms;
        float blend = (elapsed_ms >= HEIGHT_FALLBACK_BLEND_MS) ? 1.0f :
                      ((float)elapsed_ms / (float)HEIGHT_FALLBACK_BLEND_MS);
        *collective_target_us = s_height_transition_from_us +
            blend * (manual_target_us - s_height_transition_from_us);
        if (elapsed_ms >= HEIGHT_FALLBACK_BLEND_MS) {
            s_height_mode = HEIGHT_MODE_OFF;
        }
        s_height_request_prev = request;
        return 1U;
    }

    s_height_request_prev = request;
    *collective_target_us = manual_target_us;
    return 0U;
}

void HeightControl_ApplyHeadroom(float out_roll_mix,
                                  float out_pitch_mix,
                                  float out_yaw_mix,
                                  float *collective_us)
{
    float mix_term[4];
    float collective_lo = (float)PWM_MIN_PULSE_US;
    float collective_hi = (float)THR_MAX_US;
    float requested = *collective_us;
    uint8_t i;

    if (s_height_mode != HEIGHT_MODE_ACTIVE &&
        s_height_mode != HEIGHT_MODE_SENSOR_HOLD) {
        s_height_sat_high = 0U;
        s_height_sat_low = 0U;
        return;
    }

    /* 保持现有混控矩阵符号不变，仅计算总距可用的上下空间 */
    mix_term[0] =  out_roll_mix - out_pitch_mix - out_yaw_mix;
    mix_term[1] = -out_roll_mix - out_pitch_mix + out_yaw_mix;
    mix_term[2] = -out_roll_mix + out_pitch_mix - out_yaw_mix;
    mix_term[3] =  out_roll_mix + out_pitch_mix + out_yaw_mix;

    for (i = 0U; i < 4U; i++) {
        float lo = (float)PWM_MIN_PULSE_US - mix_term[i];
        float hi = (float)PWM_SAFE_MAX_US - mix_term[i];
        if (lo > collective_lo) collective_lo = lo;
        if (hi < collective_hi) collective_hi = hi;
    }

    if (collective_lo > collective_hi) {
        *collective_us = clampf(requested,
                                (float)PWM_MIN_PULSE_US,
                                (float)THR_MAX_US);
        s_height_sat_high = 1U;
        s_height_sat_low = 1U;
        return;
    }

    *collective_us = clampf(requested, collective_lo, collective_hi);
    s_height_sat_high = (requested > collective_hi ||
                         (s_height_correction_us >= g_height_corr_limit_us &&
                          s_height_vz_error_mps > 0.0f)) ? 1U : 0U;
    s_height_sat_low = (requested < collective_lo ||
                        (s_height_correction_us <= -g_height_corr_limit_us &&
                         s_height_vz_error_mps < 0.0f)) ? 1U : 0U;
}

uint16_t HeightControl_DiagFlags(void)
{
    uint16_t flags = (uint16_t)s_height_est.diag_flags;
    if (s_height_mode == HEIGHT_MODE_ACTIVE) flags |= HEIGHT_DIAG_ACTIVE;
    if (s_height_mode == HEIGHT_MODE_SENSOR_HOLD) flags |= HEIGHT_DIAG_SENSOR_HOLD;
    if (s_height_sat_high != 0U) flags |= HEIGHT_DIAG_SAT_HIGH;
    if (s_height_sat_low != 0U) flags |= HEIGHT_DIAG_SAT_LOW;
    if (s_height_entry_rejected != 0U) flags |= HEIGHT_DIAG_ENTRY_REJECTED;
    if (s_height_entry_pending != 0U) flags |= HEIGHT_DIAG_ENTRY_PENDING;
    if (s_manual_takeover_active != 0U) flags |= HEIGHT_DIAG_MANUAL_REMAP;
    return flags;
}
