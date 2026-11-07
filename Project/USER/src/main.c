#include "include.h"
#include "character_img.h"
#include "SEEKFREE_MT9V03X_CSI.h"
#include "SEEKFREE_WIRELESS.h"
#include "tune_config.h"
#include "wireless_boot_flag.h"
#include "fsl_cache.h"
#include <stdio.h>
#include <string.h>
#define BUILD_STRING_INNER(x) #x
#define BUILD_STRING(x) BUILD_STRING_INNER(x)
#define STRINGIFY_BUILD_ALGORITHM BUILD_STRING(TRACK_ALGORITHM)

/* Older parameter profiles have no curve-speed setting. */
#ifndef TRACK_CURVE_SLOWDOWN_PERCENT
#define TRACK_CURVE_SLOWDOWN_PERCENT 30U
#endif
#if TRACK_CURVE_SLOWDOWN_PERCENT > 100U
#error "TRACK_CURVE_SLOWDOWN_PERCENT must be within 0..100"
#endif


#if TRACK_ALGORITHM == 14
#include "track_ring14.h"
#define track_ring13_reset track_ring14_reset
#define track_ring13_process track_ring14_process
#define track_ring13_suspend track_ring14_suspend
#define track_ring13_state_name track_ring14_state_name
#define track_ring13_binary_preview track_ring14_binary_preview
#define track_ring13_path_x track_ring14_path_x
#define track_ring13_path_confidence track_ring14_path_confidence
#define track_ring13_side track_ring14_side
#define track_ring13_lookahead_row track_ring14_lookahead_row
#define track_ring13_left_length track_ring14_points
#define track_ring13_right_length track_ring14_error_mm
#define track_ring13_jump_count track_ring14_rejects
#define track_ring13_jump_from track_ring14_forward_mm
#define track_ring13_jump_to track_ring14_lateral_mm
#define track_ring13_phase track_fsm_state
#elif TRACK_ALGORITHM == 15
#include "track_ring15.h"
#elif TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
#include "track_mode16.h"
#endif

/* Active track algorithm selected in tune_config.h. */
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
#include "track_cross_curve.h"
#if TRACK_ALGORITHM == 8
#include "track_fusion8.h"
#elif TRACK_ALGORITHM == 10
#include "track_cross_curve10.h"
#elif TRACK_ALGORITHM == 12
#include "track_path12.h"
#elif (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
#if TRACK_ALGORITHM == 13
#include "track_ring13.h"
#endif
#elif TRACK_ALGORITHM == 11
#include "track_ring11.h"
#endif
typedef track_cross_curve_result_t track_result_t;
#if TRACK_ALGORITHM == 8
#define TRACK_MODE_RESET       track_fusion8_reset
#define TRACK_MODE_PROCESS_EX  track_fusion8_process
#define TRACK_MODE_SUSPEND     track_fusion8_suspend
#define TRACK_MODE_STATE_NAME track_fusion8_state_name
#elif TRACK_ALGORITHM == 10
#define TRACK_MODE_RESET       track_cross_curve10_reset
#define TRACK_MODE_PROCESS_EX  track_cross_curve10_process
#define TRACK_MODE_SUSPEND     track_cross_curve10_suspend
#define TRACK_MODE_STATE_NAME track_cross_curve10_state_name
#elif TRACK_ALGORITHM == 12
#define TRACK_MODE_RESET track_path12_reset
#define TRACK_MODE_PROCESS_EX track_path12_process
#define TRACK_MODE_SUSPEND track_path12_suspend
#define TRACK_MODE_STATE_NAME track_path12_state_name
#elif TRACK_ALGORITHM == 15
#define TRACK_MODE_RESET track_ring15_reset
#define TRACK_MODE_PROCESS_EX track_ring15_process
#define TRACK_MODE_SUSPEND track_ring15_suspend
#define TRACK_MODE_STATE_NAME track_ring15_state_name
#elif (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14)
#define TRACK_MODE_RESET track_ring13_reset
#define TRACK_MODE_PROCESS_EX track_ring13_process
#define TRACK_MODE_SUSPEND track_ring13_suspend
#define TRACK_MODE_STATE_NAME track_ring13_state_name
#elif TRACK_ALGORITHM == 11
#define TRACK_MODE_RESET track_ring11_reset
#define TRACK_MODE_PROCESS_EX track_ring11_process
#define TRACK_MODE_SUSPEND track_ring11_suspend
#define TRACK_MODE_STATE_NAME track_ring11_state_name
#else
#define TRACK_MODE_RESET       track_cross_curve_reset
#define TRACK_MODE_PROCESS_EX  track_cross_curve_process
#define TRACK_MODE_SUSPEND     track_cross_curve_suspend
#define TRACK_MODE_STATE_NAME track_cross_curve_state_name
#endif
#if TRACK_ALGORITHM == 10
#define TRACK_MODE_PATH_X      track_cross_curve10_path_x
#define TRACK_MODE_PATH_CONF   track_cross_curve10_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_cross_curve10_binary_preview
#elif TRACK_ALGORITHM == 12
#define TRACK_MODE_PATH_X track_path12_path_x
#define TRACK_MODE_PATH_CONF track_path12_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_path12_binary_preview
#elif TRACK_ALGORITHM == 15
#define TRACK_MODE_PATH_X track_ring15_path_x
#define TRACK_MODE_PATH_CONF track_ring15_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_ring15_binary_preview
#elif (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14)
#define TRACK_MODE_PATH_X track_ring13_path_x
#define TRACK_MODE_PATH_CONF track_ring13_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_ring13_binary_preview
#elif TRACK_ALGORITHM == 11
#define TRACK_MODE_PATH_X track_ring11_path_x
#define TRACK_MODE_PATH_CONF track_ring11_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_ring11_binary_preview
#else
#define TRACK_MODE_PATH_X      track_cross_curve_path_x
#define TRACK_MODE_PATH_CONF   track_cross_curve_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_cross_curve_binary_preview
#endif
#elif TRACK_ALGORITHM == 3
#include "track_mode4.h"
typedef track_mode4_result_t track_result_t;
#define TRACK_MODE_RESET     track_mode4_reset
#define TRACK_MODE_PROCESS   track_mode4_process
#define TRACK_MODE_PATH_X    track_mode4_path_x
#define TRACK_MODE_PATH_CONF track_mode4_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_mode4_binary_preview
#elif TRACK_ALGORITHM == 16
typedef track_mode1_result_t track_result_t;
#define TRACK_MODE_RESET track_mode16_reset
#define TRACK_MODE_PROCESS track_mode16_process
#define TRACK_MODE_PATH_X track_mode16_path_x
#define TRACK_MODE_PATH_CONF track_mode16_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_mode16_binary_preview
#elif TRACK_ALGORITHM == 17
#include "track_mode4.h"
typedef track_mode1_result_t track_result_t;
static int track17_uses_ring(void)
{
    return track_mode16_phase != A16_ROAD && track_mode16_phase != A16_PREP;
}

static int track17_corner_active;
static void track17_reset(void)
{
    track17_corner_active = 0;
    track_mode4_reset();
    track_mode16_reset();
}

static track_result_t track17_process(const uint8_t image[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH])
{
    /* Keep both observers current. Only one owns the control and LCD per frame. */
    track_mode4_result_t road = track_mode4_process(image);
    track_result_t ring = track_mode16_process(image);
    int weak_road = road.anchor_confidence < TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE ||
                    road.target_confidence < TRACK_DRIVE_MIN_TARGET_CONFIDENCE ||
                    road.valid_rows < TRACK_DRIVE_MIN_VALID_ROWS;
    track17_corner_active = track_mode16_phase == A16_ROAD &&
                            track_mode16_corner(&ring, weak_road || track17_corner_active);
    if (track17_uses_ring() || track17_corner_active) return ring;
    track_mode16_target_row = TRACK_CONT_TARGET_ROW;
    return (track_result_t){road.anchor_x, road.target_x, road.anchor_confidence,
                            road.target_confidence, road.valid_rows, road.global_threshold};
}

#define TRACK_MODE_RESET track17_reset
#define TRACK_MODE_PROCESS track17_process
#define TRACK_MODE_PATH_X ((track17_uses_ring() || track17_corner_active) ? track_mode16_path_x : track_mode4_path_x)
#define TRACK_MODE_PATH_CONF ((track17_uses_ring() || track17_corner_active) ? track_mode16_path_confidence : track_mode4_path_confidence)
#else
#include "track_mode1.h"
typedef track_mode1_result_t track_result_t;
#define TRACK_MODE_RESET     track_mode1_reset
#define TRACK_MODE_PROCESS   track_mode1_process
#define TRACK_MODE_PATH_X    track_mode1_path_x
#define TRACK_MODE_PATH_CONF track_mode1_path_confidence
#define TRACK_MODE_BINARY_PREVIEW track_mode1_binary_preview
#endif

#define CAMERA_TEST_MOTOR1_DIR C11
#define CAMERA_TEST_MOTOR1_PWM PWM2_MODULE2_CHA_C10
#define CAMERA_TEST_MOTOR2_DIR D3
#define CAMERA_TEST_MOTOR2_PWM PWM2_MODULE3_CHA_D2
#define CAMERA_TEST_MOTOR_PWM_HZ TRACK_MOTOR_PWM_HZ
#define TRACK_SERVO_PWM PWM4_MODULE2_CHA_C30
#define TRACK_SERVO_HZ 50U
#define TRACK_MOTOR_DUTY_LIMIT ((PWM_DUTY_MAX * TRACK_WHEEL_PWM_MAX_PERCENT) / 100U)
#define TRACK_MOTOR_RAMP_STEP ((PWM_DUTY_MAX * TRACK_MOTOR_RAMP_STEP_PERCENT_PER_FRAME) / 100U)
#define TRACK_SERVO_DUTY(angle) ((uint32)((float)PWM_DUTY_MAX / 20.0f * (0.5f + (float)(angle) / 90.0f)))

/* DAP-visible status: 1=waiting for camera UART config, 2=CSI running,
 * 3=at least one CSI frame received. */
volatile uint32_t camera_test_magic = 0x43414D31U;
volatile uint32_t camera_test_stage = 0U;
volatile uint32_t camera_test_frames = 0U;
volatile uint32_t camera_test_frame_min = 255U;
volatile uint32_t camera_test_frame_max = 0U;
volatile uint32_t camera_test_pixel_60_94 = 0U;
/* DAP-visible run state: start tracking automatically; STOP remains available. */
/* Algorithm14's color-state debug image boots disarmed; camera planning still
 * runs, and the user can explicitly ARM it from the control panel. */
volatile uint32_t track_arm_request = 1U;
volatile uint32_t track_motor_duty = 0U;
static uint32_t track_curve_scale_x100 = 10000U;
#if TRACK_ALGORITHM == 16
static uint8_t exit_straight_frames_needed;
#endif
volatile uint32_t track_target_x = TRACK_STEER_REFERENCE_X;
volatile uint32_t track_target_confidence = 0U;
volatile uint32_t track_servo_angle = TRACK_SERVO_CENTER_DEG;
#if TRACK_ALGORITHM == 16
static unsigned road_search_remaining_ms;
#endif
volatile uint32_t track_valid_rows = 0U;
volatile uint32_t track_mode_number = TRACK_ALGORITHM;
volatile uint32_t track_heartbeat = 0U;
volatile int32_t track_encoder1_speed_counts_per_10ms_x10 = 0;
volatile int32_t track_encoder2_speed_counts_per_10ms_x10 = 0;
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
volatile track_cross_curve_result_t track_fsm_diagnostics;
volatile uint32_t track_fsm_state = FSM_WAIT;
volatile uint32_t track_fsm_reason = FSM_REASON_INIT;
volatile uint32_t track_fsm_processing_cycles = 0U;
#endif

typedef struct
{
    int16 last_count;
    uint32 last_cycle;
    float integral;
    float last_error;
    uint8 initialized;
} track_speed_pid_state_t;

static track_speed_pid_state_t track_speed_pid_m1;
static track_speed_pid_state_t track_speed_pid_m2;

static uint8_t camera_test_image[MT9V03X_CSI_H][MT9V03X_CSI_W];
static uint8_t camera_test_display_image[MT9V03X_CSI_H][MT9V03X_CSI_W];
#if TRACK_ALGORITHM == 14
static uint16_t camera_test_debug_rgb565[MT9V03X_CSI_H][MT9V03X_CSI_W];
#endif
static char wireless_command[24];
static uint8_t wireless_command_length;
static uint8_t wireless_discard_line;
volatile uint32_t wireless_rx_error_count;
volatile uint32_t wireless_rx_overlong_count;
extern LPUART_Type *UARTN[];

static void track_speed_pid_reset(void)
{
    track_speed_pid_m1.integral = 0.0f;
    track_speed_pid_m1.last_error = 0.0f;
    track_speed_pid_m1.initialized = 0U;
    track_speed_pid_m2.integral = 0.0f;
    track_speed_pid_m2.last_error = 0.0f;
    track_speed_pid_m2.initialized = 0U;
    track_encoder1_speed_counts_per_10ms_x10 = 0;
    track_encoder2_speed_counts_per_10ms_x10 = 0;
}

#if TRACK_SPEED_PID_ENABLED
static float track_speed_pid_update(track_speed_pid_state_t *state,
                                    int16 current_count,
                                    float target_counts_per_10ms,
                                    uint32 now_cycle,
                                    float *measured_counts_per_10ms)
{
    uint32 elapsed_cycles;
    int16 delta_count;
    int32 signed_delta_count;
    float dt_seconds;
    float error;
    float prior_integral;
    float correction;
    float limit;

    if (!state->initialized)
    {
        state->last_count = current_count;
        state->last_cycle = now_cycle;
        state->integral = 0.0f;
        state->last_error = target_counts_per_10ms;
        state->initialized = 1U;
        *measured_counts_per_10ms = 0.0f;
        return 0.0f;
    }

    elapsed_cycles = (uint32)(now_cycle - state->last_cycle);
    delta_count = (int16)((uint16)current_count - (uint16)state->last_count);
    signed_delta_count = (int32)delta_count;
#if TRACK_ENCODER1_REVERSE
    if (state == &track_speed_pid_m1) signed_delta_count = -signed_delta_count;
#endif
#if TRACK_ENCODER2_REVERSE
    if (state == &track_speed_pid_m2) signed_delta_count = -signed_delta_count;
#endif
    state->last_count = current_count;
    state->last_cycle = now_cycle;
    if (elapsed_cycles == 0U)
    {
        *measured_counts_per_10ms = 0.0f;
        return 0.0f;
    }

    dt_seconds = (float)elapsed_cycles / (float)SystemCoreClock;
    if (dt_seconds <= 0.0f)
    {
        *measured_counts_per_10ms = 0.0f;
        return 0.0f;
    }
    *measured_counts_per_10ms = (float)signed_delta_count * 0.01f / dt_seconds;
    error = target_counts_per_10ms - *measured_counts_per_10ms;

    prior_integral = state->integral;
    state->integral += error * dt_seconds;
    limit = (float)TRACK_SPEED_PID_IMAX;
    if (state->integral > limit) state->integral = limit;
    if (state->integral < -limit) state->integral = -limit;

    correction = ((float)TRACK_SPEED_PID_KP * error) +
                 ((float)TRACK_SPEED_PID_KI * state->integral) +
                 ((float)TRACK_SPEED_PID_KD * (error - state->last_error) / dt_seconds);
    limit = (float)TRACK_SPEED_PID_OUTMAX;

    /* Anti-windup: do not accumulate error farther into an active output rail. */
    if ((correction > limit && error > 0.0f) ||
        (correction < -limit && error < 0.0f))
    {
        state->integral = prior_integral;
        correction = ((float)TRACK_SPEED_PID_KP * error) +
                     ((float)TRACK_SPEED_PID_KI * state->integral) +
                     ((float)TRACK_SPEED_PID_KD * (error - state->last_error) / dt_seconds);
    }
    if (correction > limit) correction = limit;
    if (correction < -limit) correction = -limit;
    state->last_error = error;
    return correction;
}
#endif

static void wireless_reply(const char *text)
{
    uart_putbuff(WIRELESS_UART, (uint8 *)text, (uint32)strlen(text));
}

static void wireless_poll(void)
{
    uint8 byte;

    for (;;)
    {
        /* OR blocks reception until cleared, even after the FIFO is read.
         * Reject the damaged line; resume parsing at its next newline. */
        uint32_t errors = UARTN[WIRELESS_UART]->STAT &
            (LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK |
             LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK);
        if (errors)
        {
            UARTN[WIRELESS_UART]->STAT = errors;
            ++wireless_rx_error_count;
            wireless_command_length = 0U;
            wireless_discard_line = 1U;
        }
        if (!uart_query(WIRELESS_UART, &byte)) break;
        if (wireless_discard_line)
        {
            if (byte == '\n') wireless_discard_line = 0U;
            continue;
        }
        if (byte == '\r') continue;
        if (byte != '\n')
        {
            if (wireless_command_length + 1U < sizeof(wireless_command))
                wireless_command[wireless_command_length++] = (char)byte;
            else
            {
                ++wireless_rx_overlong_count;
                wireless_command_length = 0U;
                wireless_discard_line = 1U;
            }
            continue;
        }

        wireless_command[wireless_command_length] = '\0';
        if (strcmp(wireless_command, "HELLO") == 0)
        {
            char status[112];
            snprintf(status, sizeof(status),
                     "SU7 TRACK%lu CAMERA=%lu FRAMES=%lu CAMCFG=%u ARM=%lu DUTY=%lu"
#ifdef SU7_WIRELESS_APP
                     " UPDATE=2"
#endif
                     "\r\n",
                     (unsigned long)track_mode_number,
                     (unsigned long)camera_test_stage,
                     (unsigned long)camera_test_frames,
                     (unsigned int)mt9v03x_config_timeout,
                     (unsigned long)track_arm_request,
                     (unsigned long)track_motor_duty);
            wireless_reply(status);
        }
#if TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
        else if (strcmp(wireless_command,"RING")==0) {
            char status[192];
#if TRACK_ALGORITHM == 12
            snprintf(status,sizeof(status),"A12 S=%lu SIDE=%ld N=%lu Y=%lu ISLAND=%ld X=%d\r\n",(unsigned long)track_fsm_state,(long)track_path12_side,(unsigned long)track_path12_path_rows,(unsigned long)track_path12_lookahead_row,(long)track_path12_island_row,(int)track_fsm_diagnostics.target_x);
#elif TRACK_ALGORITHM == 14
            snprintf(status,sizeof(status),"A14 S=%lu PH=%u F=%ld L=%ld R=%ld E=%ld N=%lu ARC=%lu H=%u C=%d,%d V=%u P=%u AGE=%lu X=%d\r\n",
                (unsigned long)track_fsm_state,(unsigned int)track_ring14_display_phase,
                (long)track_ring14_forward_mm,(long)track_ring14_lateral_mm,
                (long)track_ring14_radius_mm,(long)track_ring14_error_mm,
                (unsigned long)track_ring14_points,(unsigned long)track_ring14_arc_degrees,
                (unsigned int)track_ring14_half_arc_candidate,(int)track_ring14_center_x,
                (int)track_ring14_center_y,(unsigned int)track_ring14_center_visible,
                (unsigned int)track_ring14_center_predicted,(unsigned long)track_ring14_center_age_ms,
                (int)track_fsm_diagnostics.target_x);
#elif TRACK_ALGORITHM == 15
            snprintf(status,sizeof(status),"A15 S=%lu ST=%u PH=%u SIDE=%ld L=%d,%d M=%d,%d U=%d,%d Y=%lu X=%d\r\n",
                (unsigned long)track_fsm_state,(unsigned int)track_ring15_stage,(unsigned int)track_ring15_display_phase,
                (long)track_ring15_side,
                (int)track_ring15_lower_row,(int)track_ring15_lower_x,
                (int)track_ring15_middle_row,(int)track_ring15_middle_x,
                (int)track_ring15_upper_row,(int)track_ring15_upper_x,
                (unsigned long)track_ring15_lookahead_row,(int)track_fsm_diagnostics.target_x);
#else
            snprintf(status,sizeof(status),"A14 S=%lu P=%lu SIDE=%ld L=%ld R=%ld Y=%lu X=%d J=%lu F=%ld T=%ld\r\n",(unsigned long)track_fsm_state,(unsigned long)track_ring13_phase,(long)track_ring13_side,(long)track_ring13_left_length,(long)track_ring13_right_length,(unsigned long)track_ring13_lookahead_row,(int)track_fsm_diagnostics.target_x,(unsigned long)track_ring13_jump_count,(long)track_ring13_jump_from,(long)track_ring13_jump_to);
#endif
            wireless_reply(status);
        }
#elif TRACK_ALGORITHM == 11
        else if (strcmp(wireless_command, "RING") == 0)
        {
            char status[128];
            snprintf(status,sizeof(status),"A11 S=%lu C=%ld E=%lu ROW=%ld LOOK=%lu SEEN=%lu SH=%ld D=%ld SY=%ld\r\n",
                (unsigned long)track_fsm_state,(long)track_ring11_candidate_side,
                (unsigned long)track_ring11_entry_count,(long)track_ring11_entry_row,
                (unsigned long)track_ring11_lookahead_row,(unsigned long)track_ring11_seen_states,
                (long)track_ring11_shape_side,(long)track_ring11_shape_score,(long)track_ring11_shape_row);
            wireless_reply(status);
        }
#endif
        else if (strcmp(wireless_command, "BUILD") == 0) {
            wireless_reply("SU7 A" STRINGIFY_BUILD_ALGORITHM " VISION-FSM-20261024\r\n");
        }
        else if (strcmp(wireless_command, "ARM") == 0)
        {
#if TRACK_ALGORITHM == 16
            if (!track_arm_request) track16_new_lap();
#endif
            track_arm_request = 1U;
            wireless_reply("OK ARM\r\n");
        }
        else if (strcmp(wireless_command, "STOP") == 0)
        {
            track_arm_request = 0U;
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
            TRACK_MODE_SUSPEND(FSM_REASON_STOP);
            track_fsm_state = FSM_WAIT;
            track_fsm_reason = FSM_REASON_STOP;
#endif
            pwm_duty(CAMERA_TEST_MOTOR1_PWM, 0U);
            pwm_duty(CAMERA_TEST_MOTOR2_PWM, 0U);
            track_motor_duty = 0U;
#if TRACK_ALGORITHM == 16
            road_search_remaining_ms = 0;
            exit_straight_frames_needed = 0U;
#endif
            wireless_reply("OK STOP\r\n");
        }
        else if (strcmp(wireless_command, "ENTER") == 0)
        {
#ifdef SU7_WIRELESS_APP
            track_arm_request = 0U;
            pwm_duty(CAMERA_TEST_MOTOR1_PWM, 0U);
            pwm_duty(CAMERA_TEST_MOTOR2_PWM, 0U);
            track_motor_duty = 0U;
#if TRACK_ALGORITHM == 16
            road_search_remaining_ms = 0;
            exit_straight_frames_needed = 0U;
#endif
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
            TRACK_MODE_SUSPEND(FSM_REASON_STOP);
#endif
            su7_boot_flag_init();
            SNVS->LPGPR[0] = 0x3755424CU;
            __DSB();
            if (SNVS->LPGPR[0] != 0x3755424CU) {
                wireless_reply("ERR BOOT_FLAG\r\n");
            } else {
                wireless_reply("OK ENTER\r\n");
                systick_delay_ms(100U);
                NVIC_SystemReset();
            }
#else
            wireless_reply("ERR DAP_ONLY\r\n");
#endif
        }
        else
        {
            wireless_reply("ERR COMMAND\r\n");
        }
        wireless_command_length = 0U;
    }
}

static void track_outputs_init(void)
{
    gpio_init(CAMERA_TEST_MOTOR1_DIR, GPO, GPIO_LOW, GPIO_PIN_CONFIG);
    gpio_init(CAMERA_TEST_MOTOR2_DIR, GPO, GPIO_LOW, GPIO_PIN_CONFIG);
    pwm_init(CAMERA_TEST_MOTOR1_PWM, CAMERA_TEST_MOTOR_PWM_HZ, 0U);
    pwm_init(CAMERA_TEST_MOTOR2_PWM, CAMERA_TEST_MOTOR_PWM_HZ, 0U);
    pwm_duty(CAMERA_TEST_MOTOR1_PWM, 0U);
    pwm_duty(CAMERA_TEST_MOTOR2_PWM, 0U);
    pwm_init(TRACK_SERVO_PWM, TRACK_SERVO_HZ, TRACK_SERVO_DUTY(TRACK_SERVO_CENTER_DEG));
    pwm_duty(TRACK_SERVO_PWM, TRACK_SERVO_DUTY(TRACK_SERVO_CENTER_DEG));
#if TRACK_SPEED_PID_ENABLED
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER0_C0, QTIMER1_TIMER1_C1);
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER2_C2, QTIMER1_TIMER3_C24);
#endif
    track_speed_pid_reset();
}

static void track_drive_forward(uint32_t duty1, uint32_t duty2)
{
    gpio_set(CAMERA_TEST_MOTOR1_DIR, TRACK_MOTOR1_FORWARD_LEVEL ? GPIO_HIGH : GPIO_LOW);
    gpio_set(CAMERA_TEST_MOTOR2_DIR, TRACK_MOTOR2_FORWARD_LEVEL ? GPIO_HIGH : GPIO_LOW);
    pwm_duty(CAMERA_TEST_MOTOR1_PWM, duty1 > PWM_DUTY_MAX ? PWM_DUTY_MAX : duty1);
    pwm_duty(CAMERA_TEST_MOTOR2_PWM, duty2 > PWM_DUTY_MAX ? PWM_DUTY_MAX : duty2);
}

static void track_stop_outputs(int center_servo)
{
    pwm_duty(CAMERA_TEST_MOTOR1_PWM, 0U);
    pwm_duty(CAMERA_TEST_MOTOR2_PWM, 0U);
    track_motor_duty = 0U;
#if TRACK_ALGORITHM == 16
    exit_straight_frames_needed = 0U;
#endif
    track_curve_scale_x100 = 10000U;
#if TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
#if TRACK_ALGORITHM == 17
    if (track17_uses_ring())
#endif
    {
        if (center_servo) {
            track_servo_angle = TRACK_SERVO_CENTER_DEG;
            pwm_duty(TRACK_SERVO_PWM, TRACK_SERVO_DUTY(TRACK_SERVO_CENTER_DEG));
        }
    }
#endif
    track_speed_pid_reset();
}

static void track_stop(void)
{
#if TRACK_ALGORITHM == 16
    road_search_remaining_ms = 0;
#endif
    track_stop_outputs(1);
}

#if TRACK_ALGORITHM == 16
/* On an S-bend the linkage passes neutral while the next bend is in view.
 * Accept straight road only when both halves of the visible path agree. */
static int track16_road_is_straight(void)
{
    int low = TRACK_MODE1_WIDTH, high = -1;
    for (int y = TRACK_OTSUSCAN_ANCHOR_ROW; y >= (int)TRACK_OTSUSCAN_TARGET_ROW; y -= 4) {
        if (track_mode16_path_confidence[y] < TRACK_DRIVE_MIN_TARGET_CONFIDENCE) return 0;
        int x = track_mode16_path_x[y];
        if (x < low) low = x;
        if (x > high) high = x;
    }
    return high >= low && high - low <= 12;
}

static uint32_t track16_exit_scale(uint32_t requested, int turn_amount)
{
    if (track_mode16_phase != A16_ROAD) {
        exit_straight_frames_needed = 0U;
        return requested;
    }
    if (turn_amount >= 6) exit_straight_frames_needed = 2U;
    if (exit_straight_frames_needed) {
        if (turn_amount <= 3 && track16_road_is_straight())
            --exit_straight_frames_needed;
        else
            exit_straight_frames_needed = 2U;
        if (exit_straight_frames_needed && requested > track_curve_scale_x100)
            return track_curve_scale_x100;
    }
    return requested;
}
#endif

static void track_control(const track_result_t *line)
{
#if TRACK_ALGORITHM == 16
    if (track16_lap == A16_LAP_FINISHED) {
        track_stop();
        return;
    }
#endif
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
    int32_t angle = (line->servo_cdeg + 50) / 100;
    uint32_t pwm_cap_percent = line->pwm_cap_percent;
    uint8_t allowed = track_arm_request == 1U && line->drive_allowed;
#else
    int32_t error = (int32_t)line->target_x - (int32_t)TRACK_STEER_REFERENCE_X;
    int32_t steer_gain = (int32_t)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL;
#if TRACK_ALGORITHM == 16
    /* In ROAD, steer toward the optical axis of the centered camera.
     * Island states retain their calibrated branch-entry reference. */
    if (track_mode16_phase == A16_ROAD)
        error = (int32_t)line->target_x - (int32_t)(TRACK_MODE1_WIDTH / 2U);
    if (line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE) {
        int32_t heading = (int32_t)line->target_x - (int32_t)line->anchor_x;
        error += track_mode16_phase == A16_ROAD ? heading / 2 : heading / 4;
    }
    if (track_mode16_phase == A16_ROAD && track_mode16_target_row == TRACK_A16_LOOKAHEAD_ROW &&
        !track_mode16_corner_active && !track_mode16_obstacle_side &&
        line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE) {
        int near_offset = line->anchor_x - (int)(TRACK_MODE1_WIDTH / 2U);
        if ((near_offset > 2 && error > 2) || (near_offset < -2 && error < -2))
            error += near_offset * TRACK_A16_NEAR_GAIN_PERCENT / 100;
    }
    /* Ignore small pixel fluctuations; restore gain as path curvature grows. */
    if (error >= -2 && error <= 2) error = 0;
    if (line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE) {
        int32_t heading = (int32_t)line->target_x - (int32_t)line->anchor_x;
        if (heading < 0) heading = -heading;
        steer_gain = 18;
        if (heading >= 3) steer_gain = 27;
        if (heading >= 6) steer_gain = (int32_t)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL + 3;
    } else {
        steer_gain = 18;
    }
#endif
#if TRACK_ALGORITHM == 17
    /* Keep small corrections linear. A visible bend can use the remaining
     * travel sooner without moving neutral or the mechanical endpoints. */
    if (!track17_uses_ring() && line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE) {
        int heading = line->target_x - line->anchor_x;
        if (error > 16 && (track17_corner_active || heading >= 5))
            error += (error - 16) / 3;
        else if (error < -16 && (track17_corner_active || heading <= -5))
            error += (error + 16) / 3;
    }
#endif
#if TRACK_ALGORITHM == 16
    /* A car parallel to a straight can have almost no heading error while
     * its whole footprint is displaced. Restore steering progressively. */
    int correction = 0;
    if (track_mode16_phase == A16_ROAD && (track_mode16_straight || track_mode16_obstacle_side) &&
        line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE) {
        int anchor_error = line->anchor_x - (int)(TRACK_MODE1_WIDTH/2U);
        if ((anchor_error > 2 && error > 2) || (anchor_error < -2 && error < -2)) {
            int magnitude = anchor_error < 0 ? -anchor_error : anchor_error;
            correction = 1 + magnitude/7;
            if (correction > 5) correction = 5;
            steer_gain += 3*correction;
        }
    }
#endif
#if TRACK_ALGORITHM == 16
    int bend_support=0;
    if(track_mode16_phase==A16_ROAD && !track_mode16_obstacle_side &&
       track_mode16_path_confidence[72]>=60 && line->anchor_confidence>=60 && line->target_confidence>=60) {
        int far=track_mode16_path_x[72]-line->target_x;
        int near=line->target_x-line->anchor_x;
        if((far>=5 && near>=3 && error>4)||(far<=-5 && near<=-3 && error< -4)){
            bend_support=1;error+=far/3;steer_gain+=4;
        }
    }
#endif
    int32_t angle = (int32_t)TRACK_SERVO_CENTER_DEG -
                    (error * steer_gain) / 100;
#if TRACK_ALGORITHM == 3 || TRACK_ALGORITHM == 16
    /* Apply the user's configured maximum steering deviation. */
#if TRACK_ALGORITHM == 16
    /* Keep straight-road vision noise from commanding large servo swings,
     * while progressively restoring the full range as the traced path bends. */
    if (line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE) {
        int32_t heading = (int32_t)line->target_x - (int32_t)line->anchor_x;
        int32_t limit = 3;
        if (heading < 0) heading = -heading;
        limit += correction;
        if (heading >= 3) limit = 10;
        if (heading >= 6) limit = (int32_t)TRACK_A16_MAX_STEER_DELTA_DEG;
        if (angle < (int32_t)TRACK_SERVO_CENTER_DEG - limit)
            angle = (int32_t)TRACK_SERVO_CENTER_DEG - limit;
        if (angle > (int32_t)TRACK_SERVO_CENTER_DEG + limit)
            angle = (int32_t)TRACK_SERVO_CENTER_DEG + limit;
    }
#endif
#if TRACK_ALGORITHM == 16
    const int travel_limit = TRACK_A16_MAX_STEER_DELTA_DEG;
#else
    const int travel_limit = TRACK_A3_MAX_STEER_DELTA_DEG;
#endif
    if (angle < (int32_t)TRACK_SERVO_CENTER_DEG - travel_limit)
        angle = (int32_t)TRACK_SERVO_CENTER_DEG - travel_limit;
    if (angle > (int32_t)TRACK_SERVO_CENTER_DEG + travel_limit)
        angle = (int32_t)TRACK_SERVO_CENTER_DEG + travel_limit;
#endif
    uint32_t pwm_cap_percent = TRACK_WHEEL_PWM_MAX_PERCENT;
#if TRACK_ALGORITHM == 17
    if (!track17_uses_ring()) pwm_cap_percent = TRACK_A17_ROAD_PWM_PERCENT;
    if (track17_corner_active && pwm_cap_percent > 17U) pwm_cap_percent = 17U;
#endif
#if TRACK_ALGORITHM == 16
    if ((track_mode16_corner_active || track_mode16_obstacle_side) && pwm_cap_percent > 17U) pwm_cap_percent = 17U;
#endif
    uint8_t allowed = track_arm_request == 1U &&
        line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE &&
        line->valid_rows >= TRACK_DRIVE_MIN_VALID_ROWS;
#endif
    uint32_t base_duty;
    uint32_t curve_duty_limit;
    uint32_t speed_scale_x100 = 10000U;
    uint32_t speed_scale_delta;
    int32_t turn_amount;
    int32_t turn_span;
    uint32_t duty1;
    uint32_t duty2;
    int32_t corrected_duty1;
    int32_t corrected_duty2;
#if TRACK_SPEED_PID_ENABLED
    float measured_speed1 = 0.0f;
    float measured_speed2 = 0.0f;
    float correction1 = 0.0f;
    float correction2 = 0.0f;
#endif
#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
    if (!allowed
#if TRACK_ALGORITHM == 17
        && track17_uses_ring()
#endif
    ) {
        track_target_x = (uint32_t)line->target_x;
        track_target_confidence = line->target_confidence;
        track_valid_rows = line->valid_rows;
#if TRACK_ALGORITHM == 16
        if (track_arm_request == 1U && a16_frame_ms < TRACK_CAMERA_LOST_TIMEOUT_MS) {
            /* Enter only after a driven frame. Zero duty keeps startup and
             * an expired search stationary until a usable path returns. */
            if (road_search_remaining_ms == 0U && track_motor_duty > 0U) {
                road_search_remaining_ms = TRACK_A16_SEARCH_TIME_MS;
            } else if (road_search_remaining_ms > a16_frame_ms) {
                road_search_remaining_ms -= a16_frame_ms;
            } else {
                road_search_remaining_ms = 0U;
            }
            if (road_search_remaining_ms) {
                uint32_t duty = PWM_DUTY_MAX * TRACK_A16_SEARCH_PWM_PERCENT / 100U;
                if (duty > TRACK_MOTOR_DUTY_LIMIT) duty = TRACK_MOTOR_DUTY_LIMIT;
                if (duty > track_motor_duty + TRACK_MOTOR_RAMP_STEP)
                    duty = track_motor_duty + TRACK_MOTOR_RAMP_STEP;
                track_motor_duty = duty;
                track_drive_forward(duty * TRACK_MOTOR1_OUTPUT_SCALE_PERCENT / 100U,
                                    duty * TRACK_MOTOR2_OUTPUT_SCALE_PERCENT / 100U);
            } else {
                track_stop_outputs(0);
            }
            return;
        }
#endif
        track_stop();
        return;
    }
#endif
    if (angle < (int32_t)TRACK_SERVO_RIGHT_DEG) angle = (int32_t)TRACK_SERVO_RIGHT_DEG;
    if (angle > (int32_t)TRACK_SERVO_LEFT_DEG) angle = (int32_t)TRACK_SERVO_LEFT_DEG;
#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
    if (allowed && (track_mode16_phase==A16_ENTRY || track_mode16_phase==A16_ARC || track_mode16_phase==A16_MERGE))
        angle=(int32_t)TRACK_SERVO_CENTER_DEG + track_mode16_curve_command;
    /* Slew-limit noisy frame-to-frame commands. Keep straight-road corrections
     * gentle, but allow twice the rate when the traced path clearly bends. */
#if TRACK_ALGORITHM == 17
    if (track17_uses_ring())
#endif
    {
        int32_t previous = (int32_t)track_servo_angle;
        int32_t max_step = 1;
        if (line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
            line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE) {
            int32_t heading = (int32_t)line->target_x - (int32_t)line->anchor_x;
            if (heading < 0) heading = -heading;
            if (heading >= 3) max_step = 3;
            if (heading >= 6) max_step = 6;
#if TRACK_ALGORITHM == 16
            if (bend_support) max_step=8;
#endif
        }
        /* Unwind the existing turn promptly. For a large reversal, reach
         * neutral before applying the normal slew limit on the other side. */
        const int32_t neutral = (int32_t)TRACK_SERVO_CENTER_DEG;
        if ((previous > neutral && angle < previous) ||
            (previous < neutral && angle > previous)) {
            if ((previous > neutral && angle < neutral) ||
                (previous < neutral && angle > neutral))
                angle = neutral;
#if TRACK_ALGORITHM == 17
            max_step = track_mode16_phase == A16_ROAD ? 10 : 6;
#else
            /* Test faster unloading of an established ordinary-road turn. */
            const int32_t held = previous > neutral ? previous - neutral : neutral - previous;
            max_step = track_mode16_phase == A16_ROAD && held >= 6 &&
                       !track_mode16_obstacle_side &&
                       line->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
                       line->target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE ? 10 : 6;
#endif
        }
#if TRACK_ALGORITHM == 16
        if (track_mode16_corner_active) max_step = 15;
#endif
        if (angle > previous + max_step) angle = previous + max_step;
        else if (angle < previous - max_step) angle = previous - max_step;
    }
#endif
    if (angle < (int32_t)TRACK_SERVO_RIGHT_DEG) angle = TRACK_SERVO_RIGHT_DEG;
    if (angle > (int32_t)TRACK_SERVO_LEFT_DEG) angle = TRACK_SERVO_LEFT_DEG;
    track_servo_angle = (uint32_t)angle;
    pwm_duty(TRACK_SERVO_PWM, TRACK_SERVO_DUTY((uint32_t)angle));
    track_target_x = (uint32_t)line->target_x;
    track_target_confidence = line->target_confidence;
    track_valid_rows = line->valid_rows;
    if (!allowed ||
#if TRACK_ALGORITHM != 8 && TRACK_ALGORITHM != 9 && TRACK_ALGORITHM != 10 && TRACK_ALGORITHM != 11 && TRACK_ALGORITHM != 12 && TRACK_ALGORITHM != 13 && TRACK_ALGORITHM != 14
        line->anchor_confidence < TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE ||
        line->target_confidence < TRACK_DRIVE_MIN_TARGET_CONFIDENCE ||
        line->valid_rows < TRACK_DRIVE_MIN_VALID_ROWS ||
#endif
        (TRACK_SPEED_PID_ENABLED && TRACK_SPEED_TARGET_COUNTS_PER_10MS == 0U))
    {
        track_stop();
        return;
    }

#if TRACK_ALGORITHM == 16
    road_search_remaining_ms = 0U;
#endif
    /* Open-loop PWM: one shared ramped command, then an independent per-wheel
     * scale/trim. The configurable global cap may use the full PWM range. */
    /* Linear slowdown, normalized independently to each steering endpoint.
     * Zero at neutral, configured maximum reduction at full steering lock. */
    turn_amount = angle - (int32_t)TRACK_SERVO_CENTER_DEG;
    if (turn_amount < 0)
    {
        turn_amount = -turn_amount;
        turn_span = (int32_t)TRACK_SERVO_CENTER_DEG - (int32_t)TRACK_SERVO_RIGHT_DEG;
    }
    else
    {
        turn_span = (int32_t)TRACK_SERVO_LEFT_DEG - (int32_t)TRACK_SERVO_CENTER_DEG;
    }
    uint32_t slowdown_percent = TRACK_CURVE_SLOWDOWN_PERCENT;
#if TRACK_ALGORITHM == 17
    const int road_control = !track17_uses_ring();
    if (road_control) slowdown_percent = TRACK_A17_ROAD_SLOWDOWN_PERCENT;
#endif
    if (turn_span > 0)
        speed_scale_x100 -= (slowdown_percent * 100U *
                             (uint32_t)turn_amount) / (uint32_t)turn_span;
#if TRACK_ALGORITHM == 16
    speed_scale_x100 = track16_exit_scale(speed_scale_x100, turn_amount);
#endif
    /* Smooth steering-angle noise so curve slowdown cannot pulse wheel output
     * between straight and corner limits on successive camera frames. */
#if TRACK_ALGORITHM == 17
    if (road_control)
        track_curve_scale_x100 = speed_scale_x100;
    else
#endif
    if (speed_scale_x100 < track_curve_scale_x100)
    {
        speed_scale_delta = track_curve_scale_x100 - speed_scale_x100;
        track_curve_scale_x100 -= (speed_scale_delta + 1U) / 2U;
    }
    else if (speed_scale_x100 > track_curve_scale_x100)
    {
        speed_scale_delta = speed_scale_x100 - track_curve_scale_x100;
        track_curve_scale_x100 += (speed_scale_delta + 2U) / 3U;
    }
    speed_scale_x100 = track_curve_scale_x100;
    curve_duty_limit = (((PWM_DUTY_MAX * pwm_cap_percent) / 100U) * speed_scale_x100) / 10000U;
    if (curve_duty_limit == 0U)
    {
        track_stop();
        return;
    }
    /* Open-loop motors can stall when the curve slowdown takes the command
     * below their starting torque. Preserve the 25% global cap, but apply an
     * 18% minimum moving duty while drive is authorized. State-specific caps
     * (such as recovery) remain authoritative. */
#if TRACK_ALGORITHM == 17
    if (!road_control)
#endif
    {
        uint32_t floor_percent = TRACK_MOTOR_MIN_MOVING_PERCENT;
        uint32_t floor_duty;
        if (floor_percent > pwm_cap_percent) floor_percent = pwm_cap_percent;
        floor_duty = (PWM_DUTY_MAX * floor_percent) / 100U;
        if (curve_duty_limit < floor_duty) curve_duty_limit = floor_duty;
    }
    base_duty = track_motor_duty + TRACK_MOTOR_RAMP_STEP;
    if (base_duty > curve_duty_limit) base_duty = curve_duty_limit;
    track_motor_duty = base_duty;

    corrected_duty1 = (int32_t)((base_duty * TRACK_MOTOR1_OUTPUT_SCALE_PERCENT) / 100U);
    corrected_duty2 = (int32_t)((base_duty * TRACK_MOTOR2_OUTPUT_SCALE_PERCENT) / 100U);
#if TRACK_SPEED_PID_ENABLED
    correction1 = track_speed_pid_update(&track_speed_pid_m1,
                                         qtimer_quad_get(QTIMER_1, QTIMER1_TIMER0_C0),
                                         (float)TRACK_SPEED_TARGET_COUNTS_PER_10MS *
                                             (float)speed_scale_x100 / 10000.0f,
                                         DWT->CYCCNT,
                                         &measured_speed1);
    correction2 = track_speed_pid_update(&track_speed_pid_m2,
                                         qtimer_quad_get(QTIMER_1, QTIMER1_TIMER2_C2),
                                         (float)TRACK_SPEED_TARGET_COUNTS_PER_10MS *
                                             (float)speed_scale_x100 / 10000.0f,
                                         DWT->CYCCNT,
                                         &measured_speed2);
    track_encoder1_speed_counts_per_10ms_x10 = (int32_t)(measured_speed1 * 10.0f);
    track_encoder2_speed_counts_per_10ms_x10 = (int32_t)(measured_speed2 * 10.0f);
    corrected_duty1 += (int32_t)correction1;
    corrected_duty2 += (int32_t)correction2;
#endif
    if (corrected_duty1 < 0) corrected_duty1 = 0;
    if (corrected_duty2 < 0) corrected_duty2 = 0;
    if (corrected_duty1 > (int32_t)PWM_DUTY_MAX) corrected_duty1 = (int32_t)PWM_DUTY_MAX;
    if (corrected_duty2 > (int32_t)PWM_DUTY_MAX) corrected_duty2 = (int32_t)PWM_DUTY_MAX;
    duty1 = (uint32_t)corrected_duty1;
    duty2 = (uint32_t)corrected_duty2;

    track_drive_forward(duty1, duty2);
}

void main(void)
{
    uint16 x, y;
    uint8 (*frame)[MT9V03X_CSI_W];
    uint32_t last_frame_cycle;
    track_result_t line;
#if TRACK_ALGORITHM == 3
    track_mode4_element_t track_element;
    uint8_t track_element_confidence;
#endif
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
    uint32_t frame_dt_ms;
    uint32_t now_cycle;
    uint32_t cycles_per_ms;
    uint8_t camera_fault = 0U;
#endif

    DisableGlobalIRQ();
    board_init();
    track_outputs_init();
    TRACK_MODE_RESET();
    gpio_init(RTS_PIN, GPI, 0, GPIO_PIN_CONFIG);
    uart_init(WIRELESS_UART, WIRELESS_UART_BAUD, WIRELESS_UART_TX, WIRELESS_UART_RX);

    systick_delay_ms(300);
    lcd_init();
    EnableGlobalIRQ(0);
    /* Keep the existing image asset and show it while the sensor starts. */
    lcd_displayimage8660_zoom((uint16 *)character_img, 128, 160, 128, 160);

    camera_test_stage = 1U;
    mt9v03x_csi_init();
    EnableGlobalIRQ(0);
    camera_test_stage = 2U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    last_frame_cycle = DWT->CYCCNT;
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
    cycles_per_ms = SystemCoreClock / 1000U;
    if (cycles_per_ms == 0U) cycles_per_ms = 1U;
#endif

    while (1)
    {
        ++track_heartbeat;
        wireless_poll();
        if (!mt9v03x_csi_finish_flag)
        {
            uint32_t timeout_ms = TRACK_CAMERA_LOST_TIMEOUT_MS;
#if TRACK_ALGORITHM == 16
            if (road_search_remaining_ms && road_search_remaining_ms < timeout_ms) timeout_ms = road_search_remaining_ms;
#endif
            if ((uint32_t)(DWT->CYCCNT - last_frame_cycle) >=
                (SystemCoreClock / 1000U) * timeout_ms)
            {
                track_stop();
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
                if (!camera_fault)
                {
                    TRACK_MODE_SUSPEND(FSM_REASON_CAMERA);
                    track_fsm_state = FSM_WAIT;
                    track_fsm_reason = FSM_REASON_CAMERA;
                }
                camera_fault = 1U;
#endif
            }
            continue;
        }

        DisableGlobalIRQ();
        mt9v03x_csi_finish_flag = 0U;
        frame = mt9v03x_csi_image;
        EnableGlobalIRQ(0);

        /* Accept only the two DMA buffers owned by the CSI driver. */
        if ((frame != mt9v03x_csi1_image) && (frame != mt9v03x_csi2_image))
        {
            continue;
        }

        DCACHE_InvalidateByRange((uint32_t)frame,
                                 MT9V03X_CSI_W * MT9V03X_CSI_H);
        for (y = 0U; y < MT9V03X_CSI_H; ++y)
        {
            for (x = 0U; x < MT9V03X_CSI_W; ++x)
            {
                camera_test_image[y][x] = frame[y][x];
            }
        }

        camera_test_frame_min = 255U;
        camera_test_frame_max = 0U;
        for (y = 0U; y < MT9V03X_CSI_H; ++y)
        {
            for (x = 0U; x < MT9V03X_CSI_W; ++x)
            {
                uint8 pixel = camera_test_image[y][x];
                if (pixel < camera_test_frame_min) camera_test_frame_min = pixel;
                if (pixel > camera_test_frame_max) camera_test_frame_max = pixel;
            }
        }
        camera_test_pixel_60_94 = camera_test_image[60U][94U];
        ++camera_test_frames;
        camera_test_stage = 3U;
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
        now_cycle = DWT->CYCCNT;
        frame_dt_ms = (now_cycle - last_frame_cycle) / cycles_per_ms;
        if (frame_dt_ms == 0U) frame_dt_ms = 1U;
        last_frame_cycle = now_cycle;
        camera_fault = 0U;
        track_fsm_processing_cycles = 0U;
        now_cycle = DWT->CYCCNT;
        line = TRACK_MODE_PROCESS_EX(camera_test_image, frame_dt_ms, track_arm_request == 1U);
        track_fsm_processing_cycles = DWT->CYCCNT - now_cycle;
        track_fsm_diagnostics = line;
        track_fsm_state = line.state;
        track_fsm_reason = line.reason;
#else
#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
        track_mode16_set_frame_dt((DWT->CYCCNT - last_frame_cycle) / (SystemCoreClock / 1000U));
#endif
        last_frame_cycle = DWT->CYCCNT;
        line = TRACK_MODE_PROCESS(camera_test_image);
#endif
        track_control(&line);

        /* Show the active algorithm's binary segmentation, then its centerline. */
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
        TRACK_MODE_BINARY_PREVIEW(camera_test_image, camera_test_display_image,
                                         line.global_threshold);
#elif TRACK_ALGORITHM == 3
        track_mode4_binary_preview(camera_test_image, camera_test_display_image);
        track_element = track_mode4_detect_element(
            (const uint8_t (*)[TRACK_MODE4_WIDTH])camera_test_display_image,
            &track_element_confidence);
#elif TRACK_ALGORITHM == 17
        if (track17_uses_ring() || track17_corner_active)
            track_mode16_binary_preview(camera_test_image, camera_test_display_image, line.global_threshold);
        else
            track_mode4_binary_preview(camera_test_image, camera_test_display_image);
#elif TRACK_ALGORITHM == 16
        TRACK_MODE_BINARY_PREVIEW(camera_test_image, camera_test_display_image,
                                  line.global_threshold);
#else
        track_mode1_binary_preview(camera_test_image, camera_test_display_image,
                                   line.global_threshold);
#endif
#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
        for (y = 16U; y < MT9V03X_CSI_H - 3U; ++y)
#elif TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
        for (y = 20U; y < MT9V03X_CSI_H - 3U; ++y)
#else
        for (y = 48U; y < MT9V03X_CSI_H - 3U; ++y)
#endif
        {
            if (TRACK_MODE_PATH_CONF[y] >= 25U)
            {
                int16_t px = TRACK_MODE_PATH_X[y];
                if (px > 1 && px < (int16_t)MT9V03X_CSI_W - 2)
                {
                    camera_test_display_image[y][px - 1] = 0U;
                    camera_test_display_image[y][px] = 0U;
                    camera_test_display_image[y][px + 1] = 0U;
                }
            }
        }

#if TRACK_ALGORITHM == 11 || TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
        /* Cross marks the actual steering target, including ordinary tracking. */
        {
            int tx = line.target_x;
            #if TRACK_ALGORITHM == 12
            int ty = (int)track_path12_lookahead_row;
#elif TRACK_ALGORITHM == 15
            int ty = (int)track_ring15_lookahead_row;
#elif (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14)
            int ty = (int)track_ring13_lookahead_row;
#else
            int ty = (int)track_ring11_lookahead_row;
#endif
            int k;
            if (tx >= 5 && tx < MT9V03X_CSI_W - 5 && ty >= 5 && ty < MT9V03X_CSI_H - 5)
                for (k = -4; k <= 4; ++k) {
                    camera_test_display_image[ty][tx + k] = 0U;
                    camera_test_display_image[ty + k][tx] = 0U;
                }
        }
#endif
#if TRACK_ALGORITHM == 14
        {
            uint16_t road_color;
            if (track_ring14_display_phase == 2U)
                road_color = 0xF800U; /* red: ring execution until merge completes */
            else if (track_ring14_display_phase == 1U || line.state == FSM_RING_CANDIDATE)
                road_color = 0xFFE0U; /* yellow: latched preparation */
            else road_color = 0x07E0U; /* green: ordinary tracking */
            /* The LCD helper swaps each 16-bit word before sending it.
             * Pre-swap logical RGB565 so green/yellow/red keep their hue. */
            road_color = (uint16_t)((road_color << 8) | (road_color >> 8));
            for (y = 0U; y < MT9V03X_CSI_H; ++y)
                for (x = 0U; x < MT9V03X_CSI_W; ++x)
                    camera_test_debug_rgb565[y][x] = camera_test_display_image[y][x] ? road_color : 0x0000U;
            if (track_ring14_center_visible) {
                int cx = track_ring14_center_x, cy = track_ring14_center_y, k;
                uint16_t center_color = track_ring14_center_predicted ? 0xFF07U : 0xFFFFU;
                for (k = -5; k <= 5; ++k) {
                    int px = cx + k, py = cy + k;
                    if (px >= 0 && px < MT9V03X_CSI_W && cy >= 0 && cy < MT9V03X_CSI_H)
                        camera_test_debug_rgb565[cy][px] = center_color;
                    if (cx >= 0 && cx < MT9V03X_CSI_W && py >= 0 && py < MT9V03X_CSI_H)
                        camera_test_debug_rgb565[py][cx] = center_color;
                }
            }
        }
        lcd_displayimage8660_zoom(&camera_test_debug_rgb565[0][0],
                                  MT9V03X_CSI_W, MT9V03X_CSI_H, 128U, 160U);
#elif TRACK_ALGORITHM == 15
        {
            int16_t rows[3]={track_ring15_lower_row,track_ring15_middle_row,track_ring15_upper_row};
            int16_t xs[3]={track_ring15_lower_x,track_ring15_middle_x,track_ring15_upper_x};
            int i;
            for(i=0;i<3;i++)if(rows[i]>=0&&rows[i]<MT9V03X_CSI_H&&xs[i]>=0&&xs[i]<MT9V03X_CSI_W){
                int k;for(k=-3;k<=3;k++){
                    if(xs[i]+k>=0&&xs[i]+k<MT9V03X_CSI_W)camera_test_display_image[rows[i]][xs[i]+k]=255U;
                    if(rows[i]+k>=0&&rows[i]+k<MT9V03X_CSI_H)camera_test_display_image[rows[i]+k][xs[i]]=255U;
                }
            }
        }
        lcd_displayimage032_zoom(&camera_test_display_image[0][0],
                                 MT9V03X_CSI_W, MT9V03X_CSI_H, 128U, 160U);
#else
        lcd_displayimage032_zoom(&camera_test_display_image[0][0],
                                 MT9V03X_CSI_W, MT9V03X_CSI_H, 128U, 160U);
#endif
#if TRACK_ALGORITHM == 8
        {
            char state_text[17];
            snprintf(state_text, sizeof(state_text), "%s E%d",
                     TRACK_MODE_STATE_NAME(line.state),
                     (int)track_fusion8_error);
            lcd_showstr(0, 0, (const int8 *)"                ");
            lcd_showstr(0, 0, (const int8 *)state_text);
        }
#elif TRACK_ALGORITHM == 12 || (TRACK_ALGORITHM == 13 || TRACK_ALGORITHM == 14 || TRACK_ALGORITHM == 15)
        {
            char text[17];
            snprintf(text,sizeof(text),"A%u %s",(unsigned)TRACK_ALGORITHM,TRACK_MODE_STATE_NAME(line.state));
            lcd_showstr(0,0,(const int8 *)"                ");lcd_showstr(0,0,(const int8 *)text);
#if TRACK_ALGORITHM == 12
            snprintf(text,sizeof(text),"N%lu Y%lu %ld",(unsigned long)track_path12_path_rows,(unsigned long)track_path12_lookahead_row,(long)track_path12_side);
#elif TRACK_ALGORITHM == 15
            snprintf(text,sizeof(text),"S%u SIDE%ld",(unsigned)track_ring15_stage,(long)track_ring15_side);
#else
            snprintf(text,sizeof(text),"P%lu Y%lu %ld",(unsigned long)track_ring13_phase,(unsigned long)track_ring13_lookahead_row,(long)track_ring13_side);
#endif
            lcd_showstr(0,1,(const int8 *)"                ");lcd_showstr(0,1,(const int8 *)text);
        }
#elif TRACK_ALGORITHM == 11
        {
            char text[17];
            const char *candidate = track_ring11_candidate_side < 0 ? "L?" :
                                    (track_ring11_candidate_side > 0 ? "R?" : "--");
            snprintf(text,sizeof(text),"A11 %s %s",TRACK_MODE_STATE_NAME(line.state),candidate);
            lcd_showstr(0,0,(const int8 *)"                ");
            lcd_showstr(0,0,(const int8 *)text);
            snprintf(text,sizeof(text),"Q%u Y%lu G%ld",(unsigned)line.target_confidence,
                     (unsigned long)track_ring11_lookahead_row,(long)track_ring11_shape_side);
            lcd_showstr(0,1,(const int8 *)"                ");
            lcd_showstr(0,1,(const int8 *)text);
        }
#elif TRACK_ALGORITHM == 9
        {
            char state_text[17];
            snprintf(state_text, sizeof(state_text), "A9 %s",
                     TRACK_MODE_STATE_NAME(line.state));
            lcd_showstr(0, 0, (const int8 *)"                ");
            lcd_showstr(0, 0, (const int8 *)state_text);
        }
#elif TRACK_ALGORITHM == 10
        {
            char state_text[17];
            snprintf(state_text, sizeof(state_text), "A10 %s",
                     TRACK_MODE_STATE_NAME(line.state));
            lcd_showstr(0, 0, (const int8 *)"                ");
            lcd_showstr(0, 0, (const int8 *)state_text);
        }
#elif TRACK_ALGORITHM == 3
        {
            char element_text[17];
            snprintf(element_text, sizeof(element_text), "%s %u",
                     track_mode4_element_name(track_element),
                     (unsigned)track_element_confidence);
            lcd_showstr(0, 0, (const int8 *)"                ");
            lcd_showstr(0, 0, (const int8 *)element_text);
        }
#elif TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
        {
            char text[17];
            static const char *phase_names[] = {"ROAD", "PREP", "ENTRY", "ARC", "MERGE", "WAIT"};
            snprintf(text, sizeof(text), "%u %s %c Y%ld", (unsigned)TRACK_ALGORITHM,
#if TRACK_ALGORITHM == 16
                     track16_lap == A16_LAP_FINISHED ? "FINISH" :
#endif
                     phase_names[track_mode16_phase <= A16_WAIT ? track_mode16_phase : A16_WAIT],
                     track_mode16_island_side == 0 ? '-' : (track_mode16_island_side < 0 ? 'L' : 'R'),
                     (long)track_mode16_island_bottom);
            lcd_showstr(0, 0, (const int8 *)"                ");
            lcd_showstr(0, 0, (const int8 *)text);
        }
#endif
    }
}
