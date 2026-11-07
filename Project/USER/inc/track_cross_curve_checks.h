#ifndef TRACK_CROSS_CURVE_CHECKS_H
#define TRACK_CROSS_CURVE_CHECKS_H
#include "tune_config.h"
#if TRACK_SERVO_LEFT_DEG <= TRACK_SERVO_CENTER_DEG || TRACK_SERVO_CENTER_DEG <= TRACK_SERVO_RIGHT_DEG || TRACK_SERVO_LEFT_DEG > 180
#error "Servo limits must bracket center within 0..180"
#endif
#if TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11
#if TRACK_FSM_DETECT_TOP_ROW >= TRACK_CONT_SCAN_TOP_ROW || TRACK_CONT_SCAN_TOP_ROW > TRACK_CONT_TARGET_ROW || TRACK_CONT_TARGET_ROW >= TRACK_CONT_ANCHOR_ROW || TRACK_CONT_ANCHOR_ROW > 116
#error "FSM rows require detect_top < scan_top <= target < anchor <= 116"
#endif
#if TRACK_FSM_CONFIRM_FRAMES < 2 || TRACK_FSM_CONFIRM_MS < 1 || TRACK_FSM_PREVIOUS_PERCENT > 80
#error "Invalid confirmation or steering smoothing setting"
#endif
#if TRACK_FSM_SUPPORT_ROWS < 1 || TRACK_FSM_SUPPORT_ROWS > (117-TRACK_CONT_TARGET_ROW) || TRACK_DRIVE_MIN_VALID_ROWS > (117-TRACK_CONT_SCAN_TOP_ROW)
#error "Support requirement exceeds observed scan rows"
#endif
#if TRACK_FSM_RECOVERY_MS > TRACK_CAMERA_LOST_TIMEOUT_MS || TRACK_FSM_RECOVERY_MS < 1 || TRACK_CAMERA_LOST_TIMEOUT_MS > 2000 || TRACK_FSM_ELEMENT_TIMEOUT_MS > 60000
#error "Invalid recovery/camera/element timeout"
#endif
#endif /* camera-only configuration */
#if TRACK_CURVE_SLOWDOWN_PERCENT > 100 || TRACK_WHEEL_PWM_MAX_PERCENT > 100 || TRACK_FSM_ELEMENT_PWM_PERCENT > 100 || TRACK_FSM_RECOVERY_PWM_PERCENT > 25
#error "Invalid PWM or deceleration percentage"
#endif
#if (TRACK_ALGORITHM == 8 || TRACK_ALGORITHM == 9 || TRACK_ALGORITHM == 10 || TRACK_ALGORITHM == 11) && TRACK_SPEED_PID_ENABLED
#error "Algorithms 8, 9, and 10 are camera-only; disable encoder speed PID"
#endif
#if TRACK_CROSS_CURVE_WIDE_ROWS < 2 || TRACK_CROSS_CURVE_WIDE_ROWS > 30 || TRACK_CROSS_CURVE_WIDE_PERCENT < 110 || TRACK_CROSS_CURVE_WIDE_PERCENT > 250
#error "Invalid crossing width settings"
#endif
#endif
