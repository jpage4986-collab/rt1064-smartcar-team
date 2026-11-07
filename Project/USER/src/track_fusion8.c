#include "track_cross_curve.h"
#include "track_fusion8.h"
#include "tune_config.h"
#include "track_cross_curve_checks.h"

/* Algorithm 8 hardware port.
 * The simulator's speed slider is not available on the car, so hardware uses
 * Rio's weighted-row PD during normal tracking and CrossCurve's image-only
 * FSM for junctions, rings, recovery, and the start/finish marker. */
volatile int32_t track_fusion8_error = 0;
volatile int32_t track_fusion8_pd_output = 0;

static int32_t previous_error;

static int32_t clip_i32(int32_t value, int32_t lo, int32_t hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

void track_fusion8_reset(void)
{
    previous_error = 0;
    track_fusion8_error = 0;
    track_fusion8_pd_output = 0;
    track_cross_curve_reset();
}

void track_fusion8_suspend(uint8_t reason)
{
    previous_error = 0;
    track_fusion8_error = 0;
    track_fusion8_pd_output = 0;
    track_cross_curve_suspend(reason);
}

const char *track_fusion8_state_name(uint8_t state)
{
    return track_cross_curve_state_name(state);
}

static int16_t rio_pd_servo_cdeg(void)
{
    int32_t weighted_sum = 0;
    int32_t valid_rows = 0;
    int32_t y;
    int32_t error;
    int32_t delta_limit = (int32_t)TRACK_FUSION_PD_DELTA_LIMIT;
    int32_t limited_error;
    int32_t kp_motion_x1000;
    int32_t kp;
    int32_t kd;
    int32_t pd;
    int32_t steer_x10000;
    int32_t span;
    int32_t angle;

    /* Same bands as rio2018.js::stepFromRows, scaled to the 188-pixel image. */
    for (y = 116; y >= (int32_t)TRACK_CONT_SCAN_TOP_ROW; --y) {
        const track_cross_curve_row_t *row = &track_cross_curve_rows[y];
        int32_t offset;
        int32_t magnitude;
        int32_t band_x1000;
        int32_t weighted_offset;
        if (row->confidence < 25U) continue;
        offset = (int32_t)row->center - (int32_t)TRACK_STEER_REFERENCE_X;
        magnitude = offset < 0 ? -offset : offset;
        if (magnitude <= 4) band_x1000 = 199;
        else if (magnitude <= 8) band_x1000 = 529;
        else if (magnitude <= 14) band_x1000 = 990;
        else if (magnitude <= 19) band_x1000 = 1390;
        else if (magnitude <= 32) band_x1000 = 1580;
        else if (magnitude <= 47) band_x1000 = 1880;
        else band_x1000 = 2500;
        weighted_offset = offset * band_x1000 / 1000;
        weighted_sum += weighted_offset;
        ++valid_rows;
    }

    if (valid_rows == 0) {
        track_fusion8_error = previous_error;
        track_fusion8_pd_output = 0;
        return (int16_t)(TRACK_SERVO_CENTER_DEG * 100U);
    }

    /* Simulator intentionally divides by N+1 to soften short row sets. */
    error = weighted_sum / (valid_rows + 1);
    limited_error = clip_i32(error, previous_error - delta_limit,
                              previous_error + delta_limit);
    kp_motion_x1000 = (int32_t)TRACK_FUSION_PD_KP_MOTION_X1000 *
                      (limited_error < 0 ? -limited_error : limited_error);
    kp_motion_x1000 = clip_i32(kp_motion_x1000, 0, 3500);
    kp = ((int32_t)TRACK_FUSION_PD_KP_BASE_X100 * 10 + kp_motion_x1000) *
         limited_error / 1000;
    kp = clip_i32(kp, -1200, 1200);
    kd = (int32_t)TRACK_FUSION_PD_KD_X100 *
         (limited_error - previous_error) / 100;
    kd = clip_i32(kd, -200, 200);
    pd = clip_i32(kp + kd, -1200, 1200);
    steer_x10000 = pd * 10000 / 1200;
    steer_x10000 = clip_i32(steer_x10000, -10000, 10000);

    if (steer_x10000 >= 0) {
        span = ((int32_t)TRACK_SERVO_CENTER_DEG - (int32_t)TRACK_SERVO_RIGHT_DEG) * 100;
        angle = (int32_t)TRACK_SERVO_CENTER_DEG * 100 - span * steer_x10000 / 10000;
    } else {
        span = ((int32_t)TRACK_SERVO_LEFT_DEG - (int32_t)TRACK_SERVO_CENTER_DEG) * 100;
        angle = (int32_t)TRACK_SERVO_CENTER_DEG * 100 + span * (-steer_x10000) / 10000;
    }
    previous_error = limited_error;
    track_fusion8_error = limited_error;
    track_fusion8_pd_output = pd;
    return (int16_t)clip_i32(angle, TRACK_SERVO_RIGHT_DEG * 100,
                             TRACK_SERVO_LEFT_DEG * 100);
}

track_cross_curve_result_t track_fusion8_process(
    const uint8_t image[120][188], uint32_t dt_ms, uint8_t run_enabled)
{
    track_cross_curve_result_t result =
        track_cross_curve_process(image, dt_ms, run_enabled);

    if (result.state == FSM_NORMAL && result.drive_allowed) {
        int32_t path_angle = result.servo_cdeg;
        int32_t pd_angle = rio_pd_servo_cdeg();
        result.servo_cdeg = (int16_t)(
            (pd_angle * (int32_t)TRACK_FUSION_PD_BLEND_PERCENT +
             path_angle * (100 - (int32_t)TRACK_FUSION_PD_BLEND_PERCENT)) / 100);
    } else {
        previous_error = 0;
        track_fusion8_error = 0;
        track_fusion8_pd_output = 0;
    }
    return result;
}
