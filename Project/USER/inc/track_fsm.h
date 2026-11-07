#ifndef TRACK_FSM_INCLUDED
#define TRACK_FSM_INCLUDED
#include <stdint.h>
#define TRACK_FSM_W 188
#define TRACK_FSM_H 120

typedef enum {
    FSM_WAIT, FSM_NORMAL, FSM_CROSS_CANDIDATE, FSM_CROSS_PASS,
    FSM_CROSS_MERGE, FSM_RING_CANDIDATE, FSM_RING_ENTER,
    FSM_RING_INSIDE, FSM_RING_EXIT, FSM_RING_MERGE, FSM_RECOVER
} track_fsm_state_t;
typedef enum {
    FSM_REASON_INIT, FSM_REASON_CONFIRMED, FSM_REASON_CROSS,
    FSM_REASON_RING, FSM_REASON_ENTRY, FSM_REASON_INTERIOR,
    FSM_REASON_EXIT_FAR, FSM_REASON_EXIT_NEAR, FSM_REASON_MERGED,
    FSM_REASON_LOST, FSM_REASON_RECOVERED, FSM_REASON_RECOVERY_TIMEOUT,
    FSM_REASON_ELEMENT_TIMEOUT, FSM_REASON_CAMERA, FSM_REASON_STOP,
    FSM_REASON_REJECTED, FSM_REASON_START_LINE
} track_fsm_reason_t;
/* provenance: 0=absent, 1=two observed edges, 2=one observed edge,
 * 3=element repair supported by observations, 4=temporal prediction. */
typedef struct {
    int16_t left, right, center;
    uint8_t left_real, right_real, confidence, provenance;
} track_fsm_row_t;
typedef struct {
    int16_t anchor_x, target_x, servo_cdeg;
    uint16_t recovery_remaining_ms;
    uint8_t anchor_confidence, target_confidence, valid_rows, global_threshold;
    uint8_t state, reason, drive_allowed, pwm_cap_percent;
    int8_t ring_side; /* -1=left, +1=right, 0=unknown */
    uint8_t lost_left, lost_right, split_rows, corner_left, corner_right;
    uint8_t observed_rows, near_open, far_open, interior_seen, exit_far_seen;
} track_fsm_result_t;
extern track_fsm_row_t track_fsm_rows[TRACK_FSM_H];
extern int16_t track_fsm_path_x[TRACK_FSM_H];
extern uint8_t track_fsm_path_confidence[TRACK_FSM_H];
void track_fsm_reset(void);
void track_fsm_suspend(uint8_t reason);
track_fsm_result_t track_fsm_process(const uint8_t image[120][188],
                                    uint32_t dt_ms, uint8_t run_enabled);
const char *track_fsm_state_name(uint8_t state);
#endif
