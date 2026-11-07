#ifndef TRACK_RING15_H
#define TRACK_RING15_H
#include "track_fsm.h"

/* Algorithm 15: corner-state roundabout planner. Ordinary roads use
 * Algorithm 3 directly; ring entry/exit uses image-space corner line pulls. */
extern int16_t track_ring15_path_x[120];
extern uint8_t track_ring15_path_confidence[120];
extern volatile uint32_t track_ring15_lookahead_row,track_ring15_loss_ms;
extern volatile int32_t track_ring15_side;
extern volatile int16_t track_ring15_lower_row,track_ring15_middle_row,track_ring15_upper_row;
extern volatile int16_t track_ring15_lower_x,track_ring15_middle_x,track_ring15_upper_x;
extern volatile uint8_t track_ring15_display_phase;
extern volatile uint8_t track_ring15_stage;

void track_ring15_reset(void);
void track_ring15_suspend(uint8_t reason);
track_fsm_result_t track_ring15_process(const uint8_t image[120][188],uint32_t dt_ms,uint8_t run_enabled);
void track_ring15_binary_preview(const uint8_t input[120][188],uint8_t output[120][188],uint8_t t);
const char *track_ring15_state_name(uint8_t state);
#endif
