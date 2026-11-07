#ifndef TRACK_RING13_H
#define TRACK_RING13_H
#include "track_fsm.h"
extern int16_t track_ring13_path_x[120];
extern uint8_t track_ring13_path_confidence[120];
extern volatile uint32_t track_ring13_phase,track_ring13_lookahead_row;
extern volatile int32_t track_ring13_side,track_ring13_left_length,track_ring13_right_length;
extern volatile uint32_t track_ring13_jump_count;
extern volatile int32_t track_ring13_jump_from,track_ring13_jump_to,track_ring13_observed_bend;
void track_ring13_reset(void);
void track_ring13_suspend(uint8_t reason);
track_fsm_result_t track_ring13_process(const uint8_t image[120][188],uint32_t dt,uint8_t run);
void track_ring13_binary_preview(const uint8_t input[120][188],uint8_t output[120][188],uint8_t t);
const char *track_ring13_state_name(uint8_t state);
#endif
