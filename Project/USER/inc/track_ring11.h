#ifndef TRACK_RING11_H
#define TRACK_RING11_H
#include "track_fsm.h"
extern int16_t track_ring11_path_x[120];
extern uint8_t track_ring11_path_confidence[120];
extern volatile int32_t track_ring11_blob_side;
extern volatile uint32_t track_ring11_blob_bottom, track_ring11_blob_area;
extern volatile uint32_t track_ring11_entry_count,track_ring11_seen_states,track_ring11_lookahead_row;
extern volatile int32_t track_ring11_entry_row,track_ring11_candidate_side;
extern volatile int32_t track_ring11_shape_side,track_ring11_shape_score,track_ring11_shape_row;
void track_ring11_reset(void);
void track_ring11_suspend(uint8_t reason);
track_fsm_result_t track_ring11_process(const uint8_t image[120][188], uint32_t dt, uint8_t run);
void track_ring11_binary_preview(const uint8_t input[120][188],uint8_t output[120][188],uint8_t threshold);
const char *track_ring11_state_name(uint8_t state);
#endif
