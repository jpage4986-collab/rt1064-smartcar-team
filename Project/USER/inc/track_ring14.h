#ifndef TRACK_RING14_H
#define TRACK_RING14_H
#include "track_fsm.h"
extern int16_t track_ring14_path_x[120];
extern uint8_t track_ring14_path_confidence[120];
extern volatile uint32_t track_ring14_lookahead_row,track_ring14_points,track_ring14_loss_ms,track_ring14_rejects;
/* 0=ordinary, 1=latched preparation, 2=ring execution until merge completes. */
extern volatile uint8_t track_ring14_display_phase;
extern volatile int16_t track_ring14_center_x,track_ring14_center_y;
extern volatile uint8_t track_ring14_center_visible;
extern volatile uint8_t track_ring14_center_predicted;
extern volatile uint32_t track_ring14_center_age_ms;
extern volatile uint32_t track_ring14_arc_degrees;
extern volatile uint8_t track_ring14_half_arc_candidate;
extern volatile int32_t track_ring14_side,track_ring14_forward_mm,track_ring14_lateral_mm,track_ring14_radius_mm,track_ring14_error_mm;
void track_ring14_reset(void);
void track_ring14_suspend(uint8_t reason);
track_fsm_result_t track_ring14_process(const uint8_t image[120][188],uint32_t dt,uint8_t run);
void track_ring14_binary_preview(const uint8_t input[120][188],uint8_t output[120][188],uint8_t t);
const char *track_ring14_state_name(uint8_t state);
#endif
