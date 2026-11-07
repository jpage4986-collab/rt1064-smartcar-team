#ifndef TRACK_CROSS_CURVE_INCLUDED
#define TRACK_CROSS_CURVE_INCLUDED
#include "track_fsm.h"
/* Variant7 deliberately shares the diagnostic layout, not algorithm6 state. */
#define FSM_CROSS_CURVE 11u
#define FSM_START_STOP 12u
typedef track_fsm_row_t track_cross_curve_row_t;
typedef track_fsm_result_t track_cross_curve_result_t;
extern track_cross_curve_row_t track_cross_curve_rows[120];
extern int16_t track_cross_curve_path_x[120];
extern uint8_t track_cross_curve_path_confidence[120];
void track_cross_curve_reset(void);
void track_cross_curve_suspend(uint8_t reason);
track_cross_curve_result_t track_cross_curve_process(const uint8_t image[120][188], uint32_t dt_ms, uint8_t run_enabled);
void track_cross_curve_binary_preview(const uint8_t input[120][188],
                                      uint8_t output[120][188],
                                      uint8_t global_threshold);
const char *track_cross_curve_state_name(uint8_t state);
#endif
