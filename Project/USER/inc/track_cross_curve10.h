#ifndef TRACK_CROSS_CURVE10_INCLUDED
#define TRACK_CROSS_CURVE10_INCLUDED
#include "track_cross_curve.h"
extern track_cross_curve_row_t track_cross_curve10_rows[120];
extern int16_t track_cross_curve10_path_x[120];
extern uint8_t track_cross_curve10_path_confidence[120];
void track_cross_curve10_reset(void);
void track_cross_curve10_suspend(uint8_t reason);
track_cross_curve_result_t track_cross_curve10_process(const uint8_t image[120][188], uint32_t dt_ms, uint8_t run_enabled);
void track_cross_curve10_binary_preview(const uint8_t input[120][188], uint8_t output[120][188], uint8_t global_threshold);
const char *track_cross_curve10_state_name(uint8_t state);
#endif
