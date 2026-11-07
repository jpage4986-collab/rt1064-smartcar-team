#ifndef TRACK_FUSION8_INCLUDED
#define TRACK_FUSION8_INCLUDED

#include "track_cross_curve.h"

void track_fusion8_reset(void);
void track_fusion8_suspend(uint8_t reason);
const char *track_fusion8_state_name(uint8_t state);
track_cross_curve_result_t track_fusion8_process(
    const uint8_t image[120][188], uint32_t dt_ms, uint8_t run_enabled);

extern volatile int32_t track_fusion8_error;
extern volatile int32_t track_fusion8_pd_output;

#endif
