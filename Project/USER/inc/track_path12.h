#ifndef TRACK_PATH12_H
#define TRACK_PATH12_H
#include "track_fsm.h"
extern int16_t track_path12_path_x[120];
extern uint8_t track_path12_path_confidence[120];
extern volatile uint32_t track_path12_lookahead_row,track_path12_path_rows;
extern volatile int32_t track_path12_side,track_path12_island_row;
void track_path12_reset(void);
void track_path12_suspend(uint8_t reason);
track_fsm_result_t track_path12_process(const uint8_t image[120][188],uint32_t dt,uint8_t run);
void track_path12_binary_preview(const uint8_t input[120][188],uint8_t output[120][188],uint8_t t);
const char *track_path12_state_name(uint8_t s);
#endif
