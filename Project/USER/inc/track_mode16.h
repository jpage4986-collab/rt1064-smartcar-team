#ifndef TRACK_MODE16_H
#define TRACK_MODE16_H

#include "track_mode1.h"
#include "tune_config.h"
#ifndef TRACK_A16_MAX_STEER_DELTA_DEG
#define TRACK_A16_MAX_STEER_DELTA_DEG 15u
#endif

/* Algorithm 16: binary road tracing and an independent visible-island planner. */
enum { A16_ROAD, A16_PREP, A16_ENTRY, A16_ARC, A16_MERGE, A16_WAIT };
extern int track_mode16_curve_command;
extern int track_mode16_corner_active;
#if TRACK_ALGORITHM == 16
extern int track_mode16_obstacle_side;
extern int track_mode16_inside_px;
extern uint32_t a16_frame_ms;
extern int track_mode16_straight;
enum { A16_LAP_DEPART, A16_LAP_ARMED, A16_LAP_FINISHED };
extern unsigned track16_lap;
void track16_new_lap(void);
#endif
extern volatile uint32_t track_mode16_phase;
extern volatile int32_t track_mode16_island_side;
extern volatile int32_t track_mode16_island_bottom;
extern volatile uint32_t track_mode16_target_row;
#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
int track_mode16_corner(track_mode1_result_t *road, int weak_road);
#endif
void track_mode16_set_frame_dt(uint32_t dt_ms);
void track_mode16_reset(void);
track_mode1_result_t track_mode16_process(
    const uint8_t image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH]);
void track_mode16_binary_preview(
    const uint8_t input[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH],
    uint8_t output[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH], uint8_t unused_threshold);

extern int16_t track_mode16_path_x[TRACK_MODE1_HEIGHT];
extern uint8_t track_mode16_path_confidence[TRACK_MODE1_HEIGHT];

#endif
