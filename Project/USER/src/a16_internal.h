#ifndef A16_INTERNAL_H
#define A16_INTERNAL_H
#include "track_mode16.h"
#if TRACK_ALGORITHM == 16
#define A16_ROAD_TARGET_ROW 96U
/* Let the near lane guide entry until the side opening reaches the car. */
#define A16_CORNER_START_ROW 102
#else
#define A16_ROAD_TARGET_ROW 90U
#define A16_CORNER_START_ROW 84
#endif
#define A16_ROAD_ANCHOR_ROW 116U
#define A16_MAX_ROAD_WIDTH (TRACK_MODE1_WIDTH - 4)

/* Shared input/work buffers for one frame. Element state stays inside each module. */
extern uint8_t a16_binary_image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH];
extern uint8_t a16_binary_scratch[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH];
extern uint32_t a16_frame_ms;
void a16_ring_reset(void);
void a16_plan_island(track_mode1_result_t *road);
#if TRACK_ALGORITHM == 16
extern volatile uint32_t track_motor_duty;
extern unsigned a16_departure_mark_ms;
void a16_obstacle_reset(void);
int a16_plan_obstacle(track_mode1_result_t *road, const uint8_t image[][TRACK_MODE1_WIDTH]);
void a16_update_lap(const uint8_t image[][TRACK_MODE1_WIDTH], unsigned low, unsigned high);
void a16_join_departure_rails(void);
static inline int a16_projected_lane_width(int row)
{
    return (row - 31) * 19 / 10;
}

#endif
#endif
