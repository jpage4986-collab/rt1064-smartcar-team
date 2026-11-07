#ifndef TRACK_MODE1_H
#define TRACK_MODE1_H

#include <stdint.h>

#define TRACK_MODE1_WIDTH 188U
#define TRACK_MODE1_HEIGHT 120U
#define TRACK_MODE1_BOTTOM_ROW (TRACK_MODE1_HEIGHT - 2U)

typedef struct {
    int16_t anchor_x;
    int16_t target_x;
    uint8_t anchor_confidence;
    uint8_t target_confidence;
    uint8_t valid_rows;
    uint8_t global_threshold;
} track_mode1_result_t;

extern int16_t track_mode1_path_x[TRACK_MODE1_HEIGHT];
extern uint8_t track_mode1_path_confidence[TRACK_MODE1_HEIGHT];

void track_mode1_reset(void);
track_mode1_result_t track_mode1_process(
    const uint8_t image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH]);
void track_mode1_binary_preview(
    const uint8_t input[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH],
    uint8_t output[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH], uint8_t threshold);

#endif
