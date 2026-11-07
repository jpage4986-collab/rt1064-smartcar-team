#ifndef TRACK_MODE4_H
#define TRACK_MODE4_H

#include <stdint.h>

#define TRACK_MODE4_WIDTH 188U
#define TRACK_MODE4_HEIGHT 120U
#define TRACK_MODE4_BOTTOM_ROW (TRACK_MODE4_HEIGHT - 2U)

typedef struct {
    int16_t anchor_x;
    int16_t target_x;
    uint8_t anchor_confidence;
    uint8_t target_confidence;
    uint8_t valid_rows;
    uint8_t global_threshold;
} track_mode4_result_t;

typedef enum {
    TRACK_ELEMENT_NORMAL = 0U,
    TRACK_ELEMENT_CROSS_CANDIDATE = 1U,
    TRACK_ELEMENT_RING_LEFT_CANDIDATE = 2U,
    TRACK_ELEMENT_RING_RIGHT_CANDIDATE = 3U,
    TRACK_ELEMENT_BRANCH_UNCERTAIN = 4U
} track_mode4_element_t;

extern int16_t track_mode4_path_x[TRACK_MODE4_HEIGHT];
extern uint8_t track_mode4_path_confidence[TRACK_MODE4_HEIGHT];

void track_mode4_reset(void);
track_mode4_result_t track_mode4_process(const uint8_t image[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH]);
void track_mode4_binary_preview(
    const uint8_t input[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH],
    uint8_t output[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH]);
void track_mode4_element_reset(void);
track_mode4_element_t track_mode4_detect_element(
    const uint8_t binary[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH],
    uint8_t *confidence);
const char *track_mode4_element_name(track_mode4_element_t element);

#endif
