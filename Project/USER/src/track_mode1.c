#include "track_mode1.h"
#include "tune_config.h"
#include <string.h>

/* OtsuScan baseline: one Otsu threshold over configured top..118, then choose the
 * white interval nearest the previous row's center while scanning upward.
 * Deliberately has no local thresholding, gap closing, width prior or edge test. */
static uint32_t histogram[256];
int16_t track_mode1_path_x[TRACK_MODE1_HEIGHT];
uint8_t track_mode1_path_confidence[TRACK_MODE1_HEIGHT];

void track_mode1_reset(void)
{
    memset(track_mode1_path_x, 0, sizeof(track_mode1_path_x));
    memset(track_mode1_path_confidence, 0, sizeof(track_mode1_path_confidence));
}

static uint8_t otsu_threshold(
    const uint8_t image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH])
{
    uint32_t total = 0U, weighted_total = 0U;
    uint32_t background = 0U, weighted_background = 0U;
    float best_score = -1.0f;
    uint8_t best_threshold = 0U;
    uint16_t x, y, t;

    memset(histogram, 0, sizeof(histogram));
    for (y = (uint16_t)TRACK_OTSUSCAN_SCAN_TOP_ROW; y <= TRACK_MODE1_BOTTOM_ROW; ++y) {
        for (x = 0U; x < TRACK_MODE1_WIDTH; ++x) {
            const uint8_t value = image[y][x];
            ++histogram[value];
            ++total;
            weighted_total += value;
        }
    }

    for (t = 0U; t < 256U; ++t) {
        float mean_background, mean_foreground, difference, score;
        background += histogram[t];
        weighted_background += (uint32_t)t * histogram[t];
        if (background == 0U || background == total) continue;

        mean_background = (float)weighted_background / (float)background;
        mean_foreground = (float)(weighted_total - weighted_background) /
                          (float)(total - background);
        difference = mean_background - mean_foreground;
        score = (float)background * (float)(total - background) *
                difference * difference;
        if (score > best_score) {
            best_score = score;
            best_threshold = (uint8_t)t;
        }
    }
    return best_threshold;
}

void track_mode1_binary_preview(
    const uint8_t input[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH],
    uint8_t output[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH], uint8_t threshold)
{
    uint16_t y, x;
    for (y = 0U; y < TRACK_MODE1_HEIGHT; ++y) {
        for (x = 0U; x < TRACK_MODE1_WIDTH; ++x) {
            output[y][x] = input[y][x] > threshold ? 255U : 0U;
        }
    }
}

track_mode1_result_t track_mode1_process(
    const uint8_t image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH])
{
    track_mode1_result_t result = {94, 94, 0U, 0U, 0U, 0U};
    uint8_t threshold = otsu_threshold(image);
    int32_t seed_twice = 2 * (TRACK_MODE1_WIDTH / 2U);
    int32_t y;

    result.global_threshold = threshold;
    memset(track_mode1_path_confidence, 0, sizeof(track_mode1_path_confidence));

    for (y = (int32_t)TRACK_MODE1_BOTTOM_ROW; y >= (int32_t)TRACK_OTSUSCAN_SCAN_TOP_ROW; --y) {
        int32_t x = 0;
        int32_t best_distance = 0x7fffffff;
        int32_t best_left = -1, best_right = -1;

        while (x < (int32_t)TRACK_MODE1_WIDTH) {
            int32_t left, right, run_length, interval_distance;
            while (x < (int32_t)TRACK_MODE1_WIDTH && image[y][x] <= threshold) ++x;
            left = x;
            while (x < (int32_t)TRACK_MODE1_WIDTH && image[y][x] > threshold) ++x;
            right = x - 1;
            run_length = right - left + 1;
            if (run_length < (int32_t)TRACK_OTSUSCAN_MIN_RUN_PIXELS) continue;

            if (seed_twice < 2 * left) interval_distance = 2 * left - seed_twice;
            else if (seed_twice > 2 * right) interval_distance = seed_twice - 2 * right;
            else interval_distance = 0;

            if (interval_distance < best_distance) {
                best_distance = interval_distance;
                best_left = left;
                best_right = right;
            }
        }

        if (best_left < 0) continue;
        seed_twice = best_left + best_right;
        track_mode1_path_x[y] = (int16_t)(seed_twice / 2);
        track_mode1_path_confidence[y] =
            (best_left > 0 && best_right < (int32_t)TRACK_MODE1_WIDTH - 1) ? 100U : 35U;
        ++result.valid_rows;

        if (y == (int32_t)TRACK_OTSUSCAN_ANCHOR_ROW) {
            result.anchor_x = track_mode1_path_x[y];
            result.anchor_confidence = track_mode1_path_confidence[y];
        }
        if (y == (int32_t)TRACK_OTSUSCAN_TARGET_ROW) {
            result.target_x = track_mode1_path_x[y];
            result.target_confidence = track_mode1_path_confidence[y];
        }
    }
    return result;
}
