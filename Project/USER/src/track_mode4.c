#include "track_mode4.h"
#include "tune_config.h"
#include <string.h>

/* Fixed-memory approximation of smartcar-vision-lab ContinuityTrack(temporal=False).
 * It uses the same global/local threshold blend, 5-pixel gap closing, width
 * prior and row-to-row continuity. Previous-frame smoothing is disabled for
 * algorithm 3. OpenCV's 3x3
 * median and subpixel edge repositioning are omitted to fit the on-chip RAM. */

static int16_t previous_center[TRACK_MODE4_HEIGHT];
static uint8_t previous_valid[TRACK_MODE4_HEIGHT];
static uint8_t row_mask[TRACK_MODE4_WIDTH];
static uint16_t histogram[256];
int16_t track_mode4_path_x[TRACK_MODE4_HEIGHT];
uint8_t track_mode4_path_confidence[TRACK_MODE4_HEIGHT];
static uint8_t element_pending;
static uint8_t element_pending_frames;
static track_mode4_element_t element_stable = TRACK_ELEMENT_NORMAL;
static uint8_t element_stable_score;

static int32_t absolute(int32_t value) { return value < 0 ? -value : value; }
static int32_t clamp(int32_t value, int32_t low, int32_t high)
{ return value < low ? low : (value > high ? high : value); }

void track_mode4_reset(void)
{
    memset(previous_valid, 0, sizeof(previous_valid));
    memset(previous_center, 0, sizeof(previous_center));
    memset(track_mode4_path_confidence, 0, sizeof(track_mode4_path_confidence));
    track_mode4_element_reset();
}

static uint8_t otsu_threshold(const uint8_t image[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH],
                              uint8_t *contrast)
{
    uint32_t total = 0, weighted = 0, background = 0, weighted_background = 0;
#if TRACK_ALGORITHM == 17
    /* Match the archived Oct 22 executable, including its 64-bit wrap. */
    uint64_t best = 0;
#else
    double best = 0.0;
#endif
    uint8_t low = 255, high = 0, best_t = 127;
    uint16_t y, x, t;
    memset(histogram, 0, sizeof(histogram));
    for (y = (uint16_t)TRACK_CONT_SCAN_TOP_ROW; y <= TRACK_MODE4_BOTTOM_ROW; ++y)
        for (x = 0; x < TRACK_MODE4_WIDTH; ++x) {
            uint8_t v = image[y][x];
            ++histogram[v];
            if (v < low) low = v;
            if (v > high) high = v;
        }
    *contrast = high - low;
    for (t = 0; t < 256; ++t) { total += histogram[t]; weighted += t * histogram[t]; }
    for (t = 0; t < 256; ++t) {
        int64_t numerator;
#if TRACK_ALGORITHM == 17
        uint64_t score;
#else
        double score;
#endif
        background += histogram[t];
        weighted_background += t * histogram[t];
        if (!background || background == total) continue;
        numerator = (int64_t)weighted_background * total - (int64_t)weighted * background;
#if TRACK_ALGORITHM == 17
        score = ((uint64_t)numerator * (uint64_t)numerator) /
                ((uint64_t)background * (total - background));
#else
        /* Convert before squaring: a valid 188x120 ROI exceeds int64_t.
         * Cortex-M7 FPv5-D16 evaluates the variance without integer overflow. */
        score = (double)numerator * (double)numerator /
                ((double)background * (total - background));
#endif
        if (score > best) { best = score; best_t = (uint8_t)t; }
    }
    return best_t;
}

static void threshold_row(const uint8_t *row, uint8_t global_t)
{
    uint16_t x, total = 0;
    uint8_t low = 0, high = 255;
    int32_t local_t, threshold;
    memset(histogram, 0, sizeof(histogram));
    for (x = 0; x < TRACK_MODE4_WIDTH; ++x) ++histogram[row[x]];
    for (x = 0; x < 256; ++x) { total += histogram[x]; if (total >= 19) { low = (uint8_t)x; break; } }
    total = 0;
    for (x = 0; x < 256; ++x) { total += histogram[x]; if (total >= 169) { high = (uint8_t)x; break; } }
    local_t = low + ((int32_t)TRACK_CONT_LOCAL_PERCENT * (high - low)) / 100;
    threshold = ((int32_t)TRACK_CONT_GLOBAL_WEIGHT_PERCENT * global_t +
                 (100 - (int32_t)TRACK_CONT_GLOBAL_WEIGHT_PERCENT) * local_t) / 100;
    for (x = 0; x < TRACK_MODE4_WIDTH; ++x) row_mask[x] = row[x] > threshold;
    /* Horizontal close of dark gaps up to five pixels. */
    for (x = 1; x + 1 < TRACK_MODE4_WIDTH;) {
        uint16_t start, end;
        if (row_mask[x] || !row_mask[x-1]) { ++x; continue; }
        start = x;
        while (x < TRACK_MODE4_WIDTH && !row_mask[x]) ++x;
        end = x;
        if (end < TRACK_MODE4_WIDTH && end - start <= (uint16_t)TRACK_CONT_GAP_CLOSE_PIXELS)
            for (x = start; x < end; ++x) row_mask[x] = 1;
        x = end;
    }
}

void track_mode4_binary_preview(
    const uint8_t input[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH],
    uint8_t output[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH])
{
    uint8_t contrast, global_t = otsu_threshold(input, &contrast);
    uint16_t y, x;
    (void)contrast;
    for (y = 0U; y < TRACK_MODE4_HEIGHT; ++y) {
        threshold_row(input[y], global_t);
        for (x = 0U; x < TRACK_MODE4_WIDTH; ++x) {
            output[y][x] = row_mask[x] ? 255U : 0U;
        }
    }
}

void track_mode4_element_reset(void)
{
    element_pending = TRACK_ELEMENT_NORMAL;
    element_pending_frames = 0U;
    element_stable = TRACK_ELEMENT_NORMAL;
    element_stable_score = 0U;
}

track_mode4_element_t track_mode4_detect_element(
    const uint8_t binary[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH],
    uint8_t *confidence)
{
    uint16_t y;
    uint8_t valid_rows = 0U, cross_rows = 0U, left_open_rows = 0U, right_open_rows = 0U;
    int16_t min_center = TRACK_MODE4_WIDTH, max_center = 0;
    uint8_t raw = TRACK_ELEMENT_NORMAL;
    uint8_t score = 0U;
    const uint16_t first_row = TRACK_CONT_SCAN_TOP_ROW + 2U;
    const uint16_t last_row = TRACK_MODE4_BOTTOM_ROW - 16U;

    for (y = first_row; y <= last_row; y += 2U) {
        int32_t center, left, right, expected_half, left_extent, right_extent;
        uint16_t probe;
        if (track_mode4_path_confidence[y] < 25U) continue;
        center = track_mode4_path_x[y];
        if (center < 2 || center > TRACK_MODE4_WIDTH - 3) continue;
        if (!binary[y][center]) {
            for (probe = 1U; probe <= 3U; ++probe) {
                int32_t left_probe = center - (int32_t)probe;
                int32_t right_probe = center + (int32_t)probe;
                if (left_probe >= 0 && binary[y][left_probe]) {
                    center = left_probe;
                    break;
                }
                if (right_probe < TRACK_MODE4_WIDTH && binary[y][right_probe]) {
                    center = right_probe;
                    break;
                }
            }
            if (!binary[y][center]) continue;
        }
        left = center;
        right = center;
        while (left > 0 && binary[y][left - 1]) --left;
        while (right + 1 < TRACK_MODE4_WIDTH && binary[y][right + 1]) ++right;
        expected_half = (int32_t)TRACK_CONT_WIDTH_PRIOR * (y + 10U) / 252U;
        if (expected_half < 5) expected_half = 5;
        left_extent = center - left;
        right_extent = right - center;
        if (track_mode4_path_x[y] < min_center) min_center = track_mode4_path_x[y];
        if (track_mode4_path_x[y] > max_center) max_center = track_mode4_path_x[y];
        ++valid_rows;
        if (left_extent * 100 > expected_half * 150 &&
            right_extent * 100 > expected_half * 150) {
            ++cross_rows;
        } else if (left_extent * 100 > expected_half * 170 &&
                   right_extent * 100 < expected_half * 135) {
            ++left_open_rows;
        } else if (right_extent * 100 > expected_half * 170 &&
                   left_extent * 100 < expected_half * 135) {
            ++right_open_rows;
        }
    }

    if (valid_rows >= 8U) {
        uint8_t center_shift = (uint8_t)(max_center - min_center);
        if (cross_rows >= 4U) {
            raw = TRACK_ELEMENT_CROSS_CANDIDATE;
            score = (uint8_t)clamp((int32_t)cross_rows * 15, 0, 100);
        } else if (left_open_rows >= 4U && left_open_rows >= right_open_rows + 3U &&
                   center_shift >= 5U) {
            raw = TRACK_ELEMENT_RING_LEFT_CANDIDATE;
            score = (uint8_t)clamp((int32_t)left_open_rows * 15 + center_shift, 0, 100);
        } else if (right_open_rows >= 4U && right_open_rows >= left_open_rows + 3U &&
                   center_shift >= 5U) {
            raw = TRACK_ELEMENT_RING_RIGHT_CANDIDATE;
            score = (uint8_t)clamp((int32_t)right_open_rows * 15 + center_shift, 0, 100);
        } else if (left_open_rows >= 3U || right_open_rows >= 3U) {
            raw = TRACK_ELEMENT_BRANCH_UNCERTAIN;
            score = (uint8_t)clamp((int32_t)(left_open_rows + right_open_rows) * 10, 0, 100);
        }
    }

    if (raw == element_pending) {
        if (element_pending_frames < 3U) ++element_pending_frames;
    } else {
        element_pending = raw;
        element_pending_frames = 1U;
    }
    if (element_pending_frames >= 3U) {
        element_stable = (track_mode4_element_t)element_pending;
        element_stable_score = score;
    }
    if (confidence) *confidence = element_stable_score;
    return element_stable;
}

const char *track_mode4_element_name(track_mode4_element_t element)
{
    switch (element) {
        case TRACK_ELEMENT_CROSS_CANDIDATE: return "CROSS?";
        case TRACK_ELEMENT_RING_LEFT_CANDIDATE: return "RING-L?";
        case TRACK_ELEMENT_RING_RIGHT_CANDIDATE: return "RING-R?";
        case TRACK_ELEMENT_BRANCH_UNCERTAIN: return "BRANCH?";
        default: return "NORMAL";
    }
}

track_mode4_result_t track_mode4_process(const uint8_t image[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH])
{
    track_mode4_result_t result = {94, 94, 0, 0, 0, 0};
    uint8_t contrast, global_t = otsu_threshold(image, &contrast);
    int32_t seed = 94, width = (int32_t)TRACK_CONT_WIDTH_PRIOR, slope = 0, missing = 0, have_track = 0;
    int32_t y;
    result.global_threshold = global_t;
    memset(track_mode4_path_confidence, 0, sizeof(track_mode4_path_confidence));
    if (contrast < TRACK_CONT_MIN_CONTRAST) { track_mode4_reset(); return result; }

    for (y = (int32_t)TRACK_MODE4_BOTTOM_ROW; y >= (int32_t)TRACK_CONT_SCAN_TOP_ROW; --y) {
        int32_t expected = clamp(width * (y + 10) / 126, 8, TRACK_MODE4_WIDTH);
        int32_t pred = seed - slope;
        int32_t best_score = 1000000, best_center = 0, best_l = 0, best_r = 0;
        int32_t best_clipped = 0, x;
        uint8_t confidence = 0;
        threshold_row(image[y], global_t);
        for (x = 0; x < TRACK_MODE4_WIDTH;) {
            int32_t l, r, rw, center, score, clipped;
            if (!row_mask[x]) { ++x; continue; }
            l = x;
            while (x < TRACK_MODE4_WIDTH && row_mask[x]) ++x;
            r = x - 1; rw = r - l + 1;
            if (rw < clamp(expected / 5, 5, TRACK_MODE4_WIDTH)) continue;
            clipped = (l == 0 || r == TRACK_MODE4_WIDTH - 1);
            center = (l + r) / 2;
            if (l == 0 && r != TRACK_MODE4_WIDTH - 1) center = clamp(r - expected/2, 0, 187);
            if (r == TRACK_MODE4_WIDTH - 1 && l != 0) center = clamp(l + expected/2, 0, 187);
            score = 100 * absolute(center - pred) / expected +
                    30 * absolute(rw - expected) / expected;
            if (l == 0 && r == TRACK_MODE4_WIDTH - 1) score += 100;
            /* Require a real dark/bright transition, rejecting flat shadows. */
            if (l > 4 && image[y][l+2] <= image[y][l-2] + 8) score += 25;
            if (r < 183 && image[y][r-2] <= image[y][r+2] + 8) score += 25;
            if (score < best_score) {
                best_score = score; best_center = center; best_l = l;
                best_r = r; best_clipped = clipped;
            }
        }
        if (best_score > 110) {
            ++missing;
            if (missing <= 5 && have_track) seed = clamp(pred, 0, 187);
            previous_valid[y] = 0;
            continue;
        }
        missing = 0; have_track = 1;
        slope = (7 * slope + 3 * (seed - best_center)) / 10;
        seed = best_center;
        if (!best_clipped) {
            int32_t measured = (best_r - best_l + 1) * 126 / (y + 10);
            width = (85 * width + 15 * measured) / 100;
        }
        confidence = best_clipped ? 45 : 95;
        confidence = (uint8_t)(confidence * clamp(100 - 60 * best_score / 100, 20, 100) / 100);
        previous_center[y] = (int16_t)best_center;
        previous_valid[y] = 1;
        track_mode4_path_x[y] = (int16_t)best_center;
        track_mode4_path_confidence[y] = confidence;
        if (confidence >= 25) ++result.valid_rows;
        if (y == (int32_t)TRACK_CONT_ANCHOR_ROW) { result.anchor_x = best_center; result.anchor_confidence = confidence; }
        if (y == (int32_t)TRACK_CONT_TARGET_ROW) { result.target_x = best_center; result.target_confidence = confidence; }
    }
    return result;
}
