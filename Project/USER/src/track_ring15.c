/*
 * Algorithm 15: image-space roundabout state machine.
 *
 * Ordinary-road output is copied directly from ContinuityTrack (Algorithm 3).
 * Ring handling is independent: it thresholds with Algorithm 3's binary image,
 * extracts row runs and edge corners, then follows the article's staged
 * corner / line-pull sequence. No RGB, fitted circle, center tracking, or
 * Algorithm 14 state is read here.
 */
#include "track_ring15.h"
#include "track_mode4.h"
#include "tune_config.h"
#include <stdlib.h>
#include <string.h>

#define A15_FIRST_ROW 12
#define A15_LAST_ROW 116
#define A15_CORNER_MIN_DELTA 4
#define A15_NEAR_LOSS_ROWS 14
#define A15_CONFIRM_FRAMES 3U
#define A15_CORNER_CONFIRM_FRAMES 2U
#define A15_EXIT_CONFIRM_FRAMES 3U
#define A15_MERGE_CONFIRM_FRAMES 4U
#define A15_LINE_SEARCH_RADIUS 18

/* Article stages: 1/2 approach, 3 enter, 4 circulate, 5/6 exit,
 * 7 straighten and 8 merge back into ordinary tracking. */
enum {
    A15_NORMAL = 0,
    A15_APPROACH_STRAIGHT = 1,
    A15_APPROACH_CURVE = 2,
    A15_ENTER = 3,
    A15_CIRCULATE = 4,
    A15_EXIT_PULL = 5,
    A15_EXIT_TURN = 6,
    A15_STRAIGHTEN = 7,
    A15_MERGE = 8
};

typedef struct { int16_t row, x; uint8_t valid; } corner_t;
typedef struct {
    corner_t lower, middle, upper;
    uint8_t near_lost;
    uint8_t valid;
} side_features_t;
typedef struct {
    int16_t left, right;
    uint8_t valid, left_open, right_open;
} row_t;

static uint8_t binary[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH];
static row_t rows[TRACK_MODE4_HEIGHT];
static side_features_t selected_features;
static uint8_t stage, side, candidate_count, state_count;
static uint8_t exit_count, merge_count, display_phase;
static uint32_t loss_ms;
static corner_t remembered_upper, remembered_exit;

int16_t track_ring15_path_x[TRACK_MODE4_HEIGHT];
uint8_t track_ring15_path_confidence[TRACK_MODE4_HEIGHT];
volatile uint32_t track_ring15_lookahead_row = TRACK_CONT_TARGET_ROW;
volatile uint32_t track_ring15_loss_ms;
volatile int32_t track_ring15_side;
volatile int16_t track_ring15_lower_row = -1, track_ring15_middle_row = -1;
volatile int16_t track_ring15_upper_row = -1;
volatile int16_t track_ring15_lower_x = -1, track_ring15_middle_x = -1;
volatile int16_t track_ring15_upper_x = -1;
volatile uint8_t track_ring15_display_phase, track_ring15_stage;

static int clamp_int(int v, int lo, int hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

static int edge_at(int y, int ring_side)
{
    if (y < 0 || y >= TRACK_MODE4_HEIGHT || !rows[y].valid) return -1;
    return ring_side < 0 ? rows[y].left : rows[y].right;
}

/* Keep the foreground run that contains Algorithm 3's lane center. This
 * makes the feature scan follow the visible road as edges and scan direction
 * exchange roles through the bend. */
static void extract_rows(void)
{
    int y, x;
    memset(rows, 0, sizeof(rows));
    for (y = A15_FIRST_ROW; y <= A15_LAST_ROW; ++y) {
        int hint = track_mode4_path_confidence[y] >= 20U ?
                   track_mode4_path_x[y] : TRACK_STEER_REFERENCE_X;
        int best_left = -1, best_right = -1, best_distance = 10000;
        for (x = 0; x < TRACK_MODE4_WIDTH;) {
            int left, right, center, distance;
            while (x < TRACK_MODE4_WIDTH && !binary[y][x]) ++x;
            if (x >= TRACK_MODE4_WIDTH) break;
            left = x;
            while (x < TRACK_MODE4_WIDTH && binary[y][x]) ++x;
            right = x - 1;
            if (right - left + 1 < 5) continue;
            center = (left + right) / 2;
            if (left == 0 && right < TRACK_MODE4_WIDTH - 1)
                center = clamp_int(right - 58, 0, TRACK_MODE4_WIDTH - 1);
            else if (right == TRACK_MODE4_WIDTH - 1 && left > 0)
                center = clamp_int(left + 58, 0, TRACK_MODE4_WIDTH - 1);
            distance = abs(center - hint);
            if (distance < best_distance) {
                best_distance = distance;
                best_left = left;
                best_right = right;
            }
        }
        if (best_left >= 0 && best_distance <= 62) {
            rows[y].left = (int16_t)best_left;
            rows[y].right = (int16_t)best_right;
            rows[y].left_open = (uint8_t)(best_left == 0);
            rows[y].right_open = (uint8_t)(best_right == TRACK_MODE4_WIDTH - 1);
            rows[y].valid = 1U;
        }
    }
}

/* Detect a corner as the edge slope changes abruptly over a short scan-line
 * window. The middle feature specifically requires the edge to reverse its
 * lateral trend, matching the article's “)” / “(” middle-corner cue. */
static corner_t find_corner(int ring_side, int first, int last, int reversal)
{
    corner_t best = {-1, -1, 0U};
    int best_change = 0;
    int y;
    for (y = first + 6; y <= last - 6; ++y) {
        int a = edge_at(y - 6, ring_side);
        int b = edge_at(y, ring_side);
        int c = edge_at(y + 6, ring_side);
        int before, after, change;
        if (a < 0 || b < 0 || c < 0) continue;
        before = b - a;
        after = c - b;
        change = abs(after - before);
        if (change < A15_CORNER_MIN_DELTA) continue;
        if (reversal && (before == 0 || after == 0 || ((before < 0) == (after < 0))))
            continue;
        if (!best.valid || change > best_change) {
            best.row = (int16_t)y;
            best.x = (int16_t)b;
            best.valid = 1U;
            best_change = change;
        }
    }
    return best;
}

static side_features_t inspect_side(int ring_side)
{
    side_features_t f;
    int y, lost = 0;
    memset(&f, 0, sizeof(f));
    f.upper = find_corner(ring_side, 12, 52, 0);
    f.middle = find_corner(ring_side, 30, 82, 1);
    f.lower = find_corner(ring_side, 68, 116, 0);
    for (y = 92; y <= 116; ++y) {
        if (!rows[y].valid || (ring_side < 0 ? rows[y].left_open : rows[y].right_open))
            ++lost;
    }
    f.near_lost = (uint8_t)lost;
    /* Order and proximity tie all three corners to one visible boundary.
     * The alternative approach pattern is the article's curve-to-ring case:
     * upper and middle corners plus an opened/lost near boundary. */
    if (f.upper.valid && f.middle.valid && f.upper.row < f.middle.row &&
        f.middle.row - f.upper.row <= 38 &&
        ((f.lower.valid && f.middle.row < f.lower.row &&
          f.lower.row - f.middle.row <= 42) || lost >= A15_NEAR_LOSS_ROWS))
        f.valid = 1U;
    return f;
}

static int draw_white_line(corner_t from, corner_t to, int min_row, int max_row)
{
    int y, count = 0;
    if (!from.valid || !to.valid || from.row == to.row) return 0;
    min_row = clamp_int(min_row, A15_FIRST_ROW, A15_LAST_ROW);
    max_row = clamp_int(max_row, A15_FIRST_ROW, A15_LAST_ROW);
    if (min_row < (from.row < to.row ? from.row : to.row))
        min_row = from.row < to.row ? from.row : to.row;
    if (max_row > (from.row > to.row ? from.row : to.row))
        max_row = from.row > to.row ? from.row : to.row;
    for (y = min_row; y <= max_row; ++y) {
        int wanted = from.x + ((int)to.x - from.x) * (y - from.row) /
                     ((int)to.row - from.row);
        int distance, found = -1;
        wanted = clamp_int(wanted, 0, TRACK_MODE4_WIDTH - 1);
        for (distance = 0; distance <= A15_LINE_SEARCH_RADIUS; ++distance) {
            int left = wanted - distance, right = wanted + distance;
            if (left >= 0 && binary[y][left]) { found = left; break; }
            if (right < TRACK_MODE4_WIDTH && binary[y][right]) { found = right; break; }
        }
        if (found >= 0) {
            track_ring15_path_x[y] = (int16_t)found;
            track_ring15_path_confidence[y] = 55U;
            ++count;
        } else {
            track_ring15_path_confidence[y] = 0U;
        }
    }
    return count;
}

/* Stage 4 follows the road visible in each scan row. Do not fall back to the
 * ordinary-road A3 target here: at a ring entrance A3 can keep selecting the
 * straight branch. This is a per-frame geometric centerline, with no temporal
 * averaging. If one edge is clipped, estimate the center from the measured
 * opposite edge and the A3 width prior. */
static int draw_ring_road_center(void)
{
    int y, count = 0;
    for (y = A15_FIRST_ROW; y <= A15_LAST_ROW; ++y) {
        int width = (int)TRACK_CONT_WIDTH_PRIOR * (y + 10) / 126;
        int center;
        if (width < 8) width = 8;
        if (!rows[y].valid) {
            track_ring15_path_confidence[y] = 0U;
            continue;
        }
        if (!rows[y].left_open && !rows[y].right_open)
            center = (rows[y].left + rows[y].right) / 2;
        else if (rows[y].left_open && !rows[y].right_open)
            center = rows[y].right - width / 2;
        else if (!rows[y].left_open && rows[y].right_open)
            center = rows[y].left + width / 2;
        else {
            track_ring15_path_confidence[y] = 0U;
            continue;
        }
        center = clamp_int(center, 0, TRACK_MODE4_WIDTH - 1);
        track_ring15_path_x[y] = (int16_t)center;
        track_ring15_path_confidence[y] = 55U;
        ++count;
    }
    return count;
}

static corner_t move_edge_to_road_center(corner_t edge, int ring_side)
{
    int width;
    if (!edge.valid) return edge;
    width = (int)TRACK_CONT_WIDTH_PRIOR * (edge.row + 10) / 126;
    edge.x = (int16_t)clamp_int(edge.x + (ring_side < 0 ? width / 2 : -width / 2),
                                0, TRACK_MODE4_WIDTH - 1);
    return edge;
}

/* Follow the three ordered visible corners on the island-side boundary.
 * Each segment ends at an observed corner; it is never extrapolated beyond
 * that corner into the far image. */
static int draw_ring_corner_path(corner_t anchor, const side_features_t *f,
                                 int ring_side)
{
    corner_t upper, middle, lower;
    int count = 0;
    if (!anchor.valid || !f->upper.valid || !f->middle.valid || !f->lower.valid ||
        !(f->upper.row < f->middle.row && f->middle.row < f->lower.row))
        return 0;
    upper = move_edge_to_road_center(f->upper, ring_side);
    middle = move_edge_to_road_center(f->middle, ring_side);
    lower = move_edge_to_road_center(f->lower, ring_side);
    count += draw_white_line(anchor, lower, lower.row, anchor.row);
    count += draw_white_line(lower, middle, middle.row, lower.row);
    count += draw_white_line(middle, upper, upper.row, middle.row);
    return count;
}

static corner_t bottom_anchor(const track_mode4_result_t *a3)
{
    corner_t p = {(int16_t)TRACK_CONT_ANCHOR_ROW, a3->anchor_x, 0U};
    p.valid = (uint8_t)(a3->anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE);
    return p;
}

static void set_feature_diagnostics(const side_features_t *f)
{
    track_ring15_lower_row = f->lower.valid ? f->lower.row : -1;
    track_ring15_lower_x = f->lower.valid ? f->lower.x : -1;
    track_ring15_middle_row = f->middle.valid ? f->middle.row : -1;
    track_ring15_middle_x = f->middle.valid ? f->middle.x : -1;
    track_ring15_upper_row = f->upper.valid ? f->upper.row : -1;
    track_ring15_upper_x = f->upper.valid ? f->upper.x : -1;
}

static int16_t servo_for_x(int x)
{
    int angle = (int)TRACK_SERVO_CENTER_DEG -
        ((x - (int)TRACK_STEER_REFERENCE_X) *
         (int)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL) / 100;
    angle = clamp_int(angle,
        (int)TRACK_SERVO_CENTER_DEG - (int)TRACK_A3_MAX_STEER_DELTA_DEG,
        (int)TRACK_SERVO_CENTER_DEG + (int)TRACK_A3_MAX_STEER_DELTA_DEG);
    angle = clamp_int(angle, (int)TRACK_SERVO_RIGHT_DEG, (int)TRACK_SERVO_LEFT_DEG);
    return (int16_t)(angle * 100);
}

/* Match main.c's Algorithm 3 steering calculation for ordinary tracking,
 * preparation, merge, and any frame where a ring-specific path was not made. */
static int16_t servo_for_a3_x(int x)
{
    int angle = (int)TRACK_SERVO_CENTER_DEG -
        ((x - (int)TRACK_STEER_REFERENCE_X) *
         (int)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL) / 100;
    angle = clamp_int(angle,
        (int)TRACK_SERVO_CENTER_DEG - (int)TRACK_A3_MAX_STEER_DELTA_DEG,
        (int)TRACK_SERVO_CENTER_DEG + (int)TRACK_A3_MAX_STEER_DELTA_DEG);
    angle = clamp_int(angle, (int)TRACK_SERVO_RIGHT_DEG, (int)TRACK_SERVO_LEFT_DEG);
    return (int16_t)(angle * 100);
}

void track_ring15_reset(void)
{
    track_mode4_reset();
    stage = A15_NORMAL; side = candidate_count = state_count = 0U;
    exit_count = merge_count = display_phase = 0U;
    loss_ms = 0U;
    remembered_upper = remembered_exit = (corner_t){-1, -1, 0U};
    track_ring15_lookahead_row = TRACK_CONT_TARGET_ROW;
    track_ring15_loss_ms = 0U;
    track_ring15_side = 0;
    track_ring15_display_phase = track_ring15_stage = 0U;
    track_ring15_lower_row = track_ring15_middle_row = track_ring15_upper_row = -1;
    track_ring15_lower_x = track_ring15_middle_x = track_ring15_upper_x = -1;
    memset(binary, 0, sizeof(binary));
    memset(rows, 0, sizeof(rows));
    memset(track_ring15_path_confidence, 0, sizeof(track_ring15_path_confidence));
}

void track_ring15_suspend(uint8_t why)
{
    track_ring15_reset();
    (void)why;
}

const char *track_ring15_state_name(uint8_t s)
{
    switch (s) {
        case FSM_NORMAL: return "A15 TRACK";
        case FSM_RING_CANDIDATE: return "A15 PREP";
        case FSM_RING_ENTER: return "A15 ENTRY";
        case FSM_RING_INSIDE: return "A15 TURN";
        case FSM_RING_EXIT: return "A15 EXIT";
        case FSM_RING_MERGE: return "A15 MERGE";
        case FSM_RECOVER: return "A15 LOST";
        default: return "A15 WAIT";
    }
}

void track_ring15_binary_preview(const uint8_t input[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH],
                                 uint8_t output[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH],
                                 uint8_t threshold)
{
    (void)threshold;
    track_mode4_binary_preview(input, output);
}

track_fsm_result_t track_ring15_process(
    const uint8_t image[TRACK_MODE4_HEIGHT][TRACK_MODE4_WIDTH], uint32_t dt,
    uint8_t run_enabled)
{
    track_fsm_result_t out = {0};
    track_mode4_result_t a3 = track_mode4_process(image);
    side_features_t left, right, *features = 0;
    corner_t anchor, current_upper, current_exit;
    uint8_t raw_state = FSM_NORMAL;
    int candidate_side = 0, custom_rows = 0;
    int target_y = TRACK_CONT_TARGET_ROW;

    /* Binary data is exclusively the grayscale threshold output of A3. */
    track_mode4_binary_preview(image, binary);
    extract_rows();
    left = inspect_side(-1);
    right = inspect_side(1);
    if (left.valid && (!right.valid || left.near_lost >= right.near_lost)) {
        candidate_side = -1; features = &left;
    } else if (right.valid) {
        candidate_side = 1; features = &right;
    }
    if (features) selected_features = *features;
    else memset(&selected_features, 0, sizeof(selected_features));
    set_feature_diagnostics(&selected_features);

    if (stage == A15_NORMAL) {
        if (candidate_side) {
            if (candidate_count < 255U) ++candidate_count;
            if (candidate_count >= A15_CONFIRM_FRAMES) {
                side = (uint8_t)(candidate_side < 0 ? 1U : 2U);
                stage = features->lower.valid ? A15_APPROACH_STRAIGHT : A15_APPROACH_CURVE;
                display_phase = 1U;
                state_count = 0U;
                remembered_upper = features->upper;
            }
        } else candidate_count = 0U;
    } else if (stage == A15_APPROACH_STRAIGHT || stage == A15_APPROACH_CURVE) {
        int s = side == 1U ? -1 : 1;
        side_features_t *f = s < 0 ? &left : &right;
        if (f->upper.valid) remembered_upper = f->upper;
        /* Keep preparation latched. Entry starts when the upper corner reaches
         * the near half of the image, or the approach boundary opens. */
        if ((f->upper.valid && f->upper.row >= 42) || f->near_lost >= A15_NEAR_LOSS_ROWS) {
            stage = A15_ENTER;
            state_count = 0U;
            display_phase = 2U;
        }
    } else if (stage == A15_ENTER) {
        int s = side == 1U ? -1 : 1;
        side_features_t *f = s < 0 ? &left : &right;
        if (f->upper.valid) remembered_upper = f->upper;
        if ((f->upper.valid && f->upper.row >= 62) ||
            (!f->upper.valid && a3.valid_rows >= TRACK_DRIVE_MIN_VALID_ROWS)) {
            stage = A15_CIRCULATE;
            state_count = 0U;
        }
    } else if (stage == A15_CIRCULATE) {
        int s = side == 1U ? 1 : -1; /* opposite boundary is the exit edge */
        current_exit = find_corner(s, 70, 114, 0);
        if (current_exit.valid && current_exit.row >= 78) {
            if (exit_count < 255U) ++exit_count;
            remembered_exit = current_exit;
            if (exit_count >= A15_EXIT_CONFIRM_FRAMES) {
                stage = A15_EXIT_PULL;
                exit_count = 0U;
            }
        } else exit_count = 0U;
    } else if (stage == A15_EXIT_PULL) {
        int s = side == 1U ? 1 : -1;
        current_exit = find_corner(s, 72, 116, 0);
        if (current_exit.valid) remembered_exit = current_exit;
        if (!current_exit.valid || current_exit.row >= 108) {
            stage = A15_EXIT_TURN;
            state_count = 0U;
        }
    } else if (stage == A15_EXIT_TURN) {
        int s = side == 1U ? -1 : 1;
        side_features_t *f = s < 0 ? &left : &right;
        if (f->upper.valid) remembered_upper = f->upper;
        if (f->upper.valid && f->upper.row >= 38) {
            if (state_count < 255U) ++state_count;
            if (state_count >= A15_CORNER_CONFIRM_FRAMES) {
                stage = A15_STRAIGHTEN;
                state_count = 0U;
            }
        } else state_count = 0U;
    } else if (stage == A15_STRAIGHTEN) {
        int s = side == 1U ? -1 : 1;
        side_features_t *f = s < 0 ? &left : &right;
        if (f->upper.valid) remembered_upper = f->upper;
        if ((!f->upper.valid || f->upper.row >= 74) &&
            a3.anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
            a3.target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE &&
            a3.valid_rows >= TRACK_DRIVE_MIN_VALID_ROWS) {
            if (merge_count < 255U) ++merge_count;
            if (merge_count >= A15_MERGE_CONFIRM_FRAMES) {
                stage = A15_MERGE;
                merge_count = 0U;
            }
        } else merge_count = 0U;
    } else if (stage == A15_MERGE) {
        if (a3.anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
            a3.target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE &&
            a3.valid_rows >= TRACK_DRIVE_MIN_VALID_ROWS) {
            if (merge_count < 255U) ++merge_count;
            if (merge_count >= A15_MERGE_CONFIRM_FRAMES) {
                stage = A15_NORMAL;
                side = candidate_count = 0U;
                display_phase = 0U;
                merge_count = 0U;
            }
        } else merge_count = 0U;
    }

    memcpy(track_ring15_path_x, track_mode4_path_x, sizeof(track_ring15_path_x));
    memcpy(track_ring15_path_confidence, track_mode4_path_confidence,
           sizeof(track_ring15_path_confidence));
    anchor = bottom_anchor(&a3);
    current_upper = remembered_upper;
    current_exit = remembered_exit;
    if (stage == A15_ENTER && current_upper.valid) {
        memset(track_ring15_path_confidence, 0, sizeof(track_ring15_path_confidence));
        custom_rows = draw_white_line(anchor, current_upper, A15_FIRST_ROW, A15_LAST_ROW);
        raw_state = FSM_RING_ENTER;
    } else if (stage == A15_CIRCULATE) {
        int ring_side = side == 1U ? -1 : 1;
        side_features_t *f = ring_side < 0 ? &left : &right;
        memset(track_ring15_path_confidence, 0, sizeof(track_ring15_path_confidence));
        custom_rows = draw_ring_corner_path(anchor, f, ring_side);
        if (custom_rows < TRACK_DRIVE_MIN_VALID_ROWS)
            custom_rows = draw_ring_road_center();
        raw_state = FSM_RING_INSIDE;
    } else if (stage == A15_EXIT_PULL && current_exit.valid) {
        memset(track_ring15_path_confidence, 0, sizeof(track_ring15_path_confidence));
        custom_rows = draw_white_line(anchor, current_exit, A15_FIRST_ROW, A15_LAST_ROW);
        raw_state = FSM_RING_EXIT;
    } else if (stage == A15_EXIT_TURN && current_upper.valid) {
        memset(track_ring15_path_confidence, 0, sizeof(track_ring15_path_confidence));
        custom_rows = draw_white_line(anchor, current_upper, A15_FIRST_ROW, A15_LAST_ROW);
        raw_state = FSM_RING_MERGE;
    } else if (stage == A15_STRAIGHTEN && current_upper.valid) {
        memset(track_ring15_path_confidence, 0, sizeof(track_ring15_path_confidence));
        custom_rows = draw_white_line(anchor, current_upper, A15_FIRST_ROW, A15_LAST_ROW);
        raw_state = FSM_RING_MERGE;
    } else if (stage == A15_MERGE) {
        raw_state = FSM_RING_MERGE;
    } else if (stage == A15_APPROACH_STRAIGHT || stage == A15_APPROACH_CURVE) {
        raw_state = FSM_RING_CANDIDATE;
    }

    out.anchor_x = a3.anchor_x;
    out.target_x = a3.target_x;
    out.anchor_confidence = a3.anchor_confidence;
    out.target_confidence = a3.target_confidence;
    out.valid_rows = a3.valid_rows;
    out.observed_rows = a3.valid_rows;
    out.global_threshold = a3.global_threshold;
    out.state = raw_state;
    out.reason = FSM_REASON_RING;
    out.ring_side = side ? (side == 1U ? -1 : 1) : 0;
    out.pwm_cap_percent = TRACK_WHEEL_PWM_MAX_PERCENT;
    out.drive_allowed = (uint8_t)(run_enabled && dt < TRACK_CAMERA_LOST_TIMEOUT_MS &&
        a3.anchor_confidence >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        a3.target_confidence >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE &&
        a3.valid_rows >= TRACK_DRIVE_MIN_VALID_ROWS);

    if (custom_rows >= TRACK_DRIVE_MIN_VALID_ROWS &&
        track_ring15_path_confidence[target_y] >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE) {
        out.target_x = track_ring15_path_x[target_y];
        out.target_confidence = track_ring15_path_confidence[target_y];
        out.valid_rows = (uint8_t)custom_rows;
        out.observed_rows = (uint8_t)custom_rows;
        out.servo_cdeg = servo_for_x(out.target_x);
        out.drive_allowed = (uint8_t)(run_enabled && dt < TRACK_CAMERA_LOST_TIMEOUT_MS);
    } else {
        out.servo_cdeg = servo_for_a3_x(out.target_x);
    }

    if (!out.drive_allowed) {
        loss_ms += dt;
        if (stage >= A15_ENTER && stage <= A15_STRAIGHTEN) {
            out.state = FSM_RECOVER;
            out.reason = FSM_REASON_LOST;
        }
    } else {
        loss_ms = 0U;
    }
    if (dt >= TRACK_CAMERA_LOST_TIMEOUT_MS) {
        track_ring15_suspend(FSM_REASON_CAMERA);
        out.drive_allowed = 0U;
        out.state = FSM_WAIT;
        out.reason = FSM_REASON_CAMERA;
    }

    if (stage == A15_APPROACH_STRAIGHT || stage == A15_APPROACH_CURVE)
        display_phase = 1U;
    else if (stage >= A15_ENTER && stage <= A15_MERGE)
        display_phase = 2U;
    else display_phase = 0U;
    track_ring15_stage = stage;
    track_ring15_display_phase = display_phase;
    track_ring15_loss_ms = loss_ms;
    track_ring15_lookahead_row = (uint32_t)target_y;
    track_ring15_side = out.ring_side;
    out.state = (stage == A15_NORMAL) ? FSM_NORMAL : raw_state;
    if (raw_state == FSM_RECOVER) out.state = FSM_RECOVER;
    return out;
}
