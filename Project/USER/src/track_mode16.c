/*
 * Algorithm 16 ports the supplied 八邻域_v2.0 contour approach.
 * It owns its thresholded binary image and 8-neighbor-connected road trace. Each
 * scanline keeps the white road run connected diagonally or horizontally to
 * the preceding row; the two observed run edges define that row's center.
 * Island acquisition and corridor ownership are local to Algorithm 16.
 *
 * The road run is bounded by the camera's black side frame. A nearly full
 * width run remains valid because a broad foreground mask is common near the
 * bottom of the real camera view.
 */
#include "a16_internal.h"
#include "tune_config.h"
#include <string.h>
#include <math.h>

#ifndef TRACK_MODE16_GRAY_THRESHOLD
#define TRACK_MODE16_GRAY_THRESHOLD 232U
#endif

uint32_t a16_frame_ms=33U;

void track_mode16_set_frame_dt(uint32_t dt_ms)
{
    a16_frame_ms = dt_ms == 0U ? 1U : (dt_ms > 1000U ? 1000U : dt_ms);
}

int track_mode16_corner_active;
#if TRACK_ALGORITHM == 16
static unsigned road_recovery_frames;
static int lost_ordinary_road;
int track_mode16_obstacle_side;
int track_mode16_inside_px;
int track_mode16_straight;
#endif
uint8_t a16_binary_image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH];
uint8_t a16_binary_scratch[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH];
volatile uint32_t track_mode16_phase;
volatile int32_t track_mode16_island_side;
volatile int32_t track_mode16_island_bottom;
volatile uint32_t track_mode16_target_row = TRACK_OTSUSCAN_TARGET_ROW;

int16_t track_mode16_path_x[TRACK_MODE1_HEIGHT];
uint8_t track_mode16_path_confidence[TRACK_MODE1_HEIGHT];

static void make_binary(
    const uint8_t image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH], uint8_t threshold)
{
    uint16_t x, y;
    for (y = 0U; y < TRACK_MODE1_HEIGHT; ++y)
        for (x = 0U; x < TRACK_MODE1_WIDTH; ++x)
            a16_binary_image[y][x] = image[y][x] > threshold ? 255U : 0U;
    for (y = 0U; y < TRACK_MODE1_HEIGHT; ++y) {
        a16_binary_image[y][0] = 0U;
        a16_binary_image[y][1] = 0U;
        a16_binary_image[y][TRACK_MODE1_WIDTH - 2U] = 0U;
        a16_binary_image[y][TRACK_MODE1_WIDTH - 1U] = 0U;
    }
    for (x = 0U; x < TRACK_MODE1_WIDTH; ++x) {
        a16_binary_image[0][x] = 0U;
        a16_binary_image[1][x] = 0U;
    }
}

/* Match the reference package's 8-neighbor binary cleanup (center excluded). */
static void filter_binary(void)
{
    uint16_t x, y;
    memcpy(a16_binary_scratch, a16_binary_image, sizeof(a16_binary_image));
    for (y = 1U; y + 1U < TRACK_MODE1_HEIGHT; ++y) {
        for (x = 1U; x + 1U < TRACK_MODE1_WIDTH; ++x) {
            uint8_t count = 0U;
            int8_t dx, dy;
            for (dy = -1; dy <= 1; ++dy)
                for (dx = -1; dx <= 1; ++dx)
                    if ((dx != 0 || dy != 0) &&
                        a16_binary_scratch[(int)y + dy][(int)x + dx] != 0U)
                        ++count;
            if (a16_binary_scratch[y][x] == 0U && count >= 5U)
                a16_binary_image[y][x] = 255U;
            else if (a16_binary_scratch[y][x] != 0U && count <= 2U)
                a16_binary_image[y][x] = 0U;
        }
    }
}

/* Find the white run in this row that is connected to the previous road run
 * under 8-neighbor connectivity. Connectivity permits a one-pixel diagonal
 * offset between adjacent rows, replacing the reference trace's ambiguous
 * "pick the topmost candidate" rule. */
static int find_connected_run(int y, int previous_left, int previous_right,
                              int previous_center, int previous_delta,
                              int *run_left, int *run_right)
{
    int x = 2;
    int best_score = 0x7fffffff;
    int best_left = -1, best_right = -1;
    while (x < (int)TRACK_MODE1_WIDTH - 2) {
        int left, right, center, overlap, score;
        while (x < (int)TRACK_MODE1_WIDTH - 2 && a16_binary_image[y][x] == 0U) ++x;
        left = x;
        while (x < (int)TRACK_MODE1_WIDTH - 2 && a16_binary_image[y][x] != 0U) ++x;
        right = x - 1;
        if (right - left + 1 < 6) continue;
        center = (left + right) / 2;
        /* Predict from a low-pass path slope. Compare every plausible run:
         * unconditional preference for any connected run makes the tracer
         * snap to a straight-ahead branch at roundabout entrances. */
        if (center - previous_center > 40 || previous_center - center > 40) continue;
        overlap = (right < previous_right ? right : previous_right) -
                  (left > previous_left ? left : previous_left) + 1;
        if (overlap < 0) overlap = 0;
        {
            int predicted_center = previous_center + previous_delta / 256;
            int residual = center > predicted_center ? center - predicted_center :
                           predicted_center - center;
            int connected = right >= previous_left - 1 && left <= previous_right + 1;
            score = residual * 6 + (connected ? 0 : 4) -
                    (overlap > 8 ? 8 : overlap);
            if (track_mode16_phase == A16_ENTRY || track_mode16_phase == A16_ARC) {
                int edge_delta = track_mode16_island_side < 0 ? left-previous_left : right-previous_right;
                if (edge_delta < 0) edge_delta = -edge_delta;
                score = edge_delta*8 + residual*2 + (connected ? 0 : 8) -
                        (overlap > 8 ? 8 : overlap);
            }
        }
        if (score < best_score) {
            best_score = score;
            best_left = left;
            best_right = right;
        }
    }
    if (best_left >= 0) {
        *run_left = best_left;
        *run_right = best_right;
        return 1;
    }
    return 0;
}

void track_mode16_reset(void)
{
#if TRACK_ALGORITHM == 16
    road_recovery_frames = 0;
    lost_ordinary_road = 0;
    a16_obstacle_reset();
    track_mode16_straight = track_mode16_inside_px = 0;
#endif
    track_mode16_corner_active = 0;
    a16_ring_reset();
    memset(track_mode16_path_x, 0, sizeof(track_mode16_path_x));
    memset(track_mode16_path_confidence, 0, sizeof(track_mode16_path_confidence));
}

void track_mode16_binary_preview(
    const uint8_t input[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH],
    uint8_t output[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH], uint8_t unused_threshold)
{
    uint16_t x, y;
    (void)unused_threshold;
    for (y = 0U; y < TRACK_MODE1_HEIGHT; ++y)
        for (x = 0U; x < TRACK_MODE1_WIDTH; ++x)
            output[y][x] = a16_binary_image[y][x];
    (void)input;
}

#if TRACK_ALGORITHM == 16
/* At a tight L-turn the bottom run may fill the frame while the next
 * connected run moves more than the normal 40 pixel association limit.
 * Retry only a lost ROAD path, requiring a measured continuous exit on
 * the same side throughout the near field and a dark region ahead. */
static int retrace_corner(track_mode1_result_t *road)
{
    int y=TRACK_MODE1_BOTTOM_ROW;
    if (!a16_binary_image[y][15] || !a16_binary_image[y][94] || !a16_binary_image[y][172]) return 0;
    int l,r,side;
    y=A16_ROAD_TARGET_ROW;
    if (a16_binary_image[y][176] && !a16_binary_image[y][12]) side=1;
    else if (a16_binary_image[y][12] && !a16_binary_image[y][176]) side=-1;
    else return 0;
    int probe=side>0 ? 176 : 12;
    if (a16_binary_image[70][94] || a16_binary_image[70][probe]) return 0;
    int16_t centers[33];
    int target=0;
    for (y=116; y>=84; --y) {
        if (!a16_binary_image[y][probe]) return 0;
        l=r=probe;
        while (l>2 && a16_binary_image[y][l-1]) --l;
        while (r<185 && a16_binary_image[y][r+1]) ++r;
        if (r-l<48) return 0;
        if (y>=88 && y<=(int)A16_ROAD_TARGET_ROW) {
            if (side>0 ? l<45 || l>137 || r<182 : r<51 || r>142 || l>5) return 0;
        }
        centers[116-y]=(l+r)/2;
        if (y==(int)A16_ROAD_TARGET_ROW)
            target=side>0 ? r-(r-l)/4 : l+(r-l)/4;
    }
    for (y=116; y>=84; --y) {
        track_mode16_path_x[y]=centers[116-y];
        track_mode16_path_confidence[y]=80;
    }
    road->target_x=track_mode16_path_x[A16_ROAD_TARGET_ROW]=target;
    road->anchor_x=centers[0];
    road->valid_rows=33;
    road->target_confidence=road->anchor_confidence=80;
    track_mode16_target_row=A16_ROAD_TARGET_ROW;
    track_mode16_corner_active=1;
    return 1;
}

#endif

#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
/* Look for the end of the forward lane and a connected, one-sided exit.
 * Use the current bottom-connected contour for both the target and anchor. */
int track_mode16_corner(track_mode1_result_t *road, int weak_road)
{
    if (!track_mode16_path_confidence[116]) return 0;
    int votes = 0, side = 0;
    for (int y = weak_road ? 60 : A16_CORNER_START_ROW; y <= 110; y += 2) {
        if (!track_mode16_path_confidence[y]) { votes = 0; continue; }
        int mid = track_mode16_path_x[y], left = mid, right = mid;
        while (left > 2 && a16_binary_image[y][left-1]) --left;
        while (right < 185 && a16_binary_image[y][right+1]) ++right;
        int opening = 0;
        if (left <= 5 && right >= 40 && right < 134) opening = -1;
        if (right >= 182 && left > 53 && left <= 147) opening = 1;
        /* Ahead of the transverse strip both the middle and the opening
         * must be dark. A continuing lane or a two-sided cross fails here. */
        int probe = opening < 0 ? 35 : 152;
        if (a16_binary_image[y-24][94] ||
            a16_binary_image[y-24][probe] || !a16_binary_image[y][probe]) opening = 0;
        /* Around the apex the far boundary becomes diagonal. If the
         * ordinary anchor is lost, follow that measured edge through the turn. */
        if (y < A16_CORNER_START_ROW) opening = 0;
        if (!opening && weak_road) {
            int x = track_mode16_path_x[y-12];
            if (track_mode16_path_confidence[y-12]) {
                int lo = x, hi = x;
                while (lo > 2 && a16_binary_image[y-12][lo-1]) --lo;
                while (hi < 185 && a16_binary_image[y-12][hi+1]) ++hi;
                if (left <= 20 && lo <= 20 && right >= 45 && right <= 173 &&
                    right-hi >= 10 && hi >= 18) opening = -1;
                if (right >= 167 && hi >= 167 && left >= 14 && left <= 142 &&
                    lo-left >= 10 && lo <= 169) opening = 1;
            }
        }
        if (!opening || opening != side) votes = 0;
        side = opening;
        if (!side || ++votes < 3) continue;
        if (119-y < 27 && !track_mode16_path_confidence[90]) return 0;
        int rows = 0;
        for (int r = 116; r >= 48 && track_mode16_path_confidence[r]; --r) ++rows;
        if (rows < 27) return 0;
        road->anchor_x = track_mode16_path_x[116];
        road->anchor_confidence = track_mode16_path_confidence[116];
        road->target_x = side < 0 ? left + (right-left)/4 : right - (right-left)/4;
        road->valid_rows = rows;
        road->target_confidence = track_mode16_path_confidence[y];
        track_mode16_path_x[y] = road->target_x;
        track_mode16_target_row = y;
        return 1;
    }
    return 0;
}
#endif

#if TRACK_ALGORITHM == 16
/* Continue a measured far trunk across an opening on both sides. The two
 * side arms of a slanted cross can reach opposite edges at different rows. */
static int crossing_trunk(track_mode1_result_t *road)
{
    if (!road->anchor_confidence || road->valid_rows < 27) return 0;
    for (int y0 = 44; y0 <= 68; y0 += 4) {
        int y1 = y0 + 12;
        if (!track_mode16_path_confidence[y0] || !track_mode16_path_confidence[y1]) continue;
        int x0 = track_mode16_path_x[y0], x1 = track_mode16_path_x[y1];
        int l0 = x0, r0 = x0, l1 = x1, r1 = x1;
        while (l0 > 2 && a16_binary_image[y0][l0-1]) --l0;
        while (r0 < 185 && a16_binary_image[y0][r0+1]) ++r0;
        while (l1 > 2 && a16_binary_image[y1][l1-1]) --l1;
        while (r1 < 185 && a16_binary_image[y1][r1+1]) ++r1;
        int wl = l0 - l1, wr = r1 - r0;
        if (l1 < 8 || r1 > 179 || r0-l0 < 14 || wl < 4 || wr < 4 ||
            wl > 18 || wr > 18 || x1-x0 < -6 || x1-x0 > 6) continue;
        int left_votes = 0, right_votes = 0;
        for (int y = y1 + 4; y <= 116; y += 2) {
            int delta = y - y1;
            int l = l1 - wl * delta / 12 - 14, r = r1 + wr * delta / 12 + 14;
            int x = x1 + (x1-x0) * delta / 12;
            if (x < 2 || x > 185 || !a16_binary_image[y][x]) break;
            int li = x, ri = x;
            while (li > 2 && a16_binary_image[y][li-1]) --li;
            while (ri < 185 && a16_binary_image[y][ri+1]) ++ri;
            if (li <= l) ++left_votes;
            if (ri >= r) ++right_votes;
            if (left_votes < 3 || right_votes < 3) continue;
            int near_x = x1 + (x1 - x0) * (116 - y1) / 12;
            if (near_x < 2 || near_x > 185 || !a16_binary_image[116][near_x]) break;
            /* Verify every center pixel before replacing the measured path. */
            int yy;
            for (yy = y1; yy <= 116; ++yy) {
                int xx = x1 + (x1 - x0) * (yy - y1) / 12;
                if (!a16_binary_image[yy][xx]) break;
            }
            if (yy <= 116) break;
            for (yy = y1; yy <= 116; ++yy) {
                track_mode16_path_x[yy] = x1 + (x1 - x0) * (yy - y1) / 12;
                track_mode16_path_confidence[yy] = 80;
            }
            road->target_x = track_mode16_path_x[A16_ROAD_TARGET_ROW];
            road->anchor_x = near_x;
            road->target_confidence = road->anchor_confidence = 80;
            track_mode16_target_row = A16_ROAD_TARGET_ROW;
            return 1;
        }
    }
    return 0;
}
#endif

track_mode1_result_t track_mode16_process(
    const uint8_t image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH])
{
    track_mode1_result_t result = {94, 94, 0U, 0U, 0U, 0U};
    int previous_left = -1, previous_right = -1;
    int previous_center = TRACK_MODE1_WIDTH / 2;
    int32_t previous_delta_q8 = 0;
    int seed_y = (int)TRACK_MODE1_BOTTOM_ROW;
    int seed_left = -1, seed_right = -1;
    int seed_distance = 0x7fffffff;
    int x;
    int y;

    result.global_threshold = TRACK_MODE16_GRAY_THRESHOLD;
    unsigned hist[32]={0}, n=0;
    for (int yy=32;yy<=116;yy+=4) for (int xx=4;xx<184;xx+=4) {
        ++hist[image[yy][xx]>>3];++n;
    }
    unsigned cum=0, low=0, high=0;
    for (unsigned i=0;i<32;++i) {
        cum+=hist[i];if(cum*10<n)low=i+1;
        if(cum*10>=n*9){high=i;break;}
    }
#if TRACK_ALGORITHM == 16
    if (high >= low + 10U && high * 8U < result.global_threshold + 24U)
        result.global_threshold = (uint8_t)((low + 3U * high) * 2U);
    /* A threshold in stretched gray is equivalent to its inverse in native
     * gray. Keep original pixels for the brick and finish-mark detectors. */
    if (high >= low + 4U && (high < low + 10U || high >= 30U)) {
        /* In a narrow histogram each 8-level bin matters. Use its centre
         * for the dim-scene estimate; bright-scene cuts keep the old scale. */
        unsigned bin_center = high < low + 10U ? 4U : 0U;
        unsigned midpoint = (low + high) * 4U + bin_center;
        unsigned white_cut = (low + 3U * high) * 2U + bin_center;
        result.global_threshold = (uint8_t)(midpoint +
            (white_cut - midpoint) * 100U / TRACK_A16_CONTRAST_GAIN_PERCENT);
    }
#else
    if (high*8+7 < result.global_threshold && high>=low+10)
        result.global_threshold=(uint8_t)((low+high)*4+4);
#endif
#if TRACK_ALGORITHM == 16
    /* Measure exposure in native pixels before masking unobservable ground. */
    image = a16_reference_view(image);
    a16_update_lap(image,low,high);
#endif
    make_binary(image, result.global_threshold);
#if TRACK_ALGORITHM == 16
    /* A uniformly white exposure is a connected white run but provides no
     * observable lane edges. Count sparse minority pixels as sensor noise. */
    unsigned darker = 0, brighter = 0;
    for (unsigned bin = 0; bin < 32; ++bin) {
        if ((bin + 1U) * 8U - 1U <= result.global_threshold) darker += hist[bin];
        if (bin * 8U > result.global_threshold) brighter += hist[bin];
    }
    if (darker < 4U || brighter < 4U) {
        int was_road = track_mode16_phase == A16_ROAD ||
                       (track_mode16_phase == A16_WAIT && lost_ordinary_road);
        track_mode16_reset();
        lost_ordinary_road = was_road;
        track_mode16_phase = A16_WAIT;
        track_mode16_target_row = A16_ROAD_TARGET_ROW;
        return result;
    }
#endif
    filter_binary();
#if TRACK_ALGORITHM == 16
    if (a16_departure_mark_ms) a16_join_departure_rails();
#endif
    track_mode16_target_row = TRACK_OTSUSCAN_TARGET_ROW;
    memset(track_mode16_path_confidence, 0, sizeof(track_mode16_path_confidence));
    /* Seed from the bottom row's white run nearest the image center. */
    for (x = 2; x < (int)TRACK_MODE1_WIDTH - 2;) {
        int left, right, center, width, distance;
        while (x < (int)TRACK_MODE1_WIDTH - 2 && a16_binary_image[seed_y][x] == 0U) ++x;
        left = x;
        while (x < (int)TRACK_MODE1_WIDTH - 2 && a16_binary_image[seed_y][x] != 0U) ++x;
        right = x - 1;
        width = right - left + 1;
        if (width < 6 || width > A16_MAX_ROAD_WIDTH) continue;
        center = (left + right) / 2;
        distance = center > previous_center ? center - previous_center : previous_center - center;
        /* Keep the nearest-to-center run as the road seed. */
        if (distance < seed_distance) {
            seed_distance = distance;
            seed_left = left;
            seed_right = right;
        }
    }
    if (seed_left >= 0) {
        previous_left = seed_left;
        previous_right = seed_right;
        previous_center = (seed_left + seed_right) / 2;
        for (y = seed_y; y >= (int)TRACK_OTSUSCAN_SCAN_TOP_ROW; --y) {
            int left = previous_left;
            int right = previous_right;
            int center;
            if (y != seed_y) {
                if (!find_connected_run(y, previous_left, previous_right,
                                        previous_center, previous_delta_q8,
                                        &left, &right)) break;
                center = (left + right) / 2;
                previous_delta_q8 = (previous_delta_q8 * 3 +
                                     (center - previous_center) * 256) / 4;
                previous_left = left;
                previous_right = right;
                previous_center = center;
            } else {
                center = previous_center;
            }
            track_mode16_path_x[y] = (int16_t)center;
            track_mode16_path_confidence[y] = 90U;
            ++result.valid_rows;
            if (y == (int)TRACK_OTSUSCAN_ANCHOR_ROW) {
                result.anchor_x = (int16_t)center;
                result.anchor_confidence = 90U;
            }
            if (y == (int)TRACK_OTSUSCAN_TARGET_ROW) {
                result.target_x = (int16_t)center;
                result.target_confidence = 90U;
            }
        }
    }
#if TRACK_ALGORITHM == 16
    if (lost_ordinary_road && track_mode16_phase == A16_WAIT) {
        /* Recover a washed-out ordinary bend as soon as two fresh frames
         * contain a connected, measured near lane. Ring WAIT stays separate. */
        if (result.valid_rows >= TRACK_DRIVE_MIN_VALID_ROWS &&
            track_mode16_path_confidence[TRACK_A16_LOOKAHEAD_ROW] >= TRACK_DRIVE_MIN_TARGET_CONFIDENCE &&
            track_mode16_path_confidence[TRACK_A16_NEAR_ROW] >= TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE) {
            if (++road_recovery_frames >= 2U) track_mode16_phase = A16_ROAD;
        } else {
            road_recovery_frames = 0;
        }
    }
    if (track_mode16_phase != A16_WAIT) lost_ordinary_road = 0;
    track_mode16_straight = track_mode16_inside_px = 0;
    if (a16_plan_obstacle(&result,image)) return result;
#endif
#if TRACK_ALGORITHM == 16
    if (!a16_departure_mark_ms)
#endif
        a16_plan_island(&result);
#if TRACK_ALGORITHM == 16
    if (!a16_departure_mark_ms && track_mode16_phase == A16_ROAD && crossing_trunk(&result)) {
        track_mode16_corner_active = 0;
        return result;
    }
    track_mode16_corner_active = !a16_departure_mark_ms && track_mode16_phase == A16_ROAD &&
                                 track_mode16_corner(&result, track_mode16_corner_active);
    if (track_mode16_corner_active)
        return result;
    if (track_mode16_phase==A16_ROAD && result.valid_rows<TRACK_DRIVE_MIN_VALID_ROWS &&
        retrace_corner(&result)) return result;
#endif
#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
    if (track_mode16_phase == A16_ROAD) {
        /* Select the near-field rows only after the island planner has used
         * the original full-depth observation for its state transitions. */
#if TRACK_ALGORITHM == 16
        const int target_row = TRACK_A16_LOOKAHEAD_ROW;
        const int anchor_row = TRACK_A16_NEAR_ROW;
#else
        const int target_row = A16_ROAD_TARGET_ROW;
        const int anchor_row = A16_ROAD_ANCHOR_ROW;
#endif
        track_mode16_target_row = target_row;
        result.target_confidence = track_mode16_path_confidence[target_row];
        result.anchor_confidence = track_mode16_path_confidence[anchor_row];
#if TRACK_ALGORITHM == 16
        for (int y = anchor_row; y >= target_row; --y) {
            if (!track_mode16_path_confidence[y]) continue;
            int left = track_mode16_path_x[y], right = left;
            while (left > 2 && a16_binary_image[y][left-1]) --left;
            while (right < 185 && a16_binary_image[y][right+1]) ++right;
            /* Project a 21 cm half-lane at 23 cm height and 15 degrees down. */
            int half_width = 29 + (y - 60) * 88 / 100;
            int center = track_mode16_path_x[y];
            if (left <= 3 && right < 184)
                center = right - half_width;
            else if (right >= 184 && left > 3)
                center = left + half_width;
            if (center < left) center = left;
            if (center > right) center = right;
            track_mode16_path_x[y] = center;
        }
        /* A straight line projects to a straight image line even when the
         * car is yawed. Use three equally spaced rows to distinguish that
         * case from a bend, then taper inward shift to zero at the anchor. */
        if (track_mode16_path_confidence[60] && track_mode16_path_confidence[88] &&
            result.target_confidence && result.anchor_confidence) {
            int far = track_mode16_path_x[60], mid = track_mode16_path_x[88];
            int near = track_mode16_path_x[116], curve = far - 2*mid + near;
            track_mode16_straight = curve >= -5 && curve <= 5;
            int bias = 0;
            if (curve > 5 && far > mid+4) bias = (curve-3)/3;
            if (curve < -5 && far < mid-4) bias = (curve+3)/3;
            if (bias > 2) bias=2;
            if (bias < -2) bias=-2;
            for (int y=target_row; bias && y<=anchor_row; ++y) {
                if (!track_mode16_path_confidence[y]) continue;
                int x=track_mode16_path_x[y];
                int offset=bias*(anchor_row-y)/
                                (anchor_row-target_row);
                int guard=x+offset+(bias>0 ? 12 : -12);
                if (guard>=2 && guard<=185 && a16_binary_image[y][guard] && a16_binary_image[y][x+offset]) {
                    track_mode16_path_x[y]=x+offset;
                    if (y==target_row) track_mode16_inside_px=offset;
                }
            }
        }
#endif
        if (result.target_confidence)
            result.target_x = track_mode16_path_x[target_row];
        if (result.anchor_confidence)
            result.anchor_x = track_mode16_path_x[anchor_row];
    }
#endif
    return result;
}
