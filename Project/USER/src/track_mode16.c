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
#include "track_mode16.h"
#include "tune_config.h"
#include <string.h>
#include <math.h>

#ifndef TRACK_MODE16_GRAY_THRESHOLD
#define TRACK_MODE16_GRAY_THRESHOLD 232U
#endif

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

int track_mode16_corner_active;
#if TRACK_ALGORITHM == 16
int track_mode16_obstacle_side;
int track_mode16_inside_px;
int track_mode16_straight;
static int obstacle_vote;
static unsigned obstacle_hold_ms;
/* A stopped drive cannot clear the near-camera blind area. */
extern volatile uint32_t track_motor_duty;
#endif
static uint8_t binary_image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH];
static uint8_t binary_source[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH];
static uint16_t island_queue[TRACK_MODE1_HEIGHT * TRACK_MODE1_WIDTH];
static int16_t island_left[120], island_right[120];
static uint8_t island_votes, merge_votes;
#if TRACK_ALGORITHM == 16
static uint8_t approach_curve_frames;
#endif
static uint8_t island_open_candidate;
static uint32_t frame_dt_ms = 33U, missing_ms, cooldown_ms;
static int last_island_x, last_island_y, island_max_row, last_island_bottom;
static uint8_t island_passed;
volatile uint32_t track_mode16_phase;
volatile int32_t track_mode16_island_side;
volatile int32_t track_mode16_island_bottom;
volatile uint32_t track_mode16_target_row = TRACK_OTSUSCAN_TARGET_ROW;

/* A side-clipped island may be joined to the black image background, so it
 * cannot form a bounded dark connected component. Detect the visible cross-
 * section instead: two white road lobes separated by a coherent dark gap,
 * which closes back into a broad white roadway a few rows nearer the car.
 * This is deliberately a fallback observation; ordinary road tracing is not
 * changed and one isolated split row is never enough evidence. */
static int observe_open_island(void)
{
    int16_t gap_left[120], gap_right[120];
    int y, side = 0, count = 0, top = 120, bottom = -1, widest = 0;
    memset(gap_left, -1, sizeof(gap_left));
    memset(gap_right, -1, sizeof(gap_right));
    island_open_candidate = 0U;

    for (y = 20; y <= 76; ++y) {
        int runs_l[8], runs_r[8], nr = 0, x = 2;
        while (x <= 185 && nr < 8) {
            int l, r;
            while (x <= 185 && !binary_image[y][x]) ++x;
            if (x > 185) break;
            l = x;
            while (x <= 185 && binary_image[y][x]) ++x;
            r = x - 1;
            if (r - l + 1 >= 4) { runs_l[nr] = l; runs_r[nr] = r; ++nr; }
        }
        if (nr < 2) continue;
        for (int i = 0; i + 1 < nr; ++i) {
            int gl = runs_r[i] + 1, gr = runs_l[i+1] - 1;
            int lw = runs_r[i] - runs_l[i] + 1;
            int rw = runs_r[i+1] - runs_l[i+1] + 1;
            int center = (gl + gr) / 2, s = 0;
            if (gr - gl + 1 < 4 || gr - gl + 1 > 30) continue;
            if (center < 94 && lw >= 4 && rw >= 28) s = -1;
            if (center > 94 && rw >= 4 && lw >= 28) s = 1;
            if (!s) continue;
            /* Require the branch/gap to join the same road a short distance
             * below it, rather than treating arbitrary separated markings as
             * an island. */
            for (int yy = y + 1; yy <= y + 10 && yy <= 86; ++yy) {
                int q = 2, joined = 0;
                while (q <= 185) {
                    int l, r;
                    while (q <= 185 && !binary_image[yy][q]) ++q;
                    if (q > 185) break;
                    l = q;
                    while (q <= 185 && binary_image[yy][q]) ++q;
                    r = q - 1;
                    if (r - l + 1 >= 60 && l <= gl + 2 && r >= gr - 2) {
                        joined = 1; break;
                    }
                }
                if (joined) { gap_left[y] = (int16_t)gl; gap_right[y] = (int16_t)gr; break; }
            }
            if (gap_left[y] >= 0) break;
        }
    }

    /* Keep a coherent side gap over multiple scanlines. */
    for (y = 20; y <= 76; ++y) if (gap_left[y] >= 0) {
        int neighbors = 0;
        for (int yy = y - 2; yy <= y + 2; ++yy) {
            if (yy >= 20 && yy <= 76 && gap_left[yy] >= 0 &&
                ((gap_left[yy] + gap_right[yy]) / 2 < 94) ==
                ((gap_left[y] + gap_right[y]) / 2 < 94)) ++neighbors;
        }
        if (neighbors < 2) { gap_left[y] = gap_right[y] = -1; continue; }
        ++count;
        if (y < top) top = y;
        if (y > bottom) bottom = y;
        if (gap_right[y] - gap_left[y] + 1 > widest) widest = gap_right[y] - gap_left[y] + 1;
    }
    if (count < 3 || bottom - top < 3 || widest > 30) return 0;
#if TRACK_ALGORITHM == 16
    /* Before commitment, a split alone can be the triangular gap of an
     * oblique crossing. Require curvature: the visible gap widens and then
     * narrows. After entry its tip can leave the image on either side. */
    if (track_mode16_phase != A16_ENTRY && track_mode16_phase != A16_ARC) {
        int top_width = gap_right[top] - gap_left[top] + 1;
        int bottom_width = gap_right[bottom] - gap_left[bottom] + 1;
        if (widest - top_width < 3 || widest - bottom_width < 3) return 0;
    }
#endif
    side = (gap_left[top] + gap_right[top]) / 2 < 94 ? -1 : 1;
    if (track_mode16_phase >= A16_PREP && track_mode16_phase <= A16_ARC &&
        (side != track_mode16_island_side ||
         (island_passed && bottom + 22 < last_island_bottom))) return 0;

#if TRACK_ALGORITHM == 16
    /* A diagonal strip across a bend can leave two bright lobes. During
     * acquisition, reject a gap whose two edges sweep together faster
     * across the image than its vertical extent. */
    if (track_mode16_phase != A16_ENTRY && track_mode16_phase != A16_ARC) {
        int left_shift = gap_left[bottom] - gap_left[top];
        int right_shift = gap_right[bottom] - gap_right[top];
        int span = bottom - top;
        if ((left_shift > 0 && right_shift > 0 && (left_shift + right_shift) * 2 > span * 5) ||
            (left_shift < 0 && right_shift < 0 && -(left_shift + right_shift) * 2 > span * 5))
            return 0;
    }
#endif
    memset(island_left, 0xff, sizeof(island_left));
    memset(island_right, 0xff, sizeof(island_right));
    for (y = top; y <= bottom; ++y) {
        island_left[y] = gap_left[y];
        island_right[y] = gap_right[y];
    }
    island_max_row = top;
    track_mode16_island_bottom = bottom;
    side = (island_left[top] + island_right[top]) / 2 < 94 ? -1 : 1;
    if (track_mode16_phase == A16_ROAD || track_mode16_phase == A16_WAIT)
        track_mode16_island_side = side;
    last_island_x = (island_left[top] + island_right[top]) / 2;
    last_island_y = (top + bottom) / 2;
    island_open_candidate = 1U;
    return 1;
}

void track_mode16_set_frame_dt(uint32_t dt_ms)
{
    frame_dt_ms = dt_ms == 0U ? 1U : (dt_ms > 1000U ? 1000U : dt_ms);
}

#if TRACK_ALGORITHM == 16
/* For an island touching the image edge on every row, require a visible
 * far branch on that same side: white corridor, dark edge, white corridor. */
static int visible_far_branch(int top, int clipped_left)
{
    int col = clipped_left ? 3 : 184;
    int row = top - 1;
    while (row >= 16 && binary_image[row][col]) --row;
    int near_white = top - 1 - row;
    int far_dark_end = row;
    while (row >= 16 && !binary_image[row][col]) --row;
    int far_dark = far_dark_end - row;
    return near_white >= 2 && near_white <= 24 && far_dark >= 3 && far_dark <= 14 &&
           row >= 17 && binary_image[row][col] && binary_image[row-1][col];
}

/* Check only the newly admitted, severely cropped observations. Project
 * the visible contour with the 188 x 120 model, 75 degree horizontal field
 * of view, 23 cm optical height and 15 degree downward pitch. */
static int rounded_side_edge(const int16_t *edge, int top, int bottom, int clipped_left, float spread)
{
    enum { samples = 11 };
    float x[samples], y[samples], mean_x = 0.0f, mean_y = 0.0f;
    for (int i = 0; i < samples; ++i) {
        int row = top + (i * (bottom - top) + 5) / 10;
        if (edge[row] < 2 || edge[row] > 185) return 0;
        float u = ((float)edge[row] - 93.5f) / 122.5f;
        float v = (59.5f - (float)row) / 122.5f;
        float c = 1.0f / sqrtf(1.0f + u*u);
        float down = 0.258819f*c - 0.965926f*v;
        if (down < 0.12f) return 0;
        x[i] = 0.23f*u*c/down;
        y[i] = 0.23f*(0.965926f*c + 0.258819f*v)/down;
        mean_x += x[i];
        mean_y += y[i];
    }
    mean_x /= samples;
    mean_y /= samples;
    float xx=0.0f, yy=0.0f, xy=0.0f, xz=0.0f, yz=0.0f;
    for (int i = 0; i < samples; ++i) {
        x[i] -= mean_x;
        y[i] -= mean_y;
        float z = (x[i]*x[i] + y[i]*y[i]) * 0.5f;
        xx += x[i]*x[i]; yy += y[i]*y[i]; xy += x[i]*y[i];
        xz += x[i]*z; yz += y[i]*z;
    }
    float det = xx*yy - xy*xy;
    if (det < 1e-8f) return 0;
    float cx = (yy*xz - xy*yz)/det, cy = (xx*yz - xy*xz)/det;
    if (clipped_left ? cx >= 0.0f : cx <= 0.0f) return 0;
    float min_r = 10.0f, max_r = 0.0f;
    for (int i = 0; i < samples; ++i) {
        float dx = x[i]-cx, dy = y[i]-cy;
        float radius = sqrtf(dx*dx + dy*dy);
        if (radius < min_r) min_r = radius;
        if (radius > max_r) max_r = radius;
    }
    return min_r >= 0.12f && max_r <= 0.40f && max_r-min_r <= spread;
}

/* Classify a visible island relative to the incoming lane. An off-axis
 * camera can put a right-hand island in the left half of the frame. */
static int approach_reference_x(int bottom)
{
    int far_y = bottom + 6;
    if (far_y < 76) far_y = 76;
    int near_y = far_y + 20;
    if (near_y > 116 || !track_mode16_path_confidence[far_y] ||
        !track_mode16_path_confidence[near_y]) return 94;
    int far = track_mode16_path_x[far_y];
    int dx = (far - track_mode16_path_x[near_y]) * (far_y - bottom) / 20;
    if (dx > 35) dx = 35;
    if (dx < -35) dx = -35;
    return far + dx;
}
#endif

/* Dark components are only observations. Initial acquisition requires a dark
 * bulge with white road on both sides; a single clipped bend cannot acquire.
 * After association, the same island may be clipped by the image side. No
 * complete circle, centre position, odometry, or simulated yaw is assumed. */
static int observe_island(void)
{
    int y, x, best = -100000, found = 0;
    int16_t left[120], right[120];
    island_open_candidate = 0U;
    memset(binary_source, 0, sizeof(binary_source));
    for (y = 16; y <= 115; ++y) for (x = 2; x <= 185; ++x) {
        unsigned head = 0U, tail = 0U;
        int top = 119, bottom = 0, xmin = 188, xmax = 0;
        int support = 0, widest = 0, widest_row = y, narrowest = 188, score;
        int tracking = track_mode16_phase >= A16_PREP && track_mode16_phase <= A16_ARC;
        if (binary_image[y][x] || binary_source[y][x]) continue;
        memset(left, -1, sizeof(left)); memset(right, -1, sizeof(right));
        island_queue[tail++] = (uint16_t)(y * 188 + x);
        binary_source[y][x] = 1U;
        while (head < tail) {
            int p = island_queue[head++], yy = p / 188, xx = p % 188, dy, dx;
            if (left[yy] < 0 || xx < left[yy]) left[yy] = (int16_t)xx;
            if (xx > right[yy]) right[yy] = (int16_t)xx;
            if (yy < top) top = yy;
            if (yy > bottom) bottom = yy;
            if (xx < xmin) xmin = xx;
            if (xx > xmax) xmax = xx;
            for (dy = -1; dy <= 1; ++dy) for (dx = -1; dx <= 1; ++dx) {
                int ny = yy + dy, nx = xx + dx;
                if ((!dx && !dy) || ny < 16 || ny > 115 || nx < 2 || nx > 185) continue;
                if (!binary_image[ny][nx] && !binary_source[ny][nx]) {
                    binary_source[ny][nx] = 1U;
                    island_queue[tail++] = (uint16_t)(ny * 188 + nx);
                }
            }
        }
        if (tail < 100U || tail > 10000U || bottom - top < 6 || bottom < 28 ||
            xmax - xmin > 145 || bottom - top > (tracking ? 100 : 85)) continue;
#if TRACK_ALGORITHM == 16
        /* A distant island cannot become a near road shoulder in one vote.
         * Apply association only while confirming the first three views. */
        if (approach_curve_frames >= 2 && track_mode16_phase == A16_PREP &&
            (bottom > last_island_bottom + 20 || bottom + 20 < last_island_bottom))
            continue;
#endif
        /* Once alongside the island, a new tiny horizon component is not
         * evidence that the same nearby island jumped back into the distance. */
        if (tracking && island_passed && (bottom < 45 || bottom + 22 < last_island_bottom)) continue;
        for (int yy = top; yy <= bottom; ++yy) {
            int l = left[yy], r = right[yy], w = r-l+1;
            if (l < 0) continue;
            if (w > widest) { widest = w; widest_row = yy; }
            if (w < narrowest) narrowest = w;
            if (l >= 4 && r <= 183 && w >= 6 &&
                binary_image[yy][l-1] && binary_image[yy][r+1]) ++support;
        }
        if (widest < 10 || widest - narrowest < 4) continue;
        if (!tracking) {
#if TRACK_ALGORITHM == 16
            if (xmin <= 2 || xmax >= 185) {
                /* A partly clipped island still has a visible tip and an
                 * open corridor above and below it. Reject a black border
                 * that runs into either end of the observation window. */
                int clipped_left = xmin <= 2;
                if (top <= 16 || bottom >= 115 || widest < 18 ||
                    (clipped_left ? xmax >= 94 : xmin <= 93)) continue;
                int tip = clipped_left ? xmax : xmin;
                int upper = clipped_left ? tip - right[top] : left[top] - tip;
                int lower = clipped_left ? tip - right[bottom] : left[bottom] - tip;
                if (upper < 4 || lower < 6) continue;
                if (support < 4 && (bottom < 70 || top < 45 ||
                    (support == 0 && !visible_far_branch(top, clipped_left)) ||
                    upper < 8 || lower < 8 || bottom-top < 12 ||
                    !rounded_side_edge(clipped_left ? right : left, top, bottom, clipped_left,
                                       support ? 0.045f : 0.026f)))
                    continue;
            } else if (support < 6) continue;
#else
            if (xmin <= 2 || xmax >= 185 || support < 6) continue;
#endif
#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
            /* A marking across the near lane can clip the observation window
             * and look like an island. Apply this only to first acquisition. */
            int height = bottom - top + 1;
            int mid = (xmin + xmax) / 2;
            if (bottom == 115 && widest >= 70 && widest >= 4 * height &&
                mid >= 94 - widest / 6 && mid <= 94 + widest / 6) continue;
#endif
            score = bottom * 3 + support;
        } else {
            int cx = (xmin+xmax)/2, cy = (top+bottom)/2;
            int dx = cx-last_island_x, dy = cy-last_island_y;
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            if (dx > 45 || dy > 55) continue;
            if (track_mode16_island_side < 0 && xmax >= 183) continue;
            if (track_mode16_island_side > 0 && xmin <= 4) continue;
            score = 300 - dx*4 - dy + support;
        }
        if (score <= best) continue;
        best = score; found = 1;
        memcpy(island_left, left, sizeof(left)); memcpy(island_right, right, sizeof(right));
        track_mode16_island_bottom = bottom;
        island_max_row = widest_row;
        /* Do not update the association reference until all components scored. */
    }
    if (!found) found = observe_open_island();
    if (found && !island_open_candidate) {
        int l = 188, r = 0, t = 119, b = 0;
        for (y = 16; y <= 115; ++y) if (island_left[y] >= 0) {
            if (island_left[y] < l) l = island_left[y];
            if (island_right[y] > r) r = island_right[y];
            if (y < t) t = y;
            b = y;
        }
        if (track_mode16_phase == A16_ROAD || track_mode16_phase == A16_WAIT)
            track_mode16_island_side = (l+r)/2 < 94 ? -1 : 1;
#if TRACK_ALGORITHM == 16
        /* Recheck direction immediately before committing, once turn-in has
         * brought the camera back in line with the approach. */
        if (track_mode16_phase == A16_PREP && island_votes >= 2 &&
            l > 2 && r < 185 && b <= 75) {
            int center = (l+r)/2;
            int delta = center - approach_reference_x(b);
            int bend = track_mode16_path_x[100] - track_mode16_path_x[74];
            int sweeping = approach_curve_frames >= 2 && b <= 62 && track_mode16_path_confidence[100] &&
                           track_mode16_path_confidence[74];
            if (sweeping && bend >= 14 && delta >= 12 && center >= last_island_x+3)
                track_mode16_island_side = 1;
            else if (sweeping && bend <= -14 && delta <= -12 && center <= last_island_x-3)
                track_mode16_island_side = -1;
            else if (center >= 100 && delta >= 10 && center >= last_island_x)
                track_mode16_island_side = 1;
            else if (center <= 87 && delta <= -10 && center <= last_island_x)
                track_mode16_island_side = -1;
        }
#endif
        last_island_x = (l+r)/2; last_island_y = (t+b)/2;
    }
    if (found) last_island_bottom = track_mode16_island_bottom;
    return found;
}

#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
#include "a17_turn_samples.inc"
#else
#include "a16_turn_samples.inc"
#endif

int track_mode16_curve_command;
static unsigned curve_votes, split_votes;
enum exit_landmark { SEEK_OPENING, OPENING_SEEN, EDGE_ALONGSIDE };
static enum exit_landmark exit_landmark;
static unsigned opening_votes, exit_road_votes;

static int saw_bend;
volatile unsigned track_mode16_straight_samples;
static unsigned rearm_clear_frames;

static int sample_x(int turn, int i)
{
    int x = turn_samples[turn < 0 ? -turn : turn][i][0];
    return turn < 0 ? 187-x : x;
}

/* Score the visible prefix, including tyre clearance. A fork can offer two
 * good prefixes; prefer the one turning towards the confirmed island. */
static int plan_turn(track_mode1_result_t *out)
{
    int best = -100000, choice = 0, straight = 0;
    int side = track_mode16_island_side;
    for (int t = -14; t <= 14; ++t) {
        int k = t < 0 ? -t : t, count = 0, room = 0;
#if TRACK_ALGORITHM == 16
        int clearance = 0;
#endif
        for (int i = 0; i < 31; ++i) {
            int x = sample_x(t,i), y = turn_samples[k][i][1];
            if (y > 118) continue;
            int margin = 5 + y/18;
            if (x < margin+2 || x > 185-margin || !binary_image[y][x]) break;
            if (track_mode16_phase!=A16_MERGE && track_mode16_island_bottom >= y && island_left[y]>=0 &&
                (side<0 ? x<=island_right[y] : x>=island_left[y])) break;
            if (!binary_image[y][x-margin] || !binary_image[y][x+margin]) break;
            ++count;
#if TRACK_ALGORITHM == 16
            /* Reward space inside both lane edges along the near part of the arc.
             * Beyond the camera margin the available clearance is unknown. */
            if (track_mode16_phase == A16_ARC && count <= 16) {
                int step = 6 + y / 12;
                for (int band = 1; band <= 4; ++band) {
                    int dx = band * step;
                    if (x - dx < 2 || x + dx > 185 ||
                        !binary_image[y][x-dx] || !binary_image[y][x+dx]) break;
                    ++clearance;
                }
            }
#endif
            if (x >= margin*2+2 && x <= 185-margin*2 &&
                binary_image[y][x-margin*2] && binary_image[y][x+margin*2]) ++room;
        }
        if (!t) straight = count;
        int score;
#if TRACK_ALGORITHM == 16
        if (track_mode16_phase == A16_ARC) {
            score = (count > 22 ? 22 : count) * 13 + clearance * 3 + t * side * 5;
        } else
#endif
        {
            int bias = track_mode16_phase == A16_MERGE ? -k * 4 : t * side;
            score = (count > 18 ? 18 : count) * 13 + (room > 16 ? 16 : room) * 2
                  + bias * (track_mode16_phase == A16_ARC ? 8 : 1);
        }
        if (count >= 5 && score > best) {best=score; choice=t;}
    }
    track_mode16_straight_samples = straight;
    if (best == -100000) return 0;
    int k = choice < 0 ? -choice : choice, last_x = 94, last_y = 119;
    memset(track_mode16_path_confidence,0,sizeof(track_mode16_path_confidence));
    int target_y = -1, target_x = 94, rows = 0;
    for (int i=0;i<31;++i) {
        int x=sample_x(choice,i), y=turn_samples[k][i][1];
        if (y >= last_y) continue;
        if (x<2 || x>185 || !binary_image[y][x]) break;
        for (int yy=last_y-1; yy>=y; --yy) {
            int xx=x+(last_x-x)*(yy-y)/(last_y-y);
            if (!binary_image[yy][xx]) goto finished;
            track_mode16_path_x[yy]=xx; track_mode16_path_confidence[yy]=80; ++rows;
            if (target_y<0 || yy>=60) {target_x=xx;target_y=yy;}
        }
        last_y=y;last_x=x;
    }
finished:
    if (rows<27 || !track_mode16_path_confidence[105] || target_y<0) return 0;
    track_mode16_target_row=target_y;out->target_x=target_x;
    out->anchor_x=track_mode16_path_x[105];out->anchor_confidence=80;
    out->target_confidence=80;out->valid_rows=rows;
    /* Wheel angle versus servo linkage is not calibrated. Retain configured
     * servo travel and map signed arc curvature monotonically into it. */
    /* Preserve the curvature response near centre; allow tighter turns
     * to use the extra travel without amplifying straight-road noise. */
    track_mode16_curve_command = -choice * 12 / 10;
    if (track_mode16_curve_command > (int)TRACK_A16_MAX_STEER_DELTA_DEG)
        track_mode16_curve_command = TRACK_A16_MAX_STEER_DELTA_DEG;
    if (track_mode16_curve_command < -(int)TRACK_A16_MAX_STEER_DELTA_DEG)
        track_mode16_curve_command = -(int)TRACK_A16_MAX_STEER_DELTA_DEG;
    if (choice*side>=4 && straight<13) {if (++curve_votes>=6) saw_bend=1;}
    else curve_votes=0;
    if (saw_bend && straight>=25) ++split_votes;
    else split_votes=0;
    return 1;
}

/* Select the observed white corridor immediately outside the island edge.
 * After the island clips out, keep tracing that same side of current road;
 * never steer from a stale circle or a timer-only turn command. */
static int island_corridor(int y, int *left, int *right)
{
    int edge = track_mode16_island_side < 0 ? island_right[y] : island_left[y];
    int direction = track_mode16_island_side < 0 ? 1 : -1;
    int x, a, b;
    if (edge < 0) return 0;
    x = edge + direction;
    if (x < 2 || x > 185 || !binary_image[y][x]) return 0;
    a = b = x;
    while (a > 2 && binary_image[y][a-1]) --a;
    while (b < 185 && binary_image[y][b+1]) ++b;
    if (b-a+1 < 8) return 0;
    *left = a; *right = b; return 1;
}

/* In a flared entrance the outer boundary belongs to the straight branch.
 * Take an inward lane offset from the observed island edge. Cap the offset
 * by row depth so a wide fork cannot pull the target into the straight. */
static int island_lane_x(int y, int left, int right)
{
    int clearance = (right - left) / 3;
    int perspective_limit = 7 + y / 4;
    if (clearance > perspective_limit) clearance = perspective_limit;
    return track_mode16_island_side < 0 ? left + clearance : right - clearance;
}

#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
/* A cross has a continuous forward lane and a transverse white band that
 * opens beyond BOTH edges of that lane. One missing edge also occurs in bends. */
static int cross_before_entry(int cx)
{
    int votes = 0;
    for (int y = 40; y <= 108; y += 2) {
        int ref = y - 24, left = cx, right = cx;
        if (!binary_image[ref][cx] || !binary_image[y][cx]) { votes = 0; continue; }
        while (left > 2 && binary_image[ref][left-1]) --left;
        while (right < 185 && binary_image[ref][right+1]) ++right;
        if (right-left < 12 || left < 21 || right > 166) { votes = 0; continue; }
        int probe_left = left - 18, probe_right = right + 18;
#if TRACK_ALGORITHM == 16
        /* A cross opens on both sides and closes again toward the car.
         * A roundabout entrance can widen continuously on just one side. */
        int near_y = y + 24;
        if (near_y > 116 || !binary_image[near_y][cx]) { votes = 0; continue; }
        int near_left = cx, near_right = cx;
        while (near_left > 2 && binary_image[near_y][near_left-1]) --near_left;
        while (near_right < 185 && binary_image[near_y][near_right+1]) ++near_right;
        if (near_left - 7 < probe_left) probe_left = near_left - 7;
        if (near_right + 7 > probe_right) probe_right = near_right + 7;
        if (probe_left < 2 || probe_right > 185) { votes = 0; continue; }
#endif
        int x = probe_left;
        while (x <= probe_right && binary_image[y][x]) ++x;
        if (x > probe_right) {
            if (++votes >= 3) return 1;
        } else votes = 0;
    }
    return 0;
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
        while (left > 2 && binary_image[y][left-1]) --left;
        while (right < 185 && binary_image[y][right+1]) ++right;
        int opening = 0;
        if (left <= 5 && right >= 40 && right < 134) opening = -1;
        if (right >= 182 && left > 53 && left <= 147) opening = 1;
        /* Ahead of the transverse strip both the middle and the opening
         * must be dark. A continuing lane or a two-sided cross fails here. */
        int probe = opening < 0 ? 35 : 152;
        if (binary_image[y-24][94] ||
            binary_image[y-24][probe] || !binary_image[y][probe]) opening = 0;
        /* Around the apex the far boundary becomes diagonal. If the
         * ordinary anchor is lost, follow that measured edge through the turn. */
        if (y < A16_CORNER_START_ROW) opening = 0;
        if (!opening && weak_road) {
            int x = track_mode16_path_x[y-12];
            if (track_mode16_path_confidence[y-12]) {
                int lo = x, hi = x;
                while (lo > 2 && binary_image[y-12][lo-1]) --lo;
                while (hi < 185 && binary_image[y-12][hi+1]) ++hi;
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
        while (l0 > 2 && binary_image[y0][l0-1]) --l0;
        while (r0 < 185 && binary_image[y0][r0+1]) ++r0;
        while (l1 > 2 && binary_image[y1][l1-1]) --l1;
        while (r1 < 185 && binary_image[y1][r1+1]) ++r1;
        int wl = l0 - l1, wr = r1 - r0;
        if (l1 < 8 || r1 > 179 || r0-l0 < 14 || wl < 4 || wr < 4 ||
            wl > 18 || wr > 18 || x1-x0 < -6 || x1-x0 > 6) continue;
        int left_votes = 0, right_votes = 0;
        for (int y = y1 + 4; y <= 116; y += 2) {
            int delta = y - y1;
            int l = l1 - wl * delta / 12 - 14, r = r1 + wr * delta / 12 + 14;
            int x = x1 + (x1-x0) * delta / 12;
            if (x < 2 || x > 185 || !binary_image[y][x]) break;
            int li = x, ri = x;
            while (li > 2 && binary_image[y][li-1]) --li;
            while (ri < 185 && binary_image[y][ri+1]) ++ri;
            if (li <= l) ++left_votes;
            if (ri >= r) ++right_votes;
            if (left_votes < 3 || right_votes < 3) continue;
            int near_x = x1 + (x1 - x0) * (116 - y1) / 12;
            if (near_x < 2 || near_x > 185 || !binary_image[116][near_x]) break;
            /* Verify every center pixel before replacing the measured path. */
            int yy;
            for (yy = y1; yy <= 116; ++yy) {
                int xx = x1 + (x1 - x0) * (yy - y1) / 12;
                if (!binary_image[yy][xx]) break;
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

static void plan_island(track_mode1_result_t *out)
{
    if (track_mode16_phase==A16_MERGE) {
        cooldown_ms=cooldown_ms>frame_dt_ms?cooldown_ms-frame_dt_ms:0;
        if (!cooldown_ms && out->target_confidence>=60 && out->anchor_confidence>=60 && out->valid_rows>=55) {
            if (merge_votes < 3) ++merge_votes;
        } else merge_votes=0;
        if (merge_votes>=3) {
            merge_votes=0;
            track_mode16_phase=A16_ROAD;
        } else {
            track_mode16_island_bottom=-1;
            if (!plan_turn(out)) out->target_confidence=0;
            return;
        }
    }
#if TRACK_ALGORITHM == 16
    int committed_now = 0;
    if (track_mode16_phase == A16_ROAD) {
        int drift = track_mode16_path_x[74] - track_mode16_path_x[102];
        if (track_mode16_path_confidence[74] && track_mode16_path_confidence[102] &&
            (drift >= 20 || drift <= -20)) {
            if (approach_curve_frames < 8) ++approach_curve_frames;
        } else if (approach_curve_frames) --approach_curve_frames;
    } else if (track_mode16_phase != A16_PREP) approach_curve_frames = 0;
#endif
    int observed=observe_island();
#if TRACK_ALGORITHM == 16 || TRACK_ALGORITHM == 17
    if (observed && (track_mode16_phase == A16_ROAD || track_mode16_phase == A16_PREP ||
                     track_mode16_phase == A16_WAIT)) {
        int cx = out->anchor_confidence >= 40 ? out->anchor_x : TRACK_STEER_REFERENCE_X;
        if (cross_before_entry(cx)) {
            observed = 0;
            track_mode16_island_side = 0;
        }
    }
#endif
    if (!observed) track_mode16_island_bottom=-1;
    if (rearm_clear_frames) {
        /* Pass the old mouth completely before looking for a new island. */
        rearm_clear_frames = observed ? 8U : rearm_clear_frames-1U;
        track_mode16_island_side=0;
        return;
    }
    if (track_mode16_phase==A16_WAIT) {
        if (observed) {track_mode16_phase=A16_PREP;island_votes=0;}
        else {
            if (out->valid_rows>=55 && out->target_confidence>=60) ++merge_votes;
            else merge_votes=0;
            if (merge_votes>=8) {track_mode16_phase=A16_ROAD;merge_votes=0;}
            else out->target_confidence=0;
            return;
        }
    }
    if (track_mode16_phase==A16_ROAD) {
        if (!observed) return;
        track_mode16_phase=A16_PREP; island_votes=0; island_passed=0;
        curve_votes=split_votes=opening_votes=exit_road_votes=0;
        saw_bend=0; exit_landmark=SEEK_OPENING; missing_ms=0;
    }
    if (track_mode16_phase==A16_PREP) {
        if (!observed) {island_votes=0;track_mode16_phase=A16_ROAD;return;}
        if (++island_votes<3 || track_mode16_island_bottom<30) return;
        track_mode16_phase=A16_ENTRY;
#if TRACK_ALGORITHM == 16
        committed_now = 1;
#endif
    }
#if TRACK_ALGORITHM == 16
    if (track_mode16_island_bottom>=98) {
        if (!committed_now) track_mode16_phase=A16_ARC;
        island_passed=1;
    } else if (island_passed) track_mode16_phase=A16_ARC;
#else
    if (track_mode16_island_bottom>=98) {track_mode16_phase=A16_ARC;island_passed=1;}
#endif
    if (!island_passed && observed) {
        for (int y=island_max_row;y<=track_mode16_island_bottom;++y) {
            int l,r;
            if (island_corridor(y,&l,&r) && out->anchor_confidence>=25 && out->valid_rows>=27) {
                int tx=island_lane_x(y,l,r);
                track_mode16_target_row=y;out->target_x=tx;out->target_confidence=80;
                track_mode16_path_x[y]=tx;track_mode16_path_confidence[y]=80;
                track_mode16_curve_command=((int)TRACK_STEER_REFERENCE_X-tx)/4;
                if(track_mode16_curve_command>4)track_mode16_curve_command=4;
                if(track_mode16_curve_command< -4)track_mode16_curve_command=-4;
                return;
            }
        }
    }
    int reliable_road = out->target_confidence >= 60 && out->anchor_confidence >= 60 && out->valid_rows >= 55;
    if (!plan_turn(out)) {
        out->target_confidence=0;merge_votes=0;track_mode16_phase=A16_WAIT;return;
    }
    /* A curved exit may never offer 25 straight samples. Confirm a shorter
     * opening by its sequence of visual landmarks before yielding the lane. */
    if (island_passed && saw_bend) {
        if (exit_landmark==SEEK_OPENING) {
            if (!observed && track_mode16_straight_samples >= 20) ++opening_votes;
            else opening_votes=0;
            if (opening_votes>=3) exit_landmark=OPENING_SEEN;
        } else if (exit_landmark==OPENING_SEEN && observed && track_mode16_island_bottom>=98) {
            exit_landmark=EDGE_ALONGSIDE;
        } else if (exit_landmark==EDGE_ALONGSIDE) {
            if (!observed && reliable_road) ++exit_road_votes;
            else exit_road_votes=0;
        }
    }
    if (island_passed && (split_votes>=3 || exit_road_votes>=3)) {
        merge_votes=0;track_mode16_phase=A16_MERGE;cooldown_ms=1200;rearm_clear_frames=8;track_mode16_island_side=0;
    }
}

int16_t track_mode16_path_x[TRACK_MODE1_HEIGHT];
uint8_t track_mode16_path_confidence[TRACK_MODE1_HEIGHT];

static void make_binary(
    const uint8_t image[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH], uint8_t threshold)
{
    uint16_t x, y;
    for (y = 0U; y < TRACK_MODE1_HEIGHT; ++y)
        for (x = 0U; x < TRACK_MODE1_WIDTH; ++x)
            binary_image[y][x] = image[y][x] > threshold ? 255U : 0U;
    for (y = 0U; y < TRACK_MODE1_HEIGHT; ++y) {
        binary_image[y][0] = 0U;
        binary_image[y][1] = 0U;
        binary_image[y][TRACK_MODE1_WIDTH - 2U] = 0U;
        binary_image[y][TRACK_MODE1_WIDTH - 1U] = 0U;
    }
    for (x = 0U; x < TRACK_MODE1_WIDTH; ++x) {
        binary_image[0][x] = 0U;
        binary_image[1][x] = 0U;
    }
}

/* Match the reference package's 8-neighbor binary cleanup (center excluded). */
static void filter_binary(void)
{
    uint16_t x, y;
    memcpy(binary_source, binary_image, sizeof(binary_image));
    for (y = 1U; y + 1U < TRACK_MODE1_HEIGHT; ++y) {
        for (x = 1U; x + 1U < TRACK_MODE1_WIDTH; ++x) {
            uint8_t count = 0U;
            int8_t dx, dy;
            for (dy = -1; dy <= 1; ++dy)
                for (dx = -1; dx <= 1; ++dx)
                    if ((dx != 0 || dy != 0) &&
                        binary_source[(int)y + dy][(int)x + dx] != 0U)
                        ++count;
            if (binary_source[y][x] == 0U && count >= 5U)
                binary_image[y][x] = 255U;
            else if (binary_source[y][x] != 0U && count <= 2U)
                binary_image[y][x] = 0U;
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
        while (x < (int)TRACK_MODE1_WIDTH - 2 && binary_image[y][x] == 0U) ++x;
        left = x;
        while (x < (int)TRACK_MODE1_WIDTH - 2 && binary_image[y][x] != 0U) ++x;
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
    obstacle_vote = track_mode16_obstacle_side = 0;
    obstacle_hold_ms = 0;
    track_mode16_straight = track_mode16_inside_px = 0;
#endif
    track_mode16_corner_active = 0;
    track_mode16_phase = A16_ROAD;
    track_mode16_island_side = 0;
    track_mode16_island_bottom = -1;
#if TRACK_ALGORITHM == 16
    approach_curve_frames = 0U;
#endif
    island_votes = merge_votes = island_passed = 0U;
    island_open_candidate = 0U;
    rearm_clear_frames = 0U; curve_votes=split_votes=0; saw_bend=0;
    missing_ms = cooldown_ms = 0U;
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
            output[y][x] = binary_image[y][x];
    (void)input;
}

#if TRACK_ALGORITHM == 16
/* Expected visible lane width in the 188 x 120, 23 cm / 15 degree view. */
/* At a tight L-turn the bottom run may fill the frame while the next
 * connected run moves more than the normal 40 pixel association limit.
 * Retry only a lost ROAD path, requiring a measured continuous exit on
 * the same side throughout the near field and a dark region ahead. */
static int retrace_corner(track_mode1_result_t *road)
{
    int y=TRACK_MODE1_BOTTOM_ROW;
    if (!binary_image[y][15] || !binary_image[y][94] || !binary_image[y][172]) return 0;
    int l,r,side;
    y=A16_ROAD_TARGET_ROW;
    if (binary_image[y][176] && !binary_image[y][12]) side=1;
    else if (binary_image[y][12] && !binary_image[y][176]) side=-1;
    else return 0;
    int probe=side>0 ? 176 : 12;
    if (binary_image[70][94] || binary_image[70][probe]) return 0;
    int16_t centers[33];
    int target=0;
    for (y=116; y>=84; --y) {
        if (!binary_image[y][probe]) return 0;
        l=r=probe;
        while (l>2 && binary_image[y][l-1]) --l;
        while (r<185 && binary_image[y][r+1]) ++r;
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

static int projected_lane_width(int row)
{
    return (row - 31) * 19 / 10;
}

static int gray_span(const uint8_t *row, int seed, int floor, int *lo, int *hi)
{
    if (seed < 2 || seed > 185 || row[seed] < floor) return 0;
    int l = seed, r = seed;
    while (l > 2 && row[l-1] >= floor) --l;
    while (r < 185 && row[r+1] >= floor) ++r;
    *lo = l; *hi = r;
    return r-l+1;
}

/* Look for a one-sided step in lane width with material lighter than the
 * background and darker than the road. A normal curved silhouette alone
 * does not establish an obstacle. */
/* If the brick and the background have similar brightness, also require
 * the full lane to reappear beyond the notch and the opposite boundary
 * to align with one straight line through near, blocked and far rows. */
static int bracketed_notch(const uint8_t image[][TRACK_MODE1_WIDTH], int y,
                           int side, int floor, int l, int r, int nl, int nr)
{
    for (int far=y-10; far>=42 && far>=y-32; far-=2) {
        if (!track_mode16_path_confidence[far]) continue;
        int fl,fr;
        int fw=gray_span(image[far],track_mode16_path_x[far],floor,&fl,&fr);
        int expected=projected_lane_width(far);
        if (fw*100<expected*87 || fw*100>expected*120 || fl<=2 || fr>=185) continue;
        int span=y+12-far;
        int left_ref=fl+(nl-fl)*(y-far)/span, right_ref=fr+(nr-fr)*(y-far)/span;
        int opposite=side<0 ? r-right_ref : l-left_ref;
        int inward=side<0 ? l-left_ref : right_ref-r;
        int w=right_ref-left_ref;
        /* Both clear lane edges must continue straight on the near side.
         * The mouth of a right-angle bend can also narrow one scanline. */
        int near2=y+20, nl2,nr2;
        if (near2>116 || !track_mode16_path_confidence[near2] ||
            !gray_span(image[near2],track_mode16_path_x[near2],floor,&nl2,&nr2)) continue;
        int err_l=nl2-(nl+(nl-fl)*8/span),err_r=nr2-(nr+(nr-fr)*8/span);
        if ((nl2>2 && (err_l < -5 || err_l > 5)) ||
            (nr2<185 && (err_r < -5 || err_r > 5))) continue;
        if (opposite>=-3 && opposite<=3 && inward*100>w*22 && inward*100<w*44)
            return 1;
    }
    return 0;
}

static int obstacle_observation(const uint8_t image[][TRACK_MODE1_WIDTH])
{
    int previous = 0;
    for (int y = 50; y <= 98; y += 2) {
        const int near = y+12, width = projected_lane_width(y);
        if (!track_mode16_path_confidence[y] || !track_mode16_path_confidence[near]) {
            previous = 0; continue;
        }
        int middle = track_mode16_path_x[y], ref = track_mode16_path_x[near];
        int floor = (int)image[y][middle] - 27;
        if (floor < 130) { previous = 0; continue; }
        if (floor > 172) floor = 172;
        int l, r, nl, nr;
        int w = gray_span(image[y],middle,floor,&l,&r);
        int nw = gray_span(image[near],ref,floor,&nl,&nr);
        int side = 0;
        if (w*100 >= width*48 && w*100 <= width*82 &&
            nw*100 >= projected_lane_width(near)*86 && nw*100 <= projected_lane_width(near)*120) {
            int dl = l-nl, dr = nr-r;
            if (dl-dr > width/5 && dr >= -4 && dr <= 23) side = -1;
            if (dr-dl > width/5 && dl >= -4 && dl <= 23) side = 1;
            if (side) {
                int edge = side < 0 ? l : r;
                int material = edge + side*(width/7);
                int outside = edge + side*(width/2 + 4);
                if (outside < 1 || outside > 186) side = 0;
                else {
                    int tone = image[y][material];
                    if (tone > floor-4 || !bracketed_notch(image,y,side,floor,l,r,nl,nr))
                        side=0;
                }
            }
        }
        if (side && side == previous) return side;
        previous = side;
    }
    return 0;
}

/* Use the measured edge on the unobstructed side to follow the middle of
 * the remaining two-thirds of the lane, including the near blind area. */
static int plan_obstacle(track_mode1_result_t *road, const uint8_t image[][TRACK_MODE1_WIDTH])
{
    int side = track_mode16_corner_active ? 0 : obstacle_observation(image);
    if (side && side == obstacle_vote && (!track_mode16_obstacle_side || side == track_mode16_obstacle_side)) {
        track_mode16_obstacle_side = side;
        obstacle_hold_ms = 1800;
    } else if (track_mode16_obstacle_side && track_motor_duty) {
        obstacle_hold_ms = obstacle_hold_ms > frame_dt_ms ? obstacle_hold_ms-frame_dt_ms : 0;
        if (!obstacle_hold_ms) track_mode16_obstacle_side = 0;
    }
    obstacle_vote = side;
    if (!track_mode16_obstacle_side) return 0;
    if (track_mode16_path_confidence[A16_ROAD_ANCHOR_ROW] &&
        track_mode16_path_confidence[A16_ROAD_TARGET_ROW]) {
        int target_valid=0, anchor_valid=0;
        for (int y=A16_ROAD_TARGET_ROW; y<=A16_ROAD_ANCHOR_ROW; ++y) {
            if (!track_mode16_path_confidence[y]) continue;
            int seed=track_mode16_path_x[y],l,r;
            int floor=(int)image[y][seed]-27;
            if (floor<115) continue;
            if (floor>172) floor=172;
            int width=gray_span(image[y],seed,floor,&l,&r);
            if (width<projected_lane_width(y)/2) continue;
            if ((track_mode16_obstacle_side<0 && r>=185) ||
                (track_mode16_obstacle_side>0 && l<=2)) continue;
            if (y==(int)A16_ROAD_TARGET_ROW) target_valid=1;
            if (y==(int)A16_ROAD_ANCHOR_ROW) anchor_valid=1;
            int x=track_mode16_obstacle_side<0 ? r-projected_lane_width(y)/3 :
                                                l+projected_lane_width(y)/3;
            if (x<l) x=l;
            if (x>r) x=r;
            track_mode16_path_x[y]=x;
        }
        track_mode16_target_row=A16_ROAD_TARGET_ROW;
        road->target_x=track_mode16_path_x[A16_ROAD_TARGET_ROW];
        road->anchor_x=track_mode16_path_x[A16_ROAD_ANCHOR_ROW];
        road->target_confidence=target_valid ? track_mode16_path_confidence[A16_ROAD_TARGET_ROW] : 0;
        road->anchor_confidence=anchor_valid ? track_mode16_path_confidence[A16_ROAD_ANCHOR_ROW] : 0;
    } else road->target_confidence=0;
    track_mode16_corner_active=0;
    return 1;
}
#endif

#if TRACK_ALGORITHM == 16
/* Keep race state across vision resets (blank frames and temporary line loss). */
unsigned track16_lap;
static unsigned clear_drive_ms;
static unsigned departure_mark_ms;
static int last_pattern, last_center;
extern volatile uint32_t track_arm_request;

void track16_new_lap(void)
{
    track16_lap = A16_LAP_DEPART;
    clear_drive_ms = 0;
    departure_mark_ms = 0;
    last_pattern = last_center = 0;
}

/* A group consists of four dark bars bounded by lighter pavement. A single
 * wide band, an open lane edge or an isolated notch cannot make a group. */
static int finish_group(const uint8_t *p, int stride, int first, int end,
                        int dark, int light, int *center, int *last)
{
    int runs=0, previous_end=0, previous_width=0, previous_gap=0, group_start=0;
    int white_peak=0;
    for (int i=first; i<end;) {
        if (p[i*stride] > dark) {
            if (p[i*stride] > white_peak) white_peak=p[i*stride];
            ++i;
            continue;
        }
        int start=i;
        do { ++i; } while (i<end && p[i*stride]<=dark);
        int width=i-start, gap=start-previous_end;
        if (width>17 || white_peak<light || i==end) {
            runs=0; white_peak=0; continue;
        }
        if (!runs || gap>20 || gap>3*width+2 || width>3*gap+2 ||
            width>2*previous_width+2 || previous_width>2*width+2 ||
            (runs>=2 && (gap>2*previous_gap+2 || previous_gap>2*gap+2))) {
            runs=0; group_start=start;
        }
        ++runs;
        previous_end=i; previous_width=width; previous_gap=gap; white_peak=0;
        if (runs>=4 && i-group_start>=12) {
            /* Check the white pavement immediately beyond the fourth bar. */
            for (int j=i;j<end && j<=i+3 && p[j*stride]>dark;++j) {
                if (p[j*stride]>=light) {
                    *center=(group_start+i)/2; *last=i; return 1;
                }
            }
        }
    }
    return 0;
}

/* Scan both along and across the road: the photo alone does not determine
 * which way the bars will appear to the onboard camera. */
static int finish_pattern(const uint8_t image[][TRACK_MODE1_WIDTH], int dark,
                          int light, int *center, int *bottom)
{
    for (int along_y=0;along_y<=1;++along_y) {
        int count=0, previous=-1;
        for (int scan=along_y?34:36;scan<=(along_y?154:112);scan+=4) {
            int middle=0,end=0;
            const uint8_t *p=along_y?(const uint8_t *)(const void *)image+scan:image[scan];
            int ok=finish_group(p,along_y?TRACK_MODE1_WIDTH:1,along_y?30:14,
                                along_y?116:174,dark,light,&middle,&end);
            if (!ok || (!along_y && (middle<40 || middle>148))) {
                count=0; continue;
            }
            count=(count && middle>=previous-6 && middle<=previous+6)?count+1:1;
            previous=middle;
            if (count>=3) {
                *center=middle; *bottom=along_y?end:scan;
                if (*bottom>=58) return along_y+1;
            }
        }
    }
    return 0;
}

static void update_lap(const uint8_t image[][TRACK_MODE1_WIDTH], unsigned low,
                       unsigned high)
{
    if (track16_lap==A16_LAP_FINISHED || !track_arm_request) return;
    int center=0,bottom=0,pattern=0;
    if (high>=low+5) {
        int dark=(int)(low*8+(high-low)*3);
        pattern=finish_pattern(image,dark,dark+20,&center,&bottom);
    }
    if (track16_lap==A16_LAP_DEPART) {
        if (pattern) {
            if (!departure_mark_ms) track_mode16_reset();
            departure_mark_ms=1000;
        } else {
            departure_mark_ms=departure_mark_ms>frame_dt_ms ? departure_mark_ms-frame_dt_ms : 0;
        }
        if (pattern || !track_motor_duty || frame_dt_ms>TRACK_CAMERA_LOST_TIMEOUT_MS || high<low+3)
            clear_drive_ms=0;
        else if ((clear_drive_ms+=frame_dt_ms)>=2500U)
            track16_lap=A16_LAP_ARMED;
    } else if (pattern && pattern==last_pattern && center>=last_center-18 &&
               center<=last_center+18 && frame_dt_ms<=TRACK_CAMERA_LOST_TIMEOUT_MS) {
        track16_lap=A16_LAP_FINISHED;
    }
    last_pattern=pattern;
    last_center=center;
}

/* Remove the stripe gaps for departure only. Require actual white rails on
 * both sides, enclosing the camera axis, at a plausible projected lane width. */
static void join_departure_rails(void)
{
    for (int y=48;y<=118;++y) {
        int left=2,right=185;
        while (left<94 && !(binary_image[y][left] && binary_image[y][left+1])) ++left;
        while (right>94 && !(binary_image[y][right] && binary_image[y][right-1])) --right;
        int width=right-left+1,expected=projected_lane_width(y);
        if (left>=90 || right<=98 || width<expected*7/10 || width>expected*3/2) continue;
        memset(&binary_image[y][left],255,(size_t)width);
    }
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
#else
    if (high*8+7 < result.global_threshold && high>=low+10)
        result.global_threshold=(uint8_t)((low+high)*4+4);
#endif
#if TRACK_ALGORITHM == 16
    update_lap(image,low,high);
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
        track_mode16_reset();
        track_mode16_phase = A16_WAIT;
        track_mode16_target_row = A16_ROAD_TARGET_ROW;
        return result;
    }
#endif
    filter_binary();
#if TRACK_ALGORITHM == 16
    if (departure_mark_ms) join_departure_rails();
#endif
    track_mode16_target_row = TRACK_OTSUSCAN_TARGET_ROW;
    memset(track_mode16_path_confidence, 0, sizeof(track_mode16_path_confidence));
    /* Seed from the bottom row's white run nearest the image center. */
    for (x = 2; x < (int)TRACK_MODE1_WIDTH - 2;) {
        int left, right, center, width, distance;
        while (x < (int)TRACK_MODE1_WIDTH - 2 && binary_image[seed_y][x] == 0U) ++x;
        left = x;
        while (x < (int)TRACK_MODE1_WIDTH - 2 && binary_image[seed_y][x] != 0U) ++x;
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
    track_mode16_straight = track_mode16_inside_px = 0;
    if (!departure_mark_ms && track_mode16_phase == A16_ROAD && plan_obstacle(&result,image)) return result;
    if (track_mode16_phase != A16_ROAD) obstacle_vote = 0;
#endif
#if TRACK_ALGORITHM == 16
    if (!departure_mark_ms)
#endif
        plan_island(&result);
#if TRACK_ALGORITHM == 16
    if (!departure_mark_ms && track_mode16_phase == A16_ROAD && crossing_trunk(&result)) {
        track_mode16_corner_active = 0;
        return result;
    }
    track_mode16_corner_active = !departure_mark_ms && track_mode16_phase == A16_ROAD &&
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
        track_mode16_target_row = A16_ROAD_TARGET_ROW;
        result.target_confidence = track_mode16_path_confidence[A16_ROAD_TARGET_ROW];
        result.anchor_confidence = track_mode16_path_confidence[A16_ROAD_ANCHOR_ROW];
#if TRACK_ALGORITHM == 16
        for (int y = A16_ROAD_ANCHOR_ROW; y >= (int)A16_ROAD_TARGET_ROW; --y) {
            if (!track_mode16_path_confidence[y]) continue;
            int left = track_mode16_path_x[y], right = left;
            while (left > 2 && binary_image[y][left-1]) --left;
            while (right < 185 && binary_image[y][right+1]) ++right;
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
            for (int y=A16_ROAD_TARGET_ROW; bias && y<=A16_ROAD_ANCHOR_ROW; ++y) {
                if (!track_mode16_path_confidence[y]) continue;
                int x=track_mode16_path_x[y];
                int offset=bias*((int)A16_ROAD_ANCHOR_ROW-y)/
                                ((int)A16_ROAD_ANCHOR_ROW-(int)A16_ROAD_TARGET_ROW);
                int guard=x+offset+(bias>0 ? 12 : -12);
                if (guard>=2 && guard<=185 && binary_image[y][guard] && binary_image[y][x+offset]) {
                    track_mode16_path_x[y]=x+offset;
                    if (y==(int)A16_ROAD_TARGET_ROW) track_mode16_inside_px=offset;
                }
            }
        }
#endif
        if (result.target_confidence)
            result.target_x = track_mode16_path_x[A16_ROAD_TARGET_ROW];
        if (result.anchor_confidence)
            result.anchor_x = track_mode16_path_x[A16_ROAD_ANCHOR_ROW];
    }
#endif
    return result;
}
