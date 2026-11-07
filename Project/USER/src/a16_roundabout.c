#include "a16_internal.h"
#include <string.h>
#include <math.h>

static uint16_t island_queue[TRACK_MODE1_HEIGHT * TRACK_MODE1_WIDTH];
static int16_t island_left[120], island_right[120];
#if TRACK_HAS_A16_ELEMENTS
#define ENTRY_CONFIRM_FRAMES TRACK_A16_ENTRY_CONFIRM_FRAMES
#else
#define ENTRY_CONFIRM_FRAMES 3U
#endif
static uint8_t island_votes, merge_votes;
#if TRACK_HAS_A16_ELEMENTS
static uint8_t approach_curve_frames;
#endif
static uint8_t island_open_candidate;
static uint32_t missing_ms, cooldown_ms;
static int last_island_x, last_island_y, island_max_row, last_island_bottom;
static uint8_t island_passed;
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
            while (x <= 185 && !a16_binary_image[y][x]) ++x;
            if (x > 185) break;
            l = x;
            while (x <= 185 && a16_binary_image[y][x]) ++x;
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
                    while (q <= 185 && !a16_binary_image[yy][q]) ++q;
                    if (q > 185) break;
                    l = q;
                    while (q <= 185 && a16_binary_image[yy][q]) ++q;
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
#if TRACK_HAS_A16_ELEMENTS
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

#if TRACK_HAS_A16_ELEMENTS
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

#if TRACK_HAS_A16_ELEMENTS
/* For an island touching the image edge on every row, require a visible
 * far branch on that same side: white corridor, dark edge, white corridor. */
static int visible_far_branch(int top, int clipped_left)
{
    int col = clipped_left ? 3 : 184;
    int row = top - 1;
    while (row >= 16 && a16_binary_image[row][col]) --row;
    int near_white = top - 1 - row;
    int far_dark_end = row;
    while (row >= 16 && !a16_binary_image[row][col]) --row;
    int far_dark = far_dark_end - row;
    return near_white >= 2 && near_white <= 24 && far_dark >= 3 && far_dark <= 14 &&
           row >= 17 && a16_binary_image[row][col] && a16_binary_image[row-1][col];
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

/* Fit the ground-plane circle using only visible boundary pixels. */
static int island_shape(const int16_t *left, const int16_t *right, int top, int bottom)
{
    float xs[112], ys[112], mx=0, my=0;
    int n=0;
    if (bottom<53 || top<=20 || bottom>=115) return 0;
    for (int y=top+1;y<bottom;y+=2) {
        for (int side=0;side<2;++side) {
            int x=side ? right[y] : left[y];
            if (x<4 || x>183) continue;
            float u=(x-93.5f)/122.5f, v=(59.5f-y)/122.5f;
            float c=1.0f/sqrtf(1+u*u), down=.258819f*c-.965926f*v;
            if (down<.13f) continue;
            xs[n]=.23f*u*c/down;
            ys[n]=.23f*(.965926f*c+.258819f*v)/down;
            mx+=xs[n]; my+=ys[n]; ++n;
        }
    }
    if (n<8) return 0;
    mx/=n; my/=n;
    float xx=0,yy=0,xy=0,xz=0,yz=0;
    for(int i=0;i<n;++i){
        xs[i]-=mx;ys[i]-=my;
        float z=(xs[i]*xs[i]+ys[i]*ys[i])*.5f;
        xx+=xs[i]*xs[i];yy+=ys[i]*ys[i];xy+=xs[i]*ys[i];
        xz+=xs[i]*z;yz+=ys[i]*z;
    }
    float det=xx*yy-xy*xy;
    if(det<1e-7f)return 0;
    float cx=(yy*xz-xy*yz)/det,cy=(xx*yz-xy*xz)/det;
    float rsum=0,r2sum=0;
    for(int i=0;i<n;++i){
        float dx=xs[i]-cx,dy=ys[i]-cy,rr=dx*dx+dy*dy;
        rsum+=sqrtf(rr);r2sum+=rr;
    }
    float radius=rsum/n;
    return radius>.14f && radius<.39f && cy+my<1.25f && cy+my>.12f &&
           fabsf(cx+mx)<.80f && r2sum/n-radius*radius<.00036f;
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
    memset(a16_binary_scratch, 0, sizeof(a16_binary_scratch));
    for (y = 16; y <= 115; ++y) for (x = 2; x <= 185; ++x) {
        unsigned head = 0U, tail = 0U;
        int top = 119, bottom = 0, xmin = 188, xmax = 0;
        int support = 0, widest = 0, widest_row = y, narrowest = 188, score;
        int tracking = track_mode16_phase >= A16_PREP && track_mode16_phase <= A16_ARC;
        if (a16_binary_image[y][x] || a16_binary_scratch[y][x]) continue;
        memset(left, -1, sizeof(left)); memset(right, -1, sizeof(right));
        island_queue[tail++] = (uint16_t)(y * 188 + x);
        a16_binary_scratch[y][x] = 1U;
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
                if (!a16_binary_image[ny][nx] && !a16_binary_scratch[ny][nx]) {
                    a16_binary_scratch[ny][nx] = 1U;
                    island_queue[tail++] = (uint16_t)(ny * 188 + nx);
                }
            }
        }
        if (tail < 100U || tail > 10000U || bottom - top < 6 || bottom < 28 ||
            xmax - xmin > 145 || bottom - top > (tracking ? 100 : 85)) continue;
#if TRACK_HAS_A16_ELEMENTS
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
                a16_binary_image[yy][l-1] && a16_binary_image[yy][r+1]) ++support;
        }
        if (widest < 10 || widest - narrowest < 4) continue;
        if (!tracking) {
#if TRACK_HAS_A16_ELEMENTS
            if (!island_shape(left,right,top,bottom)) continue;
            if ((xmin<=2 || xmax>=185) && bottom>=98 && bottom-top>42 && support<8)
                continue;  /* A long near shoulder can also fit a circle on a hairpin. */
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
#if TRACK_HAS_A16_ELEMENTS || TRACK_ALGORITHM == 17
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
    if (!found) {
        found=observe_open_island();
#if TRACK_HAS_A16_ELEMENTS
        if (found && (track_mode16_phase==A16_ROAD || track_mode16_phase==A16_WAIT)) {
            int top=20;
            while (top<track_mode16_island_bottom && island_left[top]<0) ++top;
            if (!island_shape(island_left,island_right,top,track_mode16_island_bottom)) found=0;
        }
#endif
    }
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
#if TRACK_HAS_A16_ELEMENTS
        /* Recheck direction immediately before committing, once turn-in has
         * brought the camera back in line with the approach. */
        if (track_mode16_phase == A16_PREP && island_votes >= ENTRY_CONFIRM_FRAMES - 1U &&
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

#if TRACK_HAS_A16_ELEMENTS || TRACK_ALGORITHM == 17
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
#if TRACK_HAS_A16_ELEMENTS
        int clearance = 0;
#endif
        for (int i = 0; i < 31; ++i) {
            int x = sample_x(t,i), y = turn_samples[k][i][1];
            if (y > 118) continue;
            int margin = 5 + y/18;
            if (x < margin+2 || x > 185-margin || !a16_binary_image[y][x]) break;
            if (track_mode16_phase!=A16_MERGE && track_mode16_island_bottom >= y && island_left[y]>=0 &&
                (side<0 ? x<=island_right[y] : x>=island_left[y])) break;
            if (!a16_binary_image[y][x-margin] || !a16_binary_image[y][x+margin]) break;
            ++count;
#if TRACK_HAS_A16_ELEMENTS
            /* Reward space inside both lane edges along the near part of the arc.
             * Beyond the camera margin the available clearance is unknown. */
            if (track_mode16_phase == A16_ARC && count <= 16) {
                int step = 6 + y / 12;
                for (int band = 1; band <= 4; ++band) {
                    int dx = band * step;
                    if (x - dx < 2 || x + dx > 185 ||
                        !a16_binary_image[y][x-dx] || !a16_binary_image[y][x+dx]) break;
                    ++clearance;
                }
            }
#endif
            if (x >= margin*2+2 && x <= 185-margin*2 &&
                a16_binary_image[y][x-margin*2] && a16_binary_image[y][x+margin*2]) ++room;
        }
        if (!t) straight = count;
        int score;
#if TRACK_HAS_A16_ELEMENTS
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
        if (x<2 || x>185 || !a16_binary_image[y][x]) break;
        for (int yy=last_y-1; yy>=y; --yy) {
            int xx=x+(last_x-x)*(yy-y)/(last_y-y);
            if (!a16_binary_image[yy][xx]) goto finished;
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
    if (x < 2 || x > 185 || !a16_binary_image[y][x]) return 0;
    a = b = x;
    while (a > 2 && a16_binary_image[y][a-1]) --a;
    while (b < 185 && a16_binary_image[y][b+1]) ++b;
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

#if TRACK_HAS_A16_ELEMENTS || TRACK_ALGORITHM == 17
/* A cross has a continuous forward lane and a transverse white band that
 * opens beyond BOTH edges of that lane. One missing edge also occurs in bends. */
static int cross_before_entry(int cx)
{
    int votes = 0;
    for (int y = 40; y <= 108; y += 2) {
        int ref = y - 24, left = cx, right = cx;
        if (!a16_binary_image[ref][cx] || !a16_binary_image[y][cx]) { votes = 0; continue; }
        while (left > 2 && a16_binary_image[ref][left-1]) --left;
        while (right < 185 && a16_binary_image[ref][right+1]) ++right;
        if (right-left < 12 || left < 21 || right > 166) { votes = 0; continue; }
        int probe_left = left - 18, probe_right = right + 18;
#if TRACK_HAS_A16_ELEMENTS
        /* A cross opens on both sides and closes again toward the car.
         * A roundabout entrance can widen continuously on just one side. */
        int near_y = y + 24;
        if (near_y > 116 || !a16_binary_image[near_y][cx]) { votes = 0; continue; }
        int near_left = cx, near_right = cx;
        while (near_left > 2 && a16_binary_image[near_y][near_left-1]) --near_left;
        while (near_right < 185 && a16_binary_image[near_y][near_right+1]) ++near_right;
        if (near_left - 7 < probe_left) probe_left = near_left - 7;
        if (near_right + 7 > probe_right) probe_right = near_right + 7;
        if (probe_left < 2 || probe_right > 185) { votes = 0; continue; }
#endif
        int x = probe_left;
        while (x <= probe_right && a16_binary_image[y][x]) ++x;
        if (x > probe_right) {
            if (++votes >= 3) return 1;
        } else votes = 0;
    }
    return 0;
}
#endif

void a16_plan_island(track_mode1_result_t *out)
{
    if (track_mode16_phase==A16_MERGE) {
        cooldown_ms=cooldown_ms>a16_frame_ms?cooldown_ms-a16_frame_ms:0;
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
#if TRACK_HAS_A16_ELEMENTS
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
#if TRACK_HAS_A16_ELEMENTS || TRACK_ALGORITHM == 17
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
        if (island_votes < ENTRY_CONFIRM_FRAMES) ++island_votes;
        if (island_votes < ENTRY_CONFIRM_FRAMES || track_mode16_island_bottom < 30) return;
        track_mode16_phase=A16_ENTRY;
#if TRACK_HAS_A16_ELEMENTS
        committed_now = 1;
#endif
    }
#if TRACK_HAS_A16_ELEMENTS
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
                #if TRACK_HAS_A16_ELEMENTS
                const int entry_limit=10;
#else
                const int entry_limit=4;
#endif
                if(track_mode16_curve_command>entry_limit)track_mode16_curve_command=entry_limit;
                if(track_mode16_curve_command< -entry_limit)track_mode16_curve_command=-entry_limit;
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

void a16_ring_reset(void)
{
    track_mode16_phase = A16_ROAD;
    track_mode16_island_side = 0;
    track_mode16_island_bottom = -1;
#if TRACK_HAS_A16_ELEMENTS
    approach_curve_frames = 0U;
#endif
    island_votes = merge_votes = island_passed = 0U;
    island_open_candidate = 0U;
    rearm_clear_frames = 0U; curve_votes=split_votes=0; saw_bend=0;
    missing_ms = cooldown_ms = 0U;
}
