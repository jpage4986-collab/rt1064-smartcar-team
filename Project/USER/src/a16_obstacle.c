#include "a16_internal.h"
#include <string.h>
#include <math.h>

#if TRACK_HAS_A16_ELEMENTS
static int obstacle_vote;
static unsigned obstacle_hold_ms;

void a16_obstacle_reset(void)
{
    obstacle_vote=track_mode16_obstacle_side=0;
    obstacle_hold_ms=0;
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
        int expected=a16_projected_lane_width(far);
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
        const int near = y+12, width = a16_projected_lane_width(y);
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
            nw*100 >= a16_projected_lane_width(near)*86 && nw*100 <= a16_projected_lane_width(near)*120) {
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
int a16_plan_obstacle(track_mode1_result_t *road, const uint8_t image[][TRACK_MODE1_WIDTH])
{
    if (track_mode16_phase != A16_ROAD) {
        obstacle_vote = 0;
        return 0;
    }
    if (a16_departure_mark_ms) return 0;
    int side = track_mode16_corner_active ? 0 : obstacle_observation(image);
    if (side && side == obstacle_vote && (!track_mode16_obstacle_side || side == track_mode16_obstacle_side)) {
        track_mode16_obstacle_side = side;
        obstacle_hold_ms = 1800;
    } else if (track_mode16_obstacle_side && track_motor_duty) {
        obstacle_hold_ms = obstacle_hold_ms > a16_frame_ms ? obstacle_hold_ms-a16_frame_ms : 0;
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
            if (width<a16_projected_lane_width(y)/2) continue;
            if ((track_mode16_obstacle_side<0 && r>=185) ||
                (track_mode16_obstacle_side>0 && l<=2)) continue;
            if (y==(int)A16_ROAD_TARGET_ROW) target_valid=1;
            if (y==(int)A16_ROAD_ANCHOR_ROW) anchor_valid=1;
            int x=track_mode16_obstacle_side<0 ? r-a16_projected_lane_width(y)/3 :
                                                l+a16_projected_lane_width(y)/3;
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

