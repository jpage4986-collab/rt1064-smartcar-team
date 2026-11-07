#include "a16_internal.h"
#include <string.h>
#include <math.h>

#if TRACK_HAS_A16_ELEMENTS
/* Keep race state across vision resets (blank frames and temporary line loss). */
unsigned track16_lap;
static unsigned clear_drive_ms;
unsigned a16_departure_mark_ms;
static int last_pattern, last_center;
extern volatile uint32_t track_arm_request;

void track16_new_lap(void)
{
    track16_lap = A16_LAP_DEPART;
    clear_drive_ms = 0;
    a16_departure_mark_ms = 0;
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

void a16_update_lap(const uint8_t image[][TRACK_MODE1_WIDTH], unsigned low,
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
            if (!a16_departure_mark_ms) track_mode16_reset();
            a16_departure_mark_ms=1000;
        } else {
            a16_departure_mark_ms=a16_departure_mark_ms>a16_frame_ms ? a16_departure_mark_ms-a16_frame_ms : 0;
        }
        if (pattern || !track_motor_duty || a16_frame_ms>TRACK_CAMERA_LOST_TIMEOUT_MS || high<low+3)
            clear_drive_ms=0;
        else if ((clear_drive_ms+=a16_frame_ms)>=2500U)
            track16_lap=A16_LAP_ARMED;
    } else if (pattern && pattern==last_pattern && center>=last_center-18 &&
               center<=last_center+18 && a16_frame_ms<=TRACK_CAMERA_LOST_TIMEOUT_MS) {
        track16_lap=A16_LAP_FINISHED;
    }
    last_pattern=pattern;
    last_center=center;
}

/* Remove the stripe gaps for departure only. Require actual white rails on
 * both sides, enclosing the camera axis, at a plausible projected lane width. */
void a16_join_departure_rails(void)
{
    for (int y=48;y<=118;++y) {
        int left=2,right=185;
        while (left<94 && !(a16_binary_image[y][left] && a16_binary_image[y][left+1])) ++left;
        while (right>94 && !(a16_binary_image[y][right] && a16_binary_image[y][right-1])) --right;
        int width=right-left+1,expected=a16_projected_lane_width(y);
        if (left>=90 || right<=98 || width<expected*7/10 || width>expected*3/2) continue;
        memset(&a16_binary_image[y][left],255,(size_t)width);
    }
}
#endif

