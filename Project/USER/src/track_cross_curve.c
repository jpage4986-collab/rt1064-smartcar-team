/* CrossCurve image planner shared by Algorithms 7, 8, 9, and 10.
 * Bilateral crossing -> straight -> ordinary associated curve. Independent
 * rings retain the original entry/interior/exit flow below.
 * Original fixed-memory implementation. Reference ideas:
 * ContinuityTrack row scoring; Xiayu eight-neighbour loss/corner analysis;
 * 20th_smart_vision entry/interior/exit separation. No reference motion code
 * or simulator pose is used. All topology tests use observed pixels only. */
#include "track_cross_curve.h"
#include "tune_config.h"
#include "track_cross_curve_checks.h"
#include <string.h>

track_cross_curve_row_t track_cross_curve_rows[120];
int16_t track_cross_curve_path_x[120];
uint8_t track_cross_curve_path_confidence[120];
static uint8_t mask[188];
static uint16_t histogram[256], width_profile[120];
static int16_t last_path[120];
static uint8_t open_left[120], open_right[120], split[120], broad[120];
static uint8_t state, reason, saved_state, confirmation_count, last_condition;
static int8_t side;
static uint8_t cross_curve_seen, cross_gate;
static uint8_t have_last, filtered_valid, interior_seen, exit_far_seen, exit_clear_seen, rearm;
static uint8_t start_line_seen, start_line_armed, start_line_clear_frames;
static uint8_t start_line_approach, start_line_approach_miss, start_line_stop_frames;
static uint32_t confirmation_ms, element_ms, recovery_ms, since_trusted;
static int16_t filtered_angle, last_angle;

static int32_t ab(int32_t x) { return x < 0 ? -x : x; }
static int32_t clip(int32_t x, int32_t lo, int32_t hi)
{ return x < lo ? lo : (x > hi ? hi : x); }
static uint32_t add_ms(uint32_t a, uint32_t b)
{ return a > 60000U || b > 60000U || a+b > 60000U ? 60000U : a+b; }

static void set_state(uint8_t next, uint8_t why)
{
    state = next; reason = why; cross_gate = 0;
    confirmation_count = 0; confirmation_ms = 0; last_condition = 0;
    filtered_valid = 0;
}
/* First observation contributes no elapsed time: N identical zero-time calls
 * cannot satisfy a millisecond gate. dt belongs to the preceding interval. */
static uint8_t confirmed(uint8_t condition, uint32_t dt)
{
    if (!condition) {
        confirmation_count = 0; confirmation_ms = 0; last_condition = 0;
        return 0;
    }
    if (last_condition) confirmation_ms = add_ms(confirmation_ms, dt);
    if (confirmation_count < 255) ++confirmation_count;
    last_condition = 1;
    return confirmation_count >= TRACK_FSM_CONFIRM_FRAMES &&
           confirmation_ms >= TRACK_FSM_CONFIRM_MS;
}
void track_cross_curve_reset(void)
{
    uint16_t y;
    memset(track_cross_curve_rows, 0, sizeof(track_cross_curve_rows));
    memset(track_cross_curve_path_confidence, 0, sizeof(track_cross_curve_path_confidence));
    memset(last_path, 0, sizeof(last_path));
    for (y=0; y<120; ++y)
        width_profile[y] = (uint16_t)clip(TRACK_CONT_WIDTH_PRIOR*(y+10)/126, 8, 188);
    side = 0; cross_curve_seen = 0; have_last = 0; filtered_valid = 0; rearm = 0;
    start_line_seen = start_line_armed = start_line_clear_frames = 0;
    start_line_approach = start_line_approach_miss = start_line_stop_frames = 0;
    interior_seen = 0; exit_far_seen = 0; exit_clear_seen = 0;
    element_ms = recovery_ms = since_trusted = 0;
    last_angle = filtered_angle = TRACK_SERVO_CENTER_DEG*100;
    saved_state = FSM_NORMAL;
    set_state(FSM_WAIT, FSM_REASON_INIT);
}
void track_cross_curve_suspend(uint8_t why)
{
    /* A finish-line stop is latched until a power-on/reset. STOP and camera
     * suspension must never release the motors after completing a lap. */
    if (state == FSM_START_STOP) return;
    set_state(FSM_WAIT, why);
    have_last = 0; side = 0; rearm = 0;
    element_ms = recovery_ms = 0;
    interior_seen = exit_far_seen = exit_clear_seen = 0;
}
const char *track_cross_curve_state_name(uint8_t s)
{
    static const char *names[] = {"WAIT", "TRACK", "X CHECK", "X PASS",
        "X MERGE", "R CHECK", "R ENTER", "R CIRCLE", "R EXIT", "R MERGE", "FIND", "X CURVE", "START STOP"};
    return s <= FSM_START_STOP ? names[s] : "ERROR";
}

/* Detect several transverse dark bands inside the observed lane, ignoring
 * the dark lane edges. stage=1 means the line is ahead; stage=2 means its
 * lowest visible band has reached the near camera rows and the car must stop. */
static uint8_t start_line_stage(const uint8_t image[120][188], uint8_t global)
{
    uint8_t dark[120] = {0};
    uint8_t runs = 0, active = 0, run_len = 0, gap = 0;
    int32_t y, center, half, first = -1, last = -1;
    for (y=30; y<=112; ++y) {
        int32_t x, lo, hi, count = 0, total = 0;
        track_cross_curve_row_t *row = &track_cross_curve_rows[y];
        center = row->confidence >= 25 ? row->center : TRACK_STEER_REFERENCE_X;
        half = (int32_t)width_profile[y] * 34 / 100;
        if (half < 12) half = 12;
        lo = clip(center-half, 1, 186); hi = clip(center+half, 1, 186);
        for (x=lo; x<=hi; ++x) { ++total; if (image[y][x] <= global) ++count; }
        /* A zebra band spans most of the road width; edge tape occupies only
         * a small fraction of this centered corridor. */
        dark[y] = total && count * 100 >= total * 46;
    }
    for (y=30; y<=112; ++y) {
        if (dark[y]) {
            if (!active) { active=1; run_len=0; }
            if (run_len < 255) ++run_len;
            gap=0;
        } else if (active) {
            if (gap < 255) ++gap;
            if (gap >= 2) {
                if (run_len >= 1 && run_len <= 6) {
                    if (runs < 255) ++runs;
                    if (first < 0) first = y-gap-run_len;
                    last = y-gap-1;
                }
                active=0; run_len=0; gap=0;
            }
        }
    }
    if (active && run_len >= 1 && run_len <= 6) {
        if (runs < 255) ++runs;
        if (first < 0) first = 112-run_len+1;
        last=112;
    }
    if (runs < 4 || first < 0 || last-first < 9) return 0;
    return last >= 98 ? 2 : 1;
}

static uint8_t threshold(const uint8_t image[120][188], uint8_t *contrast)
{
    uint32_t total=0, weighted=0, bg=0, wb=0;
    double best=0;
    uint16_t x,y,t;
    uint8_t lo=255,hi=0,result=127;
    memset(histogram,0,sizeof(histogram));
    for(y=TRACK_FSM_DETECT_TOP_ROW;y<=116;++y) for(x=0;x<188;++x) {
        uint8_t v=image[y][x]; ++histogram[v];
        if(v<lo)lo=v;
        if(v>hi)hi=v;
    }
    *contrast=hi-lo;
    for(t=0;t<256;++t){total+=histogram[t];weighted+=t*histogram[t];}
    for(t=0;t<256;++t){
        int64_t n; double score;
        bg+=histogram[t];wb+=t*histogram[t];
        if(!bg||bg==total)continue;
        n=(int64_t)wb*total-(int64_t)weighted*bg;
        /* Expanded detection ROI can overflow signed 64-bit n*n. Convert
         * before multiplication; M7's FPv5-D16 supports this operation. */
        score=(double)n*(double)n/((double)bg*(total-bg));
        if(score>best){best=score;result=(uint8_t)t;}
    }
    return result;
}
static void binary_row(const uint8_t *row,uint8_t global)
{
    uint16_t x,n=0; uint8_t lo=0,hi=255;
    int32_t local,cut;
    memset(histogram,0,sizeof(histogram));
    for(x=0;x<188;++x)++histogram[row[x]];
    for(x=0;x<256;++x){n+=histogram[x];if(n>=19){lo=(uint8_t)x;break;}}
    n=0;
    for(x=0;x<256;++x){n+=histogram[x];if(n>=169){hi=(uint8_t)x;break;}}
    local=lo+TRACK_CONT_LOCAL_PERCENT*(hi-lo)/100;
    cut=(TRACK_CONT_GLOBAL_WEIGHT_PERCENT*global+(100-TRACK_CONT_GLOBAL_WEIGHT_PERCENT)*local)/100;
    for(x=0;x<188;++x)mask[x]=row[x]>cut;
    for(x=1;x<187;){
        uint16_t start,end;
        if(mask[x]||!mask[x-1]){++x;continue;}
        start=x;while(x<188&&!mask[x])++x;end=x;
        if(end<188&&(uint16_t)(end-start)<=TRACK_CONT_GAP_CLOSE_PIXELS)
            for(x=start;x<end;++x)mask[x]=1;
        x=end;
    }
}

void track_cross_curve_binary_preview(const uint8_t input[120][188],
                                      uint8_t output[120][188], uint8_t global)
{
    uint16_t x, y;
    for (y = 0U; y < 120U; ++y) {
        binary_row(input[y], global);
        for (x = 0U; x < 188U; ++x) {
            output[y][x] = mask[x] ? 255U : 0U;
        }
    }
}

typedef struct {
    uint8_t left_loss,right_loss,left_far,right_far,left_near,right_near;
    uint8_t splits,left_corner,right_corner,cross,ring,ordinary;
    uint8_t observed,lower_support,far_support;
    int16_t far_center,curve;
} features_t;

static uint8_t longest_run(const uint8_t *flags, uint16_t first, uint16_t last)
{
    uint8_t run=0,best=0;
    uint16_t y;
    for(y=first;y<=last;++y){
        if(flags[y]){++run;if(run>best)best=run;}
        else run=0;
    }
    return best;
}

static features_t observe(const uint8_t image[120][188],uint8_t global,uint8_t contrast)
{
    features_t f={0};
    int32_t y,seed=94,slope=0,width=TRACK_CONT_WIDTH_PRIOR;
    uint16_t bilateral=0,wide_run=0,max_wide=0,split_run=0,max_split=0;
    int16_t far_sum=0; uint8_t far_n=0;
    memset(track_cross_curve_rows,0,sizeof(track_cross_curve_rows));
    memset(track_cross_curve_path_confidence,0,sizeof(track_cross_curve_path_confidence));
    memset(open_left,0,sizeof(open_left));memset(open_right,0,sizeof(open_right));
    memset(split,0,sizeof(split));memset(broad,0,sizeof(broad));
    if(contrast<TRACK_CONT_MIN_CONTRAST)return f;
    for(y=116;y>=(int32_t)TRACK_FSM_DETECT_TOP_ROW;--y){
        int32_t expected=width_profile[y],pred=seed-slope;
        int32_t best=1000000,bl=0,br=0,bc=0,x,run_n=0,first_r=-1;
        int32_t image_lo=255,image_hi=0;
        track_cross_curve_row_t *row=&track_cross_curve_rows[y];
        /* Match baseline adaptive width while normal; retain learned per-row
         * widths once element topology makes measured width untrustworthy. */
        if(state==FSM_NORMAL||state==FSM_WAIT)expected=clip(width*(y+10)/126,8,188);
        binary_row(image[y],global);
        for(x=0;x<188;++x){if(image[y][x]<image_lo)image_lo=image[y][x];if(image[y][x]>image_hi)image_hi=image[y][x];}
        if(image_hi-image_lo<(int32_t)TRACK_CONT_MIN_CONTRAST)continue;
        for(x=0;x<188;){
            int32_t l,r,rw,c,score;
            if(!mask[x]){++x;continue;}
            l=x;while(x<188&&mask[x])++x;r=x-1;rw=r-l+1;
            if(rw<clip(expected/5,5,188))continue;
            ++run_n;
            if(first_r>=0 && l-first_r-1 >= (int32_t)TRACK_FSM_ISLAND_GAP_PIXELS &&
               first_r>25 && l<163)split[y]=1;
            first_r=r;
            c=(l+r)/2;
            if(l==0&&r!=187)c=clip(r-expected/2,0,187);
            if(r==187&&l!=0)c=clip(l+expected/2,0,187);
            score=100*ab(c-pred)/expected+30*ab(rw-expected)/expected;
            if(l==0&&r==187)score+=100;
            if(l>4&&image[y][l+2]<=image[y][l-2]+8)score+=25;
            if(r<183&&image[y][r-2]<=image[y][r+2]+8)score+=25;
            /* Once side is locked, the inner island must stay on that side.
             * Right island: take the left branch; left island: right branch. */
            if(side && run_n>1 && state>=FSM_RING_ENTER && state<=FSM_RING_MERGE){
                if(side<0)score-=80;
                else score+=80;
            }
            if(score<best){best=score;bl=l;br=r;bc=c;}
        }
        if(best>110)continue;
        row->left=(int16_t)bl;row->right=(int16_t)br;row->center=(int16_t)bc;
        row->left_real=bl>2;row->right_real=br<185;
        row->confidence=(uint8_t)((row->left_real&&row->right_real?95:45)*clip(100-60*best/100,20,100)/100);
        if(!row->left_real&&!row->right_real){row->confidence=0;continue;}
        row->provenance=row->left_real&&row->right_real?1:2;
        open_left[y]=!row->left_real;open_right[y]=!row->right_real;
        broad[y]=(br-bl+1)*100>=expected*(int32_t)TRACK_CROSS_CURVE_WIDE_PERCENT;
        slope=(7*slope+3*(seed-bc))/10;seed=bc;
        /* Do not teach the normal scorer a junction's expanded width. */
        if(row->provenance==1 && !broad[y] && !split[y] && (state==FSM_NORMAL||state==FSM_WAIT)){
            int32_t measured=(br-bl+1)*126/(y+10);
            width=(85*width+15*measured)/100;
        }
        if(y>=(int32_t)TRACK_CONT_SCAN_TOP_ROW){
            if(row->confidence>=25)++f.observed;
            if(row->left_real&&row->right_real)++bilateral;
            f.left_loss+=open_left[y];f.right_loss+=open_right[y];
            if(y>=(int32_t)TRACK_CONT_TARGET_ROW && row->confidence>=25)++f.lower_support;
            f.left_near+=(y>=(int32_t)TRACK_CONT_TARGET_ROW && open_left[y]);
            f.right_near+=(y>=(int32_t)TRACK_CONT_TARGET_ROW && open_right[y]);
        }else if(row->confidence>=25){far_sum+=bc;++far_n;}
        if(y<(int32_t)TRACK_CONT_TARGET_ROW){f.left_far+=open_left[y];f.right_far+=open_right[y];}
    }
    /* Opening geometry comes from all threshold runs, including full-width
     * rows rejected as paths. They never count as observed steering support. */
    for(y=116;y>=(int32_t)TRACK_FSM_DETECT_TOP_ROW;--y){
        uint16_t l=0,r=187;
        binary_row(image[y],global);
        while(l<188&&!mask[l])++l;
        while(r>0&&!mask[r])--r;
        if(l<r && l<=2 && r>=185){
            broad[y]=1;open_left[y]=open_right[y]=1;
        }
        {
            int32_t center=have_last?last_path[y]:(int32_t)TRACK_STEER_REFERENCE_X;
            int32_t left,right,margin;
            if(center<8||center>179)center=TRACK_STEER_REFERENCE_X;
            left=right=center;
            if(mask[center]){
                while(left>0&&mask[left-1])--left;
                while(right<187&&mask[right+1])++right;
            }
            margin=width_profile[y]*TRACK_CROSS_CURVE_WIDE_PERCENT/200;
            /* Both sides must expand around the incoming route. A single-side
             * ring mouth, or two separate arms around a dark island, is not X. */
            if(center-left>=margin && right-center>=margin){
                ++wide_run;if(wide_run>max_wide)max_wide=wide_run;
            }else wide_run=0;
        }
        if(split[y]){++split_run;if(split_run>max_split)max_split=split_run;}else split_run=0;
        if(y<114 && track_cross_curve_rows[y].confidence && track_cross_curve_rows[y+2].confidence){
            if(ab(track_cross_curve_rows[y].left-track_cross_curve_rows[y+2].left)>(int32_t)TRACK_FSM_CORNER_DELTA)++f.left_corner;
            if(ab(track_cross_curve_rows[y].right-track_cross_curve_rows[y+2].right)>(int32_t)TRACK_FSM_CORNER_DELTA)++f.right_corner;
        }
        /* A gap starts/ends at an observed corner, even when its interior row
         * has no edge. This distinguishes junctions from smooth clipped bends. */
        if(y<116){
            if(open_left[y]!=open_left[y+1] && (track_cross_curve_rows[y].right_real||track_cross_curve_rows[y+1].right_real))++f.left_corner;
            if(open_right[y]!=open_right[y+1] && (track_cross_curve_rows[y].left_real||track_cross_curve_rows[y+1].left_real))++f.right_corner;
        }
    }
    f.left_loss=longest_run(open_left,TRACK_CONT_SCAN_TOP_ROW,116);
    f.right_loss=longest_run(open_right,TRACK_CONT_SCAN_TOP_ROW,116);
    f.left_near=longest_run(open_left,TRACK_CONT_TARGET_ROW,116);
    f.right_near=longest_run(open_right,TRACK_CONT_TARGET_ROW,116);
    f.left_far=longest_run(open_left,TRACK_FSM_DETECT_TOP_ROW,TRACK_CONT_TARGET_ROW-1);
    f.right_far=longest_run(open_right,TRACK_FSM_DETECT_TOP_ROW,TRACK_CONT_TARGET_ROW-1);
    f.splits=(uint8_t)max_split;f.far_support=far_n;
    f.far_center=far_n?far_sum/far_n:(int16_t)TRACK_STEER_REFERENCE_X;
    f.curve=track_cross_curve_rows[TRACK_CONT_TARGET_ROW].center-track_cross_curve_rows[TRACK_CONT_ANCHOR_ROW].center;
    /* Width alone also fires on broad continuous bends. Require another
     * topology cue before treating the widened corridor as a junction. */
    f.cross=max_wide>=TRACK_CROSS_CURVE_WIDE_ROWS &&
        (f.splits>=TRACK_FSM_SPLIT_ROWS ||
         (f.left_corner>=TRACK_FSM_FEATURE_ROWS &&
          f.right_corner>=TRACK_FSM_FEATURE_ROWS));
    f.ring=0;
    if(max_split>=TRACK_FSM_SPLIT_ROWS){
        if(f.right_loss>=TRACK_FSM_FEATURE_ROWS && f.left_loss<TRACK_FSM_FEATURE_ROWS &&
           f.right_corner>=TRACK_FSM_FEATURE_ROWS)f.ring=2;
        if(f.left_loss>=TRACK_FSM_FEATURE_ROWS && f.right_loss<TRACK_FSM_FEATURE_ROWS &&
           f.left_corner>=TRACK_FSM_FEATURE_ROWS)f.ring=1;
    }
    f.ordinary=bilateral>=TRACK_DRIVE_MIN_VALID_ROWS && !f.cross && !f.ring &&
               f.left_loss<TRACK_FSM_FEATURE_ROWS && f.right_loss<TRACK_FSM_FEATURE_ROWS;
    return f;
}

static void plan_path(const features_t *f)
{
    int y;
    for(y=TRACK_FSM_DETECT_TOP_ROW;y<=116;++y){
        track_cross_curve_row_t *r=&track_cross_curve_rows[y];
        int32_t target=r->center;
        if(state==FSM_CROSS_PASS){
            target=TRACK_STEER_REFERENCE_X;
            r->provenance=3;
        }
#if defined(TRACK_CROSS_CURVE_VARIANT10)
        else if(state>=FSM_RING_ENTER&&state<=FSM_RING_EXIT){
#else
        else if(state>=FSM_RING_ENTER&&state<=FSM_RING_MERGE){
#endif
            int inner=state==FSM_RING_INSIDE;
            if(side>0){
                if(inner&&r->right_real)target=r->right-width_profile[y]/2;
                else if(r->left_real)target=r->left+width_profile[y]/2;
            }else{
                if(inner&&r->left_real)target=r->left+width_profile[y]/2;
                else if(r->right_real)target=r->right-width_profile[y]/2;
            }
            if(r->confidence)r->provenance=3;
        }
        track_cross_curve_path_x[y]=(int16_t)clip(target,0,187);
        track_cross_curve_path_confidence[y]=r->confidence;
        if(r->provenance==3 && !r->confidence && f->lower_support>=TRACK_FSM_SUPPORT_ROWS &&
           f->far_support>=TRACK_FSM_FEATURE_ROWS)track_cross_curve_path_confidence[y]=30;
    }
}

track_cross_curve_result_t track_cross_curve_process(const uint8_t image[120][188],uint32_t dt,uint8_t run)
{
    track_cross_curve_result_t out={0}; features_t f;
    uint8_t contrast=0,trusted,special,near_open,far_open,consistent=1,line_stage=0;
    uint8_t curve_hint, gate;
    int32_t angle;
    if(!run)track_cross_curve_suspend(FSM_REASON_STOP);
    else if(dt>=TRACK_CAMERA_LOST_TIMEOUT_MS)track_cross_curve_suspend(FSM_REASON_CAMERA);
    since_trusted=add_ms(since_trusted,dt);
    out.global_threshold=threshold(image,&contrast);
    f=observe(image,out.global_threshold,contrast);
    if(run && dt<TRACK_CAMERA_LOST_TIMEOUT_MS && state!=FSM_START_STOP){
        line_stage=start_line_stage(image,out.global_threshold);
        if(!start_line_seen){
            /* The first start/finish line seen after reset is the departure
             * marker. Ignore it; arm only after the car has cleared it. */
            if(line_stage) { start_line_seen=1; start_line_clear_frames=0; }
        }else if(!start_line_armed){
            if(line_stage) start_line_clear_frames=0;
            else if(start_line_clear_frames<255) ++start_line_clear_frames;
            if(start_line_clear_frames>=TRACK_START_LINE_CLEAR_FRAMES)
                start_line_armed=1;
        }else{
            if(line_stage){
                start_line_approach=1;start_line_approach_miss=0;
                if(line_stage==2){
                    if(start_line_stop_frames<255)++start_line_stop_frames;
                }else start_line_stop_frames=0;
            }else if(start_line_approach_miss<255){
                ++start_line_approach_miss;
                if(start_line_approach_miss>=2){
                    start_line_approach=0;start_line_approach_miss=0;
                    start_line_stop_frames=0;
                }
            }
            if(start_line_stop_frames>=TRACK_START_LINE_STOP_FRAMES){
                set_state(FSM_START_STOP,FSM_REASON_START_LINE);
                start_line_approach=0;
            }
        }
    }
    near_open=side<0?f.left_near:f.right_near;
    far_open=side<0?f.left_far:f.right_far;
    special=state>=FSM_CROSS_PASS&&state<=FSM_RING_MERGE;
    trusted=f.observed>=TRACK_DRIVE_MIN_VALID_ROWS &&
        track_cross_curve_rows[TRACK_CONT_ANCHOR_ROW].confidence>=TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE &&
        track_cross_curve_rows[TRACK_CONT_TARGET_ROW].confidence>=TRACK_DRIVE_MIN_TARGET_CONFIDENCE;
    if(special || (f.cross && state!=FSM_CROSS_CURVE))trusted=f.lower_support>=TRACK_FSM_SUPPORT_ROWS &&
        f.observed>=TRACK_FSM_SUPPORT_ROWS &&
        (f.far_support>=TRACK_FSM_FEATURE_ROWS||f.splits>=TRACK_FSM_SPLIT_ROWS||
         track_cross_curve_rows[TRACK_CONT_TARGET_ROW].confidence>=25);

    if(state!=FSM_START_STOP && run && dt<TRACK_CAMERA_LOST_TIMEOUT_MS){
        if((state>=FSM_CROSS_CANDIDATE&&state<=FSM_RING_MERGE)||state==FSM_CROSS_CURVE)element_ms=add_ms(element_ms,dt);
        if(state==FSM_RECOVER)element_ms=add_ms(element_ms,dt);
        if(element_ms>=TRACK_FSM_ELEMENT_TIMEOUT_MS){
            track_cross_curve_suspend(FSM_REASON_ELEMENT_TIMEOUT);trusted=0;
        }else if(state==FSM_WAIT){
            /* Only a newly observed, not synthetic, corridor can restart. */
            if(confirmed(trusted,dt)){side=0;rearm=0;set_state(FSM_NORMAL,FSM_REASON_CONFIRMED);}
        }else if(state==FSM_RECOVER){
            recovery_ms=add_ms(recovery_ms,dt);
            if(saved_state>=FSM_RING_ENTER&&saved_state<=FSM_RING_MERGE)
                consistent=f.splits>=TRACK_FSM_SPLIT_ROWS ||
                    (side<0?f.right_loss<TRACK_FSM_FEATURE_ROWS:f.left_loss<TRACK_FSM_FEATURE_ROWS);
            if(recovery_ms>=TRACK_FSM_RECOVERY_MS){
                track_cross_curve_suspend(FSM_REASON_RECOVERY_TIMEOUT);
            }else if(confirmed(trusted&&consistent,dt)){
                set_state(saved_state,FSM_REASON_RECOVERED);recovery_ms=0;
            }
        }else if(!trusted){
            if(have_last && since_trusted<=TRACK_FSM_RECENT_PATH_MS){
                saved_state=state;recovery_ms=0;set_state(FSM_RECOVER,FSM_REASON_LOST);
            }else track_cross_curve_suspend(FSM_REASON_LOST);
        }else{
            switch(state){
            case FSM_NORMAL:
                element_ms=0;
                if(rearm){if(confirmed(f.ordinary,dt))rearm=0;break;}
                if(f.cross){
                    side=0;cross_curve_seen=0;
                    set_state(FSM_CROSS_CANDIDATE,FSM_REASON_CROSS);
                }
                else if(f.ring){side=f.ring==1?-1:1;set_state(FSM_RING_CANDIDATE,FSM_REASON_RING);}
                break;
            case FSM_CROSS_CANDIDATE:
                if(!f.cross){set_state(FSM_NORMAL,FSM_REASON_REJECTED);element_ms=0;}
                else if(confirmed(f.cross,dt))set_state(FSM_CROSS_PASS,FSM_REASON_CONFIRMED);
                break;
            case FSM_CROSS_PASS:
                /* Release the straight override to ordinary ContinuityTrack
                 * once a curved/split route is actually supported near target.
                 * No ring direction, entry, lap or exit decision in this scope. */
                curve_hint=(f.splits>=TRACK_FSM_SPLIT_ROWS ||
                    (!f.cross && ab(f.curve)>=(int32_t)TRACK_FSM_CURVE_PIXELS)) &&
                    track_cross_curve_rows[TRACK_CONT_TARGET_ROW].confidence>=25 &&
                    f.lower_support>=TRACK_FSM_SUPPORT_ROWS;
                gate=curve_hint?1:(f.ordinary?2:0);
                if(gate!=cross_gate){confirmation_count=0;confirmation_ms=0;last_condition=0;cross_gate=gate;}
                if(confirmed(gate!=0,dt)){
                    if(gate==1)set_state(FSM_CROSS_CURVE,FSM_REASON_CONFIRMED);
                    else set_state(FSM_CROSS_MERGE,FSM_REASON_MERGED);
                }
                break;
            case FSM_CROSS_CURVE:
                /* Ordinary row continuity steers through the associated curve.
                 * Suppress independent-ring classification until straight road
                 * has been reacquired after a visible bend. */
                if(!cross_curve_seen){
                    if(confirmed(ab(f.curve)>=(int32_t)TRACK_FSM_CURVE_PIXELS,dt)){
                        cross_curve_seen=1;
                        confirmation_count=0;confirmation_ms=0;last_condition=0;
                    }
                }else if(confirmed(f.ordinary && ab(f.curve)<(int32_t)TRACK_FSM_CURVE_PIXELS &&
                    ab((track_cross_curve_rows[TRACK_CONT_TARGET_ROW].center-(int32_t)TRACK_STEER_REFERENCE_X)*
                       (int32_t)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL)<=(int32_t)TRACK_CROSS_CURVE_STRAIGHT_CDEG,dt)){
                    side=0;rearm=1;element_ms=0;
                    set_state(FSM_NORMAL,FSM_REASON_MERGED);
                }
                break;
            case FSM_CROSS_MERGE:
                /* A branch may still hide one lane edge after the car has
                 * rejoined. Require a stable observed corridor without the
                 * junction/ring cues, rather than demanding both edges. */
                if(confirmed(!f.cross && !f.ring &&
                    f.lower_support>=TRACK_FSM_SUPPORT_ROWS &&
                    f.observed>=TRACK_DRIVE_MIN_VALID_ROWS &&
                    f.splits<TRACK_FSM_SPLIT_ROWS,dt)){
                    set_state(FSM_NORMAL,FSM_REASON_MERGED);element_ms=0;rearm=1;
                }
                break;
            case FSM_RING_CANDIDATE:
                if(f.ring!=(side<0?1:2)){side=0;set_state(FSM_NORMAL,FSM_REASON_REJECTED);element_ms=0;}
                else if(confirmed(near_open>=TRACK_FSM_FEATURE_ROWS &&
                    ab(f.curve)>=(int32_t)TRACK_FSM_CURVE_PIXELS,dt)){
                    interior_seen=exit_far_seen=exit_clear_seen=0;set_state(FSM_RING_ENTER,FSM_REASON_ENTRY);
                }
                break;
            case FSM_RING_ENTER:
                /* Entry opening must disappear before an exit can be armed.
                 * A mere straight double edge cannot prove we entered a ring. */
                if(confirmed(near_open<TRACK_FSM_FEATURE_ROWS &&
                    (side<0?f.right_loss:f.left_loss)<TRACK_FSM_FEATURE_ROWS &&
                    ab(f.curve)>=(int32_t)TRACK_FSM_CURVE_PIXELS &&
                    (side<0?track_cross_curve_rows[TRACK_CONT_TARGET_ROW].left_real:
                            track_cross_curve_rows[TRACK_CONT_TARGET_ROW].right_real),dt)){
                    interior_seen=1;set_state(FSM_RING_INSIDE,FSM_REASON_INTERIOR);
                }
                break;
            case FSM_RING_INSIDE:
                /* Discard the old entrance opening before looking for a new
                 * outlet. Otherwise its far/near motion could cause a shortcut. */
                if(!exit_clear_seen){
                    if(confirmed(far_open<TRACK_FSM_FEATURE_ROWS && near_open<TRACK_FSM_FEATURE_ROWS,dt)){
                        exit_clear_seen=1;
                        confirmation_count=0;confirmation_ms=0;last_condition=0;
                    }
                }else if(!exit_far_seen){
                    if(confirmed(interior_seen && far_open>=TRACK_FSM_FEATURE_ROWS &&
                        near_open<TRACK_FSM_FEATURE_ROWS && (side<0?f.left_corner:f.right_corner),dt)){
                        exit_far_seen=1;reason=FSM_REASON_EXIT_FAR;
                        confirmation_count=0;confirmation_ms=0;last_condition=0;
                    }
                }else if(confirmed(near_open>=TRACK_FSM_FEATURE_ROWS &&
                    (side<0?f.left_corner:f.right_corner),dt))set_state(FSM_RING_EXIT,FSM_REASON_EXIT_NEAR);
                break;
            case FSM_RING_EXIT:
#if defined(TRACK_CROSS_CURVE_VARIANT10)
                /* Once the far/near outlet sequence has already confirmed an
                 * exit, hand over as soon as a stable near-field path exists.
                 * Requiring f.ordinary here deadlocks when the opening still
                 * looks like a branch, even though the car is on the exit. */
                if(confirmed(f.lower_support>=TRACK_FSM_SUPPORT_ROWS &&
                    f.observed>=TRACK_DRIVE_MIN_VALID_ROWS &&
                    track_cross_curve_rows[TRACK_CONT_TARGET_ROW].confidence>=
                        TRACK_DRIVE_MIN_TARGET_CONFIDENCE,dt))
                    set_state(FSM_RING_MERGE,FSM_REASON_MERGED);
#else
                if(confirmed(f.ordinary,dt))set_state(FSM_RING_MERGE,FSM_REASON_MERGED);
#endif
                break;
            case FSM_RING_MERGE:
#if defined(TRACK_CROSS_CURVE_VARIANT10)
                /* The ordered far-then-near outlet was already confirmed in
                 * R CIRCLE/R EXIT. Do not wait for all junction/branch cues
                 * to vanish here: hand control back after a stable supported
                 * forward path, then let normal/cross logic classify ahead. */
                if(confirmed(f.lower_support>=TRACK_FSM_SUPPORT_ROWS &&
                    f.observed>=TRACK_DRIVE_MIN_VALID_ROWS &&
                    track_cross_curve_rows[TRACK_CONT_TARGET_ROW].confidence>=
                        TRACK_DRIVE_MIN_TARGET_CONFIDENCE,dt)){
#else
                if(confirmed(f.ordinary,dt)){
#endif
                    side=0;interior_seen=exit_far_seen=exit_clear_seen=0;rearm=1;element_ms=0;
                    set_state(FSM_NORMAL,FSM_REASON_MERGED);
                }
                break;
            default:break;
            }
        }
    }
    plan_path(&f);
    if(state==FSM_RECOVER){
        int y;
        for(y=TRACK_FSM_DETECT_TOP_ROW;y<=116;++y){
            track_cross_curve_path_x[y]=last_path[y];
            /* Prediction is displayable, but never promoted to observation. */
            if(!track_cross_curve_rows[y].confidence)track_cross_curve_rows[y].provenance=4;
        }
    }
    out.anchor_x=track_cross_curve_path_x[TRACK_CONT_ANCHOR_ROW];
    out.target_x=track_cross_curve_path_x[TRACK_CONT_TARGET_ROW];
    out.anchor_confidence=track_cross_curve_rows[TRACK_CONT_ANCHOR_ROW].confidence;
    out.target_confidence=track_cross_curve_rows[TRACK_CONT_TARGET_ROW].confidence;
    out.valid_rows=f.observed;
    angle=TRACK_SERVO_CENTER_DEG*100-
          (out.target_x-(int32_t)TRACK_STEER_REFERENCE_X)*(int32_t)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL;
    angle=clip(angle,TRACK_SERVO_RIGHT_DEG*100,TRACK_SERVO_LEFT_DEG*100);
    if(state==FSM_CROSS_PASS)angle=TRACK_SERVO_CENTER_DEG*100;
    else if(state==FSM_RECOVER)angle=last_angle;
    else if(state==FSM_WAIT)angle=TRACK_SERVO_CENTER_DEG*100;
    else{
        if(filtered_valid)angle=(angle*(100-TRACK_FSM_PREVIOUS_PERCENT)+filtered_angle*TRACK_FSM_PREVIOUS_PERCENT)/100;
        filtered_angle=(int16_t)angle;filtered_valid=1;
    }
    out.servo_cdeg=(int16_t)angle;
    out.drive_allowed=run && dt<TRACK_CAMERA_LOST_TIMEOUT_MS && state!=FSM_WAIT && state!=FSM_START_STOP &&
        (trusted||state==FSM_RECOVER);
    out.pwm_cap_percent=TRACK_WHEEL_PWM_MAX_PERCENT;
    if(state>=FSM_CROSS_CANDIDATE&&state<=FSM_RING_MERGE)out.pwm_cap_percent=TRACK_FSM_ELEMENT_PWM_PERCENT;
    if(state==FSM_RECOVER){
        out.pwm_cap_percent=TRACK_FSM_RECOVERY_PWM_PERCENT;
        out.recovery_remaining_ms=(uint16_t)(TRACK_FSM_RECOVERY_MS-recovery_ms);
    }else if(out.drive_allowed && trusted){
        have_last=1;since_trusted=0;last_angle=(int16_t)angle;
        memcpy(last_path,track_cross_curve_path_x,sizeof(last_path));
    }
    if(start_line_approach && state!=FSM_START_STOP && out.pwm_cap_percent>TRACK_START_LINE_APPROACH_PWM_PERCENT)
        out.pwm_cap_percent=TRACK_START_LINE_APPROACH_PWM_PERCENT;
    if(!out.drive_allowed)out.pwm_cap_percent=0;
    if(state==FSM_NORMAL && trusted && f.ordinary){
        int y;
        for(y=TRACK_FSM_DETECT_TOP_ROW;y<=116;++y){
            track_cross_curve_row_t *r=&track_cross_curve_rows[y];
            if(r->provenance==1&&!broad[y]&&!split[y])
                width_profile[y]=(uint16_t)((85*width_profile[y]+15*(r->right-r->left+1))/100);
        }
    }
    out.state=state;out.reason=reason;out.ring_side=side;
    out.lost_left=f.left_loss;out.lost_right=f.right_loss;out.split_rows=f.splits;
    out.corner_left=f.left_corner;out.corner_right=f.right_corner;
    out.observed_rows=f.observed;out.near_open=near_open;out.far_open=far_open;
    out.interior_seen=interior_seen;out.exit_far_seen=exit_far_seen;
    return out;
}
