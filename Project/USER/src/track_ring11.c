/* Algorithm 11: algorithm-3 ordinary-road tracker plus symmetric ring topology.
 * Original implementation. Conceptual reference: HammerTa C-car's separate
 * left/right entry/exit edge repair. No TC264 hardware or source is copied.
 * No fixed direction, fixed steering lock, gyro assumption or elapsed-time lap.
 */
#include "track_ring11.h"
#include "track_mode4.h"
#include "tune_config.h"
#ifndef TRACK_RING11_SHAPE_PIXELS
#define TRACK_RING11_SHAPE_PIXELS 6u
#endif
#include <string.h>
#include <stdlib.h>
static uint8_t binary[120][188], seen[2820];
static uint16_t queue[22560], width_profile[120];
static int16_t trace_left[120],trace_right[120],trace_center[120];
static uint8_t trace_conf[120];
volatile uint32_t track_ring11_entry_count,track_ring11_seen_states;
volatile int32_t track_ring11_entry_row,track_ring11_candidate_side;
volatile uint32_t track_ring11_lookahead_row;
int16_t track_ring11_path_x[120];
uint8_t track_ring11_path_confidence[120];
volatile int32_t track_ring11_blob_side;
volatile uint32_t track_ring11_blob_bottom, track_ring11_blob_area;
volatile int32_t track_ring11_shape_side,track_ring11_shape_score,track_ring11_shape_row;
static uint8_t state,reason,saved_state,frames,interior,far_exit,rearm,body_seen;
static int8_t side;
static uint32_t gate_ms,element_ms,missing_ms,inside_ms,clear_ms;
static int clip(int x,int lo,int hi){return x<lo?lo:(x>hi?hi:x);}
static void transition(uint8_t s,uint8_t r){
 /* Count a newly confirmed entry, not every recovery into the same phase. */
 if(s==FSM_RING_ENTER && state==FSM_RING_CANDIDATE)++track_ring11_entry_count;
 state=s;reason=r;frames=0;gate_ms=0;track_ring11_seen_states|=1U<<s;
}
static int confirm(int condition,uint32_t dt){
 if(!condition){frames=0;gate_ms=0;return 0;}
 if(frames)gate_ms+=dt;
 if(frames<255)++frames;
 return frames>=TRACK_RING11_CONFIRM_FRAMES && gate_ms>=TRACK_RING11_CONFIRM_MS;
}
void track_ring11_suspend(uint8_t why){
 side=0;interior=far_exit=body_seen=0;element_ms=missing_ms=inside_ms=clear_ms=0;
 transition(FSM_WAIT,why);
}
void track_ring11_reset(void){
 unsigned y;track_mode4_reset();memset(track_ring11_path_confidence,0,120);
 for(y=0;y<120;++y)width_profile[y]=clip(TRACK_CONT_WIDTH_PRIOR*(y+10)/126,8,188);
 track_ring11_entry_count=track_ring11_seen_states=0;
 rearm=0;track_ring11_suspend(FSM_REASON_INIT);
}
const char *track_ring11_state_name(uint8_t s){
 static const char *n[]={"WAIT","TRACK","X CHECK","X PASS","X MERGE","R CHECK","R ENTER","R CIRCLE","R EXIT","R MERGE","LOST"};
 return s<=FSM_RECOVER?n[s]:"ERROR";
}
static void enqueue(unsigned x,unsigned y,unsigned *tail){
 unsigned pos=y*188+x;
 if(binary[y][x] || (seen[pos>>3]&(1U<<(pos&7))))return;
 seen[pos>>3]|=1U<<(pos&7);queue[(*tail)++]=(uint16_t)pos;
}
typedef struct{int side,bottom,area;} island_t;
static island_t island(void){
 island_t best={0};unsigned y,x;int largest_left=0,largest_right=0;
 memset(seen,0,sizeof(seen));
 for(y=TRACK_RING11_TOP_ROW;y<=110;++y)for(x=0;x<188;++x){
  unsigned head=0,tail=0;int x0=188,x1=0,y0=120,y1=0;uint32_t sx=0;
  enqueue(x,y,&tail);
   if(!tail)continue;
  while(head<tail){unsigned pos=queue[head++],u=pos%188,v=pos/188;
   if((int)u<x0)x0=u;
   if((int)u>x1)x1=u;
   if((int)v<y0)y0=v;
   if((int)v>y1)y1=v;
   sx+=u;
   if(u)enqueue(u-1,v,&tail);
   if(u<187)enqueue(u+1,v,&tail);
   if(v>TRACK_RING11_TOP_ROW)enqueue(u,v-1,&tail);
   if(v<110)enqueue(u,v+1,&tail);
  }
  /* Partial objects touching ROI top/bottom cannot establish an island.
   * Clipping at a side is allowed only with white support above and below. */
  if(tail>=TRACK_RING11_MIN_AREA && y0>(int)TRACK_RING11_TOP_ROW && y1<110 && y1-y0>=3 && y1-y0<60 && x1-x0>=6 && x1-x0<160 &&
     tail*100U>=(unsigned)((x1-x0+1)*(y1-y0+1)*25)){
   int support=0,total=0,u;int mid=(int)(sx/tail),s=mid<88?-1:(mid>99?1:0);
   for(u=x0;u<=x1;++u){support+=binary[y0-1][u]+binary[y1+1][u];total+=2;}
   if(!s || support*100<total*TRACK_RING11_WHITE_PERCENT)continue;
   if(s<0 && (int)tail>largest_left)largest_left=tail;
   if(s>0 && (int)tail>largest_right)largest_right=tail;
   if((int)tail>best.area){best.side=s;best.bottom=y1;best.area=tail;}
  }
 }
 /* A small opposite speck no longer vetoes a stronger candidate.
  * Comparable bilateral islands remain ambiguous. */
 if(largest_left && largest_right){
  int lo=largest_left<largest_right?largest_left:largest_right;
  int hi=largest_left>largest_right?largest_left:largest_right;
  if(lo*100>=hi*TRACK_RING11_RIVAL_PERCENT)best.side=0;
 }
 track_ring11_blob_side=best.side;track_ring11_blob_bottom=best.bottom;track_ring11_blob_area=best.area;
 return best;
}
/* Trace the route into the far field independently of algorithm 3's row-54
 * steering crop. Only a measured white run with a real edge is usable. */
static void trace_boundaries(void){
 int y,seed=94;
 memset(trace_conf,0,sizeof(trace_conf));
 for(y=118;y>=(int)TRACK_RING11_TOP_ROW;--y){
  int x,best=100000,bl=0,br=0,bc=0,expected=width_profile[y];
  for(x=0;x<188;){
   int l,r,w,c,score;
   if(!binary[y][x]){++x;continue;}
   l=x;while(x<188 && binary[y][x])++x;r=x-1;w=r-l+1;
   if(w<clip(expected/5,5,188) || (l==0&&r==187))continue;
   c=(l+r)/2;
   if(l==0)c=clip(r-expected/2,l,r);
   if(r==187)c=clip(l+expected/2,l,r);
   score=100*(c>seed?c-seed:seed-c)/expected+20*(w>expected?w-expected:expected-w)/expected;
   if(score<best){best=score;bl=l;br=r;bc=c;}
  }
  if(best>140)continue;
  trace_left[y]=bl;trace_right[y]=br;trace_center[y]=bc;
  trace_conf[y]=(bl>2&&br<185)?80:40;seed=bc;
 }
}
/* A side mouth terminates an inner boundary, while the opposite outer edge
 * remains visible. This also works when the island touches the image border. */
static int mouth_tip(int which){
 int y,best=-1;
 for(y=TRACK_RING11_TOP_ROW+6;y<=103;++y){
  int k,open=0,outer=0,edge=which<0?trace_left[y]:trace_right[y];
  if(!trace_conf[y] || !trace_conf[y-6] || (which<0?edge<=8:edge>=179))continue;
  /* Require an inward hook before the opening; a monotone clipped bend or
   * triangular foreground edge is not enough to claim a ring mouth. */
  {int upper=which<0?trace_left[y-6]:trace_right[y-6];
   if(which<0?edge-upper<4:upper-edge<4)continue;}

  for(k=1;k<=6;++k)if(trace_conf[y+k]){
   int e=which<0?trace_left[y+k]:trace_right[y+k];
   if(which<0?e<=3:e>=184)++open;
   if(which<0?trace_right[y+k]<185:trace_left[y+k]>2)++outer;
  }
  if(open>=4&&outer>=4)best=y;
 }
 return best;
}
/* Straight end segments with an inward bulge between them. Work on measured
 * boundaries, never on the painted path (which could create self-confirmation).
 * Chord residual removes perspective slope; both ends must be locally straight
 * and have similar slopes. A sustained one-way bend has no returning end. */
static int boundary_shape(int which,int *row){
 int a,b,y,k,best=0;const int16_t *edge=which<0?trace_left:trace_right;
 *row=-1;
 for(a=TRACK_RING11_TOP_ROW;a<=72;a+=4)for(b=a+32;b<=114 && b<=a+76;b+=4){
  int valid=1,peak=0,py=-1,support=0,span=b-a;
  for(k=0;k<=6;++k){int u=a+k,v=b-6+k;
   if(!trace_conf[u]||!trace_conf[v] || (which<0?(edge[u]<=3||edge[v]<=3):(edge[u]>=184||edge[v]>=184))){valid=0;break;}
   if(abs(6*(edge[u]-edge[a])-k*(edge[a+6]-edge[a]))>18 ||
      abs(6*(edge[v]-edge[b-6])-k*(edge[b]-edge[b-6]))>18){valid=0;break;}
  }
  if(!valid || abs((edge[a+6]-edge[a])-(edge[b]-edge[b-6]))>5)continue;
  for(y=a+7;y<=b-7;++y){int d;
   if(!trace_conf[y] || (which<0?edge[y]<=3:edge[y]>=184))continue;
   d=((edge[y]-edge[a])*span-(edge[b]-edge[a])*(y-a))*(which<0?1:-1)/span;
   if(d>=TRACK_RING11_SHAPE_PIXELS)++support;
   if(d>peak){peak=d;py=y;}
  }
  if(support>=6 && peak>best){best=peak;*row=py;}
 }
 return best;
}
void track_ring11_binary_preview(const uint8_t input[120][188],uint8_t output[120][188],uint8_t t){
 unsigned y,x;(void)input;(void)t;
 for(y=0;y<120;++y)for(x=0;x<188;++x)output[y][x]=binary[y][x]?255:0;
}
track_fsm_result_t track_ring11_process(const uint8_t image[120][188],uint32_t dt,uint8_t run){
 track_fsm_result_t out={0};track_mode4_result_t base=track_mode4_process(image);island_t blob;
 unsigned y,x;int inner_near=0,outer_near=0,inner_far_loss=0,inner_near_loss=0,far_corner=0,prev_edge=-1;
 int anchor=0,target=0,observed=0,bilateral=0;int active,good,base_good;
 int left_tip,right_tip,tip,look,planning,planned_good,entry_ready,phase,use_plan;
 int ls,rs,ly,ry,shape_side=0,shape_row=-1;
 if(dt>2000)dt=2000;
 /* Segment every row for topology, independently from the steering ROI. */
 track_mode4_binary_preview(image,binary);
 for(y=0;y<120;++y)for(x=0;x<188;++x)binary[y][x]=binary[y][x]!=0;
 blob=island();trace_boundaries();
 left_tip=mouth_tip(-1);right_tip=mouth_tip(1);
 ls=boundary_shape(-1,&ly);rs=boundary_shape(1,&ry);
 if(ls>=TRACK_RING11_SHAPE_PIXELS && ls>=rs+3){shape_side=-1;shape_row=ly;}
 if(rs>=TRACK_RING11_SHAPE_PIXELS && rs>=ls+3){shape_side=1;shape_row=ry;}
 track_ring11_shape_side=shape_side;track_ring11_shape_score=shape_side<0?ls:(shape_side>0?rs:0);
 track_ring11_shape_row=shape_row;
 /* A shape is independent evidence, including an island joined to the border. */
 if(shape_side && (!blob.side || blob.side==shape_side)){blob.side=shape_side;if(shape_row>blob.bottom)blob.bottom=shape_row;}
 if(!blob.side && ((left_tip>=0)!=(right_tip>=0))){blob.side=left_tip>=0?-1:1;blob.bottom=left_tip>=0?left_tip:right_tip;}
 track_ring11_candidate_side=blob.side;
 tip=side<0?left_tip:(side>0?right_tip:-1);
 if(blob.side && (!side||blob.side==side) && blob.bottom>tip)tip=blob.bottom;
 track_ring11_entry_row=tip;
 entry_ready=tip>=(int)TRACK_CONT_SCAN_TOP_ROW-8 || (shape_side && (!side || shape_side==side));
 /* Recovery must evaluate the saved ring route, not the normal-road route.
  * Otherwise a valid straight line restores ENTER and immediately loses the
  * ring target again, alternating LOOK=90 with the far ring target. */
 phase=state==FSM_RECOVER?saved_state:state;
 planning=phase>=FSM_RING_CANDIDATE && phase<=FSM_RING_MERGE;
 look=TRACK_CONT_TARGET_ROW;
 if(planning && phase!=FSM_RING_MERGE){
  look=TRACK_RING11_LOOKAHEAD_ROW;
  if(phase<=FSM_RING_ENTER && tip>=0 && tip-6<look)look=tip-6;
  look=clip(look,TRACK_RING11_TOP_ROW+4,TRACK_CONT_TARGET_ROW);
 }
 track_ring11_lookahead_row=look;
 base_good=base.anchor_confidence>=TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE && base.target_confidence>=TRACK_DRIVE_MIN_TARGET_CONFIDENCE && base.valid_rows>=TRACK_DRIVE_MIN_VALID_ROWS;
 active=phase>=FSM_RING_CANDIDATE && phase<=FSM_RING_MERGE;
 memcpy(track_ring11_path_x,track_mode4_path_x,sizeof(track_ring11_path_x));
 memcpy(track_ring11_path_confidence,track_mode4_path_confidence,sizeof(track_ring11_path_confidence));
 if(planning)memset(track_ring11_path_confidence,0,sizeof(track_ring11_path_confidence));
 for(y=TRACK_RING11_TOP_ROW;y<=118;++y){
  int c,l,r,lr,rr;
  if(!trace_conf[y])continue;
  l=trace_left[y];r=trace_right[y];c=trace_center[y];
  if(planning){track_ring11_path_x[y]=c;track_ring11_path_confidence[y]=trace_conf[y];}
  lr=l>2;rr=r<185;
  if(lr&&rr){if(y>=TRACK_CONT_SCAN_TOP_ROW)++bilateral;
   if(state==FSM_NORMAL && !blob.side){int measured=r-l+1;width_profile[y]=(width_profile[y]*7+measured)/8;}}
  if(side){int ir=side<0?lr:rr,orr=side<0?rr:lr,edge=side<0?l:r;
   if(y>=TRACK_CONT_TARGET_ROW){inner_near+=ir;outer_near+=orr;inner_near_loss+=!ir;}
   else {inner_far_loss+=!ir;
   if(prev_edge>=0 && (edge-prev_edge>TRACK_FSM_CORNER_DELTA || prev_edge-edge>TRACK_FSM_CORNER_DELTA))++far_corner;
   prev_edge=edge;}
   /* Entry follows the observed outer edge; inside follows the island edge.
    * Missing edges never manufacture confidence or a drive permission. */
   if(planning && phase!=FSM_RING_MERGE){int half=width_profile[y]/2;
    /* The island-side branch owns the path throughout entry/interior.
     * An available straight-road centerline must not override it. */
    if(ir)c=side<0?l+half:r-half;
    else if(orr)c=side<0?r-half:l+half;
    else if(ir)c=side<0?l+half:r-half;
    else {track_ring11_path_confidence[y]=0;continue;}
    track_ring11_path_x[y]=clip(c,l+2,r-2);
   }
  }
  if(y>=TRACK_CONT_SCAN_TOP_ROW)++observed;
   if(y==TRACK_CONT_ANCHOR_ROW)anchor=track_ring11_path_confidence[y];
   if(y==(unsigned)look)target=track_ring11_path_confidence[y];
 }
 if(planning){
  int row,best=-1,distance=1000;
  for(row=clip(look-12,TRACK_RING11_TOP_ROW+4,TRACK_CONT_TARGET_ROW);
      row<=clip(look+12,TRACK_RING11_TOP_ROW+4,TRACK_CONT_TARGET_ROW);++row){
   int d=row>look?row-look:look-row;
   if(trace_conf[row] && track_ring11_path_confidence[row]>=TRACK_DRIVE_MIN_TARGET_CONFIDENCE && d<distance){best=row;distance=d;}
  }
  if(best>=0)look=best;
  target=best>=0?track_ring11_path_confidence[best]:0;
  track_ring11_lookahead_row=look;
 }
 planned_good=anchor>=TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE && target>=TRACK_DRIVE_MIN_TARGET_CONFIDENCE && observed>=TRACK_DRIVE_MIN_VALID_ROWS;
 good=active?planned_good:base_good;
 if(!run || dt>=TRACK_CAMERA_LOST_TIMEOUT_MS)track_ring11_suspend(!run?FSM_REASON_STOP:FSM_REASON_CAMERA);
 else {
  if(state>=FSM_RING_CANDIDATE && state<=FSM_RING_MERGE)element_ms+=dt;
  if(element_ms>=TRACK_FSM_ELEMENT_TIMEOUT_MS){track_ring11_suspend(FSM_REASON_ELEMENT_TIMEOUT);good=0;}
  else if(state==FSM_RECOVER){missing_ms+=dt;
   if(confirm(good,dt)){transition(saved_state,FSM_REASON_RECOVERED);missing_ms=0;}else if(missing_ms>=TRACK_FSM_RECOVERY_MS)track_ring11_suspend(FSM_REASON_RECOVERY_TIMEOUT);}
  else if(!good && state!=FSM_WAIT && !(state==FSM_RING_CANDIDATE && entry_ready && planned_good)){saved_state=state;transition(FSM_RECOVER,FSM_REASON_LOST);missing_ms=0;}
  else switch(state){
   case FSM_WAIT:if(confirm(base_good,dt))transition(FSM_NORMAL,FSM_REASON_CONFIRMED);
   break;
   case FSM_NORMAL:
    if(rearm){if(!blob.side && base_good)clear_ms+=dt;else clear_ms=0;
   if(clear_ms>=300){rearm=0;clear_ms=0;}break;}
    if(blob.side && base_good){side=blob.side;element_ms=0;transition(FSM_RING_CANDIDATE,FSM_REASON_RING);}break;
   case FSM_RING_CANDIDATE:
    if(blob.side && blob.side!=side){side=0;transition(FSM_NORMAL,FSM_REASON_REJECTED);}
    else if(confirm((blob.side==side || (side<0?left_tip:right_tip)>=0) && entry_ready && planned_good,dt)){interior=far_exit=0;inside_ms=0;transition(FSM_RING_ENTER,FSM_REASON_ENTRY);}
    else if(!blob.side && (side<0?left_tip:right_tip)<0){missing_ms+=dt;
   if(missing_ms>200){side=0;missing_ms=0;transition(FSM_NORMAL,FSM_REASON_REJECTED);}}
    else missing_ms=0;
   break;
   case FSM_RING_ENTER:
    if((blob.side==side && blob.bottom>=(int)TRACK_CONT_TARGET_ROW) || (side<0?left_tip:right_tip)>=(int)TRACK_CONT_TARGET_ROW-8)body_seen=1;
    if(confirm(body_seen && inner_near>=TRACK_FSM_SUPPORT_ROWS && outer_near>=TRACK_FSM_SUPPORT_ROWS,dt)){interior=1;inside_ms=0;transition(FSM_RING_INSIDE,FSM_REASON_INTERIOR);}break;
   case FSM_RING_INSIDE:
    inside_ms+=dt;
    if(interior && inside_ms>=TRACK_FSM_CONFIRM_MS && inner_far_loss>=TRACK_FSM_FEATURE_ROWS && far_corner>=2)far_exit=1;
    if(confirm(far_exit && inner_near_loss>=TRACK_FSM_FEATURE_ROWS && outer_near>=TRACK_FSM_SUPPORT_ROWS,dt))transition(FSM_RING_EXIT,FSM_REASON_EXIT_NEAR);
   break;
   case FSM_RING_EXIT:
    if(confirm(inner_near>=TRACK_FSM_SUPPORT_ROWS && good,dt))transition(FSM_RING_MERGE,FSM_REASON_MERGED);
   break;
   case FSM_RING_MERGE:
    if(confirm(base_good && bilateral>=TRACK_DRIVE_MIN_VALID_ROWS,dt)){side=0;interior=far_exit=body_seen=0;element_ms=0;rearm=1;transition(FSM_NORMAL,FSM_REASON_MERGED);}break;
   default:track_ring11_suspend(FSM_REASON_REJECTED);
   break;
  }
 }
 active=state>=FSM_RING_ENTER&&state<=FSM_RING_MERGE;
 use_plan=(state>=FSM_RING_CANDIDATE && state<=FSM_RING_MERGE) || (state==FSM_RECOVER && planning);
 /* Transition from NORMAL was planned with its old phase: keep that first
  * frame on the ordinary path, and switch display/control together next frame. */
 if(!planning)use_plan=0;
 out.anchor_x=use_plan?track_ring11_path_x[TRACK_CONT_ANCHOR_ROW]:base.anchor_x;
 out.target_x=use_plan?track_ring11_path_x[look]:base.target_x;
 out.anchor_confidence=use_plan?anchor:base.anchor_confidence;out.target_confidence=use_plan?target:base.target_confidence;
 out.valid_rows=use_plan?observed:base.valid_rows;out.global_threshold=base.global_threshold;
 /* Match algorithm 3's integer-degree rounding on ordinary road. */
 out.servo_cdeg=100*clip((int)TRACK_SERVO_CENTER_DEG-
     ((int)out.target_x-(int)TRACK_STEER_REFERENCE_X)*(int)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL/100,
     TRACK_SERVO_RIGHT_DEG,TRACK_SERVO_LEFT_DEG);
 out.drive_allowed=run&&good&&state!=FSM_WAIT&&state!=FSM_RECOVER;
 out.pwm_cap_percent=state>=FSM_RING_CANDIDATE&&state<=FSM_RING_MERGE? (TRACK_FSM_ELEMENT_PWM_PERCENT<TRACK_WHEEL_PWM_MAX_PERCENT?TRACK_FSM_ELEMENT_PWM_PERCENT:TRACK_WHEEL_PWM_MAX_PERCENT):TRACK_WHEEL_PWM_MAX_PERCENT;
 out.state=state;out.reason=reason;out.ring_side=side;out.interior_seen=interior;out.exit_far_seen=far_exit;out.near_open=inner_near_loss;out.far_open=inner_far_loss;out.observed_rows=observed;
 return out;
}
