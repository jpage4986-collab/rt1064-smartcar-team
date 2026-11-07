/* Algorithm14 live-camera planner. Projection assumes HFOV90, pitch30
 * (optical axis 60 degrees from vertical), height .30m, offset .095m.
 * HFOV/offset are not measured hardware intrinsics.
 * Ring centre prediction uses observed image motion only, never simulated
 * speed/yaw or uncalibrated vehicle sensors.
 * Full-orbit integration/metric exit distance require hardware calibration. */
#include "track_ring14.h"
#include "track_mode4.h"
#include "tune_config.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define NG 10
#define NP 192
typedef struct {float f,l;} point_t;
typedef struct {int n,top,bottom,left,right,bottom_left,bottom_right;point_t p[NP];} group_t;
typedef struct {float f,l,r,e,ix,iy,arc_span;int n,arc_bins,closed,valid,image_valid;} circle_t;
static group_t groups[NG];
static uint8_t binary[120][188];
static const float camera_sin_pitch=.5f,camera_cos_pitch=.866025404f;
int16_t track_ring14_path_x[120];
uint8_t track_ring14_path_confidence[120];
volatile uint32_t track_ring14_lookahead_row=90,track_ring14_points,track_ring14_loss_ms,track_ring14_rejects;
volatile uint32_t track_ring14_joined_candidate,track_ring14_element_confidence;
volatile uint32_t track_ring14_arc_degrees;
volatile uint8_t track_ring14_half_arc_candidate;
volatile uint8_t track_ring14_display_phase;
volatile int16_t track_ring14_center_x=-1,track_ring14_center_y=-1;
volatile uint8_t track_ring14_center_visible;
volatile uint8_t track_ring14_center_predicted;
volatile uint32_t track_ring14_center_age_ms;
volatile int32_t track_ring14_side,track_ring14_forward_mm,track_ring14_lateral_mm,track_ring14_radius_mm,track_ring14_error_mm;
static circle_t pending,ring;
static uint8_t state,reason;
static unsigned confirm_frames,confirm_ms,loss_ms;
static uint8_t ring_started,ring_exiting,ring_connected,ring_have_bearing,exit_clear_frames,no_ring_good_frames;
static uint8_t ring_center_seen_once;
static int8_t ring_turn_sign;
static uint32_t ring_center_lost_ms;
static uint32_t ring_turn_ms;
static float center_track_x,center_track_y,center_vx,center_vy;
static uint32_t center_track_age_ms;
static uint8_t center_track_valid,center_track_predicted;
static uint32_t cooldown_ms;
static float last_bearing,orbit_progress;
static int last_target=88,last_servo=8800;
static float road_width=.42f;
static int clip(int v,int lo,int hi){return v<lo?lo:v>hi?hi:v;}
static int project_unbounded(point_t p,float *x,float *y){
  float f=p.f-.095f,n=sqrtf(f*f+p.l*p.l+.09f),sb=(f*camera_sin_pitch-.30f*camera_cos_pitch)/n;
  float z=(f*camera_cos_pitch+.30f*camera_sin_pitch)/n;
 if(n<.001f||z<=0||fabsf(sb)>=1)return 0;
 *x=((p.l/n)/z+1)*94-.5f;*y=(1-(sb/sqrtf(1-sb*sb))/(120.f/188))*60-.5f;
 return isfinite(*x)&&isfinite(*y);
}
static int center_in_fov(float x,float y){return x>=0.f&&x<188.f&&y>=0.f&&y<120.f;}
static int ground(float x,float y,point_t *p){
 float a=atanf(((x+.5f)/188*2-1)),b=atanf((1-(y+.5f)/120*2)*(120.f/188));
  float ca=cosf(a),cb=cosf(b),sb=sinf(b),d=cb*ca*camera_sin_pitch-sb*camera_cos_pitch;
 if(d<=.006f)return 0;
  p->f=(cb*ca*camera_cos_pitch+sb*camera_sin_pitch)/d*.30f+.095f;p->l=cb*sinf(a)/d*.30f;
 return isfinite(p->f)&&p->f<2.f;
}
static int project(point_t p,float *x,float *y){
  float f=p.f-.095f,n=sqrtf(f*f+p.l*p.l+.09f),sb=(f*camera_sin_pitch-.30f*camera_cos_pitch)/n;
  float z=(f*camera_cos_pitch+.30f*camera_sin_pitch)/n;
 if(n<.001f||z<=0||fabsf(sb)>=1)return 0;
 *x=((p.l/n)/z+1)*94-.5f;*y=(1-(sb/sqrtf(1-sb*sb))/(120.f/188))*60-.5f;
 return *x>=1&&*x<187&&*y>=1&&*y<119;
}
static circle_t fit(group_t *g){
 circle_t c={0};float m[3][4]={{0}},r2;uint16_t angle_bins=0;int i,j,k,t,max_empty=0;
  if(g->n<16)return c;
 for(t=0;t<g->n;t++){float v[3]={g->p[t].f,g->p[t].l,1},z=-(v[0]*v[0]+v[1]*v[1]);for(i=0;i<3;i++){for(j=0;j<3;j++)m[i][j]+=v[i]*v[j];m[i][3]+=v[i]*z;}}
 for(i=0;i<3;i++){float q; k=i;for(j=i+1;j<3;j++)if(fabsf(m[j][i])>fabsf(m[k][i]))k=j;
  for(t=i;t<4;t++){q=m[k][t];m[k][t]=m[i][t];m[i][t]=q;}q=m[i][i];if(fabsf(q)<1e-8f)return c;
  for(t=i;t<4;t++)m[i][t]/=q;for(j=0;j<3;j++)if(j!=i){q=m[j][i];for(t=i;t<4;t++)m[j][t]-=q*m[i][t];}}
 c.f=-m[0][3]/2;c.l=-m[1][3]/2;r2=c.f*c.f+c.l*c.l-m[2][3];if(r2<=0)return c;c.r=sqrtf(r2);
 for(i=0;i<g->n;i++){
  float a=atan2f(g->p[i].l-c.l,g->p[i].f-c.f);int bin;
  c.e+=fabsf(hypotf(g->p[i].f-c.f,g->p[i].l-c.l)-c.r);
  if(a<0)a+=6.283185307f;
  bin=(int)(a*(16.f/6.283185307f));if(bin>15)bin=15;
  angle_bins|=(uint16_t)(1U<<bin);
 }
 c.e/=g->n;c.n=g->n;
 /* Estimate visible arc coverage from angular bins around the fitted centre. */
 for(i=0;i<16;i++)if(angle_bins&(uint16_t)(1U<<i)){
  c.arc_bins++;
  int empty=0;
  for(j=1;j<16;j++)if(!(angle_bins&(uint16_t)(1U<<((i+j)&15))))empty++;else break;
  if(empty>max_empty)max_empty=empty;
 }
 c.arc_span=(16-max_empty)*(6.283185307f/16.f);
 c.valid=c.r>=.12f&&c.r<=.40f&&c.f>-.1f&&c.f<1.6f&&fabsf(c.l)<1&&c.e<.035f;
  if(c.valid){point_t center;center.f=c.f;center.l=c.l;c.image_valid=project_unbounded(center,&c.ix,&c.iy);}
 /* A bounded dark gap must merge into white pavement above and below.
  * No condition requires the circle centre itself to be inside the image. */
 if(g->bottom<114&&g->top>21){int yy,above=0,below=0;
  for(yy=g->top-1;yy>=g->top-8&&yy>=0;yy--){int white=1;for(i=g->left;i<=g->right;i++)if(!binary[yy][i]){white=0;break;}if(white){above=1;break;}}
  for(yy=g->bottom+1;yy<=g->bottom+8&&yy<120;yy++){int white=1;
   for(k=g->bottom_left;k<=g->bottom_right;k++)if(!binary[yy][k]){white=0;break;}if(white){below=1;break;}}
  c.closed=above&&below;}
 return c;
}
static circle_t detect(uint32_t dt){
  circle_t best={0};float best_score=1.0e9f,pred_x=center_track_x+center_vx*(float)dt,pred_y=center_track_y+center_vy*(float)dt;int y,x,i,used=0;memset(groups,0,sizeof(groups));
 for(y=116;y>=20;y--){int prev=-1;
  for(x=0;x<188;){int left,right,gap;group_t *g=0;
   if(!binary[y][x]){x++;continue;}left=x;while(x<188&&binary[y][x])x++;right=x-1;if(right-left<3)continue;
   if(prev>=0&&left-prev>=5){gap=left-1;
    for(i=0;i<used;i++)if(groups[i].top-y<=2&&groups[i].top>y&&groups[i].left<=gap&&groups[i].right>=prev+1){g=&groups[i];break;}
   if(!g&&used<NG){g=&groups[used++];g->bottom=y;g->bottom_left=prev+1;g->bottom_right=gap;}
    if(g){point_t p;g->top=y;g->left=prev+1;g->right=gap;
     if(g->n<NP-1&&ground(prev+1,y,&p))g->p[g->n++]=p;
     if(g->n<NP&&ground(gap,y,&p))g->p[g->n++]=p;}}
   prev=right;}}
  for(i=0;i<used;i++){circle_t c=fit(&groups[i]);if(c.valid){float score;
    if(center_track_valid&&center_track_age_ms<700U&&c.image_valid)score=hypotf(c.ix-pred_x,c.iy-pred_y);
    else score=hypotf(c.f,c.l);
    if(!best.valid||score<best_score){best=c;best_score=score;}}}
 return best;
}
void track_ring14_reset(void){track_mode4_reset();memset(&ring,0,sizeof(ring));memset(&pending,0,sizeof(pending));confirm_frames=confirm_ms=loss_ms=0;ring_started=ring_exiting=ring_connected=ring_have_bearing=exit_clear_frames=no_ring_good_frames=0;ring_center_seen_once=0;ring_turn_sign=1;ring_center_lost_ms=ring_turn_ms=0;center_track_x=center_track_y=center_vx=center_vy=0;center_track_age_ms=0;center_track_valid=center_track_predicted=track_ring14_center_predicted=0;track_ring14_center_age_ms=0;cooldown_ms=0;last_bearing=orbit_progress=0;track_ring14_joined_candidate=track_ring14_element_confidence=0;track_ring14_arc_degrees=0;track_ring14_half_arc_candidate=0;track_ring14_display_phase=0;track_ring14_center_x=track_ring14_center_y=-1;track_ring14_center_visible=0;state=FSM_WAIT;reason=FSM_REASON_INIT;last_target=88;last_servo=8800;memset(track_ring14_path_confidence,0,120);}
void track_ring14_suspend(uint8_t why){track_ring14_reset();reason=why;}
const char *track_ring14_state_name(uint8_t s){switch(s){case FSM_NORMAL:return "TRACK";case FSM_RING_CANDIDATE:return "R PREP";case FSM_RING_ENTER:return "R ENTER";case FSM_RING_INSIDE:return "R ARC";case FSM_RING_EXIT:return "R EXIT";case FSM_RING_MERGE:return "R MERGE";case FSM_RECOVER:return "RECOVER";default:return "WAIT";}}
void track_ring14_binary_preview(const uint8_t in[120][188],uint8_t out[120][188],uint8_t t){(void)in;(void)t;memcpy(out,binary,sizeof(binary));}
/* Ordinary roads use the exact algorithm-3 output contract. Element recovery
 * and arc filtering must never change normal-road target or actuator gating. */
static track_fsm_result_t ordinary(track_mode4_result_t b,uint32_t dt,uint8_t run){
 track_fsm_result_t o={0};
  o.anchor_x=b.anchor_x;o.anchor_confidence=b.anchor_confidence;
 o.target_confidence=b.target_confidence;o.valid_rows=b.valid_rows;o.observed_rows=b.valid_rows;o.global_threshold=b.global_threshold;
  /* Match Algorithm 3's target directly. Steering lag belongs to the
   * configured angle cap only; do not add A14-only temporal smoothing here. */
  o.target_x=b.target_x;
  o.servo_cdeg=100*clip((int)TRACK_SERVO_CENTER_DEG-((int)o.target_x-(int)TRACK_STEER_REFERENCE_X)*(int)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL/100,
      (int)TRACK_SERVO_CENTER_DEG-(int)TRACK_A3_MAX_STEER_DELTA_DEG,
      (int)TRACK_SERVO_CENTER_DEG+(int)TRACK_A3_MAX_STEER_DELTA_DEG);
 o.drive_allowed=run&&dt<TRACK_CAMERA_LOST_TIMEOUT_MS&&b.anchor_confidence>=TRACK_DRIVE_MIN_ANCHOR_CONFIDENCE&&b.target_confidence>=TRACK_DRIVE_MIN_TARGET_CONFIDENCE&&b.valid_rows>=TRACK_DRIVE_MIN_VALID_ROWS;
 o.state=(pending.valid||ring.valid)?FSM_RING_CANDIDATE:FSM_NORMAL;
 o.reason=FSM_REASON_INIT;o.pwm_cap_percent=TRACK_WHEEL_PWM_MAX_PERCENT;
 track_ring14_lookahead_row=TRACK_CONT_TARGET_ROW;track_ring14_side=0;track_ring14_loss_ms=0;
 if(o.drive_allowed){last_target=o.target_x;last_servo=o.servo_cdeg;}
 return o;
}
track_fsm_result_t track_ring14_process(const uint8_t image[120][188],uint32_t dt,uint8_t run){
 track_fsm_result_t o={0};track_mode4_result_t base=track_mode4_process(image);circle_t seen;uint8_t element_conf=0;track_mode4_element_t element;int joined=0,enter_now=0;int y,n=0,arc_n=0,target_y=90,prior_y=-1,prior_x=0;
 if(cooldown_ms>dt)cooldown_ms-=dt;else cooldown_ms=0;
 track_mode4_binary_preview(image,binary);memcpy(track_ring14_path_x,track_mode4_path_x,sizeof(track_ring14_path_x));memcpy(track_ring14_path_confidence,track_mode4_path_confidence,120);
 /* The ring view often contains only an open island arc connected to a
  * junction. In that case a closed-component test is too strict. Accept the
  * partial-circle fit only when its error is especially low and the ordinary
  * row topology independently reports a junction/branch. */
 element=track_mode4_detect_element(binary,&element_conf);
  seen=detect(dt);track_ring14_points=seen.n;track_ring14_error_mm=seen.valid?(int)(seen.e*1000):-1;
  track_ring14_arc_degrees=(uint32_t)(seen.arc_span*57.29578f+.5f);
  track_ring14_center_visible=0;track_ring14_center_x=track_ring14_center_y=-1;track_ring14_center_age_ms=0;center_track_predicted=0;track_ring14_center_predicted=0;
 joined=(seen.valid&&!seen.closed&&seen.e<.015f&&
         (element==TRACK_ELEMENT_CROSS_CANDIDATE||
          element==TRACK_ELEMENT_RING_LEFT_CANDIDATE||
          element==TRACK_ELEMENT_RING_RIGHT_CANDIDATE));
 track_ring14_half_arc_candidate=(uint8_t)(seen.valid&&!seen.closed&&!joined&&seen.n>=16&&seen.arc_bins>=6&&seen.arc_span>=2.74889357f&&seen.e<.020f);
 track_ring14_joined_candidate=(uint32_t)joined;track_ring14_element_confidence=element_conf;
  /* Track the fitted geometric centre separately from the island-edge fit.
   * A clipped or temporarily unfit arc does not mean the centre disappeared. */
  {
   float mx,my;int measured=seen.valid&&(ring.valid||pending.valid||seen.closed||joined||track_ring14_half_arc_candidate),accepted=0;
   if(dt<250U&&center_track_valid){center_track_x+=center_vx*(float)dt;center_track_y+=center_vy*(float)dt;}
   if(measured){point_t c={seen.f,seen.l};if(project_unbounded(c,&mx,&my)){
    if(!center_track_valid){center_track_x=mx;center_track_y=my;center_vx=center_vy=0;center_track_valid=1;center_track_age_ms=0;accepted=1;}
    else if(center_track_age_ms<700U&&hypotf(mx-center_track_x,my-center_track_y)<=24.f+(float)dt*.60f+hypotf(center_vx,center_vy)*(float)dt*1.5f){
     float md=(float)(center_track_age_ms+dt);if(md<1.f)md=(float)(dt?dt:1U);
     {float ex=mx-center_track_x,ey=my-center_track_y;center_track_x+=.70f*ex;center_track_y+=.70f*ey;center_vx=clip((int)(center_vx*1000.f+.18f*ex*1000.f/md),-1200,1200)/1000.f;center_vy=clip((int)(center_vy*1000.f+.18f*ey*1000.f/md),-1200,1200)/1000.f;}
     center_track_age_ms=0;accepted=1;
    }else if(center_track_age_ms>=700U){center_track_x=mx;center_track_y=my;center_vx=center_vy=0;center_track_age_ms=0;accepted=1;}
   }}
   if(center_track_valid){if(!accepted&&center_track_age_ms<60000U)center_track_age_ms+=dt;center_track_predicted=(uint8_t)!accepted;
    track_ring14_center_x=(int16_t)clip((int)(center_track_x+.5f),-32768,32767);track_ring14_center_y=(int16_t)clip((int)(center_track_y+.5f),-32768,32767);
     track_ring14_center_visible=(uint8_t)(center_track_age_ms<=300U&&center_in_fov(center_track_x,center_track_y));track_ring14_center_predicted=center_track_predicted;track_ring14_center_age_ms=center_track_age_ms;}
  }
 if(seen.valid){no_ring_good_frames=0;
   if(ring.valid){if(!center_track_predicted&&seen.image_valid){ring=seen;loss_ms=0;no_ring_good_frames=0;
    if(ring_started&&!ring_exiting){float b=atan2f(ring.l,ring.f);if(ring.f<=.05f){if(!ring_have_bearing){last_bearing=b;ring_have_bearing=1;}else{float db=b-last_bearing;while(db>3.14159265f)db-=6.28318531f;while(db< -3.14159265f)db+=6.28318531f;orbit_progress+=(ring.l<0?-1.f:1.f)*db;last_bearing=b;}}
     if(orbit_progress<0)orbit_progress=0;if(orbit_progress>6.28318531f)orbit_progress=6.28318531f;
     if(orbit_progress>=(ring_connected?4.71238898f:6.10865238f)){ring_exiting=1;state=FSM_RING_EXIT;reason=FSM_REASON_EXIT_FAR;exit_clear_frames=0;}}
   }else {track_ring14_rejects++;loss_ms+=dt;}}
   else if(ring_started&&(seen.closed||joined)&&!center_track_predicted&&seen.image_valid){ring=seen;loss_ms=0;no_ring_good_frames=0;}
   else if(!ring_started&&cooldown_ms==0&&(seen.closed||joined||track_ring14_half_arc_candidate)&&!center_track_predicted&&seen.image_valid){
    if(pending.valid&&pending.image_valid&&hypotf(seen.ix-pending.ix,seen.iy-pending.iy)<30.f){confirm_frames++;confirm_ms+=dt;}
    else{confirm_frames=1;confirm_ms=0;}
    pending=seen;state=FSM_RING_CANDIDATE;
    if(confirm_frames>=2U&&confirm_ms>=20U){ring=seen;ring_connected=(uint8_t)joined;ring_started=ring_exiting=ring_have_bearing=0;orbit_progress=0;ring_center_seen_once=center_track_valid;ring_center_lost_ms=0;ring_turn_sign=ring.l<0?-1:1;track_ring14_display_phase=1;state=FSM_RING_CANDIDATE;reason=FSM_REASON_RING;}}
 }else if(!ring.valid){pending.valid=0;confirm_frames=confirm_ms=0;state=FSM_NORMAL;}
 if(ring.valid&&!seen.valid){loss_ms+=dt;
  if(ring_exiting){if(base.anchor_confidence>=50&&base.target_confidence>=50&&base.valid_rows>=55){if(exit_clear_frames<255)exit_clear_frames++;}else exit_clear_frames=0;}
  else if(ring_started&&orbit_progress>=(ring_connected?4.71238898f:6.10865238f)&&element==TRACK_ELEMENT_NORMAL&&base.anchor_confidence>=50&&base.target_confidence>=50&&base.valid_rows>=55){if(no_ring_good_frames<255)no_ring_good_frames++;if(no_ring_good_frames>=4){ring_exiting=1;state=FSM_RING_EXIT;reason=FSM_REASON_EXIT_FAR;exit_clear_frames=0;}}
  else no_ring_good_frames=0;
 }
 if(ring_exiting&&exit_clear_frames>=4){memset(&ring,0,sizeof(ring));memset(&pending,0,sizeof(pending));ring_started=ring_exiting=ring_connected=ring_have_bearing=no_ring_good_frames=0;track_ring14_display_phase=0;orbit_progress=0;loss_ms=0;cooldown_ms=600;state=FSM_RING_MERGE;reason=FSM_REASON_MERGED;}
  /* Keep A3 until the tracked geometric centre itself is outside the image.
   * Missing/clipped island-edge pixels only trigger bounded image-space prediction. */
 if(ring.valid&&!ring_started){
  if(center_track_valid&&center_in_fov(center_track_x,center_track_y)){ring_center_seen_once=1;ring_center_lost_ms=0;ring_turn_sign=ring.l<0?-1:1;
   if(center_track_age_ms>700U){memset(&ring,0,sizeof(ring));memset(&pending,0,sizeof(pending));center_track_valid=center_track_predicted=track_ring14_center_predicted=0;track_ring14_center_age_ms=0;track_ring14_center_visible=0;track_ring14_center_x=track_ring14_center_y=-1;ring_center_seen_once=0;ring_center_lost_ms=0;confirm_frames=confirm_ms=0;track_ring14_display_phase=0;cooldown_ms=200;state=FSM_NORMAL;reason=FSM_REASON_INIT;}}
  else if(ring_center_seen_once&&center_track_valid){ring_center_lost_ms+=dt;if(ring_center_lost_ms>=500U){ring_started=1;track_ring14_display_phase=2;ring_turn_ms=0;enter_now=1;state=FSM_RING_ENTER;reason=FSM_REASON_ENTRY;}}
 }
 /* Apply a finite steering pulse toward the last observed centre side, then
  * hand control back to the ordinary A3 path instead of following a canned lap. */
 if(ring_started&&track_ring14_display_phase==2&&ring_center_lost_ms>=500U){
  int fx=clip((int)base.target_x+(int)ring_turn_sign*24,4,183);
  memset(track_ring14_path_confidence,0,120);n=0;
  for(y=45;y<117;y++){track_ring14_path_x[y]=(int16_t)fx;track_ring14_path_confidence[y]=40;n++;}
  target_y=90;
  o.anchor_x=base.anchor_x;o.target_x=(int16_t)fx;o.anchor_confidence=base.anchor_confidence;
  o.target_confidence=40;o.valid_rows=(uint8_t)n;o.observed_rows=base.valid_rows;o.global_threshold=base.global_threshold;
  o.drive_allowed=run&&dt<250&&base.anchor_confidence>=25&&base.target_confidence>=25&&base.valid_rows>=27;
  o.servo_cdeg=100*clip((int)TRACK_SERVO_CENTER_DEG-((int)o.target_x-(int)TRACK_STEER_REFERENCE_X)*(int)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL/100,TRACK_SERVO_RIGHT_DEG,TRACK_SERVO_LEFT_DEG);
  o.state=FSM_RING_ENTER;o.reason=FSM_REASON_ENTRY;o.ring_side=ring_turn_sign;o.pwm_cap_percent=TRACK_WHEEL_PWM_MAX_PERCENT;
  if(o.drive_allowed){ring_turn_ms+=dt;last_target=o.target_x;last_servo=o.servo_cdeg;}
  else{o.servo_cdeg=8800;}
  if(ring_turn_ms>=1000U){memset(&ring,0,sizeof(ring));memset(&pending,0,sizeof(pending));ring_started=ring_exiting=ring_connected=ring_have_bearing=0;ring_center_seen_once=0;center_track_valid=center_track_predicted=0;center_track_age_ms=0;ring_center_lost_ms=ring_turn_ms=0;track_ring14_display_phase=0;cooldown_ms=600;state=FSM_NORMAL;reason=FSM_REASON_MERGED;return ordinary(base,dt,run);}
  track_ring14_lookahead_row=target_y;track_ring14_side=ring_turn_sign;track_ring14_loss_ms=loss_ms;
  track_ring14_forward_mm=ring.f*1000;track_ring14_lateral_mm=ring.l*1000;track_ring14_radius_mm=ring.r*1000;
  return o;
 }
 /* Keep EXEC latched on visual loss. The 200ms limit disables drive below;
  * reacquired visual circles can resume planning, but ordinary A3 cannot take
  * over and claim the ring has ended before the orbit/merge condition. */
 if(ring.valid&&ring_started&&!ring_exiting&&loss_ms>200){state=FSM_WAIT;reason=FSM_REASON_RECOVERY_TIMEOUT;}
 /* Before the three-frame confirmation, PREP uses the exact A3 path. Once
  * confirmed, execution remains latched until merge or the 200ms loss timeout. */
 if(ring_exiting||!ring_started){track_fsm_result_t normal=ordinary(base,dt,run);if(ring.valid&&!ring_exiting){normal.state=FSM_RING_CANDIDATE;normal.reason=FSM_REASON_RING;normal.ring_side=ring.l<0?-1:1;}else if(ring_exiting){normal.state=FSM_RING_EXIT;normal.reason=reason;normal.ring_side=ring.l<0?-1:1;}else if(state==FSM_RING_MERGE){normal.state=FSM_RING_MERGE;normal.reason=FSM_REASON_MERGED;}return normal;}
 if(ring.valid){
  float R=ring.r+road_width*.5f,d=hypotf(ring.f,ring.l),s,sign=ring.l<0?-1.f:1.f;
  if(loss_ms>200){ring.valid=0;state=FSM_WAIT;reason=FSM_REASON_RECOVERY_TIMEOUT;}
  else if(d>.01f){memset(track_ring14_path_confidence,0,120);
   for(s=0;s<1.6f;s+=.015f){float a=sign*s/R,px,py;point_t p;
    p.f=ring.f+R*(-ring.f/d*cosf(a)+ring.l/d*sinf(a));p.l=ring.l+R*(-ring.f/d*sinf(a)-ring.l/d*cosf(a));
    if(project(p,&px,&py)){int yy=(int)(py+.5f),xx=(int)(px+.5f),k;
     /* Visible route must lie on observed white pavement. */
     if(!binary[yy][xx]){prior_y=-1;continue;}
     track_ring14_path_x[yy]=xx;track_ring14_path_confidence[yy]=70;arc_n++;
     if(prior_y>=0&&abs(yy-prior_y)<=12){int lo=yy<prior_y?yy:prior_y,hi=yy>prior_y?yy:prior_y;
      for(k=lo+1;k<hi;k++){int ix=prior_x+(xx-prior_x)*(k-prior_y)/(yy-prior_y);if(binary[k][ix]){track_ring14_path_x[k]=ix;track_ring14_path_confidence[k]=50;}}}
     prior_y=yy;prior_x=xx;}}
   state=loss_ms>200?FSM_WAIT:(loss_ms?FSM_RECOVER:(enter_now?FSM_RING_ENTER:FSM_RING_INSIDE));}}
 for(y=20;y<117;y++)if(track_ring14_path_confidence[y]>=25)n++;
 if(ring.valid){int best=200;for(y=30;y<112;y++)if(track_ring14_path_confidence[y]>=25&&abs(y-90)<best){best=abs(y-90);target_y=y;}}
 o.anchor_x=base.anchor_x;o.target_x=track_ring14_path_x[target_y];o.anchor_confidence=base.anchor_confidence;o.target_confidence=track_ring14_path_confidence[target_y];o.valid_rows=n;o.global_threshold=base.global_threshold;o.observed_rows=base.valid_rows;
 o.drive_allowed=run&&dt<250&&base.anchor_confidence>=25&&o.target_confidence>=25&&n>=27&&(!ring.valid||arc_n>=4);
 o.servo_cdeg=100*clip((int)TRACK_SERVO_CENTER_DEG-((int)o.target_x-(int)TRACK_STEER_REFERENCE_X)*(int)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL/100,TRACK_SERVO_RIGHT_DEG,TRACK_SERVO_LEFT_DEG);
 if(!ring.valid&&state!=FSM_WAIT)state=FSM_NORMAL;
 if(!o.drive_allowed){if(loss_ms<=200)loss_ms+=dt;state=loss_ms<=200?FSM_RECOVER:FSM_WAIT;o.servo_cdeg=last_servo;
  o.drive_allowed=run&&dt<250&&loss_ms<=200&&last_target!=88;
 }else if(!ring.valid)loss_ms=0;
 if(abs(o.target_x-last_target)>24&&(o.target_x-88)*(last_target-88)<0){o.drive_allowed=0;track_ring14_rejects++;state=FSM_RECOVER;}
 /* STOP disables actuation, while camera-only planning keeps updating. */
 if(dt>=250){track_ring14_suspend(FSM_REASON_CAMERA);o.drive_allowed=0;}
 if(o.drive_allowed){last_servo=o.servo_cdeg;last_target=o.target_x;}else o.servo_cdeg=8800;
 o.state=state;o.reason=reason;o.ring_side=ring.valid?(ring.l<0?-1:1):0;o.pwm_cap_percent=state==FSM_RECOVER?15:TRACK_WHEEL_PWM_MAX_PERCENT;
 track_ring14_lookahead_row=target_y;track_ring14_side=o.ring_side;track_ring14_loss_ms=loss_ms;
 track_ring14_forward_mm=ring.f*1000;track_ring14_lateral_mm=ring.l*1000;track_ring14_radius_mm=ring.r*1000;
 return o;
}
