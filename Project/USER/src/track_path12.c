/* Algorithm 12, original implementation: connected-obstacle geometry followed
 * by a full white-space path search with heading and curvature costs.
 * No algorithm-11 state, path, component detector or steering output is used. */
#include "track_path12.h"
#include "track_mode4.h"
#include "tune_config.h"
#include <string.h>
#include <stdlib.h>
#define INF 100000000
#define TOP 20
#define BOTTOM 116
static uint8_t mask[120][188],clearance[120][188],seen[2820];
static uint16_t flood[22560];
static int32_t costs[2][188][9];
static int8_t parent[120][188][9];
int16_t track_path12_path_x[120];
uint8_t track_path12_path_confidence[120];
volatile uint32_t track_path12_lookahead_row,track_path12_path_rows;
volatile int32_t track_path12_side,track_path12_island_row;
static int locked,pending,pending_frames,missing,phase;
static uint32_t elapsed;
static int clip12(int x,int a,int b){return x<a?a:(x>b?b:x);}
static void enqueue12(int x,int y,int *tail){int n=y*188+x;
 if(mask[y][x]||(seen[n>>3]&(1<<(n&7))))return;
 seen[n>>3]|=1<<(n&7);flood[(*tail)++]=n;
}
static int island12(int *lo,int *hi,int *top,int *bottom){
 int x,y,best=0,side=0;memset(seen,0,sizeof(seen));
 *lo=*hi=*top=*bottom=-1;
 for(y=TOP+1;y<108;++y)for(x=0;x<188;++x){int h=0,t=0,x0=188,x1=0,y0=120,y1=0,above=0,below=0,k,n;
  enqueue12(x,y,&t);if(!t)continue;
  while(h<t){n=flood[h++];int u=n%188,v=n/188;
   if(u<x0)x0=u;
   if(u>x1)x1=u;
   if(v<y0)y0=v;
   if(v>y1)y1=v;
   if(u)enqueue12(u-1,v,&t);
   if(u<187)enqueue12(u+1,v,&t);
   if(v>TOP)enqueue12(u,v-1,&t);
   if(v<110)enqueue12(u,v+1,&t);
  }
  if(t<30||t>6000||y0<=TOP||y1>=110||y1-y0<3||y1-y0>65||x1-x0<6||x1-x0>165)continue;
  for(k=x0;k<=x1;++k){above+=mask[y0-1][k]!=0;below+=mask[y1+1][k]!=0;}
  if(above*2<x1-x0+1||below*2<x1-x0+1)continue;
  if(abs((x0+x1)/2-94)<8)continue;
  if(t>best){best=t;side=x0+x1<188?-1:1;*lo=x0;*hi=x1;*top=y0;*bottom=y1;}
 }
 return side;
}
void track_path12_suspend(uint8_t why){(void)why;locked=pending=pending_frames=missing=0;phase=FSM_WAIT;elapsed=0;}
void track_path12_reset(void){track_mode4_reset();track_path12_suspend(FSM_REASON_INIT);memset(track_path12_path_confidence,0,120);}
const char *track_path12_state_name(uint8_t s){switch(s){case FSM_WAIT:return "WAIT";case FSM_NORMAL:return "PATH";case FSM_RING_CANDIDATE:return "R CHECK";case FSM_RING_ENTER:return "R ENTER";case FSM_RING_INSIDE:return "R CIRCLE";case FSM_RING_EXIT:return "R EXIT";default:return "NO PATH";}}
void track_path12_binary_preview(const uint8_t in[120][188],uint8_t out[120][188],uint8_t t){(void)in;(void)t;memcpy(out,mask,sizeof(mask));}
track_fsm_result_t track_path12_process(const uint8_t image[120][188],uint32_t dt,uint8_t run){
 track_fsm_result_t out={0};track_mode4_result_t segment=track_mode4_process(image);
 int y,x,k,j,lo,hi,top,bottom,candidate,goal,endpoint=-1,endslope=4,lastrow=BOTTOM,rows=0,best=INF;
 /* Shared threshold adapter only; algorithm 3's path and servo are discarded. */
 for(y=0;y<120;++y)for(x=0;x<188;++x)mask[y][x]=image[y][x]>=segment.global_threshold?255:0;
 candidate=island12(&lo,&hi,&top,&bottom);track_path12_island_row=bottom;
 if(!run||dt>=TRACK_CAMERA_LOST_TIMEOUT_MS)track_path12_suspend(run?FSM_REASON_CAMERA:FSM_REASON_STOP);
 else {
  if(phase==FSM_WAIT)phase=FSM_NORMAL;
  if(!locked && candidate){if(candidate==pending)++pending_frames;else{pending=candidate;pending_frames=1;}
   phase=FSM_RING_CANDIDATE;if(pending_frames>=2){locked=candidate;phase=FSM_RING_ENTER;elapsed=0;}}
  else if(!locked){pending=0;pending_frames=0;phase=FSM_NORMAL;}
  if(locked){elapsed+=dt;
   if(candidate==locked){missing=0;if(bottom>=90)phase=FSM_RING_INSIDE;}
   else ++missing;
   if(phase==FSM_RING_INSIDE&&missing>=6)phase=FSM_RING_EXIT;
   if((phase==FSM_RING_EXIT&&missing>=18)||elapsed>=TRACK_FSM_ELEMENT_TIMEOUT_MS){locked=0;phase=FSM_NORMAL;elapsed=0;}
  }
 }
 /* Distance to a black boundary gives every path a clearance cost. */
 for(y=TOP;y<=BOTTOM;++y){int d=0;
  for(x=0;x<188;++x){d=mask[y][x]?d+1:0;clearance[y][x]=d;}
  d=0;for(x=187;x>=0;--x){d=mask[y][x]?d+1:0;if(d<clearance[y][x])clearance[y][x]=d;}
 }
 memset(track_path12_path_confidence,0,120);
 for(x=0;x<188;++x)for(k=0;k<9;++k)costs[BOTTOM&1][x][k]=
  clearance[BOTTOM][x]>=3?abs(x-(int)TRACK_STEER_REFERENCE_X)*18+abs(k-4)*5:INF;
 /* At the far end aim above the island, forcing the visible detour around it.
  * The search itself still requires white pixels at every row and transition. */
 goal=candidate && (!locked||candidate==locked)?(lo+hi)/2:94;
 if(locked && !candidate)goal=clip12(94+locked*24,15,172);
 for(y=BOTTOM-1;y>=TOP;--y){int reachable=0;
  for(x=0;x<188;++x)for(k=0;k<9;++k){int prev=x-(k-4),value=INF,bp=4;
   if(clearance[y][x]>=3 && prev>=0&&prev<188){
    int bridge=1,q;for(q=1;q<=2;++q){int u=(x*q+prev*(3-q))/3;if(!mask[y][u])bridge=0;}
    if(bridge)for(j=0;j<9;++j){int old=costs[(y+1)&1][prev][j];int c;
     if(old>=INF)continue;
     c=old+10*(k-j)*(k-j)+2*abs(k-4)+100/(clearance[y][x]+1);
     if(candidate&&(!locked||candidate==locked)&&y>=top&&y<=bottom){int guide=candidate<0?hi+12:lo-12;c+=abs(x-guide)/3;}
     if(c<value){value=c;bp=j;}
    }
   }
   costs[y&1][x][k]=value;parent[y][x][k]=bp;if(value<INF)reachable=1;
  }
  if(!reachable)break;
  lastrow=y;
  /* Keep the deepest feasible complete chain; this also stops above noise. */
  best=INF;
  for(x=0;x<188;++x)for(k=0;k<9;++k){int c=costs[y&1][x][k]+abs(x-goal)*(candidate||locked?14:3);
   if(c<best){best=c;endpoint=x;endslope=k;}}
 }
 if(endpoint>=0 && lastrow<=90){x=endpoint;k=endslope;
  for(y=lastrow;y<=BOTTOM;++y){int oldk=k;track_path12_path_x[y]=x;track_path12_path_confidence[y]=70;++rows;
   if(y<BOTTOM){k=parent[y][x][oldk];x-=oldk-4;if(x<0||x>=188){rows=0;break;}}
  }
 }
 y=clip12(locked||candidate?60:90,lastrow,BOTTOM);
 out.anchor_x=track_path12_path_x[BOTTOM];out.target_x=track_path12_path_x[y];
 out.anchor_confidence=track_path12_path_confidence[BOTTOM];out.target_confidence=track_path12_path_confidence[y];
 out.global_threshold=segment.global_threshold;out.valid_rows=rows;out.observed_rows=rows;
 out.ring_side=locked?locked:candidate;out.state=run?phase:FSM_WAIT;
 out.drive_allowed=run && dt<TRACK_CAMERA_LOST_TIMEOUT_MS && rows>=30 && lastrow<=y && out.target_confidence;
 if(run&&!out.drive_allowed)out.state=FSM_RECOVER;
 out.servo_cdeg=100*clip12((int)TRACK_SERVO_CENTER_DEG-((int)out.target_x-TRACK_STEER_REFERENCE_X)*(int)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL/100,TRACK_SERVO_RIGHT_DEG,TRACK_SERVO_LEFT_DEG);
 out.pwm_cap_percent=TRACK_WHEEL_PWM_MAX_PERCENT;
 track_path12_side=out.ring_side;track_path12_lookahead_row=y;track_path12_path_rows=rows;
 return out;
}
