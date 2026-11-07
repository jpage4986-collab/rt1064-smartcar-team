/* Imported ring vision pipeline from HammerTa/intelegent_car_race_program_17,
 * CODE/IMG_DEAL.c. Upstream author: HammerTa. Source is public, no repository
 * license declaration found. Retain attribution; no license grant is invented.
 * Port: RT1064 188x120 input, bounded indices, stable variance, output adapter.
 * No upstream GPIO, motor, garage, fork or ramp control is executed here. */
#include "track_ring13.h"
#include "track_mode4.h"
#include "tune_config.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
typedef enum{NO_JUGED,IN_JUGED}ALL_enum;
typedef enum{NO_FORK,IN_FORK}FORK_enum;
typedef enum{NO_T,IN_T,T_READY}T_CONNER_enum;
typedef enum{GET_OUT,FIRST,READY,GET_IN,NO_GARAGE}Garage_enmum;
typedef enum{ROUND_READY,GET_IN_READY,GET_IN_ROUND,IN_ROUND,GET_OUT_ROUND,OUTTING,OUT_WAY,OUT_ROUND,NO_ROUND}Roundabout_enmum;
typedef enum{ROUND_L,ROUND_R,NOROUND}Roundabout_position_enmum;
typedef enum{NO_RAMP,RAMP_READY,IN_RAMP}Ramp_enmum;
#define ROW 120
#define COL 188
typedef uint8_t uint8;
#define gpio_set(a,b) ((void)0)
#define FMQ 0
#define dj_end h13_dj_end
#define stop h13_stop
#define go h13_go
#define deal_flag h13_deal_flag
#define fork_flag h13_fork_flag
#define T_go_flag h13_T_go_flag
#define T_go_flag_pin h13_T_go_flag_pin
#define fork_turn h13_fork_turn
#define distance_flag h13_distance_flag
#define fork_times h13_fork_times
#define Boost_flag h13_Boost_flag
#define element_flag h13_element_flag
#define Fork_Flag h13_Fork_Flag
#define T_flag h13_T_flag
#define Garage_flag h13_Garage_flag
#define Roundabout_flag h13_Roundabout_flag
#define Roundabout_flag_position h13_Roundabout_flag_position
#define ramp_flag h13_ramp_flag
#define speed_flag h13_speed_flag
#define round_row_flag h13_round_row_flag
#define row_lim h13_row_lim
#define right_flag h13_right_flag
#define left_flag h13_left_flag
#define right_start h13_right_start
#define left_start h13_left_start
#define right_end_row h13_right_end_row
#define left_end_row h13_left_end_row
#define right_end_col h13_right_end_col
#define left_end_col h13_left_end_col
#define middle_start h13_middle_start
#define middle_end h13_middle_end
#define middleline h13_middleline
#define middleline0 h13_middleline0
#define middleline_l h13_middleline_l
#define middleline_r h13_middleline_r
#define SUM h13_SUM
#define wild1 h13_wild1
#define wild10 h13_wild10
#define RacingLine h13_RacingLine
#define left h13_left
#define right h13_right
#define forck_L h13_forck_L
#define forck_R h13_forck_R
#define T_line h13_T_line
#define forck_line_L h13_forck_line_L
#define forck_line_R h13_forck_line_R
#define Round_L h13_Round_L
#define Round_R h13_Round_R
#define left_apex h13_left_apex
#define right_apex h13_right_apex
#define right_ag h13_right_ag
#define left_ag h13_left_ag
#define Vector_r h13_Vector_r
#define Vector_l h13_Vector_l
#define right_line h13_right_line
#define left_line h13_left_line
#define max h13_max
#define min h13_min
#define line h13_line
#define Threshold h13_Threshold
#define IMG_DATA h13_IMG_DATA
#define mid_row_R h13_mid_row_R
#define mid_row_L h13_mid_row_L
#define mid_row h13_mid_row
#define mid_flag h13_mid_flag
#define L_lenth h13_L_lenth
#define R_lenth h13_R_lenth
#define Angle_end h13_Angle_end
#define Angle_End h13_Angle_End
#define ang_l h13_ang_l
#define ang_r h13_ang_r
#define ang h13_ang
#define L_S h13_L_S
#define R_S h13_R_S
#define K_left h13_K_left
#define K_right h13_K_right
#define CorssCol h13_CorssCol
#define weight h13_weight
#define TrackWild h13_TrackWild
#define InitData h13_InitData
#define Cross_col h13_Cross_col
#define CrossRow_L h13_CrossRow_L
#define CrossRow_R h13_CrossRow_R
#define S_jugde h13_S_jugde
#define LeftStartFind h13_LeftStartFind
#define RightStartFind h13_RightStartFind
#define fixpoint h13_fixpoint
#define left_jump h13_left_jump
#define right_jump h13_right_jump
#define Left_Apex h13_Left_Apex
#define Right_Apex h13_Right_Apex
#define Roundabout_Deal h13_Roundabout_Deal
#define Racing_Line h13_Racing_Line
#define RacingLine_L h13_RacingLine_L
#define RacingLine_R h13_RacingLine_R
#define Raceing_line_RoundIn_L_Blan_B h13_Raceing_line_RoundIn_L_Blan_B
#define Raceing_line_RoundIn_R_Blan_B h13_Raceing_line_RoundIn_R_Blan_B
#define Raceing_line_RoundOUT_L h13_Raceing_line_RoundOUT_L
#define Raceing_line_RoundOUT_R h13_Raceing_line_RoundOUT_R
int  dj_end;
unsigned char stop=1;
unsigned char go=0;
unsigned char deal_flag=0;
unsigned char fork_flag=0;
unsigned char T_go_flag[2]={1,0};
unsigned char T_go_flag_pin=0;
unsigned char fork_turn=2;
uint8 distance_flag=0;
uint8 fork_times=0;
uint8 Boost_flag=0;
ALL_enum element_flag=NO_JUGED;
FORK_enum Fork_Flag=NO_FORK;
T_CONNER_enum T_flag=NO_T;
Garage_enmum Garage_flag=GET_OUT;
Roundabout_enmum Roundabout_flag=NO_ROUND;
Roundabout_position_enmum Roundabout_flag_position=NOROUND;
Ramp_enmum ramp_flag=NO_RAMP;
int speed_flag=254;
int round_row_flag;
int row_lim=20;





int right_flag = 0;		
int left_flag = 0;
int right_start = 0;	
int left_start = 0;
int right_end_row = 0;		
int left_end_row = 0;
int right_end_col = 0;      
int left_end_col = 0;
int middle_start = 0;	
int middle_end = 0;
int middleline[240];
int middleline0[240];
int middleline_l[240];
int middleline_r[240];
int SUM=0;
int wild1,wild10;
struct DIV 
{ 
	int Row[240]; 
	int Col[240]; 
};

struct AG
{
	float k;
	float SIN;
};

struct APEX
{
	int Apex_Row;
	int Apex_Col;
	int Mark;
};

struct Vector
{
    int star[2];
    int end[2];
};

struct DIV RacingLine;
struct DIV left,right;
struct DIV forck_L,forck_R;
struct DIV T_line;
struct DIV forck_line_L,forck_line_R;
struct DIV Round_L,Round_R;
struct APEX left_apex,right_apex;
struct AG right_ag,left_ag;
struct Vector Vector_r,Vector_l;

int right_line[120] = {160};

int left_line[120] = {0};
int max = 0;
int min  = 0;
int line[120] = {0};
int Threshold;
unsigned char IMG_DATA[ROW][COL];
int mid_row_R[240];
int mid_row_L[240];
int mid_row[240];
int mid_flag=0;
int L_lenth,R_lenth;

float Angle_end;
float Angle_End[10]={0};
float ang_l,ang_r,ang;
float L_S,R_S;
float K_left,K_right;
float CorssCol=0;




#define WHITE_IMG    255
#define BLACK_IMG    0
#define RED_IMG      128
#define BLUE_IMG     254
#define GREEN_IMG    100

#define u8      unsigned char   
#define u16     unsigned short











int weight[120]={
2,2,2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,2,2,
4,4,4,4,4,4,4,4,4,4,
8,8,8,8,8,8,8,8,8,8,
10,10,10,10,10,10,10,10,10,10,
12,12,12,12,12,12,12,12,12,12,
14,14,14,14,14,14,14,14,14,14,
16,16,16,16,16,16,16,16,16,16,
10,10,10,10,10,10,10,10,10,10,
8,8,8,8,8,8,8,8,8,8,
4,4,4,4,4,4,4,4,4,4,
3,3,3,3,3,3,3,3,3,3,
};


int TrackWild[120]={
        2, 3, 5, 6, 7, 8,10,11,12,14,15,16,17,19,20,21,22,24,25,
        26,27,29,30,31,33,34,35,36,38,39,40,41,43,44,45,47,48,49,
        50,52,53,54,55,57,58,59,61,62,63,64,66,67,68,69,71,72,73,
        75,76,77,78,80,81,82,83,85,86,87,89,90,91,92,94,95,96,97,
        99,100,101,103,104,105,106,108,109,110,111,113,114,115,116,118,119,120,122,
        123,124,125,127,128,129,130,132,133,134,136,137,138,139,141,142,143,144,146,
        147,148,150,151,152,153};

















static int h13_index(int n){return n<0?0:(n>239?239:n);}
static int h13_row_index(int n){return n<0?0:(n>119?119:n);}
static float h13_nonzero(float n){return fabsf(n)<0.001f?1.0f:n;}
static unsigned char *h13_pixel(int y,int x){static unsigned char sink;sink=0;
 return y>=0&&y<120&&x>=0&&x<188?&IMG_DATA[y][x]:&sink;}
void InitData();
float Cross_col();
float CrossRow_L(int star);
float CrossRow_R(int star);
float S_jugde(int row[],int col[],int N);
void LeftStartFind();
void RightStartFind();
void fixpoint();
void left_jump();
void right_jump();
void Left_Apex();
void Right_Apex();
void Roundabout_Deal();
void Racing_Line();
void RacingLine_L(int k);
void RacingLine_R(int k);
void Raceing_line_RoundIn_L_Blan_B();
void Raceing_line_RoundIn_R_Blan_B();
void Raceing_line_RoundOUT_L();
void Raceing_line_RoundOUT_R();

 void InitData() 
{ 
        int pin; 
        for(pin=0;pin<240;pin++) 
	{ 
		left.Row[h13_index(pin)]=254;
		left.Col[h13_index(pin)]=254; 
		right.Row[h13_index(pin)]=254;
		right.Col[h13_index(pin)]=254;
		if(Fork_Flag==NO_FORK)
		{
		    forck_L.Row[h13_index(pin)]=254;
		    forck_L.Col[h13_index(pin)]=254;
		    forck_R.Row[h13_index(pin)]=254;
		    forck_R.Col[h13_index(pin)]=254;
		}
		if(Roundabout_flag==NO_ROUND)
		{
		    Round_L.Row[h13_index(pin)]=254;
		    Round_L.Col[h13_index(pin)]=254;
		    Round_R.Row[h13_index(pin)]=254;
		    Round_R.Col[h13_index(pin)]=254;
		}
		T_line.Row[h13_index(pin)]=254;
		T_line.Col[h13_index(pin)]=254;
        RacingLine.Row[h13_index(pin)]=254;
        RacingLine.Col[h13_index(pin)]=254;
        middleline[pin]=254;
        middleline_l[pin]=254;
        middleline_r[pin]=254;
        mid_row_R[pin]=254;
        mid_row_L[pin]=254;
        mid_row[pin]=254;
	}
    gpio_set(FMQ,0);
    speed_flag=254;
    L_S=0;
    R_S=0;
    left_apex.Mark=254;
	right_apex.Mark=254;
	Vector_r.star[0]=254;
	Vector_r.star[1]=254;
	Vector_r.end[0]=254;
	Vector_r.end[1]=254;
    Vector_l.star[0]=254;
    Vector_l.star[1]=254;
    Vector_l.end[0]=254;
    Vector_l.end[1]=254;
    L_lenth=0;
    R_lenth=0;
    fork_flag=0;
    ang_l=0;
    ang_r=0;
    ang=0;
}

 
 
 
 
 
 
 float Cross_col()
 {
     float W_percent=0;
     int i,j;
     int White_point=0;
     int Black_point=0;
     int point=600;
     int col[5]={84,90,94,98,102};
     for(i=0;i<5;i++)
     {
         for(j=119;j>0;j--)
         {
             if((*h13_pixel(j,col[i]))==WHITE_IMG)
                 White_point++;
             else
             {
                 Black_point++;
                 if(Black_point>4)
                 {
                     Black_point=0;
                     break;
                 }
             }
         }
     }
     W_percent=1.0*White_point/point;
     return W_percent;
 }
 float CrossRow_L(int star)
 {
      float W_percent=0;
      int i,j;
      int White_point=0;
      int Black_point=0;
      int point=470;
      int row[5];
      if(star>=100) return -1;
      for(i=0;i<5;i++) row[i]=star+5*i;
      for(i=0;i<5;i++)
      {
          for(j=94;j>5;j--)
          {
              if((*h13_pixel(row[i],j))==WHITE_IMG)
                  White_point++;
              else if((*h13_pixel(row[i],j))==BLACK_IMG)
              {
                  Black_point++;
                  if(Black_point>4)
                  {
                      Black_point=0;
                      break;
                  }
              }
          }
      }
      W_percent=(float)White_point/point;
      return W_percent;
 }

 float CrossRow_R(int star)
 {
      float W_percent=0;
      int i,j;
      int White_point=0;
      int Black_point=0;
      int point=470;
      int row[5];
      if(star>=100) return -1;
      for(i=0;i<5;i++) row[i]=star+5*i;
      for(i=0;i<5;i++)
      {
          for(j=94;j<183;j++)
          {
              if((*h13_pixel(row[i],j))==WHITE_IMG)
                  White_point++;
              else if((*h13_pixel(row[i],j))==BLACK_IMG)
              {
                  Black_point++;
                  if(Black_point>4)
                  {
                      Black_point=0;
                      break;
                  }
              }
          }
      }
      W_percent=(float)White_point/point;
      return W_percent;
 }
float S_jugde(int row[],int col[],int N){
   /* Stable x(y) residual. Upstream y(x) divided by zero on vertical edges. */
   float sy=0,sx=0,syy=0,syx=0,err=0,k,b;int i,n=0;
   for(i=0;i<N && i<239 && row[i]!=254 && col[i]!=254;++i){sy+=row[i];sx+=col[i];syy+=row[i]*row[i];syx+=row[i]*col[i];++n;}
   if(n<6)return 254;
   k=(n*syy-sy*sy);if(fabsf(k)<1)return 254;
   k=(n*syx-sy*sx)/k;b=(sx-k*sy)/n;
   for(i=0;i<n;++i){float d=col[i]-(k*row[i]+b);err+=d*d;}
   return sqrtf(err/(n-2))/sqrtf(n);
  }








void LeftStartFind() 
{ 
  int row,col;
  for(row=ROW-5;row>40 && left.Row[h13_index(0)]==254;row--)
  {
    for(col = 5;col<COL - 55 && left.Col[h13_index(0)] == 254;col++)
    {
      if((*h13_pixel(row,col))==BLACK_IMG && (*h13_pixel(row,col+1))==BLACK_IMG)
      {
        if((*h13_pixel(row,col+2))==WHITE_IMG && (*h13_pixel(row,col+3))==WHITE_IMG)
        {
           left.Row[h13_index(0)]=row;
           left.Col[h13_index(0)]=col+1;
           left_flag=1;
        }
      }
    }
  }
  left_start=left.Row[h13_index(0)];
}
  






void RightStartFind() 
{
  int row,col;
  for(row=ROW-5;row > 40 && right.Row[h13_index(0)] == 254;row--)
  {
    for(col=COL - 5;col > 55 && right.Col[h13_index(0)] == 254;col--)
    {
      if((*h13_pixel(row,col))==BLACK_IMG && (*h13_pixel(row,col-1))==BLACK_IMG)
      {
        if((*h13_pixel(row,col-2))==WHITE_IMG && (*h13_pixel(row,col-3))==WHITE_IMG)
        {
        right.Row[h13_index(0)]=row;
        right.Col[h13_index(0)]=col-1;
        right_flag=1;
        }
      }
    }
  }
  right_start=right.Row[h13_index(0)];
}







void fixpoint()
{
    if(left.Col[h13_index(0)] == 254 && left.Row[h13_index(0)]==254)
	{
		return;
	}
	if(right.Col[h13_index(0)] == 254 && right.Row[h13_index(0)] == 254)
	{
		return;
	}
	float k = 0;
	int UpPointCol = 254, UpPointRow = 254;
	int DownPointCol = 254,DownPointRow = 254;
	int row, col, pin, BP;
	BP = 0;
	if(left.Row[h13_index(0)] < right.Row[h13_index(0)])
	{
		UpPointCol = left.Col[h13_index(0)];
		UpPointRow = left.Row[h13_index(0)];
		DownPointCol = right.Col[h13_index(0)];
		DownPointRow = right.Row[h13_index(0)];
	}
	else
	{
		UpPointCol = right.Col[h13_index(0)];
		UpPointRow = right.Row[h13_index(0)];
		DownPointCol = left.Col[h13_index(0)];
		DownPointRow = left.Row[h13_index(0)];
	}
	if(UpPointRow != DownPointRow)
	{
		k = (float)(UpPointCol - DownPointCol) / (UpPointRow - DownPointRow);
	}
	for(pin=0, row = DownPointRow;row >= UpPointRow;pin++,row--)
	{
		col = (int)(DownPointCol - k * pin);
		if(col > -1 && col < COL && row >-1 && row < ROW)
		{
			if((*h13_pixel(row,col)) == BLACK_IMG)
			{
				BP++;
			}
		}
	}
	if(BP > 8)
	{
		if(left.Row[h13_index(0)] < right.Row[h13_index(0)])
		{
			left.Col[h13_index(0)]=254;
			left.Row[h13_index(0)]=254;
            left_flag=0;
		}
		else if(left.Row[h13_index(0)] > right.Row[h13_index(0)])
		{
			right.Col[h13_index(0)]=254;
			right.Row[h13_index(0)]=254;
            right_flag=0;
		}
	}
}




void left_jump()
{
	int pin;
	int row, col,find;




    int colmin, colmax;
    if(left.Row[h13_index(0)] == 254)
	{
		return;
	}
    left_flag=1;
    pin=1;
    for(row=left.Row[h13_index(0)] - pin;pin<240;pin++)
	{
		find = 0;
		row = left.Row[h13_index(0)] - pin;
		if(row<=row_lim)break;
		colmin = left.Col[h13_index(pin - 1)] - 10;
		colmax = left.Col[h13_index(pin - 1)] + 10;
		for(col = colmin;col <= colmax;col++)
		{
		    if(col<5) col=5;
			if(col < COL - 5)
			{
				if((*h13_pixel(row,col)) == BLACK_IMG && (*h13_pixel(row,col + 1)) == BLACK_IMG)
				{
					if((*h13_pixel(row,col + 2)) == WHITE_IMG && (*h13_pixel(row,col + 3)) == WHITE_IMG)
					{
						left.Row[h13_index(pin)] = row;
						left.Col[h13_index(pin)] = col + 1;
						(*h13_pixel(row,col + 1)) = RED_IMG;
						find = 1;
						L_lenth++;
						left_end_row=row;
						left_end_col=col;
						break;
					}
				}
			}
		}
		if(find == 0)
		{
			break;
		}
    }
	if(L_lenth<=5)
	{
	    left_flag=0;
	    left.Row[h13_index(0)]=254;
	    left.Col[h13_index(0)]=254;
	}

}
void right_jump()
{
	int pin;
	int row, col,find;


    int colmin, colmax;


    if(right.Row[h13_index(0)] == 254)
	{
		return;
	}
    right_flag=1;
    for(pin=1,row=right.Row[h13_index(0)] - pin;pin<240;pin++)
	{
		find = 0;
		row = right.Row[h13_index(0)] - pin;
		if(row<=row_lim)break;
		colmin = right.Col[h13_index(pin - 1)] - 10;
		colmax = right.Col[h13_index(pin - 1)] + 10;
		for(col = colmax;col >= colmin;col--)
		{
		    if(col > COL - 5) col=COL - 5;
			if(col > 5)
			{
				if((*h13_pixel(row,col)) == BLACK_IMG && (*h13_pixel(row,col - 1)) == BLACK_IMG)
				{
					if((*h13_pixel(row,col - 2)) == WHITE_IMG && (*h13_pixel(row,col - 3)) == WHITE_IMG)
					{
						right.Row[h13_index(pin)] = row;
						right.Col[h13_index(pin)] = col - 1;
						(*h13_pixel(row,col - 1)) = BLUE_IMG;
						find = 1;
						R_lenth++;
						right_end_row=row;
						right_end_col=col;
						break;
					}
				}
			}
		}
		if(find == 0)
		{
			break;
		}
	}
	if(R_lenth<=5)
	{
	    right_flag=0;
	    right.Row[h13_index(0)]=254;
	    right.Col[h13_index(0)]=254;
	}
  

}

void Left_Apex()
{
    if(left.Row[h13_index(0)]==254&&left.Col[h13_index(0)]==254)
    {
        return;
    }
    int pin,col_max=0;
    int row_flag=254,col_flag=254;
    for(pin=0;pin<240;pin++)
    {
        if(left.Row[h13_index(pin)]==254)
        {
            break;
        }
        if(col_max <= left.Col[h13_index(pin)])
        {
            col_max=left.Col[h13_index(pin)];
            row_flag=left.Row[h13_index(pin)];
            col_flag=left.Col[h13_index(pin)];
            left_apex.Mark=pin;
        }
    }
    left_apex.Apex_Row=row_flag;
    left_apex.Apex_Col=col_flag;
	L_S=S_jugde(left.Row,left.Col,240);
}

void Right_Apex()
{
    if(right.Row[h13_index(0)]==254&&right.Col[h13_index(0)]==254)
    {
        return;
    }
    int pin;
    int row_flag=254,col_flag=254;
    int col_min=188;
    for(pin=0;pin<240;pin++)
    {
        if(right.Row[h13_index(pin)]==254)
        {
            break;
        }
        if(right.Col[h13_index(pin)] <= col_min)
        {
            col_min=right.Col[h13_index(pin)];
            row_flag=right.Row[h13_index(pin)];
            col_flag=right.Col[h13_index(pin)];
            right_apex.Mark=pin;
        }
        
    }
    right_apex.Apex_Row=row_flag;
    right_apex.Apex_Col=col_flag;
	R_S=S_jugde(right.Row,right.Col,240);
}







void Roundabout_Deal()
{
    int row,col,i,find;
    int col_min,col_max;
    float cross_row_L,cross_row_R;
    float cross_col,crossrow_juged;
    float round_l_s,round_r_s;
    float k;
    cross_col=CorssCol;
    find=0;
    switch(Roundabout_flag)
    {
        case NO_ROUND:
            cross_row_L=CrossRow_L(65);
            cross_row_R=CrossRow_R(65);
            if(element_flag==IN_JUGED && Roundabout_flag==NO_ROUND)
            {
                Roundabout_flag_position=NOROUND;
                Roundabout_flag=NO_ROUND;
                return;
            }
            if(cross_row_L>0.9 && cross_row_R<0.8 && R_S<0.4)
            {
                Roundabout_flag_position=ROUND_L;
                Roundabout_flag=ROUND_READY;
                element_flag=NO_JUGED;
            }
            else if(cross_row_L<0.8 && cross_row_R>0.9 && L_S<0.4)
            {
                Roundabout_flag_position=ROUND_R;
                Roundabout_flag=ROUND_READY;
                element_flag=NO_JUGED;
            }
            if(right_apex.Apex_Col<78 || left_apex.Apex_Col>110)
            {
                Roundabout_flag_position=NOROUND;
                Roundabout_flag=NO_ROUND;
                return;
            }
            break;
        case ROUND_READY:
            round_l_s=L_S;
            round_r_s=R_S;
            if(right_apex.Apex_Col<78 || left_apex.Apex_Col>110)
            {
                Roundabout_flag_position=NOROUND;
                Roundabout_flag=NO_ROUND;
                return;
            }
            if(element_flag==IN_JUGED)
            {
                Roundabout_flag_position=NOROUND;
                Roundabout_flag=NO_ROUND;
                return;
            }
            if(CorssCol<0.75)
            {
                Roundabout_flag=NO_ROUND;
                element_flag=NO_JUGED;
                return;
            }
            switch (Roundabout_flag_position)
            {
                case ROUND_L:
                    if(right_apex.Apex_Col<78 || R_lenth<80 || L_lenth<20)
                    {
                        Roundabout_flag_position=NOROUND;
                        Roundabout_flag=NO_ROUND;
                        return;
                    }
                    if(round_l_s>0.9 && round_r_s<0.4)
                    {
                        Roundabout_flag=GET_IN_READY;
                        element_flag=NO_JUGED;
                    }
                    else if(round_r_s>0.5)
                    {
                        Roundabout_flag=NO_ROUND;
                        element_flag=NO_JUGED;
                    }
                    break;
                case ROUND_R:
                    if(left_apex.Apex_Col>110 || L_lenth<80 || R_lenth<20)
                    {
                        Roundabout_flag_position=NOROUND;
                        Roundabout_flag=NO_ROUND;
                        return;
                    }
                    if(round_l_s<0.4 && round_r_s>0.9)
                    {
                        Roundabout_flag=GET_IN_READY;
                        element_flag=NO_JUGED;
                    }
                    else if(round_l_s>0.5)
                    {
                        Roundabout_flag=NO_ROUND;
                        element_flag=NO_JUGED;
                    }
                    break;
                case NOROUND:
                    break;
            }
            break;
        case GET_IN_READY:
            round_l_s=L_S;
            round_r_s=R_S;
            cross_row_L=CrossRow_L(65);
            cross_row_R=CrossRow_R(65);
            switch (Roundabout_flag_position)
            {
                case ROUND_L:
                    col_min=left_apex.Apex_Col-10;
                    col_max=COL/2;
                    if(round_r_s>0.5)
                    {
                        Roundabout_flag=NO_ROUND;
                        element_flag=NO_JUGED;
                    }
                    for(row=50;row>40;row--)
                    {
                        for(col=col_min;col<col_max;col++)
                        {
                            if((*h13_pixel(row,col))==WHITE_IMG && (*h13_pixel(row,col+1))==BLACK_IMG)
                            {
                                Round_L.Row[h13_index(0)]=row;
                                Round_L.Col[h13_index(0)]=col+1;
                                crossrow_juged=CrossRow_L(row);
                                if(crossrow_juged>0.8 && cross_col>0.75)
                                {
                                    find=1;
                                    gpio_set(FMQ,1);
                                    Roundabout_flag=GET_IN_ROUND;
                                    element_flag=IN_JUGED;
                                }
                            }
                        }
                    }
                    break;
                case ROUND_R:
                    col_min=COL/2;
                    col_max=right_apex.Apex_Col+10;
                    if(round_l_s>0.5)
                    {
                        Roundabout_flag=NO_ROUND;
                        element_flag=NO_JUGED;
                    }
                    for(row=50;row>40;row--)
                    {
                        for(col=col_max;col>col_min;col--)
                        {
                            if((*h13_pixel(row,col))==WHITE_IMG && (*h13_pixel(row,col-1))==BLACK_IMG)
                            {
                                Round_R.Row[h13_index(0)]=row;
                                Round_R.Col[h13_index(0)]=col-1;
                                crossrow_juged=CrossRow_R(row);
                                if(crossrow_juged>0.75 && cross_col>0.75)
                                {
                                    find=1;
                                    gpio_set(FMQ,1);
                                    Roundabout_flag=GET_IN_ROUND;
                                    element_flag=IN_JUGED;
                                }
                            }
                        }
                    }
                    break;
                case NOROUND:
                    break;
            }
            break;
        case GET_IN_ROUND:
            gpio_set(FMQ,1);
            row_lim=25;
            switch (Roundabout_flag_position)
            {
                case ROUND_L:
                    if(right_apex.Apex_Col<47) Roundabout_flag=IN_ROUND;
                    break;
                case ROUND_R:
                    if(left_apex.Apex_Col>141) Roundabout_flag=IN_ROUND;
                    break;
                case NOROUND:
                    break;
            }
            break;
        case IN_ROUND:
            row_lim=15;
            round_row_flag=254;
            switch (Roundabout_flag_position)
            {
                case ROUND_L:
                    if(right_apex.Apex_Col>94)
                    {
                        Roundabout_flag=GET_OUT_ROUND;
                        element_flag=IN_JUGED;
                        k=1.0*(right_apex.Apex_Col-right.Col[h13_index(0)])/(right_apex.Apex_Row-right.Row[h13_index(0)]);
                        for(i=0;i<120;i++)
                        {
                            if(i<right_apex.Mark)
                            {
                                Round_L.Row[h13_index(i)]=right.Row[h13_index(i)];
                                Round_L.Col[h13_index(i)]=right.Col[h13_index(i)];
                            }
                            else
                            {
                                Round_L.Row[h13_index(i)]=Round_L.Row[h13_index(i-1)]-1;
                                Round_L.Col[h13_index(i)]=k*(Round_L.Row[h13_index(i)]-right_apex.Apex_Row)+right_apex.Apex_Col;
                            }
                        }
                    }
                    break;
                case ROUND_R:
                    if(left_apex.Apex_Col<94)
                    {
                        Roundabout_flag=GET_OUT_ROUND;
                        element_flag=IN_JUGED;
                        k=1.0*(left_apex.Apex_Col-left.Col[h13_index(0)])/(left_apex.Apex_Row-left.Row[h13_index(0)]);
                        for(i=0;i<120;i++)
                        {
                            if(i<left_apex.Mark)
                            {
                                Round_R.Row[h13_index(i)]=left.Row[h13_index(i)];
                                Round_R.Col[h13_index(i)]=left.Col[h13_index(i)];
                            }
                            else
                            {
                                Round_R.Row[h13_index(i)]=Round_R.Row[h13_index(i-1)]-1;
                                Round_R.Col[h13_index(i)]=k*(Round_R.Row[h13_index(i)]-left_apex.Apex_Row)+left_apex.Apex_Col;
                            }
                        }
                    }
                    break;
                case NOROUND:
                    break;
            }
            break;
        case GET_OUT_ROUND:
            switch (Roundabout_flag_position)
            {
                case ROUND_L:
                    cross_row_R=CrossRow_R(75);
                    if(cross_row_R>0.9)
                    {
                        Roundabout_flag=OUTTING;
                        element_flag=IN_JUGED;
                    }
                    break;
                case ROUND_R:
                    cross_row_L=CrossRow_L(75);
                    if(cross_row_L>0.9)
                    {
                        Roundabout_flag=OUTTING;
                        element_flag=IN_JUGED;
                    }
                    break;
                case NOROUND:
                    break;
            }
            break;
         case OUTTING:
             switch (Roundabout_flag_position)
             {
                 case ROUND_L:
                     cross_row_R=CrossRow_R(75);
                     if(cross_row_R<0.8)
                     {
                         Roundabout_flag=OUT_WAY;
                         element_flag=IN_JUGED;
                     }
                     break;
                 case ROUND_R:
                     cross_row_L=CrossRow_L(75);
                     if(cross_row_L<0.8)
                     {
                         Roundabout_flag=OUT_WAY;
                         element_flag=IN_JUGED;
                     }
                     break;
                 case NOROUND:
                     break;
             }
             break;
         case OUT_WAY:
             switch (Roundabout_flag_position)
             {
                 case ROUND_L:
                     cross_row_L=CrossRow_L(75);
                     cross_row_R=CrossRow_R(75);
                     if(cross_row_L>0.9 && cross_row_R<0.6)
                     {
                         Roundabout_flag=OUT_ROUND;
                         element_flag=IN_JUGED;
                         }
                     break;
                 case ROUND_R:
                     cross_row_L=CrossRow_L(75);
                     cross_row_R=CrossRow_R(75);
                     if(cross_row_R>0.9 && cross_row_L<0.6)
                     {
                         Roundabout_flag=OUT_ROUND;
                         element_flag=IN_JUGED;
                     }
                     break;
                 case NOROUND:
                     break;
             }
             break;
         case OUT_ROUND:
             switch (Roundabout_flag_position)
             {
                 case ROUND_L:
                     cross_row_L=CrossRow_L(75);
                     if(cross_row_L<0.7)
                     {
                         Roundabout_flag=NO_ROUND;
                         element_flag=NO_JUGED;
                     }
                     break;
                 case ROUND_R:
                     cross_row_R=CrossRow_R(75);
                     if(cross_row_R<0.7)
                     {
                         Roundabout_flag=NO_ROUND;
                         element_flag=NO_JUGED;
                     }
                     break;
                 case NOROUND:
                     break;
             }
             break;
    }
}








void Racing_Line()
{
        int pin,i;
        int row,col;
        int flag;
        int mark;
        int l_flag,r_flag;
        l_flag=0;
        r_flag=0;
        i=0;
        if(left_flag==0&&right_flag==0)
        {
          return;
        }
        if(left_flag==0)
        {
            mark=right_apex.Mark+(115-right.Row[h13_index(5)]);
            row=right.Row[h13_index(5)];
            i=5;
            for(pin=0;pin<mark;pin++)
            {
                flag=119-pin;
                if(flag>right.Row[h13_index(5)])
                {
                  middleline[pin]=right.Col[h13_index(5)]-TrackWild[h13_row_index(row)]/2;
                  col=middleline[pin];
                  mid_row[pin]=flag;
                  (*h13_pixel(flag,col))=GREEN_IMG;
                }
                else
                {
                  row=right.Row[h13_index(i)];
                  col=right.Col[h13_index(i)]-TrackWild[h13_row_index(row)]/2;
                  middleline[pin]=col;
                  mid_row[pin]=row;
                  i++;
                  if(row>5&&row<ROW-5)
                  {
                    if(col>5&&col<COL-5)
                    {
                        (*h13_pixel(row,col))=GREEN_IMG;
                    }
                  }
                }
            }
            return;
        }
        else if(right_flag==0)
        {
          mark=left_apex.Mark+(115-left.Row[h13_index(5)]);
          row=left.Row[h13_index(5)];
          i=5;
          for(pin=0;pin<mark;pin++)
            {
              flag=119-pin;
              if(flag>left.Row[h13_index(5)])
              {
                middleline[pin]=left.Col[h13_index(5)]+TrackWild[h13_row_index(row)]/2;
                  col=middleline[pin];
                  (*h13_pixel(flag,col))=GREEN_IMG;
                  mid_row[pin]=flag;
              }
              else
              {
                row=left.Row[h13_index(i)];
                col=left.Col[h13_index(i)]+TrackWild[h13_row_index(row)]/2;
                middleline[pin]=col;
                mid_row[pin]=row;
                i++;
                if(row>5&&row<ROW-5)
                {
                  if(col>5&&col<COL-5)
                  {
                      (*h13_pixel(row,col))=GREEN_IMG;
                  }
                }
              }
            }
            return;
        }
        if(left_apex.Mark>=right_apex.Mark)
        {
            mark=left_apex.Mark+(115-left.Row[h13_index(5)]);
            row=left.Row[h13_index(5)];
            i=5;
            for(pin=0;pin<mark;pin++)
            {
              flag=119-pin;
              if(flag>left.Row[h13_index(5)])
              {
                middleline[pin]=left.Col[h13_index(5)]+TrackWild[h13_row_index(row)]/2;
                  col=middleline[pin];
                  mid_row[pin]=flag;
                  (*h13_pixel(flag,col))=GREEN_IMG;
              }
              else
              {
                row=left.Row[h13_index(i)];
                col=left.Col[h13_index(i)]+TrackWild[h13_row_index(row)]/2;
                middleline[pin]=col;
                mid_row[pin]=row;
                i++;
                if(row>5&&row<ROW-5)
                {
                  if(col>5&&col<COL-5)
                  {
                      (*h13_pixel(row,col))=GREEN_IMG;
                  }
                }
              }
              l_flag++;
            }
            return;
        }
        else if(left_apex.Mark<right_apex.Mark)
        {
            mark=right_apex.Mark+(115-right.Row[h13_index(5)]);
            row=right.Row[h13_index(5)];
            i=5;
            for(pin=0;pin<mark;pin++)
            {
              flag=119-pin;
              if(flag>right.Row[h13_index(5)])
              {
                middleline[pin]=right.Col[h13_index(5)]-TrackWild[h13_row_index(row)]/2;
                  col=middleline[pin];
                  mid_row[pin]=flag;
                  (*h13_pixel(flag,col))=GREEN_IMG;
              }
              else
              {
                row=right.Row[h13_index(i)];
                col=right.Col[h13_index(i)]-TrackWild[h13_row_index(row)]/2;
                middleline[pin]=col;
                i++;
                mid_row[pin]=row;
                if(row>5&&row<ROW-5)
                {
                  if(col>5&&col<COL-5)
                  {
                      (*h13_pixel(row,col))=GREEN_IMG;
                  }
                }
              }
              r_flag++;
            }
            return;
        }
}







void RacingLine_L(int k)
{
    int pin,i;
    int row,col;
    int flag;
    int mark;
    int l_flag,r_flag;
    l_flag=0;
    r_flag=0;
    i=0;
    mark=L_lenth+(115-left.Row[h13_index(0)]);
    if(mark>240) mark=240;
    row=left.Row[h13_index(0)];
    for(pin=0;pin<240;pin++)
    {
        flag=119-pin;
        if(left.Row[h13_index(i)]==254) break;
        if(flag>left.Row[h13_index(0)])
        {
            middleline[pin]=left.Col[h13_index(0)]+TrackWild[h13_row_index(row)]/k;
            col=middleline[pin];
            (*h13_pixel(flag,col))=GREEN_IMG;
            mid_row[pin]=flag;
        }
        else
        {
            row=left.Row[h13_index(i)];
            col=left.Col[h13_index(i)]+TrackWild[h13_row_index(row)]/k;
            middleline[pin]=col;
            mid_row[pin]=row;
            i++;
            if(row>5&&row<ROW-5)
            {
                if(col>5&&col<COL-5)
                {
                    (*h13_pixel(row,col))=GREEN_IMG;
                }
            }
        }

    }
    return;
}







void RacingLine_R(int k)
{
    int pin,i;
    int row,col;
    int flag;
    int mark;
    int l_flag,r_flag;
    l_flag=0;
    r_flag=0;
    i=0;
    if(left_flag==0&&right_flag==0)
    {
        return;
    }
    mark=R_lenth+(115-right.Row[h13_index(0)]);
    if(mark>240) mark=240;
    row=right.Row[h13_index(0)];
    for(pin=0;pin<mark;pin++)
    {
        flag=119-pin;
        if(right.Row[h13_index(i)]==254) break;
        if(flag>right.Row[h13_index(0)])
        {
            middleline[pin]=right.Col[h13_index(0)]-TrackWild[h13_row_index(row)]/k;
            col=middleline[pin];
            mid_row[pin]=flag;
            (*h13_pixel(flag,col))=GREEN_IMG;
        }
        else
        {
            row=right.Row[h13_index(i)];
            col=right.Col[h13_index(i)]-TrackWild[h13_row_index(row)]/k;
            middleline[pin]=col;
            mid_row[pin]=row;
            i++;
            if(row>5&&row<ROW-5)
            {
                if(col>5&&col<COL-5)
                {
                    (*h13_pixel(row,col))=GREEN_IMG;
                }
            }
        }
    }
    return;
}

void Raceing_line_RoundIn_L_Blan_B()
{
    int row,col,pin,find,i;
    int col_min,col_max,flag;
    int star_row,star_col,star_flag;
    int max_row,max_col;
    float k;
    find=0;max_row=0;max_col=0;
    i=1;
    col_min=left_apex.Apex_Col+1;
    col_max=right_apex.Apex_Col;
    for(col=col_max;col>col_min;col--)
    {
        if(Roundabout_flag==IN_ROUND)
        {
            max_row=right.Row[h13_index(0)];
            max_col=right.Col[h13_index(0)];
            break;
        }
        if(col<5) break;
        if(left_apex.Apex_Row<right_apex.Apex_Row+20 && left.Row[h13_index(0)]<100)
        {
            max_row=left.Row[h13_index(0)];
            max_col=left.Col[h13_index(0)];
            break;
        }
        for(row=left_end_row-5;row>5;row--)
        {
            if((*h13_pixel(row,col))==WHITE_IMG && (*h13_pixel(row-1,col))==WHITE_IMG)
            {
                if((*h13_pixel(row-2,col))==BLACK_IMG && (*h13_pixel(row-3,col))==BLACK_IMG)
                {
                    if(row>max_row)
                    {
                        max_row=row;
                        max_col=col;
                    }
                    break;
                }
            }
        }
    }
    Round_L.Row[h13_index(0)]=max_row;
    Round_L.Col[h13_index(0)]=max_col;
    star_row=115;
    star_col=180;
    for(pin=0;pin<240;pin++)
    {
        if(Roundabout_flag==IN_ROUND)
        {
            star_row=115;
            star_col=180;
            break;
        }
        if(right.Row[h13_index(pin)]==254) break;
        if(left_apex.Apex_Row<right_apex.Apex_Row+20 && left.Row[h13_index(0)]<100)
        {
            star_row=115;
            star_col=180;
            break;
        }
        if(right.Row[h13_index(pin)]<=left_apex.Apex_Row)
        {
            star_row=right.Row[h13_index(pin)];
            star_col=right.Col[h13_index(pin)];
            star_flag=pin;
            break;
        }
    }
    (*h13_pixel(star_row,star_col))=GREEN_IMG;
    k=(star_col*1.0-Round_L.Col[h13_index(0)])/(star_row*1.0-Round_L.Row[h13_index(0)]);
    row=star_row;
    col=star_col;
    for(pin=1;pin<240;pin++)
    {
        if(star_row-pin>=Round_L.Row[h13_index(0)])
        {
            Round_L.Row[h13_index(pin)]=star_row-pin;
            Round_L.Col[h13_index(pin)]=star_col-(int)pin*k;
            (*h13_pixel(Round_L.Row[h13_index(pin)],Round_L.Col[h13_index(pin)]))=BLUE_IMG;
        }
        else if(Roundabout_flag==IN_ROUND)
        {
            if(right.Row[h13_index(pin)]==254) break;
            Round_L.Row[h13_index(pin)]=right.Row[h13_index(pin)];
            Round_L.Col[h13_index(pin)]=right.Col[h13_index(pin)];
        }
        else
        {
            find = 0;
            row = star_row - pin;
            if(row<=15)break;
            col_min = Round_L.Col[h13_index(pin - 1)] - 10;
            col_max = Round_L.Col[h13_index(pin - 1)] + 10;
            for(col = col_max;col >= col_min;col--)
            {
                if(col > COL - 5) col=COL - 5;
                if(col > 5)
                {
                    if((*h13_pixel(row,col)) == BLACK_IMG && (*h13_pixel(row,col - 1)) == BLACK_IMG)
                    {
                        if((*h13_pixel(row,col - 2)) == WHITE_IMG && (*h13_pixel(row,col - 3)) == WHITE_IMG)
                        {
                            Round_L.Row[h13_index(pin)] = row;
                            Round_L.Col[h13_index(pin)] = col - 1;
                            if(pin>1) (*h13_pixel(Round_L.Row[h13_index(pin)],Round_L.Col[h13_index(pin)]))=BLUE_IMG;
                            find = 1;
                            break;
                        }
                    }
                }
            }
            if(find == 0)
            {
                break;
            }
        }
    }
    row=Round_L.Row[h13_index(1)];
    i=1;
    for(pin=0;pin<240;pin++)
    {
        flag=119-pin;
        if(Round_L.Row[h13_index(i)]==254) break;
        if(flag>Round_L.Row[h13_index(1)])
        {
            middleline[pin]=Round_L.Col[h13_index(1)]-TrackWild[h13_row_index(row)]/2;
            col=middleline[pin];
            mid_row[pin]=flag;
            (*h13_pixel(flag,col))=GREEN_IMG;
        }
        else
        {
            row=Round_L.Row[h13_index(i)];
            col=Round_L.Col[h13_index(i)]-TrackWild[h13_row_index(row)]/2;
            middleline[pin]=col;
            mid_row[pin]=row;
            i++;
            if(row>5&&row<ROW-5)
            {
                if(col>5&&col<COL-5)
                {
                    (*h13_pixel(row,col))=GREEN_IMG;
                }
            }
        }
    }
}

void Raceing_line_RoundIn_R_Blan_B()
{
    int row,col,pin,find,i;
    int col_min,col_max,flag;
    int star_row,star_col,star_flag;
    int max_row,max_col;
    float k;
    find=0;max_row=0;max_col=188;
    i=1;
    col_min=left_apex.Apex_Col;
    col_max=right_apex.Apex_Col-1;
    for(col=col_min;col<col_max;col++)
    {
        if(col>COL-5) break;
        if(left_apex.Apex_Row+20>right_apex.Apex_Row && right.Row[h13_index(0)]<100)
        {
            max_row=right.Row[h13_index(0)];
            max_col=right.Col[h13_index(0)];
            break;
        }
        for(row=right_end_row-5;row>5;row--)
        {
            if((*h13_pixel(row,col))==WHITE_IMG && (*h13_pixel(row-1,col))==WHITE_IMG)
            {
                if((*h13_pixel(row-2,col))==BLACK_IMG && (*h13_pixel(row-3,col))==BLACK_IMG)
                {
                    if(row>max_row)
                    {
                        max_row=row;
                        max_col=col;
                    }
                    break;
                }
            }
        }
    }
    Round_R.Row[h13_index(0)]=max_row;
    Round_R.Col[h13_index(0)]=max_col;
    round_row_flag=max_row;
    star_row=115;
    star_col=8;
    for(pin=0;pin<240;pin++)
    {
        if(left.Row[h13_index(pin)]==254) break;
        if(left_apex.Apex_Row+20>right_apex.Apex_Row && right.Row[h13_index(0)]<100)
        {
            star_row=115;
            star_col=8;
            break;
        }
        if(left.Row[h13_index(pin)]==right_apex.Apex_Row)
        {
            star_row=left.Row[h13_index(pin)];
            star_col=left.Col[h13_index(pin)];
            star_flag=pin;
            break;
        }
    }
    (*h13_pixel(star_row,star_col))=GREEN_IMG;
    k=(star_col*1.0-Round_R.Col[h13_index(0)])/(star_row*1.0-Round_R.Row[h13_index(0)]);
    row=star_row;
    col=star_col;
    for(pin=1;pin<240;pin++)
    {
        if(star_row-pin>=Round_R.Row[h13_index(0)])
        {
            Round_R.Row[h13_index(pin)]=star_row-pin;
            Round_R.Col[h13_index(pin)]=star_col-(int)pin*k;
            (*h13_pixel(Round_R.Row[h13_index(pin)],Round_R.Col[h13_index(pin)]))=BLUE_IMG;
        }
        else
        {
            find = 0;
            row = star_row - pin;
            if(row<=15)break;
            col_min = Round_R.Col[h13_index(pin - 1)] - 10;
            col_max = Round_R.Col[h13_index(pin - 1)] + 10;
            for(col = col_min;col <= col_max;col++)
            {
                if(col < 5) col=5;
                if(col < COL - 5)
                {
                    if((*h13_pixel(row,col)) == BLACK_IMG && (*h13_pixel(row,col + 1)) == BLACK_IMG)
                    {
                        if((*h13_pixel(row,col + 2)) == WHITE_IMG && (*h13_pixel(row,col + 3)) == WHITE_IMG)
                        {
                            Round_R.Row[h13_index(pin)] = row;
                            Round_R.Col[h13_index(pin)] = col + 1;
                            if(pin>1) (*h13_pixel(Round_R.Row[h13_index(pin)],Round_R.Col[h13_index(pin)]))=BLUE_IMG;
                            find = 1;
                            break;
                        }
                    }
                }
            }
            if(find == 0)
            {
                break;
            }
        }
    }
    row=Round_R.Row[h13_index(1)];
    i=1;
    for(pin=0;pin<240;pin++)
    {
        flag=119-pin;
        if(Round_R.Row[h13_index(i)]==254) break;
        if(flag>Round_R.Row[h13_index(1)])
        {
            middleline[pin]=Round_R.Col[h13_index(1)]+TrackWild[h13_row_index(row)]/2;
            col=middleline[pin];
            mid_row[pin]=flag;
            (*h13_pixel(flag,col))=GREEN_IMG;
        }
        else
        {
            row=Round_R.Row[h13_index(i)];
            col=Round_R.Col[h13_index(i)]+TrackWild[h13_row_index(row)]/2;
            middleline[pin]=col;
            mid_row[pin]=row;
            i++;
            if(row>5&&row<ROW-5)
            {
                if(col>5&&col<COL-5)
                {
                    (*h13_pixel(row,col))=GREEN_IMG;
                }
            }
        }
    }
}







void Raceing_line_RoundOUT_L()
{
    int i,pin;
    if(right_flag==1)
    {
        RacingLine_R(2);
        return;
    }
    for(i=120,pin=0;i>60;i--)
    {
        middleline[pin]=0;
        mid_row[pin]=i;
        pin++;
    }
    return;
}







void Raceing_line_RoundOUT_R()
{
    int i,pin;
    if(left_flag==1)
    {
        RacingLine_L(2);
        return;
    }
    for(i=120,pin=0;i>60;i--)
    {
        middleline[pin]=188;
        mid_row[pin]=i;
        pin++;
    }
    return;
}
int16_t track_ring13_path_x[120];
uint8_t track_ring13_path_confidence[120];
volatile uint32_t track_ring13_phase,track_ring13_lookahead_row;
volatile int32_t track_ring13_side,track_ring13_left_length,track_ring13_right_length;
static uint8_t h13_preview[120][188],h13_reason;
static uint32_t h13_element_ms;
volatile uint32_t track_ring13_jump_count;
volatile int32_t track_ring13_jump_from,track_ring13_jump_to;
volatile int32_t track_ring13_observed_bend;
static int h13_previous_target,h13_previous_valid,h13_pending_target,h13_jump_frames;
static int h13_clip(int n,int a,int b){return n<a?a:(n>b?b:n);}
/* The source's ordinary tracker selects the longer edge and offsets it with
 * a different vehicle's fixed width. Changing the selected edge can reverse
 * the estimated bend. Use measured paired edges wherever they both exist. */
static void h13_normal_path(void){
 int l[120],r[120],i,y,n=0,c;
 for(y=0;y<120;++y)l[y]=r[y]=-1;
 for(i=0;i<239;++i){
  y=left.Row[i];if(y>=20&&y<120&&left.Col[i]>=3&&left.Col[i]<=184)l[y]=left.Col[i];
  y=right.Row[i];if(y>=20&&y<120&&right.Col[i]>=3&&right.Col[i]<=184)r[y]=right.Col[i];
 }
 for(y=119;y>=20;--y){
  if(l[y]>=0 && r[y]>=0){int w=r[y]-l[y];
   if(w<10)continue;
   c=(l[y]+r[y])/2;
   if(Roundabout_flag==NO_ROUND && L_S<0.8f && R_S<0.8f && w>TrackWild[y]*3/5 && w<TrackWild[y]*3/2)
    TrackWild[y]=(7*TrackWild[y]+w)/8;
  }else if(l[y]>=0)c=l[y]+TrackWild[y]/2;
  else if(r[y]>=0)c=r[y]-TrackWild[y]/2;
  else continue;
  if(c<3||c>184||!h13_preview[y][c])continue;
  middleline[n]=c;mid_row[n]=y;++n;
 }
 for(;n<240;++n){middleline[n]=254;mid_row[n]=254;}
}
static int h13_observed_curve(void){
 int l[120],r[120],i,y,far=0,near=0,nf=0,nn=0;
 for(y=0;y<120;++y)l[y]=r[y]=-1;
 for(i=0;i<239;++i){
  y=left.Row[i];if(y>=20&&y<120&&left.Col[i]>=3&&left.Col[i]<=184)l[y]=left.Col[i];
  y=right.Row[i];if(y>=20&&y<120&&right.Col[i]>=3&&right.Col[i]<=184)r[y]=right.Col[i];
 }
 for(y=62;y<=80;++y)if(l[y]>=0&&r[y]>l[y]+10){far+=(l[y]+r[y])/2;++nf;}
 for(y=96;y<=110;++y)if(l[y]>=0&&r[y]>l[y]+10){near+=(l[y]+r[y])/2;++nn;}
 return nf>=5&&nn>=5?far/nf-near/nn:999;
}
void track_ring13_suspend(uint8_t why){
 Roundabout_flag=NO_ROUND;Roundabout_flag_position=NOROUND;element_flag=NO_JUGED;
 h13_reason=why;h13_element_ms=0;row_lim=20;
 h13_previous_valid=0;h13_jump_frames=0;
}
void track_ring13_reset(void){
 track_mode4_reset();memset(track_ring13_path_confidence,0,120);
 left_apex.Apex_Row=right_apex.Apex_Row=254;
 left_apex.Apex_Col=right_apex.Apex_Col=254;
 track_ring13_suspend(FSM_REASON_INIT);Garage_flag=NO_GARAGE;
 {int y;for(y=0;y<120;++y)TrackWild[y]=h13_clip(TRACK_CONT_WIDTH_PRIOR*(y+10)/126,8,187);}
 track_ring13_jump_count=0;track_ring13_jump_from=track_ring13_jump_to=0;
}
const char *track_ring13_state_name(uint8_t s){
 switch(s){case FSM_WAIT:return "WAIT";case FSM_NORMAL:return "TRACK";
 case FSM_RING_CANDIDATE:return "R CHECK";case FSM_RING_ENTER:return "R ENTER";
 case FSM_RING_INSIDE:return "R CIRCLE";case FSM_RING_EXIT:return "R EXIT";
 case FSM_RING_MERGE:return "R MERGE";default:return "NO PATH";}
}
void track_ring13_binary_preview(const uint8_t input[120][188],uint8_t output[120][188],uint8_t t){
 (void)input;(void)t;memcpy(output,h13_preview,sizeof(h13_preview));
}
track_fsm_result_t track_ring13_process(const uint8_t image[120][188],uint32_t dt,uint8_t run){
 track_fsm_result_t out={0};track_mode4_result_t segment=track_mode4_process(image);
 int x,y,i,last=-1,count=0,target=-1,anchor=-1,continuous=1,first=-1;
 /* The camera/threshold adapter replaces Deal_Init, not the imported geometry. */
 for(y=0;y<120;++y)for(x=0;x<188;++x)
  h13_preview[y][x]=IMG_DATA[y][x]=image[y][x]>=segment.global_threshold?255:0;
 InitData();CorssCol=Cross_col();LeftStartFind();RightStartFind();fixpoint();
 left_jump();right_jump();Left_Apex();Right_Apex();
 track_ring13_observed_bend=h13_observed_curve();
 /* Never carry an old apex into a frame where its boundary disappeared. */
 if(!left_flag){left_apex.Apex_Row=left_apex.Apex_Col=254;L_S=254;}
 if(!right_flag){right_apex.Apex_Row=right_apex.Apex_Col=254;R_S=254;}
 if(run && dt<TRACK_CAMERA_LOST_TIMEOUT_MS){Roundabout_Deal();
  if(Roundabout_flag!=NO_ROUND)h13_element_ms+=dt;else h13_element_ms=0;
  if(h13_element_ms>=TRACK_FSM_ELEMENT_TIMEOUT_MS)track_ring13_suspend(FSM_REASON_ELEMENT_TIMEOUT);
 }else track_ring13_suspend(run?FSM_REASON_CAMERA:FSM_REASON_STOP);
 /* Complete ring dispatch. Use the author's RoundOUT routines for exit, rather
  * than his garage dispatch which can emit 0/188 or no path at all. */
 switch(Roundabout_flag){
 case GET_IN_ROUND:
  if(Roundabout_flag_position==ROUND_L)Raceing_line_RoundIn_L_Blan_B();
  else Raceing_line_RoundIn_R_Blan_B();break;
 case IN_ROUND:
  if(Roundabout_flag_position==ROUND_L){if(right.Col[0]<140)RacingLine_R(2);else h13_normal_path();}
  else {if(left.Col[0]>48)RacingLine_L(2);else h13_normal_path();}break;
 case GET_OUT_ROUND:case OUTTING:case OUT_WAY:case OUT_ROUND:
  if(Roundabout_flag_position==ROUND_L)Raceing_line_RoundOUT_L();else Raceing_line_RoundOUT_R();break;
 default:h13_normal_path();break;
 }
 memset(track_ring13_path_confidence,0,120);
 for(i=0;i<239;++i){y=mid_row[i];x=middleline[i];
  if(y<20||y>119||x<3||x>184)continue;
  if(!h13_preview[y][x])continue;
  track_ring13_path_x[y]=x;track_ring13_path_confidence[y]=70;
 }
 /* Connect only bounded gaps whose entire interpolated line remains white. */
 for(y=20;y<=119;++y)if(track_ring13_path_confidence[y]){
  if(last>=0 && y-last<=20){int k,ok=1;
   for(k=last+1;k<y;++k){x=track_ring13_path_x[last]+(track_ring13_path_x[y]-track_ring13_path_x[last])*(k-last)/(y-last);
    if(!h13_preview[k][x]){ok=0;break;}}
   if(ok)for(k=last+1;k<y;++k){track_ring13_path_x[k]=track_ring13_path_x[last]+(track_ring13_path_x[y]-track_ring13_path_x[last])*(k-last)/(y-last);track_ring13_path_confidence[k]=40;}
  }last=y;
 }
 for(y=118;y>=20;--y)if(track_ring13_path_confidence[y]){if(anchor<0)anchor=y;++count;first=y;}
 if(anchor>=108 && anchor<116)for(y=anchor+1;y<=116;++y){x=track_ring13_path_x[anchor];if(!h13_preview[y][x])break;track_ring13_path_x[y]=x;track_ring13_path_confidence[y]=40;}
 for(i=0;i<=20 && target<0;++i){int a=72+i,b=72-i;
  if(a<=100 && track_ring13_path_confidence[a])target=a;
  else if(b>=30 && track_ring13_path_confidence[b])target=b;
 }
 if(target>=0)for(y=target;y<=110;++y){
  if(!track_ring13_path_confidence[y] || (y<110 && track_ring13_path_confidence[y+1] && abs(track_ring13_path_x[y+1]-track_ring13_path_x[y])>8)){continuous=0;break;}}
 else continuous=0;
 (void)first;
 out.state=Roundabout_flag==NO_ROUND?FSM_NORMAL:
  (Roundabout_flag<=GET_IN_READY?FSM_RING_CANDIDATE:(Roundabout_flag==GET_IN_ROUND?FSM_RING_ENTER:
  (Roundabout_flag==IN_ROUND?FSM_RING_INSIDE:(Roundabout_flag<=OUTTING?FSM_RING_EXIT:FSM_RING_MERGE))));
 if(!run)out.state=FSM_WAIT;else if(!continuous)out.state=FSM_RECOVER;
 out.anchor_x=anchor>=0?track_ring13_path_x[anchor]:TRACK_STEER_REFERENCE_X;
 out.target_x=target>=0?track_ring13_path_x[target]:TRACK_STEER_REFERENCE_X;
 out.anchor_confidence=anchor>=108?70:0;out.target_confidence=target>=0?track_ring13_path_confidence[target]:0;
 out.valid_rows=count;out.observed_rows=L_lenth>R_lenth?L_lenth:R_lenth;
 out.global_threshold=segment.global_threshold;out.reason=h13_reason;
 out.ring_side=Roundabout_flag_position==ROUND_L?-1:(Roundabout_flag_position==ROUND_R?1:0);
 out.servo_cdeg=100*h13_clip((int)TRACK_SERVO_CENTER_DEG-((int)out.target_x-TRACK_STEER_REFERENCE_X)*(int)TRACK_STEER_GAIN_CENTIDEG_PER_PIXEL/100,TRACK_SERVO_RIGHT_DEG,TRACK_SERVO_LEFT_DEG);
 out.drive_allowed=run && dt<TRACK_CAMERA_LOST_TIMEOUT_MS && continuous && anchor>=108 && count>=30;
 /* A repaired route must not oppose the bend seen by two real boundaries. */
 if(out.drive_allowed && track_ring13_observed_bend!=999 &&
    ((track_ring13_observed_bend<-8 && out.target_x-out.anchor_x>8) ||
     (track_ring13_observed_bend>8 && out.target_x-out.anchor_x<-8))){
  ++track_ring13_jump_count;track_ring13_jump_from=out.anchor_x;track_ring13_jump_to=out.target_x;
  out.drive_allowed=0;out.state=FSM_RECOVER;out.reason=FSM_REASON_REJECTED;
 }
 /* A large opposite-side target jump is held for confirmation, never sent
  * immediately to the steering actuator. Keep its evidence after STOP. */
 if(out.drive_allowed && h13_previous_valid && abs((int)out.target_x-h13_previous_target)>20 &&
    ((out.target_x<TRACK_STEER_REFERENCE_X-6 && h13_previous_target>TRACK_STEER_REFERENCE_X+6) ||
     (out.target_x>TRACK_STEER_REFERENCE_X+6 && h13_previous_target<TRACK_STEER_REFERENCE_X-6))){
  if(!h13_jump_frames || abs((int)out.target_x-h13_pending_target)>8){
   ++track_ring13_jump_count;track_ring13_jump_from=h13_previous_target;
   track_ring13_jump_to=out.target_x;h13_pending_target=out.target_x;h13_jump_frames=1;
  }else ++h13_jump_frames;
  if(h13_jump_frames<3){out.drive_allowed=0;out.state=FSM_RECOVER;out.reason=FSM_REASON_REJECTED;}
 }
 else h13_jump_frames=0;
 if(out.drive_allowed){h13_previous_target=out.target_x;h13_previous_valid=1;h13_jump_frames=0;}
 if(!out.drive_allowed)out.servo_cdeg=100*TRACK_SERVO_CENTER_DEG;
 out.pwm_cap_percent=TRACK_WHEEL_PWM_MAX_PERCENT;
 track_ring13_phase=Roundabout_flag;track_ring13_side=out.ring_side;
 track_ring13_lookahead_row=target>=0?target:72;
 track_ring13_left_length=L_lenth;track_ring13_right_length=R_lenth;
 return out;
}
