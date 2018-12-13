#include "track.h"
float chassis_speed_max = 1500;//底盘速度曲线最大速度
float chassis_xpos[NUM_POINTS] = {0};//X坐标 
float chassis_ypos[NUM_POINTS] = {0};//Y坐标 
float chassis_turn[NUM_POINTS] = {0};//自转 
int chassis_posnum = 1;

PointF in[input_num] = {{0,1.5},{1.5,2.5},{3,0.5},{4,5},{6,7},{7,9},{6,0},{8,6}}; // 输入点


void track_goline(int n, float x1, float y1, float x2, float y2)
//参数：分段数 起始点x 起始点y 终止点x 终止点y // n >= 1
{
	if(n <= 0) n = 1;
    float deltaX = x2 - x1;
    float deltaY = y2 - y1;
    for(int i = 1; i <= n; i++)
    {
    	chassis_xpos[chassis_posnum] = x1 + deltaX * i / n;
    	chassis_ypos[chassis_posnum] = y1 + deltaY * i / n;
    	if(chassis_posnum == 0)
    	{
    		chassis_turn[chassis_posnum] = 0;//初始自转为0 
		}
		else
		{
			chassis_turn[chassis_posnum] = chassis_turn[chassis_posnum - 1];//保持上一自转角度 
		}
    	chassis_posnum++;
	}
}

void track_goarc(int n, float X0, float Y0, float X, float Y, float Angle, int rotateflag)
//参数：分段数 圆心x 圆心y 起始点x 起始点y 旋转角度(不是弧度，旋转方向逆时针为正) 是否跟随自转 
{
	if(n <= 0) n = 1;
    float r = sqrt((X - X0) * (X - X0) + (Y - Y0) * (Y - Y0));
    float theta = atan2(Y0 - Y, X - X0);
    float Rad = PI * Angle / 180;
    for(int i = 1; i <= n; i++)
    {
    	float theta2 = theta - i * Rad / n;
    	chassis_xpos[chassis_posnum] = X0 + r * cos(theta2);
    	chassis_ypos[chassis_posnum] = Y0 - r * sin(theta2);
    	if(rotateflag)
    	{
    		chassis_turn[chassis_posnum] = chassis_turn[chassis_posnum - 1] + Rad / n; 
		}
    	else
    	{
    		chassis_turn[chassis_posnum] = chassis_turn[chassis_posnum - 1];
		}
		chassis_posnum++;
	}
}

void track_init()
{
//	track_goline(30, 0, 0, 0.6, 0.3464);
//	track_goarc(60, 0.8, 0, 0.6, 0.3464, -120, 0);
//    track_goarc(60, 1.6, 0, 1.2, 0, 180, 0);
//    track_goarc(60, 2.4, 0, 2, 0, -180, 0);
//    
//    track_goarc(60, 2.4, 0, 2.8, 0, -180, 0);
//    track_goarc(60, 1.6, 0, 2, 0, 180, 0);
//    track_goarc(60, 0.8, 0, 1.2, 0, -120, 0);
//    track_goline(30, 0.6, -0.3464, 0, 0);
//      track_goline(60, 0, 0, 0, -2.5);
//      track_goarc(60, -0.5, -2.5, 0, -2.5, -360, 1);
//      track_goline(60, 0, -2.5, 0, 0);
        track_bezier_init();
}
