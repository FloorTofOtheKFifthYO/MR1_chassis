/*******************************************************************************
Copyright:      2018/12/18
File name:      track.c
Description:    ���̹켣�滮
Author:         ������
Version��       1.0
Data:           2018/12/18 22:36
History:        �� 
Bug:            NUM_POINTS>5000���һ����
*******************************************************************************/
#include "track.h"
float chassis_speed_max = 3000;//�����ٶ���������ٶ�
float chassis_xpos[NUM_POINTS] = {0};//X���� 
float chassis_ypos[NUM_POINTS] = {0};//Y���� 
float chassis_turn[NUM_POINTS] = {0};//��ת 
int chassis_posnum = 1;

PointF in[input_num] = {{0,0},{1.0,1.0},{-1.0,2.0},{0,3.0},{1.0,2.0},{-1.0,1.0},{0,0}}; // ����� 
//����������������.h�������input_num��Ŀ

void track_goline(int n, float x1, float y1, float x2, float y2)
//�������ֶ��� ��ʼ��x ��ʼ��y ��ֹ��x ��ֹ��y // n >= 1
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
    		chassis_turn[chassis_posnum] = 0;//��ʼ��תΪ0 
		}
		else
		{
			chassis_turn[chassis_posnum] = chassis_turn[chassis_posnum - 1];//������һ��ת�Ƕ� 
		}
    	chassis_posnum++;
	}
}

void track_goarc(int n, float X0, float Y0, float X, float Y, float Angle, int rotateflag)
//�������ֶ��� Բ��x Բ��y ��ʼ��x ��ʼ��y ��ת�Ƕ�(���ǻ��ȣ���ת������ʱ��Ϊ��) �Ƿ������ת 
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
//        track_goarc(60, 1.6, 0, 1.2, 0, 180, 0);
//        track_goarc(60, 2.4, 0, 2, 0, -180, 0);
//    
//        track_goarc(60, 2.4, 0, 2.8, 0, -180, 0);
//        track_goarc(60, 1.6, 0, 2, 0, 180, 0);
//        track_goarc(60, 0.8, 0, 1.2, 0, -120, 0);
//        track_goline(30, 0.6, -0.3464, 0, 0);
//        for(int i = 0;i<chassis_posnum;i++)
//        {
//            uprintf(CMD_USART,"x=%f  y=%f\r\n",chassis_xpos[i],chassis_ypos[i]);
//        }
//      track_goline(6, 0, 0, 0, -1);
//      track_goline(6, 0, -1, -1, -1);
//      track_goline(6, -1, -1, -1, 0);
//      track_goline(6, -1, 0, 0, 0);
//      
//      track_goline(6, 0, 0, 0, -1);
//      track_goline(6, 0, -1, -1, -1);
//      track_goline(6, -1, -1, -1, 0);
//      track_goline(6, -1, 0, 0, 0);
//      
//      track_goline(6, 0, 0, 0, -1);
//      track_goline(6, 0, -1, -1, -1);
//      track_goline(6, -1, -1, -1, 0);
//      track_goline(6, -1, 0, 0, 0);
//      
//      track_goline(6, 0, 0, 0, -1);
//      track_goline(6, 0, -1, -1, -1);
//      track_goline(6, -1, -1, -1, 0);
//      track_goline(6, -1, 0, 0, 0);
//      //track_goarc(60, -0.5, -1, 0
//      track_goline(6, 0, 0, 0, -1);
//      track_goline(6, 0, -1, -1, -1);
//      track_goline(6, -1, -1, -1, 0);
//      track_goline(6, -1, 0, 0, 0);
      //track_goarc(60, -0.5, -1, -1, -1, -180, 0);
           //track_goarc(60, -0.5, -1, 0, -1, -360, 0);
      //track_goline(6, 0, -1, 0, 0);
          track_bezier_init();
//        track_goarc(60, 0, -0.5, 0, 0, -180, 0);
//        track_goarc(60, 0, -0.5, 0, -1, -180, 0);
//        
//        track_goarc(60, 0, -0.5, 0, 0, -180, 0);
//        track_goarc(60, 0, -0.5, 0, -1, -180, 0);
//        
//        track_goarc(60, 0, -0.5, 0, 0, -180, 0);
//        track_goarc(60, 0, -0.5, 0, -1, -180, 0);
//        
//        track_goarc(60, 0, -0.5, 0, 0, -180, 0);
//        track_goarc(60, 0, -0.5, 0, -1, -180, 0);
//        
//        track_goarc(60, 0, -0.5, 0, 0, -180, 0);
//        track_goarc(60, 0, -0.5, 0, -1, -180, 0);
}
