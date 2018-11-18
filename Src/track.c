#include "track.h"
double chassis_speed_max = 1500;//�����ٶ���������ٶ�
double chassis_xpos[NUM_POINTS] = {0};//X���� 
double chassis_ypos[NUM_POINTS] = {0};//Y���� 
double chassis_turn[NUM_POINTS] = {0};//��ת 
int chassis_posnum = 1;
void track_goline(int n, double x1, double y1, double x2, double y2)
//�������ֶ��� ��ʼ��x ��ʼ��y ��ֹ��x ��ֹ��y // n >= 1
{
	if(n <= 0) n = 1;
    double deltaX = x2 - x1;
    double deltaY = y2 - y1;
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

void track_goarc(int n, double X0, double Y0, double X, double Y, double Angle, int rotateflag)
//�������ֶ��� Բ��x Բ��y ��ʼ��x ��ʼ��y ��ת�Ƕ�(���ǻ��ȣ���ת������ʱ��Ϊ��) �Ƿ������ת 
{
	if(n <= 0) n = 1;
    double r = sqrt((X - X0) * (X - X0) + (Y0 - Y) * (Y - Y0));
    double theta = atan2(Y - Y0, X - X0);
    double Rad = PI * Angle / 180;
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
	track_goline(10, 0, 0, 1, 1);
	track_goarc(70, 2, 1, 1, 1, 180, 1);
} 