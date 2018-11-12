#include "chassis.h"
Chassis chassis;
//������ȫ����λģ�鰲װƫ��
float ERR_angle_m3 = 0 , ERR_angle_m1 = 0 + 1*2*PI/3 , ERR_angle_m0 = 0 + 2*2*PI/3 ;
static float Chassis_motor0 =0 , Chassis_motor1 =0 , Chassis_motor3 =0;
static int chassis_poscnt = 0;
static int chassis_posnum = 5;
static int routeflag = 0;
double chassis_xpos[] = { 0, 1, 1, 0, 0};
double chassis_ypos[] = { 0, 0, -1, -1, 0};
double *chassis_dis_to_bgn;
double *chassis_dis_to_end;

float Angle_KP = -1500;
float Angle_KD = 0;
float turn_output = 0;

int g_ispeed = 0;
float g_fangle = 0;
float g_fturn = 0;

/**���̳�ʼ��
*��������
*����ֵ�� ��
*/

void chassis_init(void)
{
    chassis_calculate_dis_matrix();
    vega_init(&(chassis.g_vega_pos_x),&(chassis.g_vega_pos_y),&(chassis.g_vega_angle));
    vega_reset();

    HAL_Delay(2000);//��ʱ�ȴ���λ
    vega_set_pos(0,0);
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0);
    uprintf("chassis and vega are ready!!!\r\n");
}
/**���̸�������
*��������
*����ֵ�� ��
*/
void chassis_update(void)
{
    chassis.pos_x = chassis.g_vega_pos_x* 0.0001 * 0.81;
    chassis.pos_y = - chassis.g_vega_pos_y* 0.0001 * 0.81;//����ط�ȫ����λy���ˣ�ע�⣡������
    chassis.angle = - (chassis.g_vega_angle/180.f)*PI;
}



float chassis_angle_subtract(float a, float b)
{
    float out = a - b;
    while(out > PI)
	{
		out -= 2 * PI;
	}
	while(a - b < - PI)
	{
		out += 2 * PI;
	}
    return out;
    /*
	if(a - b > PI)
	{
		return - 2 * PI + a - b;
	}
	else if(a - b < - PI)
	{
		return 2 * PI + a - b;
	}
	else 
	{
		return a - b;
	}*/
}

float chassis_PID_Angle_Control(float target_angle){
  
  float angle_err=chassis_angle_subtract(target_angle, chassis.angle); 
  static float angle_last_err = 0;
  
  float P_out=angle_err*Angle_KP;
  float D_out=(angle_last_err-angle_err)*Angle_KD;
  angle_last_err=angle_err;
  
  return P_out+D_out;
}



/**������ֱ��
*������float angle 	�����
*      int   speed    �ٶ�
*����ֵ�� ��
*˵����������ת������
*/
void chassis_gostraight(int speed , float angle, float turn)
{
    Chassis_motor0 = -(speed*cos((ERR_angle_m0) - angle));
    Chassis_motor1 = -(speed*cos((ERR_angle_m1) - angle));
    Chassis_motor3 = -(speed*cos((ERR_angle_m3) - angle));
    //Chassis_motor3 = -(speed*cos((ERR_angle_m3 + chassis.angle) - angle));
    
    turn_output = chassis_PID_Angle_Control(turn);
    if(turn_output >2000)
    {
        turn_output = 2000;
    }
    if(turn_output < -2000)
    {
        turn_output = -2000;
    }
        maxon_setSpeed(&MOTOR0_ID,(int)(Chassis_motor0 + turn_output));
        maxon_setSpeed(&MOTOR3_ID,(int)(Chassis_motor3 + turn_output));
        maxon_setSpeed(&MOTOR1_ID,(int)(Chassis_motor1 + turn_output));
}
/*
int chassis_withinpoint(double x1, double y1, double x2, double y2)
{
    if((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) <= 0.005 * 0.005) return 1;
    else return 0;
}*/

void chassis_go_route(int flag)
{
    routeflag = flag;
    uprintf("go_route flag = %d \r\n",flag);
}
//�������
void chassis_calculate_dis_matrix()
{
    chassis_dis_to_bgn = (double*)malloc((chassis_posnum - 1) * sizeof(double));
    chassis_dis_to_end = (double*)malloc((chassis_posnum - 1) * sizeof(double));
    double lastdis = 0;
    chassis_dis_to_bgn[0] = 0;
    for(int i = 1; i < chassis_posnum; i++)
    {
        double deltaX = chassis_xpos[i] - chassis_xpos[i - 1];
        double deltaY = chassis_ypos[i] - chassis_ypos[i - 1];
        chassis_dis_to_bgn[i] = sqrt(deltaX * deltaX + deltaY * deltaY) + lastdis;
        lastdis = chassis_dis_to_bgn[i];
    }
    lastdis = 0;
    chassis_dis_to_end[chassis_posnum - 1] = 0;
    for(int i = chassis_posnum - 2; i >= 0; i--)
    {
        double deltaX = chassis_xpos[i] - chassis_xpos[i + 1];
        double deltaY = chassis_ypos[i] - chassis_ypos[i + 1];
        chassis_dis_to_end[i] = sqrt(deltaX * deltaX + deltaY * deltaY) + lastdis;
        lastdis = chassis_dis_to_end[i];
    }
}
int start_point=0;
int chassis_calculate_speed(int target_speed,int target_point)
{
      double x1 = 0;
      double param_a = 0, param_b = 0, param_c = 0, param_d = 0;
      double delt_dis = 0;
      //double delta_speed = last_speed - target_speed;
      static double speed;
      x1 = chassis_dis_to_bgn[target_point];
      delt_dis = sqrt(pow(chassis.pos_x - chassis_xpos[target_point - 1] , 2) + pow(chassis.pos_y - chassis_ypos[target_point - 1] , 2));
      if( speed < target_speed )//С��Ŀ���ٶ�
          {
              speed =( param_c / ( 1 + pow( EXP , ( - log ( pow( EXP , param_a * ( delt_dis + param_b ) )-1) ) ) ) ) + param_d;             
          }
      if( abs ( (int)speed - target_speed ) <= 1 )//Լ����Ŀ���ٶ�
              speed = target_speed;
      if( speed > target_speed )//����Ŀ���ٶ�
          {
              speed =( param_c / ( 1 + pow( EXP , ( - log ( pow( EXP , param_a * ( x1 - delt_dis + param_b ) )-1) ) ) ) ) + param_d;
          }
      if(delt_dis >= chassis_dis_to_bgn[target_point])
              start_point++;
      return (int)speed;
//      if( abs ( (int)speed - target_speed ) <= 10 )//���ٵ�Ŀ���ٶ�����
//          {
//              if( flag == 0)//��һ�ε�Ŀ���ٶ�
//                  {
//                      flag=1;
//                      acc_delt_dis = delt_dis;//��ǰλ�ƴ�Ϊ���ٶε�λ��
//                  }
//              speed = target_speed;//�ٶ��ٲ��仯
//          }
//      if( flag == 1 && ( dis_to_nextpos - delt_dis ) <= acc_delt_dis )//�ƶ����˼��ٶ�
//          {
//              speed =( c / ( 1 + pow( EXP , ( - log ( pow( EXP , a * ( dis_to_nextpos - delt_dis + b ) )-1) ) ) ) ) + d; 
//              flag = 0;
//          }
//      if( ( dis_to_nextpos - delt_dis) <= delt_dis && flag = 0)//��û������ٽ׶Σ����ҵ�ǰλ���ٲ����ټ��پ����Ѿ������˵Ļ�
//          {
//              speed =( c / ( 1 + pow( EXP , ( - log ( pow( EXP , a * ( dis_to_nextpos - delt_dis + b ) )-1) ) ) ) ) + d; 
//          }          
//      if( abs( dis_to_nextpos - delt_dis ) <= 1)//�����յ�ǳ���������ǿ��ͣ��
//          {
//              speed = 0;
//              flag = 0;
//          }
//    double x1 = 0, x2 = 0;
//    double param_a = 3, param_b = 5;
//    double delta_speed = last_speed - target_speed;
//    x1 = chassis_dis_to_bgn[chassis_poscnt] - dis_to_nextpos;
//    x2 = chassis_dis_to_bgn[chassis_poscnt - 1] + dis_to_nextpos;
//    
//    return 1;
}

//����ִ�к���
void chassis_exe()
{
    if(1 == routeflag)
    {
        double deltaX = chassis.pos_x - chassis_xpos[chassis_poscnt];
        double deltaY = chassis.pos_y - chassis_ypos[chassis_poscnt];
        double dis_to_next = sqrt(deltaX * deltaX + deltaY * deltaY);
        //if(!chassis_withinpoint(chassis.pos_x, chassis.pos_y, chassis_xpos[chassis_poscnt], chassis_ypos[chassis_poscnt]))
        if(dis_to_next <= 0.005)
        {
            double angle = atan2( chassis_ypos[chassis_poscnt] - chassis.pos_y,  chassis_xpos[chassis_poscnt] - chassis.pos_x);
            g_ispeed = chassis_calculate_speed( 600);
            g_fangle = angle;
            g_fturn = 0;
        }
        else
        {
            chassis_poscnt++;
            if(chassis_poscnt >= chassis_posnum)
            {
                chassis_poscnt = 0;//���ι켣
                g_ispeed = 0;
                g_fangle = 0;
                g_fturn = 0;
                routeflag = 0;
            }
        }
    }
    chassis_gostraight(g_ispeed ,g_fangle,g_fturn);
}