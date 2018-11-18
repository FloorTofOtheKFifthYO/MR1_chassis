#include "chassis.h"
Chassis chassis;
//三轮与全场定位模块安装偏角
float ERR_angle_m3 = 0 , ERR_angle_m1 = 0 + 1*2*PI/3 , ERR_angle_m0 = 0 + 2*2*PI/3 ;
static float Chassis_motor0 =0 , Chassis_motor1 =0 , Chassis_motor3 =0;
static int chassis_poscnt = 0;
static int chassis_posnum = 73;
static int routeflag = 0;
double param_a = 0, param_b = 0,setspeed=0; 
//double chassis_xpos[NUM_POINTS] =  { 0, 1,  2,  1, 0};
//double chassis_ypos[NUM_POINTS] =  { 0, 0, 0, 0, 0};
//double chassis_speed[NUM_POINTS] = { 0,2000,0,2000,0};//速度表
double chassis_turn[NUM_POINTS] = { 0, 0,  0,  0, 0};//自转角度表
//double chassis_xpos[NUM_POINTS] =  { 0, 1, 1.5,2,2.5};
//double chassis_ypos[NUM_POINTS] =  { 0, 0, 0,0};
//double chassis_speed[NUM_POINTS] = { 0,100,200,100,0};//速度表
//double chassis_turn[NUM_POINTS] = { 0, 0,  0,  0, 0};//自转角度表
//double chassis_xpos[NUM_POINTS] = {0, 0.0151922, 0.0603074, 0.133975, 0.233956, 0.357212, 0.5, 0.65798, 0.826352, 1, 1.17365, 1.34202, 1.5, 1.64279, 1.76604, 1.86603, 1.93969, 1.98481, 2, 1.98481, 1.93969, 1.86603, 1.76604, 1.64279, 1.5, 1.34202, 1.17365, 1, 0.826352, 0.65798, 0.5, 0.357212, 0.233956, 0.133975, 0.0603074, 0.0151922, 0};
//double chassis_ypos[NUM_POINTS] = {0, 0.173648, 0.34202, 0.5, 0.642788, 0.766044, 0.866025, 0.939693, 0.984808, 1, 0.984808, 0.939693, 0.866025, 0.766044, 0.642788, 0.5, 0.34202, 0.173648, 1.22461e-016, -0.173648, -0.34202, -0.5, -0.642788, -0.766044, -0.866025, -0.939693, -0.984808, -1, -0.984808, -0.939693, -0.866025, -0.766044, -0.642788, -0.5, -0.34202, -0.173648, 0};
double chassis_xpos[NUM_POINTS] =  {0, 0.0038053, 0.0151922, 0.0340742, 0.0603074, 0.0936922, 0.133975, 0.180848, 0.233956, 0.292893, 0.357212, 0.426424, 0.5, 0.577382, 0.65798, 0.741181, 0.826352, 0.912844, 1, 1.08716, 1.17365, 1.25882, 1.34202, 1.42262, 1.5, 1.57358, 1.64279, 1.70711, 1.76604, 1.81915, 1.86603, 1.90631, 1.93969, 1.96593, 1.98481, 1.99619, 2, 1.99619, 1.98481, 1.96593, 1.93969, 1.90631, 1.86603, 1.81915, 1.76604, 1.70711, 1.64279, 1.57358, 1.5, 1.42262, 1.34202, 1.25882, 1.17365, 1.08716, 1, 0.912844, 0.826352, 0.741181, 0.65798, 0.577382, 0.5, 0.426424, 0.357212, 0.292893, 0.233956, 0.180848, 0.133975, 0.0936922, 0.0603074, 0.0340742, 0.0151922, 0.0038053, 0};
double chassis_ypos[NUM_POINTS] =  {0, 0.0871557, 0.173648, 0.258819, 0.34202, 0.422618, 0.5, 0.573576, 0.642788, 0.707107, 0.766044, 0.819152, 0.866025, 0.906308, 0.939693, 0.965926, 0.984808, 0.996195, 1, 0.996195, 0.984808, 0.965926, 0.939693, 0.906308, 0.866025, 0.819152, 0.766044, 0.707107, 0.642788, 0.573576, 0.5, 0.422618, 0.34202, 0.258819, 0.173648, 0.0871557, 1.22461e-016, -0.0871557, -0.173648, -0.258819, -0.34202, -0.422618, -0.5, -0.573576, -0.642788, -0.707107, -0.766044, -0.819152, -0.866025, -0.906308, -0.939693, -0.965926, -0.984808, -0.996195, -1, -0.996195, -0.984808, -0.965926, -0.939693, -0.906308, -0.866025, -0.819152, -0.766044, -0.707107, -0.642788, -0.573576, -0.5, -0.422618, -0.34202, -0.258819, -0.173648, -0.0871557, 0};
//double chassis_speed[NUM_POINTS] = { 0,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,0};//速度表
double chassis_dis_to_bgn[NUM_POINTS];
double chassis_dis_to_end[NUM_POINTS];
double chassis_speed_max = 1500;
  
        
float Angle_KP = -1500;
float Angle_KD = 0;
float turn_output = 0;

int g_ispeed = 0;
float g_fangle = 0;
float g_fturn = 0;

int start_point=0;

int start_flag=0;
/**底盘初始化
*参数：无
*返回值： 无
*/

void chassis_init(void)
{
    chassis_calculate_dis_matrix();
    vega_init(&(chassis.g_vega_pos_x),&(chassis.g_vega_pos_y),&(chassis.g_vega_angle));
    vega_reset();

    HAL_Delay(2000);//延时等待复位
    vega_set_pos(0,0);
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0);
    uprintf("chassis and vega are ready!!!\r\n");
}
/**底盘更新坐标
*参数：无
*返回值： 无
*/
void chassis_update(void)
{
    chassis.pos_x = chassis.g_vega_pos_x* 0.0001 * 0.81;
    chassis.pos_y = - chassis.g_vega_pos_y* 0.0001 * 0.81;//这个地方全场定位y反了，注意！！！！
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
}

float chassis_PID_Angle_Control(float target_angle){
  
  float angle_err=chassis_angle_subtract(target_angle, chassis.angle); 
  static float angle_last_err = 0;
  
  float P_out=angle_err*Angle_KP;
  float D_out=(angle_last_err-angle_err)*Angle_KD;
  angle_last_err=angle_err;
  
  return P_out+D_out;
}



/**底盘走直线
*参数：float angle 	方向角
*      int   speed    速度
*返回值： 无
*说明：不会自转！！！
*/
void chassis_gostraight(int speed , float angle, float turn)
{
    Chassis_motor0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - angle));
    Chassis_motor1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - angle));
    Chassis_motor3 = -(speed*cos((ERR_angle_m3 + chassis.angle) - angle));
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

void chassis_go_route(int flag)
{
    routeflag = flag;
    uprintf("go_route flag = %d \r\n",flag);
}
//计算距离
void chassis_calculate_dis_matrix()
{
    //chassis_dis_to_bgn = (double*)malloc((chassis_posnum) * sizeof(double));
    //chassis_dis_to_end = (double*)malloc((chassis_posnum) * sizeof(double));
    double lastdis = 0;
    chassis_dis_to_bgn[0] = 0;
    for(int i = 1; i < chassis_posnum; i++)
    {
        double deltaX = chassis_xpos[i] - chassis_xpos[i - 1];
        double deltaY = chassis_ypos[i] - chassis_ypos[i - 1];
        double distance = sqrt(deltaX * deltaX + deltaY * deltaY) + lastdis;
        chassis_dis_to_bgn[i] = distance;
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
/****
    *@brief 速度计算
    *@param int target_point :目标点
    *@retval 返回当前速度
*/
int chassis_calculate_speed(int target_point)//target_point从1开始
{
       // static double now_speed = 0;
        double speed = 0;
        double x0 = 0,delt_dis_bgn = 0,delt_dis_end = 0;//存放进入上一个点起始速度对应的初始位移     
        //double angle = 0;
        static double xtrue = 0;//速度位移曲线的自变量
        //double dis_betwin_two = chassis_dis_to_bgn[target_point];
        
        delt_dis_bgn = sqrt(pow(chassis.pos_x - chassis_xpos[target_point - 1] , 2) + pow(chassis.pos_y - chassis_ypos[target_point - 1] , 2));
        //uprintf("delt_dis_bgn = %f\r\n",delt_dis_bgn);
        delt_dis_end = sqrt(pow(chassis.pos_x - chassis_xpos[target_point] , 2) + pow(chassis.pos_y - chassis_ypos[target_point] , 2));
        //uprintf("delt_dis_end = %f\r\n",delt_dis_end); 
//        if(delt_dis_end <= 1)
//            return 1;
        
        //angle = atan2(chassis.pos_x - chassis_xpos[target_point - 1],chassis.pos_y - chassis_ypos[target_point - 1]);
        //uprintf("angle = %f\r\n",angle); 
        if(chassis_speed[target_point - 1] < chassis_speed[target_point])
            {
                x0=param_a*(log(param_a/(param_a/2-chassis_speed[target_point - 1]))-0.5*log((2*chassis_speed[target_point - 1]+param_a)/(param_a-2*chassis_speed[target_point - 1]))-log(2));
                xtrue = x0 + delt_dis_bgn*param_b;
                //speed=(param_a/2)*pow(1-pow(EXP,(-2*xtrue/param_a)),0.5);
                //uprintf("less!!! speed=%f\r\n",speed);
            }
        if(chassis_speed[target_point-1] == chassis_speed[target_point])
            {
                //speed = chassis_speed[target_point];
                //uprintf("same!!! speed=%f\r\n",speed);
            }
        if(chassis_speed[target_point - 1] > chassis_speed[target_point])
            {
                x0=param_a*(log(param_a/(param_a/2-chassis_speed[target_point]))-0.5*log((2*chassis_speed[target_point]+param_a)/(param_a-2*chassis_speed[target_point]))-log(2));
                xtrue = x0 - delt_dis_end*param_b;
                //speed=(param_a/2)*pow(1-pow(EXP,(-2*xtrue/param_a)),0.5);
                //printf("more!!! speed=%f\r\n",speed);
            }
        speed=(param_a/2)*pow(1-pow(EXP,(-2*xtrue/param_a)),0.5);
        return (int)speed;
} 


int chassis_calculate_speed1(int target_point)
{
        double speed = 0;
        double delt_dis_bgn = 0,delt_dis_end = 0;//存放进入上一个点起始速度对应的初始位移     
        //double angle = 0;
        //static double xtrue = 0;//速度位移曲线的自变量
        //double dis_betwin_two = chassis_dis_to_bgn[target_point];

        delt_dis_bgn = sqrt(pow(chassis.pos_x - chassis_xpos[target_point - 1] , 2) + pow(chassis.pos_y - chassis_ypos[target_point - 1] , 2));
        //uprintf("delt_dis_bgn = %f\r\n",delt_dis_bgn);
        delt_dis_end = sqrt(pow(chassis.pos_x - chassis_xpos[target_point] , 2) + pow(chassis.pos_y - chassis_ypos[target_point] , 2));

        if(chassis_speed[target_point-1] == 0)//从零启动的加速曲线
            {
                speed=(param_a/2)*pow(1-pow(EXP,(-2*delt_dis_bgn/param_b)),0.5)+200;
                if(speed>=chassis_speed[target_point])
                    speed=chassis_speed[target_point];
            }
        else if(chassis_speed[target_point] == 0)//目标点停
            {
                speed=(param_a/2)*pow(1-pow(EXP,(-2*delt_dis_end/param_b)),0.5);
                if(speed>=chassis_speed[target_point-1])
                    speed=chassis_speed[target_point-1];
            }
        else
            {
                speed = chassis_speed[target_point-1];
            }
        
        return (int)speed;
}

int chassis_calculate_speed2(double dis_to_nextpos)
{
    double x1 = chassis_dis_to_bgn[chassis_poscnt] - dis_to_nextpos;
    double x2 = chassis_dis_to_end[chassis_poscnt] + dis_to_nextpos;
    double speed = 0;
    if(x1 <= x2)
    {
         speed=(param_a/2)*pow(1-pow(EXP,(-2*x1/param_b)),0.5)+200;
         if(speed>=chassis_speed_max)
         {
             speed=chassis_speed_max ;
         }
    }
    else 
    {
        speed=(param_a/2)*pow(1-pow(EXP,(-2*x2/param_b)),0.5);
         if(speed>=chassis_speed_max)
         {
             speed=chassis_speed_max ;
         }
    }
    return (int)speed;
}

//底盘执行函数
void chassis_exe()
{
    if(1 == routeflag)
    {
        double deltaX = chassis.pos_x - chassis_xpos[chassis_poscnt];
        double deltaY = chassis.pos_y - chassis_ypos[chassis_poscnt];
        double dis_to_next = sqrt(deltaX * deltaX + deltaY * deltaY);
        //if(!chassis_withinpoint(chassis.pos_x, chassis.pos_y, chassis_xpos[chassis_poscnt], chassis_ypos[chassis_poscnt]))
        if((dis_to_next <= 0.005 && (chassis_poscnt <= 1 || chassis_poscnt >= chassis_posnum - 1)) || (dis_to_next <= 0.02 && chassis_poscnt > 1 && chassis_poscnt < chassis_posnum - 1) )
        {
            chassis_poscnt++;
            g_fturn -= 20 * PI / 180;
            if(chassis_poscnt >= chassis_posnum)
            {
                chassis_poscnt = 0;//环形轨迹
                g_ispeed = 0;
                g_fangle = 0;
                g_fturn = 0;
                routeflag = 0;
            }
        }
        else
        {
            double angle = atan2( chassis_ypos[chassis_poscnt] - chassis.pos_y,  chassis_xpos[chassis_poscnt] - chassis.pos_x);
            g_ispeed = chassis_calculate_speed2(dis_to_next);
            g_fangle = angle;
            //g_fturn = 0;
        }
    }
    chassis_gostraight(g_ispeed ,g_fangle, g_fturn);
}
void chassis_stop()
{
        start_flag=0;
        maxon_setSpeed(&MOTOR0_ID,0);
        maxon_setSpeed(&MOTOR3_ID,0);
        maxon_setSpeed(&MOTOR1_ID,0);

}