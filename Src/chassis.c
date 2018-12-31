#include "chassis.h"
Chassis chassis;
//三轮与全场定位模块安装偏角
float ERR_angle_m3 = -PI/3 , ERR_angle_m1 = -PI/3 + 1*2*PI/3 , ERR_angle_m0 = -PI/3 + 2*2*PI/3 ;
static int chassis_poscnt = 0;//点计数
//曲线开始标志位
static int routeflag = 0;
static int handleflag = 0;
//手柄按键状态
double param_a = 0, param_b = 0; 
//曲线中某一个点距离开头和结尾的距离数组
double chassis_dis_to_bgn[NUM_POINTS];
double chassis_dis_to_end[NUM_POINTS];

//底盘自转pid参数
float chassis_turn_angle_KP = -1500;
float chassis_turn_angle_KD = 0;

//速度 方向角 自转方位角
int g_ispeed = 0;
float g_fangle = 0;
float g_fturn = 0;

int rocker_x = 0;
int rocker_y = 0;
int rocker_z = 0;


/**底盘初始化
*参数：无
*返回值： 无
*/
void chassis_init(void)
{
    can_add_callback(325,chassis_handle);
    can_add_callback(324,chassis_rocker);
  track_init();
  chassis_calculate_dis_matrix();
  vega_action_init();
  
  maxon_setSpeed(&MOTOR0_ID, 0);
  maxon_setSpeed(&MOTOR3_ID, 0);
  maxon_setSpeed(&MOTOR1_ID, 0);
  uprintf(CMD_USART,"chassis and vega are ready!!!\r\n");
}
/**底盘更新坐标
*参数：无
*返回值： 无
*/
void chassis_update(void)
{
  //chassis.pos_x = chassis.g_vega_pos_x * 0.0001 * 0.81;
  //chassis.pos_y = - chassis.g_vega_pos_y * 0.0001 * 0.81;//这个地方全场定位y反了，注意！！！！
  //chassis.angle = - (chassis.g_vega_angle / 180.f) * PI;
    chassis.pos_x = chassis.g_vega_pos_x/1000;
    chassis.pos_y = chassis.g_vega_pos_y/1000;
    chassis.angle = (chassis.g_vega_angle / 180.f) * PI;

}

/*
角度减法函数
*/
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

/*
角度pid控制
*/
float chassis_PID_Angle_Control(float target_angle){
  
  float angle_err = chassis_angle_subtract(target_angle, chassis.angle); 
  static float angle_last_err = 0;
  float P_out = angle_err * chassis_turn_angle_KP;
  float D_out = (angle_last_err - angle_err) * chassis_turn_angle_KD;
  angle_last_err = angle_err;
  return P_out + D_out;
}

/**底盘驱动
*参数：float angle 	方向角
*      int   speed    速度
float turn  自转方位角
*返回值： 无
*说明:
*/
void chassis_gostraight(int speed , float angle, float turn)
{
  float Chassis_motor0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - angle));
  float Chassis_motor1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - angle));
  float Chassis_motor3 = -(speed*cos((ERR_angle_m3 + chassis.angle) - angle));
  
  float turn_output = chassis_PID_Angle_Control(turn);
  if(turn_output >2000)//角度pid限幅
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
  uprintf(CMD_USART,"go_route flag = %d \r\n",flag);
}
//计算距离
void chassis_calculate_dis_matrix()
{
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

/*速度曲线计算
传入到下一个点的实际距离
返回此刻速度值
*/
int chassis_calculate_speed(double dis_to_nextpos)
{
  double x1 = chassis_dis_to_bgn[chassis_poscnt] - dis_to_nextpos;
  double x2 = chassis_dis_to_end[chassis_poscnt] + dis_to_nextpos;
  double speed = 0;
  if(x1 <= x2)
  {
    speed = (param_a / 2) * pow(1 - pow(EXP, (-2 * x1 / param_b)), 0.5) + 300;
    if(speed >= chassis_speed_max)
    {
      speed = chassis_speed_max;
    }
  }
  else 
  {
    speed = (param_a / 2) * pow(1 - pow(EXP, (-2 * x2 / param_b)), 0.5);
    if(speed >= chassis_speed_max)
    {
      speed = chassis_speed_max;
    }
  }
  return (int)speed;
}

//底盘执行函数
void chassis_exe()
{
  chassis_update();
  if(1 == routeflag)
  {
    double deltaX = chassis.pos_x - chassis_xpos[chassis_poscnt];
    double deltaY = chassis.pos_y - chassis_ypos[chassis_poscnt];
    double dis_to_next = sqrt(deltaX * deltaX + deltaY * deltaY);
    if((dis_to_next <= 0.005 && ( chassis_poscnt >= chassis_posnum - 1)) 
       || (dis_to_next <= 0.03 && chassis_poscnt >= 0 && chassis_poscnt < chassis_posnum - 1) )//判断经过此点
    {
      chassis_poscnt++;
      
      uprintf(CMD_USART,"chassis_poscnt=%d\r\n",chassis_poscnt);
      
      g_fturn = chassis_turn[chassis_poscnt];//-= 20 * PI / 180;
      if(chassis_poscnt >= chassis_posnum)//到达目的地
      {
        chassis_poscnt = 0;
        g_ispeed = 0;
        g_fangle = 0;
        g_fturn = 0;
        routeflag = 0;
      }
    }
    else
    {
      double angle = atan2( chassis_ypos[chassis_poscnt] - chassis.pos_y,  chassis_xpos[chassis_poscnt] - chassis.pos_x);
      g_ispeed = chassis_calculate_speed(dis_to_next);
      g_fangle = angle;

          //uprintf(CMD_USART,"g_ispeed=%f    ",g_ispeed);
          //uprintf(CMD_USART,"g_fbngle=%f\r\n",g_fangle);
      
      //uprintf(CMD_USART,"g_fangle=%f\r\n",g_fangle);
    }
    //chassis_gostraight(g_ispeed ,g_fangle, g_fturn);  
  }
  if(handleflag == 1)//以手柄的形式运行
    {
        chassis_handle_control();
    }
    chassis_gostraight(g_ispeed ,g_fangle, g_fturn); //放在上面的括号里了
}

//手柄can的按键id是325（10）
//此函数接收can发来的消息
void chassis_handle(CanRxMsgTypeDef* pRxMsg)
{
    if(0 == main_flag) return;
    uint8_t Data[8];
    int i;
    for(i = 0; i < 8; i++)
    {
        Data[i] = pRxMsg->Data[i];
    }
    
    if(Data[0] == '0')//手柄模式启动
    {
        handleflag = 1;
        routeflag = 0;
        chassis_speed_max += 300;//增加最大速度
    }
    else if(Data[0] == '1')
    {
        chassis_speed_max -= 300;//减小最大速度
        if(chassis_speed_max < 0) chassis_speed_max = 0;
    }
    else if(Data[0] == 'l')//2号按键
    {
        if(Data[2] == 'b')
        {
            can_send_msg(322,"1",1);
        }
        else if(Data[2] == 'e')
        {
            can_send_msg(322,"2",1);
        }
    }
    else if(Data[0] == 'r')//3号按键
    {
        if(Data[2] == 'b')
        {
            can_send_msg(322,"3",1);
        }
        else if(Data[2] == 'e')
        {
            can_send_msg(322,"4",1);
        }
    }  
    else if(Data[0] == '4')
    {
        vavle_control();
    }
    else if(Data[0] == 't')
    {
        if(Data[2] == 'b')
        {
            can_send_msg(320,"1",1);
        }
        else if (Data[2] == 'e')
        {
            can_send_msg(320,"2",1);
        }
    }
}

//摇杆的can的id为324
void chassis_rocker(CanRxMsgTypeDef* pRxMsg)
{
    if(0 == main_flag) return;
    uint8_t Data[8];
    int i;
    for(i = 0; i < 8; i++)
    {
        Data[i] = pRxMsg->Data[i];
    }
    rocker_x = (int)(Data[0]*128 + Data[1]);
    rocker_z = (int)(Data[2]*128 + Data[3]);
    rocker_y = (int)(Data[4]*128 + Data[5]);
    
    rocker_x -= 128;
    rocker_z -= 128;
    rocker_y -= 128;
    rocker_x = -rocker_x;
    rocker_y = -rocker_y;
    
    if (abs(rocker_x) < 10) rocker_x = 0;
    if (abs(rocker_z) < 10) rocker_z = 0;
    if (abs(rocker_y) < 10) rocker_y = 0;
}

void chassis_handle_control() 
{   
    g_fangle = atan2(rocker_y, rocker_x);//计算绝对方向角
    float dis = sqrt(rocker_y * rocker_y + rocker_x * rocker_x);
    g_ispeed = (int)(chassis_speed_max * dis * dis / 65536);//速度叠加
    if(g_ispeed > chassis_speed_max) g_ispeed = chassis_speed_max;
    if(g_ispeed < 0) g_ispeed = 0;
    if(rocker_z > 0)
        g_fturn -= 0.0522 * rocker_z * rocker_z / 65536;
    else
        g_fturn += 0.0522 * rocker_z * rocker_z / 65536;
}