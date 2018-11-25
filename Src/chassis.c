#include "chassis.h"
//����ʵ����
Chassis chassis;
//������ȫ����λģ�鰲װƫ��
float ERR_angle_m3 = 0 , ERR_angle_m1 = 0 + 1*2*PI/3 , ERR_angle_m0 = 0 + 2*2*PI/3 ;
//�켣���е����
static int chassis_poscnt = 0;
//���߿�ʼ��־λ
static int routeflag = 0;
//�ֱ����ñ�־
static int handleflag = 0;
//�ֱ�����״̬
enum Handle_state{end, bgn}go_f, go_b, go_r, go_l, turn_l, turn_r;
//�ֱ������ĸ�����ļ���ʱ��
int ftime = 0, btime = 0, ltime = 0, rtime = 0;
//�������߲���
double param_a = 0, param_b = 0; 
//������ĳһ������뿪ͷ�ͽ�β�ľ�������
double chassis_dis_to_bgn[NUM_POINTS];
double chassis_dis_to_end[NUM_POINTS];
//������תpid����
float chassis_turn_angle_KP = -1500;
float chassis_turn_angle_KD = 0;

float quickturn_KP = 0;
float quickturn_KD = 0;

//�ٶ� ����� ��ת��λ��
int g_ispeed = 0;
float g_fangle = 0;
float g_fturn = 0;

/**���̳�ʼ��
*��������
*����ֵ�� ��
*/
void chassis_init(void)
{
    can_add_callback(325,chassis_handle);
    track_init();
    chassis_calculate_dis_matrix();
    vega_init(&(chassis.g_vega_pos_x), &(chassis.g_vega_pos_y), &(chassis.g_vega_angle));
    vega_reset();

    HAL_Delay(2000);//��ʱ�ȴ���λ
    vega_set_pos(0, 0);
    maxon_setSpeed(&MOTOR0_ID, 0);
    maxon_setSpeed(&MOTOR3_ID, 0);
    maxon_setSpeed(&MOTOR1_ID, 0);
    uprintf("chassis and vega are ready!!!\r\n");
}
/**���̸�������
*��������
*����ֵ�� ��
*/
void chassis_update(void)
{
    chassis.pos_x = chassis.g_vega_pos_x * 0.0001 * 0.81;
    chassis.pos_y = - chassis.g_vega_pos_y * 0.0001 * 0.81;//����ط�ȫ����λy���ˣ�ע�⣡������
    chassis.angle = - (chassis.g_vega_angle / 180.f) * PI;
}

/*
�Ƕȼ�������
*/
float chassis_angle_subtract(float a, float b)
{
    float out = a - b;
    while(out > PI)
	{
		out -= 2 * PI;
	}
	while(out < - PI)
	{
		out += 2 * PI;
	}
    return out;
}

/*
�Ƕ�pid����
*/
float chassis_PID_Angle_Control(float target_angle){
  
  float angle_err = chassis_angle_subtract(target_angle, chassis.angle); 
  static float angle_last_err = 0;
  float P_out = angle_err * chassis_turn_angle_KP;
  float D_out = (angle_last_err - angle_err) * chassis_turn_angle_KD;
  angle_last_err = angle_err;
  return P_out + D_out;
}

float chassis_quickturn_pid(float target_angle, float now_angle)
{
    
    float err = chassis_angle_subtract(target_angle, now_angle); 
    static float last_err = 0;
    float P_out = err * quickturn_KP;
    float D_out = (last_err - err) * quickturn_KD;
    last_err = err;
    return P_out + D_out;
}

/**��������
*������float angle 	�����
*      int   speed    �ٶ�
        float turn  ��ת��λ��
*����ֵ�� ��
*/
void chassis_gostraight(int speed , float angle, float turn)
{
    
    static float lastxpos = 0;
    static float lastypos = 0;
    float now_angle = atan2(chassis.pos_y - lastypos, chassis.pos_x - lastxpos);
    if(chassis.pos_y == lastypos && chassis.pos_x == lastxpos) now_angle = angle;
    lastxpos = chassis.pos_x;
    lastypos = chassis.pos_y;
    float quickturn_output = chassis_quickturn_pid(angle, now_angle);  
    
    if(quickturn_output > PI) quickturn_output = PI;
    if(quickturn_output < -PI) quickturn_output = -PI;
    
    float Chassis_motor0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - angle - quickturn_output));
    float Chassis_motor1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - angle - quickturn_output));
    float Chassis_motor3 = -(speed*cos((ERR_angle_m3 + chassis.angle) - angle - quickturn_output));
    /*
    float Chassis_motor0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - angle));
    float Chassis_motor1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - angle));
    float Chassis_motor3 = -(speed*cos((ERR_angle_m3 + chassis.angle) - angle));    */
    float turn_output = chassis_PID_Angle_Control(turn);
    if(turn_output >2000)//�Ƕ�pid�޷�
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
//�������
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

/*�ٶ����߼���
���뵽��һ�����ʵ�ʾ���
���ش˿��ٶ�ֵ
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

//����ִ�к���
void chassis_exe()
{
    chassis_update();//���������
    if(1 == routeflag)//�Թ켣����ʽ����
    {
        double deltaX = chassis.pos_x - chassis_xpos[chassis_poscnt];
        double deltaY = chassis.pos_y - chassis_ypos[chassis_poscnt];
        double dis_to_next = sqrt(deltaX * deltaX + deltaY * deltaY);
        if((dis_to_next <= 0.005 && ( chassis_poscnt >= chassis_posnum - 1)) 
            || (dis_to_next <= 0.05 && chassis_poscnt >= 0 && chassis_poscnt < chassis_posnum - 1) )//�жϾ����˵�
        {
            chassis_poscnt++;
            g_fturn = chassis_turn[chassis_poscnt];//-= 20 * PI / 180;
            
        }
            double angle = atan2( chassis_ypos[chassis_poscnt] - chassis.pos_y,  chassis_xpos[chassis_poscnt] - chassis.pos_x);
            g_ispeed = chassis_calculate_speed(dis_to_next);
            g_fangle = angle;
            if(chassis_poscnt >= chassis_posnum)//����Ŀ�ĵ�
            {
                chassis_poscnt = 0;
                g_ispeed = 0;
                g_fangle = 0;
                g_fturn = 0;
                routeflag = 0;
            }
       
    }
    if(handleflag == 1)//���ֱ�����ʽ����
    {
        chassis_handle_control();
    }
    chassis_gostraight(g_ispeed ,g_fangle, g_fturn);
}

//���ߺ���
float chassis_handle_get_speed(int time)
{
    return (2 / (1 + exp(-0.05 * time)) - 1) * 5000;
}

//�ٶ����߼��㺯��
//timeΪ��ʱ���۵ļ���ʱ��
//ÿ�μ��پͻ��ۼ���ʱ�䣬����ʱ�Ѽ���ʱ�����0Ϊֹ
float chassis_handle_speed_control(enum Handle_state go, int* time)
{
    float speed = chassis_handle_get_speed(*time);
    if(speed < chassis_speed_max && go == bgn)
    {
        (*time)++;
    }
    else if(speed > 0 && go == end)
    {
        (*time)--;
    }
    
    if(speed > chassis_speed_max) speed = chassis_speed_max;
    if(speed < 0) speed = 0;
    return speed;
}

//�ֱ�����
void chassis_handle_control() 
{
    float fspeed, bspeed, lspeed, rspeed;
    fspeed = chassis_handle_speed_control(go_f, &ftime);
    bspeed = chassis_handle_speed_control(go_b, &btime);
    lspeed = chassis_handle_speed_control(go_l, &ltime);
    rspeed = chassis_handle_speed_control(go_r, &rtime);
    
    g_fangle = atan2(fspeed - bspeed, rspeed - lspeed);//������Է����
    g_ispeed = (int)(abs(fspeed - bspeed) + abs(rspeed - lspeed));//�ٶȵ���
    if(g_ispeed > chassis_speed_max) g_ispeed = chassis_speed_max;
    if(g_ispeed < 0) g_ispeed = 0;
    
    if(turn_l == bgn)//��תÿ����ת1��
    {
        g_fturn += 0.0174;
    }
    if(turn_r == bgn)
    {
        g_fturn -= 0.0174;
    }
}

//�ֱ�can��id��325��10��
//�˺�������can��������Ϣ
void chassis_handle(CanRxMsgTypeDef* pRxMsg)
{
    if(0 == main_flag) return;
    uint8_t Data[8];
    int i;
    for(i = 0; i < 8; i++)
    {
        Data[i] = pRxMsg->Data[i];
    }
    if(Data[0] == 'f')
    {
        if(Data[1] == 'b')
        {
            uprintf("fb\r\n");//��ǰ����
            go_f = bgn;
        }
        else if(Data[1] == 'e')//��ǰ�ɿ�
        {
            uprintf("fe\r\n");
            go_f = end;
        }
    }
    else if(Data[0] == 'r')
    {
        if(Data[1] == 'b')//��������
        {
            uprintf("rb\r\n");
            go_r = bgn;
        }
        else if(Data[1] == 'e')//�����ɿ�
        {
            uprintf("re\r\n");
            go_r = end;
        }
    }
    else if(Data[0] == 'l')
    {
        if(Data[1] == 'b')//��������
        {
            uprintf("lb\r\n");
            go_l = bgn;
        }
        else if(Data[1] == 'e')//�����ɿ�
        {
            uprintf("le\r\n");
            go_l = end;
        }
    }
    else if(Data[0] == 'b')
    {
        if(Data[1] == 'b')
        {
            uprintf("bb\r\n");//�������
            go_b = bgn;
        }
        else if(Data[1] == 'e')//����ɿ�
        {
            uprintf("be\r\n");
            go_b = end;
        }
    }
    else if(Data[0] == 's' && Data[1] == 't')//stop����
    {
            uprintf("st\r\n");
            ftime = 0;
            btime = 0;
            ltime = 0;
            rtime = 0;
            go_f = end;
            go_b = end;
            go_r = end;
            go_l = end;
            turn_l = end;
            turn_r = end;
    }
    else if(Data[0] == 't')
    {
         if(Data[1] == 'l')
        {
               //���Ǽ�
            if(Data[2] == 'b')//����ת����
            {
                uprintf("tlb\r\n");
                turn_l = bgn;
            }
            else if(Data[2] == 'e')//����ת�ɿ�
            {
                uprintf("tle\r\n");
                turn_l = end;
            }
        }
        else if(Data[1] == 'r')
        {
            //���Ǽ�
            if(Data[2] == 'b')
            {
                uprintf("trb\r\n");//����ת����
                turn_r = bgn;
            }
            else if(Data[2] == 'e')//����ת�ɿ�
            {
                uprintf("tre\r\n");
                turn_r = end;
            }
        }
    }
    else if(Data[0] == '0')//�ֱ�ģʽ����
    {
        uprintf("0\r\n");
        handleflag = 1;
        routeflag = 0;
    }
    else if(Data[0] == '1')
    {
        chassis_speed_max += 300;//��������ٶ�
    }
    else if(Data[0] == '3')
    {
        chassis_speed_max -= 300;//��С����ٶ�
        if(chassis_speed_max < 0) chassis_speed_max = 0;
    }   
}