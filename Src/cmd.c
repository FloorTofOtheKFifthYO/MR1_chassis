/*******************************************************************************
Copyright:      2018/12/18
File name:      cmd.c
Description:    ��Ŵ������cmd��ʼ����ʹ��ǰ�����ʼ������ռ䣩��cmd������
                cmdִ�к�����cmd��������
Author:         ����Զ
Version��       1.0
Data:           2018/12/18 22:36
History:        ��
Bug:            ��
*******************************************************************************/
#include "cmd.h"
static cmd_struct cmd_tbl[] = {
    /*
     *�������Ľṹ��
     * ������������Ҫ�ڴ˼��ϣ�
     * CMD_ADD("������","����ʹ�÷�������Ϊ�ո�,�����ܲ���˫���ţ�",��Ӧ�����ִ�к�����)
     * ע�����һ������Ҫ���ţ�ǰ��Ķ���Ҫ����
     */
    CMD_ADD("help","������eg��help",cmd_help_func),
    CMD_ADD("hello","�ʺá�eg:hello",cmd_hello_func),
    CMD_ADD("stop","��ͣ��eg:stop",cmd_stop_func),
    CMD_ADD("go_straight","ĳ�Ƕ�ֱ�С�eg:go_straight speed angle",cmd_go_straight_func),
    CMD_ADD("go_route","��·�����ߡ�eg:go_route 1 5000 1 2500",cmd_go_route_func),
    CMD_ADD("reset_vega","��λȫ����λ.eg:reset_vega",cmd_reset_vega),
    CMD_ADD("modify_angle","�ı䷽�����ǡ�eg��modify_angle 30 30",cmd_modify_angle),
    CMD_ADD("angle_pid","������ת��pid",cmd_angle_pid),
    CMD_ADD("print_pos","��ӡȫ����λλ��",cmd_print_pos),
    CMD_ADD("go_point","ȥĿ���",cmd_go_targetpoint),
    CMD_ADD("read","������������ֵ",cmd_read_distance),
    CMD_ADD("lock_x_y_angle","����x y angle",cmd_lock_x_y_angle)
};
char cmd_line[MAX_CMD_LINE_LENGTH + 1];
char *cmd_argv[MAX_ARGC]; 
    
void cmd_init()
{
    for(int i = 0;i < MAX_ARGC;i++){
        cmd_argv[i] = (char *)malloc(MAX_CMD_ARG_LENGTH + 1);//��ȷ���������ݵ��ڴ�ռ䣬���Է���һ��
    }
}
/*
*���������
*/
int cmd_parse(char *cmd_line,int *argc,char *argv[]){
    char c_temp;
    int i = 0,arg_index = 0;
    int arg_cnt = 0;
    c_temp = cmd_line[i++];  
    while(c_temp != '\r'){
        if(c_temp == ' '){
            if(arg_index == 0){   //���������߲����ַ�����һ���ǿո������   
                c_temp = cmd_line[i++];
                continue;
            }
            //�ո�Ϊ������������ķָ���
            if(arg_cnt == MAX_ARGC){   //���������������,�򷵻�
                return -1;
            }
            argv[arg_cnt][arg_index] = 0;
            arg_cnt++;
            arg_index = 0;
            c_temp = cmd_line[i++];
            continue;
        }
        if(arg_index == MAX_CMD_ARG_LENGTH){   //����������ȹ������򱨴���
            return -2;
        }
        argv[arg_cnt][arg_index++] = c_temp;
        c_temp = cmd_line[i++];
    }
    if(arg_cnt == 0 && arg_index == 0){  //���������߲����ǿյģ��򷵻�
        return -3;
    }
    //���һ�������Ľ���û���������whileѭ���н�����
    argv[arg_cnt++][arg_index] = 0;
    *argc = arg_cnt;//������
    return 0;
}

int cmd_exec(int argc,char *argv[]){
    int cmd_index = 0;
    uint32_t cmd_num;
 
    cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);

    if(argc == 0){  //��������ǿյģ��򷵻�
        return -1;
    }
    for(cmd_index = 0;cmd_index < cmd_num;cmd_index++){   //��������
        if(strcmp((char *)(cmd_tbl[cmd_index].cmd_name),(char *)argv[0]) == 0){  //����ҵ��������ִ���������Ӧ�ĺ���
            cmd_tbl[cmd_index].cmd_func(argc,argv);
            memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
            return 0;
        }
    }
    return -2;
}

void cmd_help_func(int argc,char *argv[]){
     int i;
    uint32_t cmd_num;
    cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
    if(argc > 1){
        uprintf(CMD_USART,"msg:\n help�����������\n\n");      
        return;         
    }
    for(i = 0;i < cmd_num;i++){
        uprintf(CMD_USART,"cmd:%s\nusage:%s\n\n",cmd_tbl[i].cmd_name,cmd_tbl[i].cmd_usage);
    }
}