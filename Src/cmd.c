#include "cmd.h"
static cmd_struct cmd_tbl[] = {
    /*
     *存放命令的结构体
     * 若需添加命令，需要在此加上：
     * CMD_ADD("命令名","命令使用方法（可为空格,但不能不加双引号）",对应命令的执行函数名)
     * 注意最后一个不需要逗号，前面的都需要逗号
     */
    CMD_ADD("help"," Print all command and usage ",cmd_help_func),
    CMD_ADD("hello"," ",cmd_hello_func),
    CMD_ADD("set_speed","设置电机速度",cmd_set_speed_func),
    CMD_ADD("stop","急停",cmd_stop_func),
    CMD_ADD("go_straight","某角度直行",cmd_go_straight_func),
    CMD_ADD("go_route","按路径行走",cmd_go_route_func),
    CMD_ADD("reset_vega","复位全场定位",cmd_reset_vega),
    CMD_ADD("modify_angle","改变方向解算角",cmd_modify_angle),
    CMD_ADD("angle_pid","底盘自转角pid",cmd_angle_pid)
};
char cmd_line[MAX_CMD_LINE_LENGTH + 1];
char *cmd_argv[MAX_ARGC]; 
    
void cmd_init()
{
    for(int i = 0;i < MAX_ARGC;i++){
        cmd_argv[i] = (char *)malloc(MAX_CMD_ARG_LENGTH + 1);//不确定输入数据的内存空间，所以分配一块
    }
}
/*
*解析命令函数
*/
int cmd_parse(char *cmd_line,int *argc,char *argv[]){
    char c_temp;
    int i = 0,arg_index = 0;
    int arg_cnt = 0;
    c_temp = cmd_line[i++];  
    while(c_temp != '\r'){
        if(c_temp == ' '){
            if(arg_index == 0){   //如果命令或者参数字符串第一个是空格，则忽略   
                c_temp = cmd_line[i++];
                continue;
            }
            //空格为参数或者命令的分隔符
            if(arg_cnt == MAX_ARGC){   //如果参数个数过多,则返回
                return -1;
            }
            argv[arg_cnt][arg_index] = 0;
            arg_cnt++;
            arg_index = 0;
            c_temp = cmd_line[i++];
            continue;
        }
        if(arg_index == MAX_CMD_ARG_LENGTH){   //如果参数长度过长，则报错返回
            return -2;
        }
        argv[arg_cnt][arg_index++] = c_temp;
        c_temp = cmd_line[i++];
    }
    if(arg_cnt == 0 && arg_index == 0){  //如果命令或者参数是空的，则返回
        return -3;
    }
    //最后一个参数的结束没有在上面的while循环中解析到
    argv[arg_cnt++][arg_index] = 0;
    *argc = arg_cnt;//命令数
    return 0;
}

int cmd_exec(int argc,char *argv[]){
    int cmd_index = 0;
    uint32_t cmd_num;
 
    cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);

    if(argc == 0){  //如果参数是空的，则返回
        return -1;
    }
    for(cmd_index = 0;cmd_index < cmd_num;cmd_index++){   //查找命令
        if(strcmp((char *)(cmd_tbl[cmd_index].cmd_name),(char *)argv[0]) == 0){  //如果找到了命令，则执行命令相对应的函数
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
        uprintf("msg:\n help命令参数过多\n\n");      
        return;         
    }
    for(i = 0;i < cmd_num;i++){
        uprintf("cmd:%s\nusage:%s\n\n",cmd_tbl[i].cmd_name,cmd_tbl[i].cmd_usage);
    }
}