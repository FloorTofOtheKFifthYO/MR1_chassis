#include "vavle.h"

void vavle_open(uint8_t id)
{
    switch (id)
    {
      case 1:
        can_send_msg(0x77,"open1",5);
        break;
      case 2:
        can_send_msg(0x77,"open2",5);
        break;
      case 3:
        can_send_msg(0x77,"open3",5);
        break;
      default:
        break;
    }
}

void vavle_close(uint8_t id)
{
    switch (id)
    {
      case 1:
        can_send_msg(0x77,"close1",6);
        break;
      case 2:
        can_send_msg(0x77,"close2",6);
        break;
      case 3:
        can_send_msg(0x77,"close3",6);
        break;
      default:
        break;
    }
}

void vavle_control()
{
    static int ctrl_num = 0;
    switch(ctrl_num)
    {
    case 0:
        vavle_open(1);
        break;
    case 1:
        vavle_open(2);
        break;
    case 2:
        vavle_open(3);
        break;
    case 3:
        vavle_close(1);
        break;
    case 4:
        vavle_close(2);
        break;
    case 5:
        vavle_close(3);
        break;
    }
    ctrl_num = (ctrl_num + 1) % 6;
}