/*文件：贝塞尔曲线
* 说明：外部调用track_bezier_init（）即可把贝塞尔曲线上的点传到底盘
*将要走的坐标上，修改输入的控制点在track.c中
*/

#include "bezier.h"
#include "math.h"
#include "assert.h" 

PointF out[out_num];    // 输出点数组

PointF ctrl_points[input_num*2-4];//控制点 

PointF combine_points[4];//二阶贝塞尔曲线辅助点


/*
*说明：计算两个控制点之间的插样点
*/
PointF bezier_interpolation_func(float t, PointF* points, int count)
{
    assert(count>0);
    
    PointF tmp_points[200];//3000随便给的，大于两个点之间的采样点就可以，iar不能传变量给数组
    for (int i = 1; i < count; ++i)
    {
        for (int j = 0; j < count - i; ++j)
        {
            if (i == 1)
            {
                tmp_points[j].X = (float)(points[j].X * (1 - t) + points[j + 1].X * t);
                tmp_points[j].Y = (float)(points[j].Y * (1 - t) + points[j + 1].Y * t);
                continue;
            }
            tmp_points[j].X = (float)(tmp_points[j].X * (1 - t) + tmp_points[j + 1].X * t);
            tmp_points[j].Y = (float)(tmp_points[j].Y * (1 - t) + tmp_points[j + 1].Y * t);
        }
    }
    return tmp_points[0];
}

/*
*说明：根据插值点计算轨迹贝塞尔曲线轨迹点
*/
void draw_bezier_curves(PointF* points, int count, PointF* out_points,int out_count,int start)
{
    float step = 1.0 / out_count;
    float t =0;
    for(int i=start; i<(start+out_count); i++)
    {
        PointF temp_point = bezier_interpolation_func(t, points, count);    // 计算插值点
        t += step;
        out_points[i] = temp_point;
    }
}

/*
*说明：计算两点距离
*/
float dis_btw_points(PointF P1,PointF P2)
{
    return sqrt((P1.X-P2.X)*(P1.X-P2.X)+(P1.Y-P2.Y)*(P1.Y-P2.Y));
}

/*
*说明：产生辅助控制点，控制二阶贝塞尔曲线连续
*/
void bezier_produce_ctrlpoints(PointF* points)//传入路径上控制点数组和个数 
{
    PointF p1,p2,p3;
    float l1,l2;
    for(int i = 0;i < (input_num-2);i++)
    {
        p1.X = (points[i].X + points[i+1].X)/2;
        p1.Y = (points[i].Y + points[i+1].Y)/2;
        p2.X = (points[i+1].X + points[i+2].X)/2;
        p2.Y = (points[i+1].Y + points[i+2].Y)/2;//
        
        l1 = dis_btw_points(points[i],points[i+1]);
        l2 = dis_btw_points(points[i],points[i+1]);
        
        p3.X = (l1*p2.X+l2*p1.X)/(l1+l2);
        p3.Y = (l1*p2.Y+l2*p1.Y)/(l1+l2);
        
        ctrl_points[2*i].X = (p1.X - p3.X) * K + points[i+1].X;
        ctrl_points[2*i].Y = (p1.Y - p3.Y) * K + points[i+1].Y ;
        ctrl_points[2*i+1].X = (p2.X - p3.X) * K + points[i+1].X ;
        ctrl_points[2*i+1].Y = (p2.Y - p3.Y) * K + points[i+1].Y ;
    }
    
}
/*
*说明：贝塞尔曲线初始化，传入数组in写在track.c，传出到out数组，
*/
void track_bezier_init()
{
    bezier_produce_ctrlpoints(in);
    
    for(int j = 0;j<=(input_num-2);j++)
    {
        if(j==0)//第一段折线 
        {
            combine_points[0] = in [0];
            combine_points[1] = ctrl_points[0];
            combine_points[2] = in [1]; 
            draw_bezier_curves(combine_points,3,out,num,0);// 二阶贝塞尔曲线
        }
        else if(j==input_num-2)
        {
            combine_points[0] = in [j];
            combine_points[1] = ctrl_points[input_num*2-5];
            combine_points[2] = in [j+1]; 
            draw_bezier_curves(combine_points,3,out,num,num*input_num-2*num);// 二阶贝塞尔曲线
        }else 
        {
            combine_points[0] = in [j];
            combine_points[1] = ctrl_points[2*j-1];
            combine_points[2] = ctrl_points[2*j];
            combine_points[3] = in [j+1];	
            draw_bezier_curves(combine_points,4,out,num,num*j);		
        }
        
    }
    	//以下打印为了给matlab生成m文件用，可以忽略
//    uprintf("close all;\r\n");
//    uprintf("x=[\r\n");
//    for(int j=0; j<input_num; j++)    // 输出控制点x 
//    {
//		uprintf("%f ",in[j].X);
//    }
//    uprintf("];\r\n");
//    uprintf("y=[\r\n");
//    for(int j=0; j<input_num; j++)    // 输出控制点y 
//    {
//		uprintf("%f ",in[j].Y);
//    }
//    uprintf("];\r\n");
//    
//    uprintf("plot(x,y,':');\r\n");
//    uprintf("hold on;\r\n");
//    
//    uprintf("x=[\r\n");
//    for(int j=0; j<(input_num*2-4); j++)    // 输出控制点x 
//    {
//		uprintf("%f ",ctrl_points[j].X);		
//    }
//    uprintf("];\r\n");
//    uprintf("y=[\r\n");
//    for(int j=0; j<(input_num*2-4); j++)    // 输出控制点y 
//    {
//		uprintf("%f ",ctrl_points[j].Y);
//    }
//    uprintf("];\r\n");
//    
//    uprintf("plot(x,y,':');\r\n");
//    uprintf("hold on;\r\n");
//    uprintf("x=[\r\n");
//    for(int j=0; j<out_num-2; j++)    // 输出路径点
//    {
//    	if(j!=0)
//        	uprintf("%f ",out[j].X);
//        if(j%10==0&&j!=0)
//        	uprintf("\r\n");
//    }
//    uprintf("];\r\n");
//    uprintf("y=[\r\n");
//    for(int j=0; j<out_num-2; j++)    // 输出路径点
//    {
//    	if(j!=0)
//        	uprintf("%f ",out[j].Y);
//        if(j%10==0&&j!=0)
//        	uprintf("\r\n");
//    }
//    uprintf("];\r\n");
//	  
//	uprintf("plot(x,y,'r');");
//    
//    for(int j = 0;j<out_num;j++)
//    {
//        chassis_xpos[j]=out[j].X;
//        chassis_ypos[j]=out[j].Y;
//    }
}