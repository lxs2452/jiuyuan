#include "file.h"
uint8_t iiiii;
uint8_t sta=0;
int falg=0;
uint8_t type=0;
uint8_t agree[9];
uint8_t agree1[9];
uint8_t agree2[9];
uint8_t p;
uint8_t p1=0;
float pitch1,roll1,yaw2;
float yaw_g;

void jy61p_pack(uint8_t data)
{
	if(data==0x55&&sta==0)
	{
		sta=1;
		iiiii++;
//		return;
	}
	else if(sta==1)
	{
		if(data == 0x53)
            type = 0, sta = 2;// 角度
        else if(data == 0x51)
            type = 1, sta = 2;// 加速度
        else if(data == 0x52)
            type = 2, sta = 2;// 角速度
        else if(data == 0x5A)
            type = 3, sta = 2;// 角度
		else if(data == 0x54)
            type = 4, sta = 2;// 角度
		else if(data == 0x55)
            type = 5, sta = 2;// 角度
		else if(data == 0x56)
            type = 6, sta = 2;// 角度
		else if(data == 0x57)
            type = 7, sta = 2;// 角度
		else if(data == 0x58)
            type = 8, sta = 2;// 角度
        else if(data == 0x59)
            type = 9, sta = 2;// 四元数
		 else if(data == 0x5F)
            type = 10, sta = 2;// 四元数
        else
            sta=0;
	}else if(sta==2)
	{
		switch (type)
		{
			case 0:
			{
				agree[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree[8]=1;
				}	
			}
			break;
			case 2:
			{
				agree2[p1]=data;
				p1++;
				if(p1>=7)
				{
					p1=0;
					type=0xff;
					sta=3;
					agree2[8]=1;
				}	
			}
			break;
			case 1:
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
			case 3:
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
			case 4:
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
			case 5:
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
			case 6:
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
			case 7:
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
			case 8:
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
			case 9:
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
			case 10 :
			{
				agree1[p]=data;
				p++;
				if(p>=7)
				{
					p=0;
					type=0xff;
					sta=3;
					agree1[8]=1;
				}	
			}
			break;
				
		}
	}else if(sta==3)
	{
		sta=0;
	}
	yaw2=((YAWH)<<8|YAWL)/32768.00f*180.0;//角度
	yaw_g=(float)((agree2[5])<<8|agree2[4])/32768.00f*2000;
	if(yaw_g>2000)//角速度
	{
		yaw_g=4000.00-yaw_g;
		yaw_g=(-1)*yaw_g;
	}
//	roll1=((ROLLH)<<8|YAWL)/32768.00f*180.0;
//	pitch1=((PICH)<<8|PICL)/32768.00f*180.0;
}



float yaw_v;//为z轴的角速度
float yaw_kp=6;
float yaw_kd=-0.05;
float yaw_kd;
float goal_yaw;
int F_R_Speed=0;
int F_L_Speed=0;
float yaw_err;



int turn_jy61p_yaw(float goal)//右转度数为0到360；左转度数为360到0。这里的goal为目标转弯角度，当角度为正的时向右转，角度为负的时，为向左转。
{
	yaw_err=goal-yaw2;
	if(yaw_err>=240)//最短距离
	{
		yaw_err=yaw2+(360.0f-goal);
		yaw_err=(-1.0)*yaw_err;
	}
	if(yaw_err<=-240)
	{
		yaw_err=(360.0-yaw2)+goal;
	}
	
	F_R_Speed=yaw_kp*yaw_err;//两个轮原地转
	F_L_Speed=-yaw_kp*yaw_err;
	if(F_R_Speed==0)//防止误差太小
	{
		if(yaw_err>1.00)
		{
			F_R_Speed=1;
			F_L_Speed=-1;
		}
		if(yaw_err<-1.00)
		{
			F_R_Speed=-1;
			F_L_Speed=1;
		}
	}

	return 0;
}



