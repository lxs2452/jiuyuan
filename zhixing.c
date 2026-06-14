#include "file.h"
uint8_t count;
uint8_t count1;
uint8_t flag;

// 状态定义
// 0 = 原地自转找球
// 1 = 视觉定位追球
// 2 = 到位稳停
// 3 = 抓球
// 4 = 后退30cm
// 5 = 陀螺仪+距离去安全区

int robot_state = 0;   // 默认从 0 开始

// 标志位
int have_ball    = 0;  // 1=看到球ball.label
int catch_ok     = 0;  // 1=抓完球
int back_ok      = 0;  // 1=后退完成
int safe_arrive  = 0;  // 1=到安全区

void Ball_Track_Task(void)//追踪小球逻辑函数
{

    switch(robot_state)//robot_state执行顺序
    {
        // ==============================
        // 0：没球 → 原地自转寻找
        // ==============================
        case 0:
            if(ball.label ==2)//有红球
            {
                Load(0,0);
                robot_state = 1; // 看到球 → 切换到追球
				catch_ok = 1;
            }
//			else if ((catch_ok == 1&ball.label ==3)||(catch_ok == 1&ball.label ==4))
//			{
//			    Load(0,0);
//                robot_state = 1; // 看到球 → 切换到追球
//			}	
            else if(ball.label ==9)
            {
                Load(-800, 800); // 原地自转
            }
            break;

        // ==============================
        // 1：有球 → 位置环追球
        // ==============================
        case 1:
            if(ball.label == 9)
            {
                robot_state = 0; // 球丢了 → 回去找
                break;
            }

            // 视觉PID定位
            CascadedPID_Calc(ball.x , ball.y );	

            // 判断是否到位
            if(abs(ball.x - TARGET_X) < 5 && abs(ball.y - TARGET_Y) < 10)
            {
          		Load(0, 0);   
				 robot_state = 2; // 到位 → 稳停
				
            }
            break;

        // ==============================
        // 2：已到位 → 往前走
        // ==============================
        case 2:	
            Load(1500,2000);//调一下直走
		    HAL_Delay(2500);
		    Load(0,0); 
		    robot_state = 3; 
            break;

        // ==============================
        // 3：抓球
        // ==============================
        case 3:
		    Servo_Grab();
		    robot_state = 5;
            break;

        // ==============================
        // 4：抓完球 → 陀螺仪转90度
        // ==============================
        case 4:
//            Turn_To_Target(90.0);	//robot_state = 5;在里面
//            break;
              //robot_state = 5; 
        // ==============================
        // 5：去安全区并放球
        // ==============================
        case 5://把安全区定位到中间
           
		 if(safe.label2  == 9)
            {
               Load (-900,900); // 找安全区   
            }

		// 视觉PID定位
	  else if(safe.label2  == 6)//红色安全区
			{
			 CascadedPID_Calc(safe .ax  , safe .ay  );	
			 // 判断是否到位
            if(abs(safe .ax - TARGET_X) < 15 && abs(safe .ay - TARGET_Y) < 30)
            {
          		Load(0, 0); 
				robot_state = 6;
            }
			}	   
            break;
			
		case 6:
			
		Load (1700,2000);//去安全区放球，快一点
		HAL_Delay(4000);
		Load (0,0);
		
		Servo_Release();//松爪
		HAL_Delay(1000);
		
		Load (-1000,-1000);//放完球后退
		HAL_Delay(4000); 
		
        robot_state = 0;//重新找球
            break;
    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM7)//10ms
	{
	left=-Read_Speed(&htim1);
	right=Read_Speed(&htim2);	
			
       	
//Ball_Track_Task();	
	
	}
}

