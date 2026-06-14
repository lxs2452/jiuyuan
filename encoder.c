#include "file.h"
int left;
int right;
uint32_t distance;  
uint32_t juli=0;// 测距结果(cm)
uint8_t  is_capture_ok = 0;
uint32_t cap_val1 = 0;
uint32_t cap_val2 = 0;

//读取编码器
int Read_Speed(TIM_HandleTypeDef *htim)
{
	int temp;
	temp=(short)__HAL_TIM_GetCounter(htim);
	if (htim->Instance == TIM2)
	 {		 
		temp = -temp;
	 }

	__HAL_TIM_SetCounter(htim,0	);
	
	return temp;
}

// 舵机角度控制
void Servo_SetAngle(uint16_t angle)
{
    // 舵机公式：0~180° → 对应PWM比较值 500~2500
    // 自动限幅 0~180
    if(angle > 180) angle = 180;

    uint32_t pwm = 50 + (angle * 200 / 180);

    // 如果你用的是 TIM8_CH2
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm);
}

// 抓取球
void Servo_Grab(void)
{
    Servo_SetAngle(SERVO_CLOSE);
}

// 松开球
void Servo_Release(void)
{
    Servo_SetAngle(SERVO_OPEN);
}

void delay_us(uint32_t us)
{
    uint32_t delay = us * (SystemCoreClock / 1000000 / 5);
    while(delay--);
}

//PB8Trig触发  PB2Echo输入模式 超声波
void SR04_Read(void)
{
    // 1. 发10us触发脉冲
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(20);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

    // 2. 等待回响高电平
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);

    // 3. 开启定时器
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Base_Start(&htim4);

    // 4. 等待回响结束
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1);

    // 5. 读取时间 计算距离
    uint32_t time = __HAL_TIM_GET_COUNTER(&htim4);
    distance = time /58;  // cm

    HAL_TIM_Base_Stop(&htim4);
}

//均值滤波减小测量误差
int  Distance(uint8_t cnt)
{
    int  sum = 0;
	SR04_Read();
    for(int i =0;i<cnt;i++)
    {
        sum+=distance;
    }
 return sum/cnt;
}

