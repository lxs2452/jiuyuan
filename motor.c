#include "file.h"

void Load(int moto1,int moto2)
{
	if(moto1>0){//这个为左轮
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 , GPIO_PIN_SET);//往前
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 , GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 , GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,abs(moto1));//设置占空比
	if(moto2<0){//这个为右轮
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,abs(moto2));//设置占空比
}
