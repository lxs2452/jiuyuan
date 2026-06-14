#ifndef _JY61P_H
#define _JY61P_H

#include "stm32f1xx_hal.h"
extern uint8_t agree[9];
void jy61p_pack(uint8_t data);
int turn_jy61p_yaw(float goal);
#define YAWH agree[5]
#define YAWL agree[4]
extern uint8_t pdata;
extern float yaw_g;
extern int F_R_Speed;
extern int F_L_Speed;
extern float pitch1,roll1,yaw2;
#define ROLLH agree[3]
#define ROLLL agree[2]

#define PICH agree[1] 
#define PICL agree[0]

#define YAWGH agree2[5]
#define YAWGL agree2[4]

#endif
