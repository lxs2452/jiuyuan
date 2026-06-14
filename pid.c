#include "file.h"
BallInfo_TypeDef ball;
SafeAreaType safe;
uint8_t reach_flag = 0;
uint8_t Rtdata;
uint8_t tiaoshi;
// 串级PID实例
CascadedPID_TypeDef cas_pid_left;
CascadedPID_TypeDef cas_pid_right;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

uint8_t uart2_dma_buf[RX_BUFFER_SIZE];
uint8_t uart_buf[30];
uint8_t buf_len = 0;
uint8_t ch;

// ==============================
// 1. 初始化串级PID参数
// ==============================
void PID_Init(void)
{
    // ==================== 位置环（外层：视觉位置 -> 速度指令）====================
    // X轴（转向）
    cas_pid_left.pos.Kp = 25.3f;
    cas_pid_left.pos.Ki = 0.0f;
    cas_pid_left.pos.Kd = 0.5f;

    // Y轴（前进）
    cas_pid_right.pos.Kp = 25.3f;
    cas_pid_right.pos.Ki = 0.0f;
    cas_pid_right.pos.Kd = 0.5f;

    // ==================== 速度环（内层：速度指令 -> PWM输出）====================
    cas_pid_left.spd.Kp = 0.5f;
    cas_pid_left.spd.Ki = 3.0f;
    cas_pid_left.spd.Kd = 0.0f;

    cas_pid_right.spd.Kp = 0.5f;
    cas_pid_right.spd.Ki = 3.0f;
    cas_pid_right.spd.Kd = 0.0f;

    // ==================== 预留 ====================
    cas_pid_left.jlh.Kp = 1.0f;
    cas_pid_left.jlh.Ki = 0.0f;
    cas_pid_left.jlh.Kd = 0.5f;

    cas_pid_right.jlh.Kp = 1.0f;
    cas_pid_right.jlh.Ki = 0.0f;
    cas_pid_right.jlh.Kd = 0.5f;

    // ==================== 角度环（陀螺仪转向用）====================
    cas_pid_left.jdh.Kp = 0.2f;
    cas_pid_left.jdh.Ki = 0.0f;
    cas_pid_left.jdh.Kd = 0.0f;

    cas_pid_right.jdh.Kp = 3.0f;
    cas_pid_right.jdh.Ki = 0.0f;
    cas_pid_right.jdh.Kd = 0.0f;
}

// ==============================
// 2. 位置环PID（外层：视觉位置误差 -> 目标速度）
//    使用PD控制，不积累积分防止外层饱和
// ==============================
float PosPID_Calc(PID_TypeDef *pid, float target, float current)
{
    pid->err = target - current;

    pid->output = pid->Kp * pid->err
                + pid->Kd * (pid->err - pid->last_err);

    pid->last_err = pid->err;

    // 输出限幅 = 目标速度（编码器速度单位）
    if(pid->output > 300.0f) pid->output = 300.0f;
    if(pid->output < -300.0f) pid->output = -300.0f;

    return pid->output;
}

// ==============================
// 3. 速度环PID（内层：速度误差 -> PWM输出）
//    使用增量式PI控制，直接驱动电机
// ==============================
float SpeedPID_Calc(PID_TypeDef *pid, float target, float current)
{
    pid->err = target - current;

    pid->output += pid->Kp * (pid->err - pid->last_err)
                + pid->Ki * pid->err;

    pid->last_err = pid->err;

    // 输出限幅 = PWM占空比范围
    if(pid->output > 3000.0f) pid->output = 3000.0f;
    if(pid->output < -3000.0f) pid->output = -3000.0f;

    return pid->output;
}

//////////////////////////////////////////////////////
// 以下为兼容旧代码保留的全局变量版速度PID（Turn_To_Target使用）
//////////////////////////////////////////////////////
int err;
int last_err;
int output;
float kp=2;
float ki=2;
int SpeedRPID_Calc(int target, int current)
{
    err = target - current;
    output += kp *(err-last_err)+ ki * err;
    last_err = err;

    // PWM限幅
    if(output > 800) output = 800;
    if(output < -800) output = -800;

    return output;
}

int SpeedLPID_Calc(int target, int current)
{
    err = target - current;
    output += kp *(err-last_err)+ ki * err;
    last_err = err;

    // PWM限幅
    if(output > 3000) output = 3000;
    if(output < -3000) output = -3000;

    return output;
}

// ==============================
// 4. 角度环PID（陀螺仪转向用）
// ==============================
float Gyro_PID_Calc(PID_TypeDef *pid, float target, float current)
{
    pid->err = target - current;

    pid->output = pid->Kp * pid->err
                + pid->Ki * (pid->err - pid->last_err);

    pid->last_err = pid->err;

    // PWM限幅
    if(pid->output > 2000) pid->output = 2000;
    if(pid->output < -2000) pid->output = -2000;

    return pid->output;
}

void Turn_To_Target(float angle)
{
    float pwm = Gyro_PID_Calc(&cas_pid_left.jdh, angle, yaw2);
    float pwm1 = SpeedRPID_Calc(pwm, left);
    // 原地旋转控制：左右轮反向
    Load(-pwm1, pwm1);

    // 到达角度关闭转向标志
    if(abs(angle - yaw2) < 3)
    {
        Load(0,0);
        robot_state = 5;
    }
}

// ==============================
// 5. 串级PID总计算 —— 视觉定位（2层串级：位置环→速度环）
//
//   结构：
//     位置环(外层)              速度环(内层)
//   ball_x ─→ PosPID ─→ vel_x ─┐
//                                ├→ left_vel_sp  ─→ SpdPID ─→ PWM_left
//   ball_y ─→ PosPID ─→ vel_y ─┘
//                                └→ right_vel_sp ─→ SpdPID ─→ PWM_right
//
//   视觉位置误差 → 目标速度 → PWM
// ==============================
void CascadedPID_Calc(float ball_x, float ball_y)
{
    // ===== 第1层：视觉位置环（PD控制，输出目标速度）=====
    float target_vel_x = PosPID_Calc(&cas_pid_left.pos,  TARGET_X, ball_x);
    float target_vel_y = PosPID_Calc(&cas_pid_right.pos, TARGET_Y, ball_y);

    // 将XY速度指令转换为左右轮速度指令（差速运动学）
    float target_vel_left  = target_vel_y - (target_vel_x * TURN_GAIN);
    float target_vel_right = target_vel_y + (target_vel_x * TURN_GAIN);

    // ===== 第2层：速度环（增量PI控制，直接输出PWM）=====
    float pwm_left  = SpeedPID_Calc(&cas_pid_left.spd,  target_vel_left,  (float)left);
    float pwm_right = SpeedPID_Calc(&cas_pid_right.spd, target_vel_right, (float)right);

    // 输出到电机
    Load((int)pwm_left, (int)pwm_right);
}

// ==============================
// UART2 空闲中断处理（MaixCam视觉数据）
// ==============================
void USART2_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        HAL_UART_DMAStop(&huart2);

        uint16_t rx_len = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

        buf_len = 0;
        memset(uart_buf, 0, sizeof(uart_buf));

        for(uint16_t i=0; i<rx_len; i++)
        {
            uint8_t ch = uart2_dma_buf[i];

            if(buf_len == 0)
            {
                if(ch == 0xAA || ch == 0xBB)
                {
                    uart_buf[buf_len++] = ch;
                }
            }
            else
            {
                if(ch == 0xAF)
                {
                    uart_buf[buf_len] = '\0';
                    if(uart_buf[0] == 0xAA)
                    {
                        int tx, ty, tcolor;
                        if(sscanf((char*)uart_buf+1, "%d,%d,%d", &tx, &ty, &tcolor) == 3)
                        {
                            if(tx >= 0 && tx <= 320 && ty >=0 && ty <=320)
                            {
                                ball.x = tx;
                                ball.y = ty;
                                ball.label = tcolor;
                                ball.is_valid = 1;
                            }
                        }
                    }
                    else if(uart_buf[0] == 0xBB)
                    {
                        int sx1,sy1,sy2;
                        if(sscanf((char*)uart_buf+1, "%d,%d,%d", &sx1, &sy1, &sy2) == 3)
                        {
                            safe.ax = sx1;
                            safe.ay = sy1;
                            safe.label2 = sy2;
                            safe.is_ok = 1;
                        }
                    }
                    buf_len = 0;
                    memset(uart_buf, 0, sizeof(uart_buf));
                }
                else
                {
                    if(buf_len < sizeof(uart_buf)-1)
                    {
                        uart_buf[buf_len++] = ch;
                    }
                }
            }
        }

        HAL_UART_Receive_DMA(&huart2, uart2_dma_buf, RX_BUFFER_SIZE);
    }
    HAL_UART_IRQHandler(&huart2);
}

// ====================== UART3接收中断回调（JY61P陀螺仪）=====================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART3)
    {
        jy61p_pack(Rtdata);
        HAL_UART_Receive_IT(&huart3,&Rtdata,1);
    }

}