#include "Int_TB6612.h"





/**
 * @description: 控制电机A的方向
 * @param {uint8_t} direct 期望的转动方向
 * @return {*}
 */
void Int_TB6612_MotorA(uint8_t direct)
{
    if(direct == GO)
    {
        /* 正转：AIN1=0,AIN2=1 */
       AIN1_L;
       AIN2_H;
    }
    else if(direct == BACK)
    {
        /* 反转：AIN1=1,AIN2=0 */
        AIN1_H;
        AIN2_L;
    }
    else 
    {
        /* 其他刹车：AIN1=1,AIN2=1 */
        AIN1_H;
        AIN2_H;
    }
}


/**
 * @description: 控制电机B的方向
 * @param {uint8_t} direct 期望的转动方向
 * @return {*}
 */
void Int_TB6612_MotorB(uint8_t direct)
{
    if(direct == GO)
    {
        /* 正转：BIN1=0,BIN2=1 */
       BIN1_L;
       BIN2_H;
    }
    else if(direct == BACK)
    {
        /* 反转：BIN1=1,BIN2=0 */
        BIN1_H;
        BIN2_L;
    }
    else 
    {
        /* 其他刹车：BIN1=1,BIN2=1 */
        BIN1_H;
        BIN2_H;
    }
}


/**
 * @description: 根据带符号的pwm值，设置转速和方向
 * @param {int} pwma   A电机，带符号
 * @param {int} pwmb   B电机，带符号
 * @return {*}
 */
void Int_TB6612_SetPWM(int pwma,int pwmb)
{
    /* 根据带符号的pwm值：1、处理方向；2、设置pwm占空比 */


    /* 1.处理pwma */
    /* 1.1 根据符号处理方向 */
    if(pwma > 0)
    {
        /* 正转 */
        Int_TB6612_MotorA(GO);
    }
    else if(pwma < 0)
    {
        /* 反转 */
        Int_TB6612_MotorA(BACK);
        /* 为后续设置ccr做准备，取绝对值 */
        pwma = -pwma;
    }
    else 
    {
        /* 刹车 */
        Int_TB6612_MotorA(STOP);
    }
    /* 1.2 设置对应的pwm值 =》 对应定时器通道CH4的ccr值 */
    // TIM4 ->CCR4 = pwma;
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pwma);


    /* 2.处理pwmb */
    /* 1.1 根据符号处理方向 */
    if(pwmb > 0)
    {
        /* 正转 */
        Int_TB6612_MotorB(GO);
    }
    else if(pwmb < 0)
    {
        /* 反转 */
        Int_TB6612_MotorB(BACK);
        /* 为后续设置ccr做准备，取绝对值 */
        pwmb = -pwmb;
    }
    else 
    {
        /* 刹车 */
        Int_TB6612_MotorB(STOP);
    }
    /* 1.2 设置对应的pwm值 =》 对应定时器通道CH3的ccr值 */
    // TIM4 ->CCR3 = pwmb;
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pwmb);
}



