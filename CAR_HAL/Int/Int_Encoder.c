#include "Int_Encoder.h"



/**
 * @description: 读取编码器模式的计数值，处理成带符号的值
 * @param {uint8_t} timx 要读取的定时器的编号
 * @return {*} 读到的带符号的计数值
 */
int Int_Encoder_ReadCounter(uint8_t timx)
{
    int encoder_value = 0;
    switch (timx)
    {
    case 2:
    {
        /* 读取定时器2的计数值，并且转换成带符号类型 */
        // encoder_value = (short)TIM2->CNT;
        // TIM2->CNT = 0;
        encoder_value = (short)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        break;
    }
    case 3:
    {
        /* 读取定时器3的计数值，并且转换成带符号类型 */
        // encoder_value = (short)TIM3->CNT;
        // TIM3->CNT = 0;
        encoder_value = (short)__HAL_TIM_GET_COUNTER(&htim3);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        break;
    }
    default:
        break;
    }

    /* 由于电机安装是相差180度，后续读取时，要手动统一方向符号 */
    return encoder_value;
}
