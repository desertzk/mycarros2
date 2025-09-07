#include "Com_PID.h"

/**
 * @description: 直立环PD控制
 * @param {float} kp  p系数
 * @param {float} kd  d系数
 * @param {float} kalman_angle 卡尔曼滤波后的角度
 * @param {float} balance_angle 期望的平衡时的角度
 * @param {short} gy y轴角速度原始值
 * @return {*}
 */
int Com_PID_Balance(float kp, float kd, float kalman_angle, float balance_angle, short gy)
{
    /* 1.计算角度偏差 */
    float bias_angle = 0.0f;
    bias_angle = kalman_angle - balance_angle;
    /* 2.进行PID计算 */
    return kp * bias_angle + kd * gy;
}

/**
 * @description: 速度环PI控制
 * @param {float} kp p系数
 * @param {float} ki i系数
 * @param {int} encoder_a A电机读到的编码值
 * @param {int} encoder_b B电机读到的编码值
 * @param {int} remove_move 遥控前后运动的控制量
 * @return {*}
 */
int Com_PID_Velocity(float kp, float ki, int encoder_a, int encoder_b,int remove_move)
{
    int bias_velocity = 0;
    static int least_velocity = 0; // 速度积分值（累加值）
    static int last_velocity = 0;  // 上一次的速度偏差值
    /* 1.计算速度偏差 */
    bias_velocity = (encoder_a + encoder_b) - 0;

    /* 2.对速度偏差做一个滤波（一阶低通），因为我们使用叠在的方式，减轻速度环的影响，只是辅助 */
    /* 最终本次的值 = k*上次的值 + （1-k）*本次的值 */
    bias_velocity = 0.8 * last_velocity + 0.2 * bias_velocity;
    /* 保存本次的偏差值，下一次计算用 */
    last_velocity = bias_velocity;

    /* 3.计算积分(没有乘时间，回头调整ki的大小就可以) */
    least_velocity += bias_velocity;
    /* 遥控前后运动，从位移上留出来 */
    least_velocity -= remove_move;

    /* 4.积分限幅（一般积分操作，都要考虑限幅的问题） */
    /* 对速度偏差的累加， 速度取值是-32768~32767之间，考虑积分制在1/3的范围内 */
    if(least_velocity > 10000)
    {
        least_velocity = 10000;
    }
    else if(least_velocity < -10000)
    {
        least_velocity = -10000;
    }

    /* 5.计算PID */
    return kp * bias_velocity + ki * least_velocity;
}

/**
 * @description: 转向环P控制（辅助作用） keep car run in straight line
 * @param {float} kp p系数
 * @param {short} gz z轴角速度
 * @return {*}
 */
int Com_PID_Turn(float kp,short gz)
{
    /* kp*Z轴角速度 */
    return kp*(gz-0);
}
