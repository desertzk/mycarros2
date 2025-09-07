#include <string.h>
#include "App_Car.h"


short gx, gy, gz;
short ax, ay, az;

float accel_angle;  // 通过加速度计算出来的角度
float gyro_y;       // Y轴角速度，采样值转换成角度
extern float angle; // 卡尔曼滤波后的角度

int ea, eb;

char bat_str[5];   // 2个整数位+1个小数点+1个小数位+1个字符串结尾\0
char ea_str[7];    // 1个符号位+5个整数位+1个字符串结尾\0
char eb_str[7];    // 1个符号位+5个整数位+1个字符串结尾\0
char angle_str[7]; // 1个符号位+3个整数位+1个小数点+1个小数位+1个字符串结尾\0

/* =======直立环PID参数========= */
float balance_kp = -720.0;  // 刚出现低频震荡的值=-900，乘0.6=-540
float balance_kd = 0.72;    // 刚出现高频震荡的值+1.5，乘0.6=+0.9
float balance_angle = -1.0; // -1.0
/* =======速度环PID参数========= */
float velocity_kp = 170.0; // +220
float velocity_ki = 0.85;  // +1.10
/* =======转向环PID参数========= */
float turn_kp = 0.5; // 0.5

/* 遥控运动的标志位 */
uint8_t flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 0;
int remote_move = 0; // 遥控前后移动的控制量
int remote_turn = 0; // 遥控左右移动的控制量

/**
 * @description: 计算小车倾角
 * @return {*}
 */
void App_Car_GetAngle(void)
{
    /* 1. 读取MPU6050的数据 */
    Int_MPU6050_Get_Accel(&ax, &ay, &az);
    Int_MPU6050_Get_Gyro(&gx, &gy, &gz);

    /* 2. 通过加速度计算倾角 */
    /* atan2得到的是弧度， 角度 = 弧度 * 180/PI */
    accel_angle = atan2(ax, az) * 180 / PI;

    /* 3. 角速度：量程是+-2000°/s ，65536/4000=16.4  */
    /* 注意，角速度的符号要和加速度计算的角度符号方向一致 */
    gyro_y = -gy / 16.4;

    /* 4. 计算的倾角和角速度，进行卡尔曼滤波 */
    Com_Filter_Kalman(accel_angle, gyro_y);

    // printf("accel_angle=%.1f\r\n", accel_angle);
    // printf("gyro_y=%.1f\r\n", gyro_y);
    // printf("angle=%.1f\r\n", angle);

    /* 将读取编码器的值，也放到获取角度的函数中，这样两类数据就可以同频 */
    ea = Int_Encoder_ReadCounter(2);
    eb = -Int_Encoder_ReadCounter(3);
}

/**
 * @description: 显示任务：填充电池电压值、两个编码器的值、计算的角度值
 * @return {*}
 */
void App_Car_Display(void)
{
    /* 1.填充电压值 */
    double bat_vol = 0.0;
    /* 禁用了连续转换，每次读取前手动启动adc */
    HAL_ADC_Start(&hadc1);
    bat_vol = (HAL_ADC_GetValue(&hadc1) * 3.3 / 4095) * 4;

    sprintf(bat_str, "%3.1f", bat_vol);
    OLED_ShowString(32, 0, bat_str, 16, 1);

    /* 2.填充编码器值 */
    sprintf(ea_str, "%6d", ea);
    sprintf(eb_str, "%6d", eb);
    OLED_ShowString(24, 16, ea_str, 16, 1); // 前面有EA:三个字符，x=3*8=24开始;第二行，y=16
    OLED_ShowString(24, 32, eb_str, 16, 1); // 前面有EB:三个字符，x=3*8=24开始;第三行，y=32

    /* 3.填充角度值 */
    sprintf(angle_str, "%5.1f", angle);
    OLED_ShowString(48, 48, angle_str, 16, 1); // 前面有Angle:六个字符，x=6*8=48开始;第四行，y=48

    /* 4.刷写到显存中 */
    OLED_Refresh();
}

void App_Car_PID(void)
{
    int balance_out = 0;
    int velocity_out = 0;
    int turn_out = 0;
    int pwma = 0, pwmb = 0;
    /* 1.直立环控制 */
    balance_out = Com_PID_Balance(balance_kp, balance_kd, angle, balance_angle, gy);

    /* 2.速度环控制 */
    /* 添加遥控前后移动的逻辑 */
    if (flag_up)
    {
        remote_move = 50;
    }
    else if (flag_down)
    {
        remote_move = -50;
    }
    else
    {
        /* 如果没有前后控制，需要主动清零控制量，不要影响正常的平衡控制 */
        remote_move = 0;
    }
    velocity_out = Com_PID_Velocity(velocity_kp, velocity_ki, ea, eb, remote_move);

    /* 3.转向环控制 */
    if (flag_left)
    {
        remote_turn += -20;
    }
    else if (flag_right)
    {
        remote_turn += 20;
    }
    else
    {
        /* 如果不左右转，要清零控制量 */
        remote_turn = 0;
        /* 不左右转，才进行转向环的控制 */
        turn_out = Com_PID_Turn(turn_kp, gz);
    }
    /* 一直按住会不断累加，所以加个限幅 */
    if (remote_turn > 500)
    {
        remote_turn = 500;
    }
    else if (remote_turn < -500)
    {
        remote_turn = -500;
    }

    /* 4.叠加PID结果，作用到电机上 */
    pwma = balance_out + velocity_out + turn_out + remote_turn;
    pwmb = balance_out + velocity_out - turn_out - remote_turn;
    Int_TB6612_SetPWM(pwma, pwmb);
}

/**
 * @description: USART2的中断处理函数，无线遥控相关的逻辑  receive fixed size data
 * @return {*}  
 */
extern uint8_t buff[BUFF_SIZE];
extern uint8_t buff2[BUFF_SIZE];

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART2)
//    {
//        switch (buff[0])
//        {
//        case 'U':
//            flag_up = 1, flag_down = 0, flag_left = 0, flag_right = 0;
//            break;
//        case 'D':
//            flag_up = 0, flag_down = 1, flag_left = 0, flag_right = 0;
//            break;
//        case 'L':
//            flag_up = 0, flag_down = 0, flag_left = 1, flag_right = 0;
//            break;
//        case 'R':
//            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 1;
//            break;
//        case 'S':
//            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 0;
//            break;
//        default:
//            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 0;
//            break;
//        }
//    }
//    HAL_UART_Receive_IT(&huart2, buff, 1);//receive char to buff
//}

extern int16_t g_size;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Prevent unused argument(s) compilation warning */
	printf("size %d\n",Size);
	
	
    if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_IT(&huart2,buff2,BUFF_SIZE);
        switch (buff2[0])
        {
        case 'U':
            flag_up = 1, flag_down = 0, flag_left = 0, flag_right = 0;
            break;
        case 'D':
            flag_up = 0, flag_down = 1, flag_left = 0, flag_right = 0;
            break;
        case 'L':
            flag_up = 0, flag_down = 0, flag_left = 1, flag_right = 0;
            break;
        case 'R':
            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 1;
            break;
        case 'S':
            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 0;
            break;
        default:
            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 0;
            break;
        }
        printf("receive from uart2: %.*s\n", (int)Size, buff2);
				HAL_UART_Transmit(&huart2,buff2,Size,BUFF_SIZE);
				memset(buff2,0,sizeof(buff2));
    }else if(huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_IT(&huart1,buff,BUFF_SIZE);
        //in order to send cmd to Uart2 BLE we use Uart1 to transfer cmd to Uart2
        HAL_UART_Transmit(&huart2,buff,Size,200);
        printf("receive from uart1: %.*s\n", (int)Size, buff);
				memset(buff,0,sizeof(buff));
    }
    //HAL_UART_Receive_IT(&huart2, buff, 1);//receive char to buff
    


  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
}
