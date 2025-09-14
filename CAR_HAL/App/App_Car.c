/* Put these near the other includes at the top of App_Car.c */
#include <stdint.h>   // uint8_t, etc
#include <stddef.h>   // size_t
#include <stdbool.h>  // bool, true, false
#include <ctype.h>    // isprint()
#include <string.h>   // memcpy, memset, strlen
#include <stdio.h>    // snprintf, printf

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

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//  /* Prevent unused argument(s) compilation warning */
//	//printf("size %d\n",Size);
//	
//	
//    if (huart->Instance == USART2)
//    {
//        HAL_UARTEx_ReceiveToIdle_IT(&huart2,buff2,BUFF_SIZE);
//        switch (buff2[0])
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
//        printf("receive from uart2: %.*s\n", (int)Size, buff2);
//				HAL_UART_Transmit(&huart2,buff2,Size,BUFF_SIZE);
//				memset(buff2,0,sizeof(buff2));
//    }else if(huart->Instance == USART1)
//    {
//        HAL_UARTEx_ReceiveToIdle_IT(&huart1,buff,BUFF_SIZE);
//        //in order to send cmd to Uart2 BLE we use Uart1 to transfer cmd to Uart2
//        HAL_UART_Transmit(&huart2,buff,Size,200);
//        printf("receive from uart1: %.*s\n", (int)Size, buff);
//				memset(buff,0,sizeof(buff));
//    }
//    
//    


//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
//   */
//}

// Add these defines for protocol parsing
#define PROTOCOL_HEADER 0x5A
#define PROTOCOL_BUFFER_SIZE 16

uint8_t protocol_buffer[PROTOCOL_BUFFER_SIZE];
uint8_t protocol_index = 0;
uint8_t expected_length = 0;
uint8_t parsing_header = 1;


/* helper: returns true if buffer looks like an AT/text response */
static bool looks_like_at_text(const uint8_t *buf, size_t len)
{
    if (len == 0) return false;

    // Check common AT patterns: starts with "AT" or '+' or printable text (like "+VER:", "OK", etc)
    if (len >= 2 && buf[0] == 'A' && buf[1] == 'T') return true;
    if (buf[0] == '+') return true;
    if (len >= 2 && buf[0] == 'O' && buf[1] == 'K') return true;

    // Fallback: require all bytes printable or whitespace (CR/LF, TAB, space)
    for (size_t i = 0; i < len; ++i) {
        unsigned char c = buf[i];
        if (c == '\r' || c == '\n' || c == '\t' || c == ' ') continue;
        if (!isprint(c)) return false;
    }
    return true;
}


void parse_custom_protocol(uint8_t* data, uint8_t length)
{
		char ack[64] = "OK\r\n";
    // Debug: Print the received frame
    printf("Received frame: ");
    for(int i = 0; i < length; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    
    // Check if we have a complete frame
    if(length < 4) {
        printf("Frame too short (%d bytes), expected at least 4\n", length);
        return;
    }
    
    uint8_t device_id = data[2];
    uint8_t function_code = data[3];
    
    printf("Device ID: 0x%02X, Function Code: 0x%02X\n", device_id, function_code);
    
    if(function_code == 0x00) { // Movement control
        if(length >= 6) { // Ensure we have data bytes
            int8_t linear_x = (int8_t)data[4];
            int8_t angular_z = (int8_t)data[5];
            
            printf("Linear X: %d, Angular Z: %d\n", linear_x, angular_z);
            
            // Convert to your car's control format
            // Adjust these calculations based on your car's behavior
            if(linear_x > 0) {
                flag_up = 1;
                flag_down = 0;
                remote_move = linear_x * 0.5; // Scale to your range
                printf("Moving forward, remote_move: %d\n", remote_move);
            } else if(linear_x < 0) {
                flag_up = 0;
                flag_down = 1;
                remote_move = linear_x * 0.5; // Scale to your range
                printf("Moving backward, remote_move: %d\n", remote_move);
            } else {
                flag_up = 0;
                flag_down = 0;
                remote_move = 0;
                printf("Stopping linear movement\n");
            }
            
            if(angular_z > 0) {
                flag_left = 0;
                flag_right = 1;
                remote_turn = angular_z * 2; // Scale to your range
                printf("Turning right, remote_turn: %d\n", remote_turn);
            } else if(angular_z < 0) {
                flag_left = 1;
                flag_right = 0;
                remote_turn = angular_z * 2; // Scale to your range
                printf("Turning left, remote_turn: %d\n", remote_turn);
            } else {
                flag_left = 0;
                flag_right = 0;
                remote_turn = 0;
                printf("No turning\n");
            }
            
            
            printf("Flags: UP=%d, DOWN=%d, LEFT=%d, RIGHT=%d\n", 
                   flag_up, flag_down, flag_left, flag_right);
        } else {
					  sprintf(ack,"Error frame too short (%d bytes), expected \r\n", length);
            printf("Movement control frame too short (%d bytes), expected 6\n", length);
        }
    } else {
			  sprintf(ack,"Unknown function code: 0x%02X\n", function_code);
        printf("Unknown function code: 0x%02X\n", function_code);
    }
		//response 
		HAL_UART_Transmit(&huart2, (uint8_t*)ack, strlen(ack), HAL_MAX_DELAY);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, buff2, BUFF_SIZE);
        
        printf("Received %d bytes on USART2: ", Size);
        for(int i = 0; i < Size; i++) {
            printf("%02X ", buff2[i]);
        }
        printf("\n");
        
        // Process received data for custom protocol
        for(int i = 0; i < Size; i++) {
            if(parsing_header) {
                if(buff2[i] == PROTOCOL_HEADER) {
                    printf("Found protocol header 0x5A\n");
                    protocol_buffer[0] = buff2[i];
                    protocol_index = 1;
                    parsing_header = 0;
                }
            } else {
                if(protocol_index == 1) {
                    // This is the length byte
                    expected_length = buff2[i];
                    protocol_buffer[protocol_index++] = buff2[i];
                    printf("Expected frame length: %d\n", expected_length);
                } else if(protocol_index < expected_length) {
                    protocol_buffer[protocol_index++] = buff2[i];
                    
                    // Check if we've received the complete frame
                    if(protocol_index == expected_length) {
                        printf("Complete frame received\n");
                        parse_custom_protocol(protocol_buffer, expected_length);
                        protocol_index = 0;
                        parsing_header = 1;
                    }
                } else {
                    // Error in protocol, reset
                    printf("Protocol error, resetting parser\n");
                    protocol_index = 0;
                    parsing_header = 1;
                }
            }
        }
        if (looks_like_at_text(buff2, Size)) {
            // Safe: create a null-terminated copy for printing as string
            size_t copy_len = (Size < BUFF_SIZE-1) ? Size : (BUFF_SIZE-1);
            char tmp[BUFF_SIZE];
            memcpy(tmp, buff2, copy_len);
            tmp[copy_len] = '\0';

            printf("Raw data as string: %s\n", tmp);

            // forward text exactly Size bytes (do NOT use BUFF_SIZE here)
            HAL_UART_Transmit(&huart2, buff2, Size, HAL_MAX_DELAY);
        } else {
            // For binary frames: print hex (already done above) and DO NOT echo raw bytes
            // If you want to forward a hex string to central, uncomment and use the code below:

            
            char hexbuf[128];
            int n = 0;
            for (int i = 0; i < Size && n + 3 < (int)sizeof(hexbuf); ++i) {
                n += snprintf(&hexbuf[n], sizeof(hexbuf) - n, "%02X ", buff2[i]);
            }
            hexbuf[n++] = '\r';
            hexbuf[n++] = '\n';
            HAL_UART_Transmit(&huart2, (uint8_t*)hexbuf, n, HAL_MAX_DELAY);
            /**/
        }

        // clear buffer
        memset(buff2, 0, sizeof(buff2));
    } else if(huart->Instance == USART1) {
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, buff, BUFF_SIZE);
        HAL_UART_Transmit(&huart2, buff, Size, 200);
        printf("Received %d bytes on USART1: %.*s\n", (int)Size, (int)Size, buff);
        memset(buff, 0, sizeof(buff));
    }
}

