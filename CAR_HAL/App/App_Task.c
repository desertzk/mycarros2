#include "App_Task.h"

TaskHandle_t start_task_handle;
#define START_TASK_STACK 128
#define START_TASK_PRIORITY 1
void App_Task_Start(void *pvParameters);

/* 获取姿态数据的任务配置 */
TaskHandle_t data_task_handle;
#define DATA_TASK_STACK 128
#define DATA_TASK_PRIORITY 3
void App_Task_GetData(void *pvParameters);

/* 进行PID控制的任务配置 */
TaskHandle_t pid_task_handle;
#define PID_TASK_STACK 128
#define PID_TASK_PRIORITY 3
void App_Task_PID(void *pvParameters);

/* 进行PID控制的任务配置 */
TaskHandle_t display_task_handle;
#define DISPLAY_TASK_STACK 128
#define DISPLAY_TASK_PRIORITY 3
void App_Task_Display(void *pvParameters);

void App_Task_Init(void)
{
    /* 1. 创建启动任务 */
    xTaskCreate(
        (TaskFunction_t)App_Task_Start,
        (char *)"App_Task_Start",
        (configSTACK_DEPTH_TYPE)START_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)START_TASK_PRIORITY,
        (TaskHandle_t *)&start_task_handle);

    /* 2. 启动调度器 */
    vTaskStartScheduler();
}

/**
 * @description: 启动任务：用于创建其他任务
 * @param {void *} pvParameters
 * @return {*}
 */
void App_Task_Start(void *pvParameters)
{
    taskENTER_CRITICAL();
    xTaskCreate(
        (TaskFunction_t)App_Task_GetData,
        (char *)"App_Task_GetData",
        (configSTACK_DEPTH_TYPE)DATA_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)DATA_TASK_PRIORITY,
        (TaskHandle_t *)&data_task_handle);
    xTaskCreate(
        (TaskFunction_t)App_Task_PID,
        (char *)"App_Task_PID",
        (configSTACK_DEPTH_TYPE)PID_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)PID_TASK_PRIORITY,
        (TaskHandle_t *)&pid_task_handle);
    xTaskCreate(
        (TaskFunction_t)App_Task_Display,
        (char *)"App_Task_Display",
        (configSTACK_DEPTH_TYPE)DISPLAY_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)DISPLAY_TASK_PRIORITY,
        (TaskHandle_t *)&display_task_handle);
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

/**
 * @description: 获取姿态数据：角度、编码器值
 * @param {void} *pvParameters
 * @return {*}
 */
void App_Task_GetData(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    while(1)
    {
        App_Car_GetAngle();
        /* 获取完数据，通知pid控制任务 */
        xTaskNotifyGive(pid_task_handle);
        vTaskDelayUntil(&pxPreviousWakeTime,10);
    }

}

/**
 * @description: 进行PID控制的任务
 * @param {void} *pvParameters
 * @return {*}
 */
void App_Task_PID(void *pvParameters)
{
    while(1)
    {
        /* 等待“获取数据任务”的通知 */
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        App_Car_PID();
    }
}

/**
 * @description: OLED显示任务
 * @param {void} *pvParameters
 * @return {*}
 */
void App_Task_Display(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while(1)
    {
        App_Car_Display();
        vTaskDelayUntil(&pxPreviousWakeTime,50);
    }

}
