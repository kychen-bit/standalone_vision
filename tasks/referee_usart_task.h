/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"
#include "fifo.h"
#include "protocol.h"
//#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
#define REFEREE_USART_RX_BUF_LENGHT 512

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);


/* 裁判系统串口双缓冲区 */
extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* 裁判系统接收数据队列 */
extern fifo_s_t Referee_FIFO;
extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
extern unpack_data_t Referee_Unpack_OBJ;
#endif
