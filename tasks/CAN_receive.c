/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      该文件有CAN中断函数接收电机数据, CAN发送函数发送电机电流控制电机
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "dm_motor.h"
#include "remote_control.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* 底盘 4 个张大头电机 (ZDT电机) */
static zdt_motor_t zdt_chassis_motor[4];
/* 抓取 2 个达妙 4310 电机 */
static motor_t dm_grab_motor[2];

/**
  * @brief          hal库CAN中断回调函数, 接收并处理电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  * @note           CAN1处理: 张大头电机 ExtId=0x01~0x04
  *                 CAN2处理: 达妙电机 StdId=0x01,0x02
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance == CAN1)  /* ZDT电机, 扩展帧, ExtId = (motor_id << 8) | packet_num */
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        uint8_t motor_id = (rx_header.ExtId >> 8) & 0xFF;
        switch (motor_id)
        {
            case CHASSIS_M1_CAN_ID:
                zdt_fbdata(&zdt_chassis_motor[0], rx_data, rx_header.ExtId);
                detect_hook(CHASSIS_MOTOR1_TOE);
                break;
            case CHASSIS_M2_CAN_ID:
                zdt_fbdata(&zdt_chassis_motor[1], rx_data, rx_header.ExtId);
                detect_hook(CHASSIS_MOTOR2_TOE);
                break;
            case CHASSIS_M3_CAN_ID:
                zdt_fbdata(&zdt_chassis_motor[2], rx_data, rx_header.ExtId);
                detect_hook(CHASSIS_MOTOR3_TOE);
                break;
            case CHASSIS_M4_CAN_ID:
                zdt_fbdata(&zdt_chassis_motor[3], rx_data, rx_header.ExtId);
                detect_hook(CHASSIS_MOTOR4_TOE);
                break;
            default:
                break;
        }
    }
    else if (hcan->Instance == CAN2)  /* 达妙电机, 标准帧ID */
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        switch (rx_header.StdId)
        {
            case GRAB_M1_CAN_ID:
                dm_fbdata(&dm_grab_motor[0], rx_data);
                detect_hook(GRAB_MOTOR1_TOE);
                break;
            case GRAB_M2_CAN_ID:
                dm_fbdata(&dm_grab_motor[1], rx_data);
                detect_hook(GRAB_MOTOR2_TOE);
                break;
            default:
                break;
        }
    }
}




/**
  * @brief          获取底盘电机指针 (张大头, 0~3)
  * @param[in]      i: 轮子索引 0~3
  * @retval         电机结构体指针
  */
zdt_motor_t *get_chassis_motor_point(uint8_t i)
{
    if (i > 3)
    {
        i = 3;
    }
    return &zdt_chassis_motor[i];
}

/**
  * @brief          获取抓取电机指针 (达妙4310, 0~1)
  * @param[in]      i: 电机索引 0~1
  * @retval         电机结构体指针
  */
motor_t *get_grab_motor_point(uint8_t i)
{
    if (i > 1)
    {
        i = 1;
    }
    return &dm_grab_motor[i];
}
