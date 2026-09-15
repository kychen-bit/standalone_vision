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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "dm_motor.h"
#include "zdt_motor.h"
#include "can.h"

/* ==================== 电机 CAN ID 定义(底盘和抓取电机) ==================== */
/* 底盘 4 个张大头电机 -> CAN1 电机ID, 注意 ExtId = 电机 CAN ID */
#define CHASSIS_M1_CAN_ID   0x01
#define CHASSIS_M2_CAN_ID   0x02
#define CHASSIS_M3_CAN_ID   0x03
#define CHASSIS_M4_CAN_ID   0x04
/* 抓取 2 个达妙 4310 (机械臂) -> CAN2 电机ID, MIT 模式 */
/* 抓取电机接收ID (对应电机上报 StdId) */
#define GRAB_M1_CAN_ID      0x01
#define GRAB_M2_CAN_ID      0x02
/* 抓取电机发送ID (对应电机控制 StdId) */
#define GRAB_M1_MASTER_ID   0x11
#define GRAB_M2_MASTER_ID   0x12

/* 底盘 CAN 总线定义 */
#define CHASSIS_DM_CAN      hcan1
#define GRAB_DM_CAN         hcan2

/**
  * @brief          获取底盘电机指针 (张大头, 0~3)
  * @param[in]      i: 轮子索引 0~3
  * @retval         电机结构体指针
  */
extern zdt_motor_t *get_chassis_motor_point(uint8_t i);

/**
  * @brief          获取抓取电机指针 (达妙4310, 0~1)
  * @param[in]      i: 电机索引 0~1
  * @retval         电机结构体指针
  */
extern motor_t *get_grab_motor_point(uint8_t i);

#endif
