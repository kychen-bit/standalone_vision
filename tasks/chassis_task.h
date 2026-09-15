/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      底盘控制任务, 包括模式切换、遥控控制、反馈里程计和电机输出
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "user_lib.h"

// 底盘任务初始化时间
#define CHASSIS_TASK_INIT_TIME 200

// 底盘控制周期 1ms
#define CHASSIS_CONTROL_TIME_MS 1
#define CHASSIS_CONTROL_TIME 0.001f

// ==================== 遥控器通道 -> 底盘方向映射 ====================
// 大疆遥控器通道布局 (日本手 Mode 2):
//   左摇杆:  CH2(左右=yaw)  CH3(上下=油门)
//   右摇杆:  CH0(左右=横移)  CH1(上下=前后)
// 底盘坐标系: X前 Y左  Z上(旋转逆时针为正)
//
// 映射关系:
//   CH0 (右摇杆左右) -> vy_set (底盘Y方向, 左移/右移) 已取反
//   CH1 (右摇杆上下) -> vx_set (底盘X方向, 前进/后退)
//   CH2 (左摇杆左右) -> 未使用
//   CH3 (左摇杆上下) -> wz_set (底盘旋转, 逆时针/顺时针)
// ====================================================================

#define CHASSIS_X_CHANNEL  1
#define CHASSIS_Y_CHANNEL  0
#define CHASSIS_WZ_CHANNEL 3

// 遥控器死区
#define CHASSIS_RC_DEADLINE 10
// 遥控器 -> 底盘速度
#define CHASSIS_VX_RC_SEN 0.01f
#define CHASSIS_VY_RC_SEN 0.01f
#define CHASSIS_WZ_RC_SEN 0.02f
// 遥控旋转方向: 1 = 保持原映射, -1 = 反向 (用于旋向不符合预期时)
#define CHASSIS_WZ_RC_SIGN -1.0f

// 底盘尺寸参数 (单位 m, 实测: R=40mm a=95mm b=94mm)
#define CHASSIS_HALF_WHEELBASE  0.095f  // 半轴距 a (前/后轮心到中心)
#define CHASSIS_HALF_WHEELTRACK 0.094f  // 半轮距 b (左/右轮心到中心)
#define CHASSIS_WHEEL_RADIUS    0.040f  // 轮子半径 R (线速度->角速度 rad/s 换算)

// 最大速度限制
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f   // X方向 m/s
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f   // Y方向 m/s
#define NORMAL_MAX_CHASSIS_SPEED_Z 8.0f   // 旋转 rad/s

// 最大轮速 (rad/s), 需 >= 平移满速对应轮速 + 旋转分量: 3m/s / 0.04m = 75, 预留旋转余量取 85
#define MAX_WHEEL_SPEED 85.0f

typedef enum
{
    CHASSIS_ZERO_FORCE,       // 无力模式 输出0
    CHASSIS_REMOTE_CONTROL,   // 遥控器控制
    CHASSIS_AUTO,             // 自动模式(暂时未使用)
} chassis_mode_e;

typedef struct
{
    zdt_motor_t *motor;   // 指向ZDT电机对象 (ZDT电机)
    fp32 wheel_speed_set;  // 目标轮速 rad/s
    fp32 wheel_speed;      // 实际轮速 rad/s
    fp32 last_wheel_speed; // 上一周期轮速 (用于加速度计算)
    uint32_t last_upd_ms;  // 上次位置帧所在控制节拍(ms), 用于按真实间隔测速
    fp32 last_angle_deg;   // 上次电机累计角度(°) 用于位置差分里程计
    uint8_t odom_ready;    // 里程计基准已建立标志
    int8_t rev;            // 正反转标志 1/-1
} chassis_wheel_t;

typedef struct
{
    const RC_ctrl_t *chassis_RC;  // 遥控器数据
    const fp32 *INS_accel;        // IMU加速度指针 (x/y/z, m/s²)
    const fp32 *INS_angle;        // IMU欧拉角指针 (yaw/pitch/roll, rad)
    const fp32 *INS_gyro;         // IMU角速度指针 (x/y/z, rad/s), 航向环微分用
    chassis_mode_e chassis_mode;  // 底盘模式
    chassis_wheel_t wheel[4];     // 四个轮子

    // 底盘尺寸 (运动学解算)
    fp32 half_wheelbase;          // 半轴距 a (m)
    fp32 half_wheeltrack;         // 半轮距 b (m)
    fp32 wheelbase_sum;           // D = a + b
    fp32 wheel_radius;            // 轮子半径 R (m)

    // 底盘运动状态 (麦克纳姆轮解算)
    fp32 vx, vy, wz;              // m/s, m/s, rad/s
    // 目标速度
    fp32 vx_set, vy_set, wz_set;
    fp32 vx_max_speed, vx_min_speed;
    fp32 vy_max_speed, vy_min_speed;
    fp32 wz_max_speed, wz_min_speed;

    // 里程计: 位置积分 (轮式里程计积分)
    fp32 position_x;              // X方向位置 m
    fp32 position_y;              // Y方向位置 m
    fp32 position_angle;          // IMU航向角 rad
    fp32 total_travel;            // 累计路程 m (每步 √(Δx²+Δy²) 累加)

    // 恒定航向控制: 车头始终自动拉回 yaw_ref(默认0), 按角度修正 vx/vy/wz
    fp32 yaw_ref;                 // 恒定目标航向 rad (默认0, 改 CHASSIS_HEADING_REF_YAW)
    uint8_t heading_en;           // 1=恒定航向回正(S3=s[3] 上), 0=纯手动无角度修正(S3 下)
} chassis_move_t;

/**
  * @brief          底盘任务
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          将遥控器通道值转换为底盘速度控制向量
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

extern chassis_move_t chassis_move;
#endif
