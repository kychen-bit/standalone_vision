/**
  ******************************************************************************
  * @file       grab_task.h
  * @brief      抓取任务: 控制两个 DM4310 电机实现机械臂末端抓取
  *             位置模式(POS, DM4310 位置-速度): 直发角度目标+限速 2rad/s;
  *             原位置+速度级联 PID 已注释保留(不再输出扭矩); 电机硬件零位已置竖直
  * @note       电机主控ID/反馈ID 在 CAN_receive.h 中定义
  ******************************************************************************
  */
#ifndef GRAB_TASK_H
#define GRAB_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "dm_motor.h"
#include "pid.h"
#include "user_lib.h"

//任务初始化延时
#define GRAB_TASK_INIT_TIME 200

//抓取任务控制间隔 1ms
#define GRAB_CONTROL_TIME_MS 1
#define GRAB_CONTROL_TIME 0.001f

//位置模式: 直发角度时 DM4310 的最大关节速度限幅 (rad/s)
#define GRAB_POS_SPEED_LIMIT 2.0f

//==================== 自动使能管理 ====================
//达妙 MIT 反馈第0字节高4位为状态(state); 0=失能, 非0=已使能/其它状态
#define GRAB_DM_STATE_DISABLED  0
//主循环: 失能时补发使能命令的间隔(ms); 想每周期都补发就设 1
#define GRAB_ENABLE_RETRY_MS    50u
//初始化阶段: 等待电机使能的最长阻塞时间(ms); 超时后交给主循环继续补发
#define GRAB_ENABLE_INIT_TIMEOUT_MS 1000u

//==================== PID 参数: 电机1 / 电机2 两套独立整定 ====================
//位置环 PID 电机1 (输出为速度设定 rad/s)
#define GRAB_M1_POS_PID_KP       10.0f
#define GRAB_M1_POS_PID_KI       0.0f
#define GRAB_M1_POS_PID_KD       0.0f
#define GRAB_M1_POS_PID_MAX_OUT  5.0f    //限速: 关节最大转速 ≈ ±1.0 rad/s
#define GRAB_M1_POS_PID_MAX_IOUT 5.0f

//位置环 PID 电机2
#define GRAB_M2_POS_PID_KP       10.0f
#define GRAB_M2_POS_PID_KI       0.0f
#define GRAB_M2_POS_PID_KD       0.0f
#define GRAB_M2_POS_PID_MAX_OUT  5.0f    //限速: 关节最大转速 ≈ ±1.0 rad/s
#define GRAB_M2_POS_PID_MAX_IOUT 5.0f

//速度环 PID 电机1 (输出为转矩 Nm)
#define GRAB_M1_SPEED_PID_KP       1.0f
#define GRAB_M1_SPEED_PID_KI       0.0f
#define GRAB_M1_SPEED_PID_KD       0.0f
#define GRAB_M1_SPEED_PID_MAX_OUT  5.0f   //限矩: 最大转矩 ±2.5 Nm
#define GRAB_M1_SPEED_PID_MAX_IOUT 2.0f

//速度环 PID 电机2
#define GRAB_M2_SPEED_PID_KP       1.0f
#define GRAB_M2_SPEED_PID_KI       0.0f
#define GRAB_M2_SPEED_PID_KD       0.0f
#define GRAB_M2_SPEED_PID_MAX_OUT  5.0f   //限矩: 最大转矩 ±2.5 Nm
#define GRAB_M2_SPEED_PID_MAX_IOUT 2.0f

//(2026-09-05 软件限位已禁用: 不再夹 θ, 由机构硬限位保护; 宏保留备用,
// 要恢复就在 grab_set_control 里加回 constrain)
//两个关节“统一机械角度 θ”的原限幅 (rad) —— 注意是 θ, 不是原始反馈(raw)
#define GRAB_M1_ANGLE_MIN (-1.5f)
#define GRAB_M1_ANGLE_MAX ( 1.5f)
#define GRAB_M2_ANGLE_MIN (-1.5f)
#define GRAB_M2_ANGLE_MAX ( 1.5f)

//==================== 遥控 / 自动 配置 ====================
//模式选择拨杆 (遥控 s[] 索引, 与 remote_control.h 一致):
//  S1(s[1], CH6 两档), S2(s[2], CH7 三档)
#define GRAB_REMOTE_SW_INDEX  1   //S1 拨下 -> REMOTE 遥控
#define GRAB_AUTO_SW_INDEX    2   //S2 拨下 -> AUTO 自动

//REMOTE: 摇杆通道与末端速度灵敏度 (末端速度 mm/s = ch * SEN)
#define GRAB_RC_X_CH          0   //CH0 -> 末端 X (沿两电机连线方向)
#define GRAB_RC_Y_CH          1   //CH1 -> 末端 Y (竖直上/下)
#define GRAB_RC_X_SEN         0.1f   //原来0.4 太快, 降10倍
#define GRAB_RC_Y_SEN         0.1f   //原来0.4 太快, 降10倍

//REMOTE 末端方向符号(Watch 可改 ±1, 现场判方向; 不影响 AUTO)
extern float grab_rc_x_sign;   //CH0(左右)对应“五连杆 x 轴(电机连线)”的符号
extern float grab_rc_y_sign;   //CH1(上下)对应“竖直 y 轴”的符号

//AUTO: 取放任务状态机参数 (关节路径表 s_auto_path 在 grab_task.c)
#define GRAB_AUTO_IDLE_MS     300u   //进入AUTO后准备时间(ms)再开始本轮
#define GRAB_AUTO_SPEED       0.6f   //关节插值速度上限 (rad/s), 每段按此定匀速时长
#define GRAB_AUTO_SEG_MIN_MS  300u   //单段插值最短时长(ms)
#define GRAB_AUTO_ARRIVE_RAD  0.05f  //到位判定误差(rad)
#define GRAB_AUTO_ARRIVE_MS   60u    //到位稳定时间(ms)
#define GRAB_AUTO_HOLD_MS     400u   //取/放点保持时间(ms) (夹爪动作点, 后续接爪)
#define GRAB_AUTO_MID_HOLD_MS 60u    //中间路径点短停(ms)
#define GRAB_AUTO_TIMEOUT_MS  2500u  //到位等待超时(ms), 超时强制继续防卡死

//==========================================================================
// 舵机 PWM (周期 20ms=50Hz, 见 tim.c)
//   idx0 = 夹爪舵机   TIM1_CH2 / PE11   (夹紧/松开)
//   idx1 = 伸缩舵机   TIM1_CH3 / PE13   (伸缩/回缩)
// 改引脚/TIM 步骤: ①CubeMX 把需要的 TIM 通道脚配成对应 AF;
//                  ②改下面 4 个宏(定时器句柄 + 通道)即可, 代码不用动。
// 占空比 0~100%; 角度 0~180°, 按脉宽线性映射(默认 0.5ms~2.5ms, 可按舵机改)
//==========================================================================
//舵机角色索引 (给 grab_servo_set_duty/set_angle 的 idx 用)
#define GRAB_SERVO_GRIPPER_IDX   0   //夹爪舵机 (TIM1_CH2, PE11)
#define GRAB_SERVO_EXTEND_IDX    1   //伸缩舵机 (TIM1_CH3, PE13)

#define GRAB_SERVO0_TIM_HANDLE   (&htim1)         //夹爪(舵机0) 定时器句柄
#define GRAB_SERVO0_TIM_CHANNEL  (TIM_CHANNEL_2)  //夹爪 通道 (TIM1_CH2, PE11)
#define GRAB_SERVO1_TIM_HANDLE   (&htim1)         //伸缩(舵机1) 定时器句柄
#define GRAB_SERVO1_TIM_CHANNEL  (TIM_CHANNEL_3)  //伸缩 通道 (TIM1_CH3, PE13)

//PWM 时间基准 (ms) —— 角度换算用, 必须和 CubeMX 里实际周期一致!
#define GRAB_SERVO_PERIOD_MS      20.0f  //周期: 20ms = 50Hz (tim.c 现为 ARR=20000@1MHz)
#define GRAB_SERVO_PULSE_MIN_MS   0.5f   //0°   对应脉宽 (ms)
#define GRAB_SERVO_PULSE_MAX_MS   2.5f   //180° 对应脉宽 (ms)

//==================== 夹爪/伸缩 动作角度(占位默认, 请按实际舵机标定改) ======
//夹爪开/合(舵机 idx=GRAB_SERVO_GRIPPER_IDX); 伸缩伸/缩(idx=GRAB_SERVO_EXTEND_IDX)
#define GRAB_GRIPPER_CLOSE_ANG   5.0f   //夹爪闭合(夹住物料)
#define GRAB_GRIPPER_OPEN_ANG    90.0f  //夹爪张开(放下物料)
#define GRAB_EXTEND_OUT_ANG      90.0f  //伸缩伸出(够到低处台面)
#define GRAB_EXTEND_IN_ANG       20.0f  //伸缩缩回(当前上电默认值)

/**
  * @brief          初始化舵机: 启动 TIM1_CH2/CH3 PWM 输出 (不动角度, 角度由调用者设置)
  * @note           需在 MX_TIM1_Init 之后调用一次; 若已在别处 Start 过 CH2/CH3 可不再调
  */
extern void grab_servo_init(void);

/**
  * @brief          设置舵机占空比 (0~100%)
  * @param[in]      idx: GRAB_SERVO_GRIPPER_IDX(夹爪)/GRAB_SERVO_EXTEND_IDX(伸缩); duty: 0~100%
  */
extern void grab_servo_set_duty(uint8_t idx, fp32 duty);

/**
  * @brief          设置舵机角度 (0~180°)
  * @param[in]      idx: GRAB_SERVO_GRIPPER_IDX(夹爪)/GRAB_SERVO_EXTEND_IDX(伸缩); angle: 0~180°
  */
extern void grab_servo_set_angle(uint8_t idx, fp32 angle);

//==========================================================================
// 抓取末端运行模式 (默认 ZERO):
//   ZERO   默认: 零力(电机无力, 扭矩发0, 可手掰; 不响应遥控/自动)
//   REMOTE 遥控: S1拨下 -> CH0/CH1 速度控制末端 XY, IK解算驱动两电机
//   AUTO   自动: S2拨下 -> 真实取放状态机(关节插值; 流程方向见 grab_auto_flow)
// 模式优先级: AUTO(S2下) > REMOTE(S1下) > ZERO(默认)
// 说明: 上电不回位/不初始化角度(太危险), 直接按遥控当前状态动作;
//       想回 θ=0 初始位可手动进 REMOTE 或用 Watch 改目标
//==========================================================================
typedef enum
{
    GRAB_MODE_ZERO = 0,   //ZERO(默认): 零力, 电机扭矩发0(可手掰)
    GRAB_MODE_REMOTE,     //遥控: S1下, CH0/CH1 控制末端XY
    GRAB_MODE_AUTO,       //自动: S2下, 真实取放状态机
} grab_mode_e;

//AUTO 取放状态机状态 (真实取放任务)
//   IDLE(准备) -> MOVE(插值移动+等待到位) -> HOLD(保持) -> DONE(放完)
typedef enum
{
    GRAB_AUTO_IDLE = 0,   //准备(刚进AUTO短暂停留后开始一轮)
    GRAB_AUTO_MOVE,       //沿关节角线性匀速插值移动, 并等待物理到位
    GRAB_AUTO_HOLD,       //到位保持(取/放端点=夹爪动作, 中间点短停)
    GRAB_AUTO_DONE,       //本轮完成(停在放点位)
} grab_auto_state_e;

//AUTO 流程方向 (底盘/上层可设置; 底盘未就绪时用 Keil Watch 改 grab_auto_flow)
//路径: 抓取侧(θ=0竖直, 末端=目标物) <-> 所选置物盘(终点按 grab_auto_tray)
typedef enum
{
    GRAB_FLOW_GRAB_TO_TRAY = 0,  //抓取侧抓物 -> 置物盘放 (默认)
    GRAB_FLOW_TRAY_TO_GRAB,      //置物盘抓 -> 抓取侧放
} grab_auto_flow_e;

extern uint8_t grab_auto_flow;       //当前流程方向 (Watch 可改)
extern uint8_t grab_auto_tray;       //当前置物盘号 1/2/3 (Watch/底盘可改, 默认盘1)
extern uint8_t grab_auto_dbg_state;  //调试: 当前AUTO状态
extern int16_t grab_auto_dbg_wp;     //调试: 当前路径点下标

/* ==== AUTO 外部取放执行器(真序列)标定量 —— Keil Watch 均可改 ==== */
extern fp32 grab_ext_pick_t1;   /* 抓取侧(场地料/环)端点 θ1 rad: 现场遥控夹爪对准后读 motor[0].angle 填入 */
extern fp32 grab_ext_pick_t2;   /* 抓取侧端点 θ2 rad: 读 motor[1].angle 填入 */
extern fp32 grab_ext_way[3][2]; /* 翻越三姿态 (θ1,θ2): 默认 (1.2,-1.2)(π/2,-π/2)(2.0,-2.0) */

typedef struct
{
    motor_t *motor;           //达妙4310电机指针
    pid_type_def pos_pid;     //位置环 PID
    pid_type_def speed_pid;   //速度环 PID
    fp32 angle_set;           //目标关节角度 rad
    fp32 angle;               //当前关节角度反馈 rad
    fp32 speed_set;           //目标关节速度 rad/s
    fp32 speed;               //当前关节速度反馈 rad/s
    fp32 give_torque;         //输出转矩 Nm
    fp32 zero;                //软件零点: 主动杆竖直(θ=0)时电机反馈 raw(rad), 运行期可改
} grab_motor_t;

typedef struct
{
    grab_mode_e grab_mode;    //抓取末端三种运动状态
    grab_mode_e last_grab_mode;
    grab_motor_t motor[2];    //两个关节电机控制参数
    fp32 ep_x;                //当前末端 P 的 x (mm), 每周期由 FK 计算
    fp32 ep_y;                //当前末端 P 的 y (mm)
    fp32 tgt_x;               //末端目标 P 的 x (mm): REMOTE/AUTO 由它 IK 解算到电机角度
    fp32 tgt_y;               //末端目标 P 的 y (mm)
    uint8_t tgt_valid;        //末端目标有效标志
    uint8_t init_flag;        //初始化标志
} grab_control_t;

/**
  * @brief          抓取任务
  * @param[in]      pvParameters: 无
  * @retval         none
  */
extern void grab_task(void const *pvParameters);

/**
  * @brief          设置单个关节目标统一机械角度 θ (rad)
  * @param[in]      idx: 关节索引 0~1
  * @param[in]      theta: 目标统一机械角度 rad (θ=0 = 主动杆竖直向上)
  * @retval         none
  */
extern void grab_set_joint_angle(uint8_t idx, fp32 theta);

/**
  * @brief          同时设置两个关节的目标统一机械角度
  * @param[in]      theta1: 关节1目标 θ rad
  * @param[in]      theta2: 关节2目标 θ rad
  * @retval         none
  */
extern void grab_set_grab(fp32 theta1, fp32 theta2);

/**
  * @brief          使能抓取电机
  * @retval         none
  */
extern void grab_enable(void);

/**
  * @brief          失能抓取电机 (断电)
  * @retval         none
  */
extern void grab_disable(void);

/**
  * @brief          软件零点标定: 把当前姿态记为 θ=0
  * @note           先把机构摆到参考姿态(主动杆竖直向上), 再调用本函数;
  *                 内部把当前电机反馈(raw)记录为 motor[i].zero, 之后该姿态 θ=0
  * @retval         none
  */
extern void grab_zero_pos(void);

//==========================================================================
// 五连杆机械结构与坐标系 —— 必须严格按照以下参数实现 (单位统一为 mm/rad)
//--------------------------------------------------------------------------
// 基坐标系: 世界坐标 +x 向右, +y 向上 (右手系)
//   左驱动电机 M1 = (-100, 0)  右驱动电机 M2 = (100, 0)
//   两电机中点 O = (0, 0), 电机中心距 d = 200 mm
// 杆长 (mm), 左右对称:
//   L1 = 140  左主动杆  M1 -> A1(左肘)
//   L2 = 140  右主动杆  M2 -> A2(右肘)
//   L3 = 240  左从动杆  A1 -> P (末端)
//   L4 = 240  右从动杆  A2 -> P (末端)
//   末端执行器 P = 两从动杆交会的共用点
// 角度约定 (2026-09 实测): 两电机反馈均“逆时针增加”, 故统一机械角 θ 取逆时针为正,
//   且 θ = wrap(raw - zero) (zero=竖直位 raw, 见 ZERO_INIT; wrap 参考 RM gimbal 达妙电机
//   angle_to_relative, 回绕到 [-π,π]), θ=0 = 主动杆竖直向上。
//   这样保证: 竖直=0°, 逆时针增大变正, 顺时针减小变负, 总范围从负到正(±π内)。
//   统一机械角度 θ_i 与主动杆在基坐标系的标准角 a_i 的关系:
//     a_i = THETA0_i + θ_i    (逆时针为正 => 标准角随 θ 增大而增大)
//     THETA0_i = 保存电机零点时主动杆在基坐标系里的标准角(从+x逆时针, rad)
//     默认 π/2 => 保存零点时主动杆竖直向上。若你在别的姿态保存零点, 改它即可。
//==========================================================================
#define GRAB_BASE_X1_MM    (-100.0f)      //左电机中心 x (mm)
#define GRAB_BASE_X2_MM    ( 100.0f)      //右电机中心 x (mm)
#define GRAB_BASE_Y_MM     (   0.0f)      //电机中心 y (mm, 两电机同高)
#define GRAB_L1_MM         140.0f         //左主动杆长 (mm)
#define GRAB_L2_MM         140.0f         //右主动杆长 (mm)
#define GRAB_L3_MM         240.0f         //左从动杆长 (mm)
#define GRAB_L4_MM         240.0f         //右从动杆长 (mm)

//--------------------------------------------------------------------------
// 方向转换层: 实际电机反馈(raw) <-> 统一机械角度 θ
//   θ_i = GRAB_Mi_DIR * raw_i     (2026-09-07: 电机硬件零位已置竖直, 去掉软件零点)
//   DIR 只允许取 +1 / -1, 是左右镜像总开关。
// 实测 2026-09-05: 两电机反馈均“逆时针增加”(与统一角 θ 定义一致) => 默认 +1;
// 若 FK 端点实机验证出现左右镜像, 把这两个宏一起取反即可, 严禁改 FK/IK 公式。
//--------------------------------------------------------------------------
#define GRAB_M1_DIR         1
#define GRAB_M2_DIR         1

// 软件零点: 已废弃(位置模式+硬件零位, raw=0 即竖直位); 保留宏=0 仅作说明
#define GRAB_M1_ZERO_INIT   (0.0f)
#define GRAB_M2_ZERO_INIT   (0.0f)

// 几何零点 (rad): 电机反馈=0(竖直位)时主动杆在基坐标的标准角(默认竖直向上)
#define GRAB_M1_THETA0_STD  (1.570796326794897f)   //π/2 竖直向上
#define GRAB_M2_THETA0_STD  (1.570796326794897f)   //π/2 竖直向上

//--------------------------------------------------------------------------
// 五连杆 FK / IK 接口
//--------------------------------------------------------------------------
/**
  * @brief          FK: (θ1,θ2) -> 末端 P(x,y) [mm]
  * @note           两从动杆圆弧有两个交点时, 默认取 y 较大(向上)的一支
  * @retval          0=成功, 1=无解(机构被完全拉直/重叠)
  */
extern uint8_t grab_fk(fp32 theta1, fp32 theta2, fp32 *x_mm, fp32 *y_mm);

/**
  * @brief          FK(带连续性): 交点选与上一次 P 最近的一支, 供实时监控当前末端
  * @retval          0=成功, 1=无解
  */
extern uint8_t grab_fk_cont(fp32 theta1, fp32 theta2, fp32 *x_mm, fp32 *y_mm);

/**
  * @brief          IK: (x,y) -> (θ1,θ2)
  * @note           多解时按与 (cur_theta1,cur_theta2) 最连续的一组选择;
  *                 目标不可达/全部超限时返回 1, 不输出非法角度
  * @retval          0=成功, 1=无解
  */
extern uint8_t grab_ik(fp32 x_mm, fp32 y_mm,
                       fp32 cur_theta1, fp32 cur_theta2,
                       fp32 *theta1, fp32 *theta2);

/**
  * @brief          请求末端移动到 (x,y) mm
  * @note           IK 成功后才把两电机目标角度(raw)写入 angle_set;
  *                 IK 失败则不改任何目标, 返回非 0
  * @retval          0=已设置, 1=IK 失败(不可达)
  */
extern uint8_t grab_set_endpoint(fp32 x_mm, fp32 y_mm);

/**
  * @brief          设定末端目标 (x,y) mm, 存入 tgt_x/tgt_y 并立即 IK 解算到两电机角度设定值
  * @note           目标源 = grab_control.tgt_x/tgt_y; REMOTE/AUTO 会持续跟踪, ZERO 不跟随
  * @retval          0=可解并已写入目标, 1=目标不可达(未写入)
  */
extern uint8_t grab_set_target(fp32 x_mm, fp32 y_mm);

/**
  * @brief          读取当前末端位置 (mm), 由当前反馈经 FK 计算
  */
extern void grab_get_endpoint(fp32 *x_mm, fp32 *y_mm);

/**
  * @brief          清除 FK 交点连续性历史 (标定零点后建议调用一次)
  */
extern void grab_fk_reset_prev(void);

/**
  * @brief          FK/IK 往返自检: 多组 (θ1,θ2)->FK->IK->FK 验证
  * @retval          0=全部通过, 1=失败 (先验证通过再接入运动控制)
  */
extern uint8_t grab_kin_selftest(void);

/**
  * @brief          方向转换: 电机反馈角度(raw) -> 统一机械角度 θ
  */
extern fp32 grab_motor_to_theta(uint8_t idx, fp32 raw);

/**
  * @brief          方向转换: 统一机械角度 θ -> 电机反馈角度(raw)
  */
extern fp32 grab_theta_to_motor(uint8_t idx, fp32 theta);

extern grab_control_t grab_control;

//FK/IK 自检结果 (无串口打印, 供 Keil Watch 查看)
extern uint8_t grab_st_pass;        //1=自检通过
extern uint8_t grab_st_fail_code;   //失败原因: 0=无,1=FK无解,2=IK失败,3=闭环无解,4=误差超限
extern fp32    grab_st_err_ang;     //最大角度往返误差 (rad)
extern fp32    grab_st_err_pos;     //最大位置往返误差 (mm)

//调试用 (Keil Watch): 单关节角度阶跃测试
extern uint8_t grab_dbg_ang_en;     //1=启用: 无视模式给单电机阶跃目标
extern uint8_t grab_dbg_ang_idx;    //0=M1, 1=M2
extern fp32    grab_dbg_ang_val;    //目标统一机械角 θ(rad)

#endif
