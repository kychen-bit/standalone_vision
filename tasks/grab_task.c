/**
  ******************************************************************************
  * @file       grab_task.c
  * @brief      抓取任务: 控制两个 DM4310 电机实现机械臂末端抓取
  *             位置模式(POS, DM4310 位置-速度): 直发角度目标+限速 2rad/s;
  *             原位置+速度级联 PID 已注释保留(不再输出扭矩); 电机硬件零位已置竖直
  *             控制流程:
  *             模式设置 -> 设置目标 -> 反馈更新 -> 目标约束 -> PID计算(位置+速度级联) -> 发送CAN
  ******************************************************************************
  */
#include "grab_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "dm_motor.h"
#include "pid.h"
#include "remote_control.h"
#include "detect_task.h"
#include "auto_task.h"
#include "jetson_task.h"
#include <math.h>
#include <stdio.h>

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;   //舵机 PWM (TIM1_CH2/CH3), tim.c 里定义

//抓取控制全局变量
grab_control_t grab_control;

//FK/IK 自检结果 (无串口打印, 用 Keil Watch 查看)
uint8_t grab_st_pass;        //1=自检通过
uint8_t grab_st_fail_code;   //失败原因: 0=无, 1=FK无解, 2=IK失败, 3=闭环无解, 4=往返误差超限
fp32    grab_st_err_ang;     //最大角度往返误差 (rad)
fp32    grab_st_err_pos;     //最大位置往返误差 (mm)

//调试用 (Keil Watch): 单关节角度阶跃测试。
//  grab_dbg_ang_en=1 时, 无视当前模式, 直接把 grab_dbg_ang_idx 电机的目标设为 grab_dbg_ang_val(θ rad)
uint8_t grab_dbg_ang_en;     //0=关闭(默认), 1=启用阶跃
uint8_t grab_dbg_ang_idx;    //0=M1, 1=M2
fp32    grab_dbg_ang_val;    //目标统一机械角 θ(rad)

//位置模式+FK 验证 Watch (2026-09-07)
float   grab_dbg_raw1 = 0.0f, grab_dbg_raw2 = 0.0f;  /* 刚发到电机的目标角度 raw(rad) */
float   grab_dbg_fk_x = 0.0f, grab_dbg_fk_y = 0.0f;   /* 当前 θ 经 FK 解的末端(mm) */
uint8_t grab_dbg_fk_ok = 0u;                          /* 1=FK 有解 */
uint8_t grab_dbg_run_kin = 0u;                        /* Watch 置1: 跑一次 FK/IK 自检后自动清0 */

static const RC_ctrl_t *grab_rc = NULL;   //遥控器指针 (任务启动时赋值)

//自动使能管理: 1=已手动失能(grab_disable), 期间不再自动补发使能
static uint8_t s_enable_off = 0;

//AUTO 取放状态机内部状态 (状态见 grab_auto_state_e)
static uint8_t  s_auto_state = GRAB_AUTO_IDLE;   //当前状态
static int8_t   s_auto_step = 1;                 //路径推进方向: +1 抓->盘, -1 盘->抓
static int16_t  s_auto_wp = 0;                   //当前目标路径点下标
static int16_t  s_auto_first = 0;                //本轮起点(取)下标
static int16_t  s_auto_final = 0;                //本轮终点(放)下标
static uint16_t s_auto_idle_ms = 0;              //IDLE 等待计时(ms)
static uint16_t s_auto_timer_ms = 0;             //当前状态计时(ms)
static uint16_t s_auto_arrive_ms = 0;            //到位稳定计数(ms)
static uint16_t s_auto_seg_ms = 0;               //当前插值段时长(ms)
static fp32     s_auto_a0 = 0.0f;                //插值段起点 θ1
static fp32     s_auto_a1 = 0.0f;                //插值段起点 θ2

//AUTO 流程方向 (0=抓取侧→置物盘, 1=置物盘→抓取侧); 底盘就绪后由其设置, 目前 Keil Watch 改
uint8_t grab_auto_flow = GRAB_FLOW_GRAB_TO_TRAY;
//AUTO 当前置物盘号 (1/2/3), 每轮终点=对应盘关节角; Watch/底盘改, 默认盘1
uint8_t grab_auto_tray = 1;
//Watch: 置1 让 AUTO 跑“老的三盘演示”(不进底盘总控); 0=等总控 auto_grab_req 驱动
uint8_t grab_auto_legacy_demo = 0;
//(2026-09-06 真序列标定) 抓取侧(场地料/环)端点 θ; 翻越三姿态 —— Watch 可改
float grab_ext_pick_t1 = 0.0f;   //抓取侧端点 θ1 rad (默认0=竖直; 现场遥控对准后读 motor[0].angle 填入)
float grab_ext_pick_t2 = 0.0f;   //抓取侧端点 θ2 rad
/* Watch 一键标定“抓取端点”(上面抓取用): REMOTE 把末端对准料后, Watch 置 grab_cal_capture=1
 * -> 自动把当前 θ1/θ2(motor[0].angle/[1].angle) 存进 grab_ext_pick_t1/t2 并清 0。
 * 只允许非 AUTO(避免整场跑时误写); AUTO 运输/翻越/盘位已按固定角用(见 s_auto_tray_pose)。 */
uint8_t grab_cal_capture = 0u;
/* [抓取区视觉纠偏 —— 五连杆 FK/IK 结算, 2026-09-08; 运输仍走固定角 θ 表]
 * 抓取(上面): nominal 端点 grab_ext_pick_t1/t2 --FK(上支)--> (x0,y0);
 *             叠加视觉平面偏移: x += vfx*grab_ext_vis_fx_sign ; y += grab_ext_vis_dy_mm
 *             --IK--> 本次抓/放端点 θ(不再用旧线性 θ 增益近似)。
 *   vfx 来源: PICK 读 grasp_tar_x(臂前向 mm), PLACE/STACK 读 align_tar_x;
 *   侧向 sd(车左右)由底盘负责(本模块忽略, 需竖直额外修正用 grab_ext_vis_dy_mm)。
 * 默认关闭(抓 nominal); 台架定 vfx 正负后 Watch 置 grab_ext_vis_en=1 启用。 */
uint8_t grab_ext_vis_en     = 0u;
float   grab_ext_vis_fx_sign = 1.0f;  /* vfx 沿机构 +x(伸远) 的符号, 台架定 */
float   grab_ext_vis_dy_mm   = 0.0f;  /* 附加竖直偏移 mm(默认 0) */
float   grab_ext_vis_use1    = 0.0f;  /* Watch: 本次实际用 θ1 - nominalθ1 */
float   grab_ext_vis_use2    = 0.0f;  /* Watch: 本次实际用 θ2 - nominalθ2 */

/* REMOTE 方向符号开关(Watch 可改 ±1; 只影响 REMOTE, 不影响 AUTO):
 *   x: CH0(右摇杆左右)推动时末端沿“五连杆两电机连线(x 轴)”移动的方向;
 *   y: CH1(右摇杆上下)推动时末端沿“竖直(上/下)”移动的方向。
 * 2026-09-08 更新: 之前“目标 y 增、实际 y 反着降”的根因是电机零位设反(镜像)——
 * 零位修正后无需取反, 故 y 恢复 +1(与 x 同)。若某台机/换装后仍感觉反,
 * 直接用 Watch 把对应符号改 -1 即可, 不用改公式。 */
float grab_rc_x_sign = 1.0f;
float grab_rc_y_sign = 1.0f;

/* 五连杆 FK 交点分支(现场核对用; 只影响“位置显示/REMOTE 起点”, 不影响 AUTO θ 表与 IK 控制):
 *   grab_fk_branch : 0=连续(与上次最近, 默认)  1=强制上支(y 大)  2=强制下支(y 小)
 *   grab_fk_up_x/y : 当前姿态的“上支”候选点(Watch)
 *   grab_fk_lo_x/y : 当前姿态的“下支”候选点(Watch)
 * 说明: 同一 θ 下五连杆可能有两个交点解(臂在下方交叉时, 上≈拱起/下≈探底)。
 * 现场把实机末端与上/下两候选比一下, 哪个对就用 grab_fk_branch 锁死哪个。 */
uint8_t grab_fk_branch = 0u;
float   grab_fk_up_x = 0.0f, grab_fk_up_y = 0.0f;
float   grab_fk_lo_x = 0.0f, grab_fk_lo_y = 0.0f;
float grab_ext_way[3][2] =      //末端从一侧翻到另一侧(置物盘侧)的三组中间姿态
{
    {  1.2f,          -1.2f          },   //姿态1
    {  1.5707963f,    -1.5707963f    },   //姿态2 (π/2,-π/2)
    {  2.0f,          -2.0f          },   //姿态3
};
//AUTO 调试 (Watch): 当前状态 / 当前路径点下标
uint8_t grab_auto_dbg_state = GRAB_AUTO_IDLE;
int16_t grab_auto_dbg_wp = 0;

/* ==================== 单点“视觉抓取自测”(车静止, Watch 触发) ===========
 * 用途: 不进 AUTO/不导航, 定点测“视觉→机械臂合爪→翻越→落盘”这一条链路。
 * 用法(Keil Watch):
 *   0) 车摆好: 物料在机械臂可达、夹爪端相机能看到的位置(≈停车点姿态)。
 *   1) 真实视觉: 先让 Jetson 会话就绪(jetson.phase==TASK_READY 即已 START/READY/
 *      TASK_PLAN); 纯机构自测: 先置 grab_vis_skip_vision=1 不接视觉直接跑动作。
 *   2) Watch 置 grab_vis_test=1 触发一轮: 张爪→REQ PICK→等 GRASP_READY(取视觉
 *      纠偏 grasp_tar_x/y)→按 nominal+偏移 端点合爪→翻越三姿态→落所选盘
 *      (grab_vis_tray)张开。完成后 grab_vis_test 自动清 0; grab_vis_dbg 看阶段。
 * 说明: 与底盘 AUTO 互斥(GRAB_MODE_AUTO 不跑; 进 AUTO 自动取消);
 *      视觉纠偏是否生效看 grab_ext_vis_en 与 grab_ext_vis_fx_sign(默认关=抓 nominal 点)。
 * ======================================================================== */
uint8_t  grab_vis_test = 0u;          /* Watch: 0=关; 置1 触发一轮(完成/失败自清0) */
uint8_t  grab_vis_test_color = 1u;    /* 待抓物料颜色 1..6(REQ 用, 真实视觉) */
uint8_t  grab_vis_tray = 1u;          /* 落车盘号 1..3 */
uint8_t  grab_vis_skip_vision = 0u;   /* 1=跳过 Jetson, 直接按 nominal 跑动作(纯机构自测) */
uint8_t  grab_vis_use_exec = 1u;      /* 真实视觉: 1=按协议补 EXEC+DONE, 会话干净 */
uint8_t  grab_vis_dbg = 0u;           /* Watch: 0 idle / 1 等许可 / 2 执行中 */
uint8_t  grab_vis_sw_en = 1u;         /* 1=允许 S[0](CH5)拨下沿触发一轮单测(不影响 AUTO); 0=只用 Watch */
uint8_t  grab_vis_use_xy = 0u;        /* 1=单测也按“视觉 xy 位移”FK/IK 纠偏; 0(默认)=只要“确认抓取标志位”就按 nominal 直接抓 */
#define GRAB_VIS_TEST_TIMEOUT_MS 30000u
static uint8_t  s_vt = 0u;             /* 内部阶段 */
static uint8_t  s_vt_req_done = 0u;    /* REQ 已发标志 */
static uint16_t s_vt_wd = 0u;          /* 看门狗 */
static uint32_t s_vt_seq = 0u;         /* 会话序号(seq 文本用) */

static void grab_init(grab_control_t *init);
static void grab_set_mode(grab_control_t *gc);
static void grab_mode_change_control_transit(grab_control_t *gc);
static void grab_remote_handle(grab_control_t *gc);
static void grab_auto_handle(grab_control_t *gc);
static void grab_auto_external(grab_control_t *gc);   /* 底盘总控请求的取放执行器 */
static void grab_auto_external_reset(void);            /* 复位外部执行器(防单测残留带入 AUTO) */
static void grab_vis_test_once(grab_control_t *gc);   /* Watch 单点视觉抓取自测 */
static void grab_vis_sw_trigger(void);                /* S[0] 拨下沿触发单测 */
static void grab_set_control(grab_control_t *gc);
static void grab_feedback_update(grab_control_t *gc);
static void grab_control_loop(grab_control_t *gc);
static void grab_can_send(grab_control_t *gc);
static void grab_enable_manage(void);
static void grab_enable_init_wait(void);

/**
  * @brief          抓取任务, 1ms 周期
  * @param[in]      pvParameters: 无
  * @retval         none
  */
  float temp_ch1,temp_ch2;
void grab_task(void const *pvParameters)
{
    vTaskDelay(GRAB_TASK_INIT_TIME);

    //初始化抓取控制参数 (位置/速度 PID); 内部会取遥控器指针
    grab_init(&grab_control);

    //初始化阶段: while 循环补发使能, 直到反馈状态位显示已使能(state!=0)或超时
    grab_enable_init_wait();

    //舵机: grab_servo_init() 内部会 Start TIM1_CH2/CH3 的 PWM
    //伸缩舵机(CH3, EXTEND)初始化到 20°; 夹爪舵机(CH2, GRIPPER)角度未定, 先不设死(调试用主循环 temp_ch1 调)
    grab_servo_init();
//    grab_servo_set_angle(GRAB_SERVO_GRIPPER_IDX, 0.0f);  //夹爪: 角度未定, 暂不设
    grab_servo_set_angle(GRAB_SERVO_EXTEND_IDX, 20.0f);    //伸缩 -> 20°

    //进入主循环: 上电不回位/不初始化角度, 直接按遥控 S1/S2 决定模式动作
    while (1)
    {
        /* 夹爪手动调试(temp_ch1)只在“非 AUTO 自动流程、且非单测执行中”时接管:
         * AUTO 全程(S2下, 含站间请求空隙)夹爪只由外部执行器在端点合/张;
         * 若在空隙里强设 temp_ch1(=0≈合), 会把刚伸到场地观测位的夹爪提前夹合,
         * 误夹/扰动还没拿到 GRASP/ALIGN 许可的场地料; 单测执行中同样不能让 temp_ch1 接管。 */
        if (grab_control.grab_mode != GRAB_MODE_AUTO && grab_vis_test == 0u && auto_grab_req.active == 0u)
        {
            grab_servo_set_angle(0, temp_ch1);
        }
        //设置抓取控制模式 (S1/S2拨杆刷新 grab_mode, 参考 Big_Emb big_emb_set_mode)
        grab_set_mode(&grab_control);
        //模式切换数据保护: 模式变化时把目标末端初始化为当前位置(防止跳变, 参考 Big_Emb big_emb_mode_change_control_transit)
        grab_mode_change_control_transit(&grab_control);
        //更新电机反馈数据 (位置/速度/末端FK)
        grab_feedback_update(&grab_control);
        /* Watch 一键标定抓取 nominal: grab_cal_capture=1 -> 存当前 θ 到 grab_ext_pick_t1/t2
         * (仅非 AUTO; 存完自动清 0) */
        if (grab_cal_capture != 0u && grab_control.grab_mode != GRAB_MODE_AUTO)
        {
            grab_cal_capture = 0u;
            grab_ext_pick_t1 = grab_control.motor[0].angle;
            grab_ext_pick_t2 = grab_control.motor[1].angle;
        }
        //自动使能管理: 未使能(离线/状态位不对)则补发使能, 已使能则不再发
        grab_enable_manage();
        //S[0] 拨下沿: 触发一次“转盘视觉抓取”单测(不影响 AUTO 完赛逻辑)
        grab_vis_sw_trigger();
        //按当前模式设置关节目标 (ZERO保持 / REMOTE遥控XY / AUTO状态机) + 目标值约束(角度限幅)
        grab_set_control(&grab_control);
        //位置+速度级联 PID 计算
        grab_control_loop(&grab_control);
        //发送电机控制 MIT 指令 (CAN2)
        grab_can_send(&grab_control);

        vTaskDelay(GRAB_CONTROL_TIME_MS);
    }
}

/**
  * @brief          初始化抓取控制参数, 设置电机ID及位置/速度 PID
  * @param[in]      init: "grab_control" 结构体指针
  * @retval         none
  */
static void grab_init(grab_control_t *init)
{
    if (init == NULL)
    {
        return;
    }

    //电机1/电机2 位置/速度 PID 参数 (两套独立整定)
    const static fp32 pos_pid1[3]   = {GRAB_M1_POS_PID_KP, GRAB_M1_POS_PID_KI, GRAB_M1_POS_PID_KD};
    const static fp32 speed_pid1[3] = {GRAB_M1_SPEED_PID_KP, GRAB_M1_SPEED_PID_KI, GRAB_M1_SPEED_PID_KD};
    const static fp32 pos_pid2[3]   = {GRAB_M2_POS_PID_KP, GRAB_M2_POS_PID_KI, GRAB_M2_POS_PID_KD};
    const static fp32 speed_pid2[3] = {GRAB_M2_SPEED_PID_KP, GRAB_M2_SPEED_PID_KI, GRAB_M2_SPEED_PID_KD};

    //默认模式: ZERO (什么都不动, 保持当前位置)
    init->grab_mode = GRAB_MODE_ZERO;
    init->last_grab_mode = init->grab_mode;

    //遥控器指针 (模式由 S1/S2 拨杆决定), 初始化阶段取得
    grab_rc = get_remote_control_point();

    //================ 电机1 (主控ID 0x11 / 反馈ID 0x01) ================
    init->motor[0].motor = get_grab_motor_point(0);
    //位置模式(POS_MODE)使能; 只发角度+限速, 不再用 MIT 扭矩
    motor_set(init->motor[0].motor, GRAB_M1_CAN_ID, POS_MODE, 0.0f, GRAB_POS_SPEED_LIMIT, 0.0f, 0.0f, 0.0f);

    //PID 保留初始化(不再调用, 供注释查看/以后恢复)
    PID_init(&init->motor[0].pos_pid, PID_GIMBAL, pos_pid1, GRAB_M1_POS_PID_MAX_OUT, GRAB_M1_POS_PID_MAX_IOUT, 0.0f, 0.0f);
    PID_init(&init->motor[0].speed_pid, PID_NORMAL, speed_pid1, GRAB_M1_SPEED_PID_MAX_OUT, GRAB_M1_SPEED_PID_MAX_IOUT, 0.0f, 0.0f);

    init->motor[0].angle_set = 0.0f;
    init->motor[0].angle = 0.0f;
    init->motor[0].speed_set = 0.0f;
    init->motor[0].speed = 0.0f;
    init->motor[0].give_torque = 0.0f;
    init->motor[0].zero = 0.0f;   //硬件零位已置竖直(电机内), 不再软件标零

    //================ 电机2 (主控ID 0x12 / 反馈ID 0x02) ================
    init->motor[1].motor = get_grab_motor_point(1);
    //位置模式(POS_MODE)使能; 只发角度+限速, 不再用 MIT 扭矩
    motor_set(init->motor[1].motor, GRAB_M2_CAN_ID, POS_MODE, 0.0f, GRAB_POS_SPEED_LIMIT, 0.0f, 0.0f, 0.0f);

    //PID 保留初始化(不再调用, 供注释查看/以后恢复)
    PID_init(&init->motor[1].pos_pid, PID_GIMBAL, pos_pid2, GRAB_M2_POS_PID_MAX_OUT, GRAB_M2_POS_PID_MAX_IOUT, 0.0f, 0.0f);
    PID_init(&init->motor[1].speed_pid, PID_NORMAL, speed_pid2, GRAB_M2_SPEED_PID_MAX_OUT, GRAB_M2_SPEED_PID_MAX_IOUT, 0.0f, 0.0f);

    init->motor[1].angle_set = 0.0f;
    init->motor[1].angle = 0.0f;
    init->motor[1].speed_set = 0.0f;
    init->motor[1].speed = 0.0f;
    init->motor[1].give_torque = 0.0f;
    init->motor[1].zero = 0.0f;   //硬件零位已置竖直(电机内), 不再软件标零

    init->init_flag = 1;
}

/**
  * @brief          设置单个关节目标统一机械角度 θ (rad)
  * @note           θ=0 表示主动杆竖直向上; angle_set 直接就是 θ
  * @param[in]      idx: 关节索引 0~1
  * @param[in]      theta: 目标统一机械角度 rad
  * @retval         none
  */
void grab_set_joint_angle(uint8_t idx, fp32 theta)
{
    if (idx > 1)
    {
        idx = 1;
    }

    //目标直接是统一机械角度 θ (已去掉软件限位, 由机构硬限位保护)
    grab_control.motor[idx].angle_set = theta;
}

/**
  * @brief          同时设置两个关节目标角度
  * @param[in]      angle1: 关节1目标角度 rad
  * @param[in]      angle2: 关节2目标角度 rad
  * @retval         none
  */
void grab_set_grab(fp32 angle1, fp32 angle2)
{
    grab_set_joint_angle(0, angle1);
    grab_set_joint_angle(1, angle2);
}

/**
  * @brief          抓取电机使能 (位置模式 POS_MODE, CAN2)
  * @note           通过主控ID 0x11/0x12 发送使能命令; 之后任务每周期会由
  *                 grab_enable_manage() 自动检测, 未使能则持续补发使能
  * @retval         none
  */
void grab_enable(void)
{
    s_enable_off = 0;   //允许自动补发使能
    enable_motor_mode(&hcan2, GRAB_M1_MASTER_ID, POS_MODE);
    enable_motor_mode(&hcan2, GRAB_M2_MASTER_ID, POS_MODE);
}

/**
  * @brief          抓取电机失能 (停机, CAN2)
  * @note           失能后自动补发会被关闭(s_enable_off=1), 需再调 grab_enable() 才恢复
  * @retval         none
  */
void grab_disable(void)
{
    s_enable_off = 1;   //关闭自动补发, 防止刚停机又被管理逻辑使能
    disable_motor_mode(&hcan2, GRAB_M1_MASTER_ID, POS_MODE);
    disable_motor_mode(&hcan2, GRAB_M2_MASTER_ID, POS_MODE);
}

/*==========================================================================
 * 舵机 PWM (夹爪): TIM1_CH2/CH3
 * 换引脚/TIM 通道只改 grab_task.h 里的 GRAB_SERVO0/1_TIM_HANDLE 与
 * GRAB_SERVO0/1_TIM_CHANNEL 宏; 脉宽时间基准见 GRAB_SERVO_*_MS 宏。
 *==========================================================================*/
void grab_servo_init(void)
{
    //只启动 CH2/CH3 PWM 输出, 不预设角度(角度由调用者按需设置, 避免上电乱动)
    HAL_TIM_PWM_Start(GRAB_SERVO0_TIM_HANDLE, GRAB_SERVO0_TIM_CHANNEL);
    HAL_TIM_PWM_Start(GRAB_SERVO1_TIM_HANDLE, GRAB_SERVO1_TIM_CHANNEL);
}

void grab_servo_set_duty(uint8_t idx, fp32 duty)
{
    TIM_HandleTypeDef *htim;
    uint32_t ch, arr, ccr;

    if (idx >= 2)
    {
        idx = 1;
    }
    if (duty < 0.0f)      duty = 0.0f;
    else if (duty > 100.0f) duty = 100.0f;

    if (idx == 0)
    {
        htim = GRAB_SERVO0_TIM_HANDLE;
        ch   = GRAB_SERVO0_TIM_CHANNEL;
    }
    else
    {
        htim = GRAB_SERVO1_TIM_HANDLE;
        ch   = GRAB_SERVO1_TIM_CHANNEL;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(htim);
    ccr = (uint32_t)((fp32)arr * duty / 100.0f);
    __HAL_TIM_SET_COMPARE(htim, ch, ccr);
}

void grab_servo_set_angle(uint8_t idx, fp32 angle)
{
    fp32 pulse, duty;

    if (angle < 0.0f)       angle = 0.0f;
    else if (angle > 180.0f) angle = 180.0f;

    //角度 -> 脉宽 -> 占空比
    pulse = GRAB_SERVO_PULSE_MIN_MS +
            (GRAB_SERVO_PULSE_MAX_MS - GRAB_SERVO_PULSE_MIN_MS) * angle / 180.0f;
    duty  = pulse / GRAB_SERVO_PERIOD_MS * 100.0f;
    grab_servo_set_duty(idx, duty);
}

/* 主循环判断是否补发使能: 只看达妙反馈状态位
 *   state == 0 (GRAB_DM_STATE_DISABLED) -> 失能, 需要补发使能 */
static uint8_t grab_motor_need_enable(uint8_t idx)
{
    motor_t *m = (idx == 0) ? get_grab_motor_point(0) : get_grab_motor_point(1);
    return (m->para.state == GRAB_DM_STATE_DISABLED);
}

/* 每周期调用: 电机未处于"已使能"状态就周期性补发使能; 已使能则不再发 */
static void grab_enable_manage(void)
{
    static uint16_t retry_cnt = 0;

    if (s_enable_off)
    {
        return;                    //手动失能过, 不自动补发
    }

    retry_cnt++;
    if (retry_cnt < GRAB_ENABLE_RETRY_MS)
    {
        return;                    //节流, 防止每 1ms 反复刷使能
    }
    retry_cnt = 0;

    if (grab_motor_need_enable(0))
    {
        enable_motor_mode(&hcan2, GRAB_M1_MASTER_ID, POS_MODE);
    }
    if (grab_motor_need_enable(1))
    {
        enable_motor_mode(&hcan2, GRAB_M2_MASTER_ID, POS_MODE);
    }
}

/* 初始化阶段: 阻塞等待两个抓取电机使能。
 * 依据: 反馈状态位 state!=0 视为已使能; state==0/离线都继续补发使能。
 * 带超时: 超时后不再阻塞, 交给主循环 grab_enable_manage() 继续补发。 */
static void grab_enable_init_wait(void)
{
    uint32_t t0 = xTaskGetTickCount();

    s_enable_off = 0;
    //先各补发一次使能(位置模式)
    enable_motor_mode(&hcan2, GRAB_M1_MASTER_ID, POS_MODE);
    enable_motor_mode(&hcan2, GRAB_M2_MASTER_ID, POS_MODE);

    while (1)
    {
        //超时保护: 使能不上也不能把任务卡死
        if ((xTaskGetTickCount() - t0) > GRAB_ENABLE_INIT_TIMEOUT_MS)
        {
            break;
        }
        //两个电机状态位都非失能(已使能) -> 成功
        if (get_grab_motor_point(0)->para.state != GRAB_DM_STATE_DISABLED &&
            get_grab_motor_point(1)->para.state != GRAB_DM_STATE_DISABLED)
        {
            break;
        }
        //还有电机处于失能: 补发使能, 让出 CPU 等反馈更新
        enable_motor_mode(&hcan2, GRAB_M1_MASTER_ID, POS_MODE);
        enable_motor_mode(&hcan2, GRAB_M2_MASTER_ID, POS_MODE);
        vTaskDelay(10);
    }
}

/**
  * @brief          抓取电机零点标定 (已废弃: 电机硬件零位已置竖直)
  * @retval         none
  */
void grab_zero_pos(void)
{
    //(2026-09-07 位置模式 + 硬件零位, 软件不再标零; 保留空函数, 只清一次 FK 连续性历史)
    grab_fk_reset_prev();
}

/**
  * @brief          设置抓取运行模式 (由遥控 S1/S2 拨杆决定)
  * @note           优先级: AUTO(S2下) > REMOTE(S1下) > ZERO(默认)
  *                 遥控断连(DBUS_TOE)时强制 ZERO, 保证安全
  *                 上电不回位/不初始化角度(太危险), 直接按当前遥控决定模式;
  *                 只做决策; 模式切换的数据保护见 grab_mode_change_control_transit()
  * @param[out]     gc: "grab_control" 结构体指针
  * @retval         none
  */
static void grab_set_mode(grab_control_t *gc)
{
    grab_mode_e mode = GRAB_MODE_ZERO;

    if (gc == NULL)
    {
        return;
    }

    //============ 由遥控 S1/S2 决定目标模式 ============
    if (grab_rc == NULL || toe_is_error(DBUS_TOE))
    {
        mode = GRAB_MODE_ZERO;                 //遥控离线 -> ZERO
    }
    else
    {
        //S2(下) -> AUTO; S1(下) -> REMOTE; 否则默认 ZERO
        if (switch_is_down(grab_rc->rc.s[GRAB_AUTO_SW_INDEX]))
        {
            mode = GRAB_MODE_AUTO;
        }
        else if (switch_is_down(grab_rc->rc.s[GRAB_REMOTE_SW_INDEX]))
        {
            mode = GRAB_MODE_REMOTE;
        }
        else
        {
            mode = GRAB_MODE_ZERO;
        }
    }

    gc->grab_mode = mode;
}

/**
  * @brief          模式切换数据保护 (参考 Big_Emb big_emb_mode_change_control_transit)
  * @note           模式真的改变才处理: 清PID积分; 并把目标末端(tgt_x/tgt_y)初始化为
  *                 当前末端位置(由反馈FK得到), 防止进入新模式瞬间跳变; 进入AUTO还复位状态机
  * @param[out]     gc: "grab_control" 结构体指针
  * @retval         none
  */
static void grab_mode_change_control_transit(grab_control_t *gc)
{
    if (gc == NULL)
    {
        return;
    }

    //模式没变: 不处理
    if (gc->last_grab_mode == gc->grab_mode)
    {
        return;
    }

    //模式切换: 清PID积分, 防止跳变
    PID_clear(&gc->motor[0].pos_pid);
    PID_clear(&gc->motor[0].speed_pid);
    PID_clear(&gc->motor[1].pos_pid);
    PID_clear(&gc->motor[1].speed_pid);

    //初始化目标末端 = 当前末端位置 (当前反馈FK得到的 ep_x/ep_y), 防止进入瞬间跳变
    switch (gc->grab_mode)
    {
        case GRAB_MODE_REMOTE:
            //遥控目标从当前位置开始
            gc->tgt_x = gc->ep_x;
            gc->tgt_y = gc->ep_y;
            gc->tgt_valid = 1;
            break;

        case GRAB_MODE_AUTO:
            /* 进 AUTO: 若单测仍在跑则取消并让出(底盘总控接管)。
             * 同时彻底复位外部执行器, 防止被打断的单测把残留路径带进 AUTO。
             * (正常 AUTO 流程 grab_vis_test==0, 本块不执行, AUTO 逻辑不受影响) */
            if (grab_vis_test != 0u)
            {
                grab_vis_test = 0u;
                s_vt = 0u;
                s_vt_wd = 0u;
                auto_grab_req.active = 0u;
                auto_grab_req.done   = 0u;
                grab_auto_external_reset();
            }
            //AUTO: 开始一轮真实取放(流程方向由 grab_auto_flow 决定, Watch/底盘可改)
            s_auto_state = GRAB_AUTO_IDLE;
            s_auto_idle_ms = 0;
            s_auto_timer_ms = 0;
            s_auto_arrive_ms = 0;
            s_auto_wp = 0;
            gc->tgt_x = gc->ep_x;
            gc->tgt_y = gc->ep_y;
            gc->tgt_valid = 1;
            break;

        case GRAB_MODE_ZERO:
        default:
            //ZERO: 不跟踪末端目标
            gc->tgt_valid = 0;
            break;
    }

    gc->last_grab_mode = gc->grab_mode;
}

/**
  * @brief          REMOTE 遥控: 用 CH0/CH1 速度遥控末端 XY (mm)
  * @note           【REMOTE 全程走五连杆解算(IK)】: 摇杆改变 tgt_x/tgt_y, 每周期由 IK 解成 θ;
  *                 当前末端由 FK 显示(ep_x/ep_y)。切勿在此改成“固定角”——固定角只用于 AUTO 的
  *                 运输/翻越/盘位(抓取区若有视觉也用 FK/IK 结算)。
  *                 摇杆回中不动; 目标超出可达范围(IK失败)时停在原地
  * @param[out]     gc: "grab_control" 结构体指针
  * @retval         none
  */
static void grab_remote_handle(grab_control_t *gc)
{
    fp32 dx, dy, nx, ny;
    int16_t chx = 0, chy = 0;

    if (grab_rc != NULL)
    {
        chx = grab_rc->rc.ch[GRAB_RC_X_CH];
        chy = grab_rc->rc.ch[GRAB_RC_Y_CH];
    }

    if (!gc->tgt_valid)
    {
        gc->tgt_x = gc->ep_x;
        gc->tgt_y = gc->ep_y;
        gc->tgt_valid = 1;
    }

    //摇杆位移 -> 末端目标(tgt_x/y)速度积分 (每周期 1ms)
    //方向符号由 grab_rc_x_sign / grab_rc_y_sign 决定(Watch ±1, 现场判方向用)
    dx = (fp32)chx * GRAB_RC_X_SEN * GRAB_CONTROL_TIME * grab_rc_x_sign;
    dy = (fp32)chy * GRAB_RC_Y_SEN * GRAB_CONTROL_TIME * grab_rc_y_sign;

    nx = gc->tgt_x + dx;
    ny = gc->tgt_y + dy;

    //末端目标 -> IK -> 两电机角度设定值; 能解才提交目标(到不了就停原地)
    if (grab_set_endpoint(nx, ny) == 0)
    {
        gc->tgt_x = nx;
        gc->tgt_y = ny;
    }
}

//==========================================================================
// AUTO 真实取放: 关节空间路径插值 (θ1,θ2 rad)
// 抓取侧: 两电机 θ=0 (竖直, 末端夹爪=目标物所在位置)
// 置物盘: 三个盘各有关节终点(实测标定, 见 s_auto_tray_pose), 每轮按 grab_auto_tray 选
// 路径: (0,0) -> (1.2,-1.2) -> (π/2,-π/2) -> 所选盘终点; 抓→盘顺走, 盘→抓反向
//==========================================================================

//三个置物盘的关节终点 (θ1,θ2) rad —— 实测标定值, 别改错
static const fp32 s_auto_tray_pose[][2] =
{
    {  0.818486571f, -2.68575692f  },   //盘1 (ID1 0.818, ID2 -2.686)
    {  2.00427103f,  -1.8397541f   },   //盘2 (ID1 2.004, ID2 -1.840)
    {  2.76350641f,  -0.292232513f },   //盘3 (ID1 2.764, ID2 -0.292)
};
#define GRAB_AUTO_TRAY_NUM  (sizeof(s_auto_tray_pose) / sizeof(s_auto_tray_pose[0]))

//本轮有效路径: 前 3 点固定, 最后一点 = 所选盘的终点 (运行时按盘号填充)
#define GRAB_AUTO_PATH_NUM  4
static fp32 s_auto_path[GRAB_AUTO_PATH_NUM][2];

/* 按当前 grab_auto_tray(1/2/3) 生成本轮路径 */
static void grab_auto_path_build(void)
{
    uint8_t t = grab_auto_tray;

    if (t < 1 || t > GRAB_AUTO_TRAY_NUM)
    {
        t = 1;
    }

    s_auto_path[0][0] =  0.0f;        s_auto_path[0][1] =  0.0f;         //抓取侧(取物)
    s_auto_path[1][0] =  1.2f;        s_auto_path[1][1] = -1.2f;         //中间点1
    s_auto_path[2][0] =  1.5707963f;  s_auto_path[2][1] = -1.5707963f;   //中间点2 (π/2,-π/2)
    s_auto_path[3][0] =  s_auto_tray_pose[t - 1][0];                     //所选盘终点
    s_auto_path[3][1] =  s_auto_tray_pose[t - 1][1];
}

/* 开始朝当前路径点 s_auto_wp 的插值段: 段起点用 s_auto_a0/a1, 按关节速度算匀速时长 */
static void grab_auto_begin_move(void)
{
    fp32 b0 = s_auto_path[s_auto_wp][0];
    fp32 b1 = s_auto_path[s_auto_wp][1];
    fp32 d0 = fabsf(b0 - s_auto_a0);
    fp32 d1 = fabsf(b1 - s_auto_a1);
    fp32 d  = (d0 > d1) ? d0 : d1;
    uint32_t ms = (uint32_t)(d / GRAB_AUTO_SPEED * 1000.0f) + 1u;

    if (ms < GRAB_AUTO_SEG_MIN_MS)
    {
        ms = GRAB_AUTO_SEG_MIN_MS;
    }
    s_auto_seg_ms = (uint16_t)ms;
    s_auto_timer_ms = 0;
    s_auto_arrive_ms = 0;
    s_auto_state = GRAB_AUTO_MOVE;
}

/**
  * @brief          AUTO 真实取放状态机 (关节角线性匀速插值)
  * @note           每轮: IDLE准备 -> 朝“取”点移动(到位保持=夹爪动作) ->
  *                 沿 s_auto_path 中间点推进 -> 到“放”点(保持=放下) -> DONE。
  *                 流程方向由 grab_auto_flow 决定; 中途不处理遥控。
  * @param[out]     gc: "grab_control" 结构体指针
  * @retval         none
  */
static void grab_auto_handle(grab_control_t *gc)
{
    fp32 b0, b1;

    if (gc == NULL)
    {
        return;
    }

    //调试镜像 (Keil Watch)
    grab_auto_dbg_state = s_auto_state;
    grab_auto_dbg_wp = s_auto_wp;

    switch (s_auto_state)
    {
        case GRAB_AUTO_IDLE:
            //准备期: 先有力抓住当前位置, 短暂停留后开始
            gc->motor[0].angle_set = gc->motor[0].angle;
            gc->motor[1].angle_set = gc->motor[1].angle;
            s_auto_idle_ms++;
            if (s_auto_idle_ms >= GRAB_AUTO_IDLE_MS)
            {
                //按流程方向定本轮: first=取点(抓), final=放点(放)
                if (grab_auto_flow == GRAB_FLOW_GRAB_TO_TRAY)
                {
                    s_auto_step = 1;                              //抓取侧(0,0)->置物盘(所选盘号)
                    s_auto_first = 0;
                    s_auto_final = (int16_t)(GRAB_AUTO_PATH_NUM - 1);
                }
                else
                {
                    s_auto_step = -1;                             //置物盘(所选盘号)->抓取侧
                    s_auto_first = (int16_t)(GRAB_AUTO_PATH_NUM - 1);
                    s_auto_final = 0;
                }

                //按当前 grab_auto_tray(1/2/3) 生成本轮关节路径(终点=所选盘)
                grab_auto_path_build();

                s_auto_wp = s_auto_first;
                //首段从当前位置插值到“取”点
                s_auto_a0 = gc->motor[0].angle;
                s_auto_a1 = gc->motor[1].angle;
                grab_auto_begin_move();
            }
            break;

        case GRAB_AUTO_MOVE:
            b0 = s_auto_path[s_auto_wp][0];
            b1 = s_auto_path[s_auto_wp][1];

            if (s_auto_timer_ms < s_auto_seg_ms)
            {
                //关节角线性匀速插值
                fp32 k = (fp32)s_auto_timer_ms / (fp32)s_auto_seg_ms;
                gc->motor[0].angle_set = s_auto_a0 + (b0 - s_auto_a0) * k;
                gc->motor[1].angle_set = s_auto_a1 + (b1 - s_auto_a1) * k;
                s_auto_timer_ms++;
            }
            else
            {
                //插值段结束: 锁目标在路径点, 等物理到位
                gc->motor[0].angle_set = b0;
                gc->motor[1].angle_set = b1;
                if (fabsf(gc->motor[0].angle - b0) < GRAB_AUTO_ARRIVE_RAD &&
                    fabsf(gc->motor[1].angle - b1) < GRAB_AUTO_ARRIVE_RAD)
                {
                    s_auto_arrive_ms++;
                    if (s_auto_arrive_ms >= GRAB_AUTO_ARRIVE_MS)
                    {
                        s_auto_timer_ms = 0;
                        s_auto_state = GRAB_AUTO_HOLD;   //到位 -> 保持
                    }
                }
                else
                {
                    s_auto_arrive_ms = 0;
                }
                s_auto_timer_ms++;
                //到位超时保护: 实在到不了也不卡死
                if (s_auto_timer_ms > (uint16_t)(s_auto_seg_ms + GRAB_AUTO_TIMEOUT_MS))
                {
                    s_auto_timer_ms = 0;
                    s_auto_state = GRAB_AUTO_HOLD;
                }
            }
            break;

        case GRAB_AUTO_HOLD:
        {
            uint16_t hold = GRAB_AUTO_MID_HOLD_MS;

            //取/放端点(夹爪动作点)保持更久; 中间点短停
            if (s_auto_wp == s_auto_first || s_auto_wp == s_auto_final)
            {
                hold = GRAB_AUTO_HOLD_MS;
            }

            //保持在当前路径点
            gc->motor[0].angle_set = s_auto_path[s_auto_wp][0];
            gc->motor[1].angle_set = s_auto_path[s_auto_wp][1];

            s_auto_timer_ms++;
            if (s_auto_timer_ms >= hold)
            {
                s_auto_timer_ms = 0;
                if (s_auto_wp == s_auto_final)
                {
                    s_auto_state = GRAB_AUTO_DONE;   //到终点(放完)
                }
                else
                {
                    //推进到下一路径点: 插值段起点 = 当前点
                    s_auto_a0 = s_auto_path[s_auto_wp][0];
                    s_auto_a1 = s_auto_path[s_auto_wp][1];
                    s_auto_wp += s_auto_step;
                    grab_auto_begin_move();
                }
            }
            break;
        }

        case GRAB_AUTO_DONE:
        default:
            //完成: 停在放点位保持, 退出AUTO(回ZERO)后再重进可再来一轮
            gc->motor[0].angle_set = s_auto_path[s_auto_final][0];
            gc->motor[1].angle_set = s_auto_path[s_auto_final][1];
            break;
    }
}

/* (2026-09-07 已去掉软件零点: 电机硬件零位已置竖直, 反馈 para.pos 直接就是 θ) */

/**
  * @brief          更新电机反馈数据 (位置/速度)
  * @param[out]     gc: "grab_control" 结构体指针
  * @retval         none
  */
static void grab_feedback_update(grab_control_t *gc)
{
    if (gc == NULL)
    {
        return;
    }

    //电机1 反馈: 统一机械角度 θ = DIR * para.pos (硬件零位=竖直, 无软件零点)
    gc->motor[0].angle = ((fp32)GRAB_M1_DIR) * gc->motor[0].motor->para.pos;
    gc->motor[0].speed = ((fp32)GRAB_M1_DIR) * gc->motor[0].motor->para.vel;   //统一角速度 rad/s

    //电机2 反馈: 统一机械角度 θ
    gc->motor[1].angle = ((fp32)GRAB_M2_DIR) * gc->motor[1].motor->para.pos;
    gc->motor[1].speed = ((fp32)GRAB_M2_DIR) * gc->motor[1].motor->para.vel;   //统一角速度 rad/s

    //当前末端位置: 用当前 θ 直接 FK (Watch 验证五连杆解算 grab_dbg_fk_x/y)
    grab_dbg_fk_ok = (grab_fk_cont(gc->motor[0].angle, gc->motor[1].angle,
                                   &gc->ep_x, &gc->ep_y) == 0) ? 1u : 0u;
    if (grab_dbg_fk_ok)
    {
        grab_dbg_fk_x = gc->ep_x;
        grab_dbg_fk_y = gc->ep_y;
    }

    //Watch 置 grab_dbg_run_kin=1: 跑一次 FK/IK 往返自检(纯解算不动电机)后自动清0
    if (grab_dbg_run_kin != 0u)
    {
        grab_dbg_run_kin = 0u;
        grab_kin_selftest();
    }
}

/* ==================== AUTO: 底盘总控请求的取放执行器 (真序列) =============
 * 底盘 AUTO 到站并拿到许可后置 auto_grab_req.active, 机械臂在此执行一次取/放:
 *   PICK        : 场地抓(抓取侧端点) 夹爪合 CLOSE -> 翻越三姿态 -> 盘 tray 终点 张 OPEN
 *   PLACE/STACK : 盘 tray 终点 CLOSE(从盘取) -> 反向翻越 -> 场地(抓取侧端点) OPEN(放环)
 * 每点“到位(角度差 < GRAB_AUTO_ARRIVE_RAD, 稳定 GRAB_AUTO_ARRIVE_MS)”才推进;
 * 端点上做夹爪动作(端点1=CLOSE 夹, 端点2=OPEN 放), 中间点只路过。
 * 标定(全部 Keil Watch 可改):
 *   grab_ext_pick_t1/t2 : 抓取侧(场地料/环)端点 θ(rad)。现场把夹爪遥控对准料后,
 *                         读 motor[0].angle / motor[1].angle 填进来。
 *   grab_ext_way[3][2]   : 翻越三姿态 (默认 1.2/-1.2, π/2/-π/2, 2.0/-2.0)。
 *   盘终点               : s_auto_tray_pose[auto_grab_req.tray-1] (实测标定值)。
 * [视觉跟踪闭环] 抓取区用“五连杆 FK/IK 结算”(见 grab_ext_build_path): nominal(t1,t2)
 *   --FK(上支)--> (x,y) --加 vfx(前向mm)/dy--> --IK--> 本次 θ; 侧向 sd 由底盘负责。
 *   默认关闭(抓 nominal); 台架定好 vfx 正负(grab_ext_vis_fx_sign)后 Watch 置 grab_ext_vis_en=1。
 * ======================================================================== */
#define GRAB_AUTO_EXT_MAX 8u     /* 路径点上限(观测/往返走廊) */
static uint8_t  s_ext_state = 0u;        /* 0=空闲 1=移动到位 2=点保持 */
static uint8_t  s_ext_i = 0u;            /* 当前路径点下标 */
static uint8_t  s_ext_n = 1u;            /* 当前路径点数 */
static uint8_t  s_ext_close_i = 0xFFu;   /* 合爪点下标(0xFF=无) */
static uint8_t  s_ext_open_i  = 0xFFu;   /* 张爪点下标(0xFF=无) */
static uint16_t s_ext_arrive_ms = 0u;    /* 到位稳定计数 */
static uint16_t s_ext_wait_ms = 0u;      /* 移动计时/点保持计时 */
static fp32     s_ext_goal[2];           /* 当前点目标 θ */
static fp32     s_ext_path[GRAB_AUTO_EXT_MAX][2]; /* 运行时路径点(全部关节角) */

/* 复位外部取放执行器内部状态到“空闲”。用途: 单测被中途打断切回 AUTO 时调用,
 * 保证下一个 AUTO 请求从全新路径开始, 不会沿用单测残留的路径点/下标。
 * (正常 AUTO 流程不会走到这里: 只有进 AUTO 且检测到单测正在跑才会被调用) */
static void grab_auto_external_reset(void)
{
    s_ext_state = 0u;
    s_ext_i = 0u;
    s_ext_n = 1u;
    s_ext_close_i = 0xFFu;
    s_ext_open_i  = 0xFFu;
    s_ext_arrive_ms = 0u;
    s_ext_wait_ms = 0u;
}

static void ext_pt(uint8_t i, fp32 a, fp32 b)
{
    if (i < GRAB_AUTO_EXT_MAX)
    {
        s_ext_path[i][0] = a;
        s_ext_path[i][1] = b;
    }
}

/* 生成本次路径(关节角)。
 * cmd==AT_GRAB_CMD_SIGHT : 只伸到“抓取侧观测位”(不夹)让夹爪端相机看场地目标;
 *                          已在场地侧则单点, 在盘侧则经走廊回场地(防擦)。
 * cmd==AT_GRAB_CMD_GRAB  :
 *   PICK      场地合爪 -> 翻越(走廊) -> 盘 tray 张开(料放上盘);
 *   PLACE/STACK 此刻已在场地观测位 -> 经走廊回盘取料(合爪), 再经走廊回场地张开。
 * 抓取侧端点按 Jetson fx/sd 叠加视觉纠偏(grab_ext_vis_*, 默认关)。 */
static void grab_ext_build_path(void)
{
    uint8_t t = auto_grab_req.tray;
    fp32 tray_t1, tray_t2;
    fp32 field1, field2;

    if (t < 1u || t > 3u)
    {
        t = 1u;
    }
    tray_t1 = s_auto_tray_pose[t - 1u][0];
    tray_t2 = s_auto_tray_pose[t - 1u][1];

    /* 默认 = nominal 抓取端点(不加偏移); SIGHT(观测, 无许可数据)也用 nominal */
    field1 = grab_ext_pick_t1;
    field2 = grab_ext_pick_t2;
    grab_ext_vis_use1 = 0.0f;
    grab_ext_vis_use2 = 0.0f;

    /* 抓取区视觉纠偏(只对“拿到许可后的真抓放”cmd==GRAB) —— 五连杆 FK/IK 结算:
     * nominal(t1,t2) --FK(上支)--> (x0,y0) --加 vfx(前向mm)/dy--> --IK--> 本次 θ。
     * PICK 用 GRASP_READY 的 vfx, PLACE/STACK 用 ALIGN_READY 的; FK/IK 失败则退回 nominal(安全)。 */
    if (grab_ext_vis_en != 0u && auto_grab_req.cmd == AT_GRAB_CMD_GRAB && auto_grab_req.no_vis == 0u)
    {
        uint8_t is_align = (auto_grab_req.act == AT_KIND_PLACE || auto_grab_req.act == AT_KIND_STACK);
        fp32 vfx = is_align ? jetson.align_tar_x : jetson.grasp_tar_x;
        fp32 bx, by, tx, ty, n1, n2;
        if (grab_fk(grab_ext_pick_t1, grab_ext_pick_t2, &bx, &by) == 0)
        {
            tx = bx + vfx * grab_ext_vis_fx_sign;
            ty = by + grab_ext_vis_dy_mm;
            if (grab_ik(tx, ty, grab_ext_pick_t1, grab_ext_pick_t2, &n1, &n2) == 0)
            {
                field1 = n1;
                field2 = n2;
                grab_ext_vis_use1 = n1 - grab_ext_pick_t1;   /* Watch: 实际用了多少 θ 增量 */
                grab_ext_vis_use2 = n2 - grab_ext_pick_t2;
            }
        }
    }

    s_ext_close_i = 0xFFu;
    s_ext_open_i  = 0xFFu;

    if (auto_grab_req.cmd == AT_GRAB_CMD_SIGHT)
    {
        /* 伸到抓取侧观测位(不夹): 已在场地侧(θ1小)则单点, 在盘侧则经走廊回 */
        s_ext_n = 1u;
        if (grab_control.motor[0].angle > 1.4f)
        {
            s_ext_n = 4u;
            ext_pt(0, grab_ext_way[2][0], grab_ext_way[2][1]);
            ext_pt(1, grab_ext_way[1][0], grab_ext_way[1][1]);
            ext_pt(2, grab_ext_way[0][0], grab_ext_way[0][1]);
        }
        ext_pt(s_ext_n - 1u, field1, field2);
        return;
    }

    if (auto_grab_req.act == AT_KIND_PICK)
    {
        /* 场地合爪 -> 翻越(走廊) -> 盘 tray 张开 */
        s_ext_n = 5u;
        s_ext_close_i = 0u;
        s_ext_open_i  = 4u;
        ext_pt(0, field1, field2);
        ext_pt(1, grab_ext_way[0][0], grab_ext_way[0][1]);
        ext_pt(2, grab_ext_way[1][0], grab_ext_way[1][1]);
        ext_pt(3, grab_ext_way[2][0], grab_ext_way[2][1]);
        ext_pt(4, tray_t1, tray_t2);
    }
    else  /* PLACE / STACK: 从盘取 -> 场地放(全程走廊防擦) */
    {
        s_ext_n = 8u;
        s_ext_close_i = 3u;      /* 到达 tray 合爪取料 */
        s_ext_open_i  = 7u;      /* 回到场地 张开放环/叠垛 */
        ext_pt(0, grab_ext_way[0][0], grab_ext_way[0][1]);
        ext_pt(1, grab_ext_way[1][0], grab_ext_way[1][1]);
        ext_pt(2, grab_ext_way[2][0], grab_ext_way[2][1]);
        ext_pt(3, tray_t1, tray_t2);
        ext_pt(4, grab_ext_way[2][0], grab_ext_way[2][1]);
        ext_pt(5, grab_ext_way[1][0], grab_ext_way[1][1]);
        ext_pt(6, grab_ext_way[0][0], grab_ext_way[0][1]);
        ext_pt(7, field1, field2);
    }
}

static void grab_auto_external(grab_control_t *gc)
{
    fp32 g0, g1;
    uint32_t seg_tmo;

    if (gc == 0)
    {
        return;
    }
    if (auto_grab_req.done != 0u)
    {
        s_ext_state = 0u;        /* 底盘已收到完成, 复位等下次请求 */
        return;
    }

    /* 新请求到达: 建路径并张开夹爪(准备去抓/取) */
    if (s_ext_state == 0u)
    {
        s_ext_i = 0u;
        s_ext_arrive_ms = 0u;
        s_ext_wait_ms = 0u;
        grab_ext_build_path();
        grab_servo_set_angle(GRAB_SERVO_GRIPPER_IDX, GRAB_GRIPPER_OPEN_ANG);
        s_ext_state = 1u;
        return;
    }

    switch (s_ext_state)
    {
        case 1u:   /* 朝当前路径点移动并等物理到位 */
            if (s_ext_i >= s_ext_n)
            {
                s_ext_state = 0u;                 /* 全部走完 */
                auto_grab_req.done = 1u;
                break;
            }
            g0 = s_ext_path[s_ext_i][0];
            g1 = s_ext_path[s_ext_i][1];
            s_ext_goal[0] = g0;
            s_ext_goal[1] = g1;
            gc->motor[0].angle_set = g0;
            gc->motor[1].angle_set = g1;

            if (fabsf(gc->motor[0].angle - g0) < GRAB_AUTO_ARRIVE_RAD &&
                fabsf(gc->motor[1].angle - g1) < GRAB_AUTO_ARRIVE_RAD)
            {
                s_ext_arrive_ms++;
                if (s_ext_arrive_ms >= GRAB_AUTO_ARRIVE_MS)
                {
                    s_ext_arrive_ms = 0u;
                    s_ext_wait_ms = 0u;
                    s_ext_state = 2u;             /* 到位 -> 端点动作/短停 */
                }
            }
            else
            {
                s_ext_arrive_ms = 0u;
            }

            /* 到位超时保护: 按当前距目标差估时间+余量, 卡死也强制前进 */
            seg_tmo = (uint32_t)((fabsf(gc->motor[0].angle - g0) +
                                  fabsf(gc->motor[1].angle - g1)) /
                                 GRAB_AUTO_SPEED * 1000.0f) + 2000u;
            if (s_ext_wait_ms > seg_tmo)
            {
                s_ext_wait_ms = 0u;
                s_ext_state = 2u;
            }
            /* 已转 2(到位/超时)本周期不再 +1, 保证保持态首帧 s_ext_wait_ms==0 能执行夹爪动作 */
            if (s_ext_state == 1u)
            {
                s_ext_wait_ms++;
            }
            break;

        case 2u:   /* 保持在当前点: 在合爪/张爪点做夹爪动作, 然后推进 */
            gc->motor[0].angle_set = s_ext_goal[0];
            gc->motor[1].angle_set = s_ext_goal[1];

            if (s_ext_wait_ms == 0u)
            {
                if (s_ext_i == s_ext_close_i)
                {
                    grab_servo_set_angle(GRAB_SERVO_GRIPPER_IDX, GRAB_GRIPPER_CLOSE_ANG);
                }
                else if (s_ext_i == s_ext_open_i)
                {
                    grab_servo_set_angle(GRAB_SERVO_GRIPPER_IDX, GRAB_GRIPPER_OPEN_ANG);
                }
            }

            s_ext_wait_ms++;
            {
                uint16_t hold = (s_ext_i == s_ext_close_i || s_ext_i == s_ext_open_i)
                                    ? (uint16_t)GRAB_AUTO_HOLD_MS
                                    : (uint16_t)GRAB_AUTO_MID_HOLD_MS;
                if (s_ext_wait_ms >= hold)
                {
                    s_ext_wait_ms = 0u;
                    s_ext_i++;                     /* 推进下一路径点 */
                    if (s_ext_i >= s_ext_n)
                    {
                        s_ext_state = 0u;
                        auto_grab_req.done = 1u;    /* 本次完成 */
                    }
                    else
                    {
                        s_ext_state = 1u;
                    }
                }
            }
            break;

        default:
            s_ext_state = 0u;
            break;
    }
}

/* S[0](CH5 两档)拨下沿: 触发一轮“转盘视觉抓取”单测(等价于 Watch 置 grab_vis_test=1)。
 * 触发条件(全部满足才触发, 保证不影响 AUTO 完赛):
 *   - grab_vis_sw_en=1;
 *   - 遥控在线(非 DBUS_TOE);
 *   - 机械臂不在 AUTO(S2 不在下档)——AUTO 由底盘总控驱动, 不走这里;
 *   - 外部执行器空闲(auto_grab_req.active==0)且单测未在跑(grab_vis_test==0)。
 * 每轮做完 grab_vis_test 自动清 0; 想再来一轮把 S[0] 拨上再拨下即可。
 * 注: 若 S[0] 方向与实机相反, 交换下方 switch_is_down/switch_is_up 分支即可。 */
static void grab_vis_sw_trigger(void)
{
    static uint8_t s_s0_prev = 0u;   /* 1=上一拍 S[0] 在下 */
    uint8_t s0_down;

    if (grab_vis_sw_en == 0u)
    {
        s_s0_prev = 0u;
        return;
    }
    if (grab_rc == NULL || toe_is_error(DBUS_TOE))
    {
        s_s0_prev = 0u;             /* RC 失联: 不允许误触发 */
        return;
    }

    s0_down = switch_is_down(grab_rc->rc.s[0]) ? 1u : 0u;
    if (s0_down && s_s0_prev == 0u)
    {
        /* 只允许在“非 AUTO + 执行器空闲 + 没在跑单测”时触发 */
        if (grab_control.grab_mode != GRAB_MODE_AUTO &&
            auto_grab_req.active == 0u &&
            grab_vis_test == 0u)
        {
            grab_vis_test = 1u;     /* 触发一轮(完成后自清 0) */
        }
    }
    s_s0_prev = s0_down;
}

/* 单点“视觉抓取自测”状态机: 每 1ms 由 grab_set_control 调用(仅非 AUTO 模式)。
 * 阶段: 0 空闲(等 Watch) / 1 发 REQ 等 GRASP_READY(可跳过) / 2 执行器跑 PICK 并收尾 */
static void grab_vis_test_once(grab_control_t *gc)
{
    uint16_t ev;

    if (gc == NULL)
    {
        return;
    }
    if (gc->grab_mode == GRAB_MODE_AUTO || auto_grab_req.active != 0u)
    {
        return;   /* AUTO 或底盘请求正占用执行器时让路 */
    }

    grab_vis_dbg = s_vt;

    switch (s_vt)
    {
        case 0u:   /* 空闲: 等 Watch 触发沿 */
            if (grab_vis_test == 0u)
            {
                return;
            }
            s_vt = 1u;
            s_vt_wd = 0u;
            s_vt_req_done = 0u;
            s_vt_seq++;
            grab_servo_set_angle(GRAB_SERVO_GRIPPER_IDX, GRAB_GRIPPER_OPEN_ANG);
            return;

        case 1u:   /* 真实视觉: 发 REQ 并等 GRASP_READY(取 grasp_tar_x/y 纠偏) */
            if (grab_vis_skip_vision == 0u)
            {
                if (s_vt_req_done == 0u)
                {
                    s_vt_req_done = 1u;
                    if (jetson.phase != JETSON_PH_TASK_READY)
                    {
                        /* 会话未就绪: 无法真实视觉, 直接收场(置回 0) */
                        s_vt = 0u;
                        grab_vis_test = 0u;
                        return;
                    }
                    {
                        char seq[28], col[4];
                        uint8_t c = (grab_vis_test_color >= 1u && grab_vis_test_color <= 6u)
                                        ? grab_vis_test_color : 1u;
                        snprintf(seq, sizeof(seq), "VISTEST%lu", (unsigned long)s_vt_seq);
                        snprintf(col, sizeof(col), "%u", (unsigned)c);
                        jetson_send_req_pick(seq, "TURNTABLE", col);
                    }
                }
                ev = jetson_ev_take();
                if ((ev & (JETSON_EV_ERROR | JETSON_EV_TIMEOUT)) != 0u)
                {
                    s_vt = 0u;
                    grab_vis_test = 0u;
                    return;
                }
                if ((ev & JETSON_EV_GRASP_READY) == 0u)
                {
                    if ((ev & JETSON_EV_GRASP_REVOKED) != 0u)
                    {
                        s_vt_wd = 0u;   /* 撤销: 继续等新许可 */
                    }
                    s_vt_wd++;
                    if (s_vt_wd > GRAB_VIS_TEST_TIMEOUT_MS)
                    {
                        s_vt = 0u;
                        grab_vis_test = 0u;
                    }
                    return;
                }
                if (grab_vis_use_exec != 0u)
                {
                    jetson_send_exec(jetson.cur_seq);   /* 按协议回 EXEC */
                }
            }
            /* 已拿到许可(或占位模式): 发布 PICK 给外部执行器 */
            auto_grab_req.done   = 0u;
            auto_grab_req.cmd    = AT_GRAB_CMD_GRAB;
            auto_grab_req.act    = AT_KIND_PICK;
            auto_grab_req.scene  = AT_SCENE_TURNTABLE;
            auto_grab_req.ring   = 0u;
            auto_grab_req.tray   = (grab_vis_tray >= 1u && grab_vis_tray <= 3u) ? grab_vis_tray : 1u;
            /* 单测默认 no_vis=1: 不做 xy 位移纠偏, 认到“确认标志位(GRASP_READY)”就直接抓 nominal;
             * 想测视觉位移纠偏再 Watch 置 grab_vis_use_xy=1 */
            auto_grab_req.no_vis = (grab_vis_use_xy != 0u) ? 0u : 1u;
            auto_grab_req.active = 1u;
            s_vt = 2u;
            s_vt_wd = 0u;
            return;

        case 2u:   /* 执行器跑 PICK(含视觉纠偏), 完成后收尾 */
            grab_auto_external(gc);
            if (auto_grab_req.done != 0u)
            {
                if (grab_vis_use_exec != 0u && grab_vis_skip_vision == 0u)
                {
                    jetson_send_done(jetson.cur_seq, 1u);   /* 收会话, 回 TASK_READY */
                }
                auto_grab_req.active = 0u;
                auto_grab_req.done   = 0u;
                s_vt = 0u;
                grab_vis_test = 0u;   /* 完成: 自清, 想再测需再置 1 */
            }
            else
            {
                s_vt_wd++;
                if (s_vt_wd > GRAB_VIS_TEST_TIMEOUT_MS)
                {
                    auto_grab_req.active = 0u;
                    auto_grab_req.done   = 0u;
                    s_vt = 0u;
                    grab_vis_test = 0u;
                }
            }
            return;

        default:
            s_vt = 0u;
            grab_vis_test = 0u;
            return;
    }
}

/**
  * @brief          设置关节目标并按当前模式运行 (参考 Big_Emb big_emb_set_control)
  * @note           按当前模式更新关节目标: ZERO保持当前位置 / REMOTE遥控末端XY /
  *                 AUTO状态机按末端坐标自动跑; 最后统一做角度限幅保护。
  *                 调试钩子 grab_dbg_ang_* 可无视模式, 直接给某电机角度阶跃
  * @param[out]     gc: "grab_control" 结构体指针
  * @retval         none
  */
static void grab_set_control(grab_control_t *gc)
{
    if (gc == NULL)
    {
        return;
    }

    //调试: grab_dbg_ang_en=1 时无视模式, 直接给某电机一个角度阶跃(用于闭环方向测试)
    if (grab_dbg_ang_en)
    {
        uint8_t idx = (grab_dbg_ang_idx > 1) ? 1 : grab_dbg_ang_idx;
        gc->motor[idx].angle_set = grab_dbg_ang_val;
    }
    else if (grab_vis_test != 0u && gc->grab_mode != GRAB_MODE_AUTO)
    {
        /* Watch 单点“视觉抓取自测”: 车静止定点跑一轮 PICK(不与 AUTO 冲突) */
        grab_vis_test_once(gc);
    }
    else
    {
        //按当前模式设置关节目标: ZERO保持 / REMOTE遥控XY / AUTO状态机
        switch (gc->grab_mode)
        {
            case GRAB_MODE_REMOTE:
                grab_remote_handle(gc);
                break;

            case GRAB_MODE_AUTO:
                /* 底盘总控在跑: 由 auto_grab_req 驱动的取放执行器;
                   否则若 grab_auto_legacy_demo=1(Watch可开)跑老的三盘演示;
                   都没接管则锁定当前位置等待 */
                if (auto_grab_req.active != 0u)
                {
                    grab_auto_external(gc);
                }
                else if (grab_auto_legacy_demo != 0u)
                {
                    grab_auto_handle(gc);
                }
                else
                {
                    gc->motor[0].angle_set = gc->motor[0].angle;
                    gc->motor[1].angle_set = gc->motor[1].angle;
                }
                break;

            case GRAB_MODE_ZERO:
            default:
                //什么都不动: 目标锁定为当前反馈角度
                gc->motor[0].angle_set = gc->motor[0].angle;
                gc->motor[1].angle_set = gc->motor[1].angle;
                break;
        }
    }

    //已去掉软件角度限位(GRAB_Mi_ANGLE_MIN/MAX 不再夹 θ, 由机构硬限位保护);
    //若要恢复软件限位, 取消下面两行注释即可
    //gc->motor[0].angle_set = fp32_constrain(gc->motor[0].angle_set, GRAB_M1_ANGLE_MIN, GRAB_M1_ANGLE_MAX);
    //gc->motor[1].angle_set = fp32_constrain(gc->motor[1].angle_set, GRAB_M2_ANGLE_MIN, GRAB_M2_ANGLE_MAX);
}

/**
  * @brief          控制循环: 位置+速度级联 PID -> 输出转矩
  * @param[out]     gc: "grab_control" 结构体指针
  * @retval         none
  */
float temp_speed1_set,temp_speed2_set,temp_angle1_set,temp_angle2_set;
static void grab_control_loop(grab_control_t *gc)
{
    if (gc == NULL)
    {
        return;
    }

    //(2026-09-07 位置模式: 以下原 ZERO 零力/清PID 逻辑全部注释保留, 不再输出扭矩;
    // ZERO 的“保持当前位置”改由 grab_can_send 按当前反馈执行)
    // if (gc->grab_mode == GRAB_MODE_ZERO && !grab_dbg_ang_en)
    // {
    //     PID_clear(&gc->motor[0].pos_pid);
    //     PID_clear(&gc->motor[0].speed_pid);
    //     PID_clear(&gc->motor[1].pos_pid);
    //     PID_clear(&gc->motor[1].speed_pid);
    //     gc->motor[0].give_torque = 0.0f;
    //     gc->motor[1].give_torque = 0.0f;
    //     return;
    // }

    //位置 -> 速度期望, 速度 -> 转矩 (输出为MIT转矩指令) —— 已注释, 位置模式不再用
    // gc->motor[0].speed_set = PID_calc(&gc->motor[0].pos_pid, gc->motor[0].angle, gc->motor[0].angle_set, 0.0f);
    // gc->motor[0].give_torque = PID_calc(&gc->motor[0].speed_pid, gc->motor[0].speed, gc->motor[0].speed_set, 0.0f);
	
//    gc->motor[0].speed_set = PID_calc(&gc->motor[0].pos_pid, gc->motor[0].angle, temp_angle1_set, 0.0f);
//    gc->motor[0].give_torque = PID_calc(&gc->motor[0].speed_pid, gc->motor[0].speed, gc->motor[0].speed_set, 0.0f);
	
//	gc->motor[0].give_torque = PID_calc(&gc->motor[0].speed_pid, gc->motor[0].speed, temp_speed1_set, 0.0f);

    // gc->motor[1].speed_set = PID_calc(&gc->motor[1].pos_pid, gc->motor[1].angle, gc->motor[1].angle_set, 0.0f);
    // gc->motor[1].give_torque = PID_calc(&gc->motor[1].speed_pid, gc->motor[1].speed, gc->motor[1].speed_set, 0.0f);
	
//    gc->motor[1].speed_set = PID_calc(&gc->motor[1].pos_pid, gc->motor[1].angle, temp_angle2_set, 0.0f);
//    gc->motor[1].give_torque = PID_calc(&gc->motor[1].speed_pid, gc->motor[1].speed, gc->motor[1].speed_set, 0.0f);
//	
//	gc->motor[1].give_torque = PID_calc(&gc->motor[1].speed_pid, gc->motor[1].speed, temp_speed2_set, 0.0f);
}

/**
  * @brief          发送电机控制指令 (位置模式 POS, CAN2)
  * @note           grab电机: 电机1 主控ID 0x11 / 反馈ID 0x02
  *                           电机2 主控ID 0x12 / 反馈ID 0x02
  *                 直发角度(位置-速度帧), 限速 GRAB_POS_SPEED_LIMIT rad/s
  * @param[in]      gc: "grab_control" 结构体指针
  * @retval         none
  */
static void grab_can_send(grab_control_t *gc)
{
    fp32 raw1, raw2;
    fp32 vlim = GRAB_POS_SPEED_LIMIT;

    if (gc == NULL)
    {
        return;
    }

    if (gc->grab_mode == GRAB_MODE_ZERO && !grab_dbg_ang_en)
    {
        //ZERO(默认): 位置模式无法“零力”, 停在当前角度(发当前位置, 防止漂移/掉电自由落)
        raw1 = gc->motor[0].motor->para.pos;
        raw2 = gc->motor[1].motor->para.pos;
    }
    else
    {
        //目标角度 = DIR * 统一机械角 angle_set (硬件零位=竖直, 无软件零点)
        raw1 = ((fp32)GRAB_M1_DIR) * gc->motor[0].angle_set;
        raw2 = ((fp32)GRAB_M2_DIR) * gc->motor[1].angle_set;
    }

    //位置-速度模式直发角度 + 限速 2 rad/s
    pos_speed_ctrl(&hcan2, GRAB_M1_MASTER_ID, raw1, vlim);
    pos_speed_ctrl(&hcan2, GRAB_M2_MASTER_ID, raw2, vlim);

    //Watch 验证: 本次真正发给电机的角度(raw)
    grab_dbg_raw1 = raw1;
    grab_dbg_raw2 = raw2;
}

/*==========================================================================
 * 五连杆运动学核心 (FK/IK)
 * 坐标系与角度约定见 grab_task.h 顶部“五连杆机械结构与坐标系”说明:
 *   - 基坐标系(mm): M1=(-100,0), M2=(100,0); +x 向右 +y 向上
 *   - 主动杆 L1=L2=140, 从动杆 L3=L4=240; 末端 P 为两从动杆圆弧交点
 *   - 统一机械角度 θ: 逆时针为正, θ=DIR*(raw-zero), 竖直位=0; 方向转换层(GRAB_Mi_DIR)负责电机反馈<->θ
 *   - θ=0 时主动杆指向 GRAB_Mi_THETA0_STD (默认竖直向上 π/2)
 *==========================================================================*/

//FK 交点分支连续性历史
static fp32    s_fk_last_x = 0.0f;
static fp32    s_fk_last_y = 0.0f;
static uint8_t s_fk_have_prev = 0;

/* 主动杆单位方向向量: θ 逆时针为正(θ=0=竖直) => 标准角 a = a0 + θ */
static void grab_link_dir(fp32 theta, fp32 a0, fp32 *ux, fp32 *uy)
{
    fp32 a = a0 + theta;
    *ux = cosf(a);
    *uy = sinf(a);
}

/* 两圆交点: 圆心(c1x,c1y,r1) 与 圆心(c2x,c2y,r2)
 * 0=成功(输出两个交点), 1=无交点 */
static uint8_t grab_circle_intersect(fp32 c1x, fp32 c1y, fp32 r1,
                                     fp32 c2x, fp32 c2y, fp32 r2,
                                     fp32 *p1x, fp32 *p1y,
                                     fp32 *p2x, fp32 *p2y)
{
    fp32 dx = c2x - c1x;
    fp32 dy = c2y - c1y;
    fp32 d  = sqrtf(dx*dx + dy*dy);

    if (d < 1e-4f) return 1;                              //同心圆
    if (d > (r1 + r2) || d < fabsf(r1 - r2)) return 1;    //相离/内离

    fp32 a  = (r1*r1 - r2*r2 + d*d) / (2.0f*d);
    fp32 h2 = r1*r1 - a*a;
    fp32 h  = (h2 > 0.0f) ? sqrtf(h2) : 0.0f;
    fp32 mx = c1x + a*dx/d;
    fp32 my = c1y + a*dy/d;
    fp32 px = -dy/d;                                       //垂直方向单位向量
    fp32 py =  dx/d;

    *p1x = mx + h*px;
    *p1y = my + h*py;
    *p2x = mx - h*px;
    *p2y = my - h*py;
    return 0;
}

/**
  * @brief          方向转换: 电机反馈角度(raw) -> 统一机械角度 θ
  * @note           硬件零位=竖直, 无软件零点 => θ = DIR * raw
  */
fp32 grab_motor_to_theta(uint8_t idx, fp32 raw)
{
    fp32 dir  = (idx == 0) ? (fp32)GRAB_M1_DIR : (fp32)GRAB_M2_DIR;
    return dir * raw;
}

/**
  * @brief          方向转换: 统一机械角度 θ -> 电机反馈角度(raw)
  * @note           逆变换: raw = DIR * θ (DIR²=1)
  */
fp32 grab_theta_to_motor(uint8_t idx, fp32 theta)
{
    fp32 dir  = (idx == 0) ? (fp32)GRAB_M1_DIR : (fp32)GRAB_M2_DIR;
    return dir * theta;
}

/**
  * @brief          FK: (θ1,θ2) -> 末端 P (x,y) mm
  */
uint8_t grab_fk(fp32 theta1, fp32 theta2, fp32 *x_mm, fp32 *y_mm)
{
    fp32 u1x, u1y, u2x, u2y;
    fp32 a1x, a1y, a2x, a2y;
    fp32 p1x, p1y, p2x, p2y;

    if (x_mm == NULL || y_mm == NULL) return 1;

    grab_link_dir(theta1, GRAB_M1_THETA0_STD, &u1x, &u1y);
    grab_link_dir(theta2, GRAB_M2_THETA0_STD, &u2x, &u2y);

    //主动杆末端(肘)位置
    a1x = GRAB_BASE_X1_MM + GRAB_L1_MM*u1x;
    a1y = GRAB_BASE_Y_MM  + GRAB_L1_MM*u1y;
    a2x = GRAB_BASE_X2_MM + GRAB_L2_MM*u2x;
    a2y = GRAB_BASE_Y_MM  + GRAB_L2_MM*u2y;

    //末端 P = 圆(A1,L3) 与 圆(A2,L4) 的交点
    if (grab_circle_intersect(a1x, a1y, GRAB_L3_MM,
                              a2x, a2y, GRAB_L4_MM,
                              &p1x, &p1y, &p2x, &p2y) != 0)
    {
        return 1;                                         //机构被拉直, 无交点
    }

    //两个交点: 上支(y大)/下支(y小), 同时写入 Watch 供与实机核对
    if (p1y >= p2y)
    {
        grab_fk_up_x = p1x; grab_fk_up_y = p1y;
        grab_fk_lo_x = p2x; grab_fk_lo_y = p2y;
    }
    else
    {
        grab_fk_up_x = p2x; grab_fk_up_y = p2y;
        grab_fk_lo_x = p1x; grab_fk_lo_y = p1y;
    }
    /* 分支: 默认上支(原行为); grab_fk_branch==2 强制下支 */
    if (grab_fk_branch == 2u)
    {
        *x_mm = grab_fk_lo_x;
        *y_mm = grab_fk_lo_y;
    }
    else
    {
        *x_mm = grab_fk_up_x;
        *y_mm = grab_fk_up_y;
    }
    return 0;
}

/**
  * @brief          清除 FK 交点连续性历史 (标定零点后建议调用一次)
  */
void grab_fk_reset_prev(void)
{
    s_fk_have_prev = 0;
    s_fk_last_x = 0.0f;
    s_fk_last_y = 0.0f;
}

/**
  * @brief          FK(带连续性): 交点选与上一次 P 最近的一支
  */
uint8_t grab_fk_cont(fp32 theta1, fp32 theta2, fp32 *x_mm, fp32 *y_mm)
{
    fp32 u1x, u1y, u2x, u2y;
    fp32 a1x, a1y, a2x, a2y;
    fp32 p1x, p1y, p2x, p2y;
    fp32 d1, d2;

    if (x_mm == NULL || y_mm == NULL) return 1;

    grab_link_dir(theta1, GRAB_M1_THETA0_STD, &u1x, &u1y);
    grab_link_dir(theta2, GRAB_M2_THETA0_STD, &u2x, &u2y);
    a1x = GRAB_BASE_X1_MM + GRAB_L1_MM*u1x;
    a1y = GRAB_BASE_Y_MM  + GRAB_L1_MM*u1y;
    a2x = GRAB_BASE_X2_MM + GRAB_L2_MM*u2x;
    a2y = GRAB_BASE_Y_MM  + GRAB_L2_MM*u2y;

    if (grab_circle_intersect(a1x, a1y, GRAB_L3_MM,
                              a2x, a2y, GRAB_L4_MM,
                              &p1x, &p1y, &p2x, &p2y) != 0)
    {
        return 1;
    }

    //两个交点候选(上/下), 同时写入 Watch 供与实机核对
    if (p1y >= p2y)
    {
        grab_fk_up_x = p1x; grab_fk_up_y = p1y;
        grab_fk_lo_x = p2x; grab_fk_lo_y = p2y;
    }
    else
    {
        grab_fk_up_x = p2x; grab_fk_up_y = p2y;
        grab_fk_lo_x = p1x; grab_fk_lo_y = p1y;
    }

    if (grab_fk_branch == 1u || grab_fk_branch == 2u)
    {
        /* 强制固定某一支(现场核对后锁定, 避免连续切换歧义) */
        *x_mm = (grab_fk_branch == 2u) ? grab_fk_lo_x : grab_fk_up_x;
        *y_mm = (grab_fk_branch == 2u) ? grab_fk_lo_y : grab_fk_up_y;
    }
    else if (!s_fk_have_prev)
    {
        //没有历史: 默认取上支(y大)并记录
        *x_mm = grab_fk_up_x;
        *y_mm = grab_fk_up_y;
    }
    else
    {
        //取与上一次 P 最近的一支 (保持机构构型连续)
        d1 = (p1x - s_fk_last_x)*(p1x - s_fk_last_x) + (p1y - s_fk_last_y)*(p1y - s_fk_last_y);
        d2 = (p2x - s_fk_last_x)*(p2x - s_fk_last_x) + (p2y - s_fk_last_y)*(p2y - s_fk_last_y);
        if (d1 <= d2) { *x_mm = p1x; *y_mm = p1y; }
        else          { *x_mm = p2x; *y_mm = p2y; }
    }

    s_fk_last_x = *x_mm;
    s_fk_last_y = *y_mm;
    s_fk_have_prev = 1;
    return 0;
}

/* 单侧两连杆 IK: 基座(bx,by)+主动杆 ra+从动杆 rp 到达目标(x,y)
 * 返回该侧统一机械角度 θ 的候选个数 (0/1/2) */
static uint8_t grab_arm_ik(fp32 bx, fp32 by, fp32 ra, fp32 rp, fp32 a0,
                           fp32 x, fp32 y, fp32 out[2])
{
    fp32 vx = x - bx;
    fp32 vy = y - by;
    fp32 s  = sqrtf(vx*vx + vy*vy);
    fp32 min_s = fabsf(ra - rp);
    fp32 max_s = ra + rp;
    fp32 psi, cc, C, a_c1, a_c2;

    //可达性判断 (误差容限 1e-3 mm)
    if (s < (min_s - 1e-3f) || s > (max_s + 1e-3f)) return 0;

    psi = atan2f(vy, vx);                        //目标方向标准角
    cc  = (ra*ra + s*s - rp*rp) / (2.0f*ra*s);   //余弦定理
    cc  = (cc >  1.0f) ?  1.0f : cc;
    cc  = (cc < -1.0f) ? -1.0f : cc;
    C   = acosf(cc);                             //主动杆与目标方向的夹角

    a_c1 = psi + C;                              //两种装配构型(肘上/肘下)
    a_c2 = psi - C;
    out[0] = a_c1 - a0;                          //标准角 -> 统一机械角度 θ=a-a0
    out[1] = a_c2 - a0;
    return (C > 1e-4f) ? 2u : 1u;                //C≈0 时两解重合
}

/**
  * @brief          IK: (x,y) -> (θ1,θ2), 多解时取与当前角度最连续的一组
  */
uint8_t grab_ik(fp32 x_mm, fp32 y_mm,
                fp32 cur_theta1, fp32 cur_theta2,
                fp32 *theta1, fp32 *theta2)
{
    fp32    c1[2], c2[2];
    uint8_t n1, n2, i, j, found = 0;
    fp32    best = 1e30f;

    if (theta1 == NULL || theta2 == NULL) return 1;

    //左/右侧分别解 2 连杆 IK
    n1 = grab_arm_ik(GRAB_BASE_X1_MM, GRAB_BASE_Y_MM, GRAB_L1_MM, GRAB_L3_MM,
                     GRAB_M1_THETA0_STD, x_mm, y_mm, c1);
    n2 = grab_arm_ik(GRAB_BASE_X2_MM, GRAB_BASE_Y_MM, GRAB_L2_MM, GRAB_L4_MM,
                     GRAB_M2_THETA0_STD, x_mm, y_mm, c2);
    if (n1 == 0 || n2 == 0) return 1;            //任一侧不可达

    //组合所有解, 取与当前角度最连续的一组 (已去掉软件角度限位过滤)
    for (i = 0; i < n1; i++)
    {
        for (j = 0; j < n2; j++)
        {
            fp32 dist;
            dist = fabsf(c1[i] - cur_theta1) + fabsf(c2[j] - cur_theta2);
            if (dist < best)
            {
                best = dist;
                *theta1 = c1[i];
                *theta2 = c2[j];
                found = 1;
            }
        }
    }
    return found ? 0u : 1u;
}

/**
  * @brief          请求末端移动到 (x,y) mm, IK 成功才写入两电机目标角度
  */
uint8_t grab_set_endpoint(fp32 x_mm, fp32 y_mm)
{
    fp32 cur1, cur2, t1, t2;
    uint8_t ret;

    //连续性参考: 当前统一机械角度(motor[i].angle 已是 θ)
    cur1 = grab_control.motor[0].angle;
    cur2 = grab_control.motor[1].angle;

    ret = grab_ik(x_mm, y_mm, cur1, cur2, &t1, &t2);
    if (ret != 0) return ret;                    //IK 失败: 不改任何目标

    //统一机械角度(已在 IK 里做过 θ 限幅) 直接作为目标
    grab_control.motor[0].angle_set = t1;
    grab_control.motor[1].angle_set = t2;
    return 0;
}

/**
  * @brief          设定末端目标 (x,y) mm, 存入 tgt_x/tgt_y 并立即 IK 解算到两电机角度
  * @note           目标源 = grab_control.tgt_x/tgt_y; REMOTE/AUTO 持续跟踪, ZERO 不跟随
  * @retval          0=可解并已写入目标, 1=目标不可达(未写入)
  */
uint8_t grab_set_target(fp32 x_mm, fp32 y_mm)
{
    uint8_t ret;

    //末端目标 -> IK -> 两电机角度设定值 (可解才更新角度)
    ret = grab_set_endpoint(x_mm, y_mm);
    if (ret != 0)
    {
        return ret;              //不可达: 不改变目标
    }

    //记录目标源(结构体变量, 供 Watch/后续跟踪用)
    grab_control.tgt_x = x_mm;
    grab_control.tgt_y = y_mm;
    grab_control.tgt_valid = 1;
    return 0;
}

/**
  * @brief          读取当前末端位置 (mm), 由当前反馈经 FK 计算
  */
void grab_get_endpoint(fp32 *x_mm, fp32 *y_mm)
{
    if (x_mm == NULL || y_mm == NULL) return;
    if (grab_fk_cont(grab_control.motor[0].angle, grab_control.motor[1].angle, x_mm, y_mm) != 0)
    {
        *x_mm = 0.0f;
        *y_mm = 0.0f;
    }
}

/**
  * @brief          FK/IK 往返自检: 多组 (θ1,θ2)->FK->IK->FK 验证 (无串口打印)
  * @note           结果写全局 grab_st_pass/fail_code/err_ang/err_pos, 用 Keil Watch 查看
  * @retval          0=全部通过, 1=失败
  */
uint8_t grab_kin_selftest(void)
{
    static const fp32 test[][2] = {
        { 0.00f,  0.00f},
        { 0.35f,  0.35f},
        {-0.40f,  0.40f},
        { 0.40f, -0.40f},
        {-0.30f, -0.30f},
        { 0.70f, -0.20f},
    };
    int i;

    grab_st_pass = 0;
    grab_st_fail_code = 0;
    grab_st_err_ang = 0.0f;
    grab_st_err_pos = 0.0f;

    for (i = 0; i < (int)(sizeof(test)/sizeof(test[0])); i++)
    {
        fp32 x, y, t1, t2, x2, y2, ang_err, pos_err;

        //FK: (θ1,θ2) -> (x,y)
        if (grab_fk(test[i][0], test[i][1], &x, &y) != 0)      //FK 无解
        {
            grab_st_fail_code = 1;
            return 1;
        }
        //IK: (x,y) -> (θ1,θ2), 以原角度为连续性参考
        if (grab_ik(x, y, test[i][0], test[i][1], &t1, &t2) != 0)   //IK 失败
        {
            grab_st_fail_code = 2;
            return 1;
        }
        //闭环: IK -> FK
        if (grab_fk(t1, t2, &x2, &y2) != 0)                    //闭环无解
        {
            grab_st_fail_code = 3;
            return 1;
        }

        ang_err = fabsf(t1 - test[i][0]);
        if (fabsf(t2 - test[i][1]) > ang_err) ang_err = fabsf(t2 - test[i][1]);
        pos_err = sqrtf((x2-x)*(x2-x) + (y2-y)*(y2-y));
        if (ang_err > grab_st_err_ang) grab_st_err_ang = ang_err;
        if (pos_err > grab_st_err_pos) grab_st_err_pos = pos_err;
    }

    if (grab_st_err_ang > 1e-3f || grab_st_err_pos > 1e-2f)   //往返误差超限
    {
        grab_st_fail_code = 4;
        return 1;
    }
    grab_st_pass = 1;
    return 0;
}
