/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             chassis control task, including mode switching, remote control,
  *             feedback odometry and motor output.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "auto_task.h"
#include "jetson_task.h"
#include "task_param.h"
#include "user_lib.h"
#include "pid.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define CHASSIS_DEG2RAD 0.01745329252f   /* 角度(°)转弧度 */

/* 底盘速度斜坡规划参数(chassis_set_contorl 里用"当前速度+目标速度"限幅逼近):
 * 平移加速度 m/s², 旋转加速度 rad/s²; 值越大越跟手, 越小越柔 */
#define CHASSIS_ACC_MPS2       2.0f
#define CHASSIS_ACC_WZ_RADPS2  8.0f
/* 速度逼近: 当前速度 cur 每周期最多向目标 tgt 移动 step */
#define CHASSIS_RAMP_TO(cur, tgt, step) \
    ( ((tgt) - (cur)) >  (step) ? (cur) + (step) : \
     (((tgt) - (cur)) < -(step) ? (cur) - (step) : (tgt)) )

/* =============== 恒定航向控制(角度闭环) 参数 ===============
 * 车头始终自动对齐目标航向 yaw_ref(默认 CHASSIS_HEADING_REF_YAW = 0):
 *  - 航向环全程生效(前进/滑行/停车), 只要不在手动转向, 都把车头拉回 yaw_ref
 *    => 高速松杆甩动后的偏航会自动拉回, 下次前进就是正的, 不会斜着走
 *  - vx/vy 设定值: 遥控机体系直给(不投影), 车头被航向环锁在 yaw_ref
 *    => vx 正方向 = 当前设定 yaw_ref 方向(车头方向), 推前进就朝设定角度走
 *  - wz 设定值: 航向误差(yaw_ref - IMU yaw) 走 PID
 * 手动旋转(CH3)= 重新设定目标航向: 推 CH3 时 yaw_ref 实时跟随当前角度,
 *   松开后航向环就保持车头冲这个新 yaw_ref(不会弹回0); 不推 CH3 时目标保持上次设定
 * 调参重点: 回正慢/走弧线 -> 增大 KP; 震荡 -> 减小 KP 或增大 KD */
#define CHASSIS_HEADING_REF_YAW         0.0f   /* 恒定目标航向 rad: 想让车头固定朝某方向, 改这里 */
#define CHASSIS_HEADING_MAX_WZ          4.0f   /* 航向环最大修正角速度 rad/s */
#define CHASSIS_HEADING_PID_KP          2.5f
#define CHASSIS_HEADING_PID_KI          0.0f
#define CHASSIS_HEADING_PID_KD          0.10f  /* D 作用在 IMU z 轴角速度, 抑制回正超调 */

/* 航向闭环 PID 对象(1ms 调用一次, 输出 rad/s) */
static pid_type_def chassis_heading_pid;

/* ==================== 底盘 AUTO: 一键全自动总控 ==========================
 * 拨 S2 进 CHASSIS_AUTO 后自动执行: 自动发 START -> 等任务码(auto_plan 24步)
 * -> 逐站: 导航到停靠点 -> 发 REQ(auto_plan seq) -> 等许可
 *    (PICK: GRASP_READY->自动EXEC->EXEC_ACK; PLACE/STACK: ALIGN_READY)
 *    -> 通知机械臂执行(auto_grab_req) -> 等完成 -> DONE -> DONE_ACK -> 下一站
 * -> 24 站完成回启停2 结束。
 * 若没有 Jetson 又想测导航: Watch 置 chassis_auto_demo=1, 进 AUTO 后 1.5s
 * 自动造一份默认任务并按“到站停 CAUTO_HOLD_MS”跑完(跳过视觉握手)。
 * ======================================================================== */
#define CAUTO_SPEED_MAX      0.5f    /* 平动最大 m/s */
#define CAUTO_WZ_MAX         CHASSIS_HEADING_MAX_WZ
#define CAUTO_ARRIVE_M       0.03f   /* 到站判定 m */
#define CAUTO_HOLD_MS        900u    /* demo/机械臂占位停留 ms */
#define CAUTO_ARM_TIMEOUT_MS 30000u /* 等机械臂完成超时 ms (真序列含5点×到位判稳, 需余量) */
#define CAUTO_PERM_TIMEOUT_MS 20000u /* 等视觉许可超时 ms(识别超时) */
#define CAUTO_ACK_TIMEOUT_MS 2000u   /* 等 EXEC_ACK/DONE_ACK 超时 ms */
#define CAUTO_NAV_TIMEOUT_MS 25000u  /* 一段导航(含摆头)超时 ms: 到不了点→CAM_ERR, 防无限空转
                                      * (全场最长路由 < 4m, 0.5m/s+摆头 < 10s, 25s 余量充足) */

/* 总控阶段 */
typedef enum
{
    CAM_BOOT = 0,   /* 进 AUTO: 自动发 START, 等任务码(或 demo) */
    CAM_RUN,        /* 逐站执行 */
    CAM_ERR,        /* 出错(超时/ERROR): 停车等待人工拨出重试 */
    CAM_DONE,       /* 全部完成(已回启停2) */
} cam_master_e;

/* 单站子阶段 */
typedef enum
{
    CAS_NAV = 0,      /* 导航到本站停靠点 */
    CAS_SIGHT,        /* 真视觉: 先伸臂到抓取侧观测位(不夹), 到位后才 REQ */
    CAS_REQ,          /* 发 REQ */
    CAS_WAIT_PERM,    /* 等 GRASP_READY / ALIGN_READY */
    CAS_WAIT_ACK,     /* PICK: 已 EXEC, 等 EXEC_ACK */
    CAS_ARM,          /* 通知机械臂取/放, 等完成 */
    CAS_DONE_SEND,    /* 动作完成, 发 DONE */
    CAS_ACK,          /* 等 DONE_ACK (OK->下一站 / RETRY->回等许可) */
} cam_stage_e;

static uint8_t s_cauto_last_mode = 0xFFu;
static uint8_t s_cauto_master = CAM_BOOT;
static uint8_t s_cauto_stage = CAS_NAV;
static int16_t s_cauto_step = -1;      /* auto_plan 下标 */
static uint8_t s_cauto_home = 0u;      /* 1=回启停2 */
static uint8_t s_cauto_vision = 0u;    /* 1=真实视觉握手; 0=demo */
static uint8_t s_cauto_align = 0u;     /* 1=本站等 ALIGN(PLACE/STACK) */
static uint8_t s_cauto_retry = 0u;     /* 1=DONE RETRY 续做本站: 回观测位后直接等新许可(不再重发 REQ) */
static uint8_t s_cauto_start_sent = 0u;
static uint8_t s_cauto_done_sent = 0u;
static uint16_t s_cauto_timer = 0;     /* 通用 ms 计时 */
static uint16_t s_cauto_wd = 0;        /* 握手看门狗 */
static uint16_t s_cauto_nav_tmo = 0;   /* 导航看门狗: 一段折线(到点+摆头)超时计数 */
static fp32 s_cauto_orgx = 0.0f, s_cauto_orgy = 0.0f; /* 进 AUTO 起点 */
static fp32 s_cauto_orgyaw = 0.0f;
static uint8_t s_cauto_qr_go = 0u;   /* 1=进AUTO先去二维码板点扫码 */
/* --- 田字道网折线导航状态(2026-09-07: §6.6 停车点+车头朝向+道带折线) --- */
static uint8_t  s_cauto_zone = 0u;    /* 当前所在停车点区(见 CZ_* 枚举) */
static uint8_t  s_cauto_ring = 1u;    /* 当前所在环(粗/暂, 其他=1) */
static uint8_t  s_cauto_wp_n = 0u;    /* 折线途经点数 */
static uint8_t  s_cauto_wp_i = 0u;    /* 当前途经点下标 */
static fp32     s_cauto_wpx[8];       /* 途经点 odom m */
static fp32     s_cauto_wpy[8];
static uint8_t  s_cauto_face_en = 0u; /* 终点需摆头? */
static fp32     s_cauto_face_yaw = 0.0f; /* 终点目标航向(相对启停, rad) */
static uint8_t  s_cauto_tz = 0u;      /* 本次目标区(供到站记录) */
static uint8_t  s_cauto_tr = 1u;      /* 本次目标环 */

/* 自动执行调试(Keil Watch) */
uint8_t  chassis_auto_dbg_state = CAM_BOOT;   /* master */
uint8_t  chassis_auto_dbg_stage = CAS_NAV;    /* stage */
int16_t  chassis_auto_dbg_step = -1;          /* 当前步; -2=回启停 */
uint8_t  chassis_auto_dbg_scene = 0;
uint8_t  chassis_auto_dbg_kind = 0;
uint8_t  chassis_auto_dbg_ring = 0;
uint8_t  chassis_auto_dbg_vision = 0;         /* 1=真视觉 0=demo */
float    chassis_auto_dbg_dist_mm = 0.0f;     /* 剩余距离 mm */
uint8_t  chassis_auto_demo = 0u;              /* Watch: 1=无视觉 demo 导航 */
uint8_t  chassis_auto_dbg_zone = 0u;          /* Watch: 当前目标停车点区(CZ_*) */
uint8_t  chassis_auto_dbg_wp = 0u;            /* Watch: 当前折线途经点下标 */

static void cauto_v1_to_odom(fp32 vx1, fp32 vy1, fp32 *ox, fp32 *oy);
static void cauto_zone_stop(uint8_t zone, uint8_t ring, fp32 *sx, fp32 *sy, fp32 *hyaw);
static void cauto_build_route(uint8_t tz, uint8_t tring);
static uint8_t cauto_leg_nav(fp32 *vxo, fp32 *vyo, fp32 *wzo);
static void cam_demo_build(void);
static void cam_send_req(void);
static void cam_route_step(void);
static void cam_advance(void);
static void chassis_auto_update(fp32 *vx_o, fp32 *vy_o, fp32 *wz_o);

chassis_move_t chassis_move;

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_heading_line_hold(chassis_move_t *chassis_move_control, fp32 vx_w, fp32 vy_w, fp32 wz_rc,
                                      fp32 *vx_body_out, fp32 *vy_body_out, fp32 *wz_out);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_can_send(chassis_move_t *chassis_move_can_send);

extern CAN_HandleTypeDef hcan1;

void chassis_task(void const *pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    chassis_init(&chassis_move);

    for (uint8_t i = 0; i < 4; i++)
    {
        zdt_enable(&hcan1, chassis_move.wheel[i].motor);
		vTaskDelay(5);
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        zdt_set_auto_report(&hcan1, chassis_move.wheel[i].motor, ZDT_CMD_READ_POS, 2);
		vTaskDelay(5);
    }

    while (1)
    {
        chassis_set_mode(&chassis_move);
        chassis_feedback_update(&chassis_move);
        chassis_set_contorl(&chassis_move);
        chassis_control_loop(&chassis_move);
        chassis_can_send(&chassis_move);

        /* VOFA/上位机打印已全部移除: USART1 现为 Jetson 通信口(PB7/PA9),
         * 里程计/速度调试一律用 Keil Watch 看 chassis_move.* */

        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    /* 轮子索引(正逆解用): 0=左前LF 1=右前RF 2=左后LB 3=右后RB
     * 电机CAN ID 分布: 左前=ID1 右前=ID2 左后=ID3 右后=ID4
     * rev: 该轮正转方向与正解方向相反时设为-1 (右侧两轮实测反装) */
    const static uint8_t chassis_motor_id[4] = {CHASSIS_M1_CAN_ID, CHASSIS_M2_CAN_ID, CHASSIS_M3_CAN_ID, CHASSIS_M4_CAN_ID}; /* 1 2 3 4 = LF RF LB RB */
    const static int8_t  chassis_wheel_rev[4] = {1, -1, 1, -1};    /* LF RF LB RB */

    chassis_move_init->chassis_mode = CHASSIS_ZERO_FORCE;

    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->INS_accel = get_accel_data_point();
    chassis_move_init->INS_angle = get_INS_angle_point();
    chassis_move_init->INS_gyro  = get_gyro_data_point();

    chassis_move_init->half_wheelbase  = CHASSIS_HALF_WHEELBASE;
    chassis_move_init->half_wheeltrack = CHASSIS_HALF_WHEELTRACK;
    chassis_move_init->wheelbase_sum   = CHASSIS_HALF_WHEELBASE + CHASSIS_HALF_WHEELTRACK;
    chassis_move_init->wheel_radius    = CHASSIS_WHEEL_RADIUS;

    chassis_move_init->position_x     = 0.0f;
    chassis_move_init->position_y     = 0.0f;
    chassis_move_init->position_angle = 0.0f;
    chassis_move_init->total_travel   = 0.0f;

    /* 恒定航向控制: 目标航向 = CHASSIS_HEADING_REF_YAW; S3 上=使能, 下=纯手动 */
    chassis_move_init->yaw_ref    = CHASSIS_HEADING_REF_YAW;
    chassis_move_init->heading_en = 1;
    {
        /* PID_GIMBAL: 内部对角度误差做 ±180° 环绕; D 项用传入的 IMU z 角速度 */
        static const fp32 heading_pid[3] = {CHASSIS_HEADING_PID_KP, CHASSIS_HEADING_PID_KI, CHASSIS_HEADING_PID_KD};
        PID_init(&chassis_heading_pid, PID_GIMBAL, heading_pid,
                 CHASSIS_HEADING_MAX_WZ, 1.0f, 0.0f, 0.0f);
    }

    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->wz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;
    chassis_move_init->wz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_init->wheel[i].motor = get_chassis_motor_point(i);
        chassis_move_init->wheel[i].motor->id = chassis_motor_id[i];
        chassis_move_init->wheel[i].rev = chassis_wheel_rev[i];   /* 正反装标志 */

        chassis_move_init->wheel[i].motor->ctrl.sync_flag = 0;
        chassis_move_init->wheel[i].motor->ctrl.dir = 0;

        chassis_move_init->wheel[i].wheel_speed_set = 0.0f;
        chassis_move_init->wheel[i].wheel_speed = 0.0f;
        chassis_move_init->wheel[i].last_wheel_speed = 0.0f;
        chassis_move_init->wheel[i].last_upd_ms = 0;
        chassis_move_init->wheel[i].last_angle_deg = 0.0f;
        chassis_move_init->wheel[i].odom_ready = 0;
    }
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    // RC disconnected / data timeout -> force zero-force mode (safety)
    if (toe_is_error(DBUS_TOE))
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
        return;
    }

    // Use 3-position switch S2 to select mode:
    //   S2 up   -> CHASSIS_ZERO_FORCE    (zero)
    //   S2 mid  -> CHASSIS_REMOTE_CONTROL(remote)
    //   S2 down -> CHASSIS_AUTO          (auto)
    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[2]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[2]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_REMOTE_CONTROL;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[2]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_AUTO;
    }
    else
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
    }

    /* S3(遥控 s[3]) 切换底盘遥控的航向行为(仅遥控模式有意义):
     *   S3 上 = 恒定航向回正(yaw_ref=0, 角度闭环)    -> heading_en = 1
     *   S3 下 = 纯手动(无角度修正, 未加闭环前的状态)  -> heading_en = 0
     *   S3 中 = 保持上次状态(避免误触抖动) */
    if (chassis_move_mode->chassis_mode == CHASSIS_REMOTE_CONTROL)
    {
        static uint8_t heading_prev_en = 1;
        uint8_t heading_en = chassis_move_mode->heading_en;
        if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[3]))
        {
            heading_en = 1;
        }
        else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[3]))
        {
            heading_en = 0;
        }
        /* 重新开启时清掉 PID 历史, 防止恢复瞬间 D/积分跳变 */
        if (heading_en && !heading_prev_en)
        {
            PID_clear(&chassis_heading_pid);
        }
        heading_prev_en = heading_en;
        chassis_move_mode->heading_en = heading_en;
    }
}

/* ============================================================================
 * =========================  ESKF 融合里程计（平面 6 状态）  =================
 * 作者/日期: 2026-09-07   (Error-State EKF 的平面退化实现)
 * 目标     : 把【轮式里程计】和【IMU】融合，输出更稳更准的 position_x/y/angle。
 *
 * ── 为什么需要它 ─────────────────────────────────────────────
 *   旧积分: 直接把每轮位移 ds 投到世界系累加，航向=INS yaw。
 *     问题: ①轮子打滑/单轮空转时 ds 会虚高，位置瞬间跑偏且无法纠正；
 *           ②只“信任”INS 航向但没建模 IMU 自身零偏，长时间小漂；
 *           ③没有速度滤波，ds/Δt 噪声直接进位置。
 *   ESKF  : 把轮速当“速度观测”（打滑时调大 R 或门控跳过），用 IMU 陀螺推演航向、
 *           用 INS 航向做“航向观测”并把陀螺零偏 bg 估出来——位置由滤波后速度积分，
 *           被打滑污染的一步 ds 会被协方差“平滑掉”，航向也有零偏补偿。
 *
 * ── 坐标系（与底盘/文档一致）───────────────────────────────
 *   机体系: X 前 / Y 左 / 旋转 Z 逆时针为正 (与麦克纳姆正解输出 vx,vy,wz 一致)
 *   世界系: 初始时刻车头方向 = X，左 = Y（与旧积分同构）
 *   机体系→世界系: 向量旋转 R(ψ): dx_w = dx*cosψ - dy*sinψ ; dy_w = dx*sinψ + dy*cosψ
 *
 * ── 状态向量 x[6] ──────────────────────────────────────────
 *   x0 px   世界 X 位置 (m)
 *   x1 py   世界 Y 位置 (m)
 *   x2 psi  航向 ψ (rad, 与 INS yaw 同符号)
 *   x3 vxw  世界系 X 速度 (m/s)
 *   x4 vyw  世界系 Y 速度 (m/s)
 *   x5 bg   陀螺 Z 零偏估计 (rad/s)   ← IMU 自带的常值漂移，这里在线估
 *
 * ── 运动模型（预测，dt≈0.001s 每控制节拍一次）───────────────
 *   ω = INS_gyro[Z]            (IMU 实测 z 轴角速度)
 *   px += vxw*dt ;  py += vyw*dt
 *   psi += (ω - bg)*dt
 *   vxw/vyw/bg 不变(用过程噪声描述扰动)
 *
 * ── 观测（校正，同一节拍顺序做 3 次标量更新）─────────────────
 *   ① 轮式里程计 机体系 vx_whl / vy_whl   → 由状态反算 h_vx = vxw·cosψ+vyw·sinψ
 *   ② 轮式里程计 机体系 vy_whl            → h_vy = -vxw·sinψ+vyw·cosψ
 *   ③ IMU 航向 INS_angle[Yaw]             → h = ψ
 *   (①②给速度/位置，③给航向并让 bg 可观测)
 *
 * ── ★★★ 想让它更准，改什么？什么效果？（核心调参，均可在 Watch 实时改）★★★
 *   这些量都在 chassis_eskf 结构体里，开机进 AUTO/遥控前可在 Keil Watch 直接改：
 *
 *   量            默认     含义 / 你该往哪调                             效果
 *   -------------------------------------------------------------------------
 *   eskf_odom_en  1        0=退回旧直接积分(对照用)                     用于 A/B 对比效果
 *   r_vel         2.5e-3   轮速观测噪声方差(m/s)²。                    ↓调大=更不信轮速→抗打滑
 *                           路面滑、麦轮侧滑大→调大(1e-2~1e-1);         更好,但低速跟车会"钝"
 *                           地面抓地好、想跟得紧→调小(1e-4~1e-3)
 *   r_yaw         1e-4     INS 航向观测噪声方差(rad)²。                ↓调大=更信陀螺→短时更顺
 *                           磁干扰强时 yaw 会跳→调大(1e-3~1e-2);        但有磁干扰时不要太大
 *                           磁正常想压陀螺零偏→调小
 *   q_psi         1e-6     航向过程噪声方差(rad²)/步≈陀螺噪声。         调大=让 ψ 更快跟观测
 *   q_bg          1e-9     陀螺零偏随机游走方差(rad/s)²/步。            调大=零偏估得活(易抖)
 *   q_vel         1e-4     速度过程噪声方差(m/s)²/步≈加速度贡献。       调大=速度跟得快(噪),
 *                                                                       调小=平滑(有滞后)
 *   q_pos         1e-9     位置过程噪声额外项(m)²/步(一般不动)
 *   p0_*          init     初始协方差对角线(起步置信)
 *
 *   ★ 现场"打滑改善"最快: 先把 r_vel 加大 10 倍跑一次看轨迹; 若定位有零点几度转差,
 *     调 r_yaw 与 q_bg 的组合(想快速估出 bg 就 r_yaw 调小、q_bg 调大)。
 *   ★ 陀螺若很差: 把 q_psi 调小、r_yaw 调大 → 更多信 INS yaw(它自带加计/磁校正)。
 *   ★ 想要更强抗打滑/加计预积分: 本模块刻意没吃加速度(地面不平/倾角会引入重力耦合,
 *     需要先把俯仰/横滚补偿做对)。要加时在 predict 里把 x[3]/x[4] 的更新改成用
 *     R(ψ)·(ax,ay)-g 项，并扩状态估加速度计零偏——注释已留位置。
 * ========================================================================= */

/* ---- 平面 ESKF 参数容器(全部可 Keil Watch 调) ---- */
typedef struct
{
    uint8_t init;          /* 1=已初始化(首步以 INS yaw 定 ψ 基准) */
    /* 状态 */
    fp32 px, py, psi;      /* 世界位置(m) 与航向(rad) */
    fp32 vxw, vyw;         /* 世界速度(m/s) */
    fp32 bg;               /* 陀螺 Z 零偏(rad/s) */
    /* 协方差(6x6, 行主序) */
    fp32 P[6][6];
    /* 可调噪声(单位见上面表格) */
    fp32 q_pos, q_psi, q_vel, q_bg;
    fp32 r_vel, r_yaw;
    fp32 p0_pos, p0_psi, p0_vel, p0_bg;
} chassis_eskf_t;

/* 供 Keil Watch: ESKF 开关(1=ESKF融合, 0=旧直接积分) 与 滤波器状态/噪声 */
uint8_t        eskf_odom_en = 1u;
chassis_eskf_t chassis_eskf;

/* ---- 内部工作缓冲(static 省栈) ---- */
static fp32 eskf_F[6][6], eskf_Ft[6][6], eskf_T1[6][6], eskf_T2[6][6];
static fp32 eskf_hp[6], eskf_k[6];

static void eskf_odom_defaults(void);   /* 前向声明(定义在步函数之后) */

/* 6x6 乘: C = A*B */
static void eskf_mul6(fp32 A[6][6], fp32 B[6][6], fp32 C[6][6])
{
    uint8_t i, j, k;
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            fp32 s = 0.0f;
            for (k = 0; k < 6; k++)
            {
                s += A[i][k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}

/* 6x6 转置 */
static void eskf_trans6(fp32 A[6][6], fp32 At[6][6])
{
    uint8_t i, j;
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            At[j][i] = A[i][j];
        }
    }
}

/* 预测: 名义状态积分 + 协方差传播 P = F·P·Fᵀ + Q
 * dt: 步长(s)   gyro_z: IMU z 角速度(rad/s) */
static void eskf_predict(fp32 dt, fp32 gyro_z)
{
    uint8_t i, j, k;
    fp32 cw, sw;

    /* ① 名义状态: 位置由速度积分, 航向由(陀螺-零偏)积分 */
    chassis_eskf.px  += chassis_eskf.vxw * dt;
    chassis_eskf.py  += chassis_eskf.vyw * dt;
    chassis_eskf.psi += (gyro_z - chassis_eskf.bg) * dt;

    /* ② 状态转移雅可比 F（相对 P 的线性化; 只 3 处非恒等）:
     *    δpx ← δvxw*dt ; δpy ← δvyw*dt ; δψ ← -δbg*dt  */
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            eskf_F[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    eskf_F[0][3] =  dt;
    eskf_F[1][4] =  dt;
    eskf_F[2][5] = -dt;

    /* ③ P = F·P·Fᵀ + Q（Q 为对角过程噪声） */
    eskf_mul6(eskf_F, chassis_eskf.P, eskf_T1);      /* T1 = F·P   */
    eskf_trans6(eskf_F, eskf_Ft);
    eskf_mul6(eskf_T1, eskf_Ft, eskf_T2);            /* T2 = F·P·Fᵀ */
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            chassis_eskf.P[i][j] = eskf_T2[i][j];
        }
        /* 加对角过程噪声: 位置几乎不由过程加(由速度), 给个小量防 P 病态 */
        chassis_eskf.P[i][i] += (i == 0 || i == 1) ? chassis_eskf.q_pos
                              : (i == 2)           ? chassis_eskf.q_psi
                              : (i == 3 || i == 4) ? chassis_eskf.q_vel
                              :                      chassis_eskf.q_bg;
    }

    /* (可选) 用加速度计做速度预积分增强抗打滑: 取消下面注释前请先做好
     *  俯仰/横滚重力补偿(见头注释), 否则会把重力耦合进 px/py。
     *  cw = cosf(chassis_eskf.psi); sw = sinf(chassis_eskf.psi);
     *  ax_w = accel_body_x*cw - accel_body_y*sw;   // 已去掉 g 的水平加速度
     *  ay_w = accel_body_x*sw + accel_body_y*cw;
     *  chassis_eskf.vxw += ax_w*dt; chassis_eskf.vyw += ay_w*dt; */
    (void)cw; (void)sw; (void)k;
}

/* 一次标量(卡尔曼)校正:
 * z  观测值, h  由当前状态算出的预测, H[6] 观测行, R 观测噪声方差 */
static void eskf_correct_scalar(fp32 z, fp32 h, const fp32 H[6], fp32 R)
{
    uint8_t i, j;
    fp32 y, s;

    y = z - h;                                   /* 新息 */
    /* s = H·P·Hᵀ + R ; 先算 a = P·Hᵀ 与 hp = H·P 同元素(对称) */
    for (i = 0; i < 6; i++)
    {
        fp32 sum = 0.0f;
        for (j = 0; j < 6; j++)
        {
            sum += chassis_eskf.P[i][j] * H[j];
        }
        eskf_k[i] = sum;                         /* 暂存 P·Hᵀ */
    }
    s = R;
    for (i = 0; i < 6; i++)
    {
        s += H[i] * eskf_k[i];
    }
    if (s < 1e-12f)
    {
        s = 1e-12f;                              /* 数值保护 */
    }
    for (i = 0; i < 6; i++)
    {
        eskf_k[i] /= s;                          /* 卡尔曼增益 K */
    }

    /* 状态更新 x += K·y（逐字段直写, 结构体字段顺序见 chassis_eskf_t） */
    chassis_eskf.px  += eskf_k[0] * y;
    chassis_eskf.py  += eskf_k[1] * y;
    chassis_eskf.psi += eskf_k[2] * y;
    chassis_eskf.vxw += eskf_k[3] * y;
    chassis_eskf.vyw += eskf_k[4] * y;
    chassis_eskf.bg  += eskf_k[5] * y;

    /* P = (I - K·H)·P  用  hp[j] = ΣᵢH[i]P[i][j] */
    for (j = 0; j < 6; j++)
    {
        fp32 sum = 0.0f;
        for (i = 0; i < 6; i++)
        {
            sum += H[i] * chassis_eskf.P[i][j];
        }
        eskf_hp[j] = sum;
    }
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            chassis_eskf.P[i][j] -= eskf_k[i] * eskf_hp[j];
        }
    }
}

/* 观测1+2: 轮式里程计机体系速度 vx/vy → 校正(状态里是世界速度, 需旋转)
 *   vx_whl / vy_whl: 麦克纳姆正解得到的机体系速度(m/s)
 *   顺序更新: 每个观测前都基于"最新状态"重算雅可比, 避免级联误差 */
static void eskf_correct_wheel(fp32 vx_whl, fp32 vy_whl)
{
    fp32 cw, sw, hvx, hvy;
    fp32 H[6];

    /* 观测 x 向速度: h_vx = vxw·cosψ + vyw·sinψ
     *  ∂/∂vxw=cosψ ∂/∂vyw=sinψ ∂/∂ψ = -vxw·sinψ+vyw·cosψ = h_vy */
    cw = cosf(chassis_eskf.psi);
    sw = sinf(chassis_eskf.psi);
    hvx =  chassis_eskf.vxw * cw + chassis_eskf.vyw * sw;
    hvy = -chassis_eskf.vxw * sw + chassis_eskf.vyw * cw;
    H[0] = 0.0f; H[1] = 0.0f; H[2] = hvy; H[3] = cw; H[4] = sw; H[5] = 0.0f;
    eskf_correct_scalar(vx_whl, hvx, H, chassis_eskf.r_vel);

    /* 观测 y 向速度: h_vy = -vxw·sinψ + vyw·cosψ
     *  ∂/∂vxw=-sinψ ∂/∂vyw=cosψ ∂/∂ψ = -(vxw·cosψ+vyw·sinψ) = -h_vx
     * (基于上一个观测更新后的最新状态重算) */
    cw = cosf(chassis_eskf.psi);
    sw = sinf(chassis_eskf.psi);
    hvx =  chassis_eskf.vxw * cw + chassis_eskf.vyw * sw;
    hvy = -chassis_eskf.vxw * sw + chassis_eskf.vyw * cw;
    H[0] = 0.0f; H[1] = 0.0f; H[2] = -hvx; H[3] = -sw; H[4] = cw; H[5] = 0.0f;
    eskf_correct_scalar(vy_whl, hvy, H, chassis_eskf.r_vel);
}

/* 观测3: INS 航向 yaw_imu → 校正 ψ（H 只选 ψ），让 bg 可观测 */
static void eskf_correct_yaw(fp32 yaw_imu)
{
    static const fp32 H[6] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    eskf_correct_scalar(yaw_imu, chassis_eskf.psi, H, chassis_eskf.r_yaw);
}

/* 首次运行/重启: 以当前 INS 航向为 ψ 基准, 位置清零, 协方差置初始 */
static void eskf_odom_reset(fp32 yaw_imu)
{
    uint8_t i, j;

    chassis_eskf.init = 1u;
    chassis_eskf.px = 0.0f;
    chassis_eskf.py = 0.0f;
    chassis_eskf.psi = yaw_imu;
    chassis_eskf.vxw = 0.0f;
    chassis_eskf.vyw = 0.0f;
    chassis_eskf.bg  = 0.0f;
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            chassis_eskf.P[i][j] = 0.0f;
        }
        chassis_eskf.P[i][i] = (i == 0 || i == 1) ? chassis_eskf.p0_pos
                            : (i == 2)            ? chassis_eskf.p0_psi
                            : (i == 3 || i == 4)  ? chassis_eskf.p0_vel
                            :                      chassis_eskf.p0_bg;
    }
}

/* ESKF 单步入口(在底盘 1ms 反馈里调):
 * dt=0.001; gyro_z=INS z角速度; vx_whl/vy_whl=轮速机体系; yaw_imu=INS航向 */
static void eskf_odom_step(fp32 dt, fp32 gyro_z, fp32 vx_whl, fp32 vy_whl, fp32 yaw_imu)
{
    static uint8_t s_eskf_done = 0u;   /* 首次运行写入默认噪声(防止开机前 Watch 被清零) */

    if (s_eskf_done == 0u)
    {
        eskf_odom_defaults();
        s_eskf_done = 1u;
    }
    if (chassis_eskf.init == 0u)
    {
        eskf_odom_reset(yaw_imu);
    }
    eskf_predict(dt, gyro_z);
    eskf_correct_wheel(vx_whl, vy_whl);
    eskf_correct_yaw(yaw_imu);
}

/* 底盘任务初始化时调用: 赋 ESKF 默认噪声(开机后首次 ESKF 步前自动调一次, 之后 Watch 可改) */
static void eskf_odom_defaults(void)
{
    chassis_eskf.q_pos = 1e-9f;
    chassis_eskf.q_psi = 1e-6f;
    chassis_eskf.q_vel = 1e-4f;
    chassis_eskf.q_bg  = 1e-9f;
    chassis_eskf.r_vel = 2.5e-3f;
    chassis_eskf.r_yaw = 1e-4f;
    chassis_eskf.p0_pos = 1e-4f;
    chassis_eskf.p0_psi = 2.5e-3f;
    chassis_eskf.p0_vel = 1e-2f;
    chassis_eskf.p0_bg  = 4e-4f;
    chassis_eskf.init = 0u;
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    static uint32_t ctrl_ms = 0;   /* 1ms 控制节拍计数, 用于测速的真实时间间隔 */

    if (chassis_move_update == NULL)
    {
        return;
    }

    ctrl_ms++;

    fp32 R = chassis_move_update->wheel_radius;
    fp32 D = chassis_move_update->wheelbase_sum;
    fp32 ds[4] = {0.0f};   /* 各轮本次弧长增量(m), 只用于里程计积分 */
    fp32 w[4] = {0.0f};    /* 各轮速度(rad/s, 已乘rev) 用于机体速度解算
                            * (必须清零: 未收到首帧前 w[i] 不会被赋值,
                            *  未初始化栈值可能是 NaN/超大数, 会毒化下方 ESKF 速度观测) */

    for (uint8_t i = 0; i < 4; i++)
    {
        /* 电机自动上报位置, CAN接收已更新 para.sum_pos(相对上电累计角度°) */
        fp32 sum_deg = (fp32)chassis_move_update->wheel[i].motor->para.sum_pos;

        /* 首次收到位置只建基准, 不积分(避免上电瞬间的跳变) */
        if (chassis_move_update->wheel[i].odom_ready == 0)
        {
            chassis_move_update->wheel[i].last_angle_deg = sum_deg;
            chassis_move_update->wheel[i].last_upd_ms    = ctrl_ms;
            chassis_move_update->wheel[i].odom_ready     = 1;
            continue;
        }

        fp32 d_deg = sum_deg - chassis_move_update->wheel[i].last_angle_deg;
        chassis_move_update->wheel[i].last_angle_deg = sum_deg;

        /* 里程计位移增量(m): 只有新位置帧时非0, 位置积分不依赖时间 */
        ds[i] = d_deg * CHASSIS_DEG2RAD * R * (fp32)chassis_move_update->wheel[i].rev;

        /* 测速: 有新帧时按"真实上报间隔"算(≈2ms), 无新帧时保持上次值,
         * 避免 "2ms位移 ÷ 1ms" 造成的约2倍虚高 */
        if (d_deg != 0.0f)
        {
            fp32 gap_s = (fp32)(ctrl_ms - chassis_move_update->wheel[i].last_upd_ms) * CHASSIS_CONTROL_TIME;
            if (gap_s < CHASSIS_CONTROL_TIME)
            {
                gap_s = CHASSIS_CONTROL_TIME;
            }
            chassis_move_update->wheel[i].wheel_speed = (d_deg * CHASSIS_DEG2RAD) / gap_s; /* rad/s(电机侧) */
            chassis_move_update->wheel[i].last_upd_ms = ctrl_ms;
        }
        /* d_deg==0: 保持 wheel_speed 不变(采样保持) */

        w[i] = chassis_move_update->wheel[i].wheel_speed * (fp32)chassis_move_update->wheel[i].rev;
    }

    /* 机体速度: 用采样保持后的轮速做麦克纳姆正解, 单位正确
     * 轮序 0=左前LF 1=右前RF 2=左后LB 3=右后RB */
    chassis_move_update->vx = R * ( w[0] + w[1] + w[2] + w[3]) / 4.0f;
    chassis_move_update->vy = R * (-w[0] + w[1] + w[2] - w[3]) / 4.0f;
    chassis_move_update->wz = R * (-w[0] + w[1] - w[2] + w[3]) / (4.0f * D);

    /* 数值保护: 结果若不是正常有限值(未初始化/NaN/Inf)就按 0 处理,
     * 防止污染下方 ESKF 速度观测(px/py 一旦被 NaN 污染将永久失效) */
    {
        fp32 avx = fabsf(chassis_move_update->vx);
        fp32 avy = fabsf(chassis_move_update->vy);
        if (!(avx >= 0.0f && avx <= 10.0f))
        {
            chassis_move_update->vx = 0.0f;
        }
        if (!(avy >= 0.0f && avy <= 10.0f))
        {
            chassis_move_update->vy = 0.0f;
        }
    }

    /* ==================== 里程计（可切 ESKF / 旧直接积分）====================
     * eskf_odom_en==1 (默认, Watch 可切): ESKF 融合
     *   - 预测: 位置由速度积分; 航向由 IMU 陀螺 z 积分(估出零偏 bg)
     *   - 校正: ①轮速机体系 vx/vy(速度观测) ②INS yaw(航向观测)
     *   - 输出写回 position_x/y/position_angle(世界系, 与旧法同构)
     * eskf_odom_en==0: 旧直接积分(ds 直接投世界系) 作 A/B 对照
     * 路程 odometer(total_travel) 只与每步实际位移长度有关, 两种方式一致。
     */
    {
        fp32 dx_body = ( ds[0] + ds[1] + ds[2] + ds[3]) / 4.0f;      /* 米 */
        fp32 dy_body = (-ds[0] + ds[1] + ds[2] - ds[3]) / 4.0f;      /* 米 */
        fp32 yaw  = chassis_move_update->INS_angle[INS_YAW_ADDRESS_OFFSET];
        fp32 gz   = chassis_move_update->INS_gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        static uint8_t s_eskf_prev = 1u;  /* 上拍 ESKF 开关(现场 A/B 切换不跳变) */

        /* 路程(米): 每步真实走过长度(机体位移模长, 与融合方式无关) */
        chassis_move_update->total_travel += sqrtf(dx_body * dx_body + dy_body * dy_body);

        if (eskf_odom_en != 0u)
        {
            /* 重新使能(0→1): 用“旧积分一直保持的”当前位置/速度同步进 ESKF 状态,
             * 使 A/B 切换瞬间位置连续(否则 ESKF 冻结在旧状态会造成跳变) */
            if (s_eskf_prev == 0u)
            {
                fp32 cw = cosf(yaw), sw = sinf(yaw);
                chassis_eskf.px  = chassis_move_update->position_x;
                chassis_eskf.py  = chassis_move_update->position_y;
                chassis_eskf.psi = yaw;
                chassis_eskf.vxw =  chassis_move_update->vx * cw - chassis_move_update->vy * sw;
                chassis_eskf.vyw =  chassis_move_update->vx * sw + chassis_move_update->vy * cw;
                chassis_eskf.init = 1u;   /* 跳过复位(保留协方差), 已同步好 */
            }
            s_eskf_prev = 1u;

            /* ESKF 融合: 轮速作“速度观测”。轮速是采样保持值, 停车时若不处理
             * 会把上次速度一直当“还在动”→ 位置缓慢漂。下面按“各轮最近位置帧"
             * 是否还在更新判断移动: 超过静止阈值没新帧就强制观测速度=0。 */
            fp32 vmx = chassis_move_update->vx;
            fp32 vmy = chassis_move_update->vy;
            {
                uint8_t qi, moving = 0u;
                for (qi = 0u; qi < 4u; qi++)
                {
                    if ((uint32_t)(ctrl_ms - chassis_move_update->wheel[qi].last_upd_ms) <= 6u)
                    {
                        moving = 1u;
                        break;
                    }
                }
                if (moving == 0u)
                {
                    vmx = 0.0f;
                    vmy = 0.0f;
                }
            }
            eskf_odom_step(CHASSIS_CONTROL_TIME, gz, vmx, vmy, yaw);
            chassis_move_update->position_x     = chassis_eskf.px;
            chassis_move_update->position_y     = chassis_eskf.py;
            chassis_move_update->position_angle = chassis_eskf.psi;
        }
        else
        {
            s_eskf_prev = 0u;    /* 记录当前为关闭, 供 0→1 时同步 */

            /* 旧直接积分(对照): 机体系→世界系用 INS yaw */
            fp32 cos_yaw = cosf(yaw);
            fp32 sin_yaw = sinf(yaw);
            fp32 dx_world = dx_body * cos_yaw - dy_body * sin_yaw;
            fp32 dy_world = dx_body * sin_yaw + dy_body * cos_yaw;
            chassis_move_update->position_x     += dx_world;
            chassis_move_update->position_y     += dy_world;
            chassis_move_update->position_angle  = yaw;
        }
    }
}

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    int16_t vx_channel = 0, vy_channel = 0;
    fp32 vx_set_channel = 0.0f, vy_set_channel = 0.0f;

    if (chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] > CHASSIS_RC_DEADLINE ||
        chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] < -CHASSIS_RC_DEADLINE)
    {
        vx_channel = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL];
    }
    if (chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] > CHASSIS_RC_DEADLINE ||
        chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] < -CHASSIS_RC_DEADLINE)
    {
        vy_channel = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL];
    }

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_W)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_S)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }
    if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_A)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_D)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    *vx_set = vx_set_channel;
    *vy_set = vy_set_channel;
}

/**
  * @brief          恒定航向控制: 车头始终自动回正 yaw_ref, wz 由航向PID修正; vx/vy 机体系直给
  * @param[in]      c: 底盘数据
  * @param[in]      vx_w, vy_w: 遥控期望平动(机体系, 沿车头/yaw_ref 方向为+X, m/s)
  * @param[in]      wz_rc: 手动旋转(CH3)指令 rad/s; 0 = 无手动旋转
  * @param[out]     vx_body_out, vy_body_out: 机体系平动设定 (m/s, 直接透传 vx_w/vy_w)
  * @param[out]     wz_out: 最终旋转设定 (rad/s): 手动转向 或 航向PID输出
  * @note          手动转向优先; 只要不手动转向, 航向环就把车头拉回 yaw_ref(默认0)
  */
static void chassis_heading_line_hold(chassis_move_t *c, fp32 vx_w, fp32 vy_w, fp32 wz_rc,
                                      fp32 *vx_body_out, fp32 *vy_body_out, fp32 *wz_out)
{
    fp32 yaw = c->INS_angle[INS_YAW_ADDRESS_OFFSET];
    int16_t ch3 = c->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
    uint8_t manual_turn = (ch3 > CHASSIS_RC_DEADLINE || ch3 < -CHASSIS_RC_DEADLINE);

    if (manual_turn)
    {
        /* 手动转向优先: 车自由转, 并把"当前角度"实时设成新的目标航向 yaw_ref
         * => 松开后航向环保持车头冲这个新角度(不会弹回 0) */
        c->yaw_ref = yaw;
        *wz_out = wz_rc;
    }
    else
    {
        /* 恒定航向闭环: 无论平移/滑行/停车都生效, 误差在 ±180° 内, D 用 IMU z 角速度 */
        *wz_out = PID_calc(&chassis_heading_pid, yaw, c->yaw_ref,
                           c->INS_gyro[INS_GYRO_Z_ADDRESS_OFFSET]);
        *wz_out = fp32_constrain(*wz_out, -CHASSIS_HEADING_MAX_WZ, CHASSIS_HEADING_MAX_WZ);
    }

    /* vx/vy 机体系直给: 车头被航向环锁定在 yaw_ref, 所以前进即沿"设定 yaw"方向
     * (不做场地投影; 若以后要贴"场地绝对直线/坐标"需另加位置环) */
    *vx_body_out = vx_w;
    *vy_body_out = vy_w;
}

/* ==================== AUTO 自动导航/调度 实现 ============================ */
/* 车启动摆位(场地绝对, 原点=右上角0,0): 里程计原点在车启动位置 */
#define CAUTO_START_V1_X  150.0f
#define CAUTO_START_V1_Y  150.0f

static void cauto_v1_to_odom(fp32 vx1, fp32 vy1, fp32 *ox, fp32 *oy)
{
    /* AT_* 为场地绝对坐标(原点=右上角0,0; +x左 +y下). 启动姿态: 车中心在
     * 场地绝对(150,150), 车头(odom+X)朝场地 +x(左).
     * 里程计原点 = 车启动位置 => odom = 绝对-(150,150); odom_y = -v1_y. 单位 m */
    *ox = s_cauto_orgx + (vx1 - CAUTO_START_V1_X) * 0.001f;
    *oy = s_cauto_orgy - (vy1 - CAUTO_START_V1_Y) * 0.001f;
}

/* ==================== 停车点模型(§6.6) + 道带折线路由 ===================
 * 停车点 = 作业点(AT_* 场地绝对, 原点=右上角0,0) 沿“臂侧”退 250:
 *   转盘(1200,-75)→停(1200,175) 头+x 臂上;  粗环(环x,2325)→停(环x,2075) 头-x 臂下;
 *   暂环(2325,环y)→停(2075,环y) 头-y 臂左;  QR/回启停 停点本身。
 * 跨区按文档 §6.6 长途路由(L1~L6/H、S0)走“道带内轴对齐折线”; 区内直移; 不压四黄。
 * ======================================================================== */
#define CZ_START    0    /* 停车点区枚举 */
#define CZ_QR       1
#define CZ_TP       2
#define CZ_ROUGH    3
#define CZ_STORAGE  4

static fp32 cauto_ring_x(uint8_t r)
{
    return (r == 1u) ? AT_ROUGH_R1_X : ((r == 2u) ? AT_ROUGH_R2_X : AT_ROUGH_R3_X);
}

static fp32 cauto_ring_y(uint8_t r)
{
    return (r == 1u) ? AT_STORAGE_R1_Y : ((r == 2u) ? AT_STORAGE_R2_Y : AT_STORAGE_R3_Y);
}

/* 场地方向(dx,dy; +x左 +y下) -> 相对启停车头 yaw(odom: odom_y=-场地y) */
static fp32 cauto_dir_yaw(fp32 dx, fp32 dy)
{
    return atan2f(-dy, dx);
}

/* 区/环 -> 停车点(场地绝对) + 停车后车头朝向(相对启停 yaw) */
static void cauto_zone_stop(uint8_t zone, uint8_t ring, fp32 *sx, fp32 *sy, fp32 *hyaw)
{
    uint8_t r = (ring >= 1u && ring <= 3u) ? ring : 1u;
    switch (zone)
    {
        case CZ_START:                          /* 回启停=启动摆位 */
            *sx = AT_START2_X;
            *sy = AT_START2_Y;
            *hyaw = cauto_dir_yaw(1.0f, 0.0f);
            break;
        case CZ_QR:                             /* 二维码板观察点本身, 头+x */
            *sx = AT_QR_X;
            *sy = AT_QR_Y;
            *hyaw = cauto_dir_yaw(1.0f, 0.0f);
            break;
        case CZ_TP:                             /* (1200,175) 头+x 臂向上够转盘 */
            *sx = AT_TURNTABLE_X;
            *sy = AT_TURNTABLE_Y + AT_ARM_REACH_MM;
            *hyaw = cauto_dir_yaw(1.0f, 0.0f);
            break;
        case CZ_ROUGH:                          /* (环x,2075) 头-x 臂向下够环 */
            *sx = cauto_ring_x(r);
            *sy = AT_ROUGH_Y - AT_ARM_REACH_MM;
            *hyaw = cauto_dir_yaw(-1.0f, 0.0f);
            break;
        case CZ_STORAGE:
        default:                                /* (2075,环y) 头-y(上) 臂向左够环 */
            *sx = AT_STORAGE_X - AT_ARM_REACH_MM;
            *sy = cauto_ring_y(r);
            *hyaw = cauto_dir_yaw(0.0f, -1.0f);
            break;
    }
}

/* 往折线途经点数组推一点(绝对→odom); 满则忽略 */
static void cauto_wp_push(fp32 ax, fp32 ay)
{
    fp32 ox, oy;
    if (s_cauto_wp_n >= (uint8_t)(sizeof(s_cauto_wpx) / sizeof(s_cauto_wpx[0])))
    {
        return;
    }
    cauto_v1_to_odom(ax, ay, &ox, &oy);
    s_cauto_wpx[s_cauto_wp_n] = ox;
    s_cauto_wpy[s_cauto_wp_n] = oy;
    s_cauto_wp_n++;
}

/* 依“所在(s_cauto_zone/s_cauto_ring)”到“目标(tz,tring)”布折线(全在灰道),
 * 区同/点同 => 直移/原地; 最后推终点并设终点摆头 yaw */
static void cauto_build_route(uint8_t tz, uint8_t tring)
{
    fp32 tx, ty, tyaw, ay;
    uint8_t hz = s_cauto_zone;
    uint8_t hr = s_cauto_ring;
    fp32 rough_stop_y = AT_ROUGH_Y - AT_ARM_REACH_MM;   /* 2075 */
    fp32 tp_stop_y    = AT_TURNTABLE_Y + AT_ARM_REACH_MM; /* 175 */

    s_cauto_wp_n = 0u;
    s_cauto_wp_i = 0u;
    s_cauto_tz = tz;
    s_cauto_tr = tring;

    if (hz != tz)   /* 跨区: 按 §6.6 长途路由折线 */
    {
        if (hz == CZ_START && tz == CZ_QR)                     /* S0: 右外圈竖道 */
        {
            cauto_wp_push(AT_QR_X, AT_QR_Y);
        }
        else if (hz == CZ_START && tz == CZ_TP)                /* S0+L1(有码直去转盘) */
        {
            cauto_wp_push(AT_QR_X, AT_QR_Y);
            cauto_wp_push(AT_TURNTABLE_X, AT_QR_Y);
            cauto_wp_push(AT_TURNTABLE_X, tp_stop_y);
        }
        else if (hz == CZ_QR && tz == CZ_TP)                   /* L1 */
        {
            cauto_wp_push(AT_TURNTABLE_X, AT_QR_Y);
            cauto_wp_push(AT_TURNTABLE_X, tp_stop_y);
        }
        else if (hz == CZ_TP && tz == CZ_ROUGH)                /* L2/L5 */
        {
            cauto_wp_push(AT_TURNTABLE_X, rough_stop_y);
            cauto_wp_push(cauto_ring_x(tring), rough_stop_y);
        }
        else if (hz == CZ_ROUGH && tz == CZ_STORAGE)           /* L3/L6 */
        {
            ay = cauto_ring_y(tring);
            cauto_wp_push(cauto_ring_x(hr), ay);
            cauto_wp_push(AT_STORAGE_X - AT_ARM_REACH_MM, ay);
        }
        else if (hz == CZ_STORAGE && tz == CZ_TP)              /* L4 */
        {
            cauto_wp_push(AT_TURNTABLE_X, cauto_ring_y(hr));
            cauto_wp_push(AT_TURNTABLE_X, tp_stop_y);
        }
        else if (hz == CZ_STORAGE && tz == CZ_START)           /* H: 回启停 */
        {
            cauto_wp_push(AT_QR_X, cauto_ring_y(hr));
            cauto_wp_push(AT_QR_X, AT_START2_Y);
        }
        /* 其他组合理论不发生: 不加中间点, 直接走“终点”兜底 */
    }

    cauto_zone_stop(tz, tring, &tx, &ty, &tyaw);
    cauto_wp_push(tx, ty);
    s_cauto_face_en = (tz != CZ_START) ? 1u : 0u;  /* 回启停不必摆头 */
    s_cauto_face_yaw = tyaw;
}

/* ==================== AUTO 总控实现 ===================================== */
/* 无视觉 demo: 造一份默认任务(便于没 Jetson 也测导航) */
static void cam_demo_build(void)
{
    static const uint8_t c1[3] = {1u, 5u, 6u};
    static const uint8_t p1[3] = {1u, 2u, 3u};
    static const uint8_t c2[3] = {5u, 1u, 6u};
    static const uint8_t p2[3] = {2u, 3u, 1u};
    uint8_t i;

    for (i = 0; i < 3; i++)
    {
        task_code.batch1_color[i] = c1[i];
        task_code.batch1_place[i] = p1[i];
        task_code.batch2_color[i] = c2[i];
        task_code.batch2_place[i] = p2[i];
    }
    auto_plan_build();
    s_cauto_vision = 0u;
}

/* 本站动作 -> Jetson REQ */
static void cam_send_req(void)
{
    auto_step_t *st = &auto_plan[s_cauto_step];
    const char *sc = auto_scene_str(st->scene);
    char num[8];

    if (st->kind == AT_KIND_PICK)
    {
        snprintf(num, sizeof(num), "%u", (unsigned)st->color);
        jetson_send_req_pick(st->seq, sc, num);
        s_cauto_align = 0u;
    }
    else if (st->kind == AT_KIND_PLACE)
    {
        snprintf(num, sizeof(num), "%u", (unsigned)st->ring);
        jetson_send_req_place(st->seq, sc, num);
        s_cauto_align = 1u;
    }
    else
    {
        char num2[8];
        snprintf(num, sizeof(num), "%u", (unsigned)st->color);
        snprintf(num2, sizeof(num2), "%u", (unsigned)st->ring);
        jetson_send_req_stack(st->seq, sc, num, num2);
        s_cauto_align = 1u;
    }
}

/* 依当前步(或回启停)决定目标区/环, 并布折线 */
static void cam_route_step(void)
{
    uint8_t tz;
    uint8_t tring = 1u;

    if (s_cauto_home != 0u)
    {
        tz = CZ_START;                 /* 全部完成 -> 回启停 */
        chassis_auto_dbg_scene = 0xFFu;
        chassis_auto_dbg_kind  = 0xFFu;
        chassis_auto_dbg_ring  = 0u;
        chassis_auto_dbg_step  = -2;
    }
    else if (s_cauto_step >= 0 && s_cauto_step < (int16_t)auto_plan_n)
    {
        uint8_t sc = auto_plan[s_cauto_step].scene;
        uint8_t r  = auto_plan[s_cauto_step].ring;
        chassis_auto_dbg_scene = sc;
        chassis_auto_dbg_kind  = auto_plan[s_cauto_step].kind;
        chassis_auto_dbg_ring  = r;
        chassis_auto_dbg_step  = s_cauto_step;
        if (r < 1u || r > 3u)
        {
            r = 1u;
        }
        tring = r;
        if (sc == AT_SCENE_TURNTABLE)
        {
            tz = CZ_TP;
        }
        else if (sc == AT_SCENE_ROUGH)
        {
            tz = CZ_ROUGH;
        }
        else
        {
            tz = CZ_STORAGE;
        }
    }
    else
    {
        tz = CZ_START;
        chassis_auto_dbg_scene = 0xFFu;
        chassis_auto_dbg_kind  = 0xFFu;
        chassis_auto_dbg_ring  = 0u;
        chassis_auto_dbg_step  = -2;
    }
    cauto_build_route(tz, tring);
}

/* 推进到下一站(或回启停2) —— 只会在本站完整收尾(臂 done + DONE_ACK OK)后调用 */
static void cam_advance(void)
{
    /* 串站防御: 底盘要开动了, 绝不允许还有残留的取放请求/未清完成位 */
    auto_grab_req.active = 0u;
    auto_grab_req.done   = 0u;
    s_cauto_step++;
    s_cauto_timer = 0;
    s_cauto_wd = 0;
    s_cauto_nav_tmo = 0;      /* 新一段导航重新计时 */
    s_cauto_done_sent = 0u;
    if (s_cauto_step >= (int16_t)auto_plan_n)
    {
        s_cauto_home = 1u;   /* 全部完成 -> 回启停2 */
    }
    cam_route_step();
    s_cauto_stage = CAS_NAV;
}

static void cam_set_err(void)
{
    s_cauto_master = CAM_ERR;
    auto_grab_req.active = 0u;
    auto_grab_req.done = 0u;
}

/* 沿当前折线(道带内轴对齐)逐点导航; 到最终停车点并摆好车头朝向后返回 1 */
static uint8_t cauto_leg_nav(fp32 *vxo, fp32 *vyo, fp32 *wzo)
{
    fp32 yaw = chassis_move.INS_angle[INS_YAW_ADDRESS_OFFSET];
    fp32 rel, de;
    fp32 gx, gy, ex, ey, dist;
    fp32 face, cosy, siny, vxb, vyb, vx, vy, wz;

    vx = 0.0f;
    vy = 0.0f;
    wz = 0.0f;

    chassis_auto_dbg_zone = s_cauto_tz;
    chassis_auto_dbg_wp   = s_cauto_wp_i;

    /* 途经点全走完(已在终点): 原地旋转摆到该站车头朝向 */
    if (s_cauto_wp_i >= s_cauto_wp_n)
    {
        if (s_cauto_face_en != 0u)
        {
            rel = yaw - s_cauto_orgyaw;
            de  = s_cauto_face_yaw - rel;
            if (de >  3.14159265f) { de -= 6.2831853f; }
            if (de < -3.14159265f) { de += 6.2831853f; }
            if (fabsf(de) > 0.02f)
            {
                wz = fp32_constrain(de * 1.6f, -CAUTO_WZ_MAX, CAUTO_WZ_MAX);
                *vxo = 0.0f;
                *vyo = 0.0f;
                *wzo = wz;
                return 0;
            }
        }
        return 1;   /* 到位且已摆好头 */
    }

    gx  = s_cauto_wpx[s_cauto_wp_i];
    gy  = s_cauto_wpy[s_cauto_wp_i];
    ex  = gx - chassis_move.position_x;
    ey  = gy - chassis_move.position_y;
    dist = sqrtf(ex * ex + ey * ey);
    chassis_auto_dbg_dist_mm = dist * 1000.0f;

    if (dist < CAUTO_ARRIVE_M)
    {
        /* 到当前途经点: 中间点则推进下一段, 最终点则进入摆头 */
        if (s_cauto_wp_i + 1u < s_cauto_wp_n)
        {
            s_cauto_wp_i++;
        }
        else
        {
            s_cauto_wp_i = s_cauto_wp_n;
        }
        *vxo = 0.0f;
        *vyo = 0.0f;
        *wzo = 0.0f;
        return 0;
    }

    face = atan2f(ey, ex);
    rel  = yaw - s_cauto_orgyaw;
    de   = face - rel;
    if (de >  3.14159265f) { de -= 6.2831853f; }
    if (de < -3.14159265f) { de += 6.2831853f; }
    cosy = cosf(yaw);
    siny = sinf(yaw);
    vxb  = (ex * cosy + ey * siny);
    vyb  = (-ex * siny + ey * cosy);
    if (fabsf(de) > 1.2f)
    {
        vx = 0.0f;
        vy = 0.0f;
    }
    else
    {
        vx = fp32_constrain(vxb * 1.8f, -CAUTO_SPEED_MAX, CAUTO_SPEED_MAX);
        vy = fp32_constrain(vyb * 1.8f, -CAUTO_SPEED_MAX, CAUTO_SPEED_MAX);
    }
    wz = fp32_constrain(de * 1.6f, -CAUTO_WZ_MAX, CAUTO_WZ_MAX);
    *vxo = vx;
    *vyo = vy;
    *wzo = wz;
    return 0;
}

/* 自动模式周期: 由 CHASSIS_AUTO 分支调用(1ms) */
static void chassis_auto_update(fp32 *vx_o, fp32 *vy_o, fp32 *wz_o)
{
    fp32 vx = 0.0f, vy = 0.0f, wz = 0.0f;
    fp32 yaw = chassis_move.INS_angle[INS_YAW_ADDRESS_OFFSET];

    /* 模式切换沿检测: 任意模式变化都记录, 以便 AUTO→遥控→AUTO 再次进入也能复位。
     * 旧写法只在“last!=AUTO”时进块, 退出 AUTO 后再进 AUTO 不会复位,
     * 会继续沿用上一轮残留的 master/stage/step 状态直接开跑。 */
    if (s_cauto_last_mode != chassis_move.chassis_mode)
    {
        s_cauto_last_mode = chassis_move.chassis_mode;   /* 先记录新模式 */
        if (chassis_move.chassis_mode == CHASSIS_AUTO)
        {
            /* ---- 刚进入 AUTO: 复位总控并自动发 START(只做一次) ---- */
            s_cauto_orgx   = chassis_move.position_x;
            s_cauto_orgy   = chassis_move.position_y;
            s_cauto_orgyaw = yaw;
            s_cauto_master  = CAM_BOOT;
            s_cauto_stage   = CAS_NAV;
            s_cauto_step    = -1;
            s_cauto_home    = 0u;
            s_cauto_vision  = 0u;
            s_cauto_align   = 0u;
            s_cauto_retry   = 0u;
            s_cauto_start_sent = 0u;
            s_cauto_done_sent  = 0u;
            s_cauto_timer   = 0;
            s_cauto_wd      = 0;
            s_cauto_nav_tmo = 0;
            PID_clear(&chassis_heading_pid);
            /* 从启停出发: 复位折线导航状态; 真视觉且尚无任务码才先自动去二维码板扫码
             * (demo=1 纯测导航: 原地等 1.5s 自动造计划, 不先跑去二维码板) */
            s_cauto_zone    = CZ_START;
            s_cauto_ring    = 1u;
            s_cauto_wp_n    = 0u;
            s_cauto_wp_i    = 0u;
            s_cauto_face_en = 0u;
            s_cauto_tz      = CZ_START;
            s_cauto_tr      = 1u;
            s_cauto_qr_go   = (chassis_auto_demo != 0u) ? 0u : 1u;
            if (s_cauto_qr_go != 0u)
            {
                cauto_build_route(CZ_QR, 1u);   /* 预布 S0: 启停→二维码板 */
            }
            *vx_o = 0.0f; *vy_o = 0.0f; *wz_o = 0.0f;
            return;   /* 进入当拍只复位不出车 */
        }
    }

    if (chassis_move.chassis_mode != CHASSIS_AUTO)
    {
        return;      /* 退出 AUTO: 总控停(速度由各模式分支给) */
    }

    switch (s_cauto_master)
    {
        case CAM_BOOT:
            /* 1) 自动 START; 2) 若还没任务码: 先开到二维码板位扫码;
             *    收到 TASK_PLAN(auto_plan 就绪)或 demo 后进入逐站执行 */
            if (s_cauto_start_sent == 0u)
            {
                s_cauto_start_sent = 1u;
                jetson_send_start(jetson_cmd_run_id);
            }
            if (auto_plan_ready != 0u)
            {
                s_cauto_vision = 1u;
                s_cauto_step = -1;
                s_cauto_home = 0u;
                s_cauto_qr_go = 0u;
                s_cauto_master = CAM_RUN;
                cam_advance();
            }
            else if (chassis_auto_demo != 0u && s_cauto_timer > 1500u)
            {
                cam_demo_build();
                s_cauto_qr_go = 0u;
                s_cauto_master = CAM_RUN;
                cam_advance();
            }
            else if (s_cauto_qr_go != 0u)
            {
                /* 去二维码板观察点(停点本身, 摆头+x), 到位原地等 Jetson 扫码 */
                if (cauto_leg_nav(&vx, &vy, &wz) != 0u)
                {
                    s_cauto_nav_tmo = 0;
                    s_cauto_qr_go = 0u;
                    s_cauto_zone  = CZ_QR;   /* 记录现处于二维码板 */
                    s_cauto_ring  = 1u;
                }
                else
                {
                    s_cauto_nav_tmo++;
                    if (s_cauto_nav_tmo > CAUTO_NAV_TIMEOUT_MS)
                    {
                        cam_set_err();   /* 去扫码也到不了 → 停车报警人工处理 */
                    }
                }
            }
            /* BOOT 周期计时(ms): 等码/去扫码期间累加, 供 chassis_auto_demo 1.5s 自动启动 */
            s_cauto_timer++;
            break;

        case CAM_RUN:
        {
            switch (s_cauto_stage)
            {
                case CAS_NAV:
                    if (cauto_leg_nav(&vx, &vy, &wz) != 0u)
                    {
                        s_cauto_nav_tmo = 0;   /* 一段导航完成: 计时清零 */
                        /* 到位并摆好车头朝向: 记录本站在哪(供下次折线) */
                        s_cauto_zone = s_cauto_tz;
                        s_cauto_ring = s_cauto_tr;
                        if (s_cauto_home != 0u)
                        {
                            s_cauto_master = CAM_DONE;   /* 已回启停 */
                        }
                        else if (s_cauto_vision == 0u)
                        {
                            s_cauto_timer = 0;
                            s_cauto_stage = CAS_DONE_SEND;  /* demo */
                        }
                        else
                        {
                            s_cauto_wd = 0;
                            s_cauto_retry = 0u;   /* 新站: 清 RETRY 标志 */
                            s_cauto_stage = CAS_SIGHT;
                        }
                    }
                    else
                    {
                        /* 导航看门狗: 到不了当前站(堵住/打滑不前/摆头失败)→停车报警 */
                        s_cauto_nav_tmo++;
                        if (s_cauto_nav_tmo > CAUTO_NAV_TIMEOUT_MS)
                        {
                            cam_set_err();
                        }
                    }
                    break;

                case CAS_SIGHT:
                    /* 到站先伸臂到抓取侧观测位(不夹, 相机看到场地目标)后才发 REQ */
                    if (auto_grab_req.active == 0u)
                    {
                        /* 发布顺序: 先清完成位、再填数据, 最后才置 active=1(作为提交)。
                         * 若先把 active 置 1 再填 cmd/act/tray, 机械臂在本任务隙里看到
                         * active==1 会用上一站残留参数建路径(取错点/错命令)。 */
                        auto_grab_req.done   = 0u;
                        auto_grab_req.cmd    = AT_GRAB_CMD_SIGHT;
                        auto_grab_req.act    = auto_plan[s_cauto_step].kind;
                        auto_grab_req.scene  = auto_plan[s_cauto_step].scene;
                        auto_grab_req.ring   = auto_plan[s_cauto_step].ring;
                        auto_grab_req.tray   = (uint8_t)(s_cauto_step % 3) + 1u;
                        auto_grab_req.no_vis = 0u;   /* AUTO: 正常(允许视觉纠偏) */
                        auto_grab_req.active = 1u;
                        s_cauto_wd = 0;
                    }
                    if (auto_grab_req.done != 0u)
                    {
                        auto_grab_req.active = 0u;
                        auto_grab_req.done   = 0u;
                        s_cauto_wd = 0;
                        if (s_cauto_retry != 0u)
                        {
                            /* RETRY 续做: 已回观测位, 直接等新许可(不再重发 REQ) */
                            s_cauto_retry = 0u;
                            s_cauto_stage = CAS_WAIT_PERM;
                        }
                        else
                        {
                            s_cauto_stage = CAS_REQ;   /* 已能观测 -> 发 REQ 等视觉许可 */
                        }
                    }
                    else
                    {
                        s_cauto_wd++;
                        if (s_cauto_wd > CAUTO_ARM_TIMEOUT_MS)
                        {
                            cam_set_err();
                        }
                    }
                    break;

                case CAS_REQ:
                    cam_send_req();
                    s_cauto_wd = 0;
                    s_cauto_stage = CAS_WAIT_PERM;
                    break;

                case CAS_WAIT_PERM:
                {
                    uint16_t ev;
                    s_cauto_wd++;
                    ev = jetson_ev_take();
                    if ((ev & (JETSON_EV_ERROR | JETSON_EV_TIMEOUT)) != 0u)
                    {
                        cam_set_err();
                        break;
                    }
                    if (s_cauto_align != 0u)
                    {
                        if ((ev & JETSON_EV_ALIGN_READY) != 0u)
                        {
                            s_cauto_wd = 0;
                            s_cauto_stage = CAS_ARM;
                        }
                    }
                    else
                    {
                        if ((ev & JETSON_EV_GRASP_READY) != 0u)
                        {
                            jetson_send_exec(jetson.cur_seq);   /* 立即 EXEC */
                            s_cauto_wd = 0;
                            s_cauto_stage = CAS_WAIT_ACK;
                        }
                        else if ((ev & JETSON_EV_GRASP_REVOKED) != 0u)
                        {
                            s_cauto_wd = 0;   /* 撤销: 继续等新许可 */
                        }
                    }
                    if (s_cauto_wd > CAUTO_PERM_TIMEOUT_MS)
                    {
                        cam_set_err();
                    }
                    break;
                }

                case CAS_WAIT_ACK:
                {
                    uint16_t ev;
                    s_cauto_wd++;
                    ev = jetson_ev_take();
                    if ((ev & (JETSON_EV_ERROR | JETSON_EV_TIMEOUT)) != 0u)
                    {
                        cam_set_err();
                        break;
                    }
                    if ((ev & JETSON_EV_EXEC_ACK) != 0u)
                    {
                        s_cauto_wd = 0;
                        s_cauto_stage = CAS_ARM;
                    }
                    else if ((ev & JETSON_EV_GRASP_REVOKED) != 0u)
                    {
                        s_cauto_wd = 0;
                        s_cauto_stage = CAS_WAIT_PERM;
                    }
                    else if (s_cauto_wd > CAUTO_ACK_TIMEOUT_MS)
                    {
                        cam_set_err();
                    }
                    break;
                }

                case CAS_ARM:
                {
                    if (s_cauto_vision == 0u)
                    {
                        /* demo: 无机械臂动作, 直接按 CAUTO_HOLD_MS 停留后推进 */
                        if (++s_cauto_timer >= CAUTO_HOLD_MS)
                        {
                            s_cauto_timer = 0;
                            cam_advance();
                        }
                        break;
                    }
                    if (auto_grab_req.active == 0u)
                    {
                        /* 发布顺序同 CAS_SIGHT: done/数据先填, active=1 最后提交 */
                        auto_grab_req.done   = 0u;
                        auto_grab_req.cmd    = AT_GRAB_CMD_GRAB;
                        auto_grab_req.act    = auto_plan[s_cauto_step].kind;
                        auto_grab_req.scene  = auto_plan[s_cauto_step].scene;
                        auto_grab_req.ring   = auto_plan[s_cauto_step].ring;
                        /* 每批每区固定 3 件: 第 0/3/6/.. 件用盘1, 1/4/7 用盘2, 2/5/8 用盘3 */
                        auto_grab_req.tray   = (uint8_t)(s_cauto_step % 3) + 1u;
                        auto_grab_req.no_vis = 0u;   /* AUTO: 正常(允许视觉纠偏) */
                        auto_grab_req.active = 1u;
                        s_cauto_wd = 0;
                    }
                    if (auto_grab_req.done != 0u)
                    {
                        auto_grab_req.active = 0u;
                        auto_grab_req.done   = 0u;
                        s_cauto_done_sent = 0u;
                        s_cauto_stage = CAS_DONE_SEND;
                        s_cauto_wd = 0;
                    }
                    else
                    {
                        s_cauto_wd++;
                        if (s_cauto_wd > CAUTO_ARM_TIMEOUT_MS)
                        {
                            cam_set_err();
                        }
                    }
                    break;
                }

                case CAS_DONE_SEND:
                {
                    if (s_cauto_vision == 0u)
                    {
                        if (++s_cauto_timer >= CAUTO_HOLD_MS)
                        {
                            s_cauto_timer = 0;
                            cam_advance();
                        }
                        break;
                    }
                    if (s_cauto_done_sent == 0u)
                    {
                        s_cauto_done_sent = 1u;
                        jetson_send_done(auto_plan[s_cauto_step].seq, 1u);   /* OK */
                    }
                    s_cauto_wd = 0;
                    s_cauto_stage = CAS_ACK;
                    break;
                }

                case CAS_ACK:
                {
                    uint16_t ev;
                    s_cauto_wd++;
                    ev = jetson_ev_take();
                    if ((ev & JETSON_EV_DONE_ACK) != 0u)
                    {
                        if (jetson.done_result_ok != 0u)
                        {
                            cam_advance();            /* OK -> 下一站/回启停 */
                        }
                        else
                        {
                            s_cauto_done_sent = 0u;   /* RETRY -> 回观测位续做本站 */
                            s_cauto_wd = 0;
                            s_cauto_retry = 1u;
                            s_cauto_stage = CAS_SIGHT;
                        }
                    }
                    else if ((ev & (JETSON_EV_ERROR | JETSON_EV_TIMEOUT)) != 0u)
                    {
                        cam_set_err();
                    }
                    else if (s_cauto_wd > CAUTO_ACK_TIMEOUT_MS)
                    {
                        cam_set_err();
                    }
                    break;
                }

                default:
                    break;
            }
            break;
        }

        case CAM_ERR:
        case CAM_DONE:
        default:
            vx = 0.0f; vy = 0.0f; wz = 0.0f;
            break;
    }

    *vx_o = vx;
    *vy_o = vy;
    *wz_o = wz;
    chassis_auto_dbg_state = s_cauto_master;
    chassis_auto_dbg_stage = s_cauto_stage;
    chassis_auto_dbg_vision = s_cauto_vision;
    (void)auto_plan_ready;
}

static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

    switch (chassis_move_control->chassis_mode)
    {
        case CHASSIS_ZERO_FORCE:
        {
            vx_set = 0.0f;
            vy_set = 0.0f;
            wz_set = 0.0f;
            break;
        }

        case CHASSIS_REMOTE_CONTROL:
        {
            if (chassis_move_control->heading_en)
            {
                /* === S3上: 恒定航向(角度闭环) ===
                 * 右杆机体系直给 vx/vy(车头被锁在 yaw_ref, 前进即朝设定yaw) + CH3 重设yaw_ref
                 * wz 由航向PID把车头拉回 yaw_ref(yaw_ref 默认0, 推CH3转向即实时重设) */
                fp32 vx_w = 0.0f, vy_w = 0.0f, wz_rc = 0.0f;
                chassis_rc_to_control_vector(&vx_w, &vy_w, chassis_move_control);
                vy_w = -vy_w;
                if (chassis_move_control->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] > CHASSIS_RC_DEADLINE ||
                    chassis_move_control->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] < -CHASSIS_RC_DEADLINE)
                {
                    wz_rc = CHASSIS_WZ_RC_SIGN * CHASSIS_WZ_RC_SEN * chassis_move_control->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
                }
                chassis_heading_line_hold(chassis_move_control, vx_w, vy_w, wz_rc,
                                          &vx_set, &vy_set, &wz_set);
            }
            else
            {
                /* === S3下: 纯手动(无角度修正, 改动前的原始行为) ===
                 * 右杆直给 vx/vy, CH3 直给 wz, 不按角度旋转/不闭环 */
                chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_control);
                vy_set = -vy_set;
                wz_set = CHASSIS_WZ_RC_SIGN * CHASSIS_WZ_RC_SEN * chassis_move_control->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
            }
            break;
        }

        case CHASSIS_AUTO:
        {
            /* 全自动: 进AUTO后按 auto_plan 逐站导航; 到站占位停留;
             * 机械臂动作 + Jetson 握手在此状态机后续阶段接入 */
            chassis_auto_update(&vx_set, &vy_set, &wz_set);
            break;
        }

        default:
            break;
    }

    vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    wz_set = fp32_constrain(wz_set, chassis_move_control->wz_min_speed, chassis_move_control->wz_max_speed);

    /* ===== 速度斜坡规划: 用"当前速度(上次输出)+目标速度"按最大加速度逼近 =====
     * vx/vy 线速度(m/s), wz 角速度(rad/s); 每个控制周期(1ms)最多变化 acc*dt */
    {
        fp32 dv = CHASSIS_ACC_MPS2 * CHASSIS_CONTROL_TIME;
        fp32 dw = CHASSIS_ACC_WZ_RADPS2 * CHASSIS_CONTROL_TIME;

        chassis_move_control->vx_set = CHASSIS_RAMP_TO(chassis_move_control->vx_set, vx_set, dv);
        chassis_move_control->vy_set = CHASSIS_RAMP_TO(chassis_move_control->vy_set, vy_set, dv);
        chassis_move_control->wz_set = CHASSIS_RAMP_TO(chassis_move_control->wz_set, wz_set, dw);
    }

    fp32 D = chassis_move_control->wheelbase_sum;
    fp32 R = chassis_move_control->wheel_radius;
    fp32 max_vector = 0.0f, vector_rate = 0.0f;

    /* 逆解: (vx_set,vy_set,wz_set) -> 四轮目标转速(rad/s)
     * 轮序 0=左前LF 1=右前RF 2=左后LB 3=右后RB */
    fp32 tmp[4];
    tmp[0] = ( chassis_move_control->vx_set - chassis_move_control->vy_set - D * chassis_move_control->wz_set) / R; /* LF */
    tmp[1] = ( chassis_move_control->vx_set + chassis_move_control->vy_set + D * chassis_move_control->wz_set) / R; /* RF */
    tmp[2] = ( chassis_move_control->vx_set + chassis_move_control->vy_set - D * chassis_move_control->wz_set) / R; /* LB */
    tmp[3] = ( chassis_move_control->vx_set - chassis_move_control->vy_set + D * chassis_move_control->wz_set) / R; /* RB */

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_control->wheel[i].wheel_speed_set = tmp[i] * chassis_move_control->wheel[i].rev;

        if (fabs(chassis_move_control->wheel[i].wheel_speed_set) > max_vector)
        {
            max_vector = fabs(chassis_move_control->wheel[i].wheel_speed_set);
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_move_control->wheel[i].wheel_speed_set *= vector_rate;
        }
    }
}

/**
  * @brief          底盘控制数据准备
  * @note           将 wheel_speed_set (rad/s) 转换为 rpm, 填充到电机控制结构体
  *                 遥控器失联时自动将所有轮速置零
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    if (chassis_move_control_loop == NULL)
    {
        return;
    }

    if (toe_is_error(DBUS_TOE))
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_move_control_loop->wheel[i].wheel_speed_set = 0.0f;
        }
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        /* 速度斜坡已在 chassis_set_contorl 对 vx/vy/wz 限幅, 这里直接换算成 RPM */
        chassis_move_control_loop->wheel[i].motor->ctrl.vel_set =
            chassis_move_control_loop->wheel[i].wheel_speed_set * 9.5493f;  /* rad/s -> RPM */
        chassis_move_control_loop->wheel[i].motor->ctrl.acc_set = 0.0f;     /* 驱动侧直启 */
    }
}

/**
  * @brief          底盘CAN发送 (每周期发2个电机)
  * @note           轮询顺序: (0,1)→(2,3)→(0,1)→...
  *                 每个电机更新周期 = 2 × 1ms = 2ms
  *                 目标转速来源: chassis_control_loop 填入的 motor->ctrl.vel_set(RPM)
  *                 底层为硬件CAN发送(canx_bsp_send_ext_data)
  */
static void chassis_can_send(chassis_move_t *chassis_move_can_send)
{
    static uint8_t send_pair = 0;
    static uint8_t stat_tick = 0;   /* 状态轮询节拍 */
    static uint8_t stat_idx = 0;    /* 当前轮询到第几台电机 */

    if (chassis_move_can_send == NULL)
    {
        return;
    }

    uint8_t idx0 = send_pair * 2;
    uint8_t idx1 = idx0 + 1;

    /* 一组(2台)电机速度指令 (底层走硬件CAN发送) */
    zdt_emm_speed_ctrl(&hcan1, chassis_move_can_send->wheel[idx0].motor);
    zdt_emm_speed_ctrl(&hcan1, chassis_move_can_send->wheel[idx1].motor);

    /* 轮询0x3A状态字: 每4ms读1台, 单台16ms刷新一次
     * 解析后写入 motor->para.state_flags:
     *   bit0=已使能(锁轴) bit2=堵转标志 bit3=堵转保护 bit7=掉电 */
    if ((++stat_tick & 0x03U) == 0)
    {
        zdt_read_status(&hcan1, chassis_move_can_send->wheel[stat_idx].motor);
        stat_idx = (stat_idx + 1) & 0x03;
    }

    send_pair = (send_pair + 1) & 0x01;
}
