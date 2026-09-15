/**
  ******************************************************************************
  * @file       jetson_task.h
  * @brief      Jetson 视觉对接模块 (电控侧, USART1)
  *
  *             协议见工程根目录《电控与 Jetson 对接协议.md》:
  *               - 全双工串口: USART1(PB7 RX / PA9 TX), 115200-8N1 无校验无流控
  *                 (2026-09-08 由 USART6/PG9-14 整体迁移)
  *               - 帧格式: @NAME,FIELD1,FIELD2,...*CCCC\n
  *               - CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF, 无反射, XorOut 0)
  *                 只对 "NAME,FIELD1,..." 正文(不含 @ 和 * 和 \n) 计算
  *               - Jetson 对无效帧静默丢弃, 不回 NACK, 电控必须自己做超时
  *
  *             本模块不创建任何 FreeRTOS 任务:
  *               - 接收/拆帧/CRC/分发 全部在 USART1 中断(USART1_IRQHandler)里完成,
  *                 因此 stm32f4xx_it.c 里 CubeMX 自动生成的同名中断已注释,
  *                 重新用 CubeMX 生成工程后需再次删除(见根目录 readme.md)。
  *               - 解析结果(坐标/环号/颜色/回复/错误)全部写入全局 extern 变量 jetson,
  *                 事件位见 JETSON_EV_*, 任何外部代码(底盘/机械臂/主循环)直接读。
  *               - 启动: 调用一次 jetson_link_init() 开启 USART1 DMA 接收
  *                 (已放在 StartDefaultTask 中)。
  *               - 周期服务: jetson_service() 可选, 只处理 Watch 调试命令与本地超时,
  *                 建议放到任意现有 1ms 周期任务里调用一次(已放 StartDefaultTask)。
  *                 若不需要 Watch 命令/本地超时, 也可以完全不调用它(中断解析照常)。
  *
  *             本模块只做"对接与数据", 不直接驱动底盘/机械臂;
  *             收到事件后由上层决定动作, 动作完成后调用 jetson_send_done() 握手。
  ******************************************************************************
  */
#ifndef JETSON_TASK_H
#define JETSON_TASK_H

#include <stdint.h>

/* ==================== 事件位 (jetson.events / jetson_ev_take) ============ */
#define JETSON_EV_READY            (1u << 0)   /* 收到 READY */
#define JETSON_EV_TASK_PLAN        (1u << 1)   /* 收到 TASK_PLAN (task_code 已解析) */
#define JETSON_EV_ACCEPTED         (1u << 2)   /* 收到 ACCEPTED (REQ 受理) */
#define JETSON_EV_GRASP_READY      (1u << 3)   /* 收到 GRASP_READY (PICK 许可, 需回 EXEC) */
#define JETSON_EV_GRASP_REVOKED    (1u << 4)   /* 收到 GRASP_REVOKED (许可撤销) */
#define JETSON_EV_EXEC_ACK         (1u << 5)   /* 收到 EXEC_ACK (允许机械动作) */
#define JETSON_EV_ALIGN_READY      (1u << 6)   /* 收到 ALIGN_READY (PLACE/STACK 对准坐标) */
#define JETSON_EV_DONE_ACK         (1u << 7)   /* 收到 DONE_ACK (seq 匹配) */
#define JETSON_EV_ABORTED          (1u << 8)   /* 收到 ABORTED */
#define JETSON_EV_ERROR            (1u << 9)   /* 收到 ERROR */
#define JETSON_EV_TIMEOUT          (1u << 10)  /* 本机等待回复超时 */

/* ==================== 我方会话阶段 (电控侧视角) ========================== */
typedef enum
{
    JETSON_PH_IDLE = 0,          /* 空闲 (未 START / 已 ABORT) */
    JETSON_PH_START_WAIT,        /* 已发 START, 等 READY */
    JETSON_PH_TASK_WAIT,         /* 已 READY, 等 TASK_PLAN */
    JETSON_PH_TASK_READY,        /* 任务已锁定, 可发 REQ */
    JETSON_PH_REQ_WAIT,          /* 已发 REQ, 等 ACCEPTED */
    JETSON_PH_PICK_WAIT_GRASP,   /* PICK 受理, 等 GRASP_READY */
    JETSON_PH_PICK_WAIT_EXEC,    /* GRASP_READY 到, 等上层 EXEC (valid_ms 内) */
    JETSON_PH_PICK_WAIT_ACK,     /* EXEC 已发, 等 EXEC_ACK */
    JETSON_PH_PICK_ACT,          /* EXEC_ACK 到, 允许机械动作 */
    JETSON_PH_ALIGN_WAIT,        /* PLACE/STACK 受理, 等 ALIGN_READY */
    JETSON_PH_ALIGN_ACT,         /* ALIGN_READY 到, 允许对准+放置 */
    JETSON_PH_DONE_WAIT,         /* DONE 已发, 等 DONE_ACK */
    JETSON_PH_ERROR,             /* 出错/超时, 等上层处理(ABORT 等) */
} jetson_phase_e;

/* ==================== 解析结果 (Keil Watch 可直接看 jetson) ============= */
typedef struct
{
    uint8_t  phase;                     /* jetson_phase_e */
    char     run_id[24];                /* 本轮 run_id */
    char     cur_seq[24];               /* 当前动作 REQ 序号 */
    char     cur_kind[8];               /* 当前动作类型 PICK/PLACE/STACK */
    uint8_t  scene_idx;                 /* 当前动作所在区: AT_SCENE_*(见 auto_task.h) */

    /* --- TASK_PLAN 整轮任务 --- */
    uint8_t  task_plan_ok;              /* 1=TASK_PLAN 已收到并解析(task_code 已写入) */
    char     task_code[24];             /* 原样 "AAA+BBB+CCC+DDD" */
    uint8_t  colors[6];                 /* c1..c6 (Jetson 自动队列) */

    /* --- GRASP_READY (PICK 许可) --- */
    uint8_t  grasp_ok;                  /* 1=有新 GRASP_READY 数据 */
    char     grasp_seq[24];
    uint8_t  grasp_color;               /* 1红..6浅蓝 */
    float    grasp_x, grasp_y;          /* unit 坐标系稳定中心 */
    char     grasp_unit;                /* 'P'=PX 'M'=MM */
    float    grasp_px, grasp_py;        /* 原图像素 (PX时与x/y相同) */
    float    grasp_conf;                /* 工程质量分 0~1 */
    uint16_t grasp_stable;              /* 稳定帧数 */
    uint16_t grasp_valid_ms;            /* 许可有效时间 ms (需在此内发 EXEC) */
    uint8_t  grasp_seq_ok;              /* 1=seq 与当前 REQ 一致 */
    float    grasp_tar_x, grasp_tar_y;  /* 视觉纠偏(mm): 机械臂前向、底盘侧向 */

    /* --- ALIGN_READY (PLACE/STACK 对准坐标) --- */
    uint8_t  align_ok;
    char     align_seq[24];
    char     align_kind[8];             /* PLACE / STACK */
    int16_t  align_target;              /* PLACE=ring_id, STACK=color_id */
    float    align_x, align_y, align_yaw; /* 中心坐标 + 偏航(度, 预留) */
    char     align_unit;
    float    align_conf;
    uint16_t align_stable;
    float    align_tar_x, align_tar_y;  /* 视觉纠偏(mm): 机械臂前向、底盘侧向 */
    uint8_t  align_seq_ok;

    /* --- DONE_ACK --- */
    uint8_t  done_ack_ok;
    char     done_seq[24];
    uint8_t  done_result_ok;            /* 1=OK 0=RETRY */

    /* --- ABORTED / ERROR --- */
    uint8_t  aborted;                   /* 1=已回到空闲 */
    char     err_seq[24];
    char     err_reason[24];

    char     last_frame[12];            /* 最近有效帧名(调试) */
    uint16_t events;                    /* 累积事件位(Watch, 由 jetson_ev_take 清零) */
    uint32_t rx_ok;                     /* 有效帧计数 */
    uint32_t tx_ok;                     /* 成功发送帧计数 */
    uint32_t crc_err;                   /* CRC 校验失败计数 */
    uint32_t fmt_err;                   /* 帧格式错误计数 */
    uint32_t seq_mismatch;              /* 回复 seq 与当前不符计数 */
    uint32_t tx_drop;                   /* 发送队列溢出丢弃计数 */
} jetson_glue_t;

extern jetson_glue_t jetson;

/* ============ 调试命令 (Keil Watch 手动置 1 触发, 处理后自动清 0) ======= */
extern uint8_t jetson_cmd_start;        /* 置1: 发 START, run_id 用 jetson_cmd_run_id */
extern uint8_t jetson_cmd_abort;        /* 置1: 发 ABORT */
extern uint8_t jetson_cmd_demo_pick;    /* 置1: 在 TASK_READY 下发 REQ PICK(队列首色) */
extern uint8_t jetson_cmd_demo_place;   /* 置1: 在 TASK_READY 下发 REQ PLACE ROUGH 环1 */
extern uint8_t jetson_cmd_demo_stack;   /* 置1: 在 TASK_READY 下发 REQ STACK STORAGE 色首 环1 */
extern uint8_t jetson_cmd_exec;         /* 置1: 对当前 GRASP_READY 发 EXEC */
extern uint8_t jetson_cmd_done;         /* 置1: 发 DONE OK; 置2: 发 DONE FAIL */
extern char    jetson_cmd_run_id[24];   /* Watch 可改本轮标识 */

/* ==================== 初始化 / 周期服务 ================================= */
extern void jetson_link_init(void);     /* 启动 USART1 DMA 接收(调一次, 已放 StartDefaultTask) */
extern void jetson_service(void);       /* 可选周期服务: 处理 Watch 调试命令 + 本地超时
                                            (解析不在里面, 中断已完成; 放任意 1ms 周期代码里调) */

/* ==================== 供上层(动作流程)调用的 API ========================= */
extern uint8_t  jetson_phase(void);                 /* 读当前阶段 */
extern uint16_t jetson_ev_take(void);               /* 取并清事件位 */
/* START/ABORT: 任何状态都允许(会清任务), run_id 非空 */
extern void    jetson_send_start(const char *run_id);
extern void    jetson_send_abort(const char *run_id);
/* REQ: 仅在 TASK_READY 允许; scene 允许 NULL(用默认 TURNTABLE);
   color/ring 允许 NULL(用 Jetson 队列/默认)。返回 1=已发出 */
extern uint8_t jetson_send_req_pick (const char *seq, const char *scene, const char *color);
extern uint8_t jetson_send_req_place(const char *seq, const char *scene, const char *ring);
extern uint8_t jetson_send_req_stack(const char *seq, const char *scene, const char *color, const char *ring);
/* EXEC: 仅 PICK 且处于 PICK_WAIT_EXEC 时允许; 必须在 grasp_valid_ms 内发出 */
extern uint8_t jetson_send_exec(const char *seq);
/* DONE: 仅在 PICK_ACT/ALIGN_ACT 时允许; ok=1 发 OK, ok=0 发 FAIL */
extern uint8_t jetson_send_done(const char *seq, uint8_t ok);

#endif /* JETSON_TASK_H */
