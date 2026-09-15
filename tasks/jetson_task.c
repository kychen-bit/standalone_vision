/**
  ******************************************************************************
  * @file       jetson_task.c
  * @brief      Jetson 视觉对接模块 (电控侧, USART1) 实现
  *
  *  参考协议文档: 《电控与 Jetson 对接协议.md》
  *  2026-09-08: 由 USART6(PG9/PG14) 整体迁移到 USART1(PB7 RX / PA9 TX), 115200-8N1;
  *              收发走 USART1 现成 DMA(RX=DMA2_Stream5, TX=DMA2_Stream7)。
  *  本模块不创建独立 FreeRTOS 任务: 接收/拆帧/CRC/分发全部在 USART1 中断里完成,
  *  对外只暴露全局变量(jetson)与 API, 供其它代码直接读写。
  *  本文件同时接管 USART1 中断 (对应 Keil 向量表 USART1_IRQHandler),
  *  因此 stm32f4xx_it.c 里 CubeMX 自动生成的 USART1_IRQHandler 已注释掉,
  *  重新用 CubeMX 生成工程后需要再次删除 (见根目录 readme.md)。
  ******************************************************************************
  */

#include "jetson_task.h"
#include "main.h"
#include "usart.h"
#include "task_param.h"
#include "auto_task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/* ==========================================================================
 * 配置宏
 * ======================================================================== */
#define JRX_BUF_SIZE   512u   /* USART1 接收 DMA 缓冲长度 (circular) */
#define JFRAME_MAX     224    /* 单帧最大字节 (正文 <=176, @*CCCC\n 等) */
#define JTXQ_DEPTH     8u     /* 发送队列深度 */
#define JTX_MAX        224    /* 发送单帧最大字节 */

/* 各等待阶段超时 (ms), 0=不超时 */
#define JTO_START_MS    5000u
#define JTO_TASK_MS         0u  /* READY 后无限等待 TASK_PLAN */
#define JTO_REQ_MS      3000u
#define JTO_GRASP_MS    25000u
#define JTO_EXEC_MS     1200u
#define JTO_ACK_MS      1500u
#define JTO_ALIGN_MS    25000u
#define JTO_DONE_MS     35000u

/* ==========================================================================
 * 输出结构 + 调试命令全局
 * ======================================================================== */
jetson_glue_t jetson;

uint8_t jetson_cmd_start = 0;
uint8_t jetson_cmd_abort = 0;
uint8_t jetson_cmd_demo_pick = 0;
uint8_t jetson_cmd_demo_place = 0;
uint8_t jetson_cmd_demo_stack = 0;
uint8_t jetson_cmd_exec = 0;
uint8_t jetson_cmd_done = 0;
char    jetson_cmd_run_id[24] = "RUN001";

/* 内部状态 */
static uint16_t s_ev = 0;                /* 内部累积事件位 */
static uint32_t s_wait_t0 = 0;           /* 进入等待阶段的 tick */
static uint32_t s_seq_no = 0;            /* demo 自动序号 */

/* --------------------- 接收: DMA 缓冲(中断内直接解析) ------------------- */
static uint8_t  s_rxbuf[JRX_BUF_SIZE];
static volatile uint16_t s_rx_last = 0;  /* DMA circular 已消费到的位置 [0..N] */

/* --------------------- 发送: DMA 队列 ----------------------------------- */
typedef struct
{
    uint8_t buf[JTX_MAX];
    uint16_t len;
} jetson_tx_slot_t;
static jetson_tx_slot_t s_txq[JTXQ_DEPTH];
static uint8_t  s_tx_head = 0;   /* 写指针 */
static uint8_t  s_tx_tail = 0;   /* 读/发送指针 */
static uint8_t  s_tx_busy = 0;   /* DMA 发送中 */

/* --------------------- 帧解析状态机 ------------------------------------- */
static uint8_t  s_ps = 0;                 /* 0=找'@' 1=收正文 */
static char     s_fbuf[JFRAME_MAX];
static int      s_flen = 0;
static int      s_foverflow = 0;

static void jparser_byte(uint8_t c);               /* 前置声明(中断内逐字节解析) */
static void jframe_handle(char *fbuf, int flen);   /* 前置声明 */

/* ==========================================================================
 * 小工具
 * ======================================================================== */
static uint32_t jnow_ms(void)
{
    return HAL_GetTick();   /* 系统 tick (本工程 TIM7 累加), 供超时判断 */
}

static void jstr(char *dst, uint32_t size, const char *src)
{
    if (size == 0u)
    {
        return;
    }
    if (src == 0)
    {
        dst[0] = 0;
        return;
    }
    snprintf(dst, size, "%s", src);
}

static void jev(uint16_t bit)
{
    s_ev |= bit;
    jetson.events = s_ev;
}

/* 视觉协议 scene 文本 -> auto_task 区枚举; 未知返回 0xFF */
static uint8_t jscene_of(const char *s)
{
    if (s == 0)
    {
        return 0xFFu;
    }
    if (strcmp(s, "TURNTABLE") == 0) { return AT_SCENE_TURNTABLE; }
    if (strcmp(s, "ROUGH") == 0)     { return AT_SCENE_ROUGH; }
    if (strcmp(s, "STORAGE") == 0)   { return AT_SCENE_STORAGE; }
    return 0xFFu;
}

/* ==========================================================================
 * CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF, 无反射, XorOut 0)
 * 只对 "NAME,FIELD1,..." 正文 ASCII 计算
 * ======================================================================== */
static uint16_t jcrc(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    int i;
    while (len-- > 0)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x8000u)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static int jhexval(char c)
{
    if (c >= '0' && c <= '9')
    {
        return (int)(c - '0');
    }
    if (c >= 'A' && c <= 'F')
    {
        return (int)(c - 'A' + 10);
    }
    if (c >= 'a' && c <= 'f')
    {
        return (int)(c - 'a' + 10);
    }
    return -1;
}

static uint16_t jparse_hex4(const char *p)
{
    uint16_t v = 0u;
    int i;
    for (i = 0; i < 4; i++)
    {
        int h = jhexval(p[i]);
        if (h < 0)
        {
            return 0xFFFFu;   /* 非法, 必然 CRC 不匹配 */
        }
        v = (uint16_t)((v << 4) | (uint16_t)h);
    }
    return v;
}

/* ==========================================================================
 * 发送
 * ======================================================================== */
static void jtx_start_next(void);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)   /* USART1 = Jetson 发送口 (DMA2_Stream7 完成回调) */
    {
        s_tx_busy = 0;
        jtx_start_next();
    }
}

static void jtx_start_next(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if ((s_tx_busy == 0u) && (s_tx_tail != s_tx_head))
    {
        s_tx_busy = 1;
        jetson_tx_slot_t *s = &s_txq[s_tx_tail];
        if (HAL_UART_Transmit_DMA(&huart1, s->buf, s->len) != HAL_OK)
        {
            /* DMA 忙/错误: 标记未发, 交给下次 TxCplt 或超时轮询重发 */
            s_tx_busy = 0;
        }
        else
        {
            s_tx_tail = (uint8_t)((s_tx_tail + 1u) % JTXQ_DEPTH);
            jetson.tx_ok++;
        }
    }
    __set_PRIMASK(pm);
}

static void jtx_body(const char *body)
{
    char line[JTX_MAX];
    int n;
    uint16_t crc;

    if (body == 0 || body[0] == 0)
    {
        return;
    }
    crc = jcrc((const uint8_t *)body, (int)strlen(body));
    n = snprintf(line, sizeof(line), "@%s*%04X\n", body, (unsigned)crc);
    if (n <= 0 || n >= (int)sizeof(line))
    {
        return;
    }

    __disable_irq();
    {
        uint8_t next = (uint8_t)((s_tx_head + 1u) % JTXQ_DEPTH);
        if (next == s_tx_tail || s_tx_busy == 0xFFu)
        {
            jetson.tx_drop++;          /* 队列满 */
        }
        else
        {
            jetson_tx_slot_t *s = &s_txq[s_tx_head];
            memcpy(s->buf, line, (uint32_t)n);
            s->len = (uint16_t)n;
            s_tx_head = next;
        }
    }
    __enable_irq();

    /* 空闲则立即启动发送 */
    if (s_tx_busy == 0u)
    {
        jtx_start_next();
    }
}

/* ==========================================================================
 * 接收: USART1 中断里直接解析 (不另开任务)
 * ======================================================================== */
static void jfeed_bytes(const uint8_t *p, uint16_t n)
{
    while (n-- > 0u)
    {
        jparser_byte(*p++);
    }
}

/* 直接接管 USART1 中断: DMA circular 空闲中断 -> 就地逐字节解析 + 超载清除 */
void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    if ((sr & UART_FLAG_IDLE) != 0u)
    {
        uint16_t head;
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        head = (uint16_t)(JRX_BUF_SIZE - (uint16_t)hdma_usart1_rx.Instance->NDTR);
        if (head != (uint16_t)s_rx_last)
        {
            if (head > (uint16_t)s_rx_last)
            {
                jfeed_bytes(&s_rxbuf[s_rx_last], (uint16_t)(head - (uint16_t)s_rx_last));
            }
            else
            {
                jfeed_bytes(&s_rxbuf[s_rx_last], (uint16_t)(JRX_BUF_SIZE - (uint16_t)s_rx_last));
                jfeed_bytes(&s_rxbuf[0], head);
            }
            s_rx_last = head;
        }
    }
    if ((sr & UART_FLAG_ORE) != 0u)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
}

/* ==========================================================================
 * 帧解析
 * ======================================================================== */
static void jparser_byte(uint8_t c)
{
    if (s_ps == 0u)
    {
        if (c == '@')
        {
            s_ps = 1;
            s_flen = 0;
            s_foverflow = 0;
        }
        return;
    }

    if (c == '\n')
    {
        if (s_foverflow != 0)
        {
            jetson.fmt_err++;
        }
        else if (s_flen > 0)
        {
            jframe_handle(s_fbuf, s_flen);
        }
        s_ps = 0;
        return;
    }
    if (c == '\r')
    {
        return;
    }
    if (c == '@')
    {
        /* 帧内出现 '@'(异常): 从新帧开始 */
        s_flen = 0;
        s_foverflow = 0;
        return;
    }
    if (s_flen >= (int)sizeof(s_fbuf))
    {
        s_foverflow = 1;   /* 超长: 继续吞到 '\n' 丢弃整帧 */
        return;
    }
    s_fbuf[s_flen++] = (char)c;
}

static void jset_phase(uint8_t ph)
{
    if (jetson.phase != ph)
    {
        jetson.phase = ph;
        s_wait_t0 = jnow_ms();
    }
}

static uint32_t jphase_timeout(uint8_t ph)
{
    switch (ph)
    {
        case JETSON_PH_START_WAIT:      return JTO_START_MS;
        case JETSON_PH_TASK_WAIT:       return JTO_TASK_MS;
        case JETSON_PH_REQ_WAIT:        return JTO_REQ_MS;
        case JETSON_PH_PICK_WAIT_GRASP: return JTO_GRASP_MS;
        case JETSON_PH_PICK_WAIT_EXEC:  return JTO_EXEC_MS;
        case JETSON_PH_PICK_WAIT_ACK:   return JTO_ACK_MS;
        case JETSON_PH_ALIGN_WAIT:      return JTO_ALIGN_MS;
        case JETSON_PH_DONE_WAIT:       return JTO_DONE_MS;
        default:                        return 0u;
    }
}

/* 解析 TASK_PLAN 的 "AAA+BBB+CCC+DDD" 并写入 task_code 全局 */
static void jtask_plan_store(const char *taskstr, const char **c, int cn)
{
    uint8_t digit[12];
    int dc = 0;
    int i;
    const char *p;

    jstr(jetson.task_code, sizeof(jetson.task_code), taskstr);

    for (i = 0; i < 6; i++)
    {
        jetson.colors[i] = 0u;
    }
    for (i = 0; i < cn && i < 6; i++)
    {
        jetson.colors[i] = (uint8_t)atoi(c[i]);
    }

    for (p = taskstr; *p != 0 && dc < 12; p++)
    {
        if (*p >= '0' && *p <= '9')
        {
            digit[dc++] = (uint8_t)(*p - '0');
        }
    }
    if (dc == 12)
    {
        task_code_parse(digit);   /* 与 tasks/task_param.h 一致: 组1..4 各3位 */
    }
    jetson.task_plan_ok = 1;
    auto_plan_build();           /* 收到任务码后立即生成 24 步整轮计划(供执行层用) */
}

static void jframe_handle(char *fbuf, int flen)
{
    const char *fields[16];
    int fcnt = 0;
    int i;
    int star = -1;
    uint16_t crc;
    uint16_t rx;

    /* 定位最后一个 '*' */
    for (i = flen - 1; i >= 0; i--)
    {
        if (fbuf[i] == '*')
        {
            star = i;
            break;
        }
    }
    if (star <= 0 || (flen - star - 1) != 4)
    {
        jetson.fmt_err++;
        return;
    }
    crc = jcrc((const uint8_t *)fbuf, star);
    rx = jparse_hex4(fbuf + star + 1);
    if (crc != rx)
    {
        jetson.crc_err++;
        return;
    }

    /* 拆字段: 用 0 替换正文内的 ',' , '*' 也置 0 */
    fbuf[star] = '\0';
    fields[fcnt++] = fbuf;
    for (i = 0; i < star; i++)
    {
        if (fbuf[i] == ',')
        {
            fbuf[i] = '\0';
            if (fcnt < 16)
            {
                fields[fcnt++] = fbuf + i + 1;
            }
        }
    }

    jetson.rx_ok++;
    jstr(jetson.last_frame, sizeof(jetson.last_frame), fields[0]);

    /* ---------------- 按帧名分发 ---------------- */
    if (strcmp(fields[0], "READY") == 0)
    {
        if (fcnt >= 2)
        {
            jstr(jetson.run_id, sizeof(jetson.run_id), fields[1]);
        }
        if (jetson.phase == JETSON_PH_START_WAIT)
        {
            jset_phase(JETSON_PH_TASK_WAIT);
        }
        jev(JETSON_EV_READY);
    }
    else if (strcmp(fields[0], "TASK_PLAN") == 0)
    {
        if (fcnt >= 7)
        {
            jtask_plan_store(fields[1], &fields[2], fcnt - 2);
            jset_phase(JETSON_PH_TASK_READY);
            jev(JETSON_EV_TASK_PLAN);
        }
        else
        {
            jetson.fmt_err++;
        }
    }
    else if (strcmp(fields[0], "ACCEPTED") == 0)
    {
        if (fcnt >= 3 && strcmp(fields[1], jetson.cur_seq) == 0)
        {
            if (strcmp(fields[2], "PICK") == 0)
            {
                jset_phase(JETSON_PH_PICK_WAIT_GRASP);
            }
            else
            {
                jset_phase(JETSON_PH_ALIGN_WAIT);
            }
        }
        else
        {
            jetson.seq_mismatch++;
        }
        jev(JETSON_EV_ACCEPTED);
    }
    else if (strcmp(fields[0], "GRASP_READY") == 0)
    {
        uint8_t okseq;
        if (fcnt >= 11)
        {
            okseq = (uint8_t)(strcmp(fields[1], jetson.cur_seq) == 0);
            jetson.grasp_seq_ok = okseq;
            if (okseq == 0u)
            {
                jetson.seq_mismatch++;
            }
            jstr(jetson.grasp_seq, sizeof(jetson.grasp_seq), fields[1]);
            jetson.grasp_color  = (uint8_t)atoi(fields[2]);
            jetson.grasp_x      = (float)strtof(fields[3], 0);
            jetson.grasp_y      = (float)strtof(fields[4], 0);
            jetson.grasp_unit   = (fields[5][0] == 'M' || fields[5][0] == 'm') ? 'M' : 'P';
            jetson.grasp_px     = (float)strtof(fields[6], 0);
            jetson.grasp_py     = (float)strtof(fields[7], 0);
            jetson.grasp_conf   = (float)strtof(fields[8], 0);
            jetson.grasp_stable = (uint16_t)atoi(fields[9]);
            jetson.grasp_valid_ms = (uint16_t)atoi(fields[10]);
            jetson.grasp_ok     = 1;

            /* 空间转换: 图像偏移 -> (机械臂前向, 车侧向) 纠偏量(mm) */
            auto_vis_ground((jetson.grasp_unit == 'M') ? 1u : 0u,
                            jetson.grasp_x, jetson.grasp_y,
                            &jetson.grasp_tar_x, &jetson.grasp_tar_y);

            if (okseq != 0u && jetson.phase == JETSON_PH_PICK_WAIT_GRASP)
            {
                jset_phase(JETSON_PH_PICK_WAIT_EXEC);   /* 等上层 EXEC */
            }
            jev(JETSON_EV_GRASP_READY);
        }
        else
        {
            jetson.fmt_err++;
        }
    }
    else if (strcmp(fields[0], "GRASP_REVOKED") == 0)
    {
        /* 格式: GRASP_REVOKED,<seq>,<reason>  => 至少 3 个 token */
        if (fcnt >= 3)
        {
            jstr(jetson.err_reason, sizeof(jetson.err_reason), fields[2]);
        }
        jetson.grasp_ok = 0;
        if (jetson.phase == JETSON_PH_PICK_WAIT_EXEC || jetson.phase == JETSON_PH_PICK_WAIT_ACK)
        {
            jset_phase(JETSON_PH_PICK_WAIT_GRASP);   /* 等待新许可, 不重发 REQ */
        }
        jev(JETSON_EV_GRASP_REVOKED);
    }
    else if (strcmp(fields[0], "EXEC_ACK") == 0)
    {
        if (fcnt >= 2 && strcmp(fields[1], jetson.cur_seq) == 0)
        {
            if (jetson.phase == JETSON_PH_PICK_WAIT_ACK)
            {
                jset_phase(JETSON_PH_PICK_ACT);   /* 真正允许机械动作 */
            }
            jev(JETSON_EV_EXEC_ACK);
        }
        else
        {
            jetson.seq_mismatch++;
        }
    }
    else if (strcmp(fields[0], "ALIGN_READY") == 0)
    {
        uint8_t okseq;
        if (fcnt >= 10)
        {
            okseq = (uint8_t)(strcmp(fields[1], jetson.cur_seq) == 0);
            jetson.align_seq_ok = okseq;
            if (okseq == 0u)
            {
                jetson.seq_mismatch++;
            }
            jstr(jetson.align_seq, sizeof(jetson.align_seq), fields[1]);
            jstr(jetson.align_kind, sizeof(jetson.align_kind), fields[2]);
            jetson.align_target = (int16_t)atoi(fields[3]);
            jetson.align_x      = (float)strtof(fields[4], 0);
            jetson.align_y      = (float)strtof(fields[5], 0);
            jetson.align_yaw    = (float)strtof(fields[6], 0);
            jetson.align_unit   = (fields[7][0] == 'M' || fields[7][0] == 'm') ? 'M' : 'P';
            jetson.align_conf   = (float)strtof(fields[8], 0);
            jetson.align_stable = (uint16_t)atoi(fields[9]);
            jetson.align_ok     = 1;

            /* 空间转换: 图像偏移 -> (机械臂前向, 车侧向) 纠偏量(mm) */
            auto_vis_ground((jetson.align_unit == 'M') ? 1u : 0u,
                            jetson.align_x, jetson.align_y,
                            &jetson.align_tar_x, &jetson.align_tar_y);

            if (okseq != 0u && jetson.phase == JETSON_PH_ALIGN_WAIT)
            {
                jset_phase(JETSON_PH_ALIGN_ACT);   /* 等上层对准并 DONE */
            }
            jev(JETSON_EV_ALIGN_READY);
        }
        else
        {
            jetson.fmt_err++;
        }
    }
    else if (strcmp(fields[0], "DONE_ACK") == 0)
    {
        if (fcnt >= 3)
        {
            jetson.done_result_ok = (uint8_t)(strcmp(fields[2], "OK") == 0);
            jstr(jetson.done_seq, sizeof(jetson.done_seq), fields[1]);
            jetson.done_ack_ok = 1;

            if (strcmp(fields[1], jetson.cur_seq) == 0)
            {
                if (strcmp(fields[2], "OK") == 0)
                {
                    jset_phase(JETSON_PH_TASK_READY);          /* 可发下一个 REQ */
                }
                else
                {
                    /* RETRY: Jetson 自动重新识别, 等新的许可帧 */
                    if (strcmp(jetson.cur_kind, "PICK") == 0)
                    {
                        jset_phase(JETSON_PH_PICK_WAIT_GRASP);
                    }
                    else
                    {
                        jset_phase(JETSON_PH_ALIGN_WAIT);
                    }
                }
                jev(JETSON_EV_DONE_ACK);
            }
            else
            {
                jetson.seq_mismatch++;
            }
        }
        else
        {
            jetson.fmt_err++;
        }
    }
    else if (strcmp(fields[0], "ABORTED") == 0)
    {
        if (fcnt >= 2)
        {
            jstr(jetson.run_id, sizeof(jetson.run_id), fields[1]);
        }
        jetson.cur_seq[0] = 0;
        jetson.cur_kind[0] = 0;
        jetson.grasp_ok = 0;
        jetson.align_ok = 0;
        jetson.aborted = 1;
        jset_phase(JETSON_PH_IDLE);
        jev(JETSON_EV_ABORTED);
    }
    else if (strcmp(fields[0], "ERROR") == 0)
    {
        if (fcnt >= 2)
        {
            jstr(jetson.err_seq, sizeof(jetson.err_seq), fields[1]);
        }
        if (fcnt >= 3)
        {
            jstr(jetson.err_reason, sizeof(jetson.err_reason), fields[2]);
        }
        jset_phase(JETSON_PH_ERROR);
        jev(JETSON_EV_ERROR);
    }
    else
    {
        /* 未知帧: 按协议静默丢弃 */
    }
}

/* ==========================================================================
 * 对外 API (发送)
 * ======================================================================== */
void jetson_send_start(const char *run_id)
{
    char body[64];
    snprintf(body, sizeof(body), "START,%s", (run_id && run_id[0]) ? run_id : "RUN001");
    jstr(jetson.run_id, sizeof(jetson.run_id), body + 6);
    jset_phase(JETSON_PH_START_WAIT);
    jtx_body(body);
}

void jetson_send_abort(const char *run_id)
{
    char body[64];
    snprintf(body, sizeof(body), "ABORT,%s", (run_id && run_id[0]) ? run_id : "RUN001");
    jetson.cur_seq[0] = 0;
    jetson.cur_kind[0] = 0;
    jetson.grasp_ok = 0;
    jetson.align_ok = 0;
    jset_phase(JETSON_PH_IDLE);
    jtx_body(body);
}

static uint8_t jreq(const char *kind, const char *seq, const char *scene, const char *id1, const char *id2)
{
    char body[160];
    char seqbuf[24];
    const char *sc = scene;

    if (jetson.phase != JETSON_PH_TASK_READY)
    {
        return 0;
    }
    if (seq == 0 || seq[0] == 0)
    {
        return 0;
    }
    if (sc == 0)
    {
        if (strcmp(kind, "PICK") == 0)
        {
            sc = "TURNTABLE";
        }
        else if (strcmp(kind, "PLACE") == 0)
        {
            sc = "ROUGH";
        }
        else
        {
            sc = "STORAGE";
        }
    }

    if (strcmp(kind, "PICK") == 0)
    {
        if (id1 != 0)
        {
            snprintf(body, sizeof(body), "REQ,%s,PICK,%s,%s", seq, sc, id1);
        }
        else
        {
            snprintf(body, sizeof(body), "REQ,%s,PICK,%s", seq, sc);
        }
    }
    else if (strcmp(kind, "PLACE") == 0)
    {
        if (id1 == 0)
        {
            return 0;                       /* PLACE 必须给环号 */
        }
        snprintf(body, sizeof(body), "REQ,%s,PLACE,%s,%s", seq, sc, id1);
    }
    else
    {
        /* STACK: 颜色必填, 环号可选 */
        if (id1 == 0)
        {
            return 0;
        }
        if (id2 != 0)
        {
            snprintf(body, sizeof(body), "REQ,%s,STACK,%s,%s,%s", seq, sc, id1, id2);
        }
        else
        {
            snprintf(body, sizeof(body), "REQ,%s,STACK,%s,%s", seq, sc, id1);
        }
    }

    jstr(seqbuf, sizeof(seqbuf), seq);
    jstr(jetson.cur_seq, sizeof(jetson.cur_seq), seqbuf);
    jstr(jetson.cur_kind, sizeof(jetson.cur_kind), kind);
    {   /* 记录所在区, 供收到许可帧后做“区基准+视觉偏移”合成 */
        uint8_t si = jscene_of(sc);
        jetson.scene_idx = (si != 0xFFu) ? si : AT_SCENE_TURNTABLE;
    }
    jset_phase(JETSON_PH_REQ_WAIT);
    jtx_body(body);
    return 1;
}

uint8_t jetson_send_req_pick(const char *seq, const char *scene, const char *color)
{
    return jreq("PICK", seq, scene, color, 0);
}

uint8_t jetson_send_req_place(const char *seq, const char *scene, const char *ring)
{
    return jreq("PLACE", seq, scene, ring, 0);
}

uint8_t jetson_send_req_stack(const char *seq, const char *scene, const char *color, const char *ring)
{
    return jreq("STACK", seq, scene, color, ring);
}

uint8_t jetson_send_exec(const char *seq)
{
    char body[64];
    if (jetson.phase != JETSON_PH_PICK_WAIT_EXEC)
    {
        return 0;
    }
    if (seq == 0 || strcmp(seq, jetson.cur_seq) != 0)
    {
        jetson.seq_mismatch++;
        return 0;
    }
    snprintf(body, sizeof(body), "EXEC,%s", seq);
    jset_phase(JETSON_PH_PICK_WAIT_ACK);
    jtx_body(body);
    return 1;
}

uint8_t jetson_send_done(const char *seq, uint8_t ok)
{
    char body[64];
    if (jetson.phase != JETSON_PH_PICK_ACT && jetson.phase != JETSON_PH_ALIGN_ACT)
    {
        return 0;
    }
    if (seq == 0 || strcmp(seq, jetson.cur_seq) != 0)
    {
        jetson.seq_mismatch++;
        return 0;
    }
    snprintf(body, sizeof(body), "DONE,%s,%s", seq, ok ? "OK" : "FAIL");
    jset_phase(JETSON_PH_DONE_WAIT);
    jtx_body(body);
    return 1;
}

uint8_t jetson_phase(void)
{
    return jetson.phase;
}

uint16_t jetson_ev_take(void)
{
    uint16_t e;
    __disable_irq();
    e = s_ev;
    s_ev = 0;
    jetson.events = 0;
    __enable_irq();
    return e;
}

/* ==========================================================================
 * 调试命令处理 (在 jetson_service 中调用)
 * ======================================================================== */
static void jcmd_handle(void)
{
    if (jetson_cmd_start != 0u)
    {
        jetson_cmd_start = 0;
        jetson_send_start(jetson_cmd_run_id);
    }
    if (jetson_cmd_abort != 0u)
    {
        jetson_cmd_abort = 0;
        jetson_send_abort(jetson_cmd_run_id);
    }
    if (jetson_cmd_demo_pick != 0u)
    {
        jetson_cmd_demo_pick = 0;
        if (jetson.phase == JETSON_PH_TASK_READY)
        {
            char seq[24];
            char col[4];
            uint8_t c = jetson.task_plan_ok ? jetson.colors[0] : 1u;
            s_seq_no++;
            snprintf(seq, sizeof(seq), "DEMO_P%lu", (unsigned long)s_seq_no);
            snprintf(col, sizeof(col), "%u", (unsigned)c);
            jetson_send_req_pick(seq, "TURNTABLE", col);
        }
    }
    if (jetson_cmd_demo_place != 0u)
    {
        jetson_cmd_demo_place = 0;
        if (jetson.phase == JETSON_PH_TASK_READY)
        {
            char seq[24];
            s_seq_no++;
            snprintf(seq, sizeof(seq), "DEMO_L%lu", (unsigned long)s_seq_no);
            jetson_send_req_place(seq, "ROUGH", "1");
        }
    }
    if (jetson_cmd_demo_stack != 0u)
    {
        jetson_cmd_demo_stack = 0;
        if (jetson.phase == JETSON_PH_TASK_READY)
        {
            char seq[24];
            char col[4];
            uint8_t c = jetson.task_plan_ok ? jetson.colors[0] : 1u;
            s_seq_no++;
            snprintf(seq, sizeof(seq), "DEMO_S%lu", (unsigned long)s_seq_no);
            snprintf(col, sizeof(col), "%u", (unsigned)c);
            jetson_send_req_stack(seq, "STORAGE", col, "1");
        }
    }
    if (jetson_cmd_exec != 0u)
    {
        jetson_cmd_exec = 0;
        jetson_send_exec(jetson.cur_seq);
    }
    if (jetson_cmd_done != 0u)
    {
        uint8_t ok = (jetson_cmd_done == 1u) ? 1u : 0u;
        jetson_cmd_done = 0;
        jetson_send_done(jetson.cur_seq, ok);
    }
}

/* ==========================================================================
 * 初始化 / 周期服务
 * ======================================================================== */
void jetson_link_init(void)
{
    /* 1. 把 USART1 RX DMA(DMA2_Stream5) 重配为 circular 后启动接收 */
    HAL_DMA_DeInit(&hdma_usart1_rx);
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        return;
    }
    if (HAL_UART_Receive_DMA(&huart1, s_rxbuf, JRX_BUF_SIZE) != HAL_OK)
    {
        return;
    }
    /* 2. 打开 UART 空闲中断(收帧结束判定) */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    /* 3. 复位解析/计数状态 */
    s_ps = 0;
    s_flen = 0;
    s_foverflow = 0;
    s_rx_last = 0;
    s_tx_head = 0;
    s_tx_tail = 0;
    s_tx_busy = 0;
    s_ev = 0;
    jetson.phase = JETSON_PH_IDLE;
    jetson.events = 0;
    jetson.grasp_ok = 0;
    jetson.align_ok = 0;
    jetson.task_plan_ok = 0;
    jetson.done_ack_ok = 0;
    jetson.aborted = 1;
}

void jetson_service(void)
{
    /* 1. 处理调试命令(发送 START/ABORT/REQ/EXEC/DONE) */
    jcmd_handle();

    /* 2. 本地等待超时保护 (帧解析已在中断完成; 这里只做超时兜底) */
    {
        uint32_t to = jphase_timeout(jetson.phase);
        if (to != 0u)
        {
            if ((jnow_ms() - s_wait_t0) >= to)
            {
                jetson.cur_seq[0] = 0;
                jetson.cur_kind[0] = 0;
                jetson.grasp_ok = 0;
                jetson.align_ok = 0;
                jstr(jetson.err_reason, sizeof(jetson.err_reason), "LOCAL_TIMEOUT");
                jset_phase(JETSON_PH_ERROR);
                jev(JETSON_EV_TIMEOUT);
            }
        }
    }
}
