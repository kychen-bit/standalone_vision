/**
  ******************************************************************************
  * @file       lcd_info.c
  * @brief      3.2" LCD 运行信息显示实现 (见 lcd_info.h)
  ******************************************************************************
  */

#include "lcd_info.h"
#include "lcd.h"
#include "jetson_task.h"
#include "auto_task.h"
#include <stdio.h>

/* chassis 自动状态调试变量(定义于 chassis_task.c) */
extern uint8_t  chassis_auto_dbg_state;
extern int16_t  chassis_auto_dbg_step;
extern uint8_t  chassis_auto_dbg_scene;
extern uint8_t  chassis_auto_dbg_kind;
extern uint8_t  chassis_auto_dbg_ring;
extern float    chassis_auto_dbg_dist_mm;

/* 颜色(16bit RGB565) */
#define C_BLACK  0x0000
#define C_WHITE  0xFFFF
#define C_RED    0xF800
#define C_GREEN  0x07E0
#define C_YELLOW 0xFFE0
#define C_CYAN   0x07FF
#define C_GRAY   0x8410

#define LCD_ROW_H     18u   /* 每行高 */
#define LCD_ROW_NUM   16u
#define LCD_TOP       4u

static const char *lcd_phase_name(uint8_t ph)
{
    switch (ph)
    {
        case JETSON_PH_IDLE:           return "IDLE";
        case JETSON_PH_START_WAIT:     return "START";
        case JETSON_PH_TASK_WAIT:      return "TASKW";
        case JETSON_PH_TASK_READY:     return "READY";
        case JETSON_PH_REQ_WAIT:       return "REQ";
        case JETSON_PH_PICK_WAIT_GRASP:return "PGRASP";
        case JETSON_PH_PICK_WAIT_EXEC: return "PEXEC";
        case JETSON_PH_PICK_WAIT_ACK:  return "PACK";
        case JETSON_PH_PICK_ACT:       return "PACT";
        case JETSON_PH_ALIGN_WAIT:     return "AWAIT";
        case JETSON_PH_ALIGN_ACT:      return "AACT";
        case JETSON_PH_DONE_WAIT:      return "DONE";
        case JETSON_PH_ERROR:          return "ERROR";
        default:                       return "?";
    }
}

static const char *lcd_scene_name(uint8_t sc)
{
    if (sc == AT_SCENE_TURNTABLE) { return "TP"; }
    if (sc == AT_SCENE_ROUGH)     { return "RGH"; }
    if (sc == AT_SCENE_STORAGE)   { return "STG"; }
    return "??";
}

/* 画一行: 先清黑再写字 */
static void lcd_row(uint16_t idx, const char *s, uint16_t color)
{
    uint16_t y0 = LCD_TOP + (uint16_t)idx * LCD_ROW_H;
    LCD_Fill(0, y0, LCD_W - 1, y0 + LCD_ROW_H - 1, C_BLACK);
    LCD_ShowString(0, y0, (const u8 *)s, color, C_BLACK, 16, 0);
}

void lcd_info_init(void)
{
    LCD_Init_320();
    LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, C_BLACK);
    lcd_row(0, "GONGXUN  TRANSPORT", C_YELLOW);
    lcd_row(1, "CODE WAIT QR", C_GREEN);
}

void lcd_info_refresh(void)
{
    char buf[48];
    uint16_t r = 0;

    /* 行0: 任务码 */
    if (jetson.task_plan_ok != 0u)
    {
        snprintf(buf, sizeof(buf), "CODE %s", jetson.task_code[0] ? jetson.task_code : "----");
        lcd_row(r++, buf, C_GREEN);
    }
    else
    {
        lcd_row(r++, "CODE WAIT QR", C_GREEN);
    }

    /* 行1: 自动步骤 */
    if (auto_plan_ready != 0u)
    {
        if (chassis_auto_dbg_step >= 0)
        {
            snprintf(buf, sizeof(buf), "STP %2d/%2u %s %s r%d",
                     (int)chassis_auto_dbg_step + 1, (unsigned)auto_plan_n,
                     lcd_scene_name(chassis_auto_dbg_scene),
                     (chassis_auto_dbg_kind == AT_KIND_PICK) ? "PICK" :
                     (chassis_auto_dbg_kind == AT_KIND_PLACE) ? "PLA" : "STK",
                     (int)chassis_auto_dbg_ring);
        }
        else if (chassis_auto_dbg_step == -2)
        {
            snprintf(buf, sizeof(buf), "GO HOME (start2)");
        }
        else
        {
            snprintf(buf, sizeof(buf), "PLAN %2u steps", (unsigned)auto_plan_n);
        }
    }
    else
    {
        snprintf(buf, sizeof(buf), "STP waiting task...");
    }
    lcd_row(r++, buf, C_CYAN);

    /* 行2: Jetson 会话阶段 */
    snprintf(buf, sizeof(buf), "PH %-6s ev=%04X", lcd_phase_name(jetson.phase),
             (unsigned)jetson.events);
    lcd_row(r++, buf, C_WHITE);

    /* 行3: 视觉合成目标(GRASP) */
    if (jetson.grasp_ok != 0u)
    {
        snprintf(buf, sizeof(buf), "GRSP c%d x%.0f y%.0f", (int)jetson.grasp_color,
                 jetson.grasp_x, jetson.grasp_y);
        lcd_row(r++, buf, C_YELLOW);
        snprintf(buf, sizeof(buf), "  fx %+.0f sd %+.0f",
                 jetson.grasp_tar_x, jetson.grasp_tar_y);
        lcd_row(r++, buf, C_GRAY);
    }
    else if (jetson.align_ok != 0u)
    {
        snprintf(buf, sizeof(buf), "ALGN r%d x%.0f y%.0f",
                 (int)jetson.align_target, jetson.align_x, jetson.align_y);
        lcd_row(r++, buf, C_YELLOW);
        snprintf(buf, sizeof(buf), "  fx %+.0f sd %+.0f",
                 jetson.align_tar_x, jetson.align_tar_y);
        lcd_row(r++, buf, C_GRAY);
    }
    else
    {
        lcd_row(r++, "NO GRASP/ALIGN", C_GRAY);
        lcd_row(r++, "", C_GRAY);
    }

    /* 行5: 底盘自动导航剩余 */
    if (auto_plan_ready != 0u)
    {
        snprintf(buf, sizeof(buf), "DIST %5.0fmm st=%d",
                 (double)chassis_auto_dbg_dist_mm, (int)chassis_auto_dbg_state);
        lcd_row(r++, buf, C_CYAN);
    }

    /* 行6: 最近帧/错误 */
    if (jetson.err_reason[0] != 0)
    {
        snprintf(buf, sizeof(buf), "ERR %s", jetson.err_reason);
        lcd_row(r++, buf, C_RED);
    }
    snprintf(buf, sizeof(buf), "last %s rx%lu crc%lu",
             jetson.last_frame[0] ? jetson.last_frame : "-",
             (unsigned long)jetson.rx_ok, (unsigned long)jetson.crc_err);
    lcd_row(r++, buf, C_GRAY);
    (void)LCD_ROW_NUM;
}
