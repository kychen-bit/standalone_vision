/**
  ******************************************************************************
  * @file       lcd_info.h
  * @brief      3.2" LCD 运行信息显示: 任务码 / 自动步骤 / 会话阶段 / 视觉合成目标
  *
  *  命题要求搬运机器人必须配备“任务码显示装置”(亮光、不被遮挡)。
  *  本模块用板载 SPI2 的 3.2 寸 LCD(ST7789)显示运行状态:
  *    init(): 初始化屏幕+标题 (由 StartDefaultTask 调一次)
  *    refresh(): 刷新状态行 (建议 ~200ms 调一次, 放在任意周期代码)
  *  若本车未接 LCD, 把 freertos.c 里的调用注释即可(不影响其它功能)。
  ******************************************************************************
  */
#ifndef LCD_INFO_H
#define LCD_INFO_H

extern void lcd_info_init(void);
extern void lcd_info_refresh(void);

#endif /* LCD_INFO_H */
