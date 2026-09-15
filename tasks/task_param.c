/**
  ******************************************************************************
  * @file       task_param.c
  * @brief      任务参数 / 任务编码 实现 (工训赛 抓取搬运任务)
  *             编码格式见 task_param.h 顶部说明:
  *               第1组 3 位: 第一批物料 颜色+搬运顺序 (红1 黄2 蓝3 绿4 黑5 浅蓝6)
  *               第2组 3 位: 第一批物料 粗加工区/暂存区 放置位置
  *               第3组 3 位: 第二批物料 颜色+搬运顺序
  *               第4组 3 位: 第二批物料 粗加工区 放置位置
  ******************************************************************************
  */
#include "task_param.h"

//全局任务编码: 默认全 0 (未收到有效任务前不使用)
task_code_t task_code =
{
    {0, 0, 0},   //第1组: 第一批颜色+顺序
    {0, 0, 0},   //第2组: 第一批放置位置
    {0, 0, 0},   //第3组: 第二批颜色+顺序
    {0, 0, 0},   //第4组: 第二批放置位置
};

//颜色名称表 (下标0 不用; 1..6 = 红黄蓝绿黑浅蓝)
static const char *const s_task_color_name[7] =
{
    "?",         //0
    "RED",       //1 红
    "YELLOW",    //2 黄
    "BLUE",      //3 蓝
    "GREEN",     //4 绿
    "BLACK",     //5 黑
    "LIGHTBLUE", //6 浅蓝
};

const char *task_color_name(uint8_t color)
{
    if (color < 1 || color > TASK_COLOR_LIGHTBLUE)
    {
        return "?";
    }
    return s_task_color_name[color];
}

uint8_t task_code_parse(const uint8_t digit[12])
{
    uint8_t i, v, idx = 0;

    if (digit == 0)
    {
        return 1;
    }

    //第1组: 第一批 3 个物料颜色+顺序 (1..6)
    for (i = 0; i < TASK_BATCH_OBJ_NUM; i++)
    {
        v = digit[idx++];
        if (v < 1 || v > TASK_COLOR_LIGHTBLUE)
        {
            return 1;
        }
        task_code.batch1_color[i] = v;
    }

    //第2组: 第一批物料 粗加工区/暂存区 放置位置 (原样存入)
    for (i = 0; i < TASK_BATCH_OBJ_NUM; i++)
    {
        task_code.batch1_place[i] = digit[idx++];
    }

    //第3组: 第二批 3 个物料颜色+顺序 (1..6)
    for (i = 0; i < TASK_BATCH_OBJ_NUM; i++)
    {
        v = digit[idx++];
        if (v < 1 || v > TASK_COLOR_LIGHTBLUE)
        {
            return 1;
        }
        task_code.batch2_color[i] = v;
    }

    //第4组: 第二批物料 粗加工区 放置位置 (原样存入)
    for (i = 0; i < TASK_BATCH_OBJ_NUM; i++)
    {
        task_code.batch2_place[i] = digit[idx++];
    }

    return 0;
}
