/**
  ******************************************************************************
  * @file       task_param.h
  * @brief      任务参数 / 任务编码 (工训赛 抓取搬运任务)
  *             上位机(或裁判/视觉)下发一段 12 位任务编码, 共 4 组 3 位数:
  *               第1组 三位数: 第一批 3 个物料的“颜色 + 搬运顺序”
  *                   每一位数字 = 该物料颜色(见下方颜色表)
  *                   位序(第1/2/3位) = 搬运顺序(第1位先搬 … 第3位最后搬)
  *               第2组 三位数: 第一批物料在“粗加工区”和“暂存区”的放置位置
  *                   每一位数字 = 对应第1组顺序物料的放置位置编号
  *               第3组 三位数: 第二批 3 个物料的“颜色 + 搬运顺序”(含义同第1组)
  *               第4组 三位数: 第二批物料在“粗加工区”的放置位置
  *                   每一位数字 = 对应第3组顺序物料的粗加工区放置位置编号
  *
  *             颜色编码: 红1  黄2  蓝3  绿4  黑5  浅蓝6
  ******************************************************************************
  */
#ifndef TASK_PARAM_H
#define TASK_PARAM_H

#include <stdint.h>

/*============================================================================
 * 颜色编码表 (红1 黄2 蓝3 绿4 黑5 浅蓝6)
 *==========================================================================*/
enum
{
    TASK_COLOR_RED       = 1,   //红
    TASK_COLOR_YELLOW    = 2,   //黄
    TASK_COLOR_BLUE      = 3,   //蓝
    TASK_COLOR_GREEN     = 4,   //绿
    TASK_COLOR_BLACK     = 5,   //黑
    TASK_COLOR_LIGHTBLUE = 6,   //浅蓝
};

//任务编码组成: 4 组, 每组 3 位; 共 2 批, 每批 3 个物料
#define TASK_GROUP_NUM      4
#define TASK_GROUP_LEN      3
#define TASK_BATCH_NUM      2
#define TASK_BATCH_OBJ_NUM  3

/**
  * @brief          任务编码结构体: 四组三位数用四个数组存储
  * @note          下标 0/1/2 = 该批物料搬运顺序 第1/第2/第3
  */
typedef struct
{
    /* 第1组: 第一批 3 个物料的颜色+搬运顺序 (每元素=颜色 1..6) */
    uint8_t batch1_color[TASK_BATCH_OBJ_NUM];

    /* 第2组: 第一批 3 个物料在粗加工区/暂存区的放置位置 (每元素=位置编号) */
    uint8_t batch1_place[TASK_BATCH_OBJ_NUM];

    /* 第3组: 第二批 3 个物料的颜色+搬运顺序 (每元素=颜色 1..6) */
    uint8_t batch2_color[TASK_BATCH_OBJ_NUM];

    /* 第4组: 第二批 3 个物料在粗加工区的放置位置 (每元素=位置编号) */
    uint8_t batch2_place[TASK_BATCH_OBJ_NUM];
} task_code_t;

//全局任务编码 (默认全 0 = 未收到有效任务; 收到后调用 task_code_parse 填充)
extern task_code_t task_code;

/**
  * @brief          颜色编号 -> 名称字符串 (用于调试/打印)
  * @param[in]      color: 1=红 2=黄 3=蓝 4=绿 5=黑 6=浅蓝
  * @retval         "RED"/"YELLOW"/... 越界返回 "?"
  */
extern const char *task_color_name(uint8_t color);

/**
  * @brief          从 12 位数字解析任务编码
  * @param[in]      digit[12]: 依次为 组1(3位)组2(3位)组3(3位)组4(3位), 每位 0~9 的数值
  * @note           颜色组(组1/组3)每位必须 1..6; 位置组(组2/组4)仅按位原样存入
  * @retval         0=成功; 1=颜色越界/非法
  */
extern uint8_t task_code_parse(const uint8_t digit[12]);

#endif /* TASK_PARAM_H */
