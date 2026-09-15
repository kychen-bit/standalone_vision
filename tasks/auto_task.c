/**
  ******************************************************************************
  * @file       auto_task.c
  * @brief      智能搬运全自动任务规划 (场地模型 + 24 步动作序列) 实现
  *
  *  说明: 本文件只做“规划”, 不执行任何底盘/机械臂/舵机动作;
  *        执行层(导航到位 / Jetson 握手 / 机械臂取放)尚未接线。
  ******************************************************************************
  */

#include "auto_task.h"
#include "task_param.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ==================== 全局: 整轮计划 ==================================== */
auto_step_t auto_plan[AT_PLAN_MAX];
uint8_t     auto_plan_n = 0;
uint8_t     auto_plan_ready = 0;

/* 视觉 PX -> mm 系数(占位 0.5, 现场标定后改/或 Watch 调) */
float auto_vis_px2mm = 0.5f;
/* Jetson 未做平面标定时返回整图绝对像素，因此默认先减画面中心。 */
uint8_t auto_vis_px_rel = 0u;
/* 相机绕光轴安装角(rad, 默认0): 让图像轴与车体轴对齐的标定值 */
float auto_cam_rot = 0.0f;
/* 目标位于此像素时，夹爪处于可抓取参考位置；允许在 Watch 中现场标定。 */
float auto_vis_cx_px = 640.0f;
float auto_vis_cy_px = 360.0f;

/* 底盘总控 -> 机械臂 取放请求(见 auto_task.h) */
auto_grab_req_t auto_grab_req;

const char *auto_scene_str(uint8_t scene)
{
    switch (scene)
    {
        case AT_SCENE_TURNTABLE: return "TURNTABLE";
        case AT_SCENE_ROUGH:     return "ROUGH";
        case AT_SCENE_STORAGE:   return "STORAGE";
        default:                 return "TURNTABLE";
    }
}

/* ==================== 静态工具 ========================================== */
static const char *at_scene_tag(uint8_t scene)
{
    switch (scene)
    {
        case AT_SCENE_TURNTABLE: return "TP";
        case AT_SCENE_ROUGH:     return "RGH";
        case AT_SCENE_STORAGE:   return "STG";
        default:                 return "??";
    }
}

static const char *at_kind_tag(uint8_t kind)
{
    switch (kind)
    {
        case AT_KIND_PICK:  return "PICK";
        case AT_KIND_PLACE: return "PLACE";
        case AT_KIND_STACK: return "STACK";
        default:            return "??";
    }
}

void auto_seq_make(uint8_t batch, uint8_t scene, uint8_t kind, uint8_t idx, char *out)
{
    if (out == 0)
    {
        return;
    }
    snprintf(out, AT_SEQ_MAX, "B%d_%s_%s_%d",
             (int)(batch + 1), at_scene_tag(scene), at_kind_tag(kind), (int)idx);
}

/* 找第 1 批中某颜色的搬运位序下标(用于批2 STACK 的环号映射), 找不到返回 0xFF */
static uint8_t at_find_b1_color(uint8_t color)
{
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        if (task_code.batch1_color[i] == color)
        {
            return i;
        }
    }
    return 0xFFu;
}

/* 向计划末尾追加一步, 带自动 seq */
static uint8_t at_push(uint8_t batch, uint8_t kind, uint8_t scene,
                       uint8_t color, uint8_t ring, uint8_t idx)
{
    if (auto_plan_n >= AT_PLAN_MAX)
    {
        return 0;
    }
    auto_step_t *s = &auto_plan[auto_plan_n];
    s->kind = kind;
    s->scene = scene;
    s->color = color;
    s->ring = ring;
    auto_seq_make(batch, scene, kind, idx, s->seq);
    auto_plan_n++;
    return 1;
}

/* ==================== 生成整轮计划 ====================================== */
uint8_t auto_plan_build(void)
{
    uint8_t b;
    uint8_t i;
    uint8_t idx;

    auto_plan_clear();

    /* 任务码未解析(batch1 颜色为空)则不生成 */
    if (task_code.batch1_color[0] == 0u && task_code.batch2_color[0] == 0u)
    {
        return 0;
    }

    for (b = 0; b < 2; b++)
    {
        const uint8_t *col = (b == 0) ? task_code.batch1_color : task_code.batch2_color;
        const uint8_t *pla = (b == 0) ? task_code.batch1_place : task_code.batch2_place;

        /* 1) 原料区 按顺序抓 3 件 (每次 1 件, 放车上后再下一件) */
        for (i = 0; i < 3; i++)
        {
            at_push(b, AT_KIND_PICK, AT_SCENE_TURNTABLE, col[i], 0, i + 1);
        }

        /* 2) 粗加工区 按顺序放入对应环 */
        for (i = 0; i < 3; i++)
        {
            at_push(b, AT_KIND_PLACE, AT_SCENE_ROUGH, col[i], pla[i], i + 1);
        }

        /* 3) 粗加工区 按顺序取回放上车 */
        for (i = 0; i < 3; i++)
        {
            at_push(b, AT_KIND_PICK, AT_SCENE_ROUGH, col[i], pla[i], i + 1);
        }

        /* 4) 暂存区: 批1 平放(环); 批2 叠垛到批1同色环上 */
        for (i = 0; i < 3; i++)
        {
            if (b == 0)
            {
                at_push(b, AT_KIND_PLACE, AT_SCENE_STORAGE, col[i], pla[i], i + 1);
            }
            else
            {
                idx = at_find_b1_color(col[i]);
                if (idx != 0xFFu)
                {
                    /* 环号 = 批1 该颜色在粗加工/暂存的环位(同色叠在同位) */
                    at_push(b, AT_KIND_STACK, AT_SCENE_STORAGE, col[i],
                            task_code.batch1_place[idx], i + 1);
                }
                else
                {
                    /* 异常: 用自身 i+1 兜底 */
                    at_push(b, AT_KIND_STACK, AT_SCENE_STORAGE, col[i], pla[i], i + 1);
                }
            }
        }
    }

    auto_plan_ready = 1;
    return auto_plan_n;
}

void auto_plan_clear(void)
{
    uint8_t i;
    for (i = 0; i < AT_PLAN_MAX; i++)
    {
        memset(&auto_plan[i], 0, sizeof(auto_step_t));
    }
    auto_plan_n = 0;
    auto_plan_ready = 0;
}

/* ==================== 场地模型 ========================================== */
uint8_t auto_field_center(uint8_t scene, uint8_t ring, float *x, float *y)
{
    if (x == 0 || y == 0)
    {
        return 0;
    }
    switch (scene)
    {
        case AT_SCENE_TURNTABLE:
            *x = AT_TURNTABLE_X;
            *y = AT_TURNTABLE_Y;
            return 1;

        case AT_SCENE_ROUGH:
            *y = AT_ROUGH_Y;
            if (ring == 1) { *x = AT_ROUGH_R1_X; return 1; }
            if (ring == 2) { *x = AT_ROUGH_R2_X; return 1; }
            if (ring == 3) { *x = AT_ROUGH_R3_X; return 1; }
            return 0;

        case AT_SCENE_STORAGE:
            *x = AT_STORAGE_X;
            if (ring == 1) { *y = AT_STORAGE_R1_Y; return 1; }
            if (ring == 2) { *y = AT_STORAGE_R2_Y; return 1; }
            if (ring == 3) { *y = AT_STORAGE_R3_Y; return 1; }
            return 0;

        default:
            return 0;
    }
}

/* ==================== 视觉偏移 -> 机械臂末端偏移 ======================== */
uint8_t auto_vis_offset(uint8_t vis_mm, float vis_x, float vis_y, float *dx, float *dy)
{
    if (dx == 0 || dy == 0)
    {
        return 0;
    }
    if (vis_mm != 0u)
    {
        /* 已是毫米相对偏移 */
        *dx = vis_x;
        *dy = vis_y;
        return 1;
    }
    /* 像素: 相机在夹爪末端, 相对坐标直接×系数; 整图绝对则先减画面中心 */
    if (auto_vis_px_rel != 0u)
    {
        *dx = vis_x * auto_vis_px2mm;
        *dy = vis_y * auto_vis_px2mm;
    }
    else
    {
        *dx = (vis_x - auto_vis_cx_px) * auto_vis_px2mm;
        *dy = (vis_y - auto_vis_cy_px) * auto_vis_px2mm;
    }
    return 1;
}

/* 空间转换: 相机相对偏移 -> (机械臂前向 fx, 车侧向 sd) */
uint8_t auto_vis_ground(uint8_t vis_mm, float vis_x, float vis_y, float *fx, float *sd)
{
    float u = 0.0f, v = 0.0f;
    float ca, sa, ur, vr;

    if (fx == 0 || sd == 0)
    {
        return 0;
    }
    /* 图像坐标(相对) -> mm(u: 图像右+, v: 图像下+) */
    if (vis_mm != 0u)
    {
        u = vis_x;
        v = vis_y;
    }
    else if (auto_vis_px_rel != 0u)
    {
        u = vis_x * auto_vis_px2mm;
        v = vis_y * auto_vis_px2mm;
    }
    else
    {
        u = (vis_x - auto_vis_cx_px) * auto_vis_px2mm;
        v = (vis_y - auto_vis_cy_px) * auto_vis_px2mm;
    }

    /* 绕光轴旋转 auto_cam_rot, 使图像轴与车轴对齐 */
    ca = cosf(auto_cam_rot);
    sa = sinf(auto_cam_rot);
    ur = u * ca - v * sa;
    vr = u * sa + v * ca;

    /* 模型(相机竖直朝下装末端看台面, 0=中心即已对准):
     *   +v(图像下) 近似车后方 => 前向 fx 取 -vr
     *   +u(图像右) 近似车右  => 车左为正的侧向 sd 取 -ur
     * 两个正负号若实机反了, 改这里(或标定 auto_cam_rot=±π 之一)。
     * 目标在画面中心(x=y=0) => fx=sd=0, 不需再动。 */
    *fx = -vr;
    *sd = -ur;
    return 1;
}

/* 世界调试: 区基准(v1) + 视觉偏移, 仅观察用 */
uint8_t auto_vis_target(uint8_t scene, uint8_t ring, uint8_t vis_mm,
                        float vis_x, float vis_y, float *tx, float *ty)
{
    float bx = 0.0f, by = 0.0f, dx = 0.0f, dy = 0.0f;
    uint8_t r = ring;

    if (tx == 0 || ty == 0)
    {
        return 0;
    }
    if (r < 1u || r > 3u)
    {
        r = 1u;
    }
    if (auto_field_center(scene, r, &bx, &by) == 0u)
    {
        return 0;
    }
    if (auto_vis_offset(vis_mm, vis_x, vis_y, &dx, &dy) == 0u)
    {
        return 0;
    }
    *tx = bx + dx;
    *ty = by + dy;
    return 1;
}
