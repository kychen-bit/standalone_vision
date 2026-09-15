/**
  ******************************************************************************
  * @file       auto_task.h
  * @brief      智能搬运全自动任务规划 (场地模型 + 24 步动作序列)
  *
  *  坐标系(用户定义, v1 系, 单位 mm):
  *    原点(0,0) = 右上角启停区(启停1)中心
  *    +vx 正方向 = 车前进方向(场地向左);  +vy 正方向 = 场地向下
  *   (v1: vx = 2250 - x_V0 ; vy = 2250 - y_V0 , 其中 V0 系原点=场地左下/右为正/上为正)
  *
  *  机械臂(用户确认):
  *    0° = 竖直向上(固定, 即车载放料位方向); 需要够下方/侧方台面时由底盘旋转
  *    (可 180° 掉头) 让机械臂工作平面对准目标区; 机械臂装在车上沿 vx 方向、偏上 10mm
  *    (装车 offset 待实测, 执行层用 AT_ARM_* 宏)
  *
  *  视觉分工: MaixCAM 只读二维码(任务码); Jetson 只做物料/圆环识别并把
  *  TASK_PLAN + GRASP/ALIGN 坐标发给电控(见 jetson_task.h / 对接协议)
  *
  *  模块职责: 本文件只做“场地模型 + 由 task_code 生成 24 步动作表”, 不执行;
  *            执行层(底盘到位 / jetson 握手 / 机械臂取放)下一步接线。
  ******************************************************************************
  */
#ifndef AUTO_TASK_H
#define AUTO_TASK_H

#include <stdint.h>

/* ==================== 场景区枚举 (对应视觉协议 scene) ==================== */
#define AT_SCENE_TURNTABLE  0   /* 原料区(转盘) */
#define AT_SCENE_ROUGH      1   /* 粗加工区 */
#define AT_SCENE_STORAGE    2   /* 暂存区 */

/* ==================== 动作类型 (对应视觉协议 kind) ====================== */
#define AT_KIND_PICK        0
#define AT_KIND_PLACE       1
#define AT_KIND_STACK       2

/* auto_grab_req.cmd: 底盘→机械臂请求类型 */
#define AT_GRAB_CMD_GRAB    0u   /* 普通取放(合爪取/张开放) */
#define AT_GRAB_CMD_SIGHT   1u   /* 仅伸到“抓取侧观测位”(不夹, 让夹爪端相机看场地目标) */

/* 每步序号长度 */
#define AT_SEQ_MAX          20

/* ==================== 一个动作步 ========================================= */
typedef struct
{
    uint8_t  kind;          /* AT_KIND_* */
    uint8_t  scene;         /* AT_SCENE_* */
    uint8_t  color;         /* 1红..6浅蓝 (PICK/STACK用) */
    uint8_t  ring;          /* 环号1..3 (PLACE/STACK用; PICK时可0) */
    char     seq[AT_SEQ_MAX];  /* 协议 req seq, 如 "B1_TP_PICK_1" */
} auto_step_t;

/* ==================== 一个目标点 (v1 系 mm) ============================== */
typedef struct
{
    float x;   /* v1: 向左为正 */
    float y;   /* v1: 向下为正 */
} at_point_t;

/* 整轮计划: 最多 24 步 = 2 批 × 12 步 */
#define AT_PLAN_MAX  24

extern auto_step_t auto_plan[AT_PLAN_MAX];
extern uint8_t     auto_plan_n;      /* 实际步数(正常情况下 24) */
extern uint8_t     auto_plan_ready;  /* 1=已由 task_code 生成 */

/* ==================== 跨任务: 底盘(总控) -> 机械臂 取放请求 =============
 * 底盘 AUTO 到站并拿到许可后, 置 active=1/act=动作并等 done;
 * grab_task 的 AUTO 消费: 执行一次取/放(夹爪序列)后置 done=1;
 * 底盘读回后把 active/done 清 0。
 * ======================================================================== */
typedef struct
{
    uint8_t  active;   /* 1=有请求待执行 */
    uint8_t  act;      /* AT_KIND_PICK / PLACE / STACK (SIGHT 时仅信息用) */
    uint8_t  scene;    /* AT_SCENE_* 该动作所在区 (信息用/纠偏基准) */
    uint8_t  ring;     /* 环号 1..3 (该动作放/取的环, 信息用) */
    uint8_t  tray;     /* 车上置物盘号 1..3 (本件放/取的盘, 抓取端终点) */
    uint8_t  cmd;      /* AT_GRAB_CMD_GRAB / AT_GRAB_CMD_SIGHT */
    uint8_t  done;     /* 1=机械臂已完成本次动作 */
    uint8_t  no_vis;   /* 1=本次抓/放忽略“视觉 xy 位移纠偏”, 直接按 nominal 端点抓/放(单测用);
                        * 0=正常: 有视觉(grab_ext_vis_en)则按 FK/IK 纠偏到目标点 */
} auto_grab_req_t;

extern auto_grab_req_t auto_grab_req;

/* 场景枚举 -> 视觉协议文本 (TURNTABLE/ROUGH/STORAGE) */
extern const char *auto_scene_str(uint8_t scene);

/**
  * @brief          按 task_code 全局(见 task_param.h)生成整轮 24 步动作表
  *                 批次/环号规则与《电控与 Jetson 对接协议.md》§8 一致:
  *                 PICK(TURNTABLE,颜色AAA/CCC) ×3
  *                 PLACE(ROUGH,环 BBB/DDD)      ×3
  *                 PICK(ROUGH,颜色AAA/CCC)      ×3
  *                 暂存: 批1=PLACE(STORAGE,环BBB); 批2=STACK(STORAGE,色CCC,环=批1同色环BBB位)
  * @retval         步数
  */
extern uint8_t auto_plan_build(void);

/**
  * @brief          清空计划 (回到未就绪)
  */
extern void auto_plan_clear(void);

/* ==================== 场地模型 ===========================================
 * 坐标 = 场地绝对坐标(mm), 原点 = 场地右上角定点(0,0); +x 向左, +y 向下。
 * 用户实测, 现场微调改下面宏即可(全部集中在这)。
 * 里程计原点 = 车启动摆位(场地绝对 150,150), chassis 换算自动减(150,150)。
 * ======================================================================== */

/* 原料区(转盘)盘心 */
#define AT_TURNTABLE_X   1200.0f
#define AT_TURNTABLE_Y   -75.0f

/* 粗加工区: 环1/2/3 */
#define AT_ROUGH_Y       2325.0f
#define AT_ROUGH_R1_X    1350.0f
#define AT_ROUGH_R2_X    1200.0f
#define AT_ROUGH_R3_X    1050.0f

/* 暂存区: 环1/2/3 */
#define AT_STORAGE_X     2325.0f
#define AT_STORAGE_R1_Y  1050.0f
#define AT_STORAGE_R2_Y  1200.0f
#define AT_STORAGE_R3_Y  1350.0f

/* 回启停(终点)= 车启动摆位: 启停区(右上300×300)中心附近 (150,150) */
#define AT_START2_X      150.0f
#define AT_START2_Y      150.0f

/* 二维码板(绝对): 与车启动同列 x=150, 车往下移 1050 到板 => y=1200 */
#define AT_QR_X          150.0f
#define AT_QR_Y          1200.0f

/* ===== 机械臂/车 安装与作业参数(默认占位值, 请按实测修改) ===== */
/* 机械臂底座相对车几何中心: 沿 vx 向前(车头方向)偏移 mm */
#define AT_ARM_OFF_X     50.0f
/* 沿车左侧偏移 mm (装车"偏上10mm"若指铅垂向上则看 AT_ARM_H) */
#define AT_ARM_OFF_Y     0.0f
/* 机械臂底座相对某基准铅垂高差 mm */
#define AT_ARM_H         10.0f
/* 水平前伸: 车停准后, 机械臂末端(爪口)到车几何中心沿车头方向的距离 mm
 *  => 停车点 = 区/环心 - 沿车头方向 AT_ARM_REACH_MM (执行层换算用) */
#define AT_ARM_REACH_MM  250.0f
/* 台面/转盘顶面高度(相对车底平面) mm, 机械臂抓取前需下伸量 */
#define AT_TABLE_H_MM    100.0f

/* ===== 视觉坐标 -> 机械臂末端偏移(相机装在夹爪末端) ====================
 * 当前 Jetson 未做平面标定时回整图绝对像素，默认先减
 * AT_VIS_CX/CY=640/360 再乘比例；完成平面标定后 unit=MM。
 * auto_vis_px_rel=1 只留给上游已经输出相对像素的兼容模式。
 */
extern uint8_t auto_vis_px_rel;  /* 0=整图绝对需减中心(默认); 1=已是相对 */
extern float  auto_vis_px2mm;    /* PX->mm 系数, 默认 0.5, Watch 可改/标定 */
extern float  auto_cam_rot;      /* 相机绕光轴的安装角(rad), 让图像轴对齐车轴; 默认0 */
extern float  auto_vis_cx_px;    /* 可抓取参考点 X，默认640，Watch标定 */
extern float  auto_vis_cy_px;    /* 可抓取参考点 Y，默认360，Watch标定 */

/* ==================== 场地查询 API ====================================== */
/**
  * @brief          取某区环心世界坐标 (v1)
  * @param[in]      scene: AT_SCENE_*; ring: 1..3 (TURNTABLE 时 ring 忽略)
  * @param[out]     x, y: 环心(或转盘心)世界坐标
  * @retval         1=有效 0=参数非法
  */
extern uint8_t auto_field_center(uint8_t scene, uint8_t ring, float *x, float *y);

/**
  * @brief          生成一个动作的协议 seq 文本 (如 "B1_TP_PICK_1")
  */
extern void auto_seq_make(uint8_t batch, uint8_t scene, uint8_t kind, uint8_t idx, char *out);

/**
  * @brief          视觉相对坐标 -> 机械臂末端偏移(mm)
  * @param[in]      vis_mm: 1=已回 MM; 0=回 PX(像素)
  * @param[in]      vis_x, vis_y: GRASP_READY/ALIGN_READY 的 x/y
  * @param[out]     dx, dy: 机械臂末端还需移动的偏移(mm), 加到“停准基准点”上
  * @retval         1=成功
  * @note           当前 PX 是整图绝对坐标，默认先减画面中心再乘系数；
  *                 auto_vis_px_rel=1 时才把 PX 直接视为相对坐标。
  */
extern uint8_t auto_vis_offset(uint8_t vis_mm, float vis_x, float vis_y,
                               float *dx, float *dy);

/**
  * @brief          空间转换: 相机(刚性在夹爪末端, 0=画面中心)相对偏移
  *                 -> 机械臂“前向 fx”与 车“侧向 sd”两个纠偏量(mm)
  * @param[in]      vis_mm: 1=已回 MM; 0=回 PX(像素)
  * @param[in]      vis_x, vis_y: GRASP_READY/ALIGN_READY 的 x/y(0,0=已对准)
  * @param[out]     fx: 沿机械臂平面前向偏移(mm, 正=臂再向前伸)
  * @param[out]     sd: 沿车体侧向偏移(mm, 正=车向左/vy+方向横移)
  * @retval         1=成功
  * @note 模型: 相机竖直朝下装在末端看台面; 图像轴相对车轴有安装旋转
  *       auto_cam_rot(rad, 默认0), 尺度 auto_vis_px2mm。台架标定项:
  *       scale(px→mm), rot(让图像轴与车轴对齐), 各轴正负号。
  *       目标在中心 => (fx,sd)=(0,0), 不需再动。
  */
extern uint8_t auto_vis_ground(uint8_t vis_mm, float vis_x, float vis_y,
                               float *fx, float *sd);

/**
  * @brief          [世界调试用] 合成 = 区基准(v1 mm) + 视觉偏移
  * @param[in]      scene/ring: 区与环(基准点); 其余同 auto_vis_offset
  * @param[out]     tx, ty: 区基准 + 偏移 (v1 mm, 仅看个大概)
  */
extern uint8_t auto_vis_target(uint8_t scene, uint8_t ring, uint8_t vis_mm,
                               float vis_x, float vis_y, float *tx, float *ty);

#endif /* AUTO_TASK_H */
