# 电控 Watch 手动联调流程

本文用于没有完整底盘和机械臂实机时，通过 Keil Watch 手动触发 C 板命令，验证
`C 板 ↔ Jetson ↔ MaixCAM` 的完整协议流程。正式协议细节以
[电控对接协议](电控对接协议.md)为准。

## 1. 联调前准备

通信参数为 `115200、8N1、无流控`。Jetson 使用 `coordinator.mcu_serial` 配置的串口。
正式测试直接运行：

```bash
./run_competition.sh
```

排查 C 板接收时可临时让 Jetson 每秒重发最后一帧：

```bash
./run_competition.sh --mcu-repeat-last-frame-seconds 1
```

重复模式只用于静止联调，进入运动测试前必须关闭。日志中 `MCU_TX` 是首次发送，
`MCU_TX_REPEAT` 是调试重发，`MCU_RX` 是 Jetson 收到 C 板帧。

先在 Watch 加入：

```c
jetson.phase
jetson.last_frame
jetson.rx_ok
jetson.tx_ok
jetson.crc_err
jetson.fmt_err
jetson.err_reason
jetson.task_plan_ok
jetson.colors
task_code
auto_plan_ready
auto_plan_n
```

Watch 中的 `uint8_t` 数组应按无符号十进制或十六进制查看，不能按字符串查看。
`jetson_service()`必须周期执行，`jetson_link_init()`只能在上电后执行一次。

## 2. 阶段一：开始一轮任务

暂时把二维码移出 MaixCAM 画面。Jetson出现 `COORDINATOR_READY` 后，在 Watch 设置：

```c
jetson_cmd_start = 1;
```

该变量应被 `jetson_service()`自动清零。C 板发送：

```text
@START,RUN001*E367\n
```

Jetson回复 `READY,RUN001`。应观察到：

```text
jetson.phase      = JETSON_PH_TASK_WAIT
jetson.last_frame = "READY"
jetson.rx_ok      增加
```

每轮只能发送一次 START。重复 START 会清空已经收到的任务码和当前动作。
READY 之后 C 板无限等待 TASK_PLAN，不因扫码或人工摆放时间较长而进入本地超时。

## 3. 阶段二：接收二维码任务计划

确认收到 READY 后再把二维码放入 MaixCAM 画面。MaixCAM 向 Jetson发送 `MAIX_QR`，
C 板不需要向 MaixCAM 发送内容。Jetson接受任务码后向 C 板发送：

```text
TASK_PLAN,<原始任务码>,<第1批3个颜色>,<第2批3个颜色>
```

例如二维码 `156+123+516+231` 对应：

```text
TASK_PLAN,156+123+516+231,1,5,6,5,1,6
```

C 板应进入：

```text
jetson.phase        = JETSON_PH_TASK_READY
jetson.task_plan_ok = 1
jetson.colors       = {1,5,6,5,1,6}
auto_plan_ready     = 1
auto_plan_n         = 24
```

完整四组任务保存在 `task_code`，24步执行计划保存在 `auto_plan`。如果二维码在 START 前
已经被发送，应移开若干帧后重新展示。

## 4. 阶段三：手动验证 PICK

确认 `jetson.phase == JETSON_PH_TASK_READY`，在相机下放置任务队列首个颜色物料，然后设置：

```c
jetson_cmd_demo_pick = 1;
```

C 板发送 `REQ,<seq>,PICK,TURNTABLE,<color>`。Jetson依次回复：

```text
ACCEPTED,<seq>,PICK
GRASP_READY,<seq>,<color>,<x>,<y>,<unit>,<pixel_x>,<pixel_y>,<confidence>,<stable>,<valid_ms>
```

收到 GRASP_READY 后应看到：

```text
jetson.phase     = JETSON_PH_PICK_WAIT_EXEC
jetson.grasp_ok  = 1
jetson.grasp_x/y = 识别坐标
```

在有效期内设置：

```c
jetson_cmd_exec = 1;
```

C 板发送 `EXEC,<seq>`，收到 `EXEC_ACK,<seq>` 后进入 `JETSON_PH_PICK_ACT`。无实机时把它
理解为“模拟机械动作完成”，设置：

```c
jetson_cmd_done = 1;
```

C 板发送 `DONE,<seq>,OK`，收到 `DONE_ACK,<seq>,OK` 后回到
`JETSON_PH_TASK_READY`。若要模拟动作失败，将 `jetson_cmd_done` 设为 `2`。

若不确定当前观察位上是哪种颜色，用转盘巡检（协议 3.7）：

```text
C 板发送 REQ,<seq>,CLASSIFY,TURNTABLE,<本轮3色>
Jetson 回复 ACCEPTED，随后 ALIGN_READY,<seq>,CLASSIFY,<实际颜色>,<x>,<y>,0.000,<unit>,<confidence>,<stable>
```

第 3 字段就是实际识别到的颜色号；同样不发 `EXEC`，用完设 `jetson_cmd_done = 1`。
决定要抓之后再发 `REQ,<seq>,PICK,TURNTABLE,<该颜色>` 走正常抓取流程。
观察位里什么都没有时不会回帧，等到 `TIMEOUT`（当前 `timeout_s`），电控应把它当成"没看到物料"。

## 5. 阶段四：手动验证 PLACE

在圆环区测试画面准备好后设置：

```c
jetson_cmd_demo_place = 1;
```

C 板发送 `REQ,<seq>,PLACE,ROUGH,1`，Jetson回复：

```text
ACCEPTED,<seq>,PLACE
ALIGN_READY,<seq>,PLACE,1,<x>,<y>,<yaw>,<unit>,<confidence>,<stable>
```

PLACE 没有 EXEC。收到 ALIGN_READY 后模拟放置完成：

```c
jetson_cmd_done = 1;
```

同一个圆环区还要做**车体纠偏**时用 `REQ,<seq>,LOCATE,ROUGH`（协议 3.6）：回复同样是
`ALIGN_READY`，但 `kind=LOCATE`、目标位是锚点环号（默认 `2`），同样不发 `EXEC`；电控按
`x/y` 把车开到目标位置后再设 `jetson_cmd_done = 1`。注意 `unit=PX` 时这个偏移不能用于车体定位。

收到 DONE_ACK 后应回到 `JETSON_PH_TASK_READY`。

## 6. 阶段五：手动验证 STACK

在暂存区测试画面准备好后设置：

```c
jetson_cmd_demo_stack = 1;
```

C 板发送 `REQ,<seq>,STACK,STORAGE,<color>,1`，Jetson回复 ACCEPTED 和
`ALIGN_READY,<seq>,STACK,...`。同样不发送 EXEC，模拟动作完成时设置：

```c
jetson_cmd_done = 1;
```

## 7. 中止并重新开始

任意阶段要清空任务时设置：

```c
jetson_cmd_abort = 1;
```

收到 ABORTED 后 `jetson.phase` 回到 `JETSON_PH_IDLE`。重新测试必须再次设置
`jetson_cmd_start=1`，收到 READY 后重新展示二维码。

## 8. 单环节模式（逐个动作）

正式流程之前，建议 C 板也支持"一个变量触发一个动作"：帧序列与正式流程完全相同，只是不依赖
任务码里的顺序。视觉侧对应的真机脚本看 [整机闭环测试](整机闭环测试.md) 的"单环节实机联调"一节。

| 单环节 | C 板变量 | C 板发送 | 收到什么算通过 |
|---|---|---|---|
| 物料识别抓取 | `jetson_cmd_demo_pick = 1` | `REQ,<seq>,PICK,TURNTABLE,<color>` | `GRASP_READY` → `EXEC` → `EXEC_ACK` → `DONE` |
| 车舱取料放入圆环 | `jetson_cmd_demo_place = 1` | `REQ,<seq>,PLACE,ROUGH,<ring>` | `ALIGN_READY`（第 3 字段=环号）→ `DONE` |
| 圆环区取回 | 同上，`scene=ROUGH` | `REQ,<seq>,PICK,ROUGH,<color>` | 同 PICK |
| 暂存区码垛 | `jetson_cmd_demo_stack = 1` | `REQ,<seq>,STACK,STORAGE,<color>,<ring>` | `ALIGN_READY`（第 3 字段=颜色）→ `DONE` |
| 转盘巡检 | `jetson_cmd_demo_classify = 1` ★新增 | `REQ,<seq>,CLASSIFY,TURNTABLE,<c1>,<c2>,<c3>` | `ALIGN_READY` 第 3 字段=实际颜色；没看到物料时接受 `TIMEOUT` |
| 圆环区纠偏 | `jetson_cmd_demo_locate = 1` ★新增 | `REQ,<seq>,LOCATE,ROUGH,2` | `ALIGN_READY` 第 3 字段=锚点环号 |

标 ★ 的两个变量是本次新增建议，按前四个的风格实现即可。四个 `ALIGN_READY` 类环节都不发
`EXEC`，电控用完结果后统一设 `jetson_cmd_done = 1`。

## 8. 坐标偏差与当前能力

当前 [calibration.json](../config/calibration.json) 的三个作业平面均为
`calibrated:false`。Jetson因此发送 `unit=PX`，其中 `x/y` 是1280×720整幅图像中的
绝对像素坐标，原点在左上角，X向右，Y向下。当前代码以可调变量
`(auto_vis_cx_px,auto_vis_cy_px)` 作为参考点，默认 `(640,360)`：

```text
du = x - auto_vis_cx_px
dv = y - auto_vis_cy_px
```

C 板的 `auto_vis_ground()`再使用 `auto_vis_px2mm`、`auto_cam_rot` 和轴符号转换为：

```text
grasp_tar_x / align_tar_x：机械臂前后修正量
grasp_tar_y / align_tar_y：底盘左右修正量
```

现有执行能力有以下限制：

1. `auto_vis_px2mm=0.5` 只是占位值，没有实测标定。
2. `auto_cam_rot=0` 只是安装角占位值。
3. `grab_ext_vis_en=0`，机械臂前后视觉修正默认关闭。
4. 机械臂代码启用后只使用 `grasp_tar_x/align_tar_x`；计算出的左右修正量目前没有接入
   底盘二次横移控制。
5. 因此当前程序可以识别并传递 XY，也能计算两个方向的候选修正量，但尚不能可靠完成
   “偏了以后底盘自动移到相机正下方”的完整 XY 闭环。

在没有完成标定前，不要直接启用视觉位移控制。联调时先只验证坐标随目标移动的方向和重复性。

## 9. 必须完成的坐标标定

至少需要完成以下三项：

1. **参考中心**：把物料人工放到夹爪真正能够垂直抓取的位置，记录此时图像坐标。它不一定
   恰好是 `(640,360)`。
2. **像素比例**：让目标沿场地 X、Y 各移动一个已知毫米距离，计算每个方向的 px/mm；5–6 cm
   的近距离成像仍需按实际工作高度分别验证。
3. **轴向与旋转**：确认图像向右、向下分别对应车体哪个方向，标定 `auto_cam_rot` 和正负号。

更完整的做法是在 TURNTABLE、ROUGH、STORAGE 三个固定高度平面分别采集至少4个不共线点，
填写像素到平面毫米的单应矩阵，并把可抓取参考点定义为毫米坐标原点 `(0,0)`；否则 C 板还要
再减去对应的毫米参考点。完成后 Jetson会发送 `unit=MM`。镜头内参与畸变标定只能修正镜头
变形，不能代替像素到作业平面的坐标标定。

最终闭环建议分两步执行：底盘先使用侧向偏差做限幅的小步横移并重新请求识别；进入允许范围后，
机械臂再使用前后偏差调整抓取端点。每次移动后都重新识别，不能把一次未经标定的像素结果直接
作为毫米位移执行。

## 10. 单一高度的现场标定步骤

到站位置每次不同并不妨碍视觉纠偏。参考点固定在相机/夹爪上，每次到站后重新识别目标位置，
本次测得的位置与参考点之差就是本次偏差。目标必须仍在视野和检测 ROI 内；超出视野时需要底盘
搜索策略，视觉无法从空画面推断方向。

所有任务使用同一识别高度可以只标定一次，但每次识别还必须使用同一个机械臂观测姿态：相机
高度、俯视角和绕光轴角度都要重复。识别完成后的机械臂旋转不影响该次结果；在不同姿态下识别
则需要按关节角动态旋转坐标，当前代码没有实现这种动态外参。

当前没有单独的自动 mm/px 标定程序。使用整机识别和 Watch 按以下步骤完成局部标定：

1. 固定机械臂观测姿态和识别高度，保持 `auto_vis_px_rel=0`。
2. 把物料中心人工对准夹爪真实可抓取点，执行一次 PICK 识别；把收到的
   `jetson.grasp_x/y` 分别写入 `auto_vis_cx_px/cy_px`。
3. 沿场地直线把物料平移已知距离 `L`（建议50 mm以上），相机姿态不能动，再识别得到新像素
   `(x1,y1)`。
4. 计算 `pixel_distance = sqrt((x1-cx)^2+(y1-cy)^2)`，设置
   `auto_vis_px2mm = L/pixel_distance`。
5. 分别沿车体前向和侧向各移动一次，检查 `grasp_tar_x/y`：前向移动应主要改变
   `grasp_tar_x`，侧向移动应主要改变 `grasp_tar_y`。调节 `auto_cam_rot` 使串轴分量最小，
   再核对两个方向的正负号。

`auto_vis_px2mm` 是单一等比例模型。若两个方向测出的 mm/px 差异明显，或目标在视野不同位置时
比例变化明显，应改用同一高度的二维单应矩阵。三个场景可以复用同一矩阵，但矩阵输出的毫米
坐标必须以夹爪可抓取参考点为 `(0,0)`。
