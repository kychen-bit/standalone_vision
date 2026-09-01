# Jetson、MaixCAM Pro 与电控通信协议

正式整车使用 `coordinator`；`live --serial` 的 `VISION_RESULT` 只用于单模块调试。

## 1. 公共帧格式

```text
@NAME,FIELD1,FIELD2,...*CRC16\n
```

- 全部字段使用 ASCII；
- 字段内禁止逗号、`*`、`@` 和换行；
- CRC 为 CRC16/CCITT-FALSE；
- 初值 `0xFFFF`，多项式 `0x1021`；
- CRC 计算范围只包含 `NAME,FIELD1,...` 正文。

## 2. 正式颜色抓取闭环

下面的示例均省略动态计算的 CRC：

```text
电控   → START,SIM_RUN
Jetson → READY,SIM_RUN

Maix   → MAIX_QR,156+123+516+231,320,240,120,5,STABLE
Jetson → TASK_PLAN,156+123+516+231,1,5,6,5,1,6

电控   → REQ,SIM001,PICK,TURNTABLE,1
Jetson → ACCEPTED,SIM001,PICK

Jetson → GRASP_READY,SIM001,1,590.000,370.000,PX,590.000,370.000,1.000,5,250
电控   → EXEC,SIM001
Jetson → EXEC_ACK,SIM001

电控   → DONE,SIM001,OK
Jetson → DONE_ACK,SIM001,OK
```

`GRASP_READY` 字段依次为：

```text
seq,color,x,y,unit,pixel_x,pixel_y,confidence,stable_frames,valid_ms
```

当前平面没有标定时 `unit=PX`，因此 `x/y` 与 `pixel_x/pixel_y` 相同。它表示视觉结果稳定，
并不自动完成像素到机械臂坐标的换算。

电控必须在 `valid_ms` 内发送同序号 `EXEC`。在收到 `EXEC` 前：

- 连续丢失目标会发送 `GRASP_REVOKED,<seq>,LOST`；
- 中心重新变得不稳定会发送 `GRASP_REVOKED,<seq>,UNSTABLE`；
- 超过有效期会发送 `GRASP_REVOKED,<seq>,EXPIRED`。

收到撤销后不要执行抓取；协调器会回到搜索状态，重新满足稳定条件后再次发送许可。

## 3. 电控输入

| 帧 | 作用 |
|---|---|
| `START,<run_id>` | 开始一轮任务并清空旧状态 |
| `REQ,<seq>,PICK,<scene>,<color>` | 显式识别指定颜色 |
| `REQ,<seq>,PICK` | 使用任务队列当前颜色，不立即推进队列 |
| `REQ,<seq>,PLACE,<scene>,<ring>` | 识别指定圆环 |
| `REQ,<seq>,STACK,<scene>,<color>,<ring>` | 识别指定环内的颜色物料 |
| `EXEC,<seq>` | 确认已收到且开始执行当前抓取许可 |
| `DONE,<seq>,OK` | 整个动作成功，自动颜色队列此时才推进 |
| `DONE,<seq>,FAIL` | 动作失败，保持同一目标并重试 |
| `ABORT,<run_id>` | 中止并清空任务，Jetson 返回 `ABORTED` |

正式运行建议显式传颜色，避免电控重启或人工恢复时与自动队列不同步。

## 4. 状态约束

```text
IDLE
  └─ START → WAIT_TASK
       └─ MAIX_QR → TASK_READY
            └─ REQ → BUSY
                 ├─ PICK稳定 → WAIT_EXEC
                 │    ├─ EXEC → WAIT_DONE
                 │    └─ 丢失/超时 → BUSY重新搜索
                 └─ PLACE/STACK稳定 → WAIT_DONE
                      └─ DONE → TASK_READY
```

- 只有 `TASK_READY` 接受新 `REQ`；
- `WAIT_EXEC/WAIT_DONE` 不允许新请求覆盖旧请求；
- 搜索超时、`DONE FAIL` 不推进自动颜色队列；
- 只有 `DONE OK` 推进自动颜色队列；
- 错误帧使用 `ERROR,<seq或NONE>,<reason>,...`，不发送空字段。

## 5. 单模块调试输出

`live --serial` 仍可发送：

```text
VISION_RESULT,state,kind,target_id,x,y,yaw,confidence,unit,stable_frames
```

它不包含 `REQ/EXEC/DONE` 状态机，不能与正式 `coordinator` 同时占用相机或串口。

## 6. 无硬件闭环测试

```bash
python3 tools/simulate_color_control_loop.py
```

该工具使用真实 HSV 检测、五帧稳定、CRC 编解码和协调器状态机，但用合成红色色块代替相机、
用内存帧代替串口。成功结束时输出：

```text
SIMULATION_OK
```
