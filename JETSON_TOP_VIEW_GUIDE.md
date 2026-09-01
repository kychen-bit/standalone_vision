# Jetson 俯视视觉：赛题理解、项目取舍与启动说明

> 历史架构分析：其中 Lab 分类、动态 ROI 和旧颜色标定流程已被 `HSV_CONTOUR_V1` 禁用。
> 当前颜色运行与调参只以根目录 `JETSON_RECOGNITION_TEST_GUIDE.md` 为准。

> 审查日期：2026-08-29  
> 适用目录：`standalone_vision/`  
> 赛题依据：`附件2-1-第十二届河北省大学生大赛智能+创新赛道命题与运行.pdf` 中的“智能搬运赛项”。PDF 只作为赛题资料读取。

## 1. 先说结论

这个项目的总体分工符合需求：

```text
MaixCAM Pro（前视扫码）
          │ MAIX_QR 任务码
          ▼
Jetson（俯视识别 + 协调服务）
          │ TASK_PLAN / GRASP_READY / ALIGN_READY / ERROR
          ▼
电控（导航、机械臂、夹爪、动作完成反馈）
```

Jetson 正式运行应使用 `coordinator`，不是 `live`、`image`、`pickup` 或 `bridge`。后四者是单项测试工具。

公共通信 P0 已完成代码修复和无硬件 CRC 闭环测试。完整上车前仍需处理现场配置项：

1. `config/jetson.json` 中两路串口仍是 `REPLACE_WITH_...`；
2. 三个平面都未标定，当前只能返回 `PX`，不能直接当毫米坐标使用；
3. 任务码的放置环号没有自动进入 `PLACE/STACK` 流程；电控仍需解析原始任务码或显式给每个请求传环号；
4. `GRASP_READY` 当前代表“颜色中心稳定”，电控仍需负责像素坐标到实际动作的换算和安全判断；
5. 真实 Maix—Jetson—电控双串口硬件还未接线验证。

已修复的公共问题包括：`TASK_PLAN` 可编码、串口非阻塞、五帧稳定、许可有效期与 `EXEC` 握手、
新请求状态门、自动颜色队列在 `DONE OK` 后推进以及发送异常保护。

## 2. 赛题对视觉的实际要求

### 2.1 初赛任务

初赛需要读取四组三位任务码，例如：

```text
156+123+516+231
```

- 第一组 `156`：第一批颜色和抓取顺序，即红、黑、浅蓝；
- 第二组 `123`：第一批在粗加工区、暂存区的环号；
- 第三组 `516`：第二批颜色和抓取顺序；
- 第四组 `231`：第二批在粗加工区的环号；
- 第二批到暂存区时，要叠放在第一批的同色物料上。

颜色编号：`1=红，2=黄，3=蓝，4=绿，5=黑，6=浅蓝`。

视觉相关环境条件：原料区是直径约 300 mm 的电动转盘，转速约 6～10 秒/圈并会停留；二维码约 80×80 mm；现场位置、色差和摆放会有偏差。规则还要求不遮挡场地，补光只允许垂直向下。

### 2.2 建议硬件职责

| 板卡 | 必须承担 | 不应放在这里 |
|---|---|---|
| MaixCAM Pro | 读取、校验并稳定确认二维码任务码 | 俯视物料与圆环识别 |
| Jetson | 俯视识别颜色物料、圆环中心、码垛目标；收任务码；响应电控请求 | 导航、底盘控制、机械臂轨迹、夹爪闭环 |
| 电控 | 一键启动、区域移动、机械臂和夹爪动作、向 Jetson 发 `REQ/DONE` | 图像算法 |

这与当前项目的目标分工基本一致。

## 3. 当前视觉功能如何取舍

### 3.1 正式任务需要保留

| 模块/模式 | 用途 |
|---|---|
| `COLOR` | 在原料转盘中找当前任务颜色 |
| `PickupSession` | 检查目标是否进入抓取窗口、是否低速、是否多帧稳定和最终复检 |
| `RING` | 找粗加工区 1/2/3 号圆环中心，支持 `PLACE` |
| `STACK` | 在暂存区指定圆环 ROI 内找第一批同色物料中心，支持第二批码垛 |
| `StableWindow` | 防止单帧抖动或误检直接触发动作 |
| `task_code.py` | 校验和解析四组三位任务码 |
| `protocol.py` | Maix、Jetson、电控之间的 CRC16 帧 |
| `coordinator.py` | 正式整机服务入口 |
| `camera.py` | OG05B10/DECXIN UVC 相机输入和可选畸变校正 |

### 3.2 视觉定位辅助，可保留但不是当前主流程必需

| 功能 | 结论 |
|---|---|
| `STATION` | 用多个圆环估计整个工位平移和偏航；当前 `coordinator` 不接受这种 `REQ`，属于辅助对位/诊断 |
| `TURNTABLE` | 只检测转盘外圆中心；当前 `PICK` 实际直接做颜色检测，没有调用转盘圆心检测 |
| `geometry.py` 的刚体位姿 | 只服务 `STATION` |
| 平面单应性 | 如果要直接给电控毫米坐标则需要；如果电控明确按像素误差闭环，可先作为后续项 |
| 相机内参畸变校正 | 边缘精度要求高时有用，不影响代码先跑通 |

这些辅助内容不会主动导航，也不会控制机械臂。无需为了精简而立即删除；正式入口不调用的功能不会占用每帧算力。当前更重要的是修复协调链路和完成标定。

当前已通过 `runtime.enabled_modes` 默认只启用：

```json
["COLOR", "RING", "STACK"]
```

需要辅助定位时，把 `STATION` 或 `TURNTABLE` 加入数组即可，无需恢复或复制代码。

### 3.3 只用于测试，不是整机入口

- `image`：单张保存图片检测；
- `live`：相机实时观察某一种算法；
- `pickup`：单独验证转盘抓取安全门；
- `bridge`：只接收/转发 Maix 帧；
- `ResultOutput` 和顶层 `serial` 配置：服务单项测试输出，正式 `coordinator` 不使用；
- `run_jetson_live.sh`、`run_jetson_image.sh`：只是命令包装器，而且当前没有可执行权限，应使用 `bash run_jetson_live.sh ...`。

## 4. 代码实际数据流

### 4.1 Maix 到 Jetson

Maix 连续确认合法二维码后发送：

```text
@MAIX_QR,156+123+516+231,320.0,240.0,120.0,4,STABLE*CRC16\n
```

Jetson 只使用第一个字段，即完整任务码；二维码中心、边长、确认帧数和 `STABLE` 当前不参与后续逻辑。

Maix 默认 `SERIAL_ENABLED = False`，联合运行前必须在 `maixcam_pro/config.py` 改为 `True`。Maix 对同一个持续可见的二维码只发一次；所以应先启动 Jetson，再让二维码进入 Maix 画面。若错过，需让二维码离开至少 8 帧后重新进入。

### 4.2 Jetson 到电控

电控通过另一条双向串口驱动视觉：

```text
START → READY
Maix任务码 → TASK_PLAN
REQ PICK → ACCEPTED → GRASP_READY → EXEC → DONE OK/FAIL
REQ PLACE → ACCEPTED → ALIGN_READY → DONE OK/FAIL
REQ STACK → ACCEPTED → ALIGN_READY → DONE OK/FAIL
ABORT → ABORTED → 回到 IDLE
```

所有线上帧都必须是：

```text
@NAME,FIELD,...*四位十六进制CRC\n
```

CRC 算法是 CRC16/CCITT-FALSE，初值 `0xFFFF`，多项式 `0x1021`，计算范围仅为 `NAME,FIELD,...` 正文。

### 4.3 三种正式请求

以下只写 CRC 前的正文，实际发送必须补 `@`、`*CRC` 和换行。

```text
REQ,0001,PICK
```

使用任务颜色队列的当前颜色。只有收到同请求的 `DONE OK` 后才推进；超时、许可撤销和
`DONE FAIL` 都保持当前颜色。恢复时仍建议显式指定颜色。

```text
REQ,0002,PICK,TURNTABLE,5
```

显式找黑色，不消耗自动颜色队列。

```text
REQ,0003,PLACE,ROUGH,2
```

找粗加工区 2 号圆环中心。

```text
REQ,0004,STACK,STORAGE,5,2
```

在暂存区 2 号环 ROI 内找黑色物料中心，用于码垛。

`PLACE` 和 `STACK` 的环号不会由协调器根据任务码自动生成，必须由电控显式传入。

## 5. Jetson 部署与设备确认

### 5.1 安装依赖

Jetson 上优先使用 JetPack 自带 OpenCV：

```bash
cd /home/ysu/standalone_vision
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-pip v4l-utils
python3 -m pip install --user -r requirements-jetson.txt
```

验证：

```bash
python3 -c "import cv2, numpy, serial; print(cv2.__version__, numpy.__version__, serial.VERSION)"
python3 -m unittest discover -s tests -v
python3 -m compileall -q jetson_recognition maixcam_pro tests tools
```

当前测试包含最终 CRC 编解码，以及使用真实 HSV 检测器的模拟电控颜色闭环；真实相机和双串口
仍需上板验证。

### 5.2 查三个设备名

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* 2>/dev/null
```

需要确认：

- 一个 UVC 相机设备；
- 一条 Maix → Jetson 串口；
- 一条 Jetson ↔ 电控串口。

优先用 `/dev/.../by-id/...`，避免重启后 `/dev/video0`、`/dev/ttyUSB0` 编号互换。若使用 Jetson GPIO 原生 UART，设备可能是 `/dev/ttyTHS*`，不会出现在 `by-id` 中。

TTL 连接必须确认 3.3 V 电平、TX/RX 交叉和共地。不要把 5 V TTL 直接接到 Jetson 或 Maix IO。

### 5.3 检查相机实际格式

已有记录显示 DECXIN 相机支持 1280×720 MJPG 30/60 FPS，当前配置已适配为 1280×720、MJPG、60 FPS，并使用后台持续取帧、只消费最新帧的低延迟模式。上板后仍应重新确认：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --get-fmt-video
v4l2-ctl -d /dev/video0 --get-parm
v4l2-ctl -d /dev/video0 --list-ctrls
```

后续提供的 `DECXIN_controls.txt` 和 `DECXIN_all.txt` 已确认真实控件及当前值。由于实测发现
直接打开相机时颜色正常、强制手动白平衡/焦距后出现偏色和模糊，因此当前打印纸与无补光调试
默认使用 `camera_default_auto`；实物固定补光后再用 `physical_4600k` 作为
初值。启动后仍会回读实际分辨率、FPS 和 FourCC，最终参数应根据运动模糊、亮度和颜色分离
效果继续调整。

## 6. `config/jetson.json` 必填项

至少修改：

```json
{
  "camera": {
    "device": "/dev/v4l/by-id/实际相机设备",
    "width": 1280,
    "height": 720,
    "fps": 60,
    "fourcc": "MJPG",
    "buffer_size": 1,
    "threaded_capture": true,
    "read_timeout_s": 0.25,
    "strict_settings": false
  },
  "coordinator": {
    "maix_serial": "/dev/serial/by-id/实际Maix串口",
    "mcu_serial": "/dev/serial/by-id/实际电控串口",
    "baud": 115200,
    "scene": "TURNTABLE",
    "timeout_s": 20.0,
    "max_attempts": 3,
    "target_report_every": 0,
    "action_timeout_s": 30.0,
    "result_valid_ms": 250,
    "revoke_after_misses": 2,
    "require_exec_ack": true
  }
}
```

还必须用实拍数据调整：

- `scenes.TURNTABLE.object_roi`：转盘物料搜索范围；
- `scenes.ROUGH.ring_search_roi`：机械臂相机可能看到的粗加工工位搜索范围；
- `scenes.STORAGE.ring_search_roi`：暂存工位搜索范围；`ring_rois.1/2/3` 仅保留给旧兼容路径和 STACK；
- `colors.1..6.ranges`：六种实物在现场光照下的 HSV 范围；
- 颜色面积、形状过滤和圆半径范围；
- `stability` 的颜色连续稳定帧数和中心允许偏移；
- `runtime` 的圆环/堆叠稳定参数，以及 `coordinator` 的等待、许可有效期和动作超时参数。

### 6.1 旧版单项 `pickup` 抓取窗口工具

下面的工具只服务于旧版单项 `pickup` 测试命令，不参与当前正式
`coordinator` 的 `REQ → GRASP_READY → EXEC → DONE` 闭环。正式模式只给出颜色中心，
导航、图像坐标到机械动作的换算和安全抓取窗口判断均由电控负责，因此部署正式协议时不需要配置
或复制 `coordinator.pickup`。

带桌面：

```bash
python3 tools/capture_zone_calibrator.py \
  --camera /dev/v4l/by-id/实际相机设备 \
  --target 1 --scene TURNTABLE
```

SSH 无桌面时可用保存图片和已测矩形：

```bash
python3 tools/capture_zone_calibrator.py \
  --image frame.jpg --target 1 --scene TURNTABLE \
  --x 420 --y 220 --w 180 --h 160 --headless --save
```

这里的数字只是命令格式示例，不能直接照抄。窗口应表示机械结构真正可抓取的区域，而不是整个转盘 ROI。

工具当前只会保存到：

```text
pickup.capture_zone_px
```

该值仅供 `python3 -m jetson_recognition.run pickup ...` 使用。若未来确实需要在 Jetson
正式协调器中增加第二层抓取安全窗口，应单独设计明确的配置字段和协议语义，不要复用这一旧配置。

### 6.2 坐标标定

当前 `config/calibration.json` 中 `TURNTABLE`、`ROUGH`、`STORAGE` 都是 `calibrated: false`，输出单位为 `PX`。

如果电控期待机器人平面毫米坐标，需要分别标定三个固定观察姿态/平面，并填入各自的 `homography_px_to_robot_mm`。当前仓库只有读取单应矩阵的代码，没有生成该矩阵的标定工具。

相机若装在夹爪末端，不同高度、姿态和目标平面不能共用同一个单应矩阵。第一版建议让电控先到固定观察位，视觉给 XY，再由电控垂直下降。

## 7. 命令行参数怎么填

### 7.1 全局参数

全局参数必须放在子命令之前：

| 参数 | 含义 | 正式模式是否使用 |
|---|---|---|
| `--config PATH` | 配置文件；相对路径以项目根目录解析 | 是 |
| `--serial DEV` | 单项测试结果输出串口 | 否，`coordinator` 忽略 |
| `--baud N` | 单项测试串口波特率 | 正式模式不要依赖它 |

正确：

```bash
python3 -m jetson_recognition.run --config config/jetson.json coordinator ...
```

不要写成：

```bash
python3 -m jetson_recognition.run coordinator --config config/jetson.json
```

### 7.2 `coordinator`：正式整机入口

| 参数 | 填法 |
|---|---|
| `--maix-serial` | 接收 Maix `MAIX_QR` 的设备名；给出时覆盖 JSON |
| `--mcu-serial` | 与电控双向通信的设备名；给出时覆盖 JSON |
| `--camera` | UVC 相机编号或设备路径；给出时覆盖 JSON |
| `--baud` | 代码看似支持，但 JSON 中已有 `coordinator.baud` 时 JSON 优先，因此当前不能真正覆盖；直接修改 JSON |

完整命令格式：

```bash
cd /home/ysu/standalone_vision
python3 -m jetson_recognition.run --config config/jetson.json coordinator \
  --maix-serial /dev/serial/by-id/实际Maix串口 \
  --mcu-serial /dev/serial/by-id/实际电控串口 \
  --camera /dev/v4l/by-id/实际相机设备
```

如果 JSON 已填好：

```bash
cd /home/ysu/standalone_vision
python3 -m jetson_recognition.run coordinator
```

启动成功只会打印：

```json
{"state":"COORDINATOR_READY","maix":"...","mcu":"..."}
```

收到任务码和发给电控的帧默认不会打印到控制台，调试时要在电控端监视串口。

### 7.3 `bridge`：只验证 Maix → Jetson

```bash
python3 -m jetson_recognition.run bridge \
  --in-serial /dev/serial/by-id/实际Maix串口 \
  --baud 115200
```

可选加 `--out-serial` 原样转发到第二串口。`bridge` 会占用 Maix 串口，验证完成后必须退出，不能和 `coordinator` 同时运行。

### 7.4 `live`：单项实时识别

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene TURNTABLE --target 1 \
  --camera /dev/v4l/by-id/实际相机设备 --headless
```

参数：

- `--mode`：`COLOR/RING/STACK/STATION/TURNTABLE`；
- `--scene`：`TURNTABLE/ROUGH/STORAGE`，必须与配置键一致；
- `--target`：颜色号或圆环号；
- `--ring`：仅 `STACK` 使用；
- `--headless`：SSH 无桌面时必须加；
- `--print-every N`：跟踪状态每 N 帧打印一次，不改变算法频率。

圆环和码垛示例：

```bash
python3 -m jetson_recognition.run live \
  --mode RING --scene ROUGH --target 2 --camera /dev/video0 --headless

python3 -m jetson_recognition.run live \
  --mode STACK --scene STORAGE --target 5 --ring 2 \
  --camera /dev/video0 --headless
```

### 7.5 `pickup`：单独验证转盘抓取门

```bash
python3 -m jetson_recognition.run pickup \
  --target 1 --scene TURNTABLE --camera /dev/video0 \
  --capture-zone X Y W H --headless --timeout 20 --exit-on-ready
```

- `--capture-zone` 只临时覆盖顶层 `pickup` 配置；
- `--timeout` 覆盖搜索超时秒数；
- `--exit-on-ready` 首次许可后退出；
- 这个模式只打印/可选发送测试结果，不接收电控 `REQ/DONE`。

### 7.6 `image`：保存图片离线调参

```bash
python3 -m jetson_recognition.run image frame.jpg \
  --mode COLOR --scene TURNTABLE --target 1 \
  --save result.jpg
```

有桌面时可用 `--show`。检测到目标退出码为 0，未检测到为 2。

## 8. 推荐的整机启动顺序

修复本文第 1 节阻塞项并完成配置后，按以下顺序启动：

1. 上电，但先不要让二维码持续出现在 Maix 画面中；
2. 确认相机和两路串口设备名没有互换；
3. 启动 Jetson `coordinator`，看到 `COORDINATOR_READY`；
4. 启动 Maix 程序，确认 `SERIAL_ENABLED=True`；
5. 电控一键启动后发送 `START,<run_id>`，等待 `READY,<run_id>`；
6. 让 Maix 扫到二维码，电控接收并核对 `TASK_PLAN`；
7. 电控移动到原料观察位，发第一个 `REQ,...,PICK`；
8. 收到 `GRASP_READY` 后在 `valid_ms` 内回复 `EXEC,<seq>`，收到 `EXEC_ACK` 才执行；完整抓取并放上车载平台后发 `DONE,<seq>,OK`；
9. 到粗加工区后，电控按任务码环号发 `PLACE`；
10. 到暂存区后，第一批可按环号放置，第二批发 `STACK` 找同色物料中心；
11. 每次完整动作结束都必须发 `DONE`；`WAIT_EXEC/WAIT_DONE` 中的新 `REQ` 会被明确拒绝；
12. 异常时发 `ABORT,<run_id>`，重新建立明确状态。

正式比赛前不要只验证“看到了目标”，必须完整走通：

```text
START → MAIX_QR → TASK_PLAN → 6次PICK（每次GRASP_READY/EXEC/DONE）→ PLACE/STACK → ABORT/结束
```

## 9. 实时性分析

### 9.1 当前已经完成的识别侧优化

- HSV 阈值数组、形态学核和 CLAHE 对象只在启动时创建，不再逐帧重复分配；
- 静止/连续目标优先在上一位置附近的小 ROI 检测，失败时在同一帧自动回退全场景 ROI；
- 圆环在工位搜索 ROI 内找出全部同心轮廓，再动态裁剪中央数字做 1/2/3 模板匹配；
- 相机采集线程持续排空 V4L2，算法只取得最新帧，避免处理积压旧画面；
- 相机启动后回读实际宽、高、FPS 和 FourCC；不匹配时告警，`strict_settings=true` 可改为直接拒绝启动；
- OpenCV 优化开启，线程数由 `runtime.opencv_threads` 配置；
- ROI 在程序启动时检查，越界配置不再等到比赛运行中才报错；
- `STATION/TURNTABLE` 默认关闭，但保留配置接口。

本次无相机开发环境的合成图多轮基准：稳定颜色目标约 1.2～1.5 ms/帧，强制全 ROI 约
当前合成三圆与内置模板约 4.6 ms/帧。该数字只说明代码链路可实时运行，Jetson 实机结果必须用下面
的工具重新测量：

```bash
python3 tools/benchmark_detector.py --camera /dev/v4l/by-id/实际相机设备 \
  --mode COLOR --scene TURNTABLE --target 1 --iterations 300 --warmup 30

# 测量不使用跟踪小ROI时的最差颜色处理成本
python3 tools/benchmark_detector.py --camera /dev/v4l/by-id/实际相机设备 \
  --mode COLOR --scene TURNTABLE --target 1 --iterations 300 --warmup 30 \
  --disable-tracking

python3 tools/benchmark_detector.py --camera /dev/v4l/by-id/实际相机设备 \
  --mode RING --scene ROUGH --target 2 --iterations 300 --warmup 30
```

重点记录输出中的 `camera_settings`、`detect_ms.p95`、`read_ms.p95`、`wall_fps` 和
`detections`。性能合格但 `detections` 不稳定仍然不能采用。

第一轮实机验收建议逐项执行：

| 画面条件 | 运行方式 | 初始目标 |
|---|---|---|
| 每种目标颜色位于转盘 ROI | `COLOR`，颜色 1～6 分别测 | `detection_rate >= 0.98` |
| ROI 中无目标颜色 | 对相应 `COLOR` 测试 | `detection_rate == 0` |
| 目标静止 | 连续 300 帧 | `detect_ms.p95 < 16.7 ms`，坐标极差小于抓取容差 |
| 目标随转盘移动 | 连续 300 帧 | 不能因小 ROI 跟踪而漏掉；自动回退后应恢复 |
| 粗加工区三个环 | `RING` 1～3 分别测 | 每个环稳定检出，中心无跳环 |
| 暂存区已有同色物料 | `STACK` 逐颜色、逐环测 | 只在指定环 ROI 返回目标 |

这些是第一轮工程目标，不是脱离实拍数据的准确率承诺。若 60 FPS 相机输入下要做到逐帧处理，
识别 `p95` 应低于 16.7 ms；若只要求 30 FPS，则应低于 33.3 ms。

### 9.2 当前瓶颈优先级

**已修复：串口阻塞。** Maix、MCU 两路串口均使用 `timeout=0`，不会把无数据等待叠加到相机循环。

**已修复：许可有效性。** `GRASP_READY` 携带 `valid_ms`；Jetson 在收到同序号 `EXEC` 前持续复检，丢失、失稳或过期会发送 `GRASP_REVOKED`。

**已修复：端到端编码。** `TASK_PLAN` 的队列元素拆成独立字段，错误帧不再含空字段；发送编码异常会记录 `COORDINATOR_TX_ERROR` 而不退出服务。

**P1：相机与处理解耦。** 建议使用采集线程持续取最新帧，队列长度为 1，识别永远处理最新画面，不积压旧帧。

**部分完成：坐标与任务状态。** 自动颜色队列已在 `DONE OK` 后提交推进，超时/失败不推进；毫米坐标和放置环自动计划仍待实机方案确认。

**已修复：状态转换校验。** `REQ` 只在 `TASK_READY` 接受；`WAIT_EXEC/WAIT_DONE` 拒绝覆盖；`ABORT` 返回 `ABORTED`，错误序号和错误状态可观测。

**P1：性能观测。** 增加实际采集 FPS、处理 FPS、单帧耗时、REQ 到 READY 延迟、丢帧数和串口 CRC 错误计数。没有这些指标无法判断“实时”。

### 9.3 相机与算法建议

- 先测试 1280×720 MJPG 60 FPS；若 `camera_settings` 未协商到 60 FPS，或 Jetson 的
  `wall_fps`、温度和功耗不稳定，再把配置退回 30 FPS 对比；
- ROI 要尽量小，当前颜色与圆环检测都会随 ROI 面积增加而变慢；
- 只有收到 `REQ` 时才做目标算法，当前代码已经基本符合这一点；
- 不要为追求帧率盲目减少稳定帧，先测转盘停止到 `GRASP_READY` 的真实延迟；
- 锁定曝光、增益、白平衡，垂直向下补光，避免 HSV 阈值随时间漂移；
- 相机缓冲应保持最新帧策略，当前设置 `CAP_PROP_BUFFERSIZE=1`，但不同 V4L2 驱动不保证严格生效。

## 10. 上场前检查表

- [x] 修复 `TASK_PLAN` 含逗号字段导致的编码崩溃；
- [x] 两路串口改为非阻塞读取；实机仍需测循环频率；
- [x] 正式 `GRASP_READY` 具备有效期、持续复检和 `EXEC` 握手；
- [ ] 电控完成图像坐标到夹爪/底盘动作的换算与安全窗口判断；
- [x] 自动任务队列只在 `DONE OK` 后推进，超时、失败和重试不推进；
- [x] 修复 `WAIT_EXEC/WAIT_DONE` 新 `REQ` 覆盖旧请求的问题；
- [ ] 电控能够解析完整任务码中的颜色—环号对应关系；
- [ ] 三个场景 ROI 都基于实机 1280×720 画面重画；
- [ ] 六色 HSV 在正常光、阴影、反光、转盘运动条件下测试；
- [ ] 决定使用 `PX` 还是 `MM`，两端对单位理解完全一致；
- [ ] 相机实际协商到 MJPG、目标分辨率和目标 FPS；
- [ ] Maix 串口已开启，启动顺序不会丢失唯一一次任务码发送；
- [ ] 电控对每次 `ACCEPTED/READY/ERROR/DONE_ACK` 有超时和恢复策略；
- [ ] 停掉所有 `live/pickup/bridge` 测试进程后再启 `coordinator`；
- [ ] 完成有硬件端到端压力测试，而不只依赖当前单元测试。

## 11. 最终建议的精简边界

Jetson 正式服务只需要暴露四类业务输入输出：

```text
输入：MAIX_QR
输入：REQ PICK / PLACE / STACK
输入：EXEC / DONE / ABORT
输出：TASK_PLAN / GRASP_READY / ALIGN_READY / ERROR
```

内部保留 `COLOR + RING + STACK + 稳定门 + 坐标换算`。`STATION`、转盘外圆检测、单图/实时显示、bridge 都作为调试工具留在仓库即可，不要放进正式每帧路径。这样既不丢失后续定位能力，也能让比赛运行链路保持清晰。
