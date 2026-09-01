# Jetson HSV 颜色识别 V1 调试手册

## 1. 当前阶段边界

当前只用电脑屏幕色块验证程序链路。配置中的 HSV、相机自动档和 ROI 都不是比赛最终参数：

```text
TEMPORARY SCREEN TEST PARAMETER
NEED RECALIBRATION WITH REAL MATERIAL
```

现在只确认以下事情：相机能取流、ROI 正确、六个 Mask 正确、轮廓和全局中心正确、三帧稳定、
输出及串口协议没有断。不要为了屏幕效果添加 Lab、自动阈值或复杂分类器。

## 2. 当前处理流程

```text
相机取一帧
→ 裁剪 scenes.<场景>.object_roi
→ BGR 转 HSV
→ 当前目标颜色 cv2.inRange
→ 轻量开/闭运算
→ cv2.findContours
→ min_area/max_area
→ 选择面积最大的有效轮廓
→ cv2.moments 计算中心
→ 加回 ROI 左上角得到原图坐标
→ 连续3帧颜色相同且相邻中心位移不超过阈值
→ 稳定后使用“外接框+边距”动态ROI，失败时同帧回退固定ROI
→ LOST / UNSTABLE / STABLE
```

黑色只使用 `V <= v_max` 和可选 S 范围，不判断 H。

## 3. 启动命令

### 3.1 可视化识别

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene PAPER_TEST --target red \
  --show-mask --show-fps
```

只会出现：

- `Detection`：原图、固定 ROI、目标框、中心、颜色和状态；
- `Mask`：当前指定颜色的 Mask。

把 `--target red` 依次替换为：

```text
red  yellow  blue  green  black  light_blue
```

也可以继续使用协议编号 `1 2 3 4 5 6`。

### 3.2 查看鼠标位置 HSV

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene PAPER_TEST --target light_blue \
  --show-mask --show-hsv
```

移动鼠标到色块内部，底部显示该区域的 `HSV05/HSV50/HSV95`。这里只用于手工确定临时阈值，
程序不会自动修改配置。

### 3.3 无界面与性能

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene PAPER_TEST --target red \
  --headless --show-fps --stats-every 60
```

默认只输出状态变化。需要每隔 `--print-every` 帧输出一次检测结果时增加：

```bash
--debug --print-every 30
```

独立基准：

```bash
python3 tools/benchmark_detector.py \
  --camera /dev/video0 \
  --mode COLOR --scene PAPER_TEST --target red \
  --iterations 300 --warmup 30
```

### 3.4 静态图片

```bash
python3 -m jetson_recognition.run image samples/red.jpg \
  --mode COLOR --scene PAPER_TEST --target red \
  --show --show-mask
```

## 4. 只调一个配置文件

编辑 `config/jetson.json`。

### 4.1 固定 ROI

当前屏幕测试 ROI：

```json
"PAPER_TEST": {
  "object_roi": {"x": 160, "y": 90, "width": 960, "height": 540},
  "min_area": 300,
  "max_area": 500000
}
```

目标必须完整位于蓝色 ROI 框内。中心输出自动换算为原图坐标：

```text
global_x = roi.x + local_x
global_y = roi.y + local_y
```

### 4.2 HSV

彩色配置示例：

```json
"4": {
  "name": "GREEN",
  "hsv_ranges": [
    {"h_min": 45, "h_max": 85, "s_min": 80, "s_max": 255,
     "v_min": 40, "v_max": 255}
  ]
}
```

红色有两段 `hsv_ranges`，分别覆盖 H 低端和 170～179。黑色配置为：

```json
"black_threshold": {"v_max": 80, "s_min": 0, "s_max": 255}
```

屏幕调参顺序：

1. 打开 `--show-hsv --show-mask`；
2. 鼠标放色块内部，记录 H/S/V 的 5%～95%范围；
3. H 两边各留少量余量；
4. 用 `s_min/v_min` 排除灰暗背景；
5. 检查无目标时 Mask 是否基本全黑；
6. 每次只改当前一种颜色，不同时改形态学和面积。

蓝色与浅蓝色当前直接使用独立 H 范围。只有真实物料证明 HSV 无法分开时，才考虑 TODO 中的
可选 Lab 二次判断。

### 4.3 形态学和面积

```json
"morphology": {
  "kernel_size": 3,
  "open_iterations": 1,
  "close_iterations": 1
},
"geometry": {
  "min_area": 300,
  "max_area": 500000
}
```

调节原则：

- Mask 有散点：先确认 HSV，再考虑把 `open_iterations` 从 1 改为 2；
- 色块内部小孔：把 `close_iterations` 从 1 改为 2；
- 小噪声形成框：提高 `min_area`；
- 真目标太小被拒绝：降低 `min_area`；
- 不调长宽比、圆度、实心度、多边形或 Hu 矩。

### 4.4 三帧稳定

```json
"stability": {
  "stable_frames": 3,
  "max_center_delta_px": 6.0
}
```

- 第一、二帧：`UNSTABLE`；
- 连续三帧相邻中心位移均不超过 6 px：`STABLE`；
- 中心突然跳动：重新从一帧累计；
- 目标消失：立即 `LOST`。

### 4.5 稳定后动态 ROI

```json
"dynamic_roi": {
  "enabled": true,
  "margin_px": 80,
  "activate_after": "STABLE",
  "fallback": "FIXED_ROI_SAME_FRAME"
}
```

- 前三帧和未识别状态只处理固定 ROI；
- 达到 `STABLE` 后，下一帧处理上次目标外接框向四周扩展 80 px 的区域；
- 目标触碰动态 ROI 边界或动态 ROI 未命中，会在同一帧重跑固定 ROI；
- 调试画面蓝框是固定 ROI，浅蓝细框是动态 ROI；
- `PERF.roi_mode` 显示 `FIXED_ROI` 或 `DYNAMIC_ROI`。

边距至少应覆盖“目标单帧最大位移＋边缘抖动”。频繁回退时增加到 100～140；目标基本静止时可
降到 50～80。动态 ROI 只降低检测耗时，不能把相机本身的 30 FPS 变成 60 FPS。

## 5. 输出

控制台颜色结果示例：

```json
{
  "state": "STABLE",
  "kind": "COLOR",
  "target_id": "1",
  "color": "RED",
  "center_x": 550.0,
  "center_y": 350.0,
  "confidence": 1.0,
  "stable_frames": 3
}
```

`confidence` 当前只是通过面积门槛后的简单面积分数，不是分类概率。串口帧名称和既有字段顺序
没有改变。

## 6. 相机、光照与锁参

### 6.1 当前到底是不是默认参数

当前请求格式是 `1280×720 / MJPG / 60 FPS / buffer_size=1`，并启用一个只保存最新帧的取流线程。
相机后端默认是 `gstreamer_nv`：使用连续 MJPEG 对应的 `jpegparse ! nvv4l2decoder mjpeg=1`，
只把 MJPEG 解码和像素格式转换交给 Jetson 硬件，HSV、Mask、
轮廓及稳定判断仍使用普通 CPU OpenCV。
当前活动档是 `camera_default_auto`：

```json
"white_balance_automatic": 1,
"auto_exposure": 3,
"focus_automatic_continuous": 1,
"saturation": 60
```

因为该档有 `_replace_base=true`，配置里旧的手动温度、曝光、焦距、锐度不会参与当前运行。
所以目前白平衡、曝光和焦距都是相机自动控制，不是固定值。启动时 `CAMERA_READY` 会同时输出：

- `settings`：OpenCV 实际协商到的宽高、FourCC、FPS；
- `startup_controls`：打开相机时回读的自动开关、温度、曝光、增益和焦距。

自动模式下后三项会继续变化，`startup_controls` 只是启动瞬间值。实时查看：

```bash
v4l2-ctl -d /dev/video0 \
  --get-ctrl=white_balance_automatic,white_balance_temperature,auto_exposure,exposure_time_absolute,gain,focus_automatic_continuous,focus_absolute
```

### 6.2 当前屏幕测试遇到光照变化

1. 先让自动曝光/白平衡稳定 5～10 秒，不移动相机；
2. 使用 `--show-hsv --show-mask`，记录色块内部 `HSV05/HSV50/HSV95`；
3. 判断问题发生在哪一级：
   - 原图明显变色或亮暗漂移：先处理相机/光源，不改 HSV；
   - 原图正常但 Mask 全黑：放宽当前颜色 H，或降低 `s_min/v_min`；
   - 背景大量变白：缩窄 H，或提高 `s_min/v_min`；
   - 黑色漏检：适当提高 `black_threshold.v_max`；
   - 阴影被当黑色：降低 `v_max`，真实物料阶段仍不行再考虑局部对比升级。
4. 每次只改一种颜色的一类参数，然后同时测试“有目标”和“无目标”；
5. 不要为白天、晚上、台灯分别保存大量 HSV。屏幕阶段只验证流程。

### 6.3 真实物料与补光到位后的标准流程

1. 固定相机高度、俯视角、工作距离、ROI 和补光位置；
2. 放入真实背景及中性灰卡，让自动曝光、白平衡、对焦收敛 10 秒；
3. 确认画面不过曝、暗部有区分、运动目标不拖影；
4. 自动值稳定后生成并启用锁定档：

```bash
python3 tools/capture_camera_profile.py \
  --profile competition_locked \
  --settle-seconds 10 \
  --activate --headless
```

5. 确认终端出现 `CAMERA_PROFILE_CAPTURED` 和 `verified_controls`；
6. 回读必须为白平衡自动 `0`、曝光模式 `1`、连续对焦 `0`；
7. 使用这个锁定档重新测六种真实物料 HSV；
8. 依次测试暗处、亮处、ROI 四角、运动、停止和无目标背景。

如果自动曝光为了补偿暗环境把曝光拉得过长，应先增加补光，再锁较短曝光；单纯提高 gain 会增加
噪声和 Mask 散点。比赛运行时不要保持自动白平衡，否则大面积背景变化仍会改变色温。

### 6.4 30 FPS 分层排查

日志中如果出现 `detect_ms≈7`、`capture_fps≈29`，可以直接确定 HSV 检测不是瓶颈：动态 ROI
发生在 MJPEG 解码之后，也不能解决相机读取/解码只有30帧的问题。项目默认改用 Jetson
GStreamer 硬件 MJPEG 解码后，再运行：

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene PAPER_TEST --target red \
  --headless --show-fps --stats-every 60
```

`CAMERA_READY.settings.backend` 应为 `gstreamer_nv`。比较旧 OpenCV 后端时使用：

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene PAPER_TEST --target red \
  --camera-backend opencv_v4l2 \
  --headless --show-fps --stats-every 60
```

该参数只在本次启动生效。若硬件管线协商失败，程序输出 `CAMERA_BACKEND_WARNING` 后自动回退
`opencv_v4l2`，不会中断识别；`CAMERA_READY.settings.backend` 表示真正使用的后端，
`requested_backend` 表示原请求。同时检查：

```bash
gst-inspect-1.0 nvv4l2decoder
gst-inspect-1.0 nvvidconv
```

按输出判断：

| 现象 | 结论 | 优先处理 |
|---|---|---|
| `settings.fps` 就是 30 | OpenCV/驱动只协商到30 | 查格式，确认 MJPG 1280×720@60 |
| `settings.fps=60`，`capture_fps≈30` | 相机实际只出30或曝光/USB限制 | 查曝光、USB口、线材、集线器和裸流 |
| `capture_fps≈60`，但总 `fps≈30` | Python循环、显示或检测耗时限制 | 看 `detect_ms`，关闭GUI/HSV探针 |
| `detect_ms>16.7` | 处理赶不上60FPS | 缩固定ROI、确认动态ROI、减少形态学次数 |
| `detect_ms<5` 且 `capture_fps≈30` | 算法不是瓶颈 | 调 ROI 不会突破30，处理相机端 |

检查相机格式和裸流：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext

v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=1280,height=720,pixelformat=MJPG \
  --set-parm=60

v4l2-ctl -d /dev/video0 \
  --stream-mmap=4 --stream-count=600 --stream-to=/dev/null
```

如果裸流稳定 60，而 Python 的 `capture_fps` 仍只有 30，确认配置中的 `threaded_capture=true`，并
关闭其他占用摄像头的软件。这个线程只负责持续取走旧帧、保留最新帧，不参与视觉算法。
无头模式下单独使用 `--show-fps` 只收集轻量性能数据，不再生成轮廓绘图数据；只有 `--debug`、
`--show-mask` 或图形界面才会准备完整调试轮廓。

真实物料到位后的正确顺序：

1. 固定相机高度、角度和 ROI；
2. 安装稳定补光；
3. 再固定曝光、白平衡和焦距；
4. 最后重新测六色 HSV 和面积。

## 7. 明确禁用的旧机制

- `color_classifier` 和全部 Lab 原型；
- 第一/第二颜色距离 margin；
- 黑色 Lab/L/V 局部亮度环；
- 颜色原型自动标定。

旧 `pickup` 速度/安全区/最终复检门控在配置中 `enabled=false`，协调器 `PICK` 已直接使用三帧
COLOR 稳定结果，并继续发送原协议的 `GRASP_READY`。

旧命令 `tools/calibrate_color_prototype.py` 只保留兼容提示，不再写配置。

未来升级只留决策接口，不在 V1 实现：

- 蓝/浅蓝真实物料确实重叠：TODO 可选 Lab 二次分类；
- 黑色与阴影真实环境确实混淆：TODO 可选局部亮度对比；
- 光照漂移：优先固定曝光、白平衡和补光，而不是在线自适应。

## 8. 基础验证

```bash
python3 -m json.tool config/jetson.json >/dev/null
python3 -m compileall -q jetson_recognition tools tests
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```
