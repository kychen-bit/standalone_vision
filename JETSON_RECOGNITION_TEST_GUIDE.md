# Jetson 俯视识别：测试命令、调参与启动

本文只说明三件事：各识别模式怎样运行、怎样调参数、整个识别系统怎样启动。

## 1. 已适配的 DECXIN 相机配置

`config/jetson.json` 已按 `DECXIN_controls.txt` 和 `DECXIN_all.txt` 适配：

```text
图像格式：1280×720，MJPG
请求帧率：60 FPS（相机能力表支持；--all 采集时的状态是 30 FPS）
防频闪：50 Hz
自动曝光：关闭，手动曝光初值 156
自动白平衡：关闭，色温初值 4600
连续自动对焦：关闭，焦距初值 414
增益：0
饱和度：60
锐度：2000
后台取帧：开启，只处理最新帧
```

每次打开相机前，程序会用 `v4l2-ctl` 自动应用 `camera.v4l2_controls`。任意控件设置失败时
默认拒绝启动，防止在未知自动参数下继续调 HSV。

以上手动值是从相机当前状态取得的调试起点，不代表最终最优值。加入垂直补光后，通常应优先
降低 `exposure_time_absolute`，直到转盘运动不拖影且图像仍足够亮；亮度不足再小幅增加 `gain`。

启动后如出现 `CAMERA_SETTINGS_WARNING`，重点看实际 FPS。如果一直只能协商到 30 FPS，把
`camera.fps` 改成 30，再按 33.3 ms 的单帧预算测试。

## 2. 命令格式和公共参数

所有命令先进入项目目录：

```bash
cd /home/ysu/standalone_vision
```

基本格式：

```text
python3 -m jetson_recognition.run [全局参数] <image|live|pickup|coordinator> [模式参数]
```

公共参数：

| 参数 | 含义 |
|---|---|
| `--config PATH` | 指定配置文件；默认 `config/jetson.json`，必须放在入口名称前 |
| `--mode` | `COLOR/RING/STACK/STATION/TURNTABLE` |
| `--scene` | `TURNTABLE/ROUGH/STORAGE` |
| `--target` | COLOR/STACK 时是颜色号；RING 时是圆环号 |
| `--ring` | 只供 STACK 使用，表示在哪个圆环 ROI 内搜索 |
| `--camera` | 临时覆盖配置中的相机设备；配置已正确时省略 |
| `--headless` | 不打开 GUI，SSH 运行时使用 |
| `--print-every N` | TRACKING 每 N 帧打印一次，不改变识别频率 |

颜色编号：

```text
1=红色  2=黄色  3=蓝色  4=绿色  5=黑色  6=浅蓝色
```

场景：

```text
TURNTABLE = 原料转盘
ROUGH     = 粗加工区
STORAGE   = 暂存区
```

## 3. COLOR：指定颜色物料

### 测试命令

保存图片测试：

```bash
python3 -m jetson_recognition.run image samples/red.jpg \
  --mode COLOR --scene TURNTABLE --target 1 \
  --save result_red.jpg
```

实时窗口测试：

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene TURNTABLE --target 1
```

SSH 测试：

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene TURNTABLE --target 1 --headless
```

性能和检出率测试：

```bash
python3 tools/benchmark_detector.py --camera /dev/video0 \
  --mode COLOR --scene TURNTABLE --target 1 \
  --iterations 300 --warmup 30
```

完整 ROI 最差耗时：

```bash
python3 tools/benchmark_detector.py --camera /dev/video0 \
  --mode COLOR --scene TURNTABLE --target 1 \
  --iterations 300 --warmup 30 --disable-tracking
```

把 `--target` 依次改成 1～6，六种颜色必须分别测试。

### 一轮批次入口（推荐）

现场调色优先用批次工具，它一次给出这一轮三块物料各自是什么颜色：

```bash
python3 tools/test_turntable_color_live.py \
  --task-code 234+123+432+231 --batch 1 --target 3
```

输出中的 `round_batch` 是本轮判定结果，`batch_agreement` / `batch_stable` 是一致性，
`materials[]` 给出每块的 `core_hsv`、`distance`、`mass_ratio`、`solidity` 等实测值，
`illumination` 给出白场与增益诊断。

常用开关与快捷键：

| 项目 | 作用 |
|---|---|
| `--fill-light` / `--no-fill-light` | 覆盖 `lighting.fill_light`，切换有/无补光方案 |
| `--no-batch` | 退回逐色号独立掩码的旧路径（A/B 用） |
| `--no-illumination` | 关闭白场归一化（A/B 用） |
| 鼠标 + 按 `1`~`6` | 把鼠标下那块物料记成该色号的原型（颜色标定） |
| 按 `w` | 把已标定的原型写回 `config/jetson.json`（会备份） |

判色不准时先看 `illumination.exposure_hint`：`RAISE_EXPOSURE` 表示这张图本来就欠曝，
要加曝光而不是调阈值。详见 [环境光鲁棒色块识别](docs/环境光鲁棒色块识别.md)。

### 调参

按以下顺序一次只改一类参数：

1. `scenes.TURNTABLE.object_roi`
   - `[X,Y,W,H]` 只覆盖物料实际活动区域；
   - 太大增加背景误报和耗时，太小会截断目标。
2. `colors.<颜色号>.ranges`
   - 在最终补光、手动曝光、手动白平衡下采样 HSV；
   - 红色保留 0 附近和 180 附近两段；
   - 黑色重点调 V 上限并测试阴影；
   - 蓝色和浅蓝色重点分开 H、S 范围。
3. `min_object_area_px/max_object_area_px`
   - 从最小和最大真实物料轮廓面积取得；
   - 小碎片误报时提高最小值，真目标被拒绝时放宽。
4. `min_aspect/min_solidity/min_fill`
   - 细长噪声多时提高 `min_aspect`；
   - 破碎轮廓多时提高 `min_solidity`；
   - 线条/空心噪声多时提高 `min_fill`。
5. `tracking.margin_px`
   - 应覆盖物料单帧最大位移、物料半尺寸和安全余量；
   - 太小会频繁回退完整 ROI，太大则跟踪加速效果下降。

每种颜色都要测试：ROI 中心、四边、转盘运动、转盘停止、阴影、高光和无该颜色背景。

benchmark 重点看：

```text
detection_rate：有目标时建议 >= 0.98，无目标时应为 0
detect_ms.p95：60 FPS 时应 < 16.7 ms；30 FPS 时应 < 33.3 ms
wall_fps：应接近相机实际 FPS
camera_settings：确认实际分辨率、FourCC 和 FPS
```

## 4. RING：指定圆环中心

### 测试命令

粗加工区 1、2、3 号环：

```bash
python3 -m jetson_recognition.run live \
  --mode RING --scene ROUGH --target 1 --headless

python3 -m jetson_recognition.run live \
  --mode RING --scene ROUGH --target 2 --headless

python3 -m jetson_recognition.run live \
  --mode RING --scene ROUGH --target 3 --headless
```

暂存区将 `ROUGH` 改为 `STORAGE`。

性能测试：

```bash
python3 tools/benchmark_detector.py --camera /dev/video0 \
  --mode RING --scene ROUGH --target 2 \
  --iterations 300 --warmup 30
```

### 调参

1. `scenes.<场景>.ring_rois.<环号>`
   - 每个 ROI 只包含一个圆环和少量边界；
   - 先调三个 ROI，再调霍夫圆参数。
2. `expected_radius_px`
   - 使用最终观察高度的实拍图测圆环像素半径。
3. `min_radius_px/max_radius_px`
   - 覆盖相机姿态和工位误差造成的半径变化，不要设置过宽。
4. `processing_scale`
   - 先用 1.0 验证准确，再改 0.75 比较速度；
   - 当前是 0.75，漏检或中心抖动时先恢复 1.0。
5. `center_threshold`
   - 降低会增加召回，也增加假圆；提高会减少误检，也可能漏检。
6. `edge_threshold`
   - 背景假边缘多时提高，真实圆边缘弱时降低。

测试三个环时必须包含：空环、相邻环同时出现、物料部分遮挡、补光反光和工位轻微偏移。

## 5. STACK：指定环内的同色物料

### 测试命令

暂存区 2 号环内找黑色物料：

```bash
python3 -m jetson_recognition.run live \
  --mode STACK --scene STORAGE --target 5 --ring 2 --headless
```

性能测试：

```bash
python3 tools/benchmark_detector.py --camera /dev/video0 \
  --mode STACK --scene STORAGE --target 5 --ring 2 \
  --iterations 300 --warmup 30
```

需要测试六种颜色 × 三个环，即修改 `--target 1..6` 和 `--ring 1..3`。

### 调参

STACK 没有独立 HSV，它复用：

```text
colors.<颜色号>.ranges
color_detection
scenes.STORAGE.ring_rois.<环号>
```

调参顺序：

1. 先保证相同颜色在 COLOR 模式下稳定识别；
2. 再检查对应 `STORAGE.ring_rois` 是否完整包含物料；
3. 如果环线、数字或阴影被识别成黑色，再收紧黑色 V/S、面积和形状；
4. 测试物料偏心、旋转、高光、邻环同色干扰和部分遮挡；
5. 记录静止坐标极差，必须小于允许码垛误差。

## 6. pickup：转盘抓取许可

pickup 是独立入口，不写 `--mode`。

### 标定抓取窗口

```bash
python3 tools/capture_zone_calibrator.py \
  --camera /dev/video0 --target 1 --scene TURNTABLE
```

鼠标拖出黄色外框，绿色框是内缩安全区，按 `s` 保存。

### 测试命令

```bash
python3 -m jetson_recognition.run pickup \
  --target 1 --scene TURNTABLE \
  --headless --timeout 20 --exit-on-ready
```

临时覆盖抓取窗口：

```bash
python3 -m jetson_recognition.run pickup \
  --target 1 --scene TURNTABLE \
  --capture-zone X Y W H \
  --headless --timeout 20 --exit-on-ready
```

### 调参

| 参数 | 如何取得 |
|---|---|
| `capture_zone_px` | 根据夹爪能够垂直抓取的真实投影区域标定，不是整个转盘 |
| `capture_margin_px` | 根据夹爪宽度和机械误差给外框内缩安全余量 |
| `max_target_speed_px_s` | 分别记录转盘运动和完全停止时的 `speed_px_s`，设在两者之间 |
| `stable_spread_px` | 记录静止目标坐标极差，阈值略大于正常抖动 |
| `stable_frames` | 越大越稳、越慢；60 FPS 下 5 帧约 83 ms |
| `low_speed_hold_frames` | 防止减速瞬间误判停止 |
| `final_recheck_frames` | 稳定后的额外确认帧 |
| `min_confidence` | 低于真目标置信度下界，高于常见误报 |
| `search_timeout_s` | 至少覆盖转盘一圈并留余量 |

必须测试以下过程：

```text
窗口外经过 → 窗口内运动 → 减速 → 停止 → FINAL_CHECK → GRASP_READY
```

还要测试错误颜色、目标丢失、停止后再次运动和超时。

## 7. 可选辅助模式

当前配置只启用：

```json
"enabled_modes": ["COLOR", "RING", "STACK"]
```

导航全部由电控负责时，不需要运行下面两项。

### STATION

先把 `STATION` 加入 `enabled_modes`，再运行：

```bash
python3 -m jetson_recognition.run live \
  --mode STATION --scene ROUGH --headless
```

它根据多个圆环计算工位整体平移和偏航。参数是 `station_reference_px/mm` 和
`max_station_residual`。

### TURNTABLE

先把 `TURNTABLE` 加入 `enabled_modes`，再运行：

```bash
python3 -m jetson_recognition.run live \
  --mode TURNTABLE --scene TURNTABLE --headless
```

它只检测转盘外圆中心，不参与当前 COLOR/pickup 主流程。

## 8. 整个识别系统如何启动

### 8.1 启动前检查

```bash
cd /home/ysu/standalone_vision
python3 -m unittest discover -s tests -v
python3 -m compileall -q jetson_recognition tests tools
```

确认没有其他进程占用相机：

```bash
fuser /dev/video0
```

确认配置可解析：

```bash
python3 -m json.tool config/jetson.json >/dev/null
```

### 8.2 当前识别调试启动

识别调试阶段一次运行一种视觉任务，不需要同时启动多个进程：

```bash
# 原料颜色
python3 -m jetson_recognition.run live \
  --mode COLOR --scene TURNTABLE --target 1 --headless

# 粗加工圆环
python3 -m jetson_recognition.run live \
  --mode RING --scene ROUGH --target 1 --headless

# 暂存区码垛目标
python3 -m jetson_recognition.run live \
  --mode STACK --scene STORAGE --target 1 --ring 1 --headless

# 转盘抓取许可
python3 -m jetson_recognition.run pickup \
  --target 1 --scene TURNTABLE --headless --timeout 20
```

不要同时运行多个 `live/pickup/benchmark`，否则会争抢同一相机。

### 8.3 正式完整识别服务

正式系统由电控按当前区域请求 PICK、PLACE 或 STACK，唯一入口是：

```bash
cd /home/ysu/standalone_vision
python3 -m jetson_recognition.run --config config/jetson.json coordinator
```

它会自动加载：

```text
DECXIN 手动相机控件
最新帧相机线程
COLOR / RING / STACK
多帧稳定判决
任务码解析和电控请求协调
```

正式启动前必须填写 `coordinator.maix_serial`、`coordinator.mcu_serial` 和
`coordinator.pickup.capture_zone_px`。串口尚未联调时不要运行 coordinator，继续使用 8.2
中的单项识别命令。

启动顺序：

```text
固定相机和补光
→ 启动 Jetson coordinator
→ 确认 COORDINATOR_READY
→ 启动 Maix 扫码
→ 电控发送 START/REQ
→ Jetson 按请求运行 COLOR/RING/STACK
```
