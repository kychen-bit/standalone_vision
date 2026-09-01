# Jetson Orin Nano 俯视传统视觉

当前默认颜色算法是刻意简化的 `HSV_CONTOUR_V1`：

```text
USB相机 → 固定ROI → HSV inRange → 开/闭运算
→ findContours → 面积过滤 → moments质心 → 连续5帧稳定
→ 可选动态ROI（漏检时同帧回退固定ROI）
```

默认颜色路径不使用 Lab、颜色原型、距离分类、局部亮度环、Kalman、CUDA、
深度学习或自动标定。相机设备、MaixCAM Pro 任务通信和下位机结果协议仍沿用现有接口。

> 当前 HSV 与相机设置只用于电脑屏幕色块验证。
> `TEMPORARY SCREEN TEST PARAMETER - NEED RECALIBRATION WITH REAL MATERIAL`。
> 拿到真实物料、固定补光和赛场安装后必须重新确定曝光、白平衡、ROI 与 HSV。

## 最快启动

安装依赖：

```bash
python3 -m pip install -r requirements-jetson.txt
```

红色色块可视化测试：

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene PAPER_TEST --target red \
  --show-mask --show-fps
```

程序只显示两个窗口：

- `Detection`：固定 ROI、外接框、颜色名、中心和稳定状态；
- `Mask`：当前 `--target` 对应的二值 Mask。

无界面性能测试：

```bash
python3 -m jetson_recognition.run live \
  --mode COLOR --scene PAPER_TEST --target red \
  --headless --show-fps --stats-every 60
```

Jetson 默认通过 `gstreamer_nv` 对 USB 相机 MJPEG 做硬件解码；识别算法本身仍是 CPU OpenCV。
对比原生 OpenCV V4L2 取流时增加 `--camera-backend opencv_v4l2`。

默认只在 `LOST / UNSTABLE / STABLE` 状态变化时输出。需要周期性检测详情时增加 `--debug`。

颜色既可以写编号，也可以写名称：

| 编号 | 名称 |
|---:|---|
| 1 | `red` |
| 2 | `yellow` |
| 3 | `blue` |
| 4 | `green` |
| 5 | `black` |
| 6 | `light_blue` |

## 配置

所有第一版颜色参数都在 [config/jetson.json](config/jetson.json)：

- `camera`：设备、分辨率、帧率及可选 V4L2 档；
- `scenes.*.object_roi`：`x/y/width/height` 固定 ROI；
- `color_detection.morphology`：核尺寸、开/闭次数；
- `color_detection.geometry`：默认面积范围；
- `color_detection.dynamic_roi`：稳定后局部 ROI 开关和外扩边距；
- `scenes.*.min_area/max_area`：场景面积覆盖值；
- `stability`：连续帧数和相邻中心最大位移；
- `colors`：五种彩色 HSV 范围及黑色简单 V/S 阈值。

详细调试顺序和命令见 [JETSON_RECOGNITION_TEST_GUIDE.md](JETSON_RECOGNITION_TEST_GUIDE.md)。

## 通信入口

MaixCAM Pro 与下位机协调入口保持不变：

```bash
python3 -m jetson_recognition.run coordinator \
  --maix-serial /dev/serial/by-id/MAIX_DEVICE \
  --mcu-serial /dev/serial/by-id/MCU_DEVICE
```

颜色结果仍使用既有 `VISION_RESULT`/协调器协议字段，原有颜色编号不变；控制台 JSON 额外提供
`color`、`center_x`、`center_y` 便于人工排查。

## 验证

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
python3 -m compileall -q jetson_recognition tools tests
```

主要结构保持现状，没有为了形式重写工程：

- `camera.py`：相机和 V4L2；
- `detectors.py`：HSV、Mask、轮廓、面积、质心；
- `stability.py`：简单五帧颜色稳定以及旧辅助模式所需稳定窗口；
- `run.py`：命令行、显示和输出；
- `output.py` / `coordinator.py`：已有通信协议。
