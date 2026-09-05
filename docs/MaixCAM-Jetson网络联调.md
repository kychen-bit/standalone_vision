# MaixCAM Pro 与 Jetson USB 虚拟网卡通信测试

本文只验证一条最小链路：

```text
MaixCAM Pro 识别任务二维码
        ↓ TCP（USB 虚拟网卡）
Jetson 接收并校验 MAIX_QR 数据帧
```

这一步不会启动 Jetson 的色块、圆环识别，也不会向电控发送数据。先确认 MaixCAM 到 Jetson 的网络收包正常，再把 TCP 输入接入完整协同器。

## 1. 当前已经确认的网络参数

```text
MaixCAM Pro：10.33.117.1
Jetson 网卡：Sipeed RNDIS接口（实际名称可能为usb0或usb2）
Jetson 地址：10.33.117.105/24（静态）
TCP 端口：5000
```

MaixCAM Pro 作为 TCP 客户端，Jetson 作为 TCP 服务端。Maix 发送的仍是项目现有的 `@MAIX_QR,...*CRC16` 协议，没有改变应用层数据格式。

Jetson 已创建 NetworkManager 连接`maixcam-usb-static`，配置为`ipv4.method=manual`、自动连接，地址固定为`10.33.117.105/24`。MaixCAM在不同开机顺序下可能枚举为`usb0`或`usb2`，而且还会同时出现一块`10.33.116.x`网卡，因此不要只凭接口名称修改。

项目工具会按Sipeed USB VID/PID和`rndis_host`驱动找到正确的`10.33.117.x`接口，并把静态连接绑定到它：

```bash
python3 tools/configure_maix_network.py
```

看到`[NETWORK] ... uses 10.33.117.105/24`后再检查：

比赛部署建议安装一次固定命名规则：

```bash
cd /home/ysu/standalone_vision
sudo bash tools/install_maix_udev.sh
```

安装后拔插一次MaixCAM，再执行`python3 tools/configure_maix_network.py`。以后接口固定为`maixcam0`，NetworkManager的`maixcam-usb-static`会自动分配`10.33.117.105/24`。安装动作需要本机sudo密码，密码只在Jetson自己的终端输入。

```bash
ip -brief address
ping -c 3 10.33.117.1
```

应当看到某个`usb*`接口带有`10.33.117.105/24`地址，且ping没有丢包。

## 2. 先启动 Jetson 接收端

在 Jetson 终端中执行：

```bash
cd /home/ysu/standalone_vision
python3 -m jetson_recognition.run maix-net \
  --host 0.0.0.0 \
  --port 5000
```

正常时首先显示：

```json
{"state": "MAIX_TCP_LISTENING", "host": "0.0.0.0", "port": 5000}
```

这个终端要一直保持运行。按 `Ctrl+C` 才会停止。

如果只想快速验证端口是否正在监听，可以另开一个 Jetson 终端：

```bash
ss -ltnp | grep ':5000'
```

## 3. 在 MaixVision 中运行 Maix 程序

1. 打开 MaixVision，通过 `10.33.117.1` 连接 MaixCAM Pro。
2. 将本项目的 `maixcam_pro/main.py` 和 `maixcam_pro/config.py` 放入同一个 MaixVision 项目目录。两个文件必须一起更新，避免配置版本不一致。
3. 检查 `config.py` 中以下配置：

```python
SERIAL_ENABLED = False
TCP_ENABLED = True
TCP_SERVER_HOST = "10.33.117.105"
TCP_SERVER_PORT = 5000
TCP_HEARTBEAT_INTERVAL_S = 2.0
```

4. 在 MaixVision 中运行 `main.py`。

如果 Jetson 接收端已启动，Maix 控制台应看到：

```text
TCP_CONNECTED host=10.33.117.105 port=5000
MAIX_STANDALONE_QR_READY serial=False tcp=True tcp_server=10.33.117.105:5000 detector_mode=cpu
```

Jetson 终端同时应看到类似输出：

```json
{"state": "MAIX_TCP_CONNECTED", "peer": "10.33.117.1:xxxxx"}
```

这已经证明 TCP 连接建立成功，但还没有证明二维码数据发送成功。

## 4. 发送一个真实识别结果

给 MaixCAM 摄像头展示一个内容为以下字符串的二维码：

```text
156+123+516+231
```

二维码需要连续识别到 4 帧。稳定后，Maix 控制台会显示：

```text
QR_STABLE payload=156+123+516+231 x=... y=... side=...
QR_SENT payload=156+123+516+231
```

Jetson 终端应收到：

```json
{"state": "MAIX_TCP_FRAME", "frame": "MAIX_QR", "fields": ["156+123+516+231", "...", "...", "...", "4", "STABLE"], "peer": "10.33.117.1"}
```

看到 `MAIX_TCP_FRAME` 说明以下项目全部通过：

- USB 虚拟网卡连通；
- TCP 连接正常；
- Maix 二维码识别稳定；
- 完整数据帧已经到达 Jetson；
- Jetson 的 CRC16 校验通过；
- 字段能够被正常拆分。

为避免同一个二维码重复发送，程序对持续出现在画面中的同一任务只发送一次。想再次测试同一个二维码时，先把二维码移出画面至少 8 帧，再重新放回。

## 5. 常见问题

### 一条命令分层诊断

在项目根目录执行：

```bash
python3 tools/check_maix_connection.py
```

脚本只读检查，不会修改网络。它依次确认：Maix USB 虚拟网卡、Jetson
静态地址 `10.33.117.105/24`、Maix 地址 `10.33.117.1`、Jetson TCP 5000
监听以及已经建立的 TCP 会话。出现 `[FAIL]` 时执行其后显示的 `[NEXT]`
命令即可。

注意：`run_hardware_flow_test.py` 是一次性测试。显示
`HARDWARE_FLOW_OK` 并退出后，它会关闭内部协调器，TCP 5000 监听也随之
结束。此时 Maix 显示断开是正常现象；再次测试必须重新启动该脚本，正式
运行则应持续运行 coordinator。

### ping 不通 `10.33.117.1`

先不要调程序。检查 USB 线是否支持数据、Maix 是否完成启动，以及 Jetson 是否存在 Maix RNDIS 接口（可能叫 `usb0`、`usb2` 或 `maixcam0`）。重新插拔后再次执行：

```bash
ip -brief address
ping -c 3 10.33.117.1
```

如果列表里没有Sipeed USB虚拟网卡，说明设备尚未枚举，先检查Maix是否上电以及Type-C线是否为数据线。设备出现后执行：

```bash
nmcli connection show --active
python3 tools/configure_maix_network.py
```

不要在接口不存在时强行让程序绑定`10.33.117.105`。项目现在让服务端监听`0.0.0.0`，可以先启动等待；Maix客户端仍连接`10.33.117.105`。

### Maix 显示 `TCP_CONNECT_FAILED`

依次检查：

1. Jetson 的 `maix-net` 是否先启动；
2. Jetson 的 Maix RNDIS 接口是否已启用 `maixcam-usb-static`，地址是否为 `10.33.117.105`；
3. 两端端口是否都是 `5000`；
4. Jetson 监听命令是否报“地址已占用”。

Maix 识别程序不会因为连接失败而退出。它会保留识别功能，并在发送时自动重连。

当前版本还会每 2 秒发送一次低频 `MAIX_HEARTBEAT`。它只用于及时发现上一次
测试结束后留下的失效 socket，不参与任务解析；连接恢复后，画面中已经稳定的
同一任务码会自动补发一次，不需要等待二维码偶然漏检后再复位。

### 已连接，但没有 `MAIX_TCP_FRAME`

查看 Maix 控制台：

- 一直没有 `QR_STABLE`：二维码未连续稳定识别，或内容格式不合法；
- 有 `QR_STABLE`，没有 `QR_SENT`：网络发送仍然失败；
- 已有 `QR_SENT`：查看 Jetson 是否报 `MAIX_TCP_INVALID_FRAME`。

当前有效任务码必须是四段，例如 `156+123+516+231`；颜色数字只能是 `1` 到 `6`，位置段必须各自包含一次 `1、2、3`。

### Jetson 报端口已占用

检查是否已经运行了另一个接收程序：

```bash
ss -ltnp | grep ':5000'
```

停止旧进程，或同时修改 Jetson 的 `--port` 和 Maix 的 `TCP_SERVER_PORT`。

## 6. 本阶段的通过标准

连续完成 5 次下面的操作，每次 Jetson 都只收到一条有效 `MAIX_TCP_FRAME`，即可认为 Maix → Jetson 通信测试通过：

1. 展示二维码；
2. 等待 Maix 出现 `QR_SENT`；
3. 确认 Jetson 出现内容一致的 `MAIX_TCP_FRAME`；
4. 移开二维码至少 8 帧；
5. 再次展示。

注意：`maix-net` 仅用于独立收包诊断；正式流程中的 `coordinator` 已经可以直接接收 TCP 任务码并驱动色块、圆环和堆叠任务。一次性硬件测试结束后会关闭 coordinator，正式运行时则需要让 coordinator 持续运行。
