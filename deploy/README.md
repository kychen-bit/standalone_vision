# 整机视觉 systemd 服务

安装并立即启用开机启动：

```bash
cd /home/ysu/standalone_vision
sudo ./tools/install_competition_service.sh
```

常用管理命令：

```bash
sudo systemctl start standalone-vision.service
sudo systemctl stop standalone-vision.service
sudo systemctl restart standalone-vision.service
sudo systemctl enable --now standalone-vision.service
sudo systemctl disable --now standalone-vision.service
systemctl status standalone-vision.service --no-pager
journalctl -u standalone-vision.service -f
```

服务以用户 `ysu` 在 `/home/ysu/standalone_vision` 下执行：

```text
./run_competition.sh --headless --skip-network-config
```

安装脚本同时安装 MaixCAM 热插拔规则。MaixCAM 每次插入后会自动识别
Sipeed RNDIS 网卡，将其命名为 `maixcam0`，并恢复 Jetson 侧
`10.33.117.105/24` 静态地址；比赛服务中的 TCP 监听会继续等待 MaixCAM
重新连接，无需手动运行网络配置命令。

热插拔状态检查：

```bash
ip -br address show maixcam0
systemctl status 'maixcam-network@*.service' --no-pager
journalctl -u 'maixcam-network@*.service' -n 30 --no-pager
```

修改代码或 JSON 配置后执行 `restart`。需要打开 GUI 手动调试时，先停止服务，再从桌面终端运行
`./run_competition.sh`，避免两个进程争用相机、MCU串口和 TCP 5000端口。

## 磁盘与日志防护

`/lib/udev/uvcdynctrl` 会在每次 UVC 视频节点创建时把整个 udev 环境 dump 到
`/var/log/uvcdynctrl-udev.log`（约 2.3 KB/次，无上限）。相机一旦进入 USB 重枚举循环，
该文件会把系统盘写满（2026 年 9 月曾达到 196 GB），进而导致图形界面和服务异常。

一次性安装系统侧防护（改写 `debug=0`、装 logrotate、装每 5 分钟运行的磁盘守护、
限制 journal 大小）：

```bash
cd /home/ysu/standalone_vision
sudo bash tools/install_log_guard.sh
```

状态检查：

```bash
df -h /
python3 tools/disk_guard.py --no-truncate
systemctl list-timers disk-guard.timer --no-pager
```

服务重启已改为**有界**（`StartLimitIntervalSec=600`、`StartLimitBurst=10`），
连续失败达到上限后进入 failed 状态而不再无限重启；设备到位后用
`sudo systemctl reset-failed standalone-vision.service` 再启动。
完整排查步骤见 [Jetson 系统盘写满与 USB 枚举风暴](../docs/Jetson磁盘写满与USB枚举风暴.md)。
