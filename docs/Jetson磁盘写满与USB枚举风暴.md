# Jetson 系统盘写满与 USB 枚举风暴

本文记录 2026 年 9 月那次「233 GB 系统盘被写满、图形界面无法启动」的排查结论，
并给出**避免复发**的固定动作。结论先说：

> 写满磁盘的那 196 GB `uvcdynctrl-udev.log` **不是本仓库代码写出来的**，
> 但它之所以能膨胀到 196 GB，是「DECXIN 相机 USB 反复重新枚举」和
> 「`uvcdynctrl` 的 udev 脚本每次都把整个 udev 环境 dump 到日志、且无人限制大小」
> 两件事叠加的结果。本仓库是这台 Jetson 上占用该相机的主力程序，因此它既可能是
> 枚举风暴的参与者，也必须在事后把防护做掉。

## 1. 事实链条

### 1.1 日志是谁写的（已确认）

```text
/lib/udev/rules.d/80-uvcdynctrl.rules:
    ACTION=="add", SUBSYSTEM=="video4linux", DRIVERS=="uvcvideo", RUN+="/lib/udev/uvcdynctrl"

/lib/udev/uvcdynctrl（0.2.4-1.1ubuntu2，脚本头写的是 2013 年）:
    debug=1                        # 打开日志
    logfile=/var/log/uvcdynctrl-udev.log
    echo >> $logfile ...           # 每个事件先写分隔行
    set >> $logfile                # 关键：把整个 udev 环境变量 dump 进去
    $cmd >> $logfile 2>&1          # uvcdynctrl -d $DEVNAME --addctrl=VID:PID
```

* 规则只匹配 `ACTION=="add"`、`SUBSYSTEM=="video4linux"`，即**只有视频节点被创建时**才会触发；
* 实测每次触发写入约 **2.3 KB**（当前日志 4568 B / 2 次触发），其中一个相机（`/dev/video0`+`/dev/video1`）
  一次重新枚举 = 2 次触发 ≈ **4.6 KB**；
* `/etc/logrotate.d/` 里**没有任何**针对该文件的轮转配置，`journald.conf` 也是全默认。
  也就是说：这个文件在系统层面完全没有上限。

### 1.2 为什么会触发到 196 GB（现场证据）

`/var/log/kern.log` 里 `usb 1-2.3`（DECXIN 相机所在端口）的历史记录：

| 消息 | 次数 |
|---|---|
| `Failed to query (GET_DEF) UVC control N on unit N: -N` | 11357 |
| `Failed to query (GET_INFO) UVC control N on unit N: -N` | 7217 |
| `new high-speed USB device number N using tegra-xusb` | 4410 |
| `USB disconnect, device number N` | 1819 |
| `Found UVC 1.00 device DECXIN CAMERA (1bcf:2cd1)` | 1553 |
| `Device not responding to setup address.` | 1442 |
| `device descriptor read/64, error -71` | 1247 |
| `can't read configurations, error -X` | 1050 |
| `unable to read config index N descriptor/all` | 913 |
| `device not accepting address N, error -N` | 718 |

按天统计端口 `1-2.3` 的日志行数（说明是**越来越快的失控循环**，不是偶发插拔）：

```text
Sep 9   45 行/天
Sep 10  约 1290 行/天
Sep 11  约 3070 行/天
Sep 12  约 9970 行/天
Sep 13  17718 行/天   ← 磁盘被写满前后
```

同时 `uvcvideo 1-2.2:1.1 / 1-2.3:1.1: Failed to query (129) UVC probe control : -71 (exp. 26)`
在 9 月 9 日—13 日之间反复出现 817 次，间隔只有 2~6 秒。

综合起来是一条正反馈回路：

```text
相机 XU/控制查询失败（-71 EPROTO，相机固件本身不规范）
        ↓
uvcvideo 初始化失败 → 设备被复位 / 断开重连
        ↓
内核重新枚举 → 创建 /dev/video0、/dev/video1（video4linux add 事件）
        ↓
udev 触发 /lib/udev/uvcdynctrl → 又去查询同一组 XU 控制
        ↓
写入 4.6 KB 到 /var/log/uvcdynctrl-udev.log（无上限）
        ↓
回到第一步
```

### 1.3 本仓库在其中的角色

**没有做的事**（已确认，可用 `grep` 复核）：代码里没有 `udevadm`、没有 `uvcdynctrl`、
没有 `modprobe`/unbind 之类会重置 USB 的调用，也从不写 `/var/log/uvcdynctrl-udev.log`。
安装脚本里的 `udevadm control --reload-rules` 只重载规则，不产生事件。

**有嫌疑、需要消除的放大器**：

1. **相机独占**：`run_competition.sh`（systemd）与 `tools/capture_*.py`、
   `tests/`、桌面手动运行，都可能同时打开同一台 UVC 相机。两路读同一台 UVC
   设备会把流卡死并把设备推进重枚举，这是最典型的诱因。原代码没有互斥保护。
2. **启动即写 17 个 V4L2 控制**：每次启动都用 `v4l2-ctl --set-ctrl` 逐个写
   `focus_absolute`/`exposure_time_absolute` 等，设备状态不佳时这些 ioctl 会失败
   （`v4l2_controls_strict=true` 直接抛异常退出）。
3. **无限重启**：`standalone-vision.service` 原来是
   `StartLimitIntervalSec=0` + `Restart=on-failure` + `RestartSec=3`，
   任何设备缺失都会变成「每 3 秒开一次相机、写一次控制」的永久循环。
4. **ExecStartPre 强制要求 MaixCAM**：`configure_maix_network.py` 默认 `required=True`，
   MaixCAM 没插时 `ExecStartPre` 直接失败，服务永远起不来（叠加第 3 条）。
5. **USB 拓扑**（当前机器实测）：相机 `1-2.1.4`、MaixCAM `1-2.1.1`、HID 设备
   `1-2.2` 都挂在同一个**总线供电**的 USB 2.0 Hub 链上（Genesys Hub 挂在 Realtek Hub
   下，再挂到 Bus 01 根口），而相机和 MaixCAM 各自申请 500 mA。
   相机 1280x720@60 MJPG 的等时带宽 + RNDIS 网络 + 供电余量不足，
   正是「设备无法分配地址 / 描述符读失败」的典型工况。
6. **另一套视觉栈**：`/etc/systemd/system/rm.service` → `/usr/sbin/rm_watch_dog.sh`
   会在心跳缺失时 `pkill -f ros` 并重启全部 ROS 节点，`syslog` 显示 3 月 28 日到
   9 月 8 日期间它每约 20 秒重启一轮（`Check armor_detector` 等）。它不是本仓库，
   但和本仓库同时跑会一起抢 CPU/USB/相机，比赛时应当禁用它。

## 2. 已经做的防护（本仓库内）

| 文件 | 作用 |
|---|---|
| `tools/disk_guard.py` | 采样磁盘占用与 `uvcdynctrl-udev.log` 的体积、**增长速率**和触发次数，超过阈值自动截断，输出 JSON；同时自检"防护是否装好"（`guard` 字段），触发次数/小时可作为「相机是否在枚举风暴」的指示器 |
| `deploy/disk-guard.service` + `.timer` | 每 5 分钟跑一次上面这个守护 |
| `deploy/logrotate/uvcdynctrl-udev` | 万一 udev 脚本被还原，8 MB 轮转兜底（需要 `logrotate` 包，可选） |
| `tools/install_log_guard.sh` | 一键安装：把 `/lib/udev/uvcdynctrl` 的 `debug=1` 改成 `debug=0`（日志写 `/dev/null`）、装 logrotate（缺失时跳过或加 `--with-logrotate`）、装定时守护、限制 journal（`SystemMaxUse=1G`、`SystemKeepFree=8G`）；**任何可选步骤失败都不会中断安装**，最后打印自检表 |
| `deploy/standalone-vision.service` | 重启改为有界：`StartLimitIntervalSec=600`、`StartLimitBurst=10`、`RestartSec=15`；`ExecStartPre` 改用 `--optional` |
| `tools/configure_maix_network.py --optional` | MaixCAM 没插时打印提示并以 0 退出，不再让服务卡在 `ExecStartPre` 上反复重启 |
| `jetson_recognition/camera.py` | 打开相机加**独占锁告警**（`CAMERA_BUSY_WARNING`，第二个进程打开同一台相机会被点出来）；打开失败改为按 `open_retries`/`open_retry_delay_s` 退避重试，不再立刻崩溃重启 |
| `tools/start_competition.py` | 启动前检查磁盘占用与 udev 日志体积，>95% 或日志 >64 MB 直接拒启动并给出修复命令 |

## 3. 现在要做的固定动作

### 3.1 先装防护（需要 sudo，密码在 Jetson 本机输入）

```bash
cd /home/ysu/standalone_vision
sudo bash tools/install_log_guard.sh
# 需要轮转兜底时（本机默认没装 logrotate 包）：
sudo bash tools/install_log_guard.sh --with-logrotate
```

该脚本把 udev 脚本的 `debug` 关掉后，**即使相机再进入枚举风暴也不会产生日志**；
logrotate（可选）、磁盘守护、journal 上限只是第二层保险。

脚本结尾会打印自检表，逐行确认这几项：

```text
udev helper   : debug=0     ← 关键项，必须为 0
logrotate     : not installed (optional)  或  installed, config present
guard timer   : enabled / active           ← 每 5 分钟截断兜底
journal cap   : present
```

> 踩坑记录：脚本第一版用 `set -e` + `logrotate --debug` 做校验，而这台 Jetson
> **没有装 logrotate 包**（`command not found`，退出码 127），导致安装在第 2 步
> 中断，第 3、4 步（守护定时器、journal 上限）从未执行。现在改为记录失败并继续，
> 结尾统一列出失败项，因此不会再出现"看起来装好了其实只装了一半"。

### 3.2 比赛前确认状态

```bash
# 一条命令看"还会不会爆"（不需要 sudo）
python3 tools/disk_guard.py --no-truncate
```

关键字段：

```json
"guard": {"uvcdynctrl_debug": "0", "uvcdynctrl_log_disabled": true, ...}
```

* `uvcdynctrl_log_disabled` 必须为 `true`（即 `debug=0`）：满足这一条，**日志就绝不会再被写大**；
* `disk_guard_timer_active` 建议为 `active`，缺失会给出 warning（只是兜底缺一层，不影响根因已修复的结论）；
* `uvcdynctrl_log.firings_per_hour` 应接近 0，持续上涨说明相机还在重枚举，需处理 USB 侧。

```bash
# 其它检查
df -h /
systemctl list-timers disk-guard.timer --no-pager
journalctl -u disk-guard.service -n 20 --no-pager

# 端到端验证（任选其一，之后日志大小应不变）
#   1) 拔插一次相机
#   2) sudo udevadm trigger --subsystem-match=video4linux

# 相机是否在反复重枚举（另开一个终端，运行整机程序时观察）
udevadm monitor --subsystem-match=video4linux
```

判断标准：

* `disk_guard` 报告里的 `uvcdynctrl_log.firings_per_hour` 应接近 0；
  上百次/小时说明相机正在重枚举，必须处理 USB 侧，而不是只靠截断日志；
* `udevadm monitor` 正常时只在插拔相机那一刻输出，**运行中不应持续刷屏**。

### 3.3 处理 USB 侧（真正消除风暴）

按性价比排序：

1. 相机、MaixCAM **不要挂在同一个 USB Hub 链上**。当前 Bus 01 是链路最拥挤的一条，
   Bus 02（USB 3.0）还有空口，把 MaixCAM 或相机换到另一条总线/根口；
2. 给相机换**带外接供电的 Hub**，或直插 Jetson 板载口，避免 500 mA 叠加导致掉压；
3. 换更短的 USB 线，避免劣质延长线（`device descriptor read/64, error -71` 的经典成因）；
4. 关掉相机 USB 自动挂起：`power/control=auto`、`autosuspend_delay_ms=2000` 可改为 `on`
   （临时验证：`echo on | sudo tee /sys/bus/usb/devices/1-2.1.4/power/control`）；
5. 同一时刻**只允许一个进程**用相机：跑 `run_competition.sh` 前先
   `sudo systemctl stop standalone-vision.service`，不要同时跑测试脚本；
6. 比赛期间禁用另一套视觉栈：`sudo systemctl disable --now rm.service`。

### 3.4 顺带清理的旧习惯

* 观察日志不要用 `watch -n 1 ls -lh ...`（本机曾经开着两个这样的 `watch` 常驻进程），
  用 `tools/disk_guard.py` 或 `ls -lh` 查一次即可；
* 备份/复制大文件前先看 `df -h /`，磁盘满的时候 `apt`、`journalctl`、`git` 都会表现异常，
  容易误判成「Jetson 坏了」。

## 4. 故障复发时的 30 秒定位

```bash
df -h /                                                       # 是否又满了
sudo du -xhd1 /var | sort -h | tail                            # 谁占的
sudo find /var/log -xdev -type f -size +1G -exec ls -lh {} \;  # 大文件
grep -c 'Triggered at' /var/log/uvcdynctrl-udev.log            # 触发次数
dmesg | grep -c 'new high-speed USB device'                    # 枚举次数
python3 tools/disk_guard.py                                    # 增长速率 + 自动截断
```

只要 `Triggered at` 次数或 `firings_per_hour` 在持续上涨，就说明相机在重枚举，
按第 3.3 节处理 USB 侧即可；`/lib/udev/uvcdynctrl` 是否已改回 `debug=0` 用
`grep '^debug=' /lib/udev/uvcdynctrl` 确认（`apt` 升级 `uvcdynctrl` 包会覆盖它，
重新跑一次 `install_log_guard.sh` 即可）。
