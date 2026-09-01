# 可选串口输出

串口不是项目启动条件。Jetson只有传入 `--serial` 时才打开串口；MaixCAM Pro只有将
`maixcam_pro/config.py` 中的 `SERIAL_ENABLED` 改为 `True` 时才打开UART1。

帧格式：

```text
@NAME,FIELD,...*CRC16\n
```

CRC为对 `NAME,FIELD,...` ASCII正文计算的CRC16/CCITT-FALSE。

Jetson稳定结果：

```text
@VISION_RESULT,STABLE,COLOR,1,550.000,350.000,0.000,0.950,PX,5*XXXX
```

MaixCAM Pro稳定二维码：

```text
@MAIX_QR,123+123+456+231,320.0,240.0,120.0,4,STABLE*XXXX
```

以上是单项测试程序的发送帧。若要让视觉与电控整机闭环（任务码解析、REQ 请求、GRASP_READY
许可），使用 `coordinator` 模式：它既发送也读取控制命令，双向协议见 `docs/06_协调层通信协议与部署.md`。
单项测试入口与 `coordinator` 不能同时运行，否则会抢占同一相机或串口。
