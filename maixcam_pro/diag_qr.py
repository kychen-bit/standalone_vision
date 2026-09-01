"""MaixCAM Pro 二维码排障诊断脚本。

在 MaixVision 里单独运行（和 main.py 无关），它会把以下信息打印到终端：

1. MaixPy 固件版本（判断 image.QRCodeDetector 是否支持，需 >= v4.7.9）；
2. 摄像头实际分辨率；
3. 两种检测方式的对比：
   - img.find_qrcodes()   （CPU，老固件也支持）
   - image.QRCodeDetector().detect(img)（硬件加速，需 v4.7.9+）
4. 每 0.5 秒打印一次两种方式分别检测到几个二维码，并打印内容。

用法：
- 把本文件（连同 config.py 一起）同步到板子；
- MaixVision 里运行 diag_qr.py；
- 把二维码放在摄像头前，观察终端输出。
"""

from maix import app, camera, display, image

try:
    import maix
    print("MAIX_VERSION:", getattr(maix, "__version__", "unknown"))
except Exception as exc:
    print("MAIX_VERSION read failed:", exc)

cam = camera.Camera(320, 224)
print("CAMERA SIZE:", cam.width(), "x", cam.height())
disp = display.Display()

detector = None
try:
    detector = image.QRCodeDetector()
    print("QRCodeDetector: created OK (hw mode supported)")
except Exception as exc:
    print("QRCodeDetector: FAILED ->", repr(exc))

print("DIAG_READY  (put a QR code in front of the camera)")
while not app.need_exit():
    img = cam.read()
    q_cpu = img.find_qrcodes()
    q_hw = detector.detect(img) if detector is not None else []
    line = "cpu=%d hw=%d" % (len(q_cpu), len(q_hw))
    if q_cpu:
        line += "  cpu_payload=%r" % q_cpu[0].payload()
    if q_hw:
        line += "  hw_payload=%r" % q_hw[0].payload()
    print(line, flush=True)
    disp.show(img)
    app.sleep_ms(500)
