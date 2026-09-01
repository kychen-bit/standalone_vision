"""Edit this file directly in MaixVision before deploying."""

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

DISPLAY_ENABLED = True
DRAW_BOX = True          # 是否在屏幕上画识别框（绿=有效稳定，黄=有效未稳定，红=无效）
CONFIRM_FRAMES = 4
RESET_AFTER_MISSED_FRAMES = 8
DEBUG_LOG_EVERY_FRAMES = 30

# 二维码检测方式：
#   "hw"  = 硬件加速 image.QRCodeDetector（快，但需 MaixPy >= v4.7.9）
#   "cpu" = img.find_qrcodes()（兼容老固件、不占 NPU，推荐先用它排障）
QR_DETECTOR_MODE = "cpu"

# Keep False for board-only recognition tests. Enabling it is the only action
# that opens UART1, so a disconnected Jetson never prevents camera testing.
SERIAL_ENABLED = False
UART_DEVICE = "/dev/ttyS1"
UART_BAUD = 115200
UART_TX_PIN = "A19"
UART_RX_PIN = "A18"
