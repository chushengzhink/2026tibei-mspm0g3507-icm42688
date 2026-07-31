"""独立第三题视觉端：只检测小球并发送 32 字节 Maix 报告帧。

MSPM0 端负责全部控制逻辑；本脚本不发送舵机命令，也不依赖摆杆识别。
"""

from maix import app, camera, display, image, nn, pinmap, uart
import os
import struct
import time


APP_CMD_BALL_STATE = 0x02
BALL_PACKET_FORMAT = "<IhhffB3x"
MAIX_HEADER = bytes((0xAA, 0xCA, 0xAC, 0xBB))
MAIX_FLAGS_REPORT_V1 = 0xA1
MAIX_BALL_DATA_SIZE = 24

# 第三题独立映射，正负方向沿用现有实测分段标定。
BALL_PERCENT_TO_CM_POINTS = (
    (-83.8, -12.0), (-68.8, -10.0), (-61.3, -9.0),
    (-54.4, -8.0), (-47.5, -7.0), (-41.2, -6.0),
    (-33.8, -5.0), (-26.9, -4.0), (-19.4, -3.0),
    (-12.5, -2.0), (-6.2, -1.0), (0.0, 0.0),
    (6.9, 1.0), (13.8, 2.0), (21.9, 3.0),
    (28.1, 4.0), (34.4, 5.0), (41.2, 6.0),
    (48.8, 7.0), (55.6, 8.0), (61.9, 9.0),
    (80.8, 12.0),
)
ROI_X_PERCENT_LIMIT = 84.0
ROI_Y_PERCENT_LIMIT = 20.0
DETECT_CONF_THRESHOLD = 0.20
DETECT_IOU_THRESHOLD = 0.70


def crc16_ibm(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF


def interpolate_cm(x_percent):
    if x_percent <= BALL_PERCENT_TO_CM_POINTS[0][0]:
        return BALL_PERCENT_TO_CM_POINTS[0][1]
    for index in range(1, len(BALL_PERCENT_TO_CM_POINTS)):
        left_percent, left_cm = BALL_PERCENT_TO_CM_POINTS[index - 1]
        right_percent, right_cm = BALL_PERCENT_TO_CM_POINTS[index]
        if x_percent <= right_percent:
            ratio = (x_percent - left_percent) / (right_percent - left_percent)
            return left_cm + ratio * (right_cm - left_cm)
    return BALL_PERCENT_TO_CM_POINTS[-1][1]


def measure_obj(obj):
    center_x = obj.x + obj.w // 2
    center_y = obj.y + obj.h // 2
    x_percent = (((image_width / 2.0) - center_x) /
                 (image_width / 2.0) * 100.0)
    y_percent = (((image_height / 2.0) - center_y) /
                 (image_height / 2.0) * 100.0)
    return {
        "obj": obj,
        "center_x": center_x,
        "center_y": center_y,
        "x_percent": x_percent,
        "y_percent": y_percent,
        "position_cm": interpolate_cm(x_percent),
    }


def select_ball(objects):
    selected = None
    for obj in objects:
        info = measure_obj(obj)
        if (abs(info["x_percent"]) <= ROI_X_PERCENT_LIMIT and
                abs(info["y_percent"]) <= ROI_Y_PERCENT_LIMIT):
            if selected is None or obj.score > selected["obj"].score:
                selected = info
    return selected


def encode_report(info):
    capture_ms = (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF
    if info is None:
        body = struct.pack(BALL_PACKET_FORMAT, capture_ms, 0, 0,
                           0.0, 0.0, 0)
    else:
        obj = info["obj"]
        body = struct.pack(BALL_PACKET_FORMAT, capture_ms,
                           info["center_x"], info["center_y"],
                           info["position_cm"], float(obj.score), 1)
    frame = (MAIX_HEADER + struct.pack("<I", MAIX_BALL_DATA_SIZE) +
             bytes((MAIX_FLAGS_REPORT_V1, APP_CMD_BALL_STATE)) + body)
    return frame + struct.pack("<H", crc16_ibm(frame))


image_width = 320
image_height = 224
model_path = "yolo26n_ball_img_only_hardened_v4_320x224.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/4xiao/" + model_path

detector = nn.YOLO26(model=model_path)
cam = camera.Camera(image_width, image_height, detector.input_format())
dis = display.Display()
pinmap.set_pin_function("A28", "UART2_TX")
pinmap.set_pin_function("A29", "UART2_RX")
serial_dev = uart.UART("/dev/ttyS2", 115200)

while not app.need_exit():
    img = cam.read()
    objects = detector.detect(img, DETECT_CONF_THRESHOLD,
                               DETECT_IOU_THRESHOLD)
    ball = select_ball(objects)
    serial_dev.write(encode_report(ball))

    if ball is None:
        img.draw_string(0, 0, "Q3 BALL LOST", color=image.COLOR_RED)
    else:
        obj = ball["obj"]
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_GREEN)
        img.draw_string(0, 0, "Q3 X %.2fcm S %.2f" %
                        (ball["position_cm"], obj.score),
                        color=image.COLOR_GREEN)
    dis.show(img)
