from maix import app, camera, display, gpio, image, nn, pinmap, uart
import os
import struct
import time


APP_CMD_BALL_STATE = 0x02
BALL_PACKET_FORMAT = "<IhhffB3x"
BALL_PERCENT_PER_CM = 5.8
ROI_X_PERCENT_LIMIT = 84.0
ROI_Y_PERCENT_LIMIT = 20.0
DETECT_CONF_THRESHOLD = 0.20
DETECT_IOU_THRESHOLD = 0.70
MAIX_HEADER = bytes((0xAA, 0xCA, 0xAC, 0xBB))
MAIX_FLAGS_REPORT_V1 = 0xA1
MAIX_BALL_DATA_SIZE = 24
report_on = True

pin_name = "B3"
gpio_name = "GPIOB3"
led = gpio.GPIO(gpio_name, gpio.Mode.OUT)
led.value(1)


def crc16_ibm(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def map_center_x(center_x):
    display_x = half_width - center_x
    x_percent = display_x / half_width * 100.0
    position_cm = x_percent / BALL_PERCENT_PER_CM
    return display_x, x_percent, position_cm


def measure_obj(obj):
    center_x = obj.x + obj.w // 2
    center_y = obj.y + obj.h // 2
    display_x, x_percent, position_cm = map_center_x(center_x)
    display_y = half_height - center_y
    y_percent = display_y / half_height * 100.0
    return {
        "obj": obj,
        "center_x": center_x,
        "center_y": center_y,
        "display_x": display_x,
        "display_y": display_y,
        "x_percent": x_percent,
        "y_percent": y_percent,
        "position_cm": position_cm,
    }


def select_roi_ball(objs):
    selected = None
    roi_count = 0
    for obj in objs:
        ball_info = measure_obj(obj)
        if (
            abs(ball_info["x_percent"]) <= ROI_X_PERCENT_LIMIT
            and abs(ball_info["y_percent"]) <= ROI_Y_PERCENT_LIMIT
        ):
            roi_count += 1
            if selected is None or obj.score > selected["obj"].score:
                selected = ball_info
    return selected, roi_count


def draw_center_point(img, center_x, center_y):
    point_x = max(0, min(image_width - 5, center_x - 2))
    point_y = max(0, min(image_height - 5, center_y - 2))
    img.draw_rect(point_x, point_y, 5, 5, color=image.COLOR_GREEN)


def draw_detector_status(img, total_count, roi_count):
    img.draw_string(
        0,
        image_height - 48,
        f"CAL {BALL_PERCENT_PER_CM:.1f}%/cm",
        color=image.COLOR_RED,
    )
    img.draw_string(
        0,
        image_height - 32,
        f"TH {DETECT_CONF_THRESHOLD:.2f}",
        color=image.COLOR_RED,
    )
    img.draw_string(
        0,
        image_height - 16,
        f"DET {total_count} ROI {roi_count}",
        color=image.COLOR_RED,
    )


def encode_ball(ball_info):
    """Return one fixed report frame, using valid=0 when no ball is found."""
    capture_ms = (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF
    if ball_info is None:
        center_x = 0
        center_y = 0
        position_cm = 0.0
        score = 0.0
        valid = 0
    else:
        obj = ball_info["obj"]
        center_x = ball_info["center_x"]
        center_y = ball_info["center_y"]
        position_cm = ball_info["position_cm"]
        score = float(obj.score)
        valid = 1
    body = struct.pack(
        BALL_PACKET_FORMAT,
        capture_ms,
        center_x,
        center_y,
        position_cm,
        score,
        valid,
    )
    frame_without_crc = (
        MAIX_HEADER
        + struct.pack("<I", MAIX_BALL_DATA_SIZE)
        + bytes((MAIX_FLAGS_REPORT_V1, APP_CMD_BALL_STATE))
        + body
    )
    return frame_without_crc + struct.pack("<H", crc16_ibm(frame_without_crc))


model_path = "yolo26n_ball_img_only_hardened_v9_lost12_photometric_320x224.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/4xiao/yolo26n_ball_img_only_hardened_v9_lost12_photometric_320x224.mud"

detector = nn.YOLO26(model=model_path)
image_width = 320
image_height = 224
half_width = image_width / 2.0
half_height = image_height / 2.0
cam = camera.Camera(
    image_width, image_height, detector.input_format()
)
dis = display.Display()

pinmap.set_pin_function("A28", "UART2_TX")
pinmap.set_pin_function("A29", "UART2_RX")
serial_dev = uart.UART("/dev/ttyS2", 115200)

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, DETECT_CONF_THRESHOLD, DETECT_IOU_THRESHOLD)
    ball_info, roi_count = select_roi_ball(objs)

    if report_on:
        serial_dev.write(encode_ball(ball_info))

    if ball_info is None:
        img.draw_string(
            0,
            0,
            f"ROI LOST TH{DETECT_CONF_THRESHOLD:.2f}",
            color=image.COLOR_RED,
        )
    else:
        obj = ball_info["obj"]
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
        msg = f"{detector.labels[obj.class_id]}: {obj.score:.2f}"
        img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)
        draw_center_point(img, ball_info["center_x"], ball_info["center_y"])
        img.draw_string(
            0,
            0,
            f"X {ball_info['display_x']:+.0f}px {ball_info['x_percent']:+.1f}%",
            color=image.COLOR_RED,
        )
        img.draw_string(0, 16, f"CM {ball_info['position_cm']:+.2f}", color=image.COLOR_RED)
        img.draw_string(
            0,
            32,
            f"Y {ball_info['display_y']:+.0f}px {ball_info['y_percent']:+.1f}%",
            color=image.COLOR_RED,
        )
        img.draw_string(0, 48, f"S {obj.score:.2f}", color=image.COLOR_RED)
        if roi_count > 1:
            img.draw_string(0, 64, f"ROI MULTI {roi_count}", color=image.COLOR_RED)
    draw_detector_status(img, len(objs), roi_count)
    dis.show(img)
