from maix import app, camera, display, gpio, image, nn, pinmap, uart
import os
import struct
import time


APP_CMD_BALL_STATE = 0x02
BALL_PACKET_FORMAT = "<IhhffB3x"
BALL_PERCENT_PER_CM = 5.8
BALL_PERCENT_TO_CM_POINTS = (
    (-83.8, -12.0),
    (-68.8, -10.0),
    (-61.3, -9.0),
    (-54.4, -8.0),
    (-47.5, -7.0),
    (-41.2, -6.0),
    (-33.8, -5.0),
    (-26.9, -4.0),
    (-19.4, -3.0),
    (-12.5, -2.0),
    (-6.2, -1.0),
    (0.0, 0.0),
    (6.9, 1.0),
    (13.8, 2.0),
    (21.9, 3.0),
    (28.1, 4.0),
    (34.4, 5.0),
    (41.2, 6.0),
    (48.8, 7.0),
    (55.6, 8.0),
    (61.9, 9.0),
    (80.8, 12.0),
)
ROI_X_PERCENT_LIMIT = 84.0
ROI_Y_PERCENT_LIMIT = 20.0
DETECT_CONF_THRESHOLD = 0.20
DETECT_IOU_THRESHOLD = 0.70
MAIX_HEADER = bytes((0xAA, 0xCA, 0xAC, 0xBB))
MAIX_FLAGS_REPORT_V1 = 0xA1
MAIX_BALL_DATA_SIZE = 24
LOST_CAPTURE_DIR = "/root/12"
LOST_CAPTURE_INTERVAL_MS = 200
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


def monotonic_ms():
    return time.monotonic_ns() // 1_000_000


def interpolate_position_cm(x_percent):
    if x_percent <= BALL_PERCENT_TO_CM_POINTS[0][0]:
        return BALL_PERCENT_TO_CM_POINTS[0][1]
    for index in range(1, len(BALL_PERCENT_TO_CM_POINTS)):
        left_percent, left_cm = BALL_PERCENT_TO_CM_POINTS[index - 1]
        right_percent, right_cm = BALL_PERCENT_TO_CM_POINTS[index]
        if x_percent <= right_percent:
            span = right_percent - left_percent
            ratio = (x_percent - left_percent) / span
            return left_cm + (ratio * (right_cm - left_cm))
    return BALL_PERCENT_TO_CM_POINTS[-1][1]


def map_center_x(center_x):
    display_x = half_width - center_x
    x_percent = display_x / half_width * 100.0
    raw_position_cm = x_percent / BALL_PERCENT_PER_CM
    position_cm = interpolate_position_cm(x_percent)
    return display_x, x_percent, raw_position_cm, position_cm


def measure_obj(obj):
    center_x = obj.x + obj.w // 2
    center_y = obj.y + obj.h // 2
    display_x, x_percent, raw_position_cm, position_cm = map_center_x(center_x)
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
        "raw_position_cm": raw_position_cm,
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
        "MAP FIX +/-12",
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
    """Return the fixed 32-byte Maix report frame for the selected ball."""
    obj = ball_info["obj"]
    capture_ms = monotonic_ms() & 0xFFFFFFFF
    body = struct.pack(
        BALL_PACKET_FORMAT,
        capture_ms,
        ball_info["center_x"],
        ball_info["center_y"],
        ball_info["position_cm"],
        float(obj.score),
        1,
    )
    frame_without_crc = (
        MAIX_HEADER
        + struct.pack("<I", MAIX_BALL_DATA_SIZE)
        + bytes((MAIX_FLAGS_REPORT_V1, APP_CMD_BALL_STATE))
        + body
    )
    return frame_without_crc + struct.pack("<H", crc16_ibm(frame_without_crc))


def ensure_lost_capture_dir():
    if os.path.exists(LOST_CAPTURE_DIR):
        return True
    try:
        os.makedirs(LOST_CAPTURE_DIR)
        return True
    except OSError:
        return os.path.exists(LOST_CAPTURE_DIR)


def save_lost_frame(img, now_ms):
    global lost_capture_count, lost_capture_error, next_lost_capture_ms
    if now_ms < next_lost_capture_ms:
        return
    next_lost_capture_ms = now_ms + LOST_CAPTURE_INTERVAL_MS
    if not lost_capture_dir_ready:
        lost_capture_error = True
        return

    path = "%s/lost_%010d_%04d.jpg" % (
        LOST_CAPTURE_DIR,
        now_ms & 0xFFFFFFFF,
        lost_capture_count,
    )
    try:
        result = img.save(path, quality=90)
        if result:
            lost_capture_error = True
            return
        lost_capture_count += 1
        lost_capture_error = False
    except Exception:
        lost_capture_error = True


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
lost_capture_dir_ready = ensure_lost_capture_dir()
lost_capture_count = 0
lost_capture_error = not lost_capture_dir_ready
next_lost_capture_ms = 0

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, DETECT_CONF_THRESHOLD, DETECT_IOU_THRESHOLD)
    ball_info, roi_count = select_roi_ball(objs)

    if ball_info is not None and report_on:
        body = encode_ball(ball_info)
        serial_dev.write(body)

    if ball_info is None:
        now = monotonic_ms()
        save_lost_frame(img, now)
        img.draw_string(
            0,
            0,
            f"ROI LOST TH{DETECT_CONF_THRESHOLD:.2f}",
            color=image.COLOR_RED,
        )
        img.draw_string(
            0,
            16,
            f"CAP {lost_capture_count:04d}",
            color=image.COLOR_RED,
        )
        if lost_capture_error:
            img.draw_string(0, 32, "SAVE ERR /root/12", color=image.COLOR_RED)
    else:
        next_lost_capture_ms = 0
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
        img.draw_string(0, 16, f"RAWCM {ball_info['raw_position_cm']:+.2f}", color=image.COLOR_RED)
        img.draw_string(0, 32, f"FIXCM {ball_info['position_cm']:+.2f}", color=image.COLOR_RED)
        img.draw_string(
            0,
            48,
            f"Y {ball_info['display_y']:+.0f}px {ball_info['y_percent']:+.1f}%",
            color=image.COLOR_RED,
        )
        img.draw_string(0, 64, f"S {obj.score:.2f}", color=image.COLOR_RED)
        if roi_count > 1:
            img.draw_string(0, 80, f"ROI MULTI {roi_count}", color=image.COLOR_RED)
    draw_detector_status(img, len(objs), roi_count)
    dis.show(img)
