from maix import app, camera, comm, display, image, nn
import os
import struct
import time


APP_CMD_BALL_STATE = 0x02
BALL_PACKET_FORMAT = "<IhhffB3x"
BALL_REFERENCE_PERCENT = 75.0
BALL_REFERENCE_DISTANCE_CM = 11.0


def map_center_x(center_x, half_width):
    display_x = half_width - center_x
    x_percent = display_x / half_width * 100.0
    position_cm = x_percent * BALL_REFERENCE_DISTANCE_CM / BALL_REFERENCE_PERCENT
    return display_x, x_percent, position_cm


def encode_ball(obj, half_width):
    """Return the fixed 20-byte little-endian ball report body."""
    capture_ms = (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF
    if obj is None:
        return struct.pack(BALL_PACKET_FORMAT, capture_ms, 0, 0, 0.0, 0.0, 0)

    center_x = obj.x + obj.w // 2
    center_y = obj.y + obj.h // 2
    _, _, position_cm = map_center_x(center_x, half_width)
    return struct.pack(
        BALL_PACKET_FORMAT,
        capture_ms,
        center_x,
        center_y,
        position_cm,
        float(obj.score),
        1,
    )


model_path = "model_262008.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/4xiao/yolo26n_ball_gangqiu_100e_224x320.mud"

detector = nn.YOLO26(model=model_path)
image_width = detector.input_width()
image_height = detector.input_height()
half_width = image_width / 2.0
half_height = image_height / 2.0
cam = camera.Camera(
    image_width, image_height, detector.input_format()
)
dis = display.Display()

# In MaixCam settings select UART, 115200 baud, 8N1. The MSPM0 receives
# reports on UART2 PB16; PB15 is available for MaixCam RX if needed.
protocol = comm.CommProtocol(buff_size=1024)

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th=0.35, iou_th=0.35)
    ball = max(objs, key=lambda obj: obj.score) if objs else None

    protocol.report(APP_CMD_BALL_STATE, encode_ball(ball, half_width))

    for obj in objs:
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
        msg = f"{detector.labels[obj.class_id]}: {obj.score:.2f}"
        img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)
    if ball is None:
        img.draw_string(0, 0, "ball: LOST", color=image.COLOR_RED)
    else:
        center_x = ball.x + ball.w // 2
        center_y = ball.y + ball.h // 2
        display_x, x_percent, x_cm = map_center_x(center_x, half_width)
        display_y = half_height - center_y
        y_percent = display_y / half_height * 100.0
        img.draw_string(
            0, 0, f"X {display_x:+.0f}px {x_percent:+.1f}%", color=image.COLOR_RED
        )
        img.draw_string(0, 16, f"CM {x_cm:+.2f}", color=image.COLOR_RED)
        img.draw_string(
            0, 32, f"Y {display_y:+.0f}px {y_percent:+.1f}%", color=image.COLOR_RED
        )
        img.draw_string(0, 48, f"S {ball.score:.2f}", color=image.COLOR_RED)
    dis.show(img)
