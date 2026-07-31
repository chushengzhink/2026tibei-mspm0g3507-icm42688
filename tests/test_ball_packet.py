import ast
import struct
from pathlib import Path


HEADER = bytes((0xAA, 0xCA, 0xAC, 0xBB))
FLAGS_REPORT_V1 = 0xA1
COMMAND_BALL_STATE = 0x02
BODY_FORMAT = "<IhhffB3x"


def crc16_ibm(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def encode_frame(capture_ms, x, y, position_cm, score, valid):
    body = struct.pack(
        BODY_FORMAT, capture_ms, x, y, position_cm, score, valid
    )
    data_length = 1 + 1 + len(body) + 2
    frame_without_crc = (
        HEADER
        + struct.pack("<I", data_length)
        + bytes((FLAGS_REPORT_V1, COMMAND_BALL_STATE))
        + body
    )
    return frame_without_crc + struct.pack("<H", crc16_ibm(frame_without_crc))


def run_tests():
    official_example = bytes.fromhex(
        "AA CA AC BB 09 00 00 00 00 01 68 65 6C 6C 6F"
    )
    assert crc16_ibm(official_example) == 0x442B
    assert struct.calcsize(BODY_FORMAT) == 20

    frame = encode_frame(0x12345678, 160, 112, 0.0, 0.75, 1)
    assert len(frame) == 32
    assert frame[:4] == HEADER
    assert struct.unpack_from("<I", frame, 4)[0] == 24
    assert frame[8] == FLAGS_REPORT_V1
    assert frame[9] == COMMAND_BALL_STATE
    assert struct.unpack_from("<IhhffB3x", frame, 10) == (
        0x12345678,
        160,
        112,
        0.0,
        0.75,
        1,
    )
    assert crc16_ibm(frame[:-2]) == struct.unpack_from("<H", frame, 30)[0]

    left_frame = encode_frame(10, 40, 112, 11.0, 0.9, 1)
    assert struct.unpack_from("<f", left_frame, 18)[0] == 11.0

    lost_frame = encode_frame(20, 0, 0, 0.0, 0.0, 0)
    assert struct.unpack_from("<B", lost_frame, 26)[0] == 0

    image_width = 320
    half_width = image_width / 2.0
    percent_per_cm = 5.8
    to_cm = lambda x: ((half_width - x) / half_width * 100.0) / percent_per_cm
    assert abs(to_cm(160) - 0.0) < 1e-6
    assert abs(to_cm(160 - half_width * percent_per_cm / 100.0) - 1.0) < 1e-6
    assert abs(to_cm(160 + half_width * percent_per_cm / 100.0) + 1.0) < 1e-6
    assert abs(to_cm(160 - half_width * 29.0 / 100.0) - 5.0) < 1e-6
    assert abs(to_cm(147) - 1.4008620690) < 1e-6
    assert abs(to_cm(173) + 1.4008620690) < 1e-6
    assert abs(to_cm(40) - 12.9310344828) < 1e-6
    assert abs(to_cm(280) + 12.9310344828) < 1e-6
    assert abs((to_cm(159) - to_cm(160)) - 0.1077586207) < 1e-6
    assert abs(to_cm(132) - 3.0172413793) < 1e-6

    main_source = (Path(__file__).parents[1] / "maixcam" / "main.py").read_text(
        encoding="utf-8"
    )
    assignments = {}
    for node in ast.walk(ast.parse(main_source)):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name):
            continue
        if isinstance(node.value, ast.Constant):
            assignments[target.id] = node.value.value
        elif isinstance(node.value, ast.Name) and node.value.id in assignments:
            assignments[target.id] = assignments[node.value.id]
    assert assignments["BALL_PERCENT_PER_CM"] == 5.8
    assert assignments["image_width"] == 320
    assert assignments["image_height"] == 224

    module = ast.parse(main_source)
    executable_nodes = []
    required_names = {
        "APP_CMD_BALL_STATE",
        "BALL_PACKET_FORMAT",
        "MAIX_HEADER",
        "MAIX_FLAGS_REPORT_V1",
        "MAIX_BALL_DATA_SIZE",
    }
    for node in module.body:
        if isinstance(node, ast.Assign):
            names = {
                target.id for target in node.targets
                if isinstance(target, ast.Name)
            }
            if names & required_names:
                executable_nodes.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in {
            "crc16_ibm", "encode_ball"
        }:
            executable_nodes.append(node)
    namespace = {"struct": struct, "time": __import__("time")}
    exec(compile(ast.Module(body=executable_nodes, type_ignores=[]),
                 "maixcam/main.py", "exec"), namespace)

    class BallObject:
        score = 0.75

    detected = {
        "obj": BallObject(),
        "center_x": 147,
        "center_y": 112,
        "position_cm": 1.4008620690,
    }
    frames = [
        namespace["encode_ball"](detected),
        namespace["encode_ball"](None),
        namespace["encode_ball"](detected),
    ]
    assert all(len(item) == 32 for item in frames)
    assert [struct.unpack_from("<B", item, 26)[0] for item in frames] == [1, 0, 1]
    assert struct.unpack_from("<IhhffB3x", frames[1], 10)[1:] == (
        0, 0, 0.0, 0.0, 0
    )
    assert "serial_dev.write(encode_ball(ball_info))" in main_source


if __name__ == "__main__":
    run_tests()
    print("ball packet tests: PASS")
