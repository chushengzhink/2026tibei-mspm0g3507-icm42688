import ast
import struct
from pathlib import Path


HEADER = bytes((0xAA, 0xCA, 0xAC, 0xBB))
FLAGS_REPORT_V1 = 0xA1
COMMAND_BALL_STATE = 0x02
BODY_FORMAT = "<IhhffB3x"

TEST_PERCENT_TO_CM_POINTS = (
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


def collect_constant_assignments(module, names):
    result = []
    for node in module.body:
        if not isinstance(node, ast.Assign):
            continue
        target_names = {
            target.id for target in node.targets if isinstance(target, ast.Name)
        }
        if not (target_names & names):
            continue
        result.append(node)
    return result


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

    main_source = (Path(__file__).parents[1] / "maixcam" / "main.py").read_text(
        encoding="utf-8"
    )
    module = ast.parse(main_source)
    assignments = {}
    for node in ast.walk(module):
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

    executable_nodes = []
    required_names = {
        "APP_CMD_BALL_STATE",
        "BALL_PACKET_FORMAT",
        "BALL_PERCENT_PER_CM",
        "BALL_PERCENT_TO_CM_POINTS",
        "MAIX_HEADER",
        "MAIX_FLAGS_REPORT_V1",
        "MAIX_BALL_DATA_SIZE",
    }
    executable_nodes.extend(collect_constant_assignments(module, required_names))
    for node in module.body:
        if isinstance(node, ast.FunctionDef) and node.name in {
            "crc16_ibm", "interpolate_position_cm", "map_center_x",
            "encode_ball",
        }:
            executable_nodes.append(node)
    namespace = {"struct": struct, "time": __import__("time")}
    exec(compile(ast.Module(body=executable_nodes, type_ignores=[]),
                 "maixcam/main.py", "exec"), namespace)
    namespace["half_width"] = 160.0

    assert namespace["BALL_PERCENT_TO_CM_POINTS"] == TEST_PERCENT_TO_CM_POINTS
    to_cm = namespace["interpolate_position_cm"]
    assert abs(to_cm(0.0) - 0.0) < 1e-6
    assert abs(to_cm(-33.8) + 5.0) < 1e-6
    assert abs(to_cm(34.4) - 5.0) < 1e-6
    assert abs(to_cm(-68.8) + 10.0) < 1e-6
    assert abs(to_cm(61.9) - 9.0) < 1e-6
    assert abs(to_cm((6.9 + 13.8) / 2.0) - 1.5) < 1e-6
    assert abs(to_cm(-83.8) + 12.0) < 1e-6
    assert abs(to_cm(80.8) - 12.0) < 1e-6
    assert abs(to_cm(-90.0) + 12.0) < 1e-6
    assert abs(to_cm(90.0) - 12.0) < 1e-6

    display_x, x_percent, raw_cm, fixed_cm = namespace["map_center_x"](160)
    assert display_x == 0.0
    assert x_percent == 0.0
    assert raw_cm == 0.0
    assert fixed_cm == 0.0

    center_x_for_plus_five = 160 - (160 * 34.4 / 100.0)
    _, x_percent, raw_cm, fixed_cm = namespace["map_center_x"](
        center_x_for_plus_five
    )
    assert abs(x_percent - 34.4) < 1e-6
    assert abs(raw_cm - 5.9310344828) < 1e-6
    assert abs(fixed_cm - 5.0) < 1e-6

    class BallObject:
        score = 0.75

    detected = {
        "obj": BallObject(),
        "center_x": 147,
        "center_y": 112,
        "raw_position_cm": 1.4008620690,
        "position_cm": 1.1775362319,
    }
    frames = [
        namespace["encode_ball"](detected),
        namespace["encode_ball"](None),
        namespace["encode_ball"](detected),
    ]
    assert all(len(item) == 32 for item in frames)
    assert [struct.unpack_from("<B", item, 26)[0] for item in frames] == [1, 0, 1]
    assert abs(struct.unpack_from("<f", frames[0], 18)[0] - 1.1775362319) < 1e-5
    assert struct.unpack_from("<IhhffB3x", frames[1], 10)[1:] == (
        0, 0, 0.0, 0.0, 0
    )
    assert "serial_dev.write(encode_ball(ball_info))" in main_source
    assert "RAWCM" in main_source
    assert "FIXCM" in main_source

    h6_source = (Path(__file__).parents[1] / "maixcam" / "h6_maix.py").read_text(
        encoding="utf-8"
    )
    h6_module = ast.parse(h6_source)
    h6_executable_nodes = []
    h6_executable_nodes.extend(
        collect_constant_assignments(h6_module, required_names)
    )
    for node in h6_module.body:
        if isinstance(node, ast.FunctionDef) and node.name in {
            "crc16_ibm", "interpolate_position_cm", "map_center_x",
            "encode_ball",
        }:
            h6_executable_nodes.append(node)
    h6_namespace = {"struct": struct, "time": __import__("time")}
    exec(compile(ast.Module(body=h6_executable_nodes, type_ignores=[]),
                 "maixcam/h6_maix.py", "exec"), h6_namespace)
    h6_namespace["half_width"] = 160.0
    assert h6_namespace["BALL_PERCENT_TO_CM_POINTS"] == TEST_PERCENT_TO_CM_POINTS
    h6_frame = h6_namespace["encode_ball"](detected)
    h6_lost_frame = h6_namespace["encode_ball"](None)
    assert len(h6_frame) == 32
    assert len(h6_lost_frame) == 32
    assert h6_frame[:4] == HEADER
    assert struct.unpack_from("<I", h6_frame, 4)[0] == 24
    assert struct.unpack_from("<B", h6_lost_frame, 26)[0] == 0
    assert abs(struct.unpack_from("<f", h6_frame, 18)[0] - 1.1775362319) < 1e-5
    assert "serial_dev.write(encode_ball(ball_info))" in h6_source
    assert "RAWCM" in h6_source
    assert "FIXCM" in h6_source
    assert "H6 MSP LOCK PB24" in h6_source


if __name__ == "__main__":
    run_tests()
    print("ball packet tests: PASS")
