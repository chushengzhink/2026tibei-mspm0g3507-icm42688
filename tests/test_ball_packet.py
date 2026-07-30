import struct


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


if __name__ == "__main__":
    run_tests()
    print("ball packet tests: PASS")
