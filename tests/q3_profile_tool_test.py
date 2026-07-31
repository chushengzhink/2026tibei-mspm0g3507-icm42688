from __future__ import annotations

import importlib.util
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "q3_fit_profile", ROOT / "tools" / "q3_fit_profile.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def make_csv(jump: bool = False, missing_last: bool = False) -> str:
    lines = ["Q3_MAP_BEGIN", ",".join(MODULE.FIELDS)]
    for position in range(-6, 7 - int(missing_last)):
        roll_plus = 40.0 + (100.0 if jump and position == 1 else 0.0)
        lines.append(
            f"{position:.3f},0.0,{roll_plus:.1f},42.0,105.0,110.0,"
            "7.00,7.50,3"
        )
    lines.append("Q3_MAP_END")
    return "\n".join(lines)


def expect_error(text: str) -> None:
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "map.csv"
        path.write_text(text, encoding="utf-8")
        try:
            MODULE.load_points(path)
        except MODULE.ProfileError:
            return
    raise AssertionError("invalid map was accepted")


def main() -> None:
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "map.csv"
        path.write_text(make_csv(), encoding="utf-8")
        points = MODULE.load_points(path)
        header = MODULE.render_header(points, -1, 1525.0, 0.25)
        assert len(points) == 13
        assert "Q3_PROFILE_GENERATED_POINTS" in header
        assert "Q3_PROFILE_GENERATED_AXIS_SIGN (-1)" in header
        assert "{ +0.0f, -1.0f, 40.0f, 42.0f" in header
        header_path = Path(directory) / "q3_ball_profile_generated.h"
        source_path = Path(directory) / "profile_compile.c"
        object_path = Path(directory) / "profile_compile.o"
        header_path.write_text(header, encoding="ascii")
        source_path.write_text(
            '#include "q3_ball_profile_generated.h"\n'
            "typedef struct { float position_cm; float balance_us; "
            "float roll_plus_us; float roll_minus_us; "
            "float break_plus_us; float break_minus_us; "
            "float accel_plus_cm_s2; float accel_minus_cm_s2; } point_t;\n"
            "static const point_t p[13] = Q3_PROFILE_GENERATED_POINTS;\n"
            "int profile_count(void) { return (int)(sizeof(p)/sizeof(p[0])); }\n",
            encoding="ascii",
        )
        subprocess.run(
            [
                "gcc",
                "-std=c99",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                directory,
                "-c",
                str(source_path),
                "-o",
                str(object_path),
            ],
            check=True,
        )
    expect_error(make_csv(missing_last=True))
    expect_error(make_csv(jump=True))
    print("q3 profile tool tests passed")


if __name__ == "__main__":
    main()
