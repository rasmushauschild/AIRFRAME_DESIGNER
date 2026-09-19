"""Run against the local PX4 PositionControl implementation after `make firmware`."""
import json
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
commands = json.loads((root / "firmware/atlas/build/px4_sitl_default/compile_commands.json").read_text())
entry = next(c for c in commands if c["file"].endswith("/PositionControl.cpp"))
args = shlex.split(entry["command"])
args = [a for a in args[:args.index("-o")] if a not in ("-DNDEBUG", "-Werror")]
source = Path(entry["file"]).parent
with tempfile.TemporaryDirectory(prefix="atlas-vertical-test-") as output:
    binary = str(Path(output) / "test")
    subprocess.run(args + [str(root / "tests/firmware/test_manual_vertical_control.cpp"),
                          str(source / "PositionControl.cpp"), str(source / "ControlMath.cpp"),
                          "-o", binary], cwd=entry["directory"], check=True)
    subprocess.run([binary], check=True)
