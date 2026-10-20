"""Run the sealed seven-primitive matrix only after exact B0 reproduction."""

import json
import subprocess
import sys
from pathlib import Path

from primitive_measurement import PRIMITIVES, same_start


HERE = Path(__file__).resolve().parent
repro = json.loads((HERE / "b0_reproduction.json").read_text())
if not repro["passed"]:
    raise SystemExit("B0 reproduction failed; no branches authorized by protocol")
initial = json.loads((HERE / "b0_replay/initial_execution.json").read_text())
status = {}
for name in PRIMITIVES:
    with (HERE / f"{name}.log").open("w") as log:
        code = subprocess.run([sys.executable, str(HERE / "run_one.py"), "--case", name],
                              stdout=log, stderr=subprocess.STDOUT).returncode
    status[name] = {"return_code": code}
    if code == 0:
        current = json.loads((HERE / name / "initial_execution.json").read_text())
        status[name]["initial_execution_match"] = same_start(initial, current)
        status[name]["result"] = json.loads((HERE / name / "result.json").read_text())
    (HERE / "run_status.json").write_text(json.dumps(status, indent=2) + "\n")
    print(name, json.dumps(status[name]), flush=True)
    if code or not status[name].get("initial_execution_match", False):
        raise SystemExit(f"Invalid branch {name}; stop matrix")
