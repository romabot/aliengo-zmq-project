#!/usr/bin/env python3
"""Integration test for Aliengo walking performance.

Starts the PD controller and the simulator. After the simulation finishes,
checks that the robot walked stably and reached target speed.
"""

import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


def _xvfb_prefix():
    """Return ['xvfb-run', '--auto-servernum'] if no display and xvfb is available."""
    if os.environ.get("DISPLAY"):
        return []
    if shutil.which("xvfb-run"):
        return ["xvfb-run", "--auto-servernum"]
    return []


def test_walking_performance():
    # Project root is two levels above this test file (tests/ -> root)
    project_root = Path(__file__).resolve().parent.parent

    controller_script = "src/controllers/robust_controller_zmq.py"
    simulation_script = "src/simulation/aliengo_sim_zmq.py"

    controller_duration = 20.0   # slightly longer than simulation
    simulation_duration = 15.0

    # Set up environment with PYTHONPATH pointing to project root
    env = os.environ.copy()
    existing_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(project_root) + (os.pathsep + existing_pythonpath if existing_pythonpath else "")

    # 1. Start controller
    print(f"Starting controller ({controller_script})...")
    ctrl_proc = subprocess.Popen(
        [
            sys.executable,
            str(project_root / controller_script),
            "--duration", str(controller_duration),
            "--quiet"
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
        cwd=str(project_root),
    )

    # Give controller time to bind its sockets
    time.sleep(1.0)

    # 2. Start simulation (headless, with metrics output)
    sim_cmd = _xvfb_prefix() + [
        sys.executable,
        str(project_root / simulation_script),
        "--duration", str(simulation_duration),
        "--no-viewer",
        "--output-metrics",
    ]

    print(f"Starting simulation ({simulation_script}) for {simulation_duration}s...")
    sim_proc = subprocess.run(
        sim_cmd,
        capture_output=True,
        text=True,
        timeout=600,   # 10 minutes max
        env=env,
        cwd=str(project_root),
    )

    # 3. Check simulation exit code
    if sim_proc.returncode != 0:
        print("Simulation stderr:\n", sim_proc.stderr)
        raise AssertionError(f"Simulation exited with code {sim_proc.returncode}")

    # 4. Extract JSON metrics from stdout (last line starting with '{')
    lines = sim_proc.stdout.strip().splitlines()
    json_line = None
    for line in reversed(lines):
        line = line.strip()
        if line.startswith('{'):
            json_line = line
            break

    if json_line is None:
        print("No JSON line found in simulation stdout:\n", sim_proc.stdout)
        raise AssertionError("No JSON output from simulation")

    metrics = json.loads(json_line)
    max_roll = metrics["max_roll"]
    max_pitch = metrics["max_pitch"]
    final_speed = metrics["final_speed"]

    print(f"Metrics: max_roll={max_roll:.4f}, max_pitch={max_pitch:.4f}, final_speed={final_speed:.3f}")

    # 5. Assert performance criteria
    assert max_roll < 0.1, f"Roll too large: {max_roll}"
    assert max_pitch < 0.1, f"Pitch too large: {max_pitch}"
    assert final_speed > 1.5, f"Speed too low: {final_speed}"

    # 6. Wait for controller to finish
    try:
        ctrl_proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        ctrl_proc.kill()
        ctrl_proc.wait()
        print("Warning: controller had to be killed.")

    print("Integration test PASSED.")


if __name__ == "__main__":
    test_walking_performance()