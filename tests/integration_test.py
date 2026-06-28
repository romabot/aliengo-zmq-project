#!/usr/bin/env python3
"""Integration test with fixed PYTHONPATH for subprocesses."""

import json
import os
import subprocess
import sys
import time
from pathlib import Path


def test_walking_performance():
    # Корень проекта (на две папки выше tests/)
    project_root = Path(__file__).resolve().parent.parent

    controller_script = "src/controllers/robust_controller_zmq.py"
    simulation_script = "src/simulation/aliengo_sim_zmq.py"

    controller_duration = 20.0
    simulation_duration = 15.0

    # Базовое окружение с добавленным PYTHONPATH
    env = os.environ.copy()
    existing_path = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(project_root) + (os.pathsep + existing_path if existing_path else "")

    print(f"Starting controller ({controller_script})...")
    ctrl_proc = subprocess.Popen(
        [sys.executable, str(project_root / controller_script),
         "--duration", str(controller_duration),
         "--quiet"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,                     # <-- передаём окружение
        cwd=str(project_root),       # рабочая папка — корень проекта (на всякий случай)
    )

    time.sleep(1.0)

    print(f"Starting simulation ({simulation_script}) for {simulation_duration}s...")
    sim_proc = subprocess.run(
        ["xvfb-run", "--auto-servernum", sys.executable, str(project_root / simulation_script),
        "--duration", str(simulation_duration),
        "--no-viewer",
        "--output-metrics"],
        capture_output=True,
        text=True,
        timeout=600,
        env=env,
        cwd=str(project_root),
    )

    if sim_proc.returncode != 0:
        print("Simulation stderr:\n", sim_proc.stderr)
        raise AssertionError(f"Simulation exited with code {sim_proc.returncode}")

    print("Simulation stdout:", repr(sim_proc.stdout))   # отладка
    print("Simulation stderr:", repr(sim_proc.stderr))
    lines = sim_proc.stdout.strip().splitlines()
    json_line = None
    for line in lines:
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

    assert max_roll < 0.1, f"Roll too large: {max_roll}"
    assert max_pitch < 0.1, f"Pitch too large: {max_pitch}"
    assert final_speed > 1.5, f"Speed too low: {final_speed}"

    try:
        ctrl_proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        ctrl_proc.kill()
        ctrl_proc.wait()
        print("Warning: controller had to be killed.")

    print("Integration test PASSED.")


if __name__ == "__main__":
    test_walking_performance()