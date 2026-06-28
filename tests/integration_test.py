#!/usr/bin/env python3
"""
Integration test: runs controller and simulation, verifies that the robot
walks stably and reaches required speed.

The controller runs with a fixed duration (longer than simulation time).
The simulation runs headless for a given duration and prints JSON metrics.
Metrics are parsed and checked against the criteria:
    - max |roll| < 0.1 rad
    - max |pitch| < 0.1 rad
    - final forward speed > 1.5 m/s
"""

import json
import subprocess
import sys
import time


def test_walking_performance():
    # Paths to scripts (assumed to be in the same directory)
    controller_script = "src/controllers/robust_controller_zmq.py"
    simulation_script = "src/simulation/aliengo_sim_zmq.py"

    # Start controller with a duration slightly longer than simulation time
    # to ensure it stays alive throughout the simulation.
    controller_duration = 20.0
    simulation_duration = 15.0

    print(f"Starting controller ({controller_script})...")
    ctrl_proc = subprocess.Popen(
        [
            sys.executable, controller_script,
            "--duration", str(controller_duration),
            "--quiet"
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Allow controller to bind sockets
    time.sleep(1.0)

    print(f"Starting simulation ({simulation_script}) for {simulation_duration}s...")
    sim_proc = subprocess.run(
        [
            sys.executable, simulation_script,
            "--duration", str(simulation_duration),
            "--no-viewer",
            "--output-metrics",
        ],
        capture_output=True,
        text=True,
        timeout=600,  # 10 minutes max, adjust as needed
    )

    # Check simulation return code
    if sim_proc.returncode != 0:
        print("Simulation stderr:\n", sim_proc.stderr)
        raise AssertionError(f"Simulation exited with code {sim_proc.returncode}")

    # Parse JSON metrics
    try:
        metrics = json.loads(sim_proc.stdout.strip())
        max_roll = metrics["max_roll"]
        max_pitch = metrics["max_pitch"]
        final_speed = metrics["final_speed"]
    except (json.JSONDecodeError, KeyError) as e:
        print("Simulation stdout:\n", sim_proc.stdout)
        raise AssertionError(f"Failed to parse metrics: {e}")

    print(f"Metrics: max_roll={max_roll:.4f}, max_pitch={max_pitch:.4f}, final_speed={final_speed:.3f}")

    # Assert performance criteria
    assert max_roll < 0.1, f"Roll too large: {max_roll}"
    assert max_pitch < 0.1, f"Pitch too large: {max_pitch}"
    assert final_speed > 1.5, f"Speed too low: {final_speed}"

    # Wait for controller to finish (it will stop after its --duration)
    try:
        ctrl_proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        ctrl_proc.kill()
        ctrl_proc.wait()
        print("Warning: controller had to be killed; it may not have shut down cleanly.")

    print("Integration test PASSED.")


if __name__ == "__main__":
    test_walking_performance()