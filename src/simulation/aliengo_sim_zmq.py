"""MuJoCo simulator for Aliengo with an external ZeroMQ controller."""

import argparse
import csv
import time
from pathlib import Path

import mujoco
import numpy as np
import zmq
import sys

from src.common.constants import (
    BASE_START_Z,
    STAND_FOOT_Y,
    MAX_ROLL_ANGLE,
    MAX_ROLL_RATE,
    MAX_FREE_TORQUE,
    COMMAND_TIMEOUT_SEC,
)
from src.common.kinematics import quat_to_euler_xyz, euler_xyz_to_quat, inverse_kinematics
from src.common.protocol import (
    STATE_ENDPOINT,
    CMD_ENDPOINT,
    LEGS,
    JOINTS,
    MOTOR_NAMES,
    pack_state,
    unpack_command,
)

try:
    import mujoco.viewer
except Exception:
    mujoco_viewer = None
else:
    mujoco_viewer = mujoco.viewer


class AliengoSim:
    def __init__(self, xml_path):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.free_joint_id = self.find_free_joint()
        self.base_qpos_adr = self.model.jnt_qposadr[self.free_joint_id]
        self.base_qvel_adr = self.model.jnt_dofadr[self.free_joint_id]
        self.base_body_id = self.model.jnt_bodyid[self.free_joint_id]
        self.start_time = self.data.time
        self.last_cmd_time = None
        self.last_cmd = None
        self.set_initial_stand_pose()

    def find_free_joint(self):
        for jid in range(self.model.njnt):
            if self.model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
                return jid
        raise RuntimeError("В XML модели не найден freejoint базы робота")

    def base_euler(self):
        q = self.data.qpos[self.base_qpos_adr + 3:self.base_qpos_adr + 7].copy()
        return quat_to_euler_xyz(q)

    def enforce_2d_state(self):
        roll, pitch, _ = self.base_euler()
        roll = np.clip(roll, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE)

        self.data.qpos[self.base_qpos_adr + 1] = 0.0
        self.data.qpos[self.base_qpos_adr + 3:self.base_qpos_adr + 7] = euler_xyz_to_quat(roll, pitch, 0.0)
        self.data.qvel[self.base_qvel_adr + 1] = 0.0
        self.data.qvel[self.base_qvel_adr + 3] = np.clip(self.data.qvel[self.base_qvel_adr + 3], -MAX_ROLL_RATE, MAX_ROLL_RATE)
        self.data.qvel[self.base_qvel_adr + 5] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def set_initial_stand_pose(self):
        self.data.qpos[self.base_qpos_adr + 0] = 0.0
        self.data.qpos[self.base_qpos_adr + 1] = 0.0
        self.data.qpos[self.base_qpos_adr + 2] = BASE_START_Z
        self.data.qpos[self.base_qpos_adr + 3:self.base_qpos_adr + 7] = euler_xyz_to_quat(0.0, 0.0, 0.0)
        stand_q1, stand_q2 = inverse_kinematics(0.0, STAND_FOOT_Y)
        for leg in LEGS:
            self.data.qpos[self.model.joint(f"{leg}_thigh_joint").qposadr[0]] = stand_q1
            self.data.qpos[self.model.joint(f"{leg}_calf_joint").qposadr[0]] = stand_q2
            self.data.qpos[self.model.joint(f"{leg}_hip_joint").qposadr[0]] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.enforce_2d_state()
        for _ in range(80):
            mujoco.mj_step(self.model, self.data)
            self.data.qvel[:] *= 0.5
            self.enforce_2d_state()
        self.data.qvel[:] = 0.0
        self.start_time = self.data.time

    def base_values(self):
        roll, pitch, yaw = self.base_euler()
        return [
            float(self.data.qpos[self.base_qpos_adr + 0]),
            float(self.data.qpos[self.base_qpos_adr + 2]),
            float(self.data.qvel[self.base_qvel_adr + 0]),
            float(roll),
            float(pitch),
            float(yaw),
            float(self.data.qvel[self.base_qvel_adr + 3]),
            float(self.data.qvel[self.base_qvel_adr + 4]),
        ]

    def joint_values(self):
        values = []
        for leg in LEGS:
            for joint in JOINTS:
                joint_name = f"{leg}_{joint}_joint"
                qadr = self.model.joint(joint_name).qposadr[0]
                dadr = self.model.joint(joint_name).dofadr[0]
                values += [
                    float(self.data.qpos[qadr]),
                    float(self.data.qvel[dadr]),
                    float(self.data.qfrc_bias[dadr]),
                ]
        return values

    def state_packet(self, seq):
        return pack_state(
            seq,
            float(self.data.time - self.start_time),
            float(self.model.opt.timestep),
            self.base_values(),
            self.joint_values(),
        )

    def apply_command(self, cmd):
        if cmd is None:
            self.data.ctrl[:] = 0.0
            self.data.xfrc_applied[:, :] = 0.0
            return

        _, torques, roll_torque, pitch_torque = cmd
        self.data.ctrl[:] = 0.0
        for motor_name, tau in zip(MOTOR_NAMES, torques):
            actuator_id = self.model.actuator(motor_name).id
            if self.model.actuator_ctrllimited[actuator_id]:
                low, high = self.model.actuator_ctrlrange[actuator_id]
                tau = np.clip(tau, low, high)
            else:
                tau = np.clip(tau, -MAX_FREE_TORQUE, MAX_FREE_TORQUE)
            self.data.ctrl[actuator_id] = tau

        self.data.xfrc_applied[:, :] = 0.0
        self.data.xfrc_applied[self.base_body_id, 3] = roll_torque
        self.data.xfrc_applied[self.base_body_id, 4] = pitch_torque

    def command_for_step(self):
        if self.last_cmd_time is None:
            return None
        if time.monotonic() - self.last_cmd_time > COMMAND_TIMEOUT_SEC:
            return None
        return self.last_cmd

    def update_camera(self, viewer):
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = self.base_body_id
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -18.0

    def log_row(self, seq):
        base = self.base_values()
        return {
            "seq": seq,
            "time": float(self.data.time - self.start_time),
            "speed_x": base[2],
            "base_x": base[0],
            "base_z": base[1],
            "roll": base[3],
            "pitch": base[4],
            "yaw": base[5],
            "mean_abs_torque": float(np.mean(np.abs(self.data.ctrl))),
            "has_command": int(self.command_for_step() is not None),
        }


def run(args):
    sim = AliengoSim(Path(args.xml))

    ctx = zmq.Context()
    sock_state = ctx.socket(zmq.PUB)
    sock_state.bind(args.state_endpoint)
    sock_cmd = ctx.socket(zmq.SUB)
    sock_cmd.connect(args.cmd_endpoint)
    sock_cmd.setsockopt(zmq.SUBSCRIBE, b"")

    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    print("simulator: waiting 0.3 s for ZeroMQ connections")
    time.sleep(0.3)

    seq = 0

    rolls = []
    pitches = []
    speeds = []


    def step(viewer=None):
        nonlocal seq
        sim.enforce_2d_state()
        sock_state.send(sim.state_packet(seq), zmq.NOBLOCK)
        got_cmd = False
        
        while True:
            try:
                msg = sock_cmd.recv(zmq.NOBLOCK)
                sim.last_cmd = unpack_command(msg)
                sim.last_cmd_time = time.monotonic()
                got_cmd = True
            except zmq.Again:
                break
        if not got_cmd and sim.data.time - sim.start_time > 2.0:
            print(f"t={sim.data.time:.1f}: no command received yet")


        sim.apply_command(sim.command_for_step())
        mujoco.mj_step(sim.model, sim.data)
        sim.enforce_2d_state()

        if viewer is not None:
            sim.update_camera(viewer)
            viewer.sync()

        row = sim.log_row(seq)
        seq += 1
        return row

    fieldnames = ["seq", "time", "speed_x", "base_x", "base_z",
                  "roll", "pitch", "yaw", "mean_abs_torque", "has_command"]
    with log_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        if args.no_viewer:
            while sim.data.time - sim.start_time < args.duration:
                row = step()
                writer.writerow(row)
                rolls.append(row["roll"])
                pitches.append(row["pitch"])
                speeds.append(row["speed_x"])
                if args.real_time:
                    time.sleep(sim.model.opt.timestep)
        else:
            if mujoco_viewer is None:
                raise RuntimeError("mujoco.viewer недоступен. Запустите с --no-viewer")
            with mujoco_viewer.launch_passive(sim.model, sim.data) as viewer:
                while viewer.is_running() and sim.data.time - sim.start_time < args.duration:
                    row = step(viewer)
                    writer.writerow(row)
                    rolls.append(row["roll"])
                    pitches.append(row["pitch"])
                    speeds.append(row["speed_x"])
                    if args.real_time:
                        time.sleep(sim.model.opt.timestep)

    print(f"simulator: log saved to {log_path}")

    if args.output_metrics:
        import json
        metrics = {
            "max_roll": max(abs(r) for r in rolls) if rolls else 0.0,
            "max_pitch": max(abs(p) for p in pitches) if pitches else 0.0,
            "final_speed": speeds[-1] if speeds else 0.0,
            "duration": sim.data.time - sim.start_time
        }
        print(json.dumps(metrics))
        sys.stdout.flush()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", default="robot.xml")
    parser.add_argument("--duration", type=float, default=16.0)
    parser.add_argument("--log", default="logs/sim_log.csv")
    parser.add_argument("--state-endpoint", default=STATE_ENDPOINT)
    parser.add_argument("--cmd-endpoint", default=CMD_ENDPOINT)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--real-time", action="store_true", default=True)
    parser.add_argument("--no-real-time", dest="real_time", action="store_false")
    parser.add_argument("--output-metrics", action="store_true",
                    help="Print JSON metrics to stdout")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
