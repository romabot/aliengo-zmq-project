"""External ZeroMQ controller with PD control only.

The file name is kept for compatibility with the original run command:
python -m src.controllers.robust_controller_zmq

The control law is only:
    tau = KP * (q_des - q) - KD * qd
"""

import argparse
import math
import signal
import time

import numpy as np
import zmq

from src.common.constants import (
    STAND_FOOT_Y,
    T,
    STEP_HEIGHT,
    STEP_LENGTH,
    WALK_START,
    RAMP_UP_TIME,
    WALK_END,
    RAMP_DOWN_TIME,
    MIN_GAIT_SCALE,
    HIP_ROLL_SWAY_AMPLITUDE,
    MAX_FREE_TORQUE,
)
from src.common.kinematics import inverse_kinematics, smoothstep
from src.common.protocol import (
    STATE_ENDPOINT,
    CMD_ENDPOINT,
    LEGS,
    MOTOR_NAMES,
    pack_command,
    unpack_state,
)


if hasattr(signal, "SIGPIPE"):
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)


class PDAliengoController:
    def __init__(self):
        # Gains are ordered as thigh, calf, hip because state["joints"] uses this order.
        self.kp = np.array([220.0, 220.0, 180.0], dtype=float)
        self.kd = np.array([2.5, 2.5, 2.0], dtype=float)

    def gait_scale_at(self, t):
        if t < WALK_START:
            return 0.0
        if t < WALK_START + RAMP_UP_TIME:
            return smoothstep((t - WALK_START) / RAMP_UP_TIME)
        if t < WALK_END:
            return 1.0
        if t < WALK_END + RAMP_DOWN_TIME:
            return 1.0 - smoothstep((t - WALK_END) / RAMP_DOWN_TIME)
        return 0.0

    def desired_pose(self, t):
        gait_scale = self.gait_scale_at(t)
        q1_des, q2_des, q3_des = {}, {}, {}

        if gait_scale < MIN_GAIT_SCALE:
            for leg in LEGS:
                q1_des[leg], q2_des[leg] = inverse_kinematics(0.0, STAND_FOOT_Y)
                q3_des[leg] = 0.0
            return q1_des, q2_des, q3_des, 1.0

        leg_phase = (t - WALK_START - RAMP_UP_TIME) % T
        amp = smoothstep(gait_scale)
        step_l = STEP_LENGTH * amp
        step_h = STEP_HEIGHT * amp
        k = -step_h / ((0.5 * step_l) ** 2)

        if leg_phase <= 0.5 * T:
            x1 = 0.5 * step_l - 2.0 * leg_phase * step_l / T
            y1 = k * (x1 ** 2) + step_h
            x2 = -0.5 * step_l + 2.0 * leg_phase * step_l / T
            y2 = 0.0
        else:
            x1 = -0.5 * step_l + 2.0 * (leg_phase - 0.5 * T) * step_l / T
            y1 = 0.0
            x2 = 0.5 * step_l - 2.0 * (leg_phase - 0.5 * T) * step_l / T
            y2 = k * (x2 ** 2) + step_h

        q1_des["FR"], q2_des["FR"] = inverse_kinematics(x1, STAND_FOOT_Y - y1)
        q1_des["RL"], q2_des["RL"] = inverse_kinematics(x1, STAND_FOOT_Y - y1)
        q1_des["FL"], q2_des["FL"] = inverse_kinematics(x2, STAND_FOOT_Y - y2)
        q1_des["RR"], q2_des["RR"] = inverse_kinematics(x2, STAND_FOOT_Y - y2)

        roll_sway = HIP_ROLL_SWAY_AMPLITUDE * gait_scale * math.sin(
            2.0 * math.pi * ((t - WALK_START - RAMP_UP_TIME) / T)
        )
        q3_des["FL"] = roll_sway
        q3_des["RL"] = roll_sway
        q3_des["FR"] = -roll_sway
        q3_des["RR"] = -roll_sway

        return q1_des, q2_des, q3_des, 1.0

    def leg_torque_pd(self, leg, q_des, state, k_tau):
        q = np.array([
            state["joints"][leg]["thigh"]["q"],
            state["joints"][leg]["calf"]["q"],
            state["joints"][leg]["hip"]["q"],
        ], dtype=float)

        qd = np.array([
            state["joints"][leg]["thigh"]["qd"],
            state["joints"][leg]["calf"]["qd"],
            state["joints"][leg]["hip"]["qd"],
        ], dtype=float)

        q_des = np.array(q_des, dtype=float)
        tau = k_tau * (self.kp * (q_des - q) - self.kd * qd)
        return np.clip(tau, -MAX_FREE_TORQUE, MAX_FREE_TORQUE)

    def compute(self, state):
        q1_des, q2_des, q3_des, k_tau = self.desired_pose(float(state["t"]))

        tau_by_motor = {}
        for leg in ("FL", "FR", "RR", "RL"):
            tau = self.leg_torque_pd(
                leg,
                [q1_des[leg], q2_des[leg], q3_des[leg]],
                state,
                k_tau,
            )
            tau_by_motor[f"{leg}_thigh_motor"] = float(tau[0])
            tau_by_motor[f"{leg}_calf_motor"] = float(tau[1])
            tau_by_motor[f"{leg}_hip_motor"] = float(tau[2])

        torques = [tau_by_motor[name] for name in MOTOR_NAMES]

        # Body torques are disabled because this version must be PD joint control only.
        roll_torque = 0.0
        pitch_torque = 0.0
        return torques, roll_torque, pitch_torque


def run(args):
    ctx = zmq.Context()
    sock_state = ctx.socket(zmq.SUB)
    sock_state.connect(args.state_endpoint)
    sock_state.setsockopt(zmq.SUBSCRIBE, b"")
    sock_cmd = ctx.socket(zmq.PUB)
    sock_cmd.bind(args.cmd_endpoint)

    controller = PDAliengoController()
    if not args.quiet:
        print("PD controller: waiting for state packets")
    time.sleep(0.2)

    start_time = time.monotonic()
    while True:
        # Ограничение по времени (если задано)
        if args.duration is not None and (time.monotonic() - start_time) >= args.duration:
            if not args.quiet:
                print(f"Controller finished after {args.duration} s")
            break

        #try:
           # msg = sock_state.recv(zmq.NOBLOCK)
         #
         #    state = unpack_state(msg)
        #except zmq.Again:
            # Небольшая пауза, чтобы не гонять цикл вхолостую
            #time.sleep(0.0001)
            #continue
        

        try:
            msg = sock_state.recv(zmq.NOBLOCK)
            state = unpack_state(msg)
            if state["seq"] % 100 == 0:
                print(f"Got state seq={state['seq']}, t={state['t']:.2f}")
        except zmq.Again:
            continue

        torques, roll_torque, pitch_torque = controller.compute(state)
        cmd_msg = pack_command(state["seq"], torques, roll_torque, pitch_torque)
        sock_cmd.send(cmd_msg, zmq.NOBLOCK)

        if args.verbose and state["seq"] % 500 == 0:
            base = state["base"]
            print(
                f"seq={state['seq']} "
                f"t={state['t']:.2f} "
                f"x={base['x']:.2f} "
                f"vx={base['vx']:.2f}"
            )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-endpoint", default=STATE_ENDPOINT)
    parser.add_argument("--cmd-endpoint", default=CMD_ENDPOINT)
    parser.add_argument("--verbose", action="store_true")
    #parser.add_argument("--quiet", action="store_true", help="Suppress status messages")
    parser.add_argument("--duration", type=float, default=None,
                        help="Run controller for a fixed number of seconds then exit")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
