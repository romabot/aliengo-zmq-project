"""Compact binary ZeroMQ protocol.

The protocol is intentionally close to the teacher's example:
ZeroMQ sends raw bytes, and struct.pack / struct.unpack convert them to numbers.
"""

import struct

STATE_ENDPOINT = "tcp://127.0.0.1:5555"
CMD_ENDPOINT = "tcp://127.0.0.1:5556"

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("thigh", "calf", "hip")

MOTOR_NAMES = (
    "FR_hip_motor", "FR_thigh_motor", "FR_calf_motor",
    "FL_hip_motor", "FL_thigh_motor", "FL_calf_motor",
    "RR_hip_motor", "RR_thigh_motor", "RR_calf_motor",
    "RL_hip_motor", "RL_thigh_motor", "RL_calf_motor",
)

BASE_KEYS = ("x", "z", "vx", "roll", "pitch", "yaw", "roll_rate", "pitch_rate")

STATE_FLOATS = len(BASE_KEYS) + len(LEGS) * len(JOINTS) * 3
CMD_FLOATS = len(MOTOR_NAMES) + 2

STATE_FORMAT = f"<Qdd{STATE_FLOATS}f"
CMD_FORMAT = f"<Q{CMD_FLOATS}f"

STATE_SIZE = struct.calcsize(STATE_FORMAT)
CMD_SIZE = struct.calcsize(CMD_FORMAT)


def pack_state(seq, t, dt, base_values, joint_values):
    return struct.pack(STATE_FORMAT, seq, t, dt, *(base_values + joint_values))


def unpack_state(msg):
    values = struct.unpack(STATE_FORMAT, msg)
    seq = values[0]
    t = values[1]
    dt = values[2]
    payload = values[3:]

    base = dict(zip(BASE_KEYS, payload[:len(BASE_KEYS)]))
    joint_payload = payload[len(BASE_KEYS):]

    joints = {}
    i = 0
    for leg in LEGS:
        joints[leg] = {}
        for joint in JOINTS:
            q, qd, bias = joint_payload[i:i + 3]
            joints[leg][joint] = {"q": q, "qd": qd, "bias": bias}
            i += 3

    return {"seq": seq, "t": t, "dt": dt, "base": base, "joints": joints}


def pack_command(seq, torques, roll_torque, pitch_torque):
    return struct.pack(CMD_FORMAT, seq, *(list(torques) + [roll_torque, pitch_torque]))


def unpack_command(msg):
    values = struct.unpack(CMD_FORMAT, msg)
    seq = values[0]
    payload = values[1:]
    torques = payload[:len(MOTOR_NAMES)]
    roll_torque = payload[-2]
    pitch_torque = payload[-1]
    return seq, torques, roll_torque, pitch_torque
