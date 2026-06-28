"""Plot CSV logs produced by the simulator."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def run(args):
    log_path = Path(args.log)
    if not log_path.exists():
        raise FileNotFoundError(f"Лог не найден: {log_path}")

    df = pd.read_csv(log_path)
    if df.empty:
        raise RuntimeError("Лог пустой")

    t = df["time"]
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Aliengo: gait graphs with external ZeroMQ controller", fontsize=14, fontweight="bold")

    axes[0].plot(t, df["speed_x"], label="Forward speed X")
    axes[0].axhline(0.0, linewidth=1.0, alpha=0.45)
    axes[0].set_ylabel("Speed, m/s")
    axes[0].legend()

    axes[1].plot(t, df["base_x"], label="Base X")
    axes[1].plot(t, df["base_z"], label="Base Z")
    axes[1].set_ylabel("Position, m")
    axes[1].legend()

    axes[2].plot(t, df["roll"], label="Roll")
    axes[2].plot(t, df["pitch"], label="Pitch")
    axes[2].plot(t, df["yaw"], label="Yaw")
    axes[2].set_ylabel("Angle, rad")
    axes[2].legend()

    axes[3].plot(t, df["mean_abs_torque"], label="Mean |motor torque|")
    axes[3].set_ylabel("Torque")
    axes[3].set_xlabel("Time, s")
    axes[3].legend()

    for ax in axes:
        ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output, dpi=200)
        print(f"График сохранен: {output}")
    else:
        plt.show()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", default="logs/sim_log.csv")
    parser.add_argument("--output", default=None)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
