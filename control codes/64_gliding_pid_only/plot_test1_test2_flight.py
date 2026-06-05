from pathlib import Path
import csv

import matplotlib.pyplot as plt
import numpy as np


DATA_DIR = Path(__file__).resolve().parent / "6-4-26 stair drops"
TEST1_PATH = DATA_DIR / "test1.csv"  # control ON
TEST2_PATH = DATA_DIR / "test2.csv"  # servos locked at 90 deg


def load_csv(path):
    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    if not rows:
        raise RuntimeError(f"No rows in {path}")

    data = {k: np.array([r[k] for r in rows], dtype=float) for k in rows[0]}
    data["t_s"] = (data["ms"] - data["ms"][0]) / 1000.0
    return data


def gyro_mag(data):
    return np.sqrt(data["gx_dps"] ** 2 + data["gy_dps"] ** 2 + data["gz_dps"] ** 2)


def print_summary(name, data):
    print(f"\n{name}")
    print(f"duration_s={data['t_s'][-1]:.2f}, rows={len(data['t_s'])}")
    print(f"roll_cf range={np.ptp(data['roll_cf_deg']):.1f} deg")
    print(f"pitch_cf range={np.ptp(data['pitch_cf_deg']):.1f} deg")
    print(f"max gyro={np.max(gyro_mag(data)):.1f} dps")
    print(f"a_norm range={np.min(data['a_norm_g']):.2f} to {np.max(data['a_norm_g']):.2f} g")
    print(f"left servo range={np.ptp(data['left_servo_deg']):.1f} deg")
    print(f"right servo range={np.ptp(data['right_servo_deg']):.1f} deg")


def plot_attitude(test1, test2):
    fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=False)
    fig.suptitle("Figure 1: Flight Attitude")

    ax[0].plot(test1["t_s"], test1["roll_cf_deg"], label="test1 roll cf")
    ax[0].plot(test1["t_s"], test1["pitch_cf_deg"], label="test1 pitch cf")
    ax[0].set_title("test1: control ON")
    ax[0].set_ylabel("angle [deg]")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend()

    ax[1].plot(test2["t_s"], test2["roll_cf_deg"], label="test2 roll cf")
    ax[1].plot(test2["t_s"], test2["pitch_cf_deg"], label="test2 pitch cf")
    ax[1].set_title("test2: servo locked at 90 deg")
    ax[1].set_xlabel("time since crop start [s]")
    ax[1].set_ylabel("angle [deg]")
    ax[1].grid(True, alpha=0.3)
    ax[1].legend()


def plot_control(test1, test2):
    fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=False)
    fig.suptitle("Control Output and Servo Motion")

    ax[0].plot(test1["t_s"], test1["u_roll_deg"], label="u roll")
    ax[0].plot(test1["t_s"], test1["u_pitch_deg"], label="u pitch")
    ax[0].plot(test1["t_s"], test1["left_servo_deg"] - 90.0, "--", label="left servo offset")
    ax[0].plot(test1["t_s"], test1["right_servo_deg"] - 90.0, "--", label="right servo offset")
    ax[0].set_title("test1: controller commands")
    ax[0].set_ylabel("command / servo offset [deg]")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend()

    ax[1].plot(test2["t_s"], test2["u_roll_deg"], label="u roll")
    ax[1].plot(test2["t_s"], test2["u_pitch_deg"], label="u pitch")
    ax[1].plot(test2["t_s"], test2["left_servo_deg"] - 90.0, "--", label="left servo offset")
    ax[1].plot(test2["t_s"], test2["right_servo_deg"] - 90.0, "--", label="right servo offset")
    ax[1].set_title("test2: locked servo check")
    ax[1].set_xlabel("time since crop start [s]")
    ax[1].set_ylabel("command / servo offset [deg]")
    ax[1].grid(True, alpha=0.3)
    ax[1].legend()


def plot_servo_output(test1, test2):
    fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=False)
    fig.suptitle("Figure 2: Servo Output")

    ax[0].plot(test1["t_s"], test1["left_servo_deg"], label="left servo")
    ax[0].plot(test1["t_s"], test1["right_servo_deg"], label="right servo")
    ax[0].axhline(90.0, color="k", linestyle=":", linewidth=1, label="neutral 90")
    ax[0].set_title("test1: control ON")
    ax[0].set_ylabel("servo angle [deg]")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend()

    ax[1].plot(test2["t_s"], test2["left_servo_deg"], label="left servo")
    ax[1].plot(test2["t_s"], test2["right_servo_deg"], label="right servo")
    ax[1].axhline(90.0, color="k", linestyle=":", linewidth=1, label="neutral 90")
    ax[1].set_title("test2: servo locked at 90 deg")
    ax[1].set_xlabel("time since crop start [s]")
    ax[1].set_ylabel("servo angle [deg]")
    ax[1].grid(True, alpha=0.3)
    ax[1].legend()


def plot_imu(test1, test2):
    fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=False)
    fig.suptitle("Figure 3: IMU Motion Intensity")

    ax[0].plot(test1["t_s"], gyro_mag(test1), label="gyro magnitude")
    ax[0].plot(test1["t_s"], test1["a_norm_g"] * 100.0, label="a_norm x100")
    ax[0].set_title("test1: control ON")
    ax[0].set_ylabel("dps / scaled g")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend()

    ax[1].plot(test2["t_s"], gyro_mag(test2), label="gyro magnitude")
    ax[1].plot(test2["t_s"], test2["a_norm_g"] * 100.0, label="a_norm x100")
    ax[1].set_title("test2: servo locked at 90 deg")
    ax[1].set_xlabel("time since crop start [s]")
    ax[1].set_ylabel("dps / scaled g")
    ax[1].grid(True, alpha=0.3)
    ax[1].legend()


def plot_roll_pitch_control_relation(test1, test2):
    fig, ax = plt.subplots(2, 2, figsize=(14, 9), sharex=False)
    fig.suptitle("Figure 4: Controller Response")

    ax[0, 0].plot(test1["t_s"], test1["roll_cf_deg"], label="roll cf")
    ax[0, 0].plot(test1["t_s"], test1["u_roll_deg"], label="u roll")
    ax[0, 0].set_title("test1 roll")
    ax[0, 0].set_ylabel("roll / command [deg]")
    ax[0, 0].grid(True, alpha=0.3)
    ax[0, 0].legend()

    ax[1, 0].plot(test1["t_s"], test1["pitch_cf_deg"], label="pitch cf")
    ax[1, 0].plot(test1["t_s"], test1["u_pitch_deg"], label="u pitch")
    ax[1, 0].set_title("test1 pitch")
    ax[1, 0].set_xlabel("time since crop start [s]")
    ax[1, 0].set_ylabel("pitch / command [deg]")
    ax[1, 0].grid(True, alpha=0.3)
    ax[1, 0].legend()

    ax[0, 1].plot(test2["t_s"], test2["roll_cf_deg"], label="roll cf")
    ax[0, 1].plot(test2["t_s"], test2["u_roll_deg"], label="u roll")
    ax[0, 1].set_title("test2 roll")
    ax[0, 1].set_ylabel("roll / command [deg]")
    ax[0, 1].grid(True, alpha=0.3)
    ax[0, 1].legend()

    ax[1, 1].plot(test2["t_s"], test2["pitch_cf_deg"], label="pitch cf")
    ax[1, 1].plot(test2["t_s"], test2["u_pitch_deg"], label="u pitch")
    ax[1, 1].set_title("test2 pitch")
    ax[1, 1].set_xlabel("time since crop start [s]")
    ax[1, 1].set_ylabel("pitch / command [deg]")
    ax[1, 1].grid(True, alpha=0.3)
    ax[1, 1].legend()


def main():
    test1 = load_csv(TEST1_PATH)
    test2 = load_csv(TEST2_PATH)

    print_summary("test1 control ON", test1)
    print_summary("test2 servo locked", test2)

    plot_attitude(test1, test2)
    plot_servo_output(test1, test2)
    plot_imu(test1, test2)
    plot_roll_pitch_control_relation(test1, test2)

    plt.show()


if __name__ == "__main__":
    main()
