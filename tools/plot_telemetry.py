"""Generate engineering plots from motion_simulator telemetry."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_csv(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No telemetry rows found in {path}")

    numeric_columns = [name for name in rows[0] if name != "fault"]
    return {
        name: [float(row[name]) for row in rows]
        for name in numeric_columns
    }


def save_plot(data: dict[str, list[float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    time = data["time"]
    position_error = [
        math.hypot(tx - x, ty - y)
        for tx, ty, x, y in zip(
            data["target_x"], data["target_y"], data["odom_x"], data["odom_y"]
        )
    ]

    figure, axes = plt.subplots(2, 2, figsize=(13, 9))

    path_axis = axes[0, 0]
    path_axis.plot(data["true_x"], data["true_y"], label="True path", linewidth=2)
    path_axis.plot(
        data["odom_x"], data["odom_y"], "--", label="Encoder odometry", linewidth=1.5
    )
    path_axis.scatter(
        data["target_x"], data["target_y"], s=8, alpha=0.15, label="Active waypoint"
    )
    path_axis.set_title("Planar trajectory and odometry")
    path_axis.set_xlabel("x [m]")
    path_axis.set_ylabel("y [m]")
    path_axis.axis("equal")
    path_axis.grid(alpha=0.3)
    path_axis.legend()

    speed_axis = axes[0, 1]
    speed_axis.plot(time, data["left_setpoint"], "--", label="Left setpoint")
    speed_axis.plot(time, data["left_speed"], label="Left measured")
    speed_axis.plot(time, data["right_setpoint"], "--", label="Right setpoint")
    speed_axis.plot(time, data["right_speed"], label="Right measured")
    speed_axis.set_title("Wheel-speed tracking")
    speed_axis.set_xlabel("time [s]")
    speed_axis.set_ylabel("angular velocity [rad/s]")
    speed_axis.grid(alpha=0.3)
    speed_axis.legend(ncol=2, fontsize=8)

    effort_axis = axes[1, 0]
    effort_axis.plot(time, data["left_voltage"], label="Left voltage")
    effort_axis.plot(time, data["right_voltage"], label="Right voltage")
    load_axis = effort_axis.twinx()
    load_axis.fill_between(
        time, data["right_load"], alpha=0.25, color="tab:red", label="Right load"
    )
    effort_axis.set_title("Control effort and load disturbance")
    effort_axis.set_xlabel("time [s]")
    effort_axis.set_ylabel("voltage [V]")
    load_axis.set_ylabel("load torque [N.m]")
    effort_axis.grid(alpha=0.3)
    effort_lines, effort_labels = effort_axis.get_legend_handles_labels()
    load_lines, load_labels = load_axis.get_legend_handles_labels()
    effort_axis.legend(effort_lines + load_lines, effort_labels + load_labels)

    error_axis = axes[1, 1]
    error_axis.plot(time, position_error, label="Waypoint distance")
    error_axis.plot(
        time,
        [abs(a - b) for a, b in zip(data["true_heading"], data["odom_heading"])],
        label="Heading estimation error",
    )
    error_axis.set_title("Tracking and estimation errors")
    error_axis.set_xlabel("time [s]")
    error_axis.set_ylabel("error")
    error_axis.grid(alpha=0.3)
    error_axis.legend()

    figure.suptitle("Embedded motion-control telemetry")
    figure.tight_layout()
    figure.savefig(output, dpi=170, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry", type=Path)
    parser.add_argument("--output", type=Path, default=Path("docs/motion_control.png"))
    args = parser.parse_args()
    save_plot(load_csv(args.telemetry), args.output)
    print(f"Plot: {args.output}")


if __name__ == "__main__":
    main()

