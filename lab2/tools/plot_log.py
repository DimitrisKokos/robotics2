#!/usr/bin/env python3
"""Capture, summarize and plot a pendulum run.

Reads the CSV format produced by the Arduino and pendulum simulation:

    t_s,theta_rad,omega_rad_s,torque_Nm,voltage_V,mode

Mode 0 = swing-up / motor off; mode 1 = balance.

Examples
--------
Capture from an Arduino:
    python plot_log_improved.py --port COM3
    python plot_log_improved.py --port /dev/ttyUSB0 --out run.csv

Plot a previously saved CSV:
    python plot_log_improved.py --csv run.csv
    python plot_log_improved.py --csv run.csv --save-fig run.png

Requirements: numpy, matplotlib; pyserial is only needed for --port.
"""

import argparse
import sys
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle

HEADER = "t_s,theta_rad,omega_rad_s,torque_Nm,voltage_V,mode"


def parse_row(line):
    """Return one six-column numeric row, or None for headers/messages."""
    parts = line.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        row = [float(p) for p in parts]
    except ValueError:
        return None
    if not np.all(np.isfinite(row)):
        return None
    return row


def capture(port, baud):
    """Read CSV rows from the Arduino; send 's' after #READY is received."""
    try:
        import serial  # pyserial
    except ImportError as exc:
        raise SystemExit("pyserial is required for --port. Install it with: pip install pyserial") from exc

    rows = []
    started = False

    with serial.Serial(port, baud, timeout=1) as ser:
        print(f"Listening on {port} at {baud} baud (Ctrl+C sends 'x' to stop the motor)...")
        try:
            while True:
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    continue

                if line.startswith("#READY") and not started:
                    print(line)
                    input(">>> Pendulum hanging at rest and the area clear? Press Enter to start... ")
                    ser.write(b"s")
                    started = True
                    continue

                row = parse_row(line)
                if row is not None:
                    rows.append(row)
                    continue

                # Arduino status lines and CSV header are not data rows.
                print(line)
                if line.startswith(("#DONE", "#ABORTED")):
                    # Allow any final rows/messages already in the serial buffer to arrive.
                    for _ in range(5):
                        extra = ser.readline().decode(errors="replace").strip()
                        if not extra:
                            continue
                        extra_row = parse_row(extra)
                        if extra_row is not None:
                            rows.append(extra_row)
                        else:
                            print(extra)
                    break
        except KeyboardInterrupt:
            print("\nStopped by the user; sending 'x' to command zero voltage.")
            try:
                ser.write(b"x")
            except serial.SerialException:
                pass

    return np.asarray(rows, dtype=float).reshape((-1, 6))


def load_csv(path):
    """Load valid six-column rows, ignoring comments and non-data lines."""
    rows = []
    with open(path, "r", encoding="utf-8-sig") as f:
        for line in f:
            if line.lstrip().startswith("#"):
                continue
            row = parse_row(line)
            if row is not None:
                rows.append(row)
    return np.asarray(rows, dtype=float).reshape((-1, 6))


def save_csv(path, data):
    """Save captured data with the standard CSV header."""
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(HEADER + "\n")
        for r in data:
            f.write(
                f"{r[0]:.6f},{r[1]:.6f},{r[2]:.6f},"
                f"{r[3]:.8f},{r[4]:.6f},{int(round(r[5]))}\n"
            )
    print(f"Data saved to {path}")


def wrap(angle):
    """Wrap angles to [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def validate_data(data):
    """Check the array shape and remove any non-finite rows."""
    data = np.asarray(data, dtype=float)
    if data.ndim != 2 or data.shape[1] != 6:
        raise ValueError("Expected data with six columns: " + HEADER)
    finite = np.all(np.isfinite(data), axis=1)
    removed = int(np.count_nonzero(~finite))
    if removed:
        print(f"Warning: ignored {removed} row(s) containing NaN or infinity.")
    data = data[finite]
    if len(data) == 0:
        raise ValueError("No valid data rows were found.")
    if len(data) > 1 and np.any(np.diff(data[:, 0]) <= 0):
        print("Warning: timestamps are not strictly increasing; plots use the rows in file order.")
    return data


def _time_fraction_in_mode(t, mode_mask):
    """Approximate fraction of elapsed time spent in a mode (ZOH per sample)."""
    if len(t) < 2:
        return float(mode_mask[0]) if len(mode_mask) else 0.0
    dt = np.diff(t)
    valid = dt > 0
    if not np.any(valid):
        return float(np.mean(mode_mask))
    # Attribute each interval to the mode recorded at its starting sample.
    return float(np.sum(dt[valid] * mode_mask[:-1][valid]) / np.sum(dt[valid]))


def _mode_intervals(t, mask):
    """Return (start_index, end_index) intervals for contiguous True samples."""
    if len(mask) == 0:
        return []
    edges = np.diff(np.r_[False, mask, False].astype(int))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)  # exclusive
    intervals = []
    for start, end in zip(starts, ends):
        left = t[start]
        right = t[end] if end < len(t) else t[-1]
        intervals.append((start, end, left, right))
    return intervals


def summary(data):
    """Print run statistics, including capture/loss events and balance quality."""
    t, theta, omega, tau, volt, mode = data.T
    phi = wrap(theta - np.pi)
    balance = mode > 0.5
    duration = max(0.0, float(t[-1] - t[0]))
    sample_dt = np.diff(t)
    positive_dt = sample_dt[sample_dt > 0]

    print("\n========================= Run summary =========================")
    print(f"Duration: {duration:.2f} s   Samples: {len(t)}")
    if len(positive_dt):
        print(f"Sample period: median {np.median(positive_dt):.4f} s "
              f"({1.0 / np.median(positive_dt):.1f} Hz), "
              f"range {np.min(positive_dt):.4f}–{np.max(positive_dt):.4f} s")

    if not balance.any():
        # Wrapped angle measures the orientation relative to hanging without counting turns.
        theta_wrapped = wrap(theta)
        largest_departure = float(np.max(np.abs(theta_wrapped)))
        print("The pendulum was never captured (the balance controller did not start).")
        print(f"Largest wrapped departure from hanging: {np.degrees(largest_departure):.1f} deg")
        print(f"Peak |angular velocity|: {np.max(np.abs(omega)):.2f} rad/s")
    else:
        capture_indices = np.flatnonzero(balance & ~np.r_[False, balance[:-1]])
        loss_indices = np.flatnonzero(~balance & np.r_[False, balance[:-1]])
        first = int(capture_indices[0])
        print(f"First capture: t = {t[first]:.2f} s "
              f"(|upright error| = {np.degrees(abs(phi[first])):.1f} deg, "
              f"|omega| = {abs(omega[first]):.2f} rad/s)")
        print(f"Capture events: {len(capture_indices)}   Balance losses: {len(loss_indices)}")
        print(f"Time in balance mode: {_time_fraction_in_mode(t, balance) * 100:.1f}% of elapsed time")

        tail_start = t[-1] - 5.0
        tail = balance & (t >= tail_start)
        if tail.any():
            angle_rms = np.degrees(np.sqrt(np.mean(phi[tail] ** 2)))
            angle_max = np.degrees(np.max(np.abs(phi[tail])))
            torque_rms = np.sqrt(np.mean(tau[tail] ** 2)) * 1e3
            print(f"Balance samples in final 5 s: {np.count_nonzero(tail)}")
            print(f"  Upright error: RMS {angle_rms:.2f} deg, max {angle_max:.2f} deg")
            print(f"  Torque:        RMS {torque_rms:.2f} mN·m")
        else:
            print("No balance-mode samples in the final 5 s.")

    print(f"Peak |torque|: {np.max(np.abs(tau)) * 1e3:.2f} mN·m")
    print(f"Peak |voltage|: {np.max(np.abs(volt)):.2f} V")
    print("===============================================================\n")


def _shade_balance(ax, t, balance):
    """Shade balance-mode intervals without adding repeated legend entries."""
    for _, _, left, right in _mode_intervals(t, balance):
        if right > left:
            ax.axvspan(left, right, color="tab:green", alpha=0.10, lw=0, zorder=0)


def _add_transition_lines(ax, t, balance):
    """Mark transitions into and out of balance mode."""
    capture = np.flatnonzero(balance & ~np.r_[False, balance[:-1]])
    loss = np.flatnonzero(~balance & np.r_[False, balance[:-1]])
    for i in capture:
        ax.axvline(t[i], color="tab:green", ls=":", lw=0.9, alpha=0.65, zorder=1)
    for i in loss:
        ax.axvline(t[i], color="tab:red", ls=":", lw=0.9, alpha=0.65, zorder=1)


def _add_direction_arrows(ax, x, y, max_arrows=12):
    """Add a small number of arrows to show the direction of phase motion."""
    n = len(x)
    if n < 3:
        return
    # Pick spaced indices and use local differences so arrows follow the trajectory.
    candidates = np.linspace(1, n - 2, min(max_arrows, n - 2), dtype=int)
    for i in np.unique(candidates):
        dx = x[i + 1] - x[i - 1]
        dy = y[i + 1] - y[i - 1]
        # A wrapped-angle jump is a coordinate discontinuity, not a real trajectory step.
        if abs(dx) > np.pi:
            continue
        norm = np.hypot(dx, dy)
        if norm == 0:
            continue
        ax.annotate(
            "", xy=(x[i] + 0.12 * dx, y[i] + 0.12 * dy), xytext=(x[i], y[i]),
            arrowprops=dict(arrowstyle="-|>", color="0.25", lw=0.8, mutation_scale=8),
            zorder=4,
        )


def plot(data, save_fig=None, show=True, capture_angle=0.4,
         release_angle=1.0, capture_speed=8.0, title=None):
    """Plot time histories and a phase portrait centred on the upright position."""
    t, theta, omega, tau, volt, mode = data.T
    phi = wrap(theta - np.pi)
    balance = mode > 0.5

    fig = plt.figure(figsize=(13.5, 9.0), constrained_layout=True)
    gs = fig.add_gridspec(3, 2, width_ratios=(2.15, 1.0), height_ratios=(1.0, 1.0, 1.0))
    ax_theta = fig.add_subplot(gs[0, 0])
    ax_omega = fig.add_subplot(gs[1, 0], sharex=ax_theta)
    ax_effort = fig.add_subplot(gs[2, 0], sharex=ax_theta)
    ax_phase = fig.add_subplot(gs[:, 1])

    # --- Angle history: retain the unwrapped angle to make complete turns visible. ---
    _shade_balance(ax_theta, t, balance)
    ax_theta.plot(t, theta, color="tab:blue", lw=1.25, label=r"Angle, $\theta$")
    k_lo = int(np.floor(theta.min() / (2.0 * np.pi))) - 1
    k_hi = int(np.ceil(theta.max() / (2.0 * np.pi))) + 1
    for idx, k in enumerate(range(k_lo, k_hi + 1)):
        ax_theta.axhline(2 * k * np.pi, color="0.35", ls="-.", lw=0.8,
                         alpha=0.65, label="hanging positions" if idx == 0 else "_nolegend_")
        ax_theta.axhline((2 * k + 1) * np.pi, color="0.25", ls="--", lw=0.8,
                         alpha=0.65, label="upright positions" if idx == 0 else "_nolegend_")
    _add_transition_lines(ax_theta, t, balance)
    ax_theta.set_ylabel(r"Angle, $\theta$ (rad)")
    ax_theta.set_title("Pendulum state and actuator signals")
    ax_theta.grid(True, alpha=0.25)
    time_handles, time_labels = ax_theta.get_legend_handles_labels()
    time_handles.extend([
        Patch(facecolor="tab:green", alpha=0.10, edgecolor="none", label="Balance-mode interval"),
        Line2D([0], [0], color="tab:green", ls=":", lw=1.0, label="Capture"),
        Line2D([0], [0], color="tab:red", ls=":", lw=1.0, label="Balance lost"),
    ])
    time_labels = [h.get_label() for h in time_handles]
    ax_theta.legend(time_handles, time_labels, loc="best", fontsize=8, ncol=2)

    # --- Angular velocity history. ---
    _shade_balance(ax_omega, t, balance)
    ax_omega.plot(t, omega, color="tab:orange", lw=1.1, label=r"Angular velocity, $\omega$")
    ax_omega.axhline(0.0, color="0.35", lw=0.7)
    _add_transition_lines(ax_omega, t, balance)
    ax_omega.set_ylabel(r"Velocity, $\omega$ (rad/s)")
    ax_omega.grid(True, alpha=0.25)
    ax_omega.legend(loc="best", fontsize=8)

    # --- Torque and voltage share the time axis but have independent units. ---
    _shade_balance(ax_effort, t, balance)
    torque_line, = ax_effort.plot(t, tau * 1e3, color="tab:red", lw=1.1,
                                  label=r"Motor torque, $\tau$ (mN·m)")
    ax_effort.axhline(0.0, color="0.35", lw=0.7)
    _add_transition_lines(ax_effort, t, balance)
    ax_effort.set_ylabel("Torque (mN·m)")
    ax_effort.set_xlabel("Time (s)")
    ax_effort.grid(True, alpha=0.25)
    ax_voltage = ax_effort.twinx()
    voltage_line, = ax_voltage.plot(t, volt, color="tab:purple", lw=0.9, alpha=0.85,
                                    label="Motor voltage (V)")
    ax_voltage.set_ylabel("Voltage (V)", color="tab:purple")
    ax_voltage.tick_params(axis="y", labelcolor="tab:purple")
    ax_effort.legend([torque_line, voltage_line],
                     [torque_line.get_label(), voltage_line.get_label()],
                     loc="best", fontsize=8)

    # --- Phase portrait around the upright equilibrium. ---
    # The wrapped angle error keeps the portrait local to the nearest upright position.
    ax_phase.axvspan(-capture_angle, capture_angle, color="tab:green", alpha=0.10, zorder=0)
    ax_phase.axvspan(-release_angle, release_angle, color="tab:orange", alpha=0.06, zorder=0)
    if capture_speed is not None and capture_speed > 0:
        capture_box = Rectangle(
            (-capture_angle, -capture_speed), 2.0 * capture_angle, 2.0 * capture_speed,
            fill=False, edgecolor="tab:green", linestyle="--", linewidth=1.1, zorder=2,
        )
        ax_phase.add_patch(capture_box)
        ax_phase.axhline(capture_speed, color="tab:green", ls=":", lw=0.8, alpha=0.75)
        ax_phase.axhline(-capture_speed, color="tab:green", ls=":", lw=0.8, alpha=0.75)

    # Show the trajectory faintly, then overlay samples by controller mode.
    # Break the line where the wrapped angle jumps across -pi/pi.
    phase_line = phi.copy()
    phase_line[1:][np.abs(np.diff(phi)) > np.pi] = np.nan
    ax_phase.plot(phase_line, omega, color="0.55", lw=0.55, alpha=0.45, zorder=1)
    swing = ~balance
    if np.any(swing):
        ax_phase.scatter(phi[swing], omega[swing], s=9, color="tab:blue", alpha=0.62,
                         edgecolors="none", label="Swing-up / motor off", zorder=2)
    if np.any(balance):
        ax_phase.scatter(phi[balance], omega[balance], s=11, color="tab:green", alpha=0.78,
                         edgecolors="none", label="Balance mode", zorder=3)

    # Start/end markers and the upright equilibrium.
    ax_phase.scatter(phi[0], omega[0], marker="o", s=55, facecolors="white",
                     edgecolors="black", linewidths=1.1, label="Start", zorder=5)
    ax_phase.scatter(phi[-1], omega[-1], marker="X", s=58, color="black",
                     label="End", zorder=6)
    ax_phase.scatter(0.0, 0.0, marker="+", s=100, color="black",
                     linewidths=1.5, label="Upright equilibrium", zorder=7)
    _add_direction_arrows(ax_phase, phi, omega)

    ax_phase.axvline(0.0, color="0.3", lw=0.7, alpha=0.7)
    ax_phase.axhline(0.0, color="0.3", lw=0.7, alpha=0.7)
    ax_phase.set_xlim(-np.pi, np.pi)
    ax_phase.set_xlabel(r"Wrapped upright error, $\phi=\mathrm{wrap}(\theta-\pi)$ (rad)")
    ax_phase.set_ylabel(r"Angular velocity, $\omega$ (rad/s)")
    ax_phase.set_title("Phase portrait near upright")
    ax_phase.grid(True, alpha=0.25)

    phase_handles, phase_labels = ax_phase.get_legend_handles_labels()
    phase_handles.extend([
        Patch(facecolor="tab:orange", alpha=0.10, edgecolor="none", label="Release-angle band"),
        Patch(facecolor="tab:green", alpha=0.10, edgecolor="none", label="Capture-angle band"),
    ])
    if capture_speed is not None and capture_speed > 0:
        phase_handles.append(Line2D([0], [0], color="tab:green", ls="--", lw=1.1,
                                    label="Capture-speed limits"))
    phase_labels = [h.get_label() for h in phase_handles]
    ax_phase.legend(phase_handles, phase_labels, loc="best", fontsize=8)

    if title:
        fig.suptitle(title, fontsize=14)
    else:
        fig.suptitle("Pendulum run", fontsize=14)

    if save_fig:
        fig.savefig(save_fig, dpi=180, bbox_inches="tight")
        print(f"Figure saved to {save_fig}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Capture, summarize and plot a pendulum run.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="serial port of the Arduino (e.g. COM3, /dev/ttyUSB0)")
    source.add_argument("--csv", help="CSV file already saved (hardware or simulation)")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate (default: 115200)")
    parser.add_argument("--out", help="CSV file to write when capturing (default: timestamped filename)")
    parser.add_argument("--save-fig", help="save the figure to this file (PNG, PDF, SVG, ...) ")
    parser.add_argument("--no-show", action="store_true", help="do not open the plot window")
    parser.add_argument("--capture-angle", type=float, default=0.4,
                        help="upright capture angle threshold in rad (default: 0.4)")
    parser.add_argument("--release-angle", type=float, default=1.0,
                        help="upright release angle threshold in rad (default: 1.0)")
    parser.add_argument("--capture-speed", type=float, default=8.0,
                        help="capture speed limit in rad/s; use 0 to hide it (default: 8.0)")
    parser.add_argument("--title", help="optional figure title")
    args = parser.parse_args()

    if args.capture_angle <= 0 or args.release_angle <= 0:
        parser.error("capture and release angles must be positive")
    if args.release_angle < args.capture_angle:
        parser.error("release angle should be greater than or equal to capture angle")
    if args.capture_speed < 0:
        parser.error("capture speed must be non-negative")

    try:
        if args.port:
            data = capture(args.port, args.baud)
            if data.size == 0:
                sys.exit("No data received.")
            output = args.out or datetime.now().strftime("pendulum_run_%Y%m%d_%H%M%S.csv")
            save_csv(output, data)
        else:
            data = load_csv(args.csv)
            if data.size == 0:
                sys.exit(f"No valid data rows in {args.csv}")

        data = validate_data(data)
        summary(data)
        plot(
            data,
            save_fig=args.save_fig,
            show=not args.no_show,
            capture_angle=args.capture_angle,
            release_angle=args.release_angle,
            capture_speed=args.capture_speed if args.capture_speed > 0 else None,
            title=args.title,
        )
    except (OSError, ValueError) as exc:
        sys.exit(f"Error: {exc}")


if __name__ == "__main__":
    main()
