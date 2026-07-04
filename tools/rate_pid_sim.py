#!/usr/bin/env python3
"""
Simple one-axis rate PID simulator for the CH32H417 drone project.

This is not a flight-accurate model. It is a guardrail tool for finding
conservative starting gains and checking sign/limit behavior before the ESCs
are ready for bench tests.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


DT = 0.005
PID_OUT_LIMIT_US = 180.0
MOTOR_SLEW_US_PER_STEP = 12.0


@dataclass(frozen=True)
class Plant:
    name: str
    accel_per_us: float
    damping_per_s: float
    motor_tau_s: float


@dataclass(frozen=True)
class Gains:
    kp: float
    ki: float
    kd: float


@dataclass
class Metrics:
    plant: str
    kp: float
    ki: float
    kd: float
    overshoot_pct: float
    settle_ms: float
    rise_ms: float
    max_abs_output_us: float
    saturated_steps: int
    max_slew_us: float
    steady_error_dps: float
    score: float


@dataclass
class RobustMetrics:
    kp: float
    ki: float
    kd: float
    worst_overshoot_pct: float
    worst_settle_ms: float
    worst_rise_ms: float
    worst_max_abs_output_us: float
    total_saturated_steps: int
    worst_steady_error_dps: float
    score: float


PLANTS = [
    Plant("weak_slow", accel_per_us=16.0, damping_per_s=4.0, motor_tau_s=0.090),
    Plant("nominal", accel_per_us=24.0, damping_per_s=5.5, motor_tau_s=0.060),
    Plant("strong_fast", accel_per_us=34.0, damping_per_s=7.0, motor_tau_s=0.040),
]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class Pid:
    def __init__(self, gains: Gains, out_limit: float, int_limit: float) -> None:
        self.gains = gains
        self.out_limit = out_limit
        self.int_limit = int_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.deriv_filt = 0.0

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        self.integral += error * dt
        self.integral = clamp(self.integral, -self.int_limit, self.int_limit)

        deriv_raw = (error - self.prev_error) / dt
        self.deriv_filt += 0.2 * (deriv_raw - self.deriv_filt)

        output = (
            self.gains.kp * error
            + self.gains.ki * self.integral
            + self.gains.kd * self.deriv_filt
        )
        self.prev_error = error
        return clamp(output, -self.out_limit, self.out_limit)


def simulate(
    plant: Plant,
    gains: Gains,
    target_dps: float,
    duration_s: float,
    int_limit: float,
    csv_path: Path | None = None,
) -> Metrics:
    pid = Pid(gains, PID_OUT_LIMIT_US, int_limit)
    rate = 0.0
    motor_eff = 0.0
    prev_output = 0.0
    max_rate = -1e9
    max_abs_output = 0.0
    saturated_steps = 0
    max_slew = 0.0
    rise_ms = math.inf
    settle_ms = math.inf
    samples: list[tuple[float, float, float, float, float]] = []

    step_time = 0.10
    settle_band = abs(target_dps) * 0.05

    steps = int(duration_s / DT)
    for i in range(steps):
        t = i * DT
        sp = target_dps if t >= step_time else 0.0
        raw_output = pid.update(sp, rate, DT)

        slew = raw_output - prev_output
        if slew > MOTOR_SLEW_US_PER_STEP:
            output = prev_output + MOTOR_SLEW_US_PER_STEP
        elif slew < -MOTOR_SLEW_US_PER_STEP:
            output = prev_output - MOTOR_SLEW_US_PER_STEP
        else:
            output = raw_output

        max_slew = max(max_slew, abs(output - prev_output))
        prev_output = output
        if abs(raw_output) >= PID_OUT_LIMIT_US - 1e-6:
            saturated_steps += 1

        motor_eff += (output - motor_eff) * DT / plant.motor_tau_s
        rate_dot = plant.accel_per_us * motor_eff - plant.damping_per_s * rate
        rate += rate_dot * DT

        if t >= step_time:
            max_rate = max(max_rate, rate)
            if rise_ms == math.inf and rate >= 0.9 * target_dps:
                rise_ms = (t - step_time) * 1000.0

        max_abs_output = max(max_abs_output, abs(output))
        samples.append((t, sp, rate, raw_output, output))

    final_window = samples[int(0.75 * len(samples)) :]
    steady_error = sum(sp - rate for _, sp, rate, _, _ in final_window) / len(final_window)

    for idx, (t, _, _, _, _) in enumerate(samples):
        if t < step_time:
            continue
        tail = samples[idx:]
        if all(abs(sp - rate) <= settle_band for _, sp, rate, _, _ in tail):
            settle_ms = (t - step_time) * 1000.0
            break

    overshoot_pct = max(0.0, (max_rate - target_dps) / target_dps * 100.0)
    if rise_ms == math.inf:
        rise_ms = duration_s * 1000.0
    if settle_ms == math.inf:
        settle_ms = duration_s * 1000.0

    score = (
        overshoot_pct * 3.0
        + settle_ms * 0.015
        + rise_ms * 0.004
        + saturated_steps * 0.2
        + abs(steady_error) * 1.5
        + max(0.0, max_abs_output - 120.0) * 0.2
    )

    if csv_path is not None:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "setpoint_dps", "rate_dps", "pid_raw_us", "pid_slew_us"])
            writer.writerows(samples)

    return Metrics(
        plant=plant.name,
        kp=gains.kp,
        ki=gains.ki,
        kd=gains.kd,
        overshoot_pct=overshoot_pct,
        settle_ms=settle_ms,
        rise_ms=rise_ms,
        max_abs_output_us=max_abs_output,
        saturated_steps=saturated_steps,
        max_slew_us=max_slew,
        steady_error_dps=steady_error,
        score=score,
    )


def sweep(target_dps: float, duration_s: float, int_limit: float) -> tuple[list[RobustMetrics], list[Metrics]]:
    robust_results: list[RobustMetrics] = []
    detailed_results: list[Metrics] = []
    kp_values = [round(0.10 + 0.05 * i, 3) for i in range(31)]
    kd_values = [round(0.000 + 0.001 * i, 4) for i in range(31)]

    for kp in kp_values:
        for kd in kd_values:
            gains = Gains(kp=kp, ki=0.0, kd=kd)
            group: list[Metrics] = []
            for plant in PLANTS:
                metrics = simulate(plant, gains, target_dps, duration_s, int_limit)
                group.append(metrics)
                detailed_results.append(metrics)

            worst_overshoot = max(m.overshoot_pct for m in group)
            worst_settle = max(m.settle_ms for m in group)
            worst_rise = max(m.rise_ms for m in group)
            worst_output = max(m.max_abs_output_us for m in group)
            total_sat = sum(m.saturated_steps for m in group)
            worst_steady = max(abs(m.steady_error_dps) for m in group)

            if worst_output <= 145.0 and worst_overshoot <= 20.0 and total_sat <= 20:
                score = (
                    worst_overshoot * 4.0
                    + worst_rise * 0.012
                    + total_sat * 0.25
                    + worst_steady * 0.55
                    + max(0.0, worst_output - 110.0) * 0.25
                )
                robust_results.append(
                    RobustMetrics(
                        kp=kp,
                        ki=0.0,
                        kd=kd,
                        worst_overshoot_pct=worst_overshoot,
                        worst_settle_ms=worst_settle,
                        worst_rise_ms=worst_rise,
                        worst_max_abs_output_us=worst_output,
                        total_saturated_steps=total_sat,
                        worst_steady_error_dps=worst_steady,
                        score=score,
                    )
                )

    return sorted(robust_results, key=lambda m: m.score), sorted(detailed_results, key=lambda m: m.score)


def write_summary_csv(results: list[RobustMetrics], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "plant",
                "kp",
                "ki",
                "kd",
                "worst_overshoot_pct",
                "worst_settle_ms",
                "worst_rise_ms",
                "worst_max_abs_output_us",
                "total_saturated_steps",
                "worst_steady_error_dps",
                "score",
            ]
        )
        for m in results:
            writer.writerow(
                [
                    "robust_all",
                    m.kp,
                    m.ki,
                    m.kd,
                    round(m.worst_overshoot_pct, 3),
                    round(m.worst_settle_ms, 1),
                    round(m.worst_rise_ms, 1),
                    round(m.worst_max_abs_output_us, 2),
                    m.total_saturated_steps,
                    round(m.worst_steady_error_dps, 3),
                    round(m.score, 3),
                ]
            )


def print_table(results: list[RobustMetrics], limit: int) -> None:
    print("Robust candidates across weak_slow, nominal, strong_fast; Ki fixed at 0.0")
    print("kp     kd      worst_ov  worst_rise  max_out  sat  worst_err  score")
    for m in results[:limit]:
        print(
            f"{m.kp:>4.2f}  {m.kd:>6.4f}  "
            f"{m.worst_overshoot_pct:>7.1f}% {m.worst_rise_ms:>9.0f}ms "
            f"{m.worst_max_abs_output_us:>7.1f} {m.total_saturated_steps:>4d} "
            f"{m.worst_steady_error_dps:>9.1f} {m.score:>7.1f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Sweep conservative rate PID starting gains.")
    parser.add_argument("--target-dps", type=float, default=100.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--int-limit", type=float, default=80.0)
    parser.add_argument("--top", type=int, default=18)
    parser.add_argument("--out", type=Path, default=Path("tools/output/rate_pid_sweep.csv"))
    parser.add_argument("--trace", type=Path, default=Path("tools/output/rate_pid_trace_nominal.csv"))
    args = parser.parse_args()

    robust_results, detailed_results = sweep(args.target_dps, args.duration, args.int_limit)
    if not robust_results:
        raise SystemExit("No candidate gains passed the filters.")

    write_summary_csv(robust_results, args.out)
    print_table(robust_results, args.top)

    best = robust_results[0]
    simulate(
        Plant("nominal", accel_per_us=24.0, damping_per_s=5.5, motor_tau_s=0.060),
        Gains(kp=best.kp, ki=best.ki, kd=best.kd),
        args.target_dps,
        args.duration,
        args.int_limit,
        csv_path=args.trace,
    )

    print()
    print(f"Wrote sweep CSV: {args.out}")
    print(f"Wrote nominal trace CSV: {args.trace}")
    print()
    print("Use these as conservative starting points only. Real ESC/motor/prop tests still decide final gains.")


if __name__ == "__main__":
    main()
