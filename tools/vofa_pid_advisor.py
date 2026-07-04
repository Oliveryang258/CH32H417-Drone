#!/usr/bin/env python3
"""
Analyze VOFA pitch-rate PID logs and print tuning suggestions.

Expected V3F VOFA layout, 8 floats:
  0 pitch setpoint dps
  1 pitch gyro dps
  2 pitch error dps
  3 pitch P term
  4 pitch I term
  5 pitch D term
  6 pitch motor diff, rear_avg - front_avg
  7 average motor throttle

The tool is advisory only. It never writes flight parameters.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path


DEFAULT_RATE_HZ = 50.0
CHANNELS = 8


@dataclass
class Sample:
    t: float
    sp: float
    gyro: float
    err: float
    p: float
    i: float
    d: float
    diff: float
    throttle: float

    @property
    def out(self) -> float:
        return self.p + self.i + self.d


@dataclass
class Metrics:
    n: int
    duration_s: float
    peak_abs_gyro: float
    peak_abs_error: float
    peak_abs_p: float
    peak_abs_i: float
    peak_abs_d: float
    peak_abs_out: float
    peak_abs_diff: float
    throttle_start: float
    throttle_end: float
    throttle_drop: float
    late_mean_gyro: float
    zero_crossings: int
    sign_reversal_ratio: float
    d_to_p_ratio: float
    i_to_p_ratio: float
    diff_to_out_ratio: float
    high_freq_score: float
    out_clip_pct: float
    diff_clip_pct: float
    effective_kp: float | None


def extract_numbers(line: str) -> list[float]:
    return [float(x) for x in re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", line)]


def looks_like_header(row: list[str]) -> bool:
    joined = ",".join(row).lower()
    return any(token in joined for token in ("time", "gyro", "pitch", "setpoint", "error", "pid", "throttle"))


def read_samples(path: Path, rate_hz: float, has_time: bool | None) -> list[Sample]:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8-sig", errors="ignore", newline="") as f:
        reader = csv.reader(f)
        for raw in reader:
            if not raw or looks_like_header(raw):
                continue
            nums: list[float] = []
            for cell in raw:
                nums.extend(extract_numbers(cell))
            if len(nums) >= CHANNELS:
                rows.append(nums)

    if not rows:
        raise SystemExit(f"No numeric VOFA rows found in {path}")

    if has_time is None:
        has_time = len(rows[0]) >= CHANNELS + 1

    samples: list[Sample] = []
    for idx, nums in enumerate(rows):
        if has_time:
            t = nums[0]
            values = nums[-CHANNELS:]
        else:
            t = idx / rate_hz
            values = nums[:CHANNELS]
        samples.append(Sample(t, *values))

    if has_time and len(samples) >= 2:
        span = samples[-1].t - samples[0].t
        if span > 20.0:
            t0 = samples[0].t
            samples = [Sample((s.t - t0) / 1000.0, s.sp, s.gyro, s.err, s.p, s.i, s.d, s.diff, s.throttle) for s in samples]
        else:
            t0 = samples[0].t
            samples = [Sample(s.t - t0, s.sp, s.gyro, s.err, s.p, s.i, s.d, s.diff, s.throttle) for s in samples]

    return samples


def slice_samples(samples: list[Sample], start: float | None, end: float | None) -> list[Sample]:
    if start is None and end is None:
        return samples
    selected = [s for s in samples if (start is None or s.t >= start) and (end is None or s.t <= end)]
    if len(selected) < 5:
        raise SystemExit("Selected window has fewer than 5 samples.")
    return selected


def auto_active_window(samples: list[Sample], threshold: float) -> list[Sample]:
    active = [idx for idx, s in enumerate(samples) if abs(s.gyro) >= threshold or abs(s.out) >= threshold]
    if not active:
        return samples
    start = max(0, active[0] - 5)
    end = min(len(samples), active[-1] + 6)
    return samples[start:end]


def filter_min_throttle(samples: list[Sample], min_throttle: float) -> tuple[list[Sample], str | None]:
    if min_throttle <= 0.0:
        return samples, None

    filtered = [s for s in samples if s.throttle >= min_throttle]
    removed = len(samples) - len(filtered)
    if len(filtered) < 5:
        raise SystemExit(
            f"Fewer than 5 samples remain after min-throttle filter "
            f"({min_throttle:.0f}us). Use --min-throttle 0 to disable it."
        )
    if removed == 0:
        return filtered, None
    return filtered, f"min-throttle filter: removed {removed} samples below {min_throttle:.0f}us"


def trim_after_throttle_drop(
    samples: list[Sample],
    drop_us: float,
    window_s: float,
) -> tuple[list[Sample], str | None]:
    if len(samples) < 8:
        return samples, None

    dt = (samples[-1].t - samples[0].t) / max(len(samples) - 1, 1)
    win = max(2, int(round(window_s / max(dt, 1e-6))))
    max_seen = samples[0].throttle

    for idx in range(1, len(samples)):
        max_seen = max(max_seen, samples[idx].throttle)
        if idx < win:
            continue
        recent_drop = samples[idx - win].throttle - samples[idx].throttle
        total_drop = max_seen - samples[idx].throttle
        if recent_drop >= drop_us or total_drop >= drop_us * 1.4:
            cut = max(5, idx - win)
            if cut < len(samples) - 3:
                msg = (
                    f"auto-trim: throttle drop detected at {samples[idx].t:.3f}s; "
                    f"analyzing {samples[0].t:.3f}s..{samples[cut - 1].t:.3f}s only"
                )
                return samples[:cut], msg
    return samples, None


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def rms(values: list[float]) -> float:
    return math.sqrt(mean([v * v for v in values])) if values else 0.0


def sign(value: float, deadband: float) -> int:
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0


def count_zero_crossings(values: list[float], deadband: float) -> int:
    prev = 0
    count = 0
    for value in values:
        now = sign(value, deadband)
        if now == 0:
            continue
        if prev != 0 and now != prev:
            count += 1
        prev = now
    return count


def calc_metrics(samples: list[Sample], out_limit: float, diff_limit_hint: float) -> Metrics:
    n = len(samples)
    duration = samples[-1].t - samples[0].t if n >= 2 else 0.0
    gyro = [s.gyro for s in samples]
    err = [s.err for s in samples]
    p_term = [s.p for s in samples]
    i_term = [s.i for s in samples]
    d_term = [s.d for s in samples]
    out = [s.out for s in samples]
    diff = [s.diff for s in samples]

    peak_abs_gyro = max(abs(v) for v in gyro)
    peak_abs_p = max(abs(v) for v in p_term)
    peak_abs_i = max(abs(v) for v in i_term)
    peak_abs_d = max(abs(v) for v in d_term)
    peak_abs_out = max(abs(v) for v in out)
    peak_abs_diff = max(abs(v) for v in diff)

    late = samples[int(n * 0.70) :]
    late_mean_gyro = mean([s.gyro for s in late])
    zero_crossings = count_zero_crossings(gyro, max(2.0, peak_abs_gyro * 0.08))
    pos_peak = max(gyro)
    neg_peak = min(gyro)
    if abs(pos_peak) >= abs(neg_peak):
        sign_reversal_ratio = abs(neg_peak) / max(abs(pos_peak), 1e-6)
    else:
        sign_reversal_ratio = abs(pos_peak) / max(abs(neg_peak), 1e-6)

    diffs = [gyro[idx] - gyro[idx - 1] for idx in range(1, len(gyro))]
    high_freq_score = rms(diffs) / max(peak_abs_gyro, 1e-6)
    out_clip_pct = 100.0 * sum(1 for v in out if abs(v) >= out_limit * 0.95) / n
    diff_clip_pct = 100.0 * sum(1 for v in diff if abs(v) >= diff_limit_hint * 0.95) / n
    kp_ratios = [s.p / s.err for s in samples if abs(s.err) >= 5.0 and abs(s.p) >= 1.0]
    effective_kp = median(kp_ratios) if kp_ratios else None

    return Metrics(
        n=n,
        duration_s=duration,
        peak_abs_gyro=peak_abs_gyro,
        peak_abs_error=max(abs(v) for v in err),
        peak_abs_p=peak_abs_p,
        peak_abs_i=peak_abs_i,
        peak_abs_d=peak_abs_d,
        peak_abs_out=peak_abs_out,
        peak_abs_diff=peak_abs_diff,
        throttle_start=samples[0].throttle,
        throttle_end=samples[-1].throttle,
        throttle_drop=samples[0].throttle - samples[-1].throttle,
        late_mean_gyro=late_mean_gyro,
        zero_crossings=zero_crossings,
        sign_reversal_ratio=sign_reversal_ratio,
        d_to_p_ratio=peak_abs_d / max(peak_abs_p, 1e-6),
        i_to_p_ratio=peak_abs_i / max(peak_abs_p, 1e-6),
        diff_to_out_ratio=peak_abs_diff / max(peak_abs_out, 1e-6),
        high_freq_score=high_freq_score,
        out_clip_pct=out_clip_pct,
        diff_clip_pct=diff_clip_pct,
        effective_kp=effective_kp,
    )


def format_cmd(kp: float | None, kd: float | None, ki: float | None) -> str:
    parts: list[str] = []
    if kp is not None:
        parts.append(f"pp{kp:.3g}")
    if kd is not None:
        parts.append(f"pd{kd:.4g}")
    if ki is not None:
        parts.append(f"pi{ki:.3g}")
    return " ".join(parts)


def median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) * 0.5


def advise(m: Metrics, kp: float, kd: float, ki: float, trim_msg: str | None) -> list[str]:
    advice: list[str] = []
    kp_for_advice = kp
    kp_mismatch = False
    if trim_msg:
        advice.append(trim_msg)
    advice.append(
        f"window: {m.n} samples, {m.duration_s:.2f}s | "
        f"gyro_peak {m.peak_abs_gyro:.1f} dps, out_peak {m.peak_abs_out:.1f}, "
        f"motor_diff_peak {m.peak_abs_diff:.1f}us"
    )
    advice.append(
        f"P/I/D peaks: P {m.peak_abs_p:.1f}, I {m.peak_abs_i:.1f}, D {m.peak_abs_d:.1f} | "
        f"D/P {m.d_to_p_ratio:.2f}, diff/out {m.diff_to_out_ratio:.2f}, zero_cross {m.zero_crossings}, "
        f"throttle {m.throttle_start:.0f}->{m.throttle_end:.0f}"
    )
    if m.effective_kp is not None:
        advice.append(f"Effective P from log is about {m.effective_kp:.3g} based on P_term/error.")
        kp_mismatch = kp > 0.0 and abs(m.effective_kp - kp) / kp > 0.18
        if kp_mismatch:
            kp_for_advice = m.effective_kp
            advice.append(
                f"Warning: --kp {kp:.3g} does not match the logged P term. "
                "Suggestions below use the logged effective P; use a fresh CSV or --start/--end to select the run with that exact P."
            )

    if m.out_clip_pct > 5.0:
        advice.append(f"PID output is near limit for {m.out_clip_pct:.1f}% of samples; check pl/output headroom before only raising P.")
    if m.diff_to_out_ratio < 0.35 and m.peak_abs_out > 20.0:
        advice.append("PID output is present but pitch motor diff is small; check sl, mixing limits, throttle point, or ESC response.")
    if m.high_freq_score > 0.45 and m.peak_abs_gyro > 8.0:
        advice.append("High-frequency gyro noise is high; if motor sound is sharp/choppy, reduce D or P and do not add I.")

    if m.peak_abs_gyro > 25.0 and m.sign_reversal_ratio < 0.35 and m.zero_crossings <= 1:
        next_kp = kp_for_advice * 1.10 if kp_for_advice > 0.0 else 0.2
        advice.append(f"Large gyro peak with little reversal: P looks soft. Try {format_cmd(next_kp, kd, 0.0 if ki == 0 else ki)}")
    elif m.sign_reversal_ratio >= 0.55 or m.zero_crossings >= 2:
        next_kd = kd + 0.002 if kd < 0.012 else kd
        next_kp = kp_for_advice * 0.95 if m.high_freq_score > 0.35 else kp_for_advice
        if next_kd != kd:
            advice.append(f"Clear reversal/overshoot: P has authority, add D first. Try {format_cmd(next_kp, next_kd, 0.0 if ki == 0 else ki)}")
        else:
            advice.append(f"Clear reversal and D is already high: slightly reduce P. Try {format_cmd(next_kp, kd, 0.0 if ki == 0 else ki)}")
    elif m.peak_abs_gyro > 25.0 and m.d_to_p_ratio < 0.08:
        advice.append(f"Gyro peak is still large and D contribution is small. Try {format_cmd(kp_for_advice, kd + 0.002, 0.0 if ki == 0 else ki)}")
    elif 8.0 <= m.peak_abs_gyro <= 25.0:
        advice.append("P/D is roughly usable; check long one-sided drift before adding I.")
    else:
        advice.append("Gyro peak is small; this window is not enough to judge P boundary.")

    if abs(m.late_mean_gyro) > 5.0 and m.zero_crossings <= 1 and m.high_freq_score < 0.35:
        if ki <= 0.0:
            advice.append(f"Late one-sided drift remains. After P/D is stable, try small I: {format_cmd(kp_for_advice, kd, 0.1)}")
        elif m.i_to_p_ratio < 0.35:
            advice.append(f"I is still modest and drift remains. Try {format_cmd(kp_for_advice, kd, ki + 0.1)}")
        else:
            advice.append("I is already significant but drift remains; check throttle point, CG, test stand, or motor thrust mismatch.")

    if m.peak_abs_diff > 70.0 and m.peak_abs_gyro > 25.0 and m.sign_reversal_ratio < 0.35:
        advice.append("Motor diff is already large but gyro is not controlled; check thrust chain, CG, prop/motor direction, and stand mechanics.")

    advice.append("Safety: advisory only; change one variable at a time and ignore samples after manual throttle-down.")
    return advice


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze VOFA pitch PID CSV/log and suggest tuning moves.")
    parser.add_argument("path", type=Path, help="VOFA CSV/log path")
    parser.add_argument("--kp", type=float, required=True, help="current pitch P")
    parser.add_argument("--kd", type=float, required=True, help="current pitch D")
    parser.add_argument("--ki", type=float, required=True, help="current pitch I")
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ, help="sample rate when no time column exists")
    parser.add_argument("--start", type=float, default=None, help="window start time in seconds")
    parser.add_argument("--end", type=float, default=None, help="window end time in seconds")
    parser.add_argument("--time-column", action="store_true", help="force first numeric column as time")
    parser.add_argument("--no-time-column", action="store_true", help="force first 8 numeric values as channels")
    parser.add_argument("--active-threshold", type=float, default=4.0, help="auto-window activity threshold")
    parser.add_argument("--min-throttle", type=float, default=1250.0, help="ignore samples below this average throttle; set 0 to disable")
    parser.add_argument("--throttle-drop-us", type=float, default=35.0, help="auto-trim after this throttle drop")
    parser.add_argument("--throttle-drop-window", type=float, default=0.12, help="drop detection window in seconds")
    parser.add_argument("--no-throttle-trim", action="store_true", help="do not auto-trim manual throttle-down")
    parser.add_argument("--out-limit", type=float, default=180.0, help="PID output limit used for clipping warning")
    parser.add_argument("--diff-limit-hint", type=float, default=180.0, help="motor diff clipping hint")
    args = parser.parse_args()

    if args.time_column and args.no_time_column:
        raise SystemExit("Use only one of --time-column or --no-time-column")
    has_time = True if args.time_column else False if args.no_time_column else None

    samples = read_samples(args.path, args.rate_hz, has_time)
    samples = slice_samples(samples, args.start, args.end)
    trim_msg = None
    filter_msg = None
    if args.start is None and args.end is None:
        samples, filter_msg = filter_min_throttle(samples, args.min_throttle)
        samples = auto_active_window(samples, args.active_threshold)
        if not args.no_throttle_trim:
            samples, trim_msg = trim_after_throttle_drop(samples, args.throttle_drop_us, args.throttle_drop_window)

    metrics = calc_metrics(samples, args.out_limit, args.diff_limit_hint)

    print("VOFA Pitch PID Advisor")
    print("======================")
    if filter_msg:
        print("- " + filter_msg)
    for line in advise(metrics, args.kp, args.kd, args.ki, trim_msg):
        print("- " + line)


if __name__ == "__main__":
    main()
