"""
Sensitivity of the HRV results to the (assumed) beat-detection error model.

For every combination of
    timing jitter   {2, 5, 10} ms
    missed beats    {0, 2, 5, 10} %
    extra beats     {0, 1, 3} %
on the 5-minute cycle (300 s sleep) with the sensor's 100 Hz sample rate, this runs
    prototype  - the firmware computation at 4f57341 (15 s window, 8-slot buffer)
    proposed   - the fixed algorithm with a 60 s window
and records, for RMSSD and SDNN against 5-minute HRV from the same ECG:
    r, bias, 95% limits of agreement, reading-to-reading CV, ICC
plus the low-HRV bias: RMSSD measured vs true for the older subjects (O1-O5).

Uses the same functions and random seed (7) as hrv_window_study.py; the generator
is re-seeded for every setting, exactly as that study does for each detection model,
so the setting (5 ms, 2 %, 1 %) reproduces its "realistic" rows.

    python3 hrv_sensitivity_grid.py            # writes hrv_sensitivity_grid.csv
"""
import csv
import glob
import itertools
import os

import numpy as np

import hrv_window_study as h

HERE = os.path.dirname(os.path.abspath(__file__))
JITTER_MS = [2, 5, 10]
MISSED_PCT = [0, 2, 5, 10]
EXTRA_PCT = [0, 1, 3]
SLEEP_S = 300.0
FS = 100.0
CONFIGS = [
    ("prototype_15s", "fw", h.WINDOW_S),
    ("proposed_60s", "fixed", 60.0),
]
OLDER = ("O1", "O2", "O3", "O4", "O5")


def load_subjects(data_dir):
    subjects = {}
    for f in sorted(glob.glob(os.path.join(data_dir, "*.txt"))):
        rr = h.load_rr(f)
        subjects[os.path.basename(f)[:-4]] = (h.beat_times(rr), rr)
    return subjects


def evaluate(subjects, jitter_ms, p_miss, p_extra):
    """Same procedure as hrv_window_study.main() for one detection model."""
    rng = np.random.default_rng(h.RNG_SEED)
    dets = {s: h.detect(t, rng, jitter_ms=jitter_ms, p_miss=p_miss, p_extra=p_extra, fs=FS)
            for s, (t, rr) in subjects.items()}
    results = {}
    for cfg, kind, win in CONFIGS:
        rows_s, rows_r, counts = [], [], []
        for s, (t_true, rr_true) in subjects.items():
            t_end = t_true[-1]
            if kind == "fw":
                h.stale_counts = []
                out = [(t_pub, sd, rm, n) for t_pub, sd, rm, n, _ in
                       h.run_firmware(dets[s], t_end, sleep_s=SLEEP_S)]
            else:
                out = h.run_fixed(dets[s], t_end, window_s=win, sleep_s=SLEEP_S)
            for t_pub, sd, rm, n in out:
                if n < 3:
                    continue
                ref_s, ref_r = h.reference(t_true, rr_true, t_pub - win / 2)
                rows_s.append((s, sd, ref_s))
                rows_r.append((s, rm, ref_r))
                counts.append(n)
        results[cfg] = (h.summarise(rows_s), h.summarise(rows_r), float(np.median(counts)), rows_r)
    return results


def low_hrv_bias(rows_r):
    """Older subjects: per-reading measured vs true 5-min RMSSD (same row filter as summarise)."""
    rows = [(m, t) for s, m, t in rows_r
            if s in OLDER and np.isfinite(m) and np.isfinite(t) and m > 0]
    m = np.array([r[0] for r in rows])
    t = np.array([r[1] for r in rows])
    return dict(
        n=len(rows),
        true_median=float(np.median(t)),
        measured_median=float(np.median(m)),
        bias_ms=float(np.median(m - t)),                       # median per-reading difference
        bias_pct=float((np.median(m) / np.median(t) - 1) * 100),  # as reported in the study
    )


FIELDS = [
    "jitter_ms", "missed_pct", "extra_pct", "config", "n_readings", "median_beats",
    "rmssd_r", "rmssd_bias_ms", "rmssd_loa_ms", "rmssd_cv_pct", "rmssd_icc",
    "sdnn_r", "sdnn_bias_ms", "sdnn_loa_ms", "sdnn_cv_pct", "sdnn_icc",
    "older_n", "older_true_rmssd_ms", "older_measured_rmssd_ms",
    "older_rmssd_bias_ms", "older_rmssd_bias_pct",
]


def main():
    subjects = load_subjects(os.path.join(HERE, "fantasia"))
    if len(subjects) != 10:
        raise SystemExit("Run ./fetch_fantasia.sh first")

    rows = []
    for j, mp, ep in itertools.product(JITTER_MS, MISSED_PCT, EXTRA_PCT):
        res = evaluate(subjects, j, mp / 100, ep / 100)
        for cfg, _, _ in CONFIGS:
            sd, rm, beats, rows_r = res[cfg]
            lo = low_hrv_bias(rows_r)
            rows.append({
                "jitter_ms": j, "missed_pct": mp, "extra_pct": ep, "config": cfg,
                "n_readings": rm["n"], "median_beats": beats,
                "rmssd_r": rm["r"], "rmssd_bias_ms": rm["bias"], "rmssd_loa_ms": rm["loa"],
                "rmssd_cv_pct": rm["wcv"], "rmssd_icc": rm["icc"],
                "sdnn_r": sd["r"], "sdnn_bias_ms": sd["bias"], "sdnn_loa_ms": sd["loa"],
                "sdnn_cv_pct": sd["wcv"], "sdnn_icc": sd["icc"],
                "older_n": lo["n"], "older_true_rmssd_ms": lo["true_median"],
                "older_measured_rmssd_ms": lo["measured_median"],
                "older_rmssd_bias_ms": lo["bias_ms"], "older_rmssd_bias_pct": lo["bias_pct"],
            })
        print(f"jitter {j:2d} ms, missed {mp:2d}%, extra {ep}%: done", flush=True)

    out = os.path.join(HERE, "hrv_sensitivity_grid.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()})
    print(f"\nwrote {out} ({len(rows)} rows)")

    metrics = ["rmssd_r", "rmssd_loa_ms", "rmssd_cv_pct", "rmssd_icc",
               "sdnn_r", "sdnn_loa_ms", "sdnn_cv_pct", "sdnn_icc",
               "older_rmssd_bias_ms", "older_rmssd_bias_pct"]
    for cfg, _, _ in CONFIGS:
        sub = [r for r in rows if r["config"] == cfg]
        print(f"\n{cfg}: min / max over {len(sub)} settings (setting = jitter ms / missed % / extra %)")
        for m in metrics:
            lo = min(sub, key=lambda r: r[m])
            hi = max(sub, key=lambda r: r[m])
            print(f"  {m:24s} min {lo[m]:8.2f} at {lo['jitter_ms']}/{lo['missed_pct']}/{lo['extra_pct']}"
                  f"   max {hi[m]:8.2f} at {hi['jitter_ms']}/{hi['missed_pct']}/{hi['extra_pct']}")


if __name__ == "__main__":
    main()
