"""
Replays the esp32_publisher HRV algorithm on real RR-interval data and measures
how stable and how accurate the published SDNN / RMSSD values are, compared with
the standard 5-minute HRV (Task Force 1996) from the same ECG.

Data: PhysioNet Fantasia subset (10 healthy adults, ~120 min supine rest each,
RR intervals in seconds, derived from 250 Hz ECG).

Firmware behaviour reproduced (src/main.cpp @ 4f57341):
  * 15 s collection window, then publish, then light sleep (10 s test value),
    then a blocking wake-up/reconnect phase with no beat detection.
  * lastBeat = 0 after wake, so the first beat after waking produces a huge delta
    and is rejected; lastBeat is updated *before* the 45-150 bpm filter.
  * Accepted RR intervals go into an 8-slot circular buffer that is NOT cleared
    between windows (only when no finger is detected).
  * SDNN uses the n divisor; RMSSD differences are taken in buffer-index order,
    which is not chronological once the buffer has wrapped.

Detection is modelled on top of the true (ECG) beat times:
  * 100 Hz effective sample rate (MAX30105 setup() default: 400 Hz, 4x averaging)
    -> beat times quantised to 10 ms,
  * optional PPG fiducial jitter (Gaussian, ms),
  * optional missed / extra beats (probabilities per beat).
These detection parameters are assumptions; replace them with real device
data via --serial-log (the firmware prints "[BEAT] delta=<ms>" for every beat).
"""

import argparse
import glob
import os
import re
import sys

import numpy as np

RNG_SEED = 7

# ── Firmware timing ───────────────────────────────────────────────────────────
WINDOW_S = 15.0
SLEEP_S = 300.0             # production value (5 min); the repo currently has 10 s
WAKE_OVERHEAD_S = 4.0       # 0.5 s delay + WiFi/DNS wait + 1.5 s settle + MQTT connect
RATE_SIZE = 8
stale_counts = []
REF_S = 300.0


# ── Data ──────────────────────────────────────────────────────────────────────

def load_rr(path):
    rr = np.loadtxt(path)
    # Clean to NN intervals for the reference: plausible range and <20 % from the
    # local median (standard ectopic/artefact rule).
    rr = rr[(rr > 0.3) & (rr < 2.0)]
    med = np.array([np.median(rr[max(0, i - 5):i + 6]) for i in range(len(rr))])
    keep = np.abs(rr - med) / med < 0.2
    return rr[keep]


def beat_times(rr):
    return np.concatenate([[0.0], np.cumsum(rr)])


# ── Metrics ───────────────────────────────────────────────────────────────────

def sdnn_fw(rr):
    """Firmware SDNN: population SD (divide by n)."""
    if len(rr) < 2:
        return 0.0
    return float(np.sqrt(np.mean((rr - rr.mean()) ** 2)))


def rmssd_fw(rr):
    """Firmware RMSSD: differences in array (index) order, divide by n-1."""
    if len(rr) < 2:
        return 0.0
    d = np.diff(rr)
    return float(np.sqrt(np.sum(d ** 2) / (len(rr) - 1)))


def sdnn_std(rr):
    return float(np.std(rr, ddof=1)) if len(rr) >= 2 else np.nan


def rmssd_std(segments):
    """RMSSD over successive pairs only; `segments` is a list of runs of
    consecutive (unbroken) RR intervals."""
    d = np.concatenate([np.diff(s) for s in segments if len(s) >= 2] or [np.array([])])
    return float(np.sqrt(np.mean(d ** 2))) if len(d) else np.nan


def reference(t_true, rr_true, t_centre):
    """Task Force 5-min HRV around t_centre (ms)."""
    lo, hi = t_centre - REF_S / 2, t_centre + REF_S / 2
    m = (t_true[1:] >= lo) & (t_true[1:] < hi)
    seg = rr_true[m] * 1000
    if len(seg) < 100:
        return np.nan, np.nan
    return sdnn_std(seg), rmssd_std([seg])


# ── Detection model ───────────────────────────────────────────────────────────

def detect(t_true, rng, jitter_ms, p_miss, p_extra, fs):
    """Return detected beat times (s) as seen by the firmware."""
    t = t_true.copy()
    if p_miss > 0:
        t = t[rng.random(len(t)) >= p_miss]
    if p_extra > 0:
        mids = (t[:-1] + t[1:]) / 2 + rng.normal(0, 0.05, len(t) - 1)
        extra = mids[rng.random(len(mids)) < p_extra]
        t = np.sort(np.concatenate([t, extra]))
    if jitter_ms > 0:
        t = t + rng.normal(0, jitter_ms / 1000, len(t))
    # millis() at the sample on which checkForBeat() fires: quantised to 1/fs
    t = np.ceil(t * fs) / fs
    return np.unique(t)


# ── Firmware replay ───────────────────────────────────────────────────────────

def run_firmware(t_det, t_end, window_s=WINDOW_S, sleep_s=SLEEP_S):
    """Exact replay of the current firmware state machine.
    Returns list of (publish_time_s, sdnn_ms, rmssd_ms, beat_count, window_rr_list)."""
    rates_rr = np.zeros(RATE_SIZE)
    global stale_counts
    rate_spot = 0
    rate_count = 0
    out = []
    t0 = 0.0
    last_beat = None            # None == lastBeat=0 after wake: next delta is huge
    i = 0
    while t0 + window_s < t_end:
        new_beats = 0
        window_runs, run = [], []
        while i < len(t_det) and t_det[i] < t0:
            i += 1
        while i < len(t_det) and t_det[i] < t0 + window_s:
            tb = t_det[i]
            if last_beat is None:
                delta = 1e9          # millis() - 0
            else:
                delta = (tb - last_beat) * 1000
            last_beat = tb           # set before the plausibility filter
            bpm = 60000.0 / delta
            if 45 < bpm < 150:
                rates_rr[rate_spot] = delta
                rate_spot = (rate_spot + 1) % RATE_SIZE
                rate_count = min(rate_count + 1, RATE_SIZE)
                new_beats += 1
                run.append(delta)
            else:
                if run:
                    window_runs.append(run)
                run = []
            i += 1
        if run:
            window_runs.append(run)
        t_pub = t0 + window_s
        buf = rates_rr[:rate_count].copy()
        stale = max(0, rate_count - new_beats)   # slots carried over from earlier windows
        stale_counts.append((stale, rate_count))
        out.append((t_pub, sdnn_fw(buf), rmssd_fw(buf), new_beats, window_runs))
        # sleep + blocking wake-up; lastBeat reset; FIFO cleared
        t0 = t_pub + sleep_s + WAKE_OVERHEAD_S
        last_beat = None
    return out


def fixed_from_runs(runs, window_s, artefact=True):
    """Proposed algorithm on one window: chronological, window-local,
    successive pairs only, 20 % artefact rule, sample SD."""
    clean_runs = []
    allrr = np.concatenate([np.asarray(r) for r in runs]) if runs else np.array([])
    if len(allrr) < 2:
        return np.nan, np.nan, 0
    med = np.median(allrr)
    for r in runs:
        r = np.asarray(r)
        if artefact:
            ok = np.abs(r - med) / med < 0.2
            # split runs at rejected beats so no difference spans a gap
            cur = []
            for v, good in zip(r, ok):
                if good:
                    cur.append(v)
                elif cur:
                    clean_runs.append(np.array(cur)); cur = []
            if cur:
                clean_runs.append(np.array(cur))
        else:
            clean_runs.append(r)
    nn = np.concatenate(clean_runs) if clean_runs else np.array([])
    # Quality gate: discard the window if >20 % of its beats were rejected, or if
    # the clean NN intervals cover less than 70 % of the window (signal dropout)
    if len(nn) < 0.8 * len(allrr) or len(nn) < 3 or nn.sum() / 1000 < 0.7 * window_s:
        return np.nan, np.nan, len(nn)
    return sdnn_std(nn), rmssd_std(clean_runs), len(nn)


def run_fixed(t_det, t_end, window_s, sleep_s=SLEEP_S, min_nn=None):
    """Proposed firmware: same cycle, buffer reset each window, all beats in window."""
    res = run_firmware(t_det, t_end, window_s=window_s, sleep_s=sleep_s)
    out = []
    for t_pub, _, _, n, runs in res:
        s, r, nn = fixed_from_runs(runs, window_s)
        if min_nn and nn < min_nn:
            s = r = np.nan
        out.append((t_pub, s, r, nn))
    return out


# ── Statistics ────────────────────────────────────────────────────────────────

def summarise(rows):
    """rows: list of (subject, value, ref). Returns agreement + stability stats."""
    rows = [r for r in rows if np.isfinite(r[1]) and np.isfinite(r[2]) and r[1] > 0]
    subj = np.array([r[0] for r in rows])
    v = np.array([r[1] for r in rows])
    ref = np.array([r[2] for r in rows])
    diff = v - ref
    pct_err = np.abs(diff) / ref * 100
    # within-subject variability between consecutive readings, on log scale
    wcv = []
    within_var, means = [], []
    for s in np.unique(subj):
        x = v[subj == s]
        lx = np.log(x)
        if len(lx) > 2:
            d = np.diff(lx)
            wcv.append(np.sqrt(np.mean(d ** 2) / 2) * 100)   # within-subject CV %
            within_var.append(np.var(lx, ddof=1))
            means.append(lx.mean())
    # ICC(1): between-subject variance of log means vs within-subject variance
    k = np.mean([np.sum(subj == s) for s in np.unique(subj)])
    msb = k * np.var(means, ddof=1)
    msw = np.mean(within_var)
    icc = (msb - msw) / (msb + (k - 1) * msw)
    r = np.corrcoef(np.log(v), np.log(ref))[0, 1]
    return dict(n=len(v), r=r, bias=diff.mean(), loa=1.96 * diff.std(ddof=1),
                mape=np.median(pct_err), wcv=np.median(wcv), icc=icc)


def parse_serial_log(path):
    """Extract RR (ms) from firmware serial output lines '[BEAT] delta=XXXms'."""
    pat = re.compile(r"\[BEAT\] delta=(\d+)ms")
    with open(path, errors="ignore") as f:
        return np.array([int(m.group(1)) for m in map(pat.search, f) if m], dtype=float)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.join(os.path.dirname(__file__), "fantasia"))
    ap.add_argument("--serial-log", help="analyse a real device serial log instead")
    ap.add_argument("--sleep", type=float, default=SLEEP_S, help="sleep between windows (s)")
    args = ap.parse_args()
    sleep_s = args.sleep
    print(f"Cycle: collect window, sleep {sleep_s:.0f} s, wake overhead {WAKE_OVERHEAD_S:.0f} s")

    if args.serial_log:
        rr = parse_serial_log(args.serial_log)
        print(f"{len(rr)} beat deltas parsed from {args.serial_log}")
        ok = (rr > 400) & (rr < 1333)
        print(f"accepted by 45-150 bpm filter: {ok.mean()*100:.1f}%")
        return

    files = sorted(glob.glob(os.path.join(args.data, "*.txt")))
    subjects = {}
    for f in files:
        rr = load_rr(f)
        subjects[os.path.basename(f)[:-4]] = (beat_times(rr), rr)
    print(f"Subjects: {len(subjects)}  beats: {sum(len(v[1]) for v in subjects.values())}  "
          f"duration: {np.mean([v[0][-1] for v in subjects.values()])/60:.0f} min avg")

    detection_models = {
        "ECG timing (ideal)":                     dict(jitter_ms=0, p_miss=0, p_extra=0, fs=1000.0),
        "100 Hz (current sensor config)":         dict(jitter_ms=0, p_miss=0, p_extra=0, fs=100.0),
        "100 Hz + 5 ms PPG jitter":               dict(jitter_ms=5, p_miss=0, p_extra=0, fs=100.0),
        "100 Hz + 5 ms jitter + 2% missed/1% extra": dict(jitter_ms=5, p_miss=0.02, p_extra=0.01, fs=100.0),
        "400 Hz + 5 ms jitter + 2% missed/1% extra": dict(jitter_ms=5, p_miss=0.02, p_extra=0.01, fs=400.0),
    }

    configs = [
        ("Current firmware (8-RR buffer, 15 s)", "fw", WINDOW_S),
        ("Fixed algorithm, 15 s window", "fixed", 15.0),
        ("Fixed algorithm, 30 s window", "fixed", 30.0),
        ("Fixed algorithm, 60 s window", "fixed", 60.0),
        ("Fixed algorithm, 120 s window", "fixed", 120.0),
    ]

    global stale_counts
    results = {}
    lowtab = {}
    stale_report = {}
    subj_rmssd = {s: rmssd_std([rr * 1000]) for s, (t, rr) in subjects.items()}
    print("Whole-recording RMSSD (ms): " + ", ".join(f"{s}={v:.0f}" for s, v in sorted(subj_rmssd.items())))
    for dm_name, dm in detection_models.items():
        rng = np.random.default_rng(RNG_SEED)
        dets = {s: detect(t, rng, **dm) for s, (t, rr) in subjects.items()}
        for cfg_name, kind, win in configs:
            rows_s, rows_r, counts = [], [], []
            for s, (t_true, rr_true) in subjects.items():
                t_end = t_true[-1]
                if kind == "fw":
                    stale_counts = []
                    out = run_firmware(dets[s], t_end, sleep_s=sleep_s)
                    pub = [(sc, n) for sc, (_, _, _, n, _) in zip(stale_counts, out) if n >= 3]
                    stale_report.setdefault(dm_name, []).extend(sc for sc, _ in pub)
                    for t_pub, sd, rm, n, _ in out:
                        if n < 3:            # Rust quality filter
                            continue
                        ref_s, ref_r = reference(t_true, rr_true, t_pub - win / 2)
                        rows_s.append((s, sd, ref_s)); rows_r.append((s, rm, ref_r)); counts.append(n)
                else:
                    out = run_fixed(dets[s], t_end, window_s=win, sleep_s=sleep_s)
                    for t_pub, sd, rm, n in out:
                        if n < 3:
                            continue
                        ref_s, ref_r = reference(t_true, rr_true, t_pub - win / 2)
                        rows_s.append((s, sd, ref_s)); rows_r.append((s, rm, ref_r)); counts.append(n)
            results[(dm_name, cfg_name)] = (summarise(rows_s), summarise(rows_r), np.median(counts))
            # low-HRV subjects (whole-recording RMSSD < 20 ms) — the diabetic-neuropathy range
            low = [r for r in rows_r if np.isfinite(r[1]) and np.isfinite(r[2]) and subj_rmssd[r[0]] < 20]
            if low:
                lowtab.setdefault(dm_name, {})[cfg_name] = (np.median([r[2] for r in low]), np.median([r[1] for r in low]))

    # Reference stability: 5-min HRV itself, consecutive non-overlapping blocks
    ref_rows_s, ref_rows_r = [], []
    for s, (t_true, rr_true) in subjects.items():
        for c in np.arange(REF_S / 2, t_true[-1] - REF_S / 2, REF_S):
            a, b = reference(t_true, rr_true, c)
            ref_rows_s.append((s, a, a)); ref_rows_r.append((s, b, b))
    ref_s, ref_r = summarise(ref_rows_s), summarise(ref_rows_r)

    hdr = f"{'config':40s} {'beats':>5s} | {'r':>5s} {'bias':>6s} {'±LoA':>6s} {'|err|%':>6s} {'wCV%':>5s} {'ICC':>5s}"
    for dm_name in detection_models:
        print(f"\n=== Detection model: {dm_name} ===")
        for metric_idx, metric in ((0, "SDNN"), (1, "RMSSD")):
            print(f"\n  {metric} vs 5-min reference")
            print("  " + hdr)
            for cfg_name, _, _ in configs:
                st = results[(dm_name, cfg_name)][metric_idx]
                n = results[(dm_name, cfg_name)][2]
                print(f"  {cfg_name:40s} {n:5.0f} | {st['r']:5.2f} {st['bias']:6.1f} {st['loa']:6.1f} "
                      f"{st['mape']:6.1f} {st['wcv']:5.1f} {st['icc']:5.2f}")
            rs = ref_s if metric == "SDNN" else ref_r
            print(f"  {'(5-min reference itself, block-to-block)':40s} {'':5s} | {'':5s} {'':6s} {'':6s} {'':6s} "
                  f"{rs['wcv']:5.1f} {rs['icc']:5.2f}")
        st = stale_report.get(dm_name, [])
        if st:
            frac = np.mean([a > 0 for a, _ in st]) * 100
            share = np.mean([a / b for a, b in st]) * 100
            print(f"\n  Current firmware: {frac:.1f}% of published readings include intervals from an earlier window "
                  f"({sleep_s + WAKE_OVERHEAD_S:.0f}+ s old); mean stale share of the 8-slot buffer {share:.1f}%")
        print("\n  Low-HRV subjects (RMSSD < 20 ms): median true 5-min RMSSD -> median measured")
        for cfg_name, (t_, m_) in lowtab.get(dm_name, {}).items():
            print(f"    {cfg_name:40s} {t_:5.1f} -> {m_:5.1f} ms  ({(m_/t_-1)*100:+.0f}%)")


if __name__ == "__main__":
    main()
