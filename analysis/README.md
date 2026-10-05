# HRV analysis

Simulation studies behind the HRV window and algorithm changes in the firmware.

```sh
./fetch_fantasia.sh                  # download the PhysioNet Fantasia subset (not committed)
python3 hrv_window_study.py          # window-length study, 5-minute cycle (default)
python3 hrv_window_study.py --sleep 10
```

Requires Python 3 and NumPy.

## What it does

`hrv_window_study.py` replays the firmware's HRV computation on real RR intervals
(Fantasia: 10 healthy adults, about 2 h of supine rest each). It compares each
published value with the standard 5-minute HRV computed from the same ECG.

- **Prototype firmware** (`run_firmware`): reproduces `src/main.cpp` at 4f57341.
  - 15 s window, then sleep, then about 4 s of blocking wake-up.
  - 8-slot RR buffer that isn't cleared between windows.
  - RMSSD differences taken in buffer-index order.
  - `lastBeat` updated before the 45-150 bpm filter.
  - Readings with fewer than 3 new beats dropped, as the subscriber's filter does.
- **Proposed algorithm** (`run_fixed`):
  - RR intervals collected per window, in time order, reset each window.
  - Successive differences taken only between truly consecutive beats.
  - 20% median artefact rule.
  - Quality gate: at least 80% of intervals kept, and at least 70% of the window covered.
- **Beat detection** is modelled on the true ECG beat times:
  - quantization to the sensor's sample rate (100 Hz);
  - Gaussian timing jitter;
  - missed and extra beats.

  These detection parameters are assumptions, not measurements of the MAX30102.
- **Metrics**, comparing published values with the 5-minute reference:
  - Pearson r on log values;
  - bias, and 95% limits of agreement (1.96 SD of the difference, in ms);
  - reading-to-reading within-subject CV;
  - ICC(1) on log values.

Random seed: 7 (`RNG_SEED`), re-seeded for each detection setting.

## Sensitivity grid

```sh
python3 hrv_sensitivity_grid.py      # writes hrv_sensitivity_grid.csv
```

The detection error model above is an assumption, so the grid tests how much the
conclusions depend on it. All 36 combinations of:

- timing jitter {2, 5, 10} ms
- missed beats {0, 2, 5, 10} %
- extra beats {0, 1, 3} %

run on the 5-minute cycle (300 s sleep) at the sensor's 100 Hz. Each setting runs
the prototype computation (15 s) and the proposed one (60 s), with the same functions
and seed as the window study. The setting 5 ms / 2% / 1% reproduces that study's
5-minute-cycle results exactly.

Columns in `hrv_sensitivity_grid.csv` (one row per setting and configuration):

| Column | Meaning |
|---|---|
| `n_readings`, `median_beats` | Published readings compared (after the 3-beat filter, and for `proposed_60s` the quality gate), and median beats per reading |
| `*_r` | Pearson r between log values and the 5-min reference |
| `*_bias_ms`, `*_loa_ms` | Mean difference from the reference, and 1.96 SD of the difference (ms) |
| `*_cv_pct` | Reading-to-reading within-subject CV (%). The 5-min reference itself: 9.5% (RMSSD), 16.4% (SDNN) |
| `*_icc` | ICC(1) on log values. The 5-min reference itself: 0.96 (RMSSD), 0.82 (SDNN) |
| `older_*` | Older subjects (O1-O5, low HRV): median true 5-min RMSSD, median measured RMSSD, median per-reading difference (`_bias_ms`), and ratio of medians − 1 (`_bias_pct`, the definition used in the window study) |

Results, minimum and maximum over the 36 settings:

| Metric | Prototype (15 s) | Proposed (60 s) |
|---|---|---|
| RMSSD r | 0.29 – 0.79 | 0.87 – 0.97 |
| RMSSD 95% LoA (ms) | ±39.8 – ±205.4 | ±12.5 – ±18.7 |
| RMSSD reading-to-reading CV | 32.7% – 92.4% | 13.5% – 29.1% |
| RMSSD ICC | 0.07 – 0.64 | 0.76 – 0.89 |
| SDNN r | 0.26 – 0.73 | 0.82 – 0.85 |
| SDNN 95% LoA (ms) | ±44.6 – ±169.1 | ±29.9 – ±33.4 |
| SDNN CV | 36.7% – 100.9% | 22.9% – 35.4% |
| SDNN ICC | 0.06 – 0.61 | 0.57 – 0.65 |
| Older-subject RMSSD bias (ms, median difference) | +2.2 – +20.1 | +1.2 – +13.9 |
| Older-subject RMSSD bias (%, ratio of medians) | +15.0% – +145.1% | +8.9% – +96.8% |

The older-subject bias depends mostly on jitter. For the proposed configuration:
+8.9 to +18.5% at 2 ms, +29.1 to +39.9% at 5 ms, and +85.4 to +96.8% at 10 ms
(true median RMSSD 16.1 ms). A longer window does not remove it; only better beat
timing does.
