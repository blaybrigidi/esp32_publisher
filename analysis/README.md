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
