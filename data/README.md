# Data

Raw CSV logs from each experiment. Every Part B figure in the main README was made from these files, and every number in the README's result tables was recomputed from them (details below), so anyone can re-check my numbers.

The Part A (ESP32) figures are screenshots of the Arduino Serial Plotter, so there are no CSV files for them.

## Files

| File | Experiment | Figure it feeds | Columns |
|---|---|---|---|
| `stage3/stage3_load0_v2.csv`, `stage3_load200_v2.csv`, `stage3_load400_v2.csv` | Stage 3: shoulder (ID 2) step of −60° at 0 g, 200 g, 400 g, logged at about 50 Hz for 6 s | `figures/s3_step_vs_load.png` | `t_sec, target_deg_rel, actual_deg_rel, actual_raw, target_raw` |
| `stage3/stage3_combined.csv` | The three Stage 3 runs resampled onto one common 20 ms time axis (first 3 s) for the overlay plot. Some values are interpolated, so use the three files above for any measurement. | `figures/s3_step_vs_load.png` | `t_sec, target, actual_0g, actual_200g, actual_400g` |
| `stage4/temp_0g.csv`, `temp_400g.csv` | Stage 4a: shoulder temperature every 10 s for 180 s | `figures/s4_heating_under_load.png` | `t_sec, temperature_C` |
| `stage4b/inertia_dur050.csv`, `inertia_dur030.csv`, `inertia_dur020.csv`, `inertia_dur015.csv`, `inertia_dur005.csv` | Stage 4b: base (ID 1) moved with a commanded duration of 0.5, 0.3, 0.2, 0.15 and 0.05 s | `figures/s4b_saturation_curves.png` | same as Stage 3 |
| `stage5/open_normal.csv`, `open_friction.csv`, `closed_friction.csv` | Stage 5: 10 s straight runs at 0.1 m/s. Baseline (Kp = 0) on the normal floor, baseline with paper under one side, and closed loop (Kp = 1.5) with paper | `figures/s5_heading_error.png` | `t_sec, target_yaw, actual_yaw, error_deg, omega_cmd` |
| `stage5/heading_test.csv` | Stage 5: the 3 s wheels-off-the-ground test of the controller (Kp = 1.5, Kd = 0.2). The heading error grows to about 5.2° and the commanded turn rate rises with it (0.136 rad/s = 1.5 × 5.19°). Not used in any table or figure. | none | same as the other Stage 5 files |

Notes on specific files:

- `error_deg` in the Stage 5 files is target minus actual, in degrees.
- All five Stage 4b commands target position 1250, which is past the end of the servo's 0–1000 range, so every run stops at about position 1000 (about 120° of travel).
- `inertia_dur050.csv` starts logging 0.8 s after the command and its `actual_deg_rel` column is offset by about 121° compared with the others (its zero reference appears to have been taken near position 0). The arm itself moved from position 503 to about 1000 like the other four runs. The plot aligns every run by displacement from its own start.

## How the README numbers were recomputed

| Quantity | Result from the CSVs |
|---|---|
| Stage 3 rise time (first 20 ms sample past −54°, from the command) | 0 g: 0.40 s, 200 g: 0.42 s, 400 g: 0.50 s |
| Stage 3 overshoot past −60° | 0 g: 6.00°, 200 g: 8.40°, 400 g: 10.56° |
| Stage 3 final angle (last sample) | 0 g: −63.12°, 200 g: −64.32°, 400 g: −65.76° |
| Stage 4a temperature rise over 180 s | 0 g: 57 → 68 °C (+11), 400 g: 45 → 75 °C (+30), ratio 2.7 |
| Stage 4b plateau speed (0.2–0.45 s after motion starts) | 202–209 °/s, which is 3.5–3.65 rad/s, in all five runs |
| Stage 4b time from first motion to position 995 | 0.62–0.67 s in all five runs |
| Stage 5 RMS heading error | normal floor 0.78°, paper baseline 0.37°, closed loop 0.10° |
| Stage 5 max heading error | 1.09°, 0.71°, 0.28° |
| Stage 5 final error (mean of last 0.5 s) | −0.85°, +0.59°, +0.04° |
| Stage 5 controller gains (least-squares fit of `omega_cmd` to the error and its rate) | baseline runs: Kp 0.00, Kd 0.20. Closed loop: Kp 1.50, Kd 0.20. Fit R² = 1.0 |

If a run was repeated, keep every repeat here and say which one is plotted. At the moment there is one run per condition.
