# Data

Raw CSV logs from each experiment go here. Every plot in the main README was made from these files,
so anyone can re-check my numbers.

| Folder / file (add these) | Experiment | Columns |
|---|---|---|
| `stage3/stage3_load0_v2.csv`, `stage3_load200*.csv`, `stage3_load400*.csv` | Stage 3: shoulder step response vs load | `t_sec, target_deg_rel, actual_deg_rel, actual_raw, target_raw` |
| `stage4/temp_run_0g.csv`, `temp_run_400g.csv` | Stage 4: shoulder heating under static load | `t_sec, temperature_C` |
| `stage4b/base_dur_0.5.csv` … `base_dur_0.05.csv` | Stage 4b: base-joint saturation runs | same as Stage 3 |
| `stage5/heading_openloop_floor.csv`, `heading_openloop_paper.csv`, `heading_closedloop_paper.csv` | Stage 5: heading control | `t_sec, target_yaw, actual_yaw, error_deg, omega_cmd` |

> Rename the files to match what you actually have. If a run was repeated, keep every repeat and say which one is plotted.
