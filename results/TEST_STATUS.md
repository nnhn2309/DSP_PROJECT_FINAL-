# Test status — DE2 ANC Rev C.1

Generated 2026-10-31. PASS means **executed** on this container unless marked otherwise.

| Test | Status | Evidence |
|---|---|---|
| Signed 24×16 split multiplication, 20,000 random vectors | PASS (Python) | `reference_run.log` |
| Signed 32×16 gradient split, 20,000 random vectors | PASS (Python) | `reference_run.log` |
| Small gradient accumulation Q2.38, signed rounding | PASS (Python) | `reference_run.log` |
| Kaiser FIR127 ROM, 800Hz passband, 3kHz stopband | PASS (Python) | `reference_run.log` |
| Circular ring wrap / 2,000 writes | PASS (Python) | `reference_run.log` |
| Offline LMS ideal synthetic S, NMSE | PASS (Python floating) | `reference_run.log` |
| Feedback FxLMS synthetic sine 200Hz attenuation | PASS (Python floating) | `reference_run.log`, `ideal_float_anc_200hz.png` |
| 74 structure/QSF/ROM checks | PASS (Python static) | `static_run.log` |
| Icarus tb_fixed_point, tb_dc_blocker, tb_interpolator, tb_decimator8 | NOT RUN | Icarus missing |
| Icarus tb_fxlms_calibration, tb_closed_loop_duct | NOT RUN | Icarus missing |
| ModelSim/Questa top syntax compile | NOT RUN | ModelSim missing |
| Quartus Cyclone II compilation | NOT RUN | Quartus missing |
| M4K inference/resource, clock timing/Slack | NOT RUN | Quartus missing |
| DE2 audio, calibration, acoustic cancellation | NOT RUN | Physical DE2 inaccessible here |

**Interpretation:** prototype RTL was written; algorithm feasibility was tested in a simple idealized floating-point numerical model. Neither RTL pass nor FPGA success is certified by these results. Run `bash sim/run_iverilog.sh` then Quartus before hardware.
