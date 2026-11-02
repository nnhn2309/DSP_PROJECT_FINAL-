# DE2 + WM8731 Feedback ANC – Rev C.1 (RTL prototype / verification package)

**Ngày tạo:** 2026-10-31. **Mục tiêu:** loại nhiễu vật lý tại error microphone trong ống dài 2m, D=0.2m, loa điều khiển ở nhánh 90°. Không phải chương trình lọc WAV rồi phát lại.

## Trạng thái kiểm chứng — đọc trước khi dùng

- **ĐÃ CHẠY:** `python sim/test_reference.py` (9 phép kiểm: toán 24x16, 32x16, lượng tử hóa, FIR, circular buffer, thuật toán floating-point), `python sim/static_verify.py` (74 phép kiểm cấu trúc/QSF). Xem `results/*`.
- **CHƯA CHẠY:** Icarus Verilog/ModelSim RTL simulation, Quartus Analysis & Synthesis, Fitter/TimeQuest, SignalTap, thử nghiệm vật lý trên DE2. Máy tạo project không có các phần mềm này.
- **KHÔNG ĐƯỢC** hiểu 40.26 dB trong `reference_test_results.json` là mức giảm nhiễu của RTL hay board thật. Đó là mô hình floating-point lý tưởng với S xác định chính xác, không có trễ I²S thực.
- **Rev C.1 là mã nguyên mẫu phục vụ triển khai/kiểm thử**, không phải thiết kế đã chứng minh setup slack đạt hoặc acoustic ANC hoạt động.

## 1. Sơ đồ khối

```text
MIC IN -> WM8731 16bit 48k -> I2S -> CDC toggle mailbox -> DC blocker (SW5)
 -> FIR127 anti-alias @48k -> Decimate /8 -> PCM @6k
 -> 6k Feedback FxLMS core with M4K RAM / circular pointers:
      CAL: PRBS-31 -> speaker, Offline S_hat LMS -> validation NMSE (-6 dB)
      ANC: prediction using S_hat + D_bulk; W FIR (Q2.38 -> Q2.22);
           filtered reference Xf Q2.30; W-update Q2.38 + optional leakage
 -> linear interpolator 6k->48k (FOH) -> output toggle mailbox -> I2S WM8731
 -> SW3 hard digital emergency mute -> LINE OUT -> low-gain amplifier -> control speaker
```

Important: S_hat means **LMS trained during calibration, then FROZEN during ANC**. `D_BULK` is implemented by circular address subtraction in all S paths, not a separate 512-element shift register. `DELTA` is the independent reference decorrelation delay, by default 0.

The core uses a **multicycle staged MAC**: synchronous fetch → register coefficient/data → 17×16 & 8×16 multiplications → 64-bit product + accumulate. This is NOT yet a fully overlapped 1-tap-per-clock three-stage throughput pipeline. Target throughput, not per-tap latency, is what matters for 6 kHz.

## 2. Technical settings

| Property | Value |
|---|---|
| DE2 Cyclone II | EP2C35F672C6 |
| `CLOCK_50` | 50 MHz |
| Audio MCLK | 12 MHz PLL |
| WM8731/I²S | Master, 16-bit stereo, target 48kHz; actual rate must be measured |
| Internal DSP | 6 kHz, every 8th 48-kHz frame |
| Decimator | 127-tap symmetric Kaiser FIR Q1.15, cutoff ~1.35kHz |
| Reconstruction | First-order hold 8× (NOT high-quality image-rejection FIR) |
| `S_hat` | 128 tap x 24-bit Q2.22; updated only in CAL |
| `W` | 64 tap x 40-bit Q2.38; MAC quantized 24-bit Q2.22 |
| `Xf_hist` | 256 x 32-bit Q2.30, M4K style |
| History buffers | 256 entry circular ring; synchronous 1R/1W BRAM-style |
| `D_BULK` | **12 samples = 2 ms at 6k**, only initial placeholder; measure |
| `DELTA` | 0 samples (optionally change parameter) |
| Calibration | 36,000 training + 2,048 independent validation samples (~6.34s) |
| S validation threshold | NMSE < -6.02dB, strong-signal gate; *initial target* |
| Calibration timeout | 500 million 50MHz cycles (~10s) |
| W leakage | 1 - 2^-15, always enabled in current prototype |
| W learning rate | `SW2:1`=00/01/10/11 -> 2^-6 / 2^-7 / 2^-8 / 2^-9 |
| DAC limiter | approximately ±8192 PCM counts |

### Duct measurement caveat

At 6kHz, 12 samples = 2ms. Pure secondary delay **includes** DAC interpolation + codec + speaker + side branch + main duct + ADC + FIR decimation group delay. At 48kHz FIR127 has 63/48k≈1.3125ms group delay. Do not assume full 2m propagation (~5.83ms) equals the secondary delay. The branch-to-mic distance is what matters. Before meaningful calibration, identify D_BULK from actual impulse response and recompile. The current input/output multirate filters significantly affect phase.

## 3. Controls

| Physical | Action |
|---|---|
| KEY0 | Active-low reset (press, release) |
| KEY1 | Start calibration when SW3=0 and SW6=0; press and release |
| SW0 | ANC enable after calib_valid |
| SW2:1 | W-step (11 is the smallest step) |
| SW3=1 | Digital emergency mute, immediately forces AUD_DACDAT low, aborts calibration and invalidates previous calibration; use amplifier power-off as physical backup |
| SW4 | ADC L / ADC R |
| SW5=1 | Enable additional software DC blocker; codec internal ADC HPF is already configured |
| SW6=1 | Low-level **200Hz triangular** DAC test tone, disables ANC core; SW3 must be 0 to hear tone |

For FIRST boot: `SW3=1`, `SW0=0`, `SW6=0`, amplifier gain minimum. SW3 changes mid I²S data word can create a discontinuity but always forces zeros. Release it only in a controlled test.

### LEDG indicators

- LEDG0 codec I²C config-ready (not full audio functional verification)
- LEDG1 ADC sample seen in last 100ms
- LEDG2 6k DSP sample seen in last 100ms
- LEDG3 calibration busy
- LEDG4 calibration validation passed
- LEDG5 ANC active
- LEDG6 sticky ADC/DSP/decimator/DAC-mailbox overrun
- LEDG7 calibration failed, core fault or I²C config error
- LEDG8 SW3 mute or internal safety mute

LEDR0..15 = microphone PCM peak VU meter (not SPL). LEDR16 = DSP last-cycle count >=7000; LEDR17 = calibration failure.

## 4. Run the tests, in order

### Python executed already in this package

```bash
cd DE2_ANC_RevC1
python3 -m pip install numpy scipy matplotlib
python3 sim/test_reference.py
python3 sim/static_verify.py
```

The tests are numeric/structural and **do not compile Verilog**. See logs and PNG `results/ideal_float_anc_200hz.png`.

### Run HDL regressions on Ubuntu/WSL or native Linux

```bash
sudo apt update && sudo apt install -y iverilog
bash sim/run_iverilog.sh
```

Testbenches:

- `tb_fixed_point`: 20,000 signed 24×16 split-product checks and fractional-gradient accumulation.
- `tb_dc_blocker`, `tb_interpolator`: remove DC and restore 6k targets over 8 output frames.
- `tb_decimator8`: constant input sample, filter warmup and 6k sample outputs.
- `tb_fxlms_calibration`: small-tap closed-loop S identification, validate, SW3 abort.
- `tb_closed_loop_duct`: small synthetic plant + 200Hz primary tone, tests >3dB attenuation; **this assertion has not been run in this environment**, so it may expose RTL issues to fix.

Do NOT proceed to loudspeaker testing if any test fails. For ModelSim/Questa use `sim/modelsim.do`.

### Quartus II 13.0 (or compatible with Cyclone II)

1. Open `quartus/de2_anc.qpf` (project lives in its `quartus/` directory).
2. Run Analysis & Synthesis; inspect `M4K blocks`, `Logic Elements`, `embedded multipliers`.
3. Run full compilation. Inspect timing paths, setup/hold slack and overrun signal.
4. Re-check `AUD_XCK` 12MHz, `AUD_ADCLRCK` 48kHz, `AUD_BCLK` from WM8731. The old design showed a forwarded-PLL-clock routing warning; it has not been cleared by this revision.
5. If register arrays appear as flip-flops instead of M4K, inspect the RAM inference report and read/write patterns. `ramstyle="M4K"` is a hint, **not proof**.
6. Compile `.sof`; do not skip timing signoff because software arithmetic tests passed.

### SignalTap probes

- Audio: `mic50`, `adc_sample50`, `dc_audio`, `dsp_input_valid`, `dsp_input`
- Core: `anc.state`, `anc.op`, `anc.tap`, `anc.cycles`, `anc.last_cycles`, `anc.calib_valid`, `anc.calib_failed`
- Precision: `anc.xf_current`, `anc.product_full`, `anc.debug_w_q38` (choose a small number of probes to conserve M4K)
- Safety: `anc.dsp_overrun`, `anc.fault_latch`, `decim_overrun`, `dac_mailbox_overrun`, `SW[3]`
- DAC: `interp_out`, `playback_hold50`, `playback_bclk`, `AUD_DACDAT`

## 5. Board bring-up (all checks must pass)

0. **Power/safety:** active speaker connected to LINE OUT only through line-level input/amplifier; no speaker signal into microphone port. Start with amplifier gain zero. Physically accessible power-off switch.
1. **ADC:** SW3=1, SW0=0. Check LEDG0,1,2. Talk into mic; VU changes. Switch SW4 to identify working ADC channel. If no activity, verify microphone bias and I²C routing before ANC.
2. **DAC only:** SW6=1, SW3=0, SW0=0, amplifier gain very low. Expect a *200Hz triangle tone* at LINE OUT (not a clean sine). Exercise SW3 and ensure silence. If output is not stable, no calibration.
3. **Calibration:** SW6=0, SW3=0, SW0=0, primary noise speaker OFF. Fixed speaker/mic geometry. Press-release KEY1, wait ~6.3s. Expect LEDG3 busy then LEDG4 valid. If LEDG7 or LEDR17 faults, troubleshoot input gain/D_BULK/stationary geometry before proceeding.
4. **Closed-loop 200Hz:** primary speaker emits quiet sine200. With ANC OFF, capture baseline e[n]; set SW2:1=11 and SW0=1. Watch clipping and LEDG6/7. If loudspeaker becomes louder or whistles, SW3=1 immediately and power-off amplifier if needed.
5. **Quantify:** capture same error mic channel with ADC/SigTap for ANC OFF and ON, same volume and positions. `20*log10(rmsOFF/rmsON)` must be positive to claim reduction. Check 100, 200, 300, 500, 800Hz separately and scan spectral peaks. Repeat 3 times.

### Important remaining limitations

- Not yet simulated with Icarus/ModelSim or timed with Quartus in this environment.
- Default D_BULK=12 is not measured; an incorrect delay can make ANC worsen noise.
- FOH interpolation has finite attenuation of 6kHz images; replace with a filtered interpolator before claiming broadband results.
- Python idealized S validation/ANC results do not account for actual WM8731 dynamics, quantization saturation, speaker nonlinearities, or 90-degree side-branch reflections.
- Not a validated feedback control safety product. Divergence detection currently latches sustained output limiter or near-full-scale ADC, plus overrun; it is not a complete energy-based stability monitor.
- Setup slack and acoustic cancellation must be measured, not assumed.
