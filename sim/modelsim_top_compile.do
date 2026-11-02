# Top-level RTL syntax simulation compile with behavioral PLL; no timing Fitter.
if {[file exists work]} {vdel -lib work -all}
vlib work
vlog +define+SIMULATION rtl/audio_pll.v rtl/wm8731_config.v rtl/i2s_master_codec.v rtl/lfsr_prbs31.v rtl/dc_blocker.v rtl/decimator8_fir127.v rtl/interpolator8_linear.v rtl/feedback_fxlms_bram.v rtl/de2_anc_top.v
