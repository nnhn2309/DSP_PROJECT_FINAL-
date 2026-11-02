transcript on
if {[file exists work]} {vdel -lib work -all}
vlib work
vlog -sv rtl/feedback_fxlms_bram.v rtl/decimator8_fir127.v rtl/dc_blocker.v rtl/interpolator8_linear.v tb/tb_fixed_point.sv tb/tb_decimator8.sv tb/tb_dc_blocker.sv tb/tb_interpolator.sv tb/tb_fxlms_calibration.sv tb/tb_closed_loop_duct.sv
foreach test {tb_fixed_point tb_decimator8 tb_dc_blocker tb_interpolator tb_fxlms_calibration tb_closed_loop_duct} {
  puts "=== RUNNING $test ==="
  vsim -c work.$test
  run -all
  quit -sim
}
