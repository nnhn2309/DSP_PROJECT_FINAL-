#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if ! command -v iverilog >/dev/null 2>&1;then
 echo 'ERROR: Icarus Verilog is not installed. Run on Ubuntu/WSL: sudo apt install iverilog' >&2
 exit 127
fi
mkdir -p results
run(){
 local name="$1";shift
 echo "Running $name"
 iverilog -g2012 -s "$name" -o "results/${name}.vvp" "$@" "tb/${name}.sv"
 vvp "results/${name}.vvp" | tee "results/${name}.log"
}
run tb_fixed_point
run tb_dc_blocker rtl/dc_blocker.v
run tb_interpolator rtl/interpolator8_linear.v
run tb_decimator8 rtl/decimator8_fir127.v
run tb_fxlms_calibration rtl/feedback_fxlms_bram.v
run tb_closed_loop_duct rtl/feedback_fxlms_bram.v
printf 'All executed HDL testbenches PASS. Note: this is not Quartus Fitter/TimeQuest.\n'
