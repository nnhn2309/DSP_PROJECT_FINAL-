`timescale 1ns/1ps
// Simplified full closed-loop test around the 6-kHz core (not full WM8731 48-kHz RTL).
// Synthetic plant S=0.5*z^-5+0.10*z^-8, primary 200Hz tone.
module tb_closed_loop_duct;
 reg clk=0,rst_n=0,calib_start=0,anc_enable=0,emergency_mute=0;
 reg [1:0] mu_sel=0;
 reg sample_valid=0;
 reg signed [15:0] mic_in=0,prbs_sample=0;
 wire signed [15:0] spk_out;
 wire sample_done,calib_busy,calib_valid,calib_failed,anc_active,dsp_overrun,fault_latch;
 wire [15:0] last_cycles;
 wire signed [31:0] debug_xf_q30;
 wire signed [39:0] debug_w_q38;
 wire [7:0] debug_state;
 reg signed [15:0] y[0:14000];
 reg [30:0] lfsr=31'h6c12345;
 integer n,tone,env,phase;
 reg signed [63:0] off_energy=0,on_energy=0;
 real nr;
 always #10 clk=~clk;
 feedback_fxlms_bram #(.M_TAPS(16),.L_TAPS(16),.D_BULK(5),.DELTA(0),
 .CALIB_SAMPLES(2500),.VALID_SAMPLES(256),.S_MU_SHIFT(3),.HIST_AW(6)) dut(
 .clk(clk),.rst_n(rst_n),.calib_start(calib_start),.anc_enable(anc_enable),
 .emergency_mute(emergency_mute),.mu_sel(mu_sel),.sample_valid(sample_valid),
 .mic_in(mic_in),.prbs_sample(prbs_sample),.spk_out(spk_out),
 .sample_done(sample_done),.calib_busy(calib_busy),.calib_valid(calib_valid),
 .calib_failed(calib_failed),.anc_active(anc_active),.dsp_overrun(dsp_overrun),
 .fault_latch(fault_latch),.last_cycles(last_cycles),
 .debug_xf_q30(debug_xf_q30),.debug_w_q38(debug_w_q38),.debug_state(debug_state));
 initial begin
  for(n=0;n<14001;n=n+1)y[n]=0;
  repeat(8) @(negedge clk);rst_n=1;
  repeat(75) @(negedge clk);calib_start=1;
  @(negedge clk);calib_start=0;
  repeat(70) @(negedge clk);
  // 2500 training + 256 independent validation + 1200 OFF + 7000 ANC frames.
  for(n=0;n<12000;n=n+1)begin
   if(n<3000) begin
     tone=0;prbs_sample=lfsr[30]?16'sd8192:-16'sd8192;
     lfsr={lfsr[29:0],lfsr[30]^lfsr[27]};
   end else begin
     tone=$rtoi(5500.0*$sin(6.28318530718*200.0*(n-3000)/6000.0));
     prbs_sample=0;
     if(n==4200)anc_enable=1;
   end
   env=tone;
   if(n>=5)env=env+($signed(y[n-5]) >>> 1);
   if(n>=8)env=env+($signed(y[n-8]) / 10);
   mic_in=env;
   sample_valid=1;
   @(negedge clk);sample_valid=0;
   repeat(650) @(negedge clk);
   y[n]=spk_out;
   if(n>=3200&&n<4200)off_energy=off_energy+$signed(mic_in)*$signed(mic_in);
   if(n>=11000&&n<12000)on_energy=on_energy+$signed(mic_in)*$signed(mic_in);
  end
  if(calib_failed||dsp_overrun||fault_latch||!calib_valid)
    $fatal(1,"calibration, deadline, or safety failed");
  nr=10.0*$ln(($itor(off_energy)+1.0)/($itor(on_energy)+1.0))/$ln(10.0);
  if(nr<3.0)$fatal(1,"ANC did not attenuate 200 Hz: %f dB",nr);
  $display("PASS tb_closed_loop_duct: ANC 200Hz reduction=%f dB",nr);
  $finish;
 end
endmodule
