`timescale 1ns/1ps
// Small parameter configuration for RTL regression; synthetic plant S=0.5*z^-2.
// Not an acoustic measurement and not the production 128/64-tap timing configuration.
module tb_fxlms_calibration;
 reg clk=0,rst_n=0,calib_start=0,anc_enable=0,emergency_mute=0;
 reg [1:0] mu_sel=2'b11;
 reg sample_valid=0;
 reg signed [15:0] mic_in=0,prbs_sample=0;
 wire signed [15:0] spk_out;
 wire sample_done,calib_busy,calib_valid,calib_failed,anc_active,dsp_overrun,fault_latch;
 wire [15:0] last_cycles;
 wire signed [31:0] debug_xf_q30;
 wire signed [39:0] debug_w_q38;
 wire [7:0] debug_state;
 reg signed [15:0] y_hist[0:2200];
 reg [30:0] lfsr=31'h4567891;
 integer n;
 always #10 clk=~clk;
 feedback_fxlms_bram #(.M_TAPS(8),.L_TAPS(4),.D_BULK(2),.DELTA(0),
   .CALIB_SAMPLES(1600),.VALID_SAMPLES(200),.S_MU_SHIFT(3),.HIST_AW(5)) dut(
 .clk(clk),.rst_n(rst_n),.calib_start(calib_start),.anc_enable(anc_enable),
 .emergency_mute(emergency_mute),.mu_sel(mu_sel),.sample_valid(sample_valid),
 .mic_in(mic_in),.prbs_sample(prbs_sample),.spk_out(spk_out),
 .sample_done(sample_done),.calib_busy(calib_busy),.calib_valid(calib_valid),
 .calib_failed(calib_failed),.anc_active(anc_active),
 .dsp_overrun(dsp_overrun),.fault_latch(fault_latch),.last_cycles(last_cycles),
 .debug_xf_q30(debug_xf_q30),.debug_w_q38(debug_w_q38),.debug_state(debug_state));
 initial begin
   for(n=0;n<2201;n=n+1)y_hist[n]=0;
   repeat(6) @(negedge clk);rst_n=1;
   repeat(40) @(negedge clk);calib_start=1;
   @(negedge clk);calib_start=0;
   repeat(40) @(negedge clk);
   for(n=0;n<1805;n=n+1) begin
     if(n>=2) mic_in=y_hist[n-2] >>> 1;else mic_in=0;
     prbs_sample=lfsr[30]?16'sd8192:-16'sd8192;
     lfsr={lfsr[29:0],(lfsr[30]^lfsr[27])};
     sample_valid=1;
     @(negedge clk);sample_valid=0;
     repeat(160) @(negedge clk);
     y_hist[n]=spk_out;
   end
   repeat(80) @(negedge clk);
   if(dsp_overrun||fault_latch||calib_failed) $fatal(1,"calibration fault");
   if(!calib_valid || calib_busy) $fatal(1,"calibration did not validate");
   if(dut.s_ram[0]<24'sd1500000 || dut.s_ram[0]>24'sd2500000)
     $fatal(1,"S_hat[0] not around 0.5: %0d",$signed(dut.s_ram[0]));
   emergency_mute=1;
   repeat(4) @(negedge clk);
   if(spk_out!==0 || calib_valid) $fatal(1,"emergency mute failure");
   $display("PASS tb_fxlms_calibration: validated S_hat[0]=%0d",dut.s_ram[0]);
   $finish;
 end
endmodule
