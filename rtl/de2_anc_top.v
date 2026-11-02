`timescale 1ns/1ps
// DE2 Cyclone II + WM8731. Codec-master I2S PCM 16-bit @48kHz;
// DSP internal 6 kHz, decimation FIR127, first-order-hold interpolation.
// KEY0 reset, KEY1 CALIBRATE; SW0 ANC; SW2:1 mu; SW3 emergency mute;
// SW4 select L/R ADC; SW5 DC blocker enable; SW6 200Hz low-level DAC triangle.
// SW6 diagnostic mode is for DAC / signal-chain bring-up, not ANC mode.
module de2_anc_top(
 input wire CLOCK_50,input wire [1:0] KEY,input wire [6:0] SW,
 output wire [8:0] LEDG,output wire [17:0] LEDR,
 input wire AUD_BCLK,AUD_ADCLRCK,AUD_DACLRCK,AUD_ADCDAT,
 output wire AUD_XCK,AUD_DACDAT,I2C_SCLK,
 inout wire I2C_SDAT
);
 wire pll_locked,config_ready,config_error;
 audio_pll pll(.clk50(CLOCK_50),.reset(~KEY[0]),.mclk(AUD_XCK),.locked(pll_locked));
 wire global_rst_n=KEY[0] & pll_locked;
 reg [1:0] reset50;
 always @(posedge CLOCK_50 or negedge global_rst_n)
 if(!global_rst_n) reset50<=0;else reset50<={reset50[0],1'b1};
 wm8731_config #(.USE_LINE_REF(0),.MIC_BOOST(1)) cfg(
 .clk(CLOCK_50),.rst_n(reset50[1]),.scl(I2C_SCLK),.sda(I2C_SDAT),
 .ready(config_ready),.error(config_error));
 wire codec_rst_n=global_rst_n & config_ready;
 reg [1:0] reset_bclk;
 always @(posedge AUD_BCLK or negedge codec_rst_n)
 if(!codec_rst_n) reset_bclk<=0;else reset_bclk<={reset_bclk[0],1'b1};
 wire signed [15:0] adc_left,adc_right;
 wire audio_sample_valid;
 reg signed [15:0] playback_bclk;
 wire dac_serial;
 i2s_master_codec codec(
 .bclk(AUD_BCLK),.rst_n(reset_bclk[1]),
 .adc_lr(AUD_ADCLRCK),.dac_lr(AUD_DACLRCK),.adc_data(AUD_ADCDAT),
 .playback(playback_bclk),.dac_data(dac_serial),
 .left(adc_left),.right(adc_right),.sample_valid(audio_sample_valid));

 // Immediate asynchronous ASSERTION of SW3 forces DAC serial to zero;
 // synchronously sampled SW3 also clears playback. Deassert only after reset/rearm.
 // This is digital silence, NOT amplifier power disconnect.
 assign AUD_DACDAT=SW[3] ? 1'b0 : dac_serial;
 reg [1:0] mute_bclk,channel_bclk;
 // Input toggle mailbox: 1 outstanding transfer, held 16-bit payload.
 reg [15:0] adc_hold_bclk;
 reg adc_req_bclk;
 reg [2:0] adc_ack_sync_bclk;
 reg adc_overrun_bclk;
 reg adc_ack_50;
 wire signed [15:0] selected_adc=channel_bclk[1]?adc_right:adc_left;
 always @(posedge AUD_BCLK or negedge reset_bclk[1]) begin
 if(!reset_bclk[1]) begin
   mute_bclk<=2'b11;channel_bclk<=0;adc_hold_bclk<=0;
   adc_req_bclk<=0;adc_ack_sync_bclk<=0;adc_overrun_bclk<=0;
 end else begin
   mute_bclk<={mute_bclk[0],SW[3]};
   channel_bclk<={channel_bclk[0],SW[4]};
   adc_ack_sync_bclk<={adc_ack_sync_bclk[1:0],adc_ack_50};
   if(audio_sample_valid) begin
     if(adc_req_bclk==adc_ack_sync_bclk[2]) begin
       adc_hold_bclk<=selected_adc;
       adc_req_bclk<=~adc_req_bclk;
     end else adc_overrun_bclk<=1;
   end
 end end
 reg [2:0] adc_req_sync_50;
 reg adc_sample50;
 reg signed [15:0] mic50;
 reg [6:0] sw_q1,sw_q2;
 reg [2:0] key_sync;
 always @(posedge CLOCK_50 or negedge reset50[1]) begin
 if(!reset50[1]) begin
   adc_req_sync_50<=0;adc_ack_50<=0;adc_sample50<=0;mic50<=0;
   sw_q1<=0;sw_q2<=0;key_sync<=3'b111;
 end else begin
   sw_q1<=SW;sw_q2<=sw_q1;key_sync<={key_sync[1:0],KEY[1]};
   adc_req_sync_50<={adc_req_sync_50[1:0],adc_req_bclk};
   adc_sample50<=0;
   if(adc_req_sync_50[2]!=adc_ack_50) begin
      mic50<=adc_hold_bclk;
      adc_sample50<=1;
      adc_ack_50<=adc_req_sync_50[2];
   end
 end end
 wire dc_valid;
 wire signed [15:0] dc_audio;
 dc_blocker dc(.clk(CLOCK_50),.rst_n(reset50[1]),.enable(sw_q2[5]),
 .in_valid(adc_sample50),.x_in(mic50),.out_valid(dc_valid),.y_out(dc_audio));
 wire dsp_input_valid;
 wire signed [15:0] dsp_input;
 wire decim_busy,decim_overrun;
 decimator8_fir127 decim(.clk(CLOCK_50),.rst_n(reset50[1]),
 .sample_valid(dc_valid),.sample_in(dc_audio),
 .out_valid(dsp_input_valid),.sample_out(dsp_input),
 .busy(decim_busy),.overrun(decim_overrun));
 wire signed [15:0] prbs;
 lfsr_prbs31 noise(.clk(CLOCK_50),.reset_n(reset50[1]),
 .enable(dsp_input_valid),.noise_out(prbs));
 wire signed [15:0] core_out;
 wire core_done,calib_busy,calib_valid,calib_failed,anc_active;
 wire core_overrun,core_fault;
 wire [15:0] last_cycles;
 wire signed [31:0] debug_xf;
 wire signed [39:0] debug_w;
 wire [7:0] debug_state;
 wire sticky_adc_overrun;
 reg [1:0] adc_error_sync;
 always @(posedge CLOCK_50 or negedge reset50[1])
 if(!reset50[1])adc_error_sync<=0;
 else adc_error_sync<={adc_error_sync[0],adc_overrun_bclk};
 assign sticky_adc_overrun=adc_error_sync[1];
 wire forced_mute=sw_q2[3] | sw_q2[6] | core_fault;
 feedback_fxlms_bram #(.M_TAPS(128),.L_TAPS(64),
 .D_BULK(12),.DELTA(0),.CALIB_SAMPLES(36000),.VALID_SAMPLES(2048)) anc(
 .clk(CLOCK_50),.rst_n(reset50[1]),
 .calib_start(key_sync[2:1]==2'b10),
 .anc_enable(sw_q2[0]),.emergency_mute(forced_mute),
 .mu_sel(sw_q2[2:1]),.sample_valid(dsp_input_valid),
 .mic_in(dsp_input),.prbs_sample(prbs),.spk_out(core_out),
 .sample_done(core_done),.calib_busy(calib_busy),.calib_valid(calib_valid),
 .calib_failed(calib_failed),.anc_active(anc_active),
 .dsp_overrun(core_overrun),.fault_latch(core_fault),
 .last_cycles(last_cycles),.debug_xf_q30(debug_xf),
 .debug_w_q38(debug_w),.debug_state(debug_state));
 wire interp_valid;
 wire signed [15:0] interp_out;
 interpolator8_linear interp(.clk(CLOCK_50),.rst_n(reset50[1]),
 .sample48_valid(adc_sample50),.target6_valid(core_done),
 .target6(core_out),.out48_valid(interp_valid),.out48(interp_out));

 // Low-level triangular DAC test: period 240 48kHz samples = 200Hz.
 reg [7:0] tone_idx;
 reg signed [15:0] diag_tone;
 always @(posedge CLOCK_50 or negedge reset50[1]) begin
 if(!reset50[1]) begin tone_idx<=0;diag_tone<=0;end
 else if(adc_sample50) begin
   if(tone_idx==239)tone_idx<=0;
   else tone_idx<=tone_idx+1'b1;
   if(tone_idx<60) diag_tone<=($signed({1'b0,tone_idx})*16'sd32)-16'sd960;
   else if(tone_idx<180) diag_tone<=16'sd2880-($signed({1'b0,tone_idx})*16'sd32);
   else diag_tone<=($signed({1'b0,tone_idx})*16'sd32)-16'sd8640;
 end end
 wire kill_audio=sw_q2[3] | core_fault | core_overrun |
                 decim_overrun | sticky_adc_overrun | calib_failed;
 // Prevent stale interpolation samples from reappearing after unmute.
 wire speaker_allowed=sw_q2[6] || calib_busy ||
                     (anc_active && calib_valid && sw_q2[0]);
 wire signed [15:0] dac48_50 = (kill_audio || !speaker_allowed)?16'sd0:
   sw_q2[6] ? diag_tone : interp_out;
 // 50 MHz -> BCLK DAC mailbox with receiver acknowledgement.
 reg signed [15:0] playback_hold50;
 reg playback_req50;
 reg [2:0] playback_ack_sync50;
 reg dac_mailbox_overrun;
 reg playback_ack_bclk;
 reg [2:0] playback_req_sync_bclk;
 always @(posedge CLOCK_50 or negedge reset50[1]) begin
 if(!reset50[1]) begin
   playback_hold50<=0;playback_req50<=0;
   playback_ack_sync50<=0;dac_mailbox_overrun<=0;
 end else begin
   playback_ack_sync50<={playback_ack_sync50[1:0],playback_ack_bclk};
   if(interp_valid) begin
     if(playback_req50==playback_ack_sync50[2]) begin
       playback_hold50<=dac48_50;playback_req50<=~playback_req50;
     end else dac_mailbox_overrun<=1;
   end
 end end
 reg [1:0] bclk_fault_sync;
 wire output_fault50=kill_audio|dac_mailbox_overrun|!speaker_allowed;
 always @(posedge AUD_BCLK or negedge reset_bclk[1]) begin
 if(!reset_bclk[1]) begin
   playback_ack_bclk<=0;playback_req_sync_bclk<=0;
   playback_bclk<=0;bclk_fault_sync<=2'b11;
 end else begin
   playback_req_sync_bclk<={playback_req_sync_bclk[1:0],playback_req50};
   bclk_fault_sync<={bclk_fault_sync[0],output_fault50};
   if(mute_bclk[1]||bclk_fault_sync[1])playback_bclk<=0;
   else if(playback_req_sync_bclk[2]!=playback_ack_bclk)
     playback_bclk<=playback_hold50;
   if(playback_req_sync_bclk[2]!=playback_ack_bclk)
     playback_ack_bclk<=playback_req_sync_bclk[2];
 end end

 // 100 ms LED event latch and peak meter (diagnostics, not an SPL measurement).
 reg [22:0] window_count;
 reg [15:0] peak;
 reg [15:0] vumeter;
 reg adc_seen,dsp_seen,adc_led,dsp_led;
 reg [15:0] abs_mic;
 function [15:0] threshold(input integer i);
 begin case(i)
 0:threshold=16'd16;1:threshold=16'd24;2:threshold=16'd36;3:threshold=16'd52;
 4:threshold=16'd80;5:threshold=16'd112;6:threshold=16'd160;7:threshold=16'd240;
 8:threshold=16'd360;9:threshold=16'd550;10:threshold=16'd820;11:threshold=16'd1250;
 12:threshold=16'd1900;13:threshold=16'd2900;14:threshold=16'd4400;default:threshold=16'd6700;
 endcase end endfunction
 integer i;
 always @(posedge CLOCK_50 or negedge reset50[1]) begin
 if(!reset50[1]) begin
   window_count<=0;peak<=0;vumeter<=0;adc_seen<=0;dsp_seen<=0;
   adc_led<=0;dsp_led<=0;abs_mic<=0;
 end else begin
   if(adc_sample50) begin
     adc_seen<=1;
     abs_mic<=mic50[15]?((mic50==-16'sd32768)?16'd32767:-mic50):mic50;
     if(abs_mic>peak)peak<=abs_mic;
   end
   if(dsp_input_valid)dsp_seen<=1;
   if(window_count==23'd4999999) begin
     window_count<=0;adc_led<=adc_seen;dsp_led<=dsp_seen;
     adc_seen<=0;dsp_seen<=0;
     for(i=0;i<16;i=i+1) vumeter[i]<=(peak>threshold(i));
     peak<=0;
   end else window_count<=window_count+1'b1;
 end end
 assign LEDG[0]=config_ready;
 assign LEDG[1]=adc_led;
 assign LEDG[2]=dsp_led;
 assign LEDG[3]=calib_busy;
 assign LEDG[4]=calib_valid;
 assign LEDG[5]=anc_active;
 assign LEDG[6]=core_overrun|decim_overrun|sticky_adc_overrun|dac_mailbox_overrun;
 assign LEDG[7]=core_fault|calib_failed|config_error;
 assign LEDG[8]=SW[3]|kill_audio;
 assign LEDR={calib_failed,(last_cycles>=16'd7000),vumeter};
endmodule
