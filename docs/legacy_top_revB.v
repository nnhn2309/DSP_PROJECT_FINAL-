`timescale 1ns/1ps
// DE2 EP2C35 + WM8731. MIC IN (error) -> Feedback FxLMS -> LINE OUT (active speaker).
// CLOCK_50 50MHz / PLL AUD_XCK 12MHz / codec-master BCLK and LRCK ~48kHz.
// KEY0 reset, KEY1 start identification. SW0 ANC, SW2:1 step,
// SW3 *FORCE SPEAKER MUTE* (1=muted), SW4 ADC channel L/R.
module de2_anc_top #(
  parameter integer MIC_BOOST=0
)(
  input wire CLOCK_50, input wire [1:0] KEY, input wire [4:0] SW,
  output wire [8:0] LEDG, output wire [17:0] LEDR,
  input wire AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, AUD_ADCDAT,
  output wire AUD_XCK, AUD_DACDAT, I2C_SCLK, inout wire I2C_SDAT
);
  wire pll_locked, config_ready, config_error;
  audio_pll pll(.clk50(CLOCK_50),.reset(~KEY[0]),.mclk(AUD_XCK),.locked(pll_locked));
  wire global_rst_n=KEY[0] & pll_locked;
  reg [1:0] reset50;
  always @(posedge CLOCK_50 or negedge global_rst_n)
    if(!global_rst_n) reset50<=0;
    else reset50<={reset50[0],1'b1};
  wm8731_config #(.USE_LINE_REF(0),.MIC_BOOST(MIC_BOOST)) cfg (
    .clk(CLOCK_50),.rst_n(reset50[1]),.scl(I2C_SCLK),.sda(I2C_SDAT),
    .ready(config_ready),.error(config_error));
  wire audio_rst_n=global_rst_n & config_ready;
  reg [1:0] reset_bclk;
  always @(posedge AUD_BCLK or negedge audio_rst_n)
    if(!audio_rst_n) reset_bclk<=0;
    else reset_bclk<={reset_bclk[0],1'b1};
  wire signed [15:0] adc_left,adc_right;
  wire sample_valid;
  reg signed [15:0] playback_bclk;
  i2s_master_codec codec(
    .bclk(AUD_BCLK),.rst_n(reset_bclk[1]),
    .adc_lr(AUD_ADCLRCK),.dac_lr(AUD_DACLRCK),.adc_data(AUD_ADCDAT),
    .playback(playback_bclk),.dac_data(AUD_DACDAT),
    .left(adc_left),.right(adc_right),.sample_valid(sample_valid));

  // SAMPLE BCLK -> CLOCK_50: 2-way toggle mailbox, with payload held stable.
  reg signed [15:0] mic_hold;
  reg req_toggle;
  reg [2:0] ack_sync;
  reg overrun_bclk;
  reg [1:0] channel_sync,mute_sync;
  reg ack_toggle;
  wire signed [15:0] selected_mic = channel_sync[1] ? adc_right : adc_left;
  always @(posedge AUD_BCLK or negedge reset_bclk[1]) begin
    if(!reset_bclk[1]) begin
      mic_hold<=0;req_toggle<=0;ack_sync<=0;overrun_bclk<=0;
      channel_sync<=0;mute_sync<=2'b11;
    end else begin
      ack_sync<={ack_sync[1:0],ack_toggle};
      channel_sync<={channel_sync[0],SW[4]};
      mute_sync<={mute_sync[0],SW[3]};
      if(sample_valid) begin
        if(req_toggle==ack_sync[2]) begin
          mic_hold<=selected_mic;req_toggle<=~req_toggle;
        end else overrun_bclk<=1;
      end
    end
  end
  reg [2:0] req_sync;
  reg sample50;
  reg signed [15:0] mic50;
  reg [4:0] sw_sync1,sw_sync2;
  reg [2:0] key_sync;
  reg signed [15:0] out_hold50;
  reg out_toggle;
  wire signed [15:0] speaker50;
  wire calib_busy,calib_ready,calib_failed,anc_active,sample_done,dsp_overrun;
  always @(posedge CLOCK_50 or negedge reset50[1]) begin
    if(!reset50[1]) begin
      req_sync<=0;ack_toggle<=0;sample50<=0;mic50<=0;
      sw_sync1<=0;sw_sync2<=0;key_sync<=3'b111;
      out_hold50<=0;out_toggle<=0;
    end else begin
      req_sync<={req_sync[1:0],req_toggle};
      sw_sync1<=SW;sw_sync2<=sw_sync1;
      key_sync<={key_sync[1:0],KEY[1]};
      sample50<=0;
      if(req_sync[2]!=ack_toggle) begin
        mic50<=mic_hold;sample50<=1;ack_toggle<=req_sync[2];
      end
      if(sample_done) begin
        out_hold50<=sw_sync2[3] ? 16'sd0 : speaker50;
        out_toggle<=~out_toggle;
      end
    end
  end
  // OUTPUT CLOCK_50 -> BCLK: stable payload and toggle event.
  reg [2:0] out_sync;
  always @(posedge AUD_BCLK or negedge reset_bclk[1]) begin
    if(!reset_bclk[1]) begin out_sync<=0;playback_bclk<=0;end
    else begin
      out_sync<={out_sync[1:0],out_toggle};
      if(mute_sync[1]) playback_bclk<=0; // stop output even if no DSP done
      else if(out_sync[2]!=out_sync[1]) playback_bclk<=out_hold50;
    end
  end
  wire signed [15:0] prbs;
  lfsr_prbs31 prng(.clk(CLOCK_50),.reset_n(reset50[1]),
                   .enable(sample50),.noise_out(prbs));
  feedback_fxlms_core #(.M_TAPS(128),.L_TAPS(64),
    .CALIB_SAMPLES(96000),.CALIB_TIMEOUT_CYCLES(200000000)) anc(
    .clk(CLOCK_50),.reset_n(reset50[1]),
    .btn_calib_start(key_sync[2:1]==2'b10),
    .sw_anc_enable(sw_sync2[0]),.sw_mu_sel(sw_sync2[2:1]),
    .output_mute(sw_sync2[3]),
    .sample_valid(sample50),.mic_in(mic50),.spk_out(speaker50),
    .white_noise(prbs),.calib_busy(calib_busy),.calib_ready(calib_ready),
    .calib_failed(calib_failed),.sample_done(sample_done),
    .dsp_overrun(dsp_overrun),.anc_active(anc_active));

  // Led diagnostics (100ms moving window, no numerical output claim).
  reg [22:0] window_cnt;
  reg [15:0] peak_hold;
  reg [15:0] led_levels;
  reg adc_seen,dsp_seen,output_seen,clipped_seen;
  reg adc_led,dsp_led,output_led,clip_led;
  reg [1:0] overrun_sync;
  wire [15:0] abs_mic = (mic50==16'sh8000) ? 16'h7fff :
                         (mic50[15] ? -mic50 : mic50);
  function [15:0] threshold;
    input integer idx;
    begin
      case(idx)
       0:threshold=16'd16; 1:threshold=16'd24;
       2:threshold=16'd36; 3:threshold=16'd52;
       4:threshold=16'd80; 5:threshold=16'd112;
       6:threshold=16'd160;7:threshold=16'd240;
       8:threshold=16'd360;9:threshold=16'd550;
       10:threshold=16'd820;11:threshold=16'd1250;
       12:threshold=16'd1900;13:threshold=16'd2900;
       14:threshold=16'd4400;default:threshold=16'd6700;
      endcase
    end
  endfunction
  integer j;
  always @(posedge CLOCK_50 or negedge reset50[1]) begin
    if(!reset50[1]) begin
      window_cnt<=0;peak_hold<=0;led_levels<=0;
      adc_seen<=0;dsp_seen<=0;output_seen<=0;clipped_seen<=0;
      adc_led<=0;dsp_led<=0;output_led<=0;clip_led<=0;
      overrun_sync<=0;
    end else begin
      overrun_sync<={overrun_sync[0],overrun_bclk};
      if(sample50) begin
        adc_seen<=1;
        if(abs_mic>peak_hold) peak_hold<=abs_mic;
        if(abs_mic>16'd32000) clipped_seen<=1;
      end
      if(sample_done) begin
        dsp_seen<=1;
        if(speaker50!=0 && !sw_sync2[3]) output_seen<=1;
      end
      if(window_cnt==23'd4_999_999) begin
        window_cnt<=0;
        for(j=0;j<16;j=j+1) led_levels[j]<=(peak_hold>threshold(j));
        adc_led<=adc_seen;dsp_led<=dsp_seen;
        output_led<=output_seen;clip_led<=clipped_seen;
        peak_hold<=0;adc_seen<=0;dsp_seen<=0;
        output_seen<=0;clipped_seen<=0;
      end else window_cnt<=window_cnt+1'b1;
    end
  end
  assign LEDG={output_led,dsp_led,adc_led,config_error,
               (overrun_sync[1]|dsp_overrun),anc_active,
               calib_ready,calib_busy,config_ready};
  assign LEDR={clip_led,calib_failed,led_levels};
endmodule
