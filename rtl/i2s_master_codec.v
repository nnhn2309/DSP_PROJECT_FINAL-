`timescale 1ns/1ps
// WM8731 is the clock MASTER. BCLK/LRCLK are FPGA INPUTS.
// Supports arbitrary >=17 BCLK slot lengths, including USB 125-bit slots.
// LR changes on falling BCLK; boundary rising edge is I2S delay bit.
module i2s_master_codec(
 input wire bclk, rst_n, adc_lr, dac_lr, adc_data,
 input wire signed [15:0] playback,
 output reg dac_data,
 output reg signed [15:0] left, right,
 output reg sample_valid);
 reg adc_prev, dac_prev;
 reg [4:0] rx_count, tx_count;
 reg [15:0] rx_shift, tx_word, stereo_hold;
 reg left_seen;
 reg next_dac;
 always @(posedge bclk or negedge rst_n) begin
 if(!rst_n) begin
 adc_prev<=1; dac_prev<=1; rx_count<=16; tx_count<=16;
 rx_shift<=0; tx_word<=0; stereo_hold<=0; next_dac<=0;
 left<=0; right<=0; sample_valid<=0; left_seen<=0;
 end else begin
 sample_valid<=0; adc_prev<=adc_lr; dac_prev<=dac_lr;
 if(adc_lr != adc_prev) begin rx_count<=0; rx_shift<=0; end
 else if(rx_count<16) begin
 rx_shift<={rx_shift[14:0],adc_data}; rx_count<=rx_count+1'b1;
 if(rx_count==15) begin
 if(!adc_lr) begin left<={rx_shift[14:0],adc_data}; left_seen<=1; end
 else begin
 right<={rx_shift[14:0],adc_data};
 if(left_seen) sample_valid<=1;
 left_seen<=0;
 end
 end
 end
 // Prepare data at rising edge, launch on the following falling edge.
 // On LR boundary, the following falling edge must launch the MSB.
 if(dac_lr != dac_prev) begin
 tx_count<=0;
 if(!dac_lr) begin
 stereo_hold<=playback; tx_word<=playback; next_dac<=playback[15];
 end else begin tx_word<=stereo_hold; next_dac<=stereo_hold[15]; end
 end else if(tx_count<15) begin
 tx_count<=tx_count+1'b1; next_dac<=tx_word[14-tx_count];
 end else begin tx_count<=16; next_dac<=0; end
 end
 end
 always @(negedge bclk or negedge rst_n)
 if(!rst_n) dac_data<=0; else dac_data<=next_dac;
endmodule
