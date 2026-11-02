`timescale 1ns/1ps
// 50 MHz, I2C ~100 kHz. Open-drain SDA, push-pull SCL (WM8731 no stretching).
// NACK: send STOP and latch error; KEY0 resets/retries entire sequence.
module wm8731_config #(parameter USE_LINE_REF=0, parameter MIC_BOOST=0,
 parameter TICK_DIV=125, START_WAIT=1000000, SETTLE_WAIT=5000000)(
 input wire clk, rst_n,
 output reg scl, inout wire sda,
 output reg ready, error);
 reg drive_low;
 assign sda=drive_low ? 1'b0 : 1'bz;
 (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
 reg [1:0] sda_sync;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) sda_sync<=2'b11; else sda_sync<={sda_sync[0],sda};
 reg [31:0] timer;
 reg [15:0] divider;
 reg [4:0] state;
 reg [3:0] index;
 reg [1:0] byte_no;
 reg [2:0] bit_no;
 reg [23:0] packet;
 reg nack;
 localparam WAIT_START=0, START_A=1, START_B=2, BIT_LOW=3,
 BIT_HIGH=4, BIT_HOLD=5, BIT_END=6, ACK_LOW=7, ACK_HIGH=8,
 ACK_SAMPLE=9, ACK_END=10, STOP_LOW=11, STOP_HIGH=12,
 STOP_RELEASE=13, NEXT=14, SETTLE=15, DONE=16, FAILED=17;
 function [15:0] entry;
 input [3:0] n;
 begin case(n)
 0: entry={7'd15,9'h000}; // reset
 1: entry={7'd9, 9'h000}; // inactive
 2: entry={7'd6, (USE_LINE_REF ? 9'h072 : 9'h071)}; // output off
 3: entry={7'd0, 9'h017}; // line L 0 dB, unmuted
 4: entry={7'd1, 9'h017}; // line R 0 dB
 5: entry={7'd2, 9'h060}; // headphone -25 dB
 6: entry={7'd3, 9'h060};
 7: entry={7'd4, (USE_LINE_REF ? 9'h012 : (MIC_BOOST ? 9'h015 : 9'h014))}; // DAC, no bypass/sidetone
 8: entry={7'd5, 9'h008}; // DAC muted, ADC HPF enabled
 9: entry={7'd7, 9'h042}; // MASTER, I2S, 16-bit
 10:entry={7'd8, 9'h001}; // USB, 12 MHz, ADC/DAC 48 kHz
 11:entry={7'd9, 9'h001}; // active
 12:entry={7'd6, (USE_LINE_REF ? 9'h062 : 9'h061)}; // output enabled
 13:entry={7'd5, 9'h000}; // unmute after settling
 default:entry=0;
 endcase end endfunction
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin
 scl<=1; drive_low<=0; ready<=0; error<=0; timer<=0; divider<=0;
 state<=WAIT_START; index<=0; byte_no<=0; bit_no<=7; packet<=0; nack<=0;
 end else if(state==WAIT_START || state==SETTLE) begin
 if(timer == ((state==WAIT_START)?START_WAIT-1:SETTLE_WAIT-1)) begin
 timer<=0; packet<={8'h34,entry(index)}; nack<=0;
 byte_no<=0; bit_no<=7; divider<=0; state<=START_A;
 end else timer<=timer+1'b1;
 end else if(divider==TICK_DIV-1) begin
 divider<=0;
 case(state)
 START_A: begin scl<=1; drive_low<=0; state<=START_B; end
 START_B: begin drive_low<=1; state<=BIT_LOW; end
 BIT_LOW: begin scl<=0; drive_low<=!packet[23]; state<=BIT_HIGH; end
 BIT_HIGH: begin scl<=1; state<=BIT_HOLD; end
 BIT_HOLD: state<=BIT_END;
 BIT_END: begin
 scl<=0; packet<={packet[22:0],1'b0};
 if(bit_no==0) state<=ACK_LOW;
 else begin bit_no<=bit_no-1'b1; state<=BIT_LOW; end
 end
 ACK_LOW: begin drive_low<=0; state<=ACK_HIGH; end
 ACK_HIGH: begin scl<=1; state<=ACK_SAMPLE; end
 ACK_SAMPLE: begin if(sda_sync[1]!==1'b0) nack<=1; state<=ACK_END; end
 ACK_END: begin
 scl<=0;
 if(nack || byte_no==2) state<=STOP_LOW;
 else begin byte_no<=byte_no+1'b1; bit_no<=7; state<=BIT_LOW; end
 end
 STOP_LOW: begin drive_low<=1; scl<=0; state<=STOP_HIGH; end
 STOP_HIGH: begin scl<=1; state<=STOP_RELEASE; end
 STOP_RELEASE: begin drive_low<=0; state<=NEXT; end
 NEXT: begin
 if(nack) begin error<=1; state<=FAILED; end
 else if(index==13) begin ready<=1; state<=DONE; end
 else begin
 index<=index+1'b1;
 if(index==12) begin timer<=0; state<=SETTLE; end
 else begin packet<={8'h34,entry(index+1'b1)}; byte_no<=0; bit_no<=7; state<=START_A; end
 end
 end
 DONE: begin scl<=1; drive_low<=0; end
 FAILED: begin scl<=1; drive_low<=0; end
 default: state<=WAIT_START;
 endcase
 end else divider<=divider+1'b1;
 end
endmodule
