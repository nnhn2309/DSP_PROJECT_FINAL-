`timescale 1ns/1ps
// A first-order hold for initial hardware bring-up (NOT an ideal 8x FIR).
// Updated from 6-kHz target, emits one signed output per 48-kHz tick.
// Reaches target after 8 input ticks; introduces time-varying phase delay.
module interpolator8_linear(input wire clk,rst_n,
 input wire sample48_valid,input wire target6_valid,
 input wire signed [15:0] target6,
 output reg out48_valid,output reg signed [15:0] out48);
 reg signed [15:0] target;
 reg signed [31:0] current_q3,step_q3;
 reg [3:0] remaining;
 function signed [15:0] sat16(input signed [31:0] a);
 begin if(a>32767) sat16=32767;
 else if(a< -32768) sat16=-32768;
 else sat16=a[15:0];end endfunction
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin target<=0;current_q3<=0;step_q3<=0;remaining<=0;out48<=0;out48_valid<=0;end
 else begin
 out48_valid<=0;
 if(target6_valid) begin
   target<=target6;
   step_q3<=($signed(target6)*32'sd8-current_q3) >>> 3;
   remaining<=8;
 end
 if(sample48_valid) begin
   out48_valid<=1;
   if(remaining!=0) begin
     current_q3<=current_q3+step_q3;
     out48<=sat16((current_q3+step_q3) >>> 3);
     remaining<=remaining-1'b1;
   end else out48<=sat16(current_q3 >>> 3);
 end
 end end
endmodule
