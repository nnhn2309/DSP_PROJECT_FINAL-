`timescale 1ns/1ps
// y[n] = x[n]-x[n-1]+(32700/32768)y[n-1]
// One 48-kHz sample per input_valid (CLOCK_50 domain).
module dc_blocker(input wire clk, rst_n, input wire enable,
 input wire in_valid, input wire signed [15:0] x_in,
 output reg out_valid, output reg signed [15:0] y_out);
 reg signed [15:0] x_prev;
 reg signed [31:0] y_prev;
 reg signed [47:0] tmp;
 function signed [15:0] sat16(input signed [47:0] a);
 begin if(a>32767) sat16=32767;
 else if(a< -32768) sat16=-32768;
 else sat16=a[15:0]; end endfunction
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin x_prev<=0; y_prev<=0; y_out<=0; out_valid<=0; tmp<=0; end
 else begin
   out_valid<=in_valid;
   if(in_valid) begin
     // internal Q1.15 input but 32-bit intermediate; parameterized coefficient Q0.15
     tmp=$signed(x_in)-$signed(x_prev)+(($signed(y_prev)*$signed(32'sd32700)) >>> 15);
     if(enable) begin
       y_out<=sat16(tmp);
       y_prev<=sat16(tmp);
     end else begin y_out<=x_in; y_prev<=0; end
     x_prev<=x_in;
   end
 end end
endmodule
