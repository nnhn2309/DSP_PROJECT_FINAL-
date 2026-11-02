`timescale 1ns/1ps
module tb_dc_blocker;
 reg clk=0,rst_n=0,enable=1,in_valid=0;
 reg signed [15:0] x_in=0;
 wire out_valid;
 wire signed [15:0] y_out;
 integer n;
 always #10 clk=~clk;
 dc_blocker dut(.clk(clk),.rst_n(rst_n),.enable(enable),
 .in_valid(in_valid),.x_in(x_in),.out_valid(out_valid),.y_out(y_out));
 initial begin
   repeat(4) @(negedge clk);rst_n=1;
   for(n=0;n<3000;n=n+1)begin
     x_in=3000;in_valid=1;
     @(negedge clk);in_valid=0;
     repeat(2) @(negedge clk);
   end
   if(y_out>50||y_out< -50)$fatal(1,"DC not removed: %0d",$signed(y_out));
   $display("PASS tb_dc_blocker: residue=%0d",$signed(y_out));
   $finish;
 end
endmodule
