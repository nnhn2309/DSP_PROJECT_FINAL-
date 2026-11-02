`timescale 1ns/1ps
module tb_decimator8;
 reg clk=0,rst_n=0,valid=0;
 reg signed [15:0] sample=0;
 wire out_valid,busy,overrun;
 wire signed [15:0] out;
 integer count=0,good=0;
 always #10 clk=~clk;
 decimator8_fir127 dut(.clk(clk),.rst_n(rst_n),.sample_valid(valid),
  .sample_in(sample),.out_valid(out_valid),.sample_out(out),.busy(busy),.overrun(overrun));
 always @(posedge clk) if(out_valid) begin
   count=count+1;
   if($signed(out)>1990 && $signed(out)<2010)good=good+1;
   else begin $display("FAIL FIR constant steady output %0d",$signed(out));$fatal(1);end
 end
 initial begin
  repeat(5) @(negedge clk);rst_n=1;
  repeat(4) @(negedge clk);
  for(integer n=0;n<210;n=n+1)begin
    valid=1;sample=16'sd2000;
    @(negedge clk); valid=0;
    repeat(950) @(negedge clk);
  end
  repeat(600) @(negedge clk);
  if(overrun) $fatal(1,"decimator missed 48k deadline");
  if(good<8) $fatal(1,"too few steady output samples %0d",good);
  $display("PASS tb_decimator8: %0d steady outputs",good);
  $finish;
 end
endmodule
