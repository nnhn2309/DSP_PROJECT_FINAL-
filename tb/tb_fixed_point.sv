`timescale 1ns/1ps
// Self-check arithmetic examples for RTL developer use.
module tb_fixed_point;
 reg signed [23:0] coef;
 reg signed [15:0] sample;
 reg signed [32:0] prod_lo;
 reg signed [31:0] prod_hi;
 reg signed [63:0] combined,reference;
 integer i;
 reg signed [15:0] err16;
 reg signed [31:0] xf_q30;
 reg signed [63:0] gradprod,step;
 reg signed [39:0] w_q38;
 reg signed [31:0] old_step;
 initial begin
  for(i=0;i<20000;i=i+1)begin
    coef=$random;
    sample=$random;
    prod_lo=$signed({1'b0,coef[15:0]})*$signed(sample);
    prod_hi=$signed(coef[23:16])*$signed(sample);
    combined=$signed(prod_lo)+($signed(prod_hi) <<< 16);
    reference=$signed(coef)*$signed(sample);
    if(combined!==reference) $fatal(1,"mul signed mismatch index=%0d",i);
  end
  err16=16'sd100;
  xf_q30=32'sd655360; // 20 LSB of Q1.15, now represented in Q2.30
  gradprod=$signed(err16)*$signed(xf_q30);
  step=gradprod >>> (7+11);
  old_step=(32'sd100*32'sd20) >>> 11;
  w_q38=0;
  for(i=0;i<300;i=i+1)w_q38=w_q38+step;
  if(old_step!==0 || step!==250 || (w_q38 >>> 16)<1)
    $fatal(1,"Q2.38 fractional accumulation failed");
  $display("PASS tb_fixed_point: 20000 signed multiplications and underflow residual");
  $finish;
 end
endmodule
