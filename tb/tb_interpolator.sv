`timescale 1ns/1ps
module tb_interpolator;
 reg clk=0,rst_n=0,sample48_valid=0,target6_valid=0;
 reg signed [15:0] target6=0;
 wire out48_valid;
 wire signed [15:0] out48;
 integer i;
 always #10 clk=~clk;
 interpolator8_linear dut(.clk(clk),.rst_n(rst_n),
 .sample48_valid(sample48_valid),.target6_valid(target6_valid),
 .target6(target6),.out48_valid(out48_valid),.out48(out48));
 task tick48;
 begin sample48_valid=1;@(negedge clk);sample48_valid=0;
 repeat(2)@(negedge clk);end endtask
 initial begin
 repeat(5)@(negedge clk);rst_n=1;
 target6=8000;target6_valid=1;@(negedge clk);target6_valid=0;
 repeat(2)@(negedge clk);
 for(i=0;i<8;i=i+1)tick48();
 if(out48<7999||out48>8001)$fatal(1,"FOH failed +8000: %0d",out48);
 target6=-8000;target6_valid=1;@(negedge clk);target6_valid=0;
 repeat(2)@(negedge clk);
 for(i=0;i<8;i=i+1)tick48();
 if(out48> -7999||out48< -8001)$fatal(1,"FOH failed -8000: %0d",out48);
 $display("PASS tb_interpolator: two 8-frame ramps");$finish;
 end
endmodule
