`timescale 1ns/1ps
module audio_pll(input wire clk50, reset, output wire mclk, locked);
`ifdef SIMULATION
 // Behavioral clock only: does NOT verify PLL lock or physical implementation.
 reg c=0; reg l=0;
 always #41.666667 c=~c;
 always @(posedge clk50 or posedge reset)
 if(reset) l<=0; else l<=1;
 assign mclk=c;
 assign locked=l;
`else
 wire [5:0] clocks;
 altpll pll_inst(.inclk({1'b0,clk50}),.areset(reset),.clk(clocks),
 .clkswitch(1'b0),.configupdate(1'b0),.fbin(1'b1),
 .phasecounterselect(4'b1111),.phasestep(1'b0),.phaseupdown(1'b0),
 .scanaclr(1'b0),.scanclk(1'b0),.scanclkena(1'b1),
 .scandata(1'b0),.scanread(1'b0),.scanwrite(1'b0),
 .locked(locked),.clkena(6'b111111),.extclkena(4'b1111),.pllena(1'b1),.pfdena(1'b1));
 defparam pll_inst.inclk0_input_frequency=20000,
 pll_inst.clk0_multiply_by=6, pll_inst.clk0_divide_by=25,
 pll_inst.clk0_duty_cycle=50, pll_inst.clk0_phase_shift="0",
 pll_inst.compensate_clock="CLK0", pll_inst.operation_mode="NORMAL",
 pll_inst.intended_device_family="Cyclone II", pll_inst.lpm_type="altpll",
 pll_inst.width_clock=6;
 assign mclk=clocks[0];
`endif
endmodule
