`timescale 1ns/1ps
// 127-tap Kaiser anti-alias FIR at 48 kHz; decimate-by-8 to 6 kHz.
// CLOCK_50, asynchronous input events already synchronized by mailbox.
// History RAM has synchronous read (M4K inference). No array-wide reset.
module decimator8_fir127(input wire clk,rst_n,
 input wire sample_valid,input wire signed [15:0] sample_in,
 output reg out_valid,output reg signed [15:0] sample_out,
 output reg busy,output reg overrun);
 (* ramstyle = "M4K" *) reg signed [15:0] hist [0:127];
 reg [6:0] wr_ptr,rd_addr,base_ptr;
 reg signed [15:0] rd_q;
 reg [6:0] tap;
 reg [2:0] mod8;
 reg signed [63:0] acc;
 reg signed [31:0] product;
 reg [2:0] state;
 localparam IDLE=0, REQUEST=1, WAIT_READ=2, MULTIPLY=3, ACCUM=4, COMMIT=5;
 reg [7:0] warmup;
 function signed [15:0] coeff(input [6:0] idx);
 begin case(idx)
      7'd0: coeff = 16'sd0;
      7'd1: coeff = -16'sd1;
      7'd2: coeff = -16'sd1;
      7'd3: coeff = -16'sd1;
      7'd4: coeff = -16'sd1;
      7'd5: coeff = -16'sd2;
      7'd6: coeff = -16'sd2;
      7'd7: coeff = -16'sd2;
      7'd8: coeff = -16'sd1;
      7'd9: coeff = -16'sd1;
      7'd10: coeff = 16'sd0;
      7'd11: coeff = 16'sd2;
      7'd12: coeff = 16'sd4;
      7'd13: coeff = 16'sd7;
      7'd14: coeff = 16'sd10;
      7'd15: coeff = 16'sd13;
      7'd16: coeff = 16'sd17;
      7'd17: coeff = 16'sd21;
      7'd18: coeff = 16'sd25;
      7'd19: coeff = 16'sd29;
      7'd20: coeff = 16'sd32;
      7'd21: coeff = 16'sd34;
      7'd22: coeff = 16'sd35;
      7'd23: coeff = 16'sd34;
      7'd24: coeff = 16'sd31;
      7'd25: coeff = 16'sd26;
      7'd26: coeff = 16'sd17;
      7'd27: coeff = 16'sd6;
      7'd28: coeff = -16'sd8;
      7'd29: coeff = -16'sd26;
      7'd30: coeff = -16'sd46;
      7'd31: coeff = -16'sd68;
      7'd32: coeff = -16'sd93;
      7'd33: coeff = -16'sd118;
      7'd34: coeff = -16'sd143;
      7'd35: coeff = -16'sd167;
      7'd36: coeff = -16'sd188;
      7'd37: coeff = -16'sd205;
      7'd38: coeff = -16'sd216;
      7'd39: coeff = -16'sd221;
      7'd40: coeff = -16'sd216;
      7'd41: coeff = -16'sd201;
      7'd42: coeff = -16'sd175;
      7'd43: coeff = -16'sd136;
      7'd44: coeff = -16'sd83;
      7'd45: coeff = -16'sd17;
      7'd46: coeff = 16'sd64;
      7'd47: coeff = 16'sd158;
      7'd48: coeff = 16'sd264;
      7'd49: coeff = 16'sd383;
      7'd50: coeff = 16'sd511;
      7'd51: coeff = 16'sd646;
      7'd52: coeff = 16'sd787;
      7'd53: coeff = 16'sd930;
      7'd54: coeff = 16'sd1073;
      7'd55: coeff = 16'sd1212;
      7'd56: coeff = 16'sd1344;
      7'd57: coeff = 16'sd1466;
      7'd58: coeff = 16'sd1575;
      7'd59: coeff = 16'sd1668;
      7'd60: coeff = 16'sd1743;
      7'd61: coeff = 16'sd1798;
      7'd62: coeff = 16'sd1832;
      7'd63: coeff = 16'sd1852;
      7'd64: coeff = 16'sd1832;
      7'd65: coeff = 16'sd1798;
      7'd66: coeff = 16'sd1743;
      7'd67: coeff = 16'sd1668;
      7'd68: coeff = 16'sd1575;
      7'd69: coeff = 16'sd1466;
      7'd70: coeff = 16'sd1344;
      7'd71: coeff = 16'sd1212;
      7'd72: coeff = 16'sd1073;
      7'd73: coeff = 16'sd930;
      7'd74: coeff = 16'sd787;
      7'd75: coeff = 16'sd646;
      7'd76: coeff = 16'sd511;
      7'd77: coeff = 16'sd383;
      7'd78: coeff = 16'sd264;
      7'd79: coeff = 16'sd158;
      7'd80: coeff = 16'sd64;
      7'd81: coeff = -16'sd17;
      7'd82: coeff = -16'sd83;
      7'd83: coeff = -16'sd136;
      7'd84: coeff = -16'sd175;
      7'd85: coeff = -16'sd201;
      7'd86: coeff = -16'sd216;
      7'd87: coeff = -16'sd221;
      7'd88: coeff = -16'sd216;
      7'd89: coeff = -16'sd205;
      7'd90: coeff = -16'sd188;
      7'd91: coeff = -16'sd167;
      7'd92: coeff = -16'sd143;
      7'd93: coeff = -16'sd118;
      7'd94: coeff = -16'sd93;
      7'd95: coeff = -16'sd68;
      7'd96: coeff = -16'sd46;
      7'd97: coeff = -16'sd26;
      7'd98: coeff = -16'sd8;
      7'd99: coeff = 16'sd6;
      7'd100: coeff = 16'sd17;
      7'd101: coeff = 16'sd26;
      7'd102: coeff = 16'sd31;
      7'd103: coeff = 16'sd34;
      7'd104: coeff = 16'sd35;
      7'd105: coeff = 16'sd34;
      7'd106: coeff = 16'sd32;
      7'd107: coeff = 16'sd29;
      7'd108: coeff = 16'sd25;
      7'd109: coeff = 16'sd21;
      7'd110: coeff = 16'sd17;
      7'd111: coeff = 16'sd13;
      7'd112: coeff = 16'sd10;
      7'd113: coeff = 16'sd7;
      7'd114: coeff = 16'sd4;
      7'd115: coeff = 16'sd2;
      7'd116: coeff = 16'sd0;
      7'd117: coeff = -16'sd1;
      7'd118: coeff = -16'sd1;
      7'd119: coeff = -16'sd2;
      7'd120: coeff = -16'sd2;
      7'd121: coeff = -16'sd2;
      7'd122: coeff = -16'sd1;
      7'd123: coeff = -16'sd1;
      7'd124: coeff = -16'sd1;
      7'd125: coeff = -16'sd1;
      7'd126: coeff = 16'sd0;
      default: coeff=0;
 endcase end endfunction
 function signed [15:0] sat16(input signed [63:0] a);
 begin if(a>32767) sat16=32767;
 else if(a< -32768) sat16=-32768;
 else sat16=a[15:0]; end endfunction
 // synchronous inferable 1-read/1-write memory
 always @(posedge clk) begin
    rd_q<=hist[rd_addr];
    if(sample_valid) hist[wr_ptr]<=sample_in;
 end
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin
 wr_ptr<=0;rd_addr<=0;base_ptr<=0;tap<=0;mod8<=0;
 acc<=0;product<=0;state<=IDLE;busy<=0;out_valid<=0;
 sample_out<=0;overrun<=0;warmup<=0;
 end else begin
 out_valid<=0;
 if(sample_valid) begin
   wr_ptr<=wr_ptr+1'b1;
   if(warmup<8'd127) warmup<=warmup+1'b1;
   mod8<=mod8+1'b1;
   if(mod8==3'd7) begin
     if(state!=IDLE) overrun<=1;
     else begin base_ptr<=wr_ptr;tap<=0;acc<=0;state<=REQUEST;busy<=1;end
   end
 end
 case(state)
 REQUEST: begin rd_addr<=base_ptr-tap;state<=WAIT_READ;end
 WAIT_READ: state<=MULTIPLY;
 MULTIPLY: begin product<=$signed(rd_q)*$signed(coeff(tap));state<=ACCUM;end
 ACCUM: begin
   acc<=acc+$signed(product);
   if(tap==7'd126) state<=COMMIT;
   else begin tap<=tap+1'b1;state<=REQUEST;end
 end
 COMMIT: begin
   // At 6 kHz, FIR initial history must be primed before trusting signal.
   sample_out<=sat16(acc >>> 15);
   out_valid<=(warmup>=8'd127);
   busy<=0;state<=IDLE;
 end
 default: ;
 endcase
 end end
endmodule
