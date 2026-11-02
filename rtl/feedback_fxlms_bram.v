`timescale 1ns/1ps
// Rev C.1 experimental feedback ANC core (6 kHz, 50 MHz).
// + Secondary path S_hat: Q2.22, trained during CAL only, FROZEN during ANC.
// + W accumulator: 40-bit signed Q2.38; MAC coefficient quantized to Q2.22.
// + Filtered reference x_f: signed Q2.30 with 32-bit history.
// + 24x16 coefficient MAC split into 17x16 and 8x16 multiplications;
//   32x16 gradient split into 17x16 and 16x16 multiplications.
// + Circular sample histories and synchronous M4K read, no array-wide reset.
// + D_BULK must be estimated using real system impulse response; it is not 2m/c.
// + This RTL is a prototype: run unit tests, TimeQuest and SPL safety validation.
module feedback_fxlms_bram #(
 parameter integer M_TAPS=128,
 parameter integer L_TAPS=64,
 parameter integer D_BULK=12,
 parameter integer DELTA=0,
 parameter integer CALIB_SAMPLES=36000,
 parameter integer VALID_SAMPLES=2048,
 parameter integer S_MU_SHIFT=5,
 parameter integer LIMIT_PCM=8192,
 parameter integer HIST_AW=8
)(
 input wire clk, rst_n,
 input wire calib_start, anc_enable, emergency_mute,
 input wire [1:0] mu_sel,
 input wire sample_valid,
 input wire signed [15:0] mic_in, prbs_sample,
 output reg signed [15:0] spk_out,
 output reg sample_done,
 output reg calib_busy,calib_valid,calib_failed,anc_active,
 output reg dsp_overrun, output reg fault_latch,
 output reg [15:0] last_cycles,
 output reg signed [31:0] debug_xf_q30,
 output reg signed [39:0] debug_w_q38,
 output reg [7:0] debug_state
);
 localparam integer HIST_DEPTH=(1<<HIST_AW);
 localparam signed [63:0] S_MAX=64'sd3145728;   // 0.75 * 2^22
 localparam signed [63:0] W_MAX=64'sd206158430208; // 0.75 * 2^38
 localparam [4:0]
   CLEAR=0, IDLE=1, F_REQ=2, F_WAIT=3, F_MUL_LO=4,
   F_MUL_HI=5, F_PROD=6, F_ACC=7, F_DONE=8,
   S_REQ=9, S_WAIT=10, S_MULT=11, S_COMMIT=12,
   W_REQ=13, W_WAIT=14, W_MUL_LO=15, W_MUL_HI=16,
   W_PROD=17, W_COMMIT=18, FAULT=19, F_COEF=20, W_CALC=21;
 localparam [1:0] OP_IDENT=0, OP_PRED=1, OP_W=2, OP_XF=3;
 localparam [1:0] CAL_OFF=0, CAL_TRAIN=1, CAL_VALIDATE=2;
 reg [4:0] state;
 reg [1:0] op,cal_mode;
 reg [HIST_AW-1:0] frame_ptr,clear_ptr;
 reg [8:0] tap;
 reg [31:0] sample_counter;
 reg [31:0] validate_counter;
 reg [63:0] err_energy,mic_energy;
 reg signed [15:0] e_sample,e_ident,x_current;
 reg signed [15:0] y_current;
 reg signed [31:0] xf_current;
 reg signed [63:0] acc;
 reg signed [32:0] mul_lo;
 reg signed [31:0] mul_hi;
 reg signed [63:0] product_full;
 reg signed [23:0] coef_work;
 reg signed [15:0] data_work;
 reg signed [63:0] w_new_reg;
 reg [15:0] cycles;
 reg [29:0] cal_watchdog;
 reg [5:0] limiter_run,clip_run;
 reg [7:0] history_samples;
 wire [4:0] w_mu_shift = 5'd6 + {3'b000,mu_sel}; // 1/64...1/512

 // Strict single read + single write per array; synchronous registered reads.
 (* ramstyle = "M4K" *) reg signed [23:0] s_ram[0:M_TAPS-1];
 (* ramstyle = "M4K" *) reg signed [39:0] w_ram[0:L_TAPS-1];
 (* ramstyle = "M4K" *) reg signed [15:0] v_hist[0:HIST_DEPTH-1];
 (* ramstyle = "M4K" *) reg signed [15:0] y_hist[0:HIST_DEPTH-1];
 (* ramstyle = "M4K" *) reg signed [15:0] x_hist[0:HIST_DEPTH-1];
 (* ramstyle = "M4K" *) reg signed [31:0] xf_hist[0:HIST_DEPTH-1];
 reg [8:0] s_addr,w_addr;
 reg [HIST_AW-1:0] v_addr,y_addr,x_addr,xf_addr;
 reg signed [23:0] s_q;
 reg signed [39:0] w_q;
 reg signed [15:0] v_q,y_q,x_q;
 reg signed [31:0] xf_q;

 function signed [15:0] sat_pcm(input signed [63:0] val);
 begin if(val>32767) sat_pcm=32767;
 else if(val< -32768) sat_pcm=-32768;
 else sat_pcm=val[15:0];end endfunction
 function signed [31:0] sat_q30(input signed [63:0] val);
 begin if(val>64'sd2147483647) sat_q30=32'sd2147483647;
 else if(val< -64'sd2147483648) sat_q30=-32'sd2147483648;
 else sat_q30=val[31:0];end endfunction
 function signed [23:0] sat_s(input signed [63:0] val);
 begin if(val>S_MAX) sat_s=S_MAX[23:0];
 else if(val< -S_MAX) sat_s=-24'sd3145728;
 else sat_s=val[23:0];end endfunction
 function signed [39:0] sat_w(input signed [63:0] val);
 begin if(val>W_MAX) sat_w=W_MAX[39:0];
 else if(val< -W_MAX) sat_w=-40'sd206158430208;
 else sat_w=val[39:0];end endfunction
 function signed [15:0] limit_pcm(input signed [15:0] val);
 begin if(val>LIMIT_PCM-1) limit_pcm=LIMIT_PCM-1;
 else if(val< -LIMIT_PCM) limit_pcm=-LIMIT_PCM;
 else limit_pcm=val;end endfunction
 function signed [23:0] quant_w22(input signed [39:0] q38);
 reg signed [63:0] a;
 begin
   a=q38;
   if(a>=0) quant_w22=sat_s((a+64'sd32768) >>> 16);
   else quant_w22=sat_s(-(((-a)+64'sd32768) >>> 16));
 end endfunction
 function signed [63:0] shift_to_zero(input signed [63:0] val,input integer shift);
 begin if(val<0)shift_to_zero=-((-$signed(val)) >>> shift);
 else shift_to_zero=$signed(val) >>> shift;end endfunction
 function signed [39:0] leak_tozero(input signed [39:0] q38);
 begin if(q38<0) leak_tozero=-((-$signed(q38)) >>> 15);
 else leak_tozero=q38 >>> 15;end endfunction
 function signed [15:0] diff_pcm(input signed [15:0] a,input signed [15:0] b);
 reg signed [63:0] v;
 begin v=$signed(a)-$signed(b);diff_pcm=sat_pcm(v);end endfunction

 wire signed [23:0] coef_sel=(op==OP_W)?quant_w22(w_q):s_q;
 wire signed [15:0] data_sel=(op==OP_IDENT)?v_q:
                                (op==OP_PRED)?y_q:x_q;
 wire signed [15:0] fir_out=sat_pcm(acc >>> 22); // Q3.37 -> Q1.15
 wire signed [15:0] predicted_error=diff_pcm(e_sample,fir_out);
 wire signed [31:0] xf_out=sat_q30(acc >>> 7); // Q3.37 -> Q2.30
 wire signed [63:0] s_delta=shift_to_zero(product_full,8+S_MU_SHIFT);
 wire signed [63:0] w_delta=shift_to_zero(product_full,7+w_mu_shift);
 wire signed [63:0] w_new=$signed(w_q)-$signed(leak_tozero(w_q))-$signed(w_delta);
 wire [63:0] mic_pow=$signed(e_sample)*$signed(e_sample);
 wire [63:0] err_pow=$signed(predicted_error)*$signed(predicted_error);

 always @(posedge clk) begin
   s_q <= s_ram[s_addr];
   w_q <= w_ram[w_addr];
   v_q <= v_hist[v_addr];
   y_q <= y_hist[y_addr];
   x_q <= x_hist[x_addr];
   xf_q <= xf_hist[xf_addr];
   if(state==CLEAR) begin
     if(clear_ptr<M_TAPS) s_ram[clear_ptr]<=24'sd0;
     if(clear_ptr<L_TAPS) w_ram[clear_ptr]<=40'sd0;
     v_hist[clear_ptr]<=0;
     y_hist[clear_ptr]<=0;
     x_hist[clear_ptr]<=0;
     xf_hist[clear_ptr]<=0;
   end else begin
     if(state==IDLE && sample_valid && cal_mode!=CAL_OFF && !emergency_mute)
       v_hist[frame_ptr]<=prbs_sample;
     if(state==F_DONE && op==OP_PRED)
       x_hist[frame_ptr]<=predicted_error;
     if(state==F_DONE && op==OP_W)
       y_hist[frame_ptr]<=limit_pcm(fir_out);
     if(state==F_DONE && op==OP_XF)
       xf_hist[frame_ptr]<=xf_out;
     if(state==S_COMMIT)
       s_ram[tap]<=sat_s($signed(s_q)+s_delta);
     if(state==W_COMMIT)
       w_ram[tap]<=sat_w(w_new_reg);
   end
 end
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin
   state<=CLEAR;op<=OP_IDENT;cal_mode<=CAL_OFF;
   clear_ptr<=0;frame_ptr<=0;tap<=0;
   sample_counter<=0;validate_counter<=0;err_energy<=0;mic_energy<=0;
   e_sample<=0;e_ident<=0;x_current<=0;y_current<=0;xf_current<=0;
   acc<=0;mul_lo<=0;mul_hi<=0;product_full<=0;
   coef_work<=0;data_work<=0;w_new_reg<=0;
   s_addr<=0;w_addr<=0;v_addr<=0;y_addr<=0;x_addr<=0;xf_addr<=0;
   cycles<=0;cal_watchdog<=0;last_cycles<=0;limiter_run<=0;clip_run<=0;history_samples<=0;
   spk_out<=0;sample_done<=0;calib_busy<=0;calib_valid<=0;
   calib_failed<=0;anc_active<=0;dsp_overrun<=0;fault_latch<=0;
   debug_xf_q30<=0;debug_w_q38<=0;debug_state<=0;
 end else begin
   sample_done<=0;debug_state<={3'b000,state};
   if(calib_busy) cal_watchdog<=cal_watchdog+1'b1;
   if(state!=IDLE && state!=CLEAR && state!=FAULT)
     cycles<=cycles+1'b1;
   if(sample_valid && state!=IDLE && state!=CLEAR && state!=FAULT)
     dsp_overrun<=1;
   // Emergency mute takes precedence and invalidates the current calibration.
   if(emergency_mute) begin
     spk_out<=0;anc_active<=0;calib_busy<=0;calib_valid<=0;
     cal_mode<=CAL_OFF;
     if(state!=CLEAR && state!=FAULT) state<=IDLE;
   end else if(calib_busy && cal_watchdog>=30'd500000000) begin
     spk_out<=0;anc_active<=0;calib_busy<=0;calib_valid<=0;
     calib_failed<=1;cal_mode<=CAL_OFF;state<=FAULT;fault_latch<=1;
   end else if(dsp_overrun || fault_latch) begin
     spk_out<=0;anc_active<=0;calib_busy<=0;calib_valid<=0;
     calib_failed<=1;fault_latch<=1;state<=FAULT;
   end else case(state)
   CLEAR: begin
     if(clear_ptr==HIST_DEPTH-1) begin
       clear_ptr<=0;frame_ptr<=0;state<=IDLE;
     end else clear_ptr<=clear_ptr+1'b1;
   end
   IDLE: begin
     if(calib_start && !calib_busy) begin
       clear_ptr<=0;state<=CLEAR;cal_mode<=CAL_TRAIN;
       calib_busy<=1;calib_valid<=0;calib_failed<=0;anc_active<=0;
       sample_counter<=0;validate_counter<=0;mic_energy<=0;err_energy<=0;
       spk_out<=0;cal_watchdog<=0;limiter_run<=0;clip_run<=0;history_samples<=0;
     end else if(sample_valid) begin
       e_sample<=mic_in;cycles<=0;
       if(cal_mode!=CAL_OFF) begin
         spk_out<=prbs_sample;anc_active<=0;
         op<=OP_IDENT;tap<=0;acc<=0;state<=F_REQ;
       end else if(calib_valid && anc_enable) begin
         anc_active<=1;op<=OP_PRED;tap<=0;acc<=0;state<=F_REQ;
       end else begin
         spk_out<=0;anc_active<=0;sample_done<=1;
       end
     end
   end
   F_REQ: begin
     if(op==OP_W) begin
       w_addr<=tap;
       x_addr<=frame_ptr-DELTA-tap;
     end else begin
       s_addr<=tap;
       if(op==OP_IDENT) v_addr<=frame_ptr-D_BULK-tap;
       else if(op==OP_PRED) y_addr<=frame_ptr-D_BULK-tap;
       else x_addr<=frame_ptr-D_BULK-tap;
     end
     state<=F_WAIT;
   end
   F_WAIT: state<=F_COEF;
   F_COEF: begin coef_work<=coef_sel;data_work<=data_sel;state<=F_MUL_LO;end
   F_MUL_LO: begin
     // 17x16 signed (unsigned low half of 24-bit coefficient)
     mul_lo<=$signed({1'b0,coef_work[15:0]})*$signed(data_work);
     state<=F_MUL_HI;
   end
   F_MUL_HI: begin
     // 8x16 signed high half
     mul_hi<=$signed(coef_work[23:16])*$signed(data_work);
     state<=F_PROD;
   end
   F_PROD: begin
     product_full<=$signed(mul_lo)+($signed(mul_hi) <<< 16);
     state<=F_ACC;
   end
   F_ACC: begin
     acc<=acc+product_full;
     if(tap==((op==OP_W)?L_TAPS-1:M_TAPS-1)) state<=F_DONE;
     else begin tap<=tap+1'b1;state<=F_REQ;end
   end
   F_DONE: begin
     if(op==OP_IDENT) begin
       e_ident<=predicted_error;
       if(cal_mode==CAL_TRAIN) begin tap<=0;state<=S_REQ;end
       else begin
         err_energy<=err_energy+err_pow;
         mic_energy<=mic_energy+mic_pow;
         frame_ptr<=frame_ptr+1'b1;sample_done<=1;last_cycles<=cycles;
         if(validate_counter==VALID_SAMPLES-1) begin
           cal_mode<=CAL_OFF;calib_busy<=0;spk_out<=0;
           if((mic_energy+mic_pow)>64'd2097152 &&
              ((err_energy+err_pow) << 2)<(mic_energy+mic_pow)) begin
             calib_valid<=1;calib_failed<=0;
           end else begin calib_valid<=0;calib_failed<=1;end
         end else validate_counter<=validate_counter+1'b1;
         state<=IDLE;
       end
     end else if(op==OP_PRED) begin
       x_current<=predicted_error;
       op<=OP_W;tap<=0;acc<=0;state<=F_REQ;
     end else if(op==OP_W) begin
       y_current<=limit_pcm(fir_out);
       spk_out<=limit_pcm(fir_out);
       if(fir_out != limit_pcm(fir_out)) begin
         if(limiter_run<63) limiter_run<=limiter_run+1'b1;
       end else limiter_run<=0;
       op<=OP_XF;tap<=0;acc<=0;state<=F_REQ;
     end else begin
       xf_current<=xf_out;debug_xf_q30<=xf_out;
       tap<=0;state<=W_REQ;
     end
   end
   S_REQ: begin
     s_addr<=tap;v_addr<=frame_ptr-D_BULK-tap;
     state<=S_WAIT;
   end
   S_WAIT: state<=S_MULT;
   S_MULT: begin
     product_full<=$signed(e_ident)*$signed(v_q); // Q2.30
     state<=S_COMMIT;
   end
   S_COMMIT: begin
     if(tap==M_TAPS-1) begin
       frame_ptr<=frame_ptr+1'b1;last_cycles<=cycles;sample_done<=1;
       if(sample_counter==CALIB_SAMPLES-1) begin
         cal_mode<=CAL_VALIDATE;validate_counter<=0;mic_energy<=0;err_energy<=0;
       end else sample_counter<=sample_counter+1'b1;
       state<=IDLE;
     end else begin tap<=tap+1'b1;state<=S_REQ;end
   end
   W_REQ: begin
     w_addr<=tap;xf_addr<=frame_ptr-tap;
     state<=W_WAIT;
   end
   W_WAIT: state<=W_MUL_LO;
   W_MUL_LO: begin
     // Q2.30 x_f split unsigned low-half (17x16) times signed e.
     mul_lo<=$signed({1'b0,xf_q[15:0]})*$signed(e_sample);
     state<=W_MUL_HI;
   end
   W_MUL_HI: begin
     mul_hi<=$signed(xf_q[31:16])*$signed(e_sample);
     state<=W_PROD;
   end
   W_PROD: begin
     product_full<=$signed(mul_lo)+($signed(mul_hi) <<< 16); // Q3.45
     state<=W_CALC;
   end
   W_CALC: begin w_new_reg<=w_new;state<=W_COMMIT;end
   W_COMMIT: begin
     debug_w_q38<=sat_w(w_new_reg);
     if(tap==L_TAPS-1) begin
       frame_ptr<=frame_ptr+1'b1;sample_done<=1;last_cycles<=cycles;
       if(limiter_run>=6'd16) begin
         fault_latch<=1;spk_out<=0;calib_valid<=0;state<=FAULT;
       end else if((e_sample>16'sd31000)||(e_sample< -16'sd31000)) begin
         if(clip_run>=6'd7) begin
           fault_latch<=1;spk_out<=0;calib_valid<=0;state<=FAULT;
         end else clip_run<=clip_run+1'b1;
         state<=IDLE;
       end else begin clip_run<=0;state<=IDLE;end
     end else begin tap<=tap+1'b1;state<=W_REQ;end
   end
   FAULT: begin spk_out<=0;anc_active<=0;calib_busy<=0;end
   default: state<=FAULT;
   endcase
 end end
endmodule
