`timescale 1ns/1ps
// DE2 feedback ANC, experimental. CLK = CLOCK_50. ADC/DAC nominal Fs=48 kHz.
// 16-bit audio Q1.15; FIR weights Q2.30; products/accumulators Q3.45.
// Calibration produces a *candidate* S_hat; it cannot certify acoustic fit.
// SW3 (output_mute) is a hardware-operational emergency mute/abort.
module feedback_fxlms_core #(
  parameter integer M_TAPS = 128,
  parameter integer L_TAPS = 64,
  parameter integer DELTA = 4,
  parameter integer CALIB_SAMPLES = 96000,
  parameter integer CALIB_TIMEOUT_CYCLES = 200000000,
  parameter integer MAX_DRIVE = 8192
)(
  input wire clk, reset_n,
  input wire btn_calib_start,
  input wire sw_anc_enable,
  input wire [1:0] sw_mu_sel,
  input wire output_mute,
  input wire sample_valid,
  input wire signed [15:0] mic_in,
  input wire signed [15:0] white_noise,
  output reg signed [15:0] spk_out,
  output reg calib_busy, calib_ready, calib_failed, anc_active,
  output reg sample_done, dsp_overrun
);
  reg signed [31:0] s_hat_ram [0:M_TAPS-1];
  reg signed [31:0] w_ram [0:L_TAPS-1];
  reg signed [15:0] v_buf [0:M_TAPS-1];
  reg signed [15:0] y_buf [0:M_TAPS-1];
  localparam integer X_DEPTH = (M_TAPS > L_TAPS) ? M_TAPS : L_TAPS;
  reg signed [15:0] x_buf [0:X_DEPTH-1];
  reg signed [15:0] xf_buf [0:L_TAPS-1];
  reg signed [15:0] d_buf [0:DELTA];

  localparam [3:0] IDLE=0, S_FIR=1, S_ERR=2, S_UPDATE=3,
                   Y_PRED=4, SYNTH_X=5, Y_FIR=6, X_FILTER=7,
                   W_UPDATE=8, DONE=9, Y_COMMIT=10, XF_COMMIT=11;
  reg [3:0] state;
  reg [15:0] tap;
  reg signed [63:0] acc;
  reg signed [15:0] e_sample, e_ident, xf_new;
  reg [31:0] calib_count, signal_count, timeout_count;
  reg processing_calibration;
  wire [4:0] shift_w = (sw_mu_sel==0) ? 5'd8 :
                       (sw_mu_sel==1) ? 5'd9 :
                       (sw_mu_sel==2) ? 5'd10 : 5'd11;
  localparam integer SHIFT_S = 8;
  localparam signed [63:0] COEF_MAX = 64'sd805306368; // +0.75 in Q2.30
  localparam signed [63:0] COEF_MIN = -64'sd805306368;

  // Clip signed Q3.45 -> signed Q1.15; 30 fractional bits are discarded.
  function signed [15:0] q45_to_pcm;
    input signed [63:0] q;
    reg signed [63:0] t;
    begin
      t = q >>> 30;
      if(t > 64'sd32767) q45_to_pcm=16'sh7fff;
      else if(t < -64'sd32768) q45_to_pcm=16'sh8000;
      else q45_to_pcm=t[15:0];
    end
  endfunction
  function signed [15:0] diff_pcm;
    input signed [15:0] a,b;
    reg signed [17:0] t;
    begin
      t = $signed(a)-$signed(b);
      if(t>18'sd32767) diff_pcm=16'sh7fff;
      else if(t < -18'sd32768) diff_pcm=16'sh8000;
      else diff_pcm=t[15:0];
    end
  endfunction
  // Digital output limiter to +/-0.25 full-scale (amplifier gain remains external).
  function signed [15:0] limit_drive;
    input signed [15:0] x;
    begin
      if($signed(x) > MAX_DRIVE-1) limit_drive=MAX_DRIVE-1;
      else if($signed(x) < -MAX_DRIVE) limit_drive=-MAX_DRIVE;
      else limit_drive=x;
    end
  endfunction
  function signed [31:0] clip_coef;
    input signed [63:0] a;
    begin
      if(a>COEF_MAX) clip_coef=COEF_MAX[31:0];
      else if(a<COEF_MIN) clip_coef=COEF_MIN[31:0];
      else clip_coef=a[31:0];
    end
  endfunction
  wire signed [15:0] pred_sample=q45_to_pcm(acc);
  wire signed [15:0] y_sample=q45_to_pcm(acc);
  reg signed [63:0] gradient;
  integer i;

  always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
      state<=IDLE; tap<=0; acc<=0; e_sample<=0; e_ident<=0; xf_new<=0;
      calib_busy<=0; calib_ready<=0; calib_failed<=0; anc_active<=0;
      spk_out<=0; sample_done<=0; dsp_overrun<=0;
      calib_count<=0;signal_count<=0;timeout_count<=0;
      processing_calibration<=0; gradient<=0;
      for(i=0;i<M_TAPS;i=i+1) begin
        s_hat_ram[i]<=0; v_buf[i]<=0; y_buf[i]<=0;
      end
      for(i=0;i<L_TAPS;i=i+1) begin
        w_ram[i]<=0; xf_buf[i]<=0;
      end
      for(i=0;i<X_DEPTH;i=i+1) x_buf[i]<=0;
      for(i=0;i<=DELTA;i=i+1) d_buf[i]<=0;
    end else begin
      sample_done<=0;
      if(sample_valid && state!=IDLE) dsp_overrun<=1;
      // Immediate operational mute: abort any running adaptive state, no output.
      if(output_mute) begin
        spk_out<=0; anc_active<=0;
        if(sample_valid) sample_done<=1;
        if(calib_busy || calib_ready || anc_active) begin
          calib_busy<=0; calib_ready<=0; calib_failed<=1;
        end
        state<=IDLE;
      end else if(btn_calib_start && state==IDLE) begin
        calib_busy<=1; calib_ready<=0; calib_failed<=0; anc_active<=0;
        calib_count<=0; signal_count<=0; timeout_count<=0;
        spk_out<=0;
        for(i=0;i<M_TAPS;i=i+1) begin
          s_hat_ram[i]<=0; v_buf[i]<=0; y_buf[i]<=0;
        end
        for(i=0;i<L_TAPS;i=i+1) begin
          w_ram[i]<=0; xf_buf[i]<=0;
        end
        for(i=0;i<X_DEPTH;i=i+1) x_buf[i]<=0;
      for(i=0;i<=DELTA;i=i+1) d_buf[i]<=0;
      end else if(calib_busy && timeout_count>=CALIB_TIMEOUT_CYCLES-1) begin
        // Timeout is a FAIL, never an automatic success.
        calib_busy<=0;calib_ready<=0;calib_failed<=1;anc_active<=0;
        spk_out<=0;state<=IDLE;
      end else begin
        if(calib_busy) timeout_count<=timeout_count+1'b1;
        case(state)
          IDLE: begin
            if(sample_valid) begin
              e_sample<=mic_in;
              if(calib_busy) begin
                processing_calibration<=1;
                for(i=M_TAPS-1;i>0;i=i-1) v_buf[i]<=v_buf[i-1];
                v_buf[0]<=white_noise;
                // The anti-noise speaker receives this through I2S after computation.
                spk_out<=white_noise;
                if((mic_in>16'sd128)||(mic_in < -16'sd128)) signal_count<=signal_count+1'b1;
                tap<=0;acc<=0;state<=S_FIR;
              end else if(calib_ready && sw_anc_enable) begin
                processing_calibration<=0;anc_active<=1;
                tap<=0;acc<=0;state<=Y_PRED;
              end else begin
                processing_calibration<=0;anc_active<=0;spk_out<=0;
                sample_done<=1;
              end
            end
          end
          S_FIR: begin
            acc<=acc+($signed(s_hat_ram[tap])*$signed(v_buf[tap]));
            if(tap==M_TAPS-1) state<=S_ERR;
            else tap<=tap+1'b1;
          end
          S_ERR: begin
            e_ident<=diff_pcm(e_sample,pred_sample);
            tap<=0;state<=S_UPDATE;
          end
          S_UPDATE: begin
            gradient=($signed(e_ident)*$signed(v_buf[tap])) >>> SHIFT_S;
            s_hat_ram[tap]<=clip_coef($signed(s_hat_ram[tap])+gradient);
            if(tap==M_TAPS-1) begin
              calib_count<=calib_count+1'b1;
              if(calib_count==CALIB_SAMPLES-1) begin
                calib_busy<=0;
                // Weak/no microphone signal must not be reported as ready.
                // This is only a signal-presence gate, NOT a fit-quality check.
                if(signal_count >= CALIB_SAMPLES/32) begin
                  calib_ready<=1;calib_failed<=0;
                end else begin
                  calib_ready<=0;calib_failed<=1;
                end
                spk_out<=0;
              end
              state<=DONE;
            end else tap<=tap+1'b1;
          end
          Y_PRED: begin
            // S_hat[k] refers to output created at sample n-k.
            // y_buf[0] is the LAST emitted output (n-1), so pair it
            // with S_hat[1], not S_hat[0].
            if(tap<M_TAPS-1)
              acc<=acc+($signed(s_hat_ram[tap+1'b1])*$signed(y_buf[tap]));
            if(tap==M_TAPS-1) state<=SYNTH_X;
            else tap<=tap+1'b1;
          end
          SYNTH_X: begin
            // The next state needs the corrected d=e-S*y, with the FIR result
            // available now; calculate from acc directly instead of old y_pred.
            for(i=DELTA;i>0;i=i-1) d_buf[i]<=d_buf[i-1];
            d_buf[0]<=diff_pcm(e_sample,pred_sample);
            for(i=X_DEPTH-1;i>0;i=i-1) x_buf[i]<=x_buf[i-1];
            x_buf[0]<=d_buf[DELTA];
            tap<=0;acc<=0;state<=Y_FIR;
          end
          Y_FIR: begin
            acc<=acc+($signed(w_ram[tap])*$signed(x_buf[tap]));
            if(tap==L_TAPS-1) state<=Y_COMMIT;
            else tap<=tap+1'b1;
          end
          Y_COMMIT: begin
            // All L MAC terms including the last tap are in acc now.
            spk_out<=limit_drive(y_sample);
            for(i=M_TAPS-1;i>0;i=i-1) y_buf[i]<=y_buf[i-1];
            y_buf[0]<=limit_drive(y_sample);
            tap<=0;acc<=0;state<=X_FILTER;
          end
          X_FILTER: begin
            acc<=acc+($signed(s_hat_ram[tap])*$signed(x_buf[tap]));
            if(tap==M_TAPS-1) state<=XF_COMMIT;
            else tap<=tap+1'b1;
          end
          XF_COMMIT: begin
            xf_new<=pred_sample;
            tap<=0;state<=W_UPDATE;
          end
          W_UPDATE: begin
            // x_f[n] = S_hat*x[n]; W update must use x_f[n-l].
            // Current x_f[n] is xf_new; historical x_f are xf_buf[l-1].
            if(tap==0)
              gradient=($signed(e_sample)*$signed(xf_new)) >>> shift_w;
            else
              gradient=($signed(e_sample)*$signed(xf_buf[tap-1'b1])) >>> shift_w;
            w_ram[tap]<=clip_coef($signed(w_ram[tap])-gradient);
            if(tap==L_TAPS-1) state<=DONE;
            else tap<=tap+1'b1;
          end
          DONE: begin
            if(!processing_calibration) begin
              for(i=L_TAPS-1;i>0;i=i-1) xf_buf[i]<=xf_buf[i-1];
              xf_buf[0]<=xf_new;
            end
            sample_done<=1;
            state<=IDLE;
          end
          default: state<=IDLE;
        endcase
      end
    end
  end
endmodule
