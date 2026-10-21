/*
 * Copyright (c) 2026 Arka Khorchidian
 * SPDX-License-Identifier: Apache-2.0
 *
 * Polyglot protocol-emulator core.
 *
 * One cycle-deterministic execution unit. Every core "tick" (system clock
 * divided by CLKDIV+1) either burns one cycle of an instruction's delay
 * field or executes one instruction. See docs/ISA.md for the full spec;
 * model/polyglot.py is the executable reference model.
 *
 * The datapath is written as explicit per-register write selects (rather
 * than one big behavioural case) because that keeps the mux count low.
 */

`default_nettype none

module pg_core #(
    parameter [1:0] CORE_ID = 2'd0
) (
    input  wire        clk,
    input  wire        rst_n,

    // run control
    input  wire        en,            // core enabled
    input  wire        step,          // single tick (pulse)
    input  wire        restart,       // clear internal state (pulse)
    input  wire        div_sync,      // reset clock divider phase (pulse)
    output wire        brk_hit,       // breakpoint reached: top clears en

    // configuration
    input  wire [15:0] cfg_clkdiv,
    input  wire [31:0] cfg_exec,
    input  wire [14:0] cfg_shift,
    input  wire [11:0] cfg_pin,
    input  wire [23:0] cfg_line,
    input  wire [31:0] cfg_poly,

    // instruction memory. The memory registers imem[pc_nxt] and
    // imem[dptr_nxt] at each edge, so imem_data/dmem_data are the word at
    // the current pc/dptr without a memory read in the execute path.
    output wire [6:0]  pc_o,
    output wire [6:0]  pc_nxt_o,
    input  wire [19:0] imem_data,
    output wire [6:0]  dptr_o,
    output wire [6:0]  dptr_nxt_o,
    input  wire [19:0] dmem_data,

    // host direct register writes
    input  wire        hw_pc,
    input  wire        hw_x,
    input  wire        hw_y,
    input  wire        hw_crc,
    input  wire        hw_isr,
    input  wire        hw_osr,
    input  wire        hw_instr,
    input  wire [31:0] hw_data,

    // virtual pins
    input  wire [7:0]  vin,
    output reg  [7:0]  vout,
    output reg  [7:0]  voe,

    // FIFOs
    input  wire        tx_empty,
    input  wire [31:0] tx_data,
    output wire        tx_pop,
    input  wire        rx_full,
    output wire        rx_push,
    output wire [31:0] rx_data,

    // shared IRQ flags and timestamp
    input  wire [7:0]  irq,
    output wire [7:0]  irq_set,
    output wire [7:0]  irq_clr,
    input  wire [31:0] time_i,

    // observation
    output wire [31:0] x_o,
    output wire [31:0] y_o,
    output wire [31:0] isr_o,
    output wire [31:0] osr_o,
    output wire [31:0] crc_o,
    output wire [31:0] status_o,
    output wire [19:0] instr_o
);

  localparam OP_JMP = 4'h0, OP_WAIT = 4'h1, OP_IN = 4'h2, OP_OUT = 4'h3,
             OP_PP  = 4'h4, OP_MOV  = 4'h5, OP_IRQ = 4'h6, OP_SET = 4'h7,
             OP_ADD = 4'h8, OP_CTL  = 4'h9;

  // ------------------------------------------------------------------
  // Configuration decode
  // ------------------------------------------------------------------
  wire [6:0] wrap_bot   = cfg_exec[6:0];
  wire [6:0] wrap_top   = cfg_exec[14:8];
  wire       side_dirs  = cfg_exec[15];
  wire [1:0] side_mode  = cfg_exec[17:16];
  wire [2:0] side_base  = cfg_exec[20:18];
  wire [2:0] jmp_pin    = cfg_exec[23:21];
  wire [6:0] brk_addr   = cfg_exec[30:24];
  wire       brk_en     = cfg_exec[31];

  wire       autopull   = cfg_shift[0];
  wire       autopush   = cfg_shift[1];
  wire       out_right  = cfg_shift[2];
  wire       in_right   = cfg_shift[3];
  wire [5:0] pull_th    = (cfg_shift[8:4]  == 5'd0) ? 6'd32 : {1'b0, cfg_shift[8:4]};
  wire [5:0] push_th    = (cfg_shift[13:9] == 5'd0) ? 6'd32 : {1'b0, cfg_shift[13:9]};
  wire       pull_imem  = cfg_shift[14];   // PULL / autopull read imem[dptr++]

  wire [2:0] out_base   = cfg_pin[2:0];
  wire [2:0] set_base   = cfg_pin[5:3];
  wire [3:0] set_count  = {1'b0, cfg_pin[8:6]} + 4'd1;
  wire [2:0] in_base    = cfg_pin[11:9];

  wire [1:0] line_mode  = cfg_line[1:0];   // 0 NRZ, 1 NRZI, 2 Manchester
  wire       line_inv   = cfg_line[2];
  wire       line_diff  = cfg_line[3];
  wire [2:0] stuff_len  = cfg_line[6:4];
  wire       stuff_any  = cfg_line[7];
  wire       crc_msb    = cfg_line[8];
  wire [4:0] crc_wm1    = cfg_line[13:9];
  wire       line_idle  = cfg_line[14];
  wire [7:0] manch_half = cfg_line[23:16];

  // ------------------------------------------------------------------
  // Architectural state
  // ------------------------------------------------------------------
  reg [6:0]  pc;
  reg [6:0]  dptr;
  reg [31:0] x, y, isr, osr, crc;
  reg [5:0]  isr_cnt, osr_cnt;
  reg [4:0]  delay_cnt;
  reg [15:0] div_cnt;
  reg [7:0]  vin_prev;
  reg        to_flag;        // last timeout-WAIT exited on timeout
  reg        irq_armed;      // IRQ set+wait: flag already raised
  reg        inj_pend;
  reg [19:0] inj_instr;
  reg        brk_skip;
  reg        stalled;

  // line coder
  reg [3:0]  lflags;         // {err, code_en, stuff_en, crc_en}
  reg [2:0]  run_cnt;
  reg        run_val;
  reg        nrzi_prev;
  reg        manch_pend;
  reg [7:0]  manch_cnt;
  reg        manch_lvl;
  reg        last_bit;
  reg        stuff_flag;

  assign pc_o  = pc;
  assign dptr_o = dptr;
  assign x_o   = x;
  assign y_o   = y;
  assign isr_o = isr;
  assign osr_o = osr;
  assign crc_o = crc;

  // ------------------------------------------------------------------
  // Tick / execute qualification
  // ------------------------------------------------------------------
  wire div_zero = (div_cnt == 16'd0);
  wire run_tick = en & div_zero;
  // An injected instruction on a halted core runs on the next clock.
  wire tick     = run_tick | step | (inj_pend & ~en);
  wire no_delay = (delay_cnt == 5'd0);

  wire at_brk  = brk_en & (pc == brk_addr) & ~inj_pend & ~brk_skip & ~step;
  assign brk_hit = run_tick & no_delay & at_brk;
  wire exec    = tick & no_delay & ~brk_hit;

  wire [19:0] instr = inj_pend ? inj_instr : imem_data;
  assign instr_o = imem_data;

  wire [3:0]  op  = instr[19:16];
  wire [1:0]  ss  = instr[15:14];
  wire [2:0]  dly = instr[13:11];
  wire [10:0] arg = instr[10:0];

  wire is_jmp  = (op == OP_JMP);
  wire is_wait = (op == OP_WAIT);
  wire is_in   = (op == OP_IN);
  wire is_out  = (op == OP_OUT);
  wire is_pp   = (op == OP_PP);
  wire is_mov  = (op == OP_MOV);
  wire is_irq  = (op == OP_IRQ);
  wire is_set  = (op == OP_SET);
  wire is_add  = (op == OP_ADD);
  wire is_ctl  = (op == OP_CTL);

  wire [2:0] f_hi  = arg[10:8];          // IN src / OUT, MOV, SET dst
  wire [2:0] f_src = arg[5:3];           // MOV src
  wire [1:0] f_mop = arg[7:6];           // MOV operation
  wire [5:0] cnt_f = (arg[4:0] == 5'd0) ? 6'd32 : {1'b0, arg[4:0]};

  wire in_line  = is_in  & (f_hi == 3'd4);
  wire out_line = is_out & (f_hi == 3'd4);
  wire [5:0] cnt = (in_line | out_line) ? 6'd1 : cnt_f;

  // side-set / delay split
  reg [4:0] delay_val;
  reg       side_en, side_two;
  always @* begin
    case (side_mode)
      2'd0:    begin delay_val = {ss, dly};          side_en = 1'b0;  side_two = 1'b0; end
      2'd1:    begin delay_val = {1'b0, ss[1], dly}; side_en = 1'b1;  side_two = 1'b0; end
      2'd2:    begin delay_val = {2'b00, dly};       side_en = 1'b1;  side_two = 1'b1; end
      default: begin delay_val = {2'b00, dly};       side_en = ss[1]; side_two = 1'b0; end
    endcase
  end

  // ------------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------------
  function [7:0] rotl8(input [7:0] v, input [2:0] s);
    reg [15:0] t;
    begin
      t = {v, v} << s;
      rotl8 = t[15:8];
    end
  endfunction

  function [7:0] rotr8(input [7:0] v, input [2:0] s);
    reg [15:0] t;
    begin
      t = {v, v} >> s;
      rotr8 = t[7:0];
    end
  endfunction

  function [31:0] bitrev32(input [31:0] v);
    integer i;
    begin
      for (i = 0; i < 32; i = i + 1) bitrev32[i] = v[31-i];
    end
  endfunction

  wire [7:0]  vin_rot  = rotr8(vin, in_base);
  wire [31:0] cmask    = (cnt == 6'd32) ? 32'hffffffff : ((32'h1 << cnt[4:0]) - 32'h1);
  wire [31:0] crc_mask = (crc_wm1 == 5'd31) ? 32'hffffffff : ((32'h1 << (crc_wm1 + 5'd1)) - 32'h1);

  // ------------------------------------------------------------------
  // Line coder: bit in / bit out decisions
  // ------------------------------------------------------------------
  wire stuff_active = lflags[1] & (stuff_len != 3'd0);
  wire stuff_due    = stuff_active & (run_cnt == stuff_len);
  wire code_nrzi    = lflags[2] & (line_mode == 2'd1);
  wire code_manch   = lflags[2] & (line_mode == 2'd2);

  wire rx_lvl   = vin[in_base] ^ line_inv;
  wire rx_bit   = code_nrzi ? (rx_lvl == nrzi_prev) : rx_lvl;

  // ------------------------------------------------------------------
  // Shared source mux (IN and MOV)
  // ------------------------------------------------------------------
  wire [2:0] src_sel = is_in ? f_hi : f_src;
  reg [31:0] srcv;
  always @* begin
    case (src_sel)
      3'd0: srcv = {24'h0, vin_rot};
      3'd1: srcv = x;
      3'd2: srcv = y;
      3'd3: srcv = 32'h0;
      3'd4: srcv = is_in ? {31'h0, rx_bit} : crc;
      3'd5: srcv = time_i;
      3'd6: srcv = isr;
      default: srcv = osr;
    endcase
  end

  reg [31:0] opd;   // MOV operand after operation
  always @* begin
    case (f_mop)
      2'd0: opd = srcv;
      2'd1: opd = ~srcv;
      2'd2: opd = bitrev32(srcv);
      default: opd = {srcv[7:0], srcv[15:8], srcv[23:16], srcv[31:24]};
    endcase
  end

  // ------------------------------------------------------------------
  // OUT: OSR shift (with on-demand autopull)
  // ------------------------------------------------------------------
  wire out_stuff  = out_line & stuff_due;
  // pull source: the TX FIFO, or instruction memory at dptr (never empty)
  wire [31:0] pull_data   = pull_imem ? {12'h0, dmem_data} : tx_data;
  wire        pull_empty  = ~pull_imem & tx_empty;
  wire refill     = is_out & ~out_stuff & autopull & (osr_cnt >= pull_th);
  wire [31:0] osr_eff     = refill ? pull_data : osr;
  wire [5:0]  osr_cnt_eff = refill ? 6'd0 : osr_cnt;
  wire out_stall  = refill & pull_empty;

  // One 64-bit funnel serves both directions. Left: {0,osr} << n gives the
  // shifted-out MSBs in the high word and the new OSR in the low word.
  // Right: the same on bit-reversed OSR, and the data is just a mask.
  wire [31:0] osr_fin  = out_right ? bitrev32(osr_eff) : osr_eff;
  wire [63:0] osr_fun  = {32'h0, osr_fin} << cnt;
  wire [31:0] out_data = out_right ? (osr_eff & cmask) : osr_fun[63:32];
  wire [31:0] osr_out  = out_right ? bitrev32(osr_fun[31:0]) : osr_fun[31:0];
  wire [6:0]  osr_sum  = {1'b0, osr_cnt_eff} + {1'b0, cnt};
  wire [5:0]  osr_cnt_out = osr_sum[6] | (osr_sum[5:0] > 6'd32) ? 6'd32 : osr_sum[5:0];

  // ------------------------------------------------------------------
  // IN: ISR shift (with autopush)
  // ------------------------------------------------------------------
  wire in_stuff   = in_line & stuff_due;
  // Left : isr' = (isr << n) | src[n-1:0]
  // Right: isr' = rev((rev(isr) << n) | (rev(src) >> (32-n)))
  // Both come out of the high word of one {A,B} << n funnel.
  wire [31:0] in_src  = srcv & cmask;
  wire [63:0] isr_fun = {in_right ? bitrev32(isr) : isr,
                         in_right ? bitrev32(srcv) : 32'h0} << cnt;
  wire [31:0] isr_in  = in_right ? bitrev32(isr_fun[63:32]) : (isr_fun[63:32] | in_src);
  wire [6:0]  isr_sum = {1'b0, isr_cnt} + {1'b0, cnt};
  wire [5:0]  isr_cnt_in = isr_sum[6] | (isr_sum[5:0] > 6'd32) ? 6'd32 : isr_sum[5:0];
  wire in_push    = is_in & ~in_stuff & autopush & (isr_cnt_in >= push_th);
  wire in_stall   = in_push & rx_full;

  // ------------------------------------------------------------------
  // PUSH / PULL
  // ------------------------------------------------------------------
  wire pp_pull   = arg[7];
  wire pp_if     = arg[6];
  wire pp_block  = arg[5];
  wire push_skip = pp_if & (isr_cnt < push_th);
  wire pull_skip = pp_if & (osr_cnt < pull_th);
  wire push_go   = is_pp & ~pp_pull & ~push_skip & ~rx_full;
  wire push_drop = is_pp & ~pp_pull & ~push_skip &  rx_full & ~pp_block;
  wire pull_go   = is_pp &  pp_pull & ~pull_skip & ~pull_empty;
  wire pull_nb   = is_pp &  pp_pull & ~pull_skip &  pull_empty & ~pp_block;
  wire pp_stall  = is_pp & pp_block & (pp_pull ? (~pull_skip & pull_empty)
                                               : (~push_skip & rx_full));

  // ------------------------------------------------------------------
  // JMP condition
  // ------------------------------------------------------------------
  reg cond;
  always @* begin
    case (arg[10:7])
      4'd0:  cond = 1'b1;
      4'd1:  cond = (x == 32'h0);
      4'd2:  cond = (x != 32'h0);
      4'd3:  cond = (y == 32'h0);
      4'd4:  cond = (y != 32'h0);
      4'd5:  cond = (x != y);
      4'd6:  cond = vin[jmp_pin];
      4'd7:  cond = ~vin[jmp_pin];
      4'd8:  cond = (osr_cnt < pull_th);
      4'd9:  cond = ~tx_empty;
      4'd10: cond = to_flag;
      4'd11: cond = stuff_due;
      4'd12: cond = lflags[3];
      4'd13: cond = (x < y);
      4'd14: cond = last_bit;
      default: cond = (isr_cnt >= push_th);
    endcase
  end

  // ------------------------------------------------------------------
  // WAIT
  // ------------------------------------------------------------------
  wire [2:0] w_idx = arg[2:0];
  wire       w_pol = arg[10];
  wire       w_to  = arg[7];
  reg wcond;
  always @* begin
    case (arg[9:8])
      2'd0:    wcond = (vin[w_idx] == w_pol);
      2'd1:    wcond = (irq[w_idx] == w_pol);
      2'd2:    wcond = (vin[w_idx] == w_pol) & (vin_prev[w_idx] != w_pol);
      default: wcond = 1'b1;
    endcase
  end
  wire wait_timeout = is_wait & ~wcond & w_to & (y == 32'h0);
  wire wait_stall   = is_wait & ~wcond & ~wait_timeout;

  // ------------------------------------------------------------------
  // IRQ
  // ------------------------------------------------------------------
  wire [2:0] irq_idx = arg[4] ? {arg[2], arg[1:0] + CORE_ID} : arg[2:0];
  wire irq_clear  = arg[6];
  wire irq_wait   = arg[5] & ~arg[6];
  wire irq_stall  = is_irq & irq_wait & (~irq_armed | irq[irq_idx]);

  // ------------------------------------------------------------------
  // Stall and commit
  // ------------------------------------------------------------------
  wire stall  = (is_out & out_stall) | (is_in & in_stall) | pp_stall |
                wait_stall | irq_stall;
  wire commit = exec & ~stall;

  // ------------------------------------------------------------------
  // X / Y (one adder each: decrement and ADD share it)
  // ------------------------------------------------------------------
  wire [31:0] simm   = {{22{arg[9]}}, arg[9:0]};
  wire        x_dec  = is_jmp & (arg[10:7] == 4'd2);
  wire        y_dec  = (is_jmp & (arg[10:7] == 4'd4)) | (is_wait & wait_stall & w_to);
  wire [31:0] x_sum  = x + (is_add ? simm : 32'hffffffff);
  wire [31:0] y_sum  = y + (is_add ? simm : 32'hffffffff);

  wire x_wr_add = (x_dec | (is_add & ~arg[10]));
  wire x_wr_out = is_out & (f_hi == 3'd1);
  wire x_wr_mov = is_mov & (f_hi == 3'd1);
  wire x_wr_set = is_set & (f_hi == 3'd1);
  wire y_wr_add = (y_dec | (is_add & arg[10]));
  wire y_wr_out = is_out & (f_hi == 3'd2);
  wire y_wr_mov = is_mov & (f_hi == 3'd2);
  wire y_wr_set = is_set & (f_hi == 3'd2);

  // x/y decrement for WAIT happens while stalled, so it is gated by exec,
  // everything else by commit
  wire x_we = commit & (x_wr_add | x_wr_out | x_wr_mov | x_wr_set);
  wire y_we = (commit & (y_wr_add | y_wr_out | y_wr_mov | y_wr_set)) |
              (exec & is_wait & wait_stall & w_to);
  wire [31:0] x_d = x_wr_add ? x_sum : x_wr_out ? out_data : x_wr_mov ? opd : {24'h0, arg[7:0]};
  wire [31:0] y_d = y_wr_add ? y_sum : y_wr_out ? out_data : y_wr_mov ? opd : {24'h0, arg[7:0]};

  // ------------------------------------------------------------------
  // ISR / OSR
  // ------------------------------------------------------------------
  wire isr_wr_in   = is_in & ~in_stuff;
  wire isr_wr_out  = is_out & ~out_stuff & (f_hi == 3'd7);
  wire isr_wr_mov  = is_mov & (f_hi == 3'd6);
  wire isr_clr     = push_go | push_drop | (isr_wr_in & in_push);
  wire isr_we      = commit & (isr_wr_in | isr_wr_out | isr_wr_mov | isr_clr);
  wire [31:0] isr_d     = isr_clr ? 32'h0 : isr_wr_in ? isr_in : isr_wr_out ? out_data : opd;
  wire [5:0]  isr_cnt_d = isr_clr ? 6'd0 : isr_wr_in ? isr_cnt_in : isr_wr_out ? cnt : 6'd0;

  wire osr_wr_out  = is_out & ~out_stuff;
  wire osr_wr_mov  = is_mov & (f_hi == 3'd7);
  wire osr_we      = commit & (osr_wr_out | osr_wr_mov | pull_go | pull_nb);
  wire [31:0] osr_d     = osr_wr_out ? osr_out : pull_go ? pull_data : pull_nb ? x : opd;
  wire [5:0]  osr_cnt_d = osr_wr_out ? osr_cnt_out : 6'd0;

  wire   pulled  = commit & ((osr_wr_out & refill) | pull_go);
  assign tx_pop  = pulled & ~pull_imem;
  wire   dptr_inc = pulled & pull_imem;
  wire   dptr_set = commit & is_set & (f_hi == 3'd4);
  assign rx_push = commit & (push_go | (isr_wr_in & in_push));
  assign rx_data = is_pp ? isr : isr_in;

  // ------------------------------------------------------------------
  // CRC
  // ------------------------------------------------------------------
  wire line_bit_in  = in_line & ~in_stuff;
  wire line_bit_out = out_line & ~out_stuff;
  wire crc_b  = is_in ? rx_bit : out_data[0];
  wire crc_fb = (crc_msb ? crc[crc_wm1] : crc[0]) ^ crc_b;
  wire [31:0] crc_next = ((crc_msb ? (crc << 1) : (crc >> 1)) ^ (crc_fb ? cfg_poly : 32'h0)) & crc_mask;
  wire crc_wr_bit = lflags[0] & (line_bit_in | line_bit_out);
  wire crc_wr_mov = is_mov & (f_hi == 3'd4);
  wire crc_we     = commit & (crc_wr_bit | crc_wr_mov);
  wire [31:0] crc_d = crc_wr_mov ? (opd & crc_mask) : crc_next;

  // ------------------------------------------------------------------
  // Line coder state (stuffing run tracker, NRZI, Manchester)
  // ------------------------------------------------------------------
  wire line_op   = in_line | out_line;
  wire stuff_op  = in_stuff | out_stuff;
  // the bit that enters the run tracker
  wire lbit      = out_line ? (out_stuff ? ~run_val : out_data[0]) : rx_bit;

  reg [2:0] run_cnt_t;
  reg       run_val_t;
  always @* begin
    if (!stuff_any) begin
      run_val_t = 1'b1;
      run_cnt_t = lbit ? ((run_cnt == 3'd7) ? 3'd7 : run_cnt + 3'd1) : 3'd0;
    end else if (lbit == run_val && run_cnt != 3'd0) begin
      run_val_t = run_val;
      run_cnt_t = (run_cnt == 3'd7) ? 3'd7 : run_cnt + 3'd1;
    end else begin
      run_val_t = lbit;
      run_cnt_t = 3'd1;
    end
  end

  // transmit level for the bit being emitted
  wire tx_lvl = code_nrzi ? (lbit ? nrzi_prev : ~nrzi_prev) :
                code_manch ? ~lbit : lbit;

  wire ctl_reset = is_ctl & arg[8];

  // Manchester second-half timer (runs on every tick)
  wire manch_fire = tick & manch_pend & (manch_cnt <= 8'd1);

  // ------------------------------------------------------------------
  // Pin writes: OUT/SET/MOV to PINS or PINDIRS share one rotate path
  // ------------------------------------------------------------------
  wire pin_out = is_out & ~out_stuff & ((f_hi == 3'd0) | (f_hi == 3'd5));
  wire pin_set = is_set & ((f_hi == 3'd0) | (f_hi == 3'd3));
  wire pin_mov = is_mov & ((f_hi == 3'd0) | (f_hi == 3'd3));
  wire pin_dir = (is_out & (f_hi == 3'd5)) | ((is_set | is_mov) & (f_hi == 3'd3));
  wire pin_op  = pin_out | pin_set | pin_mov;

  wire [2:0] pin_base = is_set ? set_base : out_base;
  wire [5:0] pin_cnt  = is_set ? {2'b00, set_count} : is_mov ? 6'd8 : cnt;
  wire [7:0] pin_dat8 = is_set ? arg[7:0] : is_mov ? opd[7:0] : out_data[7:0];
  wire [7:0] pin_msk8 = (pin_cnt >= 6'd8) ? 8'hff : ((8'h1 << pin_cnt[2:0]) - 8'h1);
  wire [7:0] pin_m    = rotl8(pin_msk8, pin_base);
  wire [7:0] pin_d    = rotl8(pin_dat8, pin_base);

  // line driver: out_base (and out_base+1 inverted when differential)
  wire       line_fire = (commit & out_line) | manch_fire;
  wire       line_lvl  = ((commit & out_line) ? tx_lvl : manch_lvl) ^ line_inv;
  wire [7:0] line_m    = rotl8({6'b0, line_diff, 1'b1}, out_base);
  wire [7:0] line_d    = rotl8({6'b0, ~line_lvl, line_lvl}, out_base);

  wire [7:0] side_m    = rotl8({6'b0, side_two, 1'b1}, side_base);
  wire [7:0] side_d    = rotl8({6'b0, ss[1], ss[0]}, side_base);
  wire       side_fire = exec & side_en;

  reg [7:0] n_vout, n_voe;
  always @* begin
    n_vout = vout;
    n_voe  = voe;
    if (line_fire)                n_vout = (n_vout & ~line_m) | (line_d & line_m);
    if (commit & pin_op) begin
      if (pin_dir) n_voe  = (n_voe  & ~pin_m) | (pin_d & pin_m);
      else         n_vout = (n_vout & ~pin_m) | (pin_d & pin_m);
    end
    if (side_fire) begin
      if (side_dirs) n_voe  = (n_voe  & ~side_m) | (side_d & side_m);
      else           n_vout = (n_vout & ~side_m) | (side_d & side_m);
    end
  end

  // ------------------------------------------------------------------
  // Program counter
  // ------------------------------------------------------------------
  wire jump_cond = is_jmp & cond;
  wire jump_out  = is_out & ~out_stuff & (f_hi == 3'd6);
  wire jump_mov  = is_mov & (f_hi == 3'd5);
  wire [6:0] jaddr = is_jmp ? arg[6:0] : jump_out ? out_data[6:0] : opd[6:0];
  wire [6:0] pc_seq = inj_pend ? pc : (pc == wrap_top) ? wrap_bot : pc + 7'd1;
  wire [6:0] pc_d   = (jump_cond | jump_out | jump_mov) ? jaddr : pc_seq;

  // next-state values of pc/dptr (what the registers below will hold),
  // used by the memory's fetch-next registers
  assign pc_nxt_o   = hw_pc ? hw_data[6:0] : commit ? pc_d : pc;
  assign dptr_nxt_o = dptr_set ? arg[6:0] : dptr_inc ? dptr + 7'd1 : dptr;

  // IRQ outputs
  wire irq_do_set = is_irq & ~irq_clear & (~arg[5] | ~irq_armed);
  assign irq_set = (exec & irq_do_set) ? (8'h1 << irq_idx) : 8'h0;
  assign irq_clr = (exec & is_irq & irq_clear) ? (8'h1 << irq_idx) :
                   (commit & is_wait & wcond & (arg[9:8] == 2'd1) & w_pol) ? (8'h1 << w_idx) :
                   8'h0;

  // ------------------------------------------------------------------
  // State registers
  // ------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pc         <= 7'd0;
      dptr       <= 7'd0;
      x          <= 32'h0;
      y          <= 32'h0;
      isr        <= 32'h0;
      osr        <= 32'h0;
      crc        <= 32'h0;
      isr_cnt    <= 6'd0;
      osr_cnt    <= 6'd32;
      delay_cnt  <= 5'd0;
      div_cnt    <= 16'd0;
      vin_prev   <= 8'h0;
      vout       <= 8'h0;
      voe        <= 8'h0;
      to_flag    <= 1'b0;
      irq_armed  <= 1'b0;
      inj_pend   <= 1'b0;
      inj_instr  <= 20'h0;
      brk_skip   <= 1'b0;
      stalled    <= 1'b0;
      lflags     <= 4'h0;
      run_cnt    <= 3'd0;
      run_val    <= 1'b0;
      nrzi_prev  <= 1'b0;
      manch_pend <= 1'b0;
      manch_cnt  <= 8'd0;
      manch_lvl  <= 1'b0;
      last_bit   <= 1'b0;
      stuff_flag <= 1'b0;
    end else begin
      vout <= n_vout;
      voe  <= n_voe;
      if (tick) vin_prev <= vin;

      // clock divider
      if (div_sync || !en)   div_cnt <= 16'd0;
      else if (div_zero)     div_cnt <= cfg_clkdiv;
      else                   div_cnt <= div_cnt - 16'd1;

      if (tick & ~no_delay) delay_cnt <= delay_cnt - 5'd1;

      if (exec) stalled <= stall;
      if (brk_hit) brk_skip <= 1'b1;

      if (dptr_set)      dptr <= arg[6:0];
      else if (dptr_inc) dptr <= dptr + 7'd1;

      if (x_we) x <= x_d;
      if (y_we) y <= y_d;
      if (isr_we) begin isr <= isr_d; isr_cnt <= isr_cnt_d; end
      if (osr_we) begin osr <= osr_d; osr_cnt <= osr_cnt_d; end
      if (crc_we) crc <= crc_d;

      // Manchester timer
      if (tick & manch_pend) begin
        if (manch_cnt <= 8'd1) begin
          manch_pend <= 1'b0;
          manch_cnt  <= 8'd0;
        end else begin
          manch_cnt <= manch_cnt - 8'd1;
        end
      end

      // IRQ set+wait sequencing
      if (exec & is_irq & irq_wait) begin
        if (!irq_armed)          irq_armed <= 1'b1;
        else if (!irq[irq_idx])  irq_armed <= 1'b0;
      end

      // WAIT with timeout
      if (exec & is_wait & w_to & ~wait_stall) to_flag <= wait_timeout;

      if (commit) begin
        pc        <= pc_d;
        delay_cnt <= delay_val;
        brk_skip  <= 1'b0;
        inj_pend  <= 1'b0;

        if (line_op) begin
          stuff_flag <= stuff_op;
          if (stuff_active) begin
            run_cnt <= run_cnt_t;
            run_val <= run_val_t;
          end
          if (!stuff_op) last_bit <= lbit;
          if (in_line) begin
            nrzi_prev <= rx_lvl;
            if (in_stuff && rx_bit == run_val) lflags[3] <= 1'b1;   // stuff violation
          end else if (code_nrzi) begin
            nrzi_prev <= tx_lvl;
          end
          if (out_line) begin
            manch_pend <= code_manch;
            manch_cnt  <= manch_half;
            manch_lvl  <= lbit;
          end
        end

        if (is_ctl) begin
          lflags <= (lflags & ~arg[7:4]) | (arg[3:0] & arg[7:4]);
          if (ctl_reset) begin
            run_cnt    <= 3'd0;
            run_val    <= 1'b0;
            nrzi_prev  <= line_idle;
            manch_pend <= 1'b0;
            stuff_flag <= 1'b0;
          end
        end
      end

      if (restart) begin
        isr        <= 32'h0;
        osr        <= 32'h0;
        isr_cnt    <= 6'd0;
        osr_cnt    <= 6'd32;
        delay_cnt  <= 5'd0;
        div_cnt    <= 16'd0;
        to_flag    <= 1'b0;
        irq_armed  <= 1'b0;
        inj_pend   <= 1'b0;
        brk_skip   <= 1'b0;
        stalled    <= 1'b0;
        lflags     <= 4'h0;
        run_cnt    <= 3'd0;
        run_val    <= 1'b0;
        nrzi_prev  <= line_idle;
        manch_pend <= 1'b0;
        stuff_flag <= 1'b0;
      end

      // host writes win over execution
      if (hw_pc)    pc  <= hw_data[6:0];
      if (hw_x)     x   <= hw_data;
      if (hw_y)     y   <= hw_data;
      if (hw_crc)   crc <= hw_data;
      if (hw_isr)   begin isr <= hw_data; isr_cnt <= 6'd0; end
      if (hw_osr)   begin osr <= hw_data; osr_cnt <= 6'd0; end
      if (hw_instr) begin inj_instr <= hw_data[19:0]; inj_pend <= 1'b1; end
    end
  end

  assign status_o = {2'b00, isr_cnt, 2'b00, osr_cnt,
                     lflags[2], lflags[1], lflags[0], brk_skip,
                     last_bit, stuff_flag, lflags[3], to_flag,
                     stalled, pc};

  wire _unused = &{1'b0, is_ctl};

endmodule

`default_nettype wire
