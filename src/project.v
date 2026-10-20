/*
 * Copyright (c) 2026 Arka Khorchidian
 * SPDX-License-Identifier: Apache-2.0
 *
 * Polyglot: a programmable protocol-emulator ASIC for Tiny Tapeout.
 *
 * Four cycle-exact bit-banging cores share a 128-word instruction memory,
 * a pin crossbar and a set of IRQ flags. A host loads programs and moves
 * data through an SPI slave port. See docs/info.md and docs/ISA.md.
 *
 * Pinout
 *   ui[0]  host SCK        uo[0]  host MISO
 *   ui[1]  host CS_n       uo[1]  host IRQ
 *   ui[2]  host MOSI       uo[7:2] core outputs (physical pins 18-23)
 *   ui[7:3] core inputs     uio[7:0] core bidirectional pins (physical 0-7)
 */

`default_nettype none

module tt_um_arkakhorchidian_polyglot (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  // ------------------------------------------------------------------
  // Input synchronisers
  // ------------------------------------------------------------------
  reg [15:0] sync1, sync2;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sync1 <= 16'h0200;   // CS_n idles high
      sync2 <= 16'h0200;
    end else begin
      sync1 <= {ui_in, uio_in};
      sync2 <= sync1;
    end
  end
  wire [7:0] uio_s = sync2[7:0];
  wire [7:0] ui_s  = sync2[15:8];

  // ------------------------------------------------------------------
  // Host interface
  // ------------------------------------------------------------------
  wire        h_wr, h_rd;
  wire [7:0]  h_waddr, h_addr;
  wire [31:0] h_wdata;
  reg  [31:0] h_rdata;
  wire        miso;

  pg_host_spi u_spi (
      .clk(clk), .rst_n(rst_n),
      .sck(ui_s[0]), .cs_n(ui_s[1]), .mosi(ui_s[2]), .miso(miso),
      .wr_stb(h_wr), .wr_addr(h_waddr), .addr(h_addr), .wdata(h_wdata),
      .rd_stb(h_rd), .rdata(h_rdata)
  );

  wire        wr_imem = h_wr & ~h_waddr[7];
  wire        wr_core = h_wr & (h_waddr[7:6] == 2'b10);
  wire        wr_glob = h_wr & (h_waddr[7:4] == 4'hC);
  wire        wr_fifo = h_wr & (h_waddr[7:2] == 6'b110100);
  wire [1:0]  wc      = h_waddr[5:4];
  wire [3:0]  wr_reg  = h_waddr[3:0];

  // ------------------------------------------------------------------
  // Global registers
  // ------------------------------------------------------------------
  reg  [3:0]  core_en;
  reg  [3:0]  halted;
  reg  [15:0] irq_mask;
  reg  [7:0]  bankmap;
  reg  [7:0]  irq_flags;
  reg  [31:0] timer;
  reg  [3:0]  restart_p, step_p, divsync_p, fclr_p;
  wire [3:0]  brk_hit;

  wire glob_ctrl = wr_glob & (h_waddr[3:0] == 4'h1);
  wire glob_irq  = wr_glob & (h_waddr[3:0] == 4'h2);
  wire glob_time = wr_glob & (h_waddr[3:0] == 4'h4);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      core_en   <= 4'h0;
      halted    <= 4'h0;
      irq_mask  <= 16'h0;
      bankmap   <= 8'hE4;
      restart_p <= 4'h0;
      step_p    <= 4'h0;
      divsync_p <= 4'h0;
      fclr_p    <= 4'h0;
    end else begin
      restart_p <= 4'h0;
      step_p    <= 4'h0;
      divsync_p <= 4'h0;
      fclr_p    <= 4'h0;
      core_en   <= core_en & ~brk_hit;
      halted    <= halted | brk_hit;
      if (glob_ctrl) begin
        core_en   <= h_wdata[3:0];
        halted    <= halted & ~h_wdata[3:0];
        restart_p <= h_wdata[7:4];
        step_p    <= h_wdata[11:8];
        divsync_p <= h_wdata[15:12];
        fclr_p    <= h_wdata[19:16];
      end
      if (wr_glob && h_waddr[3:0] == 4'h3) irq_mask <= h_wdata[15:0];
      if (wr_glob && h_waddr[3:0] == 4'h6) bankmap  <= h_wdata[7:0];
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) timer <= 32'h0;
    else if (glob_time) timer <= h_wdata;
    else timer <= timer + 32'h1;
  end

  // ------------------------------------------------------------------
  // Per-core configuration registers
  // ------------------------------------------------------------------
  reg [15:0] cfg_clkdiv [0:3];
  reg [31:0] cfg_exec   [0:3];
  reg [13:0] cfg_shift  [0:3];
  reg [11:0] cfg_pin    [0:3];
  reg [19:0] cfg_pmlo   [0:3];
  reg [19:0] cfg_pmhi   [0:3];
  reg [23:0] cfg_line   [0:3];
  reg [31:0] cfg_poly   [0:3];

  integer i;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (i = 0; i < 4; i = i + 1) begin
        cfg_clkdiv[i] <= 16'h0;
        cfg_exec[i]   <= 32'h00001F00;   // wrap 0..31
        cfg_shift[i]  <= 14'h000C;       // shift right (LSB first), thresholds 32
        cfg_pin[i]    <= 12'h0;
        cfg_pmlo[i]   <= 20'h0;
        cfg_pmhi[i]   <= 20'h0;
        cfg_line[i]   <= 24'h0;
        cfg_poly[i]   <= 32'h0;
      end
    end else if (wr_core) begin
      case (wr_reg)
        4'h0: cfg_clkdiv[wc] <= h_wdata[15:0];
        4'h1: cfg_exec[wc]   <= h_wdata;
        4'h2: cfg_shift[wc]  <= h_wdata[13:0];
        4'h3: cfg_pin[wc]    <= h_wdata[11:0];
        4'h4: cfg_pmlo[wc]   <= h_wdata[19:0];
        4'h5: cfg_pmhi[wc]   <= h_wdata[19:0];
        4'h6: cfg_line[wc]   <= h_wdata[23:0];
        4'h7: cfg_poly[wc]   <= h_wdata;
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------------
  // Instruction memory
  // ------------------------------------------------------------------
  wire [27:0] pcs;
  wire [79:0] imem_q;

  pg_imem u_imem (
      .clk(clk), .rst_n(rst_n), .we(wr_imem), .waddr(h_waddr[6:0]), .wdata(h_wdata[19:0]),
      .bankmap(bankmap), .pcs(pcs), .rdata(imem_q)
  );

  // ------------------------------------------------------------------
  // Cores, FIFOs, pin crossbar
  // ------------------------------------------------------------------
  wire [31:0] phys_in;
  wire [7:0]  nets;
  wire [7:0]  core_vout [0:3];
  wire [7:0]  core_voe  [0:3];
  wire [7:0]  core_iset [0:3];
  wire [7:0]  core_iclr [0:3];
  wire [31:0] c_x [0:3], c_y [0:3], c_isr [0:3], c_osr [0:3], c_crc [0:3], c_stat [0:3];
  wire [19:0] c_instr [0:3];
  wire [31:0] tx_head [0:3], rx_head [0:3];
  wire [3:0]  tx_empty, tx_full, rx_empty, rx_full;
  wire [2:0]  tx_lvl [0:3], rx_lvl [0:3];
  wire [39:0] pmap [0:3];

  genvar c, k;
  generate
    for (c = 0; c < 4; c = c + 1) begin : g_core
      wire [7:0]  vin;
      wire        tx_pop, rx_push;
      wire [31:0] rx_din;
      wire        hsel = wr_core & (wc == c);

      assign pmap[c] = {cfg_pmhi[c], cfg_pmlo[c]};
      for (k = 0; k < 8; k = k + 1) begin : g_vin
        assign vin[k] = phys_in[pmap[c][5*k +: 5]];
      end

      pg_fifo_lat #(.W(32)) u_txf (
          .clk(clk), .rst_n(rst_n), .clear(fclr_p[c]),
          .push(wr_fifo & (h_waddr[1:0] == c)), .din(h_wdata),
          .pop(tx_pop), .dout(tx_head[c]),
          .empty(tx_empty[c]), .full(tx_full[c]), .level(tx_lvl[c])
      );

      pg_fifo #(.W(32), .DEPTH(4), .AW(2)) u_rxf (
          .clk(clk), .rst_n(rst_n), .clear(fclr_p[c]),
          .push(rx_push), .din(rx_din),
          .pop(h_rd & (h_addr[7:2] == 6'b110100) & (h_addr[1:0] == c)),
          .dout(rx_head[c]),
          .empty(rx_empty[c]), .full(rx_full[c]), .level(rx_lvl[c])
      );

      pg_core #(.CORE_ID(c)) u_core (
          .clk(clk), .rst_n(rst_n),
          .en(core_en[c]), .step(step_p[c]), .restart(restart_p[c]),
          .div_sync(divsync_p[c]), .brk_hit(brk_hit[c]),
          .cfg_clkdiv(cfg_clkdiv[c]), .cfg_exec(cfg_exec[c]),
          .cfg_shift(cfg_shift[c]), .cfg_pin(cfg_pin[c]),
          .cfg_line(cfg_line[c]), .cfg_poly(cfg_poly[c]),
          .pc_o(pcs[7*c +: 7]), .imem_data(imem_q[20*c +: 20]),
          .hw_pc(hsel & (wr_reg == 4'h9)), .hw_x(hsel & (wr_reg == 4'hA)),
          .hw_y(hsel & (wr_reg == 4'hB)), .hw_crc(hsel & (wr_reg == 4'h8)),
          .hw_isr(hsel & (wr_reg == 4'hC)), .hw_osr(hsel & (wr_reg == 4'hD)),
          .hw_instr(hsel & (wr_reg == 4'hF)), .hw_data(h_wdata),
          .vin(vin), .vout(core_vout[c]), .voe(core_voe[c]),
          .tx_empty(tx_empty[c]), .tx_data(tx_head[c]), .tx_pop(tx_pop),
          .rx_full(rx_full[c]), .rx_push(rx_push), .rx_data(rx_din),
          .irq(irq_flags), .irq_set(core_iset[c]), .irq_clr(core_iclr[c]),
          .time_i(timer),
          .x_o(c_x[c]), .y_o(c_y[c]), .isr_o(c_isr[c]), .osr_o(c_osr[c]),
          .crc_o(c_crc[c]), .status_o(c_stat[c]), .instr_o(c_instr[c])
      );
    end
  endgenerate

  // Crossbar: every (core, vpin) with its output enable set drives the
  // physical pin it is mapped to. Multiple drivers are wired-OR.
  wire [31:0] drv_oe, drv_val;
  generate
    genvar p;
    for (p = 0; p < 32; p = p + 1) begin : g_phys
      wire [31:0] hit_oe, hit_val;
      for (c = 0; c < 4; c = c + 1) begin : g_c
        for (k = 0; k < 8; k = k + 1) begin : g_k
          wire sel = (pmap[c][5*k +: 5] == p) & core_voe[c][k];
          assign hit_oe[8*c + k]  = sel;
          assign hit_val[8*c + k] = sel & core_vout[c][k];
        end
      end
      assign drv_oe[p]  = |hit_oe;
      assign drv_val[p] = |hit_val;
    end
  endgenerate

  wire irq_pin = |(irq_flags & irq_mask[7:0]) |
                 |(~rx_empty & irq_mask[11:8]) |
                 |(~tx_full & irq_mask[15:12]);

  assign uio_out = drv_val[7:0];
  assign uio_oe  = drv_oe[7:0];
  assign uo_out  = {drv_val[23:18], irq_pin, miso};
  assign nets    = drv_val[31:24];
  assign phys_in = {nets, uo_out, ui_s, uio_s};

  // ------------------------------------------------------------------
  // IRQ flags
  // ------------------------------------------------------------------
  wire [7:0] all_set = core_iset[0] | core_iset[1] | core_iset[2] | core_iset[3] |
                       (glob_irq ? h_wdata[15:8] : 8'h0);
  wire [7:0] all_clr = core_iclr[0] | core_iclr[1] | core_iclr[2] | core_iclr[3] |
                       (glob_irq ? h_wdata[7:0] : 8'h0);
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) irq_flags <= 8'h0;
    else        irq_flags <= (irq_flags & ~all_clr) | all_set;
  end

  // ------------------------------------------------------------------
  // Host read mux
  // ------------------------------------------------------------------
  wire [1:0] rc = h_addr[5:4];
  wire [31:0] fifostat;
  generate
    for (c = 0; c < 4; c = c + 1) begin : g_fs
      assign fifostat[8*c +: 8] = {c_stat[c][7], halted[c], rx_lvl[c], tx_lvl[c]};
    end
  endgenerate

  always @* begin
    h_rdata = 32'h0;
    if (h_addr[7:6] == 2'b10) begin
      case (h_addr[3:0])
        // 0x0-0x7 (configuration) are write-only
        4'h8: h_rdata = c_crc[rc];
        4'h9: h_rdata = {25'h0, pcs[7*rc +: 7]};
        4'hA: h_rdata = c_x[rc];
        4'hB: h_rdata = c_y[rc];
        4'hC: h_rdata = c_isr[rc];
        4'hD: h_rdata = c_osr[rc];
        4'hE: h_rdata = c_stat[rc];
        default: h_rdata = {12'h0, c_instr[rc]};
      endcase
    end else if (h_addr[7:4] == 4'hC) begin
      case (h_addr[3:0])
        4'h0: h_rdata = 32'h504C5947;                     // "PLYG"
        4'h1: h_rdata = {12'h0, 4'h0, 4'h0, halted, 4'h0, core_en};
        4'h2: h_rdata = {24'h0, irq_flags};
        4'h3: h_rdata = {16'h0, irq_mask};
        4'h4: h_rdata = timer;
        4'h5: h_rdata = fifostat;
        4'h6: h_rdata = {24'h0, bankmap};
        4'h7: h_rdata = phys_in;
        default: h_rdata = 32'h0;
      endcase
    end else if (h_addr[7:2] == 6'b110100) begin
      h_rdata = rx_empty[h_addr[1:0]] ? 32'h0 : rx_head[h_addr[1:0]];
    end
  end

  wire _unused = &{ena, ui_s[2:0], tx_full, 1'b0};

endmodule

`default_nettype wire
