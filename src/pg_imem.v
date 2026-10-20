/*
 * Copyright (c) 2026 Arka Khorchidian
 * SPDX-License-Identifier: Apache-2.0
 *
 * Instruction memory: 4 banks x 32 words x 20 bits, one read port per bank.
 *
 * Each bank's read port is owned by one core (BANKMAP). A core fetches from
 * bank pc[6:5]; a core that owns several banks can run a program of up to
 * 128 instructions, while four cores each owning one bank run in parallel.
 *
 * Storage is latch based (about half the area of flops plus hold muxes).
 * The write strobe is registered, and one cycle later the addressed word's
 * integrated clock gate emits a single high-phase pulse that opens its
 * latches. waddr/wdata must stay stable for two cycles after `we`; they
 * come straight from the host SPI word register, which changes only once
 * every 32 SCK edges.
 *
 * Storage is write-only from the host's side; contents can be read back
 * through a halted core's INSTR register.
 *
 * Define PG_BEHAVIORAL for RTL simulation without the PDK cell models.
 */

`default_nettype none

module pg_imem (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        we,
    input  wire [6:0]  waddr,
    input  wire [19:0] wdata,
    input  wire [7:0]  bankmap,      // owner core of bank b = bankmap[2b+1:2b]
    input  wire [27:0] pcs,          // {pc3, pc2, pc1, pc0}
    output wire [79:0] rdata         // {instr3, instr2, instr1, instr0}
);

  reg wpend;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) wpend <= 1'b0;
    else        wpend <= we;
  end

  wire [19:0] q [0:127];

  genvar w, bt, b, c;
  generate
    for (w = 0; w < 128; w = w + 1) begin : g_word
      wire gclk;
      pg_icg u_icg (.clk(clk), .en(wpend & (waddr == w)), .gclk(gclk));
      for (bt = 0; bt < 20; bt = bt + 1) begin : g_bit
        pg_latch u_lat (.g(gclk), .d(wdata[bt]), .q(q[w][bt]));
      end
    end

    // per-bank read port
    for (b = 0; b < 4; b = b + 1) begin : g_bank
      wire [1:0]  owner = bankmap[2*b+1:2*b];
      wire [6:0]  opc   = pcs[7*owner +: 7];
      wire [19:0] bq    = q[b*32 + opc[4:0]];
    end
    for (c = 0; c < 4; c = c + 1) begin : g_core
      wire [6:0] cpc = pcs[7*c +: 7];
      assign rdata[20*c +: 20] = (cpc[6:5] == 2'd0) ? g_bank[0].bq :
                                 (cpc[6:5] == 2'd1) ? g_bank[1].bq :
                                 (cpc[6:5] == 2'd2) ? g_bank[2].bq :
                                                      g_bank[3].bq;
    end
  endgenerate

endmodule

// Integrated clock gate: gclk = clk & (en sampled while clk is low).
module pg_icg (
    input  wire clk,
    input  wire en,
    output wire gclk
);
`ifdef PG_BEHAVIORAL
  reg en_l;
  always @(clk or en) if (!clk) en_l = en;
  assign gclk = clk & en_l;
`else
  sg13cmos5l_lgcp_1 u_cg (.CLK(clk), .GATE(en), .GCLK(gclk));
`endif
endmodule

// Positive-level transparent latch.
module pg_latch (
    input  wire g,
    input  wire d,
    output wire q
);
`ifdef PG_BEHAVIORAL
  reg r;
  always @(g or d) if (g) r = d;
  assign q = r;
`else
  sg13cmos5l_dlhq_1 u_l (.GATE(g), .D(d), .Q(q));
`endif
endmodule

`default_nettype wire
