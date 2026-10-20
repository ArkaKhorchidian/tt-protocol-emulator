/*
 * Copyright (c) 2026 Arka Khorchidian
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-to-core FIFO with latch storage.
 *
 * `din` must be held stable from the push strobe until two cycles later;
 * the host SPI word register guarantees this (it changes only once every
 * 32 SCK edges). A push is registered, then the target entry's clock gate
 * pulses once while `din` is stable. The entry only becomes visible
 * (level/empty) after its latches have been written.
 */

`default_nettype none

module pg_fifo_lat #(
    parameter W = 32
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         clear,
    input  wire         push,
    input  wire [W-1:0] din,
    input  wire         pop,
    output wire [W-1:0] dout,
    output wire         empty,
    output wire         full,
    output wire [2:0]   level
);

  reg [1:0] rp, wp, wsel;
  reg [2:0] cnt;
  reg       wpend;

  // a push in flight counts against capacity immediately
  assign full  = (cnt + {2'b0, wpend}) >= 3'd4;
  assign empty = (cnt == 3'd0);
  assign level = cnt;

  wire do_push = push & ~full;
  wire do_pop  = pop & ~empty;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rp    <= 2'd0;
      wp    <= 2'd0;
      wsel  <= 2'd0;
      cnt   <= 3'd0;
      wpend <= 1'b0;
    end else if (clear) begin
      rp    <= 2'd0;
      wp    <= 2'd0;
      cnt   <= 3'd0;
      wpend <= 1'b0;
    end else begin
      wpend <= do_push;
      if (do_push) begin
        wsel <= wp;
        wp   <= wp + 2'd1;
      end
      if (do_pop) rp <= rp + 2'd1;
      cnt <= cnt + {2'b0, wpend} - {2'b0, do_pop};
    end
  end

  wire [W-1:0] q [0:3];
  genvar e, b;
  generate
    for (e = 0; e < 4; e = e + 1) begin : g_ent
      wire gclk;
      pg_icg u_icg (.clk(clk), .en(wpend & (wsel == e)), .gclk(gclk));
      for (b = 0; b < W; b = b + 1) begin : g_bit
        pg_latch u_lat (.g(gclk), .d(din[b]), .q(q[e][b]));
      end
    end
  endgenerate

  assign dout = q[rp];

endmodule

`default_nettype wire
