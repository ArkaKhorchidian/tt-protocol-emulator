/*
 * Copyright (c) 2026 Arka Khorchidian
 * SPDX-License-Identifier: Apache-2.0
 *
 * Small synchronous FIFO with show-ahead read (head is always visible).
 * Push when full and pop when empty are ignored.
 */

`default_nettype none

module pg_fifo #(
    parameter W     = 32,
    parameter DEPTH = 4,          // must be a power of two
    parameter AW    = 2
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
    output wire [AW:0]  level
);

  reg [W-1:0] mem [0:DEPTH-1];
  reg [AW-1:0] rp, wp;
  reg [AW:0]   cnt;

  assign dout  = mem[rp];
  assign empty = (cnt == 0);
  assign full  = (cnt == DEPTH);
  assign level = cnt;

  wire do_push = push & ~full;
  wire do_pop  = pop & ~empty;

  always @(posedge clk) begin
    if (do_push) mem[wp] <= din;
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rp  <= 0;
      wp  <= 0;
      cnt <= 0;
    end else if (clear) begin
      rp  <= 0;
      wp  <= 0;
      cnt <= 0;
    end else begin
      if (do_push) wp <= wp + 1'b1;
      if (do_pop)  rp <= rp + 1'b1;
      cnt <= cnt + {{AW{1'b0}}, do_push} - {{AW{1'b0}}, do_pop};
    end
  end

`ifdef FORMAL
  // bookkeeping invariants (formal/run.sh proves them by k-induction)
  always @* begin
    if (rst_n) begin
      assert (cnt <= DEPTH);
      assert (wp == rp + cnt[AW-1:0]);
    end
  end
`endif

endmodule

`default_nettype wire
