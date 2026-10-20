/*
 * Copyright (c) 2026 Arka Khorchidian
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host SPI slave (mode 0, MSB first), oversampled in the system clock domain.
 * Inputs must already be synchronised. Keep SCK <= clk/8.
 *
 *   write: CS_n=0, 0x02, ADDR, {D31..D0}*  (one register write per 32 bits)
 *   read : CS_n=0, 0x03, ADDR, dummy byte, {D31..D0}*
 *
 * The address auto-increments after every word, except for the FIFO ports
 * 0xD0-0xD3, which stay put so a burst streams through the FIFO.
 * A read word is committed (FIFO popped) on its first SCK rising edge, so
 * the speculative pre-load that happens after the last word never pops.
 */

`default_nettype none

module pg_host_spi (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sck,
    input  wire        cs_n,
    input  wire        mosi,
    output wire        miso,

    output reg         wr_stb,
    output reg  [7:0]  wr_addr,
    output reg  [7:0]  addr,       // current / read address
    output reg  [31:0] wdata,
    output reg         rd_stb,     // word at `addr` consumed by host
    input  wire [31:0] rdata       // combinational read of `addr`
);

  localparam PH_CMD = 3'd0, PH_ADDR = 3'd1, PH_DUMMY = 3'd2, PH_DATA = 3'd3,
             PH_IGNORE = 3'd4;

  reg        sck_q;
  reg [2:0]  phase;
  reg [4:0]  bitcnt;
  reg [31:0] sin;
  reg [31:0] sout;
  reg        is_read;

  wire sck_rise = sck & ~sck_q;
  wire sck_fall = ~sck & sck_q;
  wire [31:0] sin_n = {sin[30:0], mosi};
  wire fifo_port = (addr[7:2] == 6'b110100);

  assign miso = sout[31];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sck_q   <= 1'b0;
      phase   <= PH_CMD;
      bitcnt  <= 5'd0;
      sin     <= 32'h0;
      sout    <= 32'h0;
      is_read <= 1'b0;
      addr    <= 8'h0;
      wdata   <= 32'h0;
      wr_addr <= 8'h0;
      wr_stb  <= 1'b0;
      rd_stb  <= 1'b0;
    end else begin
      sck_q  <= sck;
      wr_stb <= 1'b0;
      rd_stb <= 1'b0;

      if (cs_n) begin
        phase  <= PH_CMD;
        bitcnt <= 5'd0;
        sout   <= 32'h0;
      end else begin
        if (sck_rise) begin
          sin    <= sin_n;
          bitcnt <= bitcnt + 5'd1;
          case (phase)
            PH_CMD: if (bitcnt == 5'd7) begin
              bitcnt  <= 5'd0;
              is_read <= (sin_n[7:0] == 8'h03);
              phase   <= (sin_n[7:0] == 8'h02 || sin_n[7:0] == 8'h03) ? PH_ADDR : PH_IGNORE;
            end
            PH_ADDR: if (bitcnt == 5'd7) begin
              bitcnt <= 5'd0;
              addr   <= sin_n[7:0];
              phase  <= is_read ? PH_DUMMY : PH_DATA;
            end
            PH_DUMMY: if (bitcnt == 5'd7) begin
              bitcnt <= 5'd0;
              phase  <= PH_DATA;
            end
            PH_DATA: begin
              if (is_read && bitcnt == 5'd0) rd_stb <= 1'b1;
              if (bitcnt == 5'd31) begin
                if (!is_read) begin
                  wr_stb  <= 1'b1;
                  wr_addr <= addr;
                  wdata   <= sin_n;
                end
                addr <= addr + (fifo_port ? 8'd0 : 8'd1);
              end
            end
            default: ;
          endcase
        end

        if (sck_fall) begin
          if (phase == PH_DATA && is_read && bitcnt == 5'd0)
            sout <= rdata;
          else
            sout <= {sout[30:0], 1'b0};
        end
      end
    end
  end

endmodule

`default_nettype wire
