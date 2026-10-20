// storage cells are irrelevant to the bookkeeping proofs
module pg_icg (input wire clk, input wire en, output wire gclk); assign gclk = 1'b0; endmodule
module pg_latch (input wire g, input wire d, output wire q); assign q = 1'b0; endmodule
