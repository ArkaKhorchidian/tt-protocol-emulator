#!/bin/sh
# k-induction proofs of the FIFO invariants with Yosys' built-in SAT solver.
set -e
cd "$(dirname "$0")"
prove() {   # $1 = top, $2 = params
  yosys -q -l $1.log -p "
    read_verilog -formal ../src/pg_fifo.v ../src/pg_fifo_lat.v stubs.v
    chparam $2 $1
    prep -top $1
    memory
    flatten
    async2sync
    sat -tempinduct -prove-asserts -set-init-zero -maxsteps 8 -verify" > /dev/null
  echo "$1: proved ($(grep -o 'Induction step proven: SUCCESS' $1.log | head -1))"
}
prove pg_fifo "-set W 4 -set DEPTH 4 -set AW 2"
prove pg_fifo_lat "-set W 4"
