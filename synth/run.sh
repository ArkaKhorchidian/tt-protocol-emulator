#!/bin/sh
# Quick area estimate: synthesise the design against the IHP SG13CMOS5L
# standard-cell liberty (downloaded on first use) and print the area.
set -e
cd "$(dirname "$0")/.."
PDK=synth/pdk
LIB=$PDK/sg13cmos5l_stdcell_typ_1p20V_25C.lib
if [ ! -f "$LIB" ]; then
  mkdir -p $PDK
  curl -sfL -o "$LIB" https://raw.githubusercontent.com/IHP-GmbH/IHP-Open-PDK/dev/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_typ_1p20V_25C.lib
fi
sed "s#../pdk/#$PDK/#g" synth/synth.ys > synth/.synth_local.ys
yosys -q -l synth/synth.log synth/.synth_local.ys > /dev/null
grep "Chip area for top" synth/synth.log
