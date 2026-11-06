#!/bin/bash
# Square-return flights in strict rotation: timeout effects and data-rate effects measured separately, across speeds.
# base = single queue plus one link, the setup most exposed to the link problems. Each variant changes one thing from base.
B=build/snap-pace
F=~/sih-work/fly.sh
C="PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1"
H="PX4_SIM_LINK_PACE=host PX4_SIM_LINK_TIMEOUTS=host"

round() { # speed
  s=$1
  $F "l$s-base" $B "$s" $C
  $F "l$s-T-host" $B "$s" $C PX4_SIM_LINK_TIMEOUTS=host
  $F "l$s-T-scaled" $B "$s" $C PX4_SIM_LINK_TIMEOUTS=scaled
  $F "l$s-R-10k" $B "$s" $C PX4_SIH_API_RATE=10000
  $F "l$s-P-host" $B "$s" $C PX4_SIM_LINK_PACE=host
  $F "l$s-PT-host" $B "$s" $C $H
  $F "l$s-default" $B "$s" PX4_DUMMY=1
  $F "l$s-default-PT" $B "$s" $H
  $F "l$s-inline-PT" $B "$s" PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1 $H
}

for r in 1 2 3 4 5; do round 1000; round 100; done
echo FAST DONE
for r in 1 2 3; do
  $F l10-base $B 10 $C
  $F l10-PT-host $B 10 $C $H
  $F l10-default $B 10 PX4_DUMMY=1
  $F l10-default-PT $B 10 $H
done
echo TEN DONE
$F l1-default $B 1 PX4_DUMMY=1
$F l1-default-PT $B 1 $H
$F l0.5-default $B 0.5 PX4_DUMMY=1
$F l0.5-default-PT $B 0.5 $H
echo LINK MATRIX DONE
