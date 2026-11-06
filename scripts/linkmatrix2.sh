#!/bin/bash
# Square-return flights in strict rotation. base = single queue plus one link. Each of the first variants changes one thing from base.
# flight none with exit 0 means the test passed without seeing a landing: not a valid pass.
B=build/snap-link2
F=~/sih-work/fly.sh
C="PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1"
A="PX4_SIM_LINK_PACE=host PX4_SIM_LINK_RATE=host PX4_SIM_LINK_TIMEOUTS=host PX4_SIH_API_RATE=1000000"

round() { # speed
  s=$1
  $F "m$s-base" $B "$s" $C
  $F "m$s-loop" $B "$s" $C PX4_SIM_LINK_PACE=host
  $F "m$s-deadline" $B "$s" $C PX4_SIM_LINK_TIMEOUTS=host
  $F "m$s-rate4M" $B "$s" $C PX4_SIM_LINK_RATE=host
  $F "m$s-rate1M" $B "$s" $C PX4_SIM_LINK_RATE=host PX4_SIH_API_RATE=1000000
  $F "m$s-all1M" $B "$s" $C $A
  $F "m$s-default" $B "$s" PX4_DUMMY=1
  $F "m$s-defaultall" $B "$s" $A
  $F "m$s-inlineall" $B "$s" PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1 $A
}

for r in 1 2 3 4 5; do round 1000; round 100; done
echo FAST DONE
for r in 1 2 3; do
  $F m10-base $B 10 $C
  $F m10-all1M $B 10 $C $A
  $F m10-default $B 10 PX4_DUMMY=1
  $F m10-defaultall $B 10 $A
done
echo TEN DONE
$F m1-default $B 1 PX4_DUMMY=1
$F m1-defaultall $B 1 $A
$F m0.5-default $B 0.5 PX4_DUMMY=1
$F m0.5-defaultall $B 0.5 $A
echo LINK MATRIX DONE
