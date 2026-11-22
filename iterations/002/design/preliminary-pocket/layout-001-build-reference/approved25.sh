#!/bin/bash
set -euo pipefail
C=$1
B=/tmp/appr25-$C; rm -rf $B; mkdir -p $B; cd $B
git clone -q /mnt/c approved
git -C approved -c advice.detachedHead=false checkout -q $C
git clone -q /mnt/cWORKSTATION/repos/agent-frc builder
git -C builder -c advice.detachedHead=false checkout -q 92ddf41933dd6026b26493c1d2c91505e9d15914
git clone -q /mnt/cWORKSTATION/repos/agent-frc verifier
git -C verifier -c advice.detachedHead=false checkout -q c69e23c
node builder/scripts/robot-highlights.mjs approved/highlights.json "$B/out" > /dev/null
node approved/scripts/publish-capture.mjs approved "$B/out" > /dev/null
echo "$C $(node verifier/scripts/local-showcase-release.mjs digest $B/out)"
