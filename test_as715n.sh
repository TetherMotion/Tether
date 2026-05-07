#!/bin/bash
set -euo pipefail
# This script builds the runec helper and tests it by running the AS715N EtherCAT master stack in a container.
./build.sh
runec build/bin/as715n_sine_motion_native