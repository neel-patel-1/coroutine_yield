#!/bin/bash

make clean
make -j

./single-request-latency | tee timeline.log
./baseline-ctx-switch | tee baseline-ctx.log
./accel_notify_context_switch | tee accel_notify_context_switch.log