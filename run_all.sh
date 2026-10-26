#!/bin/bash

# 1. Run the entire load loop with RTS/CTS Disabled
echo "Executing batch WITHOUT RTS/CTS..."
./ns3 run "scratch/wifi-ht-project --enableRtsCts=false"

# 2. Run the entire load loop with RTS/CTS Enabled
echo "Executing batch WITH RTS/CTS..."
./ns3 run "scratch/wifi-ht-project --enableRtsCts=true"

echo "All simulations completed successfully! Data saved in output/."
