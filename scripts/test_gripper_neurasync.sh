#!/bin/bash
# Test DH Gripper with NeuraSync DDS Publishing

DEVICE="${1:-/dev/ttyUSB0}"
SLAVE_ID="${2:-1}"

echo "=== DH Gripper with NeuraSync DDS Test ==="
echo "Device: $DEVICE"
echo "Slave ID: $SLAVE_ID"
echo ""
echo "This will:"
echo "  1. Initialize the gripper"
echo "  2. Publish gripper state to NeuraSync topic: qnc/modbus/device_state"
echo "  3. Run interactive control (commands: o=open, c=close, q=quit)"
echo ""
echo "In another terminal, run: ./build/gripper_dashboard"
echo "to see the published state in real-time."
echo ""
read -p "Press Enter to start or Ctrl+C to cancel..."

./build/gripper_control "$DEVICE" "$SLAVE_ID"
