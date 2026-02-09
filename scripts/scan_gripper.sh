#!/bin/bash
# Scan for DH Gripper on different slave IDs and baudrates

echo "=========================================="
echo "  DH Gripper Scanner"
echo "=========================================="
echo ""
echo "Testing /dev/ttyUSB0 with different configurations..."
echo ""

PORT="/dev/ttyUSB0"
BUILD_DIR="/home/pi/Documents/qncqnc/build"

if [ ! -e "$BUILD_DIR/modbus_rtu_example" ]; then
    echo "Error: modbus_rtu_example not found in $BUILD_DIR"
    exit 1
fi

# Test common baudrates and slave IDs
BAUDRATES=(115200 9600 19200 38400)
SLAVE_IDS=(1 2 3 247)

echo "Scanning combinations..."
echo ""

for BAUD in "${BAUDRATES[@]}"; do
    for SLAVE in "${SLAVE_IDS[@]}"; do
        echo -n "Testing: Baud=$BAUD, Slave=$SLAVE ... "

        # Run modbus_rtu_example with timeout
        timeout 2s "$BUILD_DIR/modbus_rtu_example" "$PORT" "$SLAVE" "$BAUD" 2>&1 | grep -q "Connected" && {
            echo "✓ FOUND!"
            echo ""
            echo "=========================================="
            echo "  Gripper Found!"
            echo "=========================================="
            echo "Port: $PORT"
            echo "Baudrate: $BAUD"
            echo "Slave ID: $SLAVE"
            echo ""
            echo "Run gripper control with:"
            echo "  cd $BUILD_DIR"
            echo "  ./gripper_control $PORT $SLAVE $BAUD"
            exit 0
        }

        echo "✗ No response"
    done
done

echo ""
echo "=========================================="
echo "  No gripper found"
echo "=========================================="
echo ""
echo "Troubleshooting:"
echo "1. Check power to gripper"
echo "2. Check RS485 wiring (A/B connections)"
echo "3. Verify /dev/ttyUSB0 is the correct device"
echo "4. Check if RS485 adapter needs drivers"
echo ""
