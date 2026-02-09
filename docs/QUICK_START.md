# Quick Start Guide - NeuraSync Gripper Control

## What You Have

✅ **DH Gripper Control with NeuraSync DDS Integration**

Your system can:
- Control DH grippers via Modbus RTU
- Publish gripper state in real-time over DDS
- Monitor gripper state with CLI dashboard
- Compatible with any DDS subscriber on the network

## Test the System (2 Terminals)

### Terminal 1 - Start Dashboard
```bash
cd ~/Documents/qncqnc
./build/gripper_dashboard
```

This will show real-time gripper state:
- Device ID
- Initialization state
- Gripper state (IN_MOTION, POS_REACHED, OBJ_CAUGHT, OBJ_DROPPED)
- Position (0-1000)
- Force percentage
- Speed percentage
- Online status

### Terminal 2 - Control Gripper
```bash
cd ~/Documents/qncqnc
./build/gripper_control /dev/ttyUSB0 1
```

**Interactive Commands:**
- `o` - Open gripper
- `c` - Close gripper
- `q` - Quit

## DDS Topic Details

**Published Topic:** `qnc/modbus/device_state`
**Message Type:** `GripperState::DeviceState`

Any DDS subscriber can listen to this topic and receive real-time gripper updates.

## Files You Need

```
build/gripper_control      # Main application
build/gripper_dashboard    # Monitoring dashboard
scripts/scan_gripper.sh    # Find gripper serial port
scripts/test_gripper_neurasync.sh  # Automated test script
```

## Common Commands

### Find Gripper Port
```bash
./scripts/scan_gripper.sh
```

### Run Automated Test
```bash
./scripts/test_gripper_neurasync.sh
```

### Custom Port/Slave ID
```bash
./build/gripper_control /dev/ttyUSB1 2  # Port: ttyUSB1, Slave ID: 2
```

## Troubleshooting

### Gripper Not Found
1. Check USB connection: `ls -la /dev/ttyUSB*`
2. Verify baud rate (default: 115200)
3. Run scan script: `./scripts/scan_gripper.sh`

### No DDS Messages
1. Check if dashboard shows "Waiting for data..."
2. Verify gripper_control is publishing
3. Check DDS domain ID (default: 0)

### Permission Denied on /dev/ttyUSB0
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

## Next Steps

1. Test with your gripper
2. Integrate with your robot control system
3. Subscribe to the DDS topic from other applications
4. Build custom applications using the ModbusRTU library

For more details, see:
- `docs/PROJECT_STRUCTURE.md` - Complete project layout
- `docs/NEURASYNC_BUILD_COMPLETE.md` - Build information
- `readme.md` - Full documentation
