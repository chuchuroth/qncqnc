# ModbusRTU with NeuraSync - Project Structure

## Overview
This project provides Modbus RTU communication for DH grippers with NeuraSync DDS integration for real-time state publishing.

## Directory Structure

```
qncqnc/
├── build/                      # Compiled binaries
│   ├── gripper_control         # Main gripper control with DDS publishing
│   ├── gripper_dashboard       # CLI subscriber for monitoring
│   └── modbus_rtu_example      # Basic Modbus RTU example
│
├── examples/                   # Source code examples
│   ├── gripper_control.cpp     # Gripper control + DDS publisher
│   ├── gripper_dashboard.cpp   # CLI dashboard subscriber
│   └── modbus_rtu_example.cpp  # Basic Modbus example
│
├── include/                    # Header files
│   └── modbus_rtu.h           # Header-only Modbus RTU library
│
├── idl/                        # IDL message definitions
│   ├── ModbusRTU.idl          # General Modbus RTU messages
│   └── GripperState.idl       # Custom gripper state message
│
├── neura-sync/                 # NeuraSync DDS library (source)
│   ├── neura_sync/            # Core library
│   ├── neura_sync_core_msgs/  # Message definitions (including GripperState)
│   └── build/                 # Built libraries
│
├── neura_sync_idl_tools/       # IDL code generation tools
│   └── build/                 # Built tools
│
├── schemas/                    # JSON device schemas
├── descriptors/                # Device descriptor files
│
├── scripts/                    # Utility scripts
│   ├── scan_gripper.sh        # Scan for gripper on serial ports
│   └── test_gripper_neurasync.sh  # Test gripper with DDS
│
├── docs/                       # Documentation
│   ├── NEURASYNC_BUILD_COMPLETE.md  # Build completion summary
│   └── PROJECT_STRUCTURE.md    # This file
│
├── CMakeLists.txt             # Build configuration
├── Dockerfile                 # Docker build configuration
├── docker-compose.yml         # Docker compose setup
└── readme.md                  # Main README

```

## Built Components

### 1. Gripper Control (`./build/gripper_control`)
- Controls DH gripper via Modbus RTU
- Publishes state to DDS topic: `qnc/modbus/device_state`
- Message type: `GripperState::DeviceState`
- Usage: `./build/gripper_control /dev/ttyUSB0 1`

### 2. CLI Dashboard (`./build/gripper_dashboard`)
- Subscribes to gripper DDS topic
- Displays real-time state updates
- Usage: `./build/gripper_dashboard`

### 3. Modbus RTU Example (`./build/modbus_rtu_example`)
- Basic Modbus RTU communication example
- No DDS integration
- Usage: `./build/modbus_rtu_example /dev/ttyUSB0 1`

## Key Files

### Source Files
- `examples/gripper_control.cpp` - Gripper control with NeuraSync publishing
- `examples/gripper_dashboard.cpp` - CLI subscriber for monitoring
- `include/modbus_rtu.h` - Header-only Modbus RTU library

### Message Definitions
- `idl/GripperState.idl` - Custom gripper state message
- `neura-sync/neura_sync_core_msgs/idl/` - All NeuraSync message types

### Build System
- `CMakeLists.txt` - Main build configuration
- `build/` - All compiled binaries and build artifacts

## Quick Start

### Terminal 1 - Start Dashboard
```bash
cd ~/Documents/qncqnc
./build/gripper_dashboard
```

### Terminal 2 - Control Gripper
```bash
cd ~/Documents/qncqnc
./build/gripper_control /dev/ttyUSB0 1
```

## DDS Topic Information

**Topic**: `qnc/modbus/device_state`
**Type**: `GripperState::DeviceState`

**Message Fields**:
- `device_id` - Device identifier
- `timestamp_sec/nsec` - Timestamp
- `init_state` - Initialization state (0-2)
- `gripper_state` - Gripper state (0-3)
- `position` - Current position (0-1000)
- `force` - Force percentage (0-100)
- `speed` - Speed percentage (0-100)
- `online` - Device online status

## Build Information

Built with:
- NeuraSync library (custom build with GripperState support)
- Fast-DDS 3.x
- CMake 3.16+
- C++17

For build details, see `docs/NEURASYNC_BUILD_COMPLETE.md`
