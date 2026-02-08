# ModbusRTU Protocol Library

A header-only C++ library for ModbusRTU communication with gripper devices, featuring NeuraSync DDS integration for real-time state publishing.

## Features

- Header-only ModbusRTU library (`include/modbus_rtu.h`)
- DH Gripper control examples
- NeuraSync integration for publishing gripper state to dashboard
- Docker support with FastDDS 3.x

## Directory Structure

```
modbus_rtu_protocol/
├── include/           # Header-only library
│   └── modbus_rtu.h
├── examples/          # Example applications
│   ├── gripper_control.cpp      # Gripper control + NeuraSync publisher
│   ├── gripper_dashboard.cpp    # CLI subscriber for gripper state
│   └── modbus_rtu_example.cpp   # Basic ModbusRTU example
├── idl/               # IDL type definitions
├── schemas/           # JSON device schemas
├── descriptors/       # Device descriptor files
├── scripts/           # Build scripts
├── Dockerfile         # Docker build with NeuraSync
├── CMakeLists.txt
└── docker-compose.yml
```

## Quick Start with Docker

Build and run with NeuraSync support:

```bash
# Build Docker image
docker build -t modbus-rtu-protocol:neurasync .

# Run gripper control (publisher)
docker run --rm -it --device=/dev/ttyUSB0 --network=host \
  modbus-rtu-protocol:neurasync gripper_control /dev/ttyUSB0 1 115200

# Run dashboard subscriber (in another terminal)
docker run --rm -it --network=host \
  modbus-rtu-protocol:neurasync gripper_dashboard
```

## Deploying to Other Machines

### Option 1: Export/Import Docker Image (Offline)

On the build machine:
```bash
# Build the image
docker build -t modbus-rtu-protocol:neurasync .

# Export to a tar file
docker save modbus-rtu-protocol:neurasync -o modbus-rtu-protocol.tar

# Compress for transfer (optional)
gzip modbus-rtu-protocol.tar
```

On the target machine:
```bash
# Copy the tar file to target machine, then load it
docker load -i modbus-rtu-protocol.tar
# Or if compressed:
gunzip -c modbus-rtu-protocol.tar.gz | docker load

# Verify the image is loaded
docker images | grep modbus-rtu-protocol

# Run the application
docker run --rm -it --device=/dev/ttyUSB0 --network=host \
  modbus-rtu-protocol:neurasync gripper_control /dev/ttyUSB0 1 115200
```

### Option 2: Docker Registry (Online)

Push to Docker Hub or private registry:
```bash
# Tag for your registry
docker tag modbus-rtu-protocol:neurasync your-registry/modbus-rtu-protocol:neurasync

# Push to registry
docker push your-registry/modbus-rtu-protocol:neurasync
```

On target machine:
```bash
# Pull from registry
docker pull your-registry/modbus-rtu-protocol:neurasync

# Run
docker run --rm -it --device=/dev/ttyUSB0 --network=host \
  your-registry/modbus-rtu-protocol:neurasync gripper_control /dev/ttyUSB0 1 115200
```

### Option 3: Docker Compose

Copy the project to the target machine and use docker-compose:
```bash
# Start gripper control and dashboard
docker-compose up -d gripper-control gripper-dashboard

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

## Target Machine Requirements

For running the Docker container on other machines:

- **Docker Engine** 20.10 or later
- **RS485 USB adapter** (e.g., `/dev/ttyUSB0`) for gripper communication
- **Network access** for NeuraSync DDS communication (uses UDP multicast)
- **Linux x86_64** (the image is built for this architecture)

Ensure the user has permissions for serial devices:
```bash
# Add user to dialout group for serial port access
sudo usermod -aG dialout $USER
# Logout/login for changes to take effect
```

## Building from Source

### Dependencies

For NeuraSync support, you need:
- FastDDS 3.x
- FastCDR 2.x
- neura-sync library
- neura_sync_idl_tools

### Build Steps

```bash
# Clone dependencies (if not using Docker)
# Copy neura-sync and neura_sync_idl_tools to this directory

mkdir build && cd build
cmake .. -DBUILD_WITH_NEURASYNC=ON -DENABLE_LOGGING=ON
make -j$(nproc)
```

### CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_EXAMPLES` | ON | Build example applications |
| `BUILD_WITH_DDS` | OFF | Build with raw FastDDS support |
| `BUILD_WITH_NEURASYNC` | OFF | Build with NeuraSync library |
| `ENABLE_LOGGING` | OFF | Enable debug logging |

## NeuraSync Integration

When built with `BUILD_WITH_NEURASYNC=ON`, the gripper_control publishes state to:

- **Topic:** `qnc/modbus/device_state`
- **Type:** `DeviceState`

The neurasync-dashboard can subscribe and visualize the data using the `QNC_ModbusRTU` connection profile.

### DeviceState Fields

| Field | Type | Description |
|-------|------|-------------|
| device_id | string | Gripper identifier |
| timestamp_sec | uint32 | Timestamp seconds |
| timestamp_nsec | uint32 | Timestamp nanoseconds |
| init_state | uint16 | 0=not_init, 1=initializing, 2=ready |
| gripper_state | uint16 | 0=in_motion, 1=pos_reached, 2=caught, 3=dropped |
| position | uint16 | Raw position (0-1000) |
| force | uint16 | Force percentage |
| speed | uint16 | Speed percentage |
| online | bool | Connection status |

## Usage Examples

### Basic ModbusRTU

```cpp
#include "modbus_rtu.h"

modbus_rtu mb("/dev/ttyUSB0", 115200, 8, 1, 'N');
mb.set_slave_id(1);
mb.connect();

uint16_t position;
mb.read_input_registers(514, 1, &position);
mb.write_register(259, 500);  // Set target position

mb.close();
```

### Gripper Control

```bash
# With Docker
docker run --rm -it --device=/dev/ttyUSB0 --network=host \
  modbus-rtu-protocol:neurasync gripper_control /dev/ttyUSB0 1 115200 gripper_01

# Arguments: <serial_port> <slave_id> <baudrate> [device_id]
```

## License

Copyright (C) 2025 NeuraSync
