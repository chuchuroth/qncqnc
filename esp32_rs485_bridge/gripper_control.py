#!/usr/bin/env python3
"""
DH Gripper Control via Modbus RTU
Sends commands through ESP32 RS485 bridge

Register Map (from DH AG-95 descriptor):
  Control (Holding Registers - Function 0x06):
    0x0100 (256): init_command (0x01=init, 0xA5=full init)
    0x0101 (257): force_percent (20-100%)
    0x0102 (258): speed_percent (1-100%)
    0x0103 (259): target_position (0=open, 1000=closed)

  Status (Input Registers - Function 0x04):
    0x0200 (512): init_state (0=not init, 1=initializing, 2=initialized)
    0x0201 (513): gripper_state (0=moving, 1=reached, 2=caught, 3=dropped)
    0x0202 (514): current_position (0-1000)
"""

import serial
import struct
import time
import sys

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
TIMEOUT = 0.5

# Modbus RTU settings
SLAVE_ID = 1  # Default DH gripper address

# DH Gripper register addresses (from descriptor)
REG_INIT = 0x0100          # 256 - Initialization register
REG_FORCE = 0x0101         # 257 - Force percentage (20-100)
REG_SPEED = 0x0102         # 258 - Speed percentage (1-100)
REG_POSITION = 0x0103      # 259 - Target position (0-1000)

# Status registers (input registers)
REG_INIT_STATE = 0x0200    # 512 - Init state
REG_GRIPPER_STATE = 0x0201 # 513 - Gripper state
REG_ACTUAL_POS = 0x0202    # 514 - Actual position

# Modbus function codes
FUNC_READ_INPUT = 0x04      # Read input registers (for status)
FUNC_READ_HOLDING = 0x03    # Read holding registers
FUNC_WRITE_SINGLE = 0x06    # Write single register


def crc16_modbus(data):
    """Calculate Modbus CRC16"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_write_single_register(slave_id, register, value):
    """Build Modbus RTU write single register command (function 0x06)"""
    frame = struct.pack('>BBHH', slave_id, FUNC_WRITE_SINGLE, register, value)
    crc = crc16_modbus(frame)
    frame += struct.pack('<H', crc)  # CRC is little-endian
    return frame


def build_read_input_registers(slave_id, start_register, count):
    """Build Modbus RTU read input registers command (function 0x04)"""
    frame = struct.pack('>BBHH', slave_id, FUNC_READ_INPUT, start_register, count)
    crc = crc16_modbus(frame)
    frame += struct.pack('<H', crc)
    return frame


def build_read_holding_registers(slave_id, start_register, count):
    """Build Modbus RTU read holding registers command (function 0x03)"""
    frame = struct.pack('>BBHH', slave_id, FUNC_READ_HOLDING, start_register, count)
    crc = crc16_modbus(frame)
    frame += struct.pack('<H', crc)
    return frame


def send_command(ser, command, description=""):
    """Send command and receive response"""
    print(f"\n>>> {description}")
    print(f"    TX: {command.hex(' ')}")

    ser.reset_input_buffer()
    ser.write(command)
    ser.flush()

    # Wait for the ESP32 to process and forward the response
    time.sleep(0.4)

    response = ser.read(ser.in_waiting) if ser.in_waiting else b''
    if response:
        # Filter out any ASCII text (boot noise / debug output)
        # Valid Modbus RTU responses start with the slave ID byte
        expected_slave = command[0]
        if len(response) >= 5 and response[0] == expected_slave:
            print(f"    RX: {response.hex(' ')}")
            return response
        else:
            # Not a valid Modbus response - likely noise
            try:
                text = response.decode('ascii', errors='ignore').strip()
                if text:
                    print(f"    RX: (non-Modbus data: {text[:60]})")
            except:
                print(f"    RX: (invalid data, {len(response)} bytes)")
            return None
    else:
        print("    RX: No response (timeout)")
        return None


def parse_read_response(response):
    """Parse a read registers response and return the values"""
    if not response or len(response) < 5:
        return None
    # Check for error response
    if response[1] & 0x80:
        print(f"    Error: Exception code {response[2]}")
        return None
    byte_count = response[2]
    if len(response) < 3 + byte_count + 2:  # header + data + CRC
        print(f"    Warning: Short response ({len(response)} bytes)")
        return None
    values = []
    for i in range(byte_count // 2):
        val = struct.unpack('>H', response[3 + i*2:5 + i*2])[0]
        values.append(val)
    return values


def initialize_gripper(ser, full_init=True):
    """Initialize the gripper"""
    value = 0xA5 if full_init else 0x01  # 0xA5 = full init with calibration
    cmd = build_write_single_register(SLAVE_ID, REG_INIT, value)
    return send_command(ser, cmd, f"Initialize gripper ({'full' if full_init else 'quick'})")


def set_force(ser, force):
    """Set gripper force (20-100%)"""
    force = max(20, min(100, force))
    cmd = build_write_single_register(SLAVE_ID, REG_FORCE, force)
    return send_command(ser, cmd, f"Set force to {force}%")


def set_speed(ser, speed):
    """Set gripper speed (1-100%)"""
    speed = max(1, min(100, speed))
    cmd = build_write_single_register(SLAVE_ID, REG_SPEED, speed)
    return send_command(ser, cmd, f"Set speed to {speed}%")


def set_position(ser, position):
    """Set gripper position (0=open, 1000=closed)"""
    position = max(0, min(1000, position))
    cmd = build_write_single_register(SLAVE_ID, REG_POSITION, position)
    return send_command(ser, cmd, f"Set position to {position}")


def read_init_state(ser):
    """Read gripper init state"""
    cmd = build_read_input_registers(SLAVE_ID, REG_INIT_STATE, 1)
    resp = send_command(ser, cmd, "Read init state")
    values = parse_read_response(resp)
    if values:
        states = {0: "not initialized", 1: "initializing", 2: "initialized"}
        print(f"    Init state: {values[0]} ({states.get(values[0], 'unknown')})")
    return values


def read_gripper_state(ser):
    """Read gripper operational state"""
    cmd = build_read_input_registers(SLAVE_ID, REG_GRIPPER_STATE, 1)
    resp = send_command(ser, cmd, "Read gripper state")
    values = parse_read_response(resp)
    if values:
        states = {0: "moving", 1: "position reached", 2: "object caught", 3: "object dropped"}
        print(f"    Gripper state: {values[0]} ({states.get(values[0], 'unknown')})")
    return values


def read_position(ser):
    """Read actual gripper position"""
    cmd = build_read_input_registers(SLAVE_ID, REG_ACTUAL_POS, 1)
    resp = send_command(ser, cmd, "Read actual position")
    values = parse_read_response(resp)
    if values:
        print(f"    Position: {values[0]}/1000 ({values[0]/10:.1f}%)")
    return values


def read_all_status(ser):
    """Read all status registers at once"""
    cmd = build_read_input_registers(SLAVE_ID, REG_INIT_STATE, 3)
    resp = send_command(ser, cmd, "Read all status")
    values = parse_read_response(resp)
    if values and len(values) >= 3:
        init_states = {0: "not initialized", 1: "initializing", 2: "initialized"}
        grip_states = {0: "moving", 1: "position reached", 2: "object caught", 3: "object dropped"}
        print(f"    Init state: {values[0]} ({init_states.get(values[0], 'unknown')})")
        print(f"    Gripper state: {values[1]} ({grip_states.get(values[1], 'unknown')})")
        print(f"    Position: {values[2]}/1000")
    return values


def open_gripper(ser):
    """Fully open the gripper"""
    return set_position(ser, 0)


def close_gripper(ser):
    """Fully close the gripper"""
    return set_position(ser, 1000)


def wait_for_boot(ser, timeout=10):
    """Wait for ESP32 READY marker, draining boot noise"""
    print("Waiting for ESP32 READY signal...")

    # Reset ESP32
    ser.setDTR(False)
    ser.setRTS(True)
    time.sleep(0.1)
    ser.setRTS(False)

    start = time.time()
    buf = b''
    while time.time() - start < timeout:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            # Check for READY marker in accumulated data
            if b'READY' in buf:
                # Drain any trailing boot noise thoroughly
                time.sleep(0.5)
                ser.reset_input_buffer()
                time.sleep(0.5)
                if ser.in_waiting:
                    ser.read(ser.in_waiting)
                ser.reset_input_buffer()
                print("ESP32 ready.\n")
                return True
        time.sleep(0.05)

    print("WARNING: READY signal not received, proceeding anyway...")
    ser.reset_input_buffer()
    return False


def main():
    print("=" * 50)
    print("DH Gripper Control via Modbus RTU")
    print("=" * 50)
    print(f"Port: {SERIAL_PORT}")
    print(f"Baud: {BAUD_RATE}")
    print(f"Slave ID: {SLAVE_ID}")

    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        )
        print(f"\nConnected to {SERIAL_PORT}")

        # Wait for ESP32 to boot and clear all startup noise
        wait_for_boot(ser)

        # Read initial status
        print("\n" + "-" * 50)
        print("Reading initial status...")
        read_all_status(ser)

        # Initialize gripper with full calibration
        print("\n" + "-" * 50)
        print("Initializing gripper (full calibration)...")
        initialize_gripper(ser, full_init=True)
        print("Waiting for initialization...")
        time.sleep(3)
        read_all_status(ser)

        # Set speed and force
        print("\n" + "-" * 50)
        print("Setting speed and force...")
        set_speed(ser, 50)
        time.sleep(0.1)
        set_force(ser, 50)
        time.sleep(0.1)

        # Open gripper
        print("\n" + "-" * 50)
        print("Opening gripper...")
        open_gripper(ser)
        time.sleep(2)
        read_all_status(ser)

        # Close gripper
        print("\n" + "-" * 50)
        print("Closing gripper...")
        close_gripper(ser)
        time.sleep(2)
        read_all_status(ser)

        # Final status
        print("\n" + "-" * 50)
        print("Final status:")
        read_all_status(ser)

        print("\n" + "=" * 50)
        print("Gripper control sequence complete!")
        print("=" * 50)

        ser.close()

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(0)


if __name__ == "__main__":
    main()
