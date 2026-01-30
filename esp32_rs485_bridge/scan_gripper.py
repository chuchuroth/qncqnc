#!/usr/bin/env python3
"""
Scan for Modbus RTU devices on RS485 bus
Tries different slave addresses and baud rates
"""

import serial
import struct
import time
import sys

SERIAL_PORT = '/dev/ttyUSB0'


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


def build_read_input_registers(slave_id, start_register, count):
    """Build Modbus RTU read input registers command"""
    frame = struct.pack('>BBHH', slave_id, 0x04, start_register, count)
    crc = crc16_modbus(frame)
    frame += struct.pack('<H', crc)
    return frame


def scan_address(ser, slave_id, timeout=0.1):
    """Try to read from a specific slave address"""
    cmd = build_read_input_registers(slave_id, 0x0200, 1)

    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()

    time.sleep(timeout)

    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        return response
    return None


def main():
    print("Modbus RTU Device Scanner")
    print("=" * 50)

    # Common baud rates for industrial devices
    baud_rates = [115200, 9600, 19200, 38400, 57600]

    for baud in baud_rates:
        print(f"\n>>> Scanning at {baud} baud...")

        try:
            ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.15
            )

            time.sleep(0.3)
            ser.reset_input_buffer()

            found = False
            # Scan addresses 1-10 (most common range)
            for addr in range(1, 11):
                response = scan_address(ser, addr)
                if response:
                    print(f"    Found device at address {addr}!")
                    print(f"    Response: {response.hex(' ')}")
                    found = True

            # Also try address 0 (broadcast - some devices respond)
            response = scan_address(ser, 0)
            if response:
                print(f"    Broadcast response: {response.hex(' ')}")
                found = True

            if not found:
                print("    No devices found")

            ser.close()

        except serial.SerialException as e:
            print(f"    Error: {e}")

    print("\n" + "=" * 50)
    print("\nIf no devices found, please check:")
    print("  1. RS485 wiring (A+/B- connections)")
    print("  2. Gripper power supply")
    print("  3. DE/RE pin control on RS485 transceiver")
    print("  4. Termination resistor if cable is long")


if __name__ == "__main__":
    main()
