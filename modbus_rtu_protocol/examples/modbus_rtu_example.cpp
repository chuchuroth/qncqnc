/**
 * ModbusRTU Protocol Example
 *
 * Demonstrates usage of the modbus_rtu header-only library
 * for communicating with ModbusRTU devices.
 *
 * Copyright (C) 2025 NeuraSync
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <csignal>

#ifndef ENABLE_MODBUS_RTU_LOGGING
#define ENABLE_MODBUS_RTU_LOGGING
#endif
#include "modbus_rtu.h"

// Simple JSON parsing helper (minimal, for device descriptor)
#include <map>
#include <vector>

static volatile bool g_running = true;

void signal_handler(int signum) {
    (void)signum;
    g_running = false;
}

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " <serial_port> [slave_id] [baudrate]\n"
              << "\n"
              << "Arguments:\n"
              << "  serial_port  - Serial port device (e.g., /dev/ttyUSB0)\n"
              << "  slave_id     - Modbus slave ID (default: 1)\n"
              << "  baudrate     - Baud rate (default: 115200)\n"
              << "\n"
              << "Examples:\n"
              << "  " << prog << " /dev/ttyUSB0\n"
              << "  " << prog << " /dev/ttyUSB0 1 115200\n"
              << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    // Parse arguments
    std::string port = argv[1];
    uint8_t slave_id = (argc > 2) ? std::stoi(argv[2]) : 1;
    uint32_t baudrate = (argc > 3) ? std::stoul(argv[3]) : 115200;

    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::cout << "ModbusRTU Protocol Example\n"
              << "==========================\n"
              << "Port: " << port << "\n"
              << "Slave ID: " << (int)slave_id << "\n"
              << "Baudrate: " << baudrate << "\n"
              << std::endl;

    // Create Modbus RTU connection
    modbus_rtu mb(port, baudrate, 8, 1, 'N');
    mb.set_slave_id(slave_id);
    mb.set_response_timeout_ms(500);
    mb.set_inter_frame_delay_ms(50);

    // Connect
    std::cout << "Connecting to device...\n";
    if (!mb.connect()) {
        std::cerr << "ERROR: Failed to connect: " << mb.error_msg << "\n";
        return 1;
    }
    std::cout << "Connected successfully!\n\n";

    // Example: Read input registers (device status)
    // For DH AG-95: addresses 512-514 contain init_state, gripper_state, current_position
    std::cout << "Reading input registers (512-514)...\n";
    uint16_t input_regs[3] = {0};
    int ret = mb.read_input_registers(512, 3, input_regs);
    if (ret == RTU_SUCCESS) {
        std::cout << "  Register 512 (init_state):       " << input_regs[0] << "\n";
        std::cout << "  Register 513 (gripper_state):    " << input_regs[1] << "\n";
        std::cout << "  Register 514 (current_position): " << input_regs[2] << "\n";

        // Compute position in mm (for AG-95 gripper)
        float position_mm = (1000.0f - input_regs[2]) * 0.095f;
        std::cout << "  Computed position: " << position_mm << " mm\n";
    } else {
        std::cerr << "  ERROR: " << mb.error_msg << "\n";
    }
    std::cout << "\n";

    // Example: Read holding registers (command registers)
    // For DH AG-95: addresses 256-259 contain commands
    std::cout << "Reading holding registers (256-259)...\n";
    uint16_t holding_regs[4] = {0};
    ret = mb.read_holding_registers(256, 4, holding_regs);
    if (ret == RTU_SUCCESS) {
        std::cout << "  Register 256 (init_command):    " << holding_regs[0] << "\n";
        std::cout << "  Register 257 (force_percent):   " << holding_regs[1] << "%\n";
        std::cout << "  Register 258 (speed_percent):   " << holding_regs[2] << "%\n";
        std::cout << "  Register 259 (target_position): " << holding_regs[3] << " permille\n";
    } else {
        std::cerr << "  ERROR: " << mb.error_msg << "\n";
    }
    std::cout << "\n";

    // Continuous polling loop
    std::cout << "Starting continuous polling (Ctrl+C to stop)...\n\n";
    uint32_t poll_count = 0;

    while (g_running) {
        // Read status registers
        ret = mb.read_input_registers(512, 3, input_regs);
        if (ret == RTU_SUCCESS) {
            poll_count++;

            // Interpret state values
            const char* init_state_str = "unknown";
            switch (input_regs[0]) {
                case 0: init_state_str = "not_initialized"; break;
                case 1: init_state_str = "initializing"; break;
                case 2: init_state_str = "initialized"; break;
            }

            const char* gripper_state_str = "unknown";
            switch (input_regs[1]) {
                case 0: gripper_state_str = "in_motion"; break;
                case 1: gripper_state_str = "position_reached"; break;
                case 2: gripper_state_str = "object_caught"; break;
                case 3: gripper_state_str = "object_dropped"; break;
            }

            float position_mm = (1000.0f - input_regs[2]) * 0.095f;

            std::cout << "\r[Poll " << poll_count << "] "
                      << "Init: " << init_state_str << " | "
                      << "State: " << gripper_state_str << " | "
                      << "Position: " << position_mm << " mm   " << std::flush;
        } else {
            std::cerr << "\rERROR: " << mb.error_msg << "                    " << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n\nShutting down...\n";
    mb.close();
    std::cout << "Done.\n";

    return 0;
}
