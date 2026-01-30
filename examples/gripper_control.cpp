/**
 * DH Gripper Control Example
 * Simple commands to initialize, open, and close the gripper
 */

#include <iostream>
#include <chrono>
#include <thread>

#ifndef ENABLE_MODBUS_RTU_LOGGING
#define ENABLE_MODBUS_RTU_LOGGING
#endif
#include "modbus_rtu.h"

// DH Gripper register addresses
const uint16_t REG_INIT_COMMAND = 256;
const uint16_t REG_FORCE_PERCENT = 257;
const uint16_t REG_SPEED_PERCENT = 258;
const uint16_t REG_TARGET_POSITION = 259;

const uint16_t REG_INIT_STATE = 512;
const uint16_t REG_GRIPPER_STATE = 513;
const uint16_t REG_CURRENT_POSITION = 514;

void print_status(modbus_rtu& mb) {
    uint16_t regs[3] = {0};
    int ret = mb.read_input_registers(REG_INIT_STATE, 3, regs);
    if (ret == RTU_SUCCESS) {
        const char* init_str[] = {"not_initialized", "initializing", "initialized"};
        const char* state_str[] = {"in_motion", "position_reached", "object_caught", "object_dropped"};

        std::cout << "  Init: " << (regs[0] < 3 ? init_str[regs[0]] : "unknown")
                  << " | State: " << (regs[1] < 4 ? state_str[regs[1]] : "unknown")
                  << " | Position: " << ((1000.0f - regs[2]) * 0.095f) << " mm\n";
    } else {
        std::cerr << "  Failed to read status: " << mb.error_msg << "\n";
    }
}

bool wait_for_motion_complete(modbus_rtu& mb, int timeout_ms = 5000) {
    auto start = std::chrono::steady_clock::now();
    while (true) {
        uint16_t state = 0;
        int ret = mb.read_input_registers(REG_GRIPPER_STATE, 1, &state);
        if (ret == RTU_SUCCESS && state != 0) {  // 0 = in_motion
            return true;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char* argv[]) {
    std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    uint8_t slave_id = (argc > 2) ? std::stoi(argv[2]) : 1;
    uint32_t baudrate = (argc > 3) ? std::stoul(argv[3]) : 115200;

    std::cout << "DH Gripper Control\n";
    std::cout << "==================\n";
    std::cout << "Port: " << port << ", Slave: " << (int)slave_id << ", Baud: " << baudrate << "\n\n";

    modbus_rtu mb(port, baudrate, 8, 1, 'N');
    mb.set_slave_id(slave_id);
    mb.set_response_timeout_ms(2000);  // Longer timeout
    mb.set_inter_frame_delay_ms(100);  // Longer delay between frames

    if (!mb.connect()) {
        std::cerr << "Failed to connect: " << mb.error_msg << "\n";
        return 1;
    }
    std::cout << "Connected!\n\n";

    // Read initial status
    std::cout << "Initial status:\n";
    print_status(mb);

    // Step 1: Initialize the gripper
    std::cout << "\n[1] Initializing gripper...\n";
    int ret = mb.write_register(REG_INIT_COMMAND, 1);
    if (ret != RTU_SUCCESS) {
        std::cerr << "  Failed to send init command: " << mb.error_msg << "\n";
    } else {
        std::cout << "  Init command sent\n";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        print_status(mb);
    }

    // Step 2: Set force and speed
    std::cout << "\n[2] Setting force=50%, speed=50%...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mb.write_register(REG_FORCE_PERCENT, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mb.write_register(REG_SPEED_PERCENT, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "  Parameters set\n";

    // Step 3: Open gripper (position = 0)
    std::cout << "\n[3] Opening gripper...\n";
    ret = mb.write_register(REG_TARGET_POSITION, 0);
    if (ret != RTU_SUCCESS) {
        std::cerr << "  Failed: " << mb.error_msg << "\n";
    } else {
        std::cout << "  Command sent, waiting...\n";
        wait_for_motion_complete(mb);
        print_status(mb);
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Step 4: Close gripper (position = 1000)
    std::cout << "\n[4] Closing gripper...\n";
    ret = mb.write_register(REG_TARGET_POSITION, 1000);
    if (ret != RTU_SUCCESS) {
        std::cerr << "  Failed: " << mb.error_msg << "\n";
    } else {
        std::cout << "  Command sent, waiting...\n";
        wait_for_motion_complete(mb);
        print_status(mb);
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Step 5: Open again
    std::cout << "\n[5] Opening gripper again...\n";
    ret = mb.write_register(REG_TARGET_POSITION, 0);
    if (ret != RTU_SUCCESS) {
        std::cerr << "  Failed: " << mb.error_msg << "\n";
    } else {
        std::cout << "  Command sent, waiting...\n";
        wait_for_motion_complete(mb);
        print_status(mb);
    }

    std::cout << "\nDone!\n";
    mb.close();
    return 0;
}
