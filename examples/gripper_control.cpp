/**
 * DH Gripper Control Example with NeuraSync Publisher
 * Publishes gripper state to NeuraSync Dashboard using DeviceState message
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <memory>

#ifndef ENABLE_MODBUS_RTU_LOGGING
#define ENABLE_MODBUS_RTU_LOGGING
#endif
#include "modbus_rtu.h"

#ifdef BUILD_WITH_NEURASYNC
#include <neura_sync/neura_sync.hpp>
#include <neura_sync/participant_factory.hpp>
#include <neura_sync/publisher_factory.hpp>
#include <neura_sync_core_msgs/GripperState.hpp>
#include <neura_sync_core_msgs/GripperStatePubSubTypes.hpp>

// Bring GripperState types into scope
using GripperState::DeviceState;
using GripperState::DeviceStatePubSubType;
#endif

// DH Gripper register addresses
const uint16_t REG_INIT_COMMAND = 256;
const uint16_t REG_FORCE_PERCENT = 257;
const uint16_t REG_SPEED_PERCENT = 258;
const uint16_t REG_TARGET_POSITION = 259;

const uint16_t REG_INIT_STATE = 512;
const uint16_t REG_GRIPPER_STATE = 513;
const uint16_t REG_CURRENT_POSITION = 514;

std::atomic<bool> running{true};
std::string g_device_id = "gripper_01";

void signal_handler(int) {
    running = false;
}

#ifdef BUILD_WITH_NEURASYNC
class GripperPublisher {
public:
    GripperPublisher() : node_(nullptr), pub_factory_(nullptr), initialized_(false) {}

    bool init(const std::string& device_id) {
        device_id_ = device_id;
        try {
            neura::sync::utils::Logger::instance().setLogLevel(
                neura::sync::utils::LogLevel::INFO);

            neura::sync::ParticipantFactory* part_factory =
                neura::sync::ParticipantFactory::getInstance();

            node_ = std::make_shared<neura::sync::NeuraSyncNode>(
                part_factory->createSharedMemUdpParticipant());

            pub_factory_ = neura::sync::PublisherFactory::getInstance(
                node_->get_participant());

            // Create publisher for gripper state
            // Topic: qnc/modbus/device_state (matches dashboard expectation)
            node_->add_publisher(
                pub_factory_->createDefaultPublisher<
                    DeviceState,
                    DeviceStatePubSubType>(
                    "GripperState/publisher",
                    "qnc/modbus/device_state",
                    1000000000,  // deadline 1s
                    "",          // partition
                    3000000000   // liveliness 3s
                ));

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            initialized_ = true;
            std::cout << "[NeuraSync] Publisher initialized on topic: qnc/modbus/device_state\n";
            return true;
        } catch (const std::exception& ex) {
            std::cerr << "[NeuraSync] Init failed: " << ex.what() << "\n";
            return false;
        }
    }

    void publish(uint16_t init_state, uint16_t gripper_state, uint16_t position,
                 uint16_t force, uint16_t speed, bool online) {
        if (!initialized_) return;

        DeviceState msg;

        // Set device ID
        msg.device_id(device_id_);

        // Set timestamp
        auto now = std::chrono::system_clock::now();
        auto epoch = now.time_since_epoch();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(epoch);
        auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - sec);
        msg.timestamp_sec(static_cast<uint32_t>(sec.count()));
        msg.timestamp_nsec(static_cast<uint32_t>(nsec.count()));

        // Set gripper state fields
        msg.init_state(init_state);
        msg.gripper_state(gripper_state);
        msg.position(position);
        msg.force(force);
        msg.speed(speed);
        msg.online(online);

        node_->set_publisher_buffer<DeviceState, DeviceStatePubSubType>(
            "GripperState/publisher", msg);

        node_->publish("GripperState/publisher");

        const char* init_str[] = {"NOT_INIT", "INITIALIZING", "READY"};
        const char* state_str[] = {"IN_MOTION", "POS_REACHED", "OBJ_CAUGHT", "OBJ_DROPPED"};
        std::cout << "[NeuraSync] Published: " << device_id_
                  << " init=" << (init_state < 3 ? init_str[init_state] : "?")
                  << " state=" << (gripper_state < 4 ? state_str[gripper_state] : "?")
                  << " pos=" << position
                  << " force=" << force << "% speed=" << speed << "%\n";
    }

    bool isInitialized() const { return initialized_; }

private:
    std::shared_ptr<neura::sync::NeuraSyncNode> node_;
    std::shared_ptr<neura::sync::PublisherFactory> pub_factory_;
    std::string device_id_;
    bool initialized_;
};
#endif

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

#ifdef BUILD_WITH_NEURASYNC
void publish_status(modbus_rtu& mb, GripperPublisher& pub, uint16_t force, uint16_t speed) {
    uint16_t regs[3] = {0};
    int ret = mb.read_input_registers(REG_INIT_STATE, 3, regs);
    if (ret == RTU_SUCCESS) {
        pub.publish(regs[0], regs[1], regs[2], force, speed, true);
    } else {
        // Publish offline state
        pub.publish(0, 0, 0, 0, 0, false);
    }
}
#endif

bool wait_for_motion_complete(modbus_rtu& mb, int timeout_ms = 5000) {
    auto start = std::chrono::steady_clock::now();
    while (running) {
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
    return false;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    uint8_t slave_id = (argc > 2) ? std::stoi(argv[2]) : 1;
    uint32_t baudrate = (argc > 3) ? std::stoul(argv[3]) : 115200;
    g_device_id = (argc > 4) ? argv[4] : "gripper_01";

    std::cout << "DH Gripper Control with NeuraSync\n";
    std::cout << "==================================\n";
    std::cout << "Port: " << port << ", Slave: " << (int)slave_id
              << ", Baud: " << baudrate << ", Device: " << g_device_id << "\n\n";

#ifdef BUILD_WITH_NEURASYNC
    // Initialize NeuraSync publisher
    GripperPublisher ns_pub;
    bool dds_ok = ns_pub.init(g_device_id);
    if (!dds_ok) {
        std::cerr << "Warning: NeuraSync publisher initialization failed, continuing without publishing\n";
    }
#endif

    modbus_rtu mb(port, baudrate, 8, 1, 'N');
    mb.set_slave_id(slave_id);
    mb.set_response_timeout_ms(2000);
    mb.set_inter_frame_delay_ms(100);

    if (!mb.connect()) {
        std::cerr << "Failed to connect: " << mb.error_msg << "\n";
        return 1;
    }
    std::cout << "Connected!\n\n";

    uint16_t force = 50, speed = 50, target = 0;

    // Read initial status
    std::cout << "Initial status:\n";
    print_status(mb);
#ifdef BUILD_WITH_NEURASYNC
    if (dds_ok) publish_status(mb, ns_pub, force, speed);
#endif

    // Step 1: Initialize the gripper
    std::cout << "\n[1] Initializing gripper...\n";
    int ret = mb.write_register(REG_INIT_COMMAND, 1);
    if (ret != RTU_SUCCESS) {
        std::cerr << "  Failed to send init command: " << mb.error_msg << "\n";
    } else {
        std::cout << "  Init command sent\n";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        print_status(mb);
#ifdef BUILD_WITH_NEURASYNC
        if (dds_ok) publish_status(mb, ns_pub, force, speed);
#endif
    }

    // Step 2: Set force and speed
    std::cout << "\n[2] Setting force=50%, speed=50%...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mb.write_register(REG_FORCE_PERCENT, force);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mb.write_register(REG_SPEED_PERCENT, speed);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "  Parameters set\n";
#ifdef BUILD_WITH_NEURASYNC
    if (dds_ok) publish_status(mb, ns_pub, force, speed);
#endif

    // Step 3: Open gripper (position = 0)
    std::cout << "\n[3] Opening gripper...\n";
    target = 0;
    ret = mb.write_register(REG_TARGET_POSITION, target);
    if (ret != RTU_SUCCESS) {
        std::cerr << "  Failed: " << mb.error_msg << "\n";
    } else {
        std::cout << "  Command sent, waiting...\n";
        wait_for_motion_complete(mb);
        print_status(mb);
#ifdef BUILD_WITH_NEURASYNC
        if (dds_ok) publish_status(mb, ns_pub, force, speed);
#endif
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Step 4: Close gripper (position = 1000)
    std::cout << "\n[4] Closing gripper...\n";
    target = 1000;
    ret = mb.write_register(REG_TARGET_POSITION, target);
    if (ret != RTU_SUCCESS) {
        std::cerr << "  Failed: " << mb.error_msg << "\n";
    } else {
        std::cout << "  Command sent, waiting...\n";
        wait_for_motion_complete(mb);
        print_status(mb);
#ifdef BUILD_WITH_NEURASYNC
        if (dds_ok) publish_status(mb, ns_pub, force, speed);
#endif
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Step 5: Open again
    std::cout << "\n[5] Opening gripper again...\n";
    target = 0;
    ret = mb.write_register(REG_TARGET_POSITION, target);
    if (ret != RTU_SUCCESS) {
        std::cerr << "  Failed: " << mb.error_msg << "\n";
    } else {
        std::cout << "  Command sent, waiting...\n";
        wait_for_motion_complete(mb);
        print_status(mb);
#ifdef BUILD_WITH_NEURASYNC
        if (dds_ok) publish_status(mb, ns_pub, force, speed);
#endif
    }

    // Step 6: 50 cycles with NeuraSync publishing
    std::cout << "\n[6] Performing 50 cycles of open/close...\n";
    for (int i = 0; i < 50 && running; ++i) {
        std::cout << "  Cycle " << (i + 1) << "/50\n";

        // Close
        target = 1000;
        mb.write_register(REG_TARGET_POSITION, target);
        wait_for_motion_complete(mb);
        print_status(mb);
#ifdef BUILD_WITH_NEURASYNC
        if (dds_ok) publish_status(mb, ns_pub, force, speed);
#endif

        // Open
        target = 0;
        mb.write_register(REG_TARGET_POSITION, target);
        wait_for_motion_complete(mb);
        print_status(mb);
#ifdef BUILD_WITH_NEURASYNC
        if (dds_ok) publish_status(mb, ns_pub, force, speed);
#endif
    }

    std::cout << "\nDone!\n";
    mb.close();
    return 0;
}
