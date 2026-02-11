/**
 * Gripper Dashboard Subscriber
 * Subscribes to gripper state from NeuraSync using DeviceState message
 * Compatible with neurasync-dashboard
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <iomanip>

#include <neura_sync/neura_sync.hpp>
#include <neura_sync/participant_factory.hpp>
#include <neura_sync/subscriber_factory.hpp>
#include <neura_sync_core_msgs/GripperState.hpp>
#include <neura_sync_core_msgs/GripperStatePubSubTypes.hpp>
#include <neura_sync_utils/logger.hpp>

// Bring GripperState types into scope
using GripperState::DeviceState;
using GripperState::DeviceStatePubSubType;

std::atomic<bool> running{true};

void signal_handler(int) {
    running = false;
}

// Callback when gripper state is received
void gripper_state_callback(const DeviceState& msg) {
    // State strings
    const char* init_str[] = {"NOT_INIT", "INITIALIZING", "INITIALIZED"};
    const char* state_str[] = {"IN_MOTION", "POS_REACHED", "OBJ_CAUGHT", "OBJ_DROPPED"};

    uint16_t init_state = msg.init_state();
    uint16_t gripper_state = msg.gripper_state();

    // Clear line and print dashboard
    std::cout << "\r\033[K";  // Clear line
    std::cout << "[" << msg.device_id() << "] "
              << "Init: " << std::setw(12) << (init_state < 3 ? init_str[init_state] : "UNKNOWN") << " | "
              << "State: " << std::setw(11) << (gripper_state < 4 ? state_str[gripper_state] : "UNKNOWN") << " | "
              << "Pos: " << std::setw(4) << msg.position() << " | "
              << "Force: " << std::setw(3) << msg.force() << "% | "
              << "Speed: " << std::setw(3) << msg.speed() << "% | "
              << "Online: " << (msg.online() ? "YES" : "NO");
    std::cout.flush();
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::cout << "===========================================\n";
    std::cout << "   Gripper Dashboard - NeuraSync Subscriber\n";
    std::cout << "===========================================\n";
    std::cout << "Subscribing to topic: qnc/modbus/device_state\n";
    std::cout << "Message type: DeviceState\n";
    std::cout << "Press Ctrl+C to exit\n\n";

    // Set log level
    neura::sync::utils::Logger::instance().setLogLevel(
        neura::sync::utils::LogLevel::WARNING);

    // Create participant
    neura::sync::ParticipantFactory* part_factory =
        neura::sync::ParticipantFactory::getInstance();

    auto participant = part_factory->createSharedMemUdpParticipant();
    if (!participant) {
        std::cerr << "Failed to create participant\n";
        return 1;
    }

    // Create subscriber factory
    auto sub_factory = neura::sync::SubscriberFactory::getInstance(participant);

    // Create subscriber for gripper state
    // Topic: qnc/modbus/device_state (matches dashboard and gripper_control)
    auto subscriber = sub_factory->createDefaultSubscriber<
        DeviceState,
        DeviceStatePubSubType>(
            "GripperState/subscriber",
            "qnc/modbus/device_state",
            gripper_state_callback,  // callback function
            1000000000,              // deadline 1s (in nanoseconds)
            "",                      // partition (empty = default)
            3000000000);             // lease_duration 3s

    if (!subscriber) {
        std::cerr << "Failed to create subscriber\n";
        part_factory->cleanParticipant(participant);
        return 1;
    }

    std::cout << "Waiting for gripper data...\n\n";

    // Main loop - just wait for callbacks
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n\nShutting down...\n";

    subscriber.reset();
    part_factory->cleanParticipant(participant);

    return 0;
}
