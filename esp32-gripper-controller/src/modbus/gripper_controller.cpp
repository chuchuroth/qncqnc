#include "gripper_controller.h"

GripperController::GripperController(ModbusRTU_ESP32& modbus)
    : _modbus(modbus)
    , _stateMutex(nullptr)
    , _cmdQueue(nullptr)
{
    memset(&_state, 0, sizeof(_state));
    _state.force_percent = 50;   // Sensible defaults
    _state.speed_percent = 50;
}

bool GripperController::begin()
{
    _stateMutex = xSemaphoreCreateMutex();
    if (!_stateMutex) {
        Serial.println("[GripperCtrl] Failed to create state mutex");
        return false;
    }

    _cmdQueue = xQueueCreate(8, sizeof(GripperCommandMsg));
    if (!_cmdQueue) {
        Serial.println("[GripperCtrl] Failed to create command queue");
        return false;
    }

    Serial.println("[GripperCtrl] Initialized");
    return true;
}

// ============================================================================
// Public command methods — thread-safe, enqueue for modbusTask
// ============================================================================

void GripperController::sendCommand(GripperCommand cmd, uint16_t value)
{
    GripperCommandMsg msg = {cmd, value};
    xQueueSend(_cmdQueue, &msg, pdMS_TO_TICKS(100));
}

void GripperController::init(bool full_calibration)
{
    sendCommand(full_calibration ? GripperCommand::FULL_INIT : GripperCommand::INIT);
}

void GripperController::setPosition(uint16_t position)
{
    sendCommand(GripperCommand::SET_POSITION,
                constrain(position, GRIPPER_POSITION_MIN, GRIPPER_POSITION_MAX));
}

void GripperController::setForce(uint16_t percent)
{
    sendCommand(GripperCommand::SET_FORCE,
                constrain(percent, GRIPPER_FORCE_MIN, GRIPPER_FORCE_MAX));
}

void GripperController::setSpeed(uint16_t percent)
{
    sendCommand(GripperCommand::SET_SPEED,
                constrain(percent, GRIPPER_SPEED_MIN, GRIPPER_SPEED_MAX));
}

void GripperController::open()
{
    sendCommand(GripperCommand::OPEN);
}

void GripperController::close()
{
    sendCommand(GripperCommand::CLOSE);
}

// ============================================================================
// Thread-safe state access
// ============================================================================

GripperState GripperController::getState()
{
    GripperState snapshot;
    xSemaphoreTake(_stateMutex, portMAX_DELAY);
    snapshot = _state;
    xSemaphoreGive(_stateMutex);
    return snapshot;
}

// ============================================================================
// Poll cycle — called from modbusTask every TASK_MODBUS_POLL_MS
// ============================================================================

void GripperController::poll()
{
    // Process any pending commands (non-blocking)
    GripperCommandMsg cmd;
    while (xQueueReceive(_cmdQueue, &cmd, 0) == pdTRUE) {
        processCommand(cmd);
    }

    // Read gripper status
    readStatus();
}

// ============================================================================
// Command processing — ported from gripper_control.cpp:226-298
// ============================================================================

void GripperController::processCommand(const GripperCommandMsg& cmd)
{
    int ret = RTU_SUCCESS;

    switch (cmd.cmd) {
        case GripperCommand::INIT:
            ret = _modbus.writeRegister(REG_INIT_COMMAND, 1);
            Serial.println("[GripperCtrl] Init command sent");
            break;

        case GripperCommand::FULL_INIT:
            ret = _modbus.writeRegister(REG_INIT_COMMAND, 0xA5);
            Serial.println("[GripperCtrl] Full init command sent");
            break;

        case GripperCommand::SET_POSITION:
            ret = _modbus.writeRegister(REG_TARGET_POSITION, cmd.value);
            if (ret == RTU_SUCCESS) {
                xSemaphoreTake(_stateMutex, portMAX_DELAY);
                _state.target_position = cmd.value;
                xSemaphoreGive(_stateMutex);
            }
            break;

        case GripperCommand::SET_FORCE:
            ret = _modbus.writeRegister(REG_FORCE_PERCENT, cmd.value);
            if (ret == RTU_SUCCESS) {
                xSemaphoreTake(_stateMutex, portMAX_DELAY);
                _state.force_percent = cmd.value;
                xSemaphoreGive(_stateMutex);
            }
            break;

        case GripperCommand::SET_SPEED:
            ret = _modbus.writeRegister(REG_SPEED_PERCENT, cmd.value);
            if (ret == RTU_SUCCESS) {
                xSemaphoreTake(_stateMutex, portMAX_DELAY);
                _state.speed_percent = cmd.value;
                xSemaphoreGive(_stateMutex);
            }
            break;

        case GripperCommand::OPEN:
            ret = _modbus.writeRegister(REG_TARGET_POSITION, GRIPPER_POSITION_MIN);
            if (ret == RTU_SUCCESS) {
                xSemaphoreTake(_stateMutex, portMAX_DELAY);
                _state.target_position = GRIPPER_POSITION_MIN;
                xSemaphoreGive(_stateMutex);
            }
            break;

        case GripperCommand::CLOSE:
            ret = _modbus.writeRegister(REG_TARGET_POSITION, GRIPPER_POSITION_MAX);
            if (ret == RTU_SUCCESS) {
                xSemaphoreTake(_stateMutex, portMAX_DELAY);
                _state.target_position = GRIPPER_POSITION_MAX;
                xSemaphoreGive(_stateMutex);
            }
            break;

        default:
            break;
    }

    if (ret != RTU_SUCCESS) {
        Serial.printf("[GripperCtrl] Command failed: %s\n", _modbus.getErrorMessage());
    }

    // Allow inter-frame delay before next operation
    vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_DELAY_MS));
}

// ============================================================================
// Status reading — ported from gripper_control.cpp:135-148
// ============================================================================

void GripperController::readStatus()
{
    uint16_t regs[3] = {0};
    int ret = _modbus.readInputRegisters(REG_INIT_STATE, 3, regs);

    xSemaphoreTake(_stateMutex, portMAX_DELAY);
    if (ret == RTU_SUCCESS) {
        _state.init_state = regs[0];
        _state.gripper_state = regs[1];
        _state.current_position = regs[2];
        _state.online = true;
        _state.last_update_ms = millis();
        _state.last_error = RTU_SUCCESS;
        _state.success_count++;
    } else {
        _state.online = false;
        _state.last_error = ret;
        _state.error_count++;
    }
    xSemaphoreGive(_stateMutex);
}
