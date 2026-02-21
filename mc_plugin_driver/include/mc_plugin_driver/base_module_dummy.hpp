// Auto-generated header for base_module_dummy module
// Do not edit manually

#pragma once

#include <cstdint>
#include <string>
#include <cstddef>
#include <tuple>
#include "super_types.hpp"

#include "base_module_dummy_types.hpp"

namespace mcan {

namespace base_module_dummy {

// Hardware information
struct Hardware_t {
    static constexpr const char* k_name = "base_module_dummy";
    static constexpr std::uint32_t k_time_stamp = 946681200;
    static constexpr std::uint32_t k_hw_revision = 1;
    static constexpr std::uint32_t k_fw_revision = 1;
    static constexpr std::uint64_t k_unique_id = 0x1;
    static constexpr const char* k_description = "Base module required for all modules";
};

// Message definitions

namespace configs {

/**
 * @brief Discover Devices
 * @details This will trigger all devices on the bus to respond with their uniq ID that defines the hardware
 * @permission r
 */
struct DiscoverDevices {
    using Type = uint64_t;
    static constexpr const char* k_name = "discover_devices";
    static constexpr std::uint32_t k_base_address = 0x001;
    static constexpr bool k_allow_read = true;
    static constexpr bool k_allow_write = false;

    uint64_t value;
};

/**
 * @brief Set Device Node ID
 * @details Set the device node ID of the module
 * @permission w
 */
struct SetDeviceNodeId {
    using Type = uint8_t;
    static constexpr const char* k_name = "set_device_node_id";
    static constexpr std::uint32_t k_base_address = 0x002;
    static constexpr bool k_allow_read = false;
    static constexpr bool k_allow_write = true;

    uint8_t value;
};

/**
 * @brief Enter Configuration Mode
 * @details Command to enter configuration mode on all connected modules
 * @default 0
 * @permission w
 */
struct EnterConfigurationMode {
    using Type = uint8_t;
    static constexpr const char* k_name = "enter_configuration_mode";
    static constexpr std::uint32_t k_base_address = 0x003;
    static constexpr bool k_allow_read = false;
    static constexpr bool k_allow_write = true;

    uint8_t value = {0};
};

/**
 * @details Device Identifier message
 * @permission r
 */
struct GetHardwareType {
    using Type = HardwareType_t;
    static constexpr const char* k_name = "get_hardware_type";
    static constexpr std::uint32_t k_base_address = 0x004;
    static constexpr bool k_allow_read = true;
    static constexpr bool k_allow_write = false;

    HardwareType_t value;
};

/**
 * @brief Ping Module
 * @details Ping the module to check if it's alive, the counter will increment on each ping
 * @default 0
 * @permission w
 */
struct PingModule {
    using Type = uint8_t;
    static constexpr const char* k_name = "ping_module";
    static constexpr std::uint32_t k_base_address = 0x005;
    static constexpr bool k_allow_read = false;
    static constexpr bool k_allow_write = true;

    uint8_t value = {0};
};

/**
 * @brief Flash Indicator LED
 * @details Command to flash the indicator LED on the module, useful for locating specific hardware
 * @default 0
 * @permission w
 */
struct FlashIndicatorLed {
    using Type = bool;
    static constexpr const char* k_name = "flash_indicator_led";
    static constexpr std::uint32_t k_base_address = 0x006;
    static constexpr bool k_allow_read = false;
    static constexpr bool k_allow_write = true;

    bool value = {0};
};

/**
 * @brief Dummy Command
 * @details This is a dummy command for testing purposes
 * @default 0
 * @permission rw
 */
struct DummyCommand {
    using Type = uint8_t;
    static constexpr const char* k_name = "dummy_command";
    static constexpr std::uint32_t k_base_address = 0x010;
    static constexpr bool k_allow_read = true;
    static constexpr bool k_allow_write = true;

    uint8_t value = {0};
};

}  // namespace configs

// CAN MC Slave Interface
class McCanSlaveInterface_t {
public:
    configs::DummyCommand dummy_command;

    // Write callbacks
    void callback_write_dummy_command(configs::DummyCommand& variable);

    auto get_write_callbacks() {
        return std::make_tuple(
            std::make_pair(&McCanSlaveInterface_t::callback_write_dummy_command, &McCanSlaveInterface_t::dummy_command)
        );
    }

    auto get_read_variables() {
        return std::make_tuple(
            &McCanSlaveInterface_t::dummy_command
        );
    }

    auto get_config_variables() {
        return std::make_tuple(
            &McCanSlaveInterface_t::dummy_command
        );
    }
};
}  // namespace base_module_dummy

}  // namespace mcan