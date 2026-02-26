// Auto-generated types for mcan_basic_module_msgs ROS package
//
// DO NOT EDIT MANUALLY
//

#pragma once

#include <cstdint>
#include <string>
#include <cstddef>
#include "mc_plugin_base/mc_firmware/super_types.hpp"

namespace mcan {

namespace mcan_basic_module {

// Forward declarations
enum class DeviceStatus_t : std::uint8_t;
struct DeviceNodeIdentifier_t;
struct HardwareType_t;
struct DeviceIdentifier_t;
struct DeviceInfo_t;

// Type definitions

/**
 * @brief Device Status
 * Status of the device
 */
enum class DeviceStatus_t : std::uint8_t {
    OK = 0,
    WARNING = 1,
    ERROR = 2,
    CRITICAL = 3,
    UNKNOWN = 4,
    UNCONFIGURED = 5,
    STOPPED = 6,
    WAITING = 7,
};

/**
 * @brief Device Identifier
 * Unique identifier for a device
 */
struct DeviceNodeIdentifier_t {

    /**
     * @brief Node ID
     * @type uint8_t
     * @default 0
     */
    uint8_t manufacturer_id = 0;
};

/**
 * @brief Hardware Type
 * Hardware Type Information
 */
struct HardwareType_t {

    /**
     * @brief Hardware timestamp
     * @type uint32_t
     * @default 0
     */
    uint32_t hw_time_stamp = 0;

    /**
     * @brief Hardware revision number
     * @type uint16_t
     * @default 0
     */
    uint16_t hw_revision = 0;

    /**
     * @brief Firmware revision number
     * @type uint16_t
     * @default 0
     */
    uint16_t fw_revision = 0;
};

/**
 * @brief Device Identifier
 * Unique Device Identifier
 */
struct DeviceIdentifier_t {

    /**
     * @brief Unique identifier for the device
     * @type uint64_t
     * @default 0
     */
    uint64_t unique_id = 0;
};

/**
 * @brief Device Info
 * Comprehensive Device Information
 */
struct DeviceInfo_t {

    /**
     * @brief Unique device identifier
     * @type DeviceIdentifier_t
     * @default {}
     */
    DeviceIdentifier_t device_identifier = {};

    /**
     * @brief Hardware type information
     * @type HardwareType_t
     * @default {}
     */
    HardwareType_t hardware_type = {};
};

}  // namespace mcan_basic_module

}  // namespace mcan