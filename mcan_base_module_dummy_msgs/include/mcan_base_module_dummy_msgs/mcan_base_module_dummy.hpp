// Auto-generated header for mcan_base_module_dummy module
// Do not edit manually

#pragma once

#include <cstdint>
#include <string>
#include <cstddef>
#include <tuple>
#include "mc_plugin_base/super_types.hpp"

#include <mcan_base_module_dummy_msgs/mcan_base_module_dummy_types.hpp>

namespace mcan {

namespace mcan_base_module_dummy {

// Hardware information
struct Hardware_t {
    static constexpr const char* k_name = "mcan_base_module_dummy";
    static constexpr std::uint32_t k_time_stamp = 946681200;
    static constexpr std::uint32_t k_hw_revision = 1;
    static constexpr std::uint32_t k_fw_revision = 1;
    static constexpr std::uint64_t k_unique_id = 0x1;
    static constexpr const char* k_description = "Base module required for all modules";
};

// Message definitions

namespace configs {

/**
 * @brief Dummy Command
 * @details This is a dummy command for testing purposes
 * @default 0
 * @permission rw
 */
struct DummyCommand {
    using Type = uint8_t;
    static constexpr const char* k_name = "dummy_command";
    static constexpr const char* k_group = "configs";
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

    auto get_write_variables() {
        return std::make_tuple(
            &McCanSlaveInterface_t::dummy_command
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
}  // namespace mcan_base_module_dummy

}  // namespace mcan