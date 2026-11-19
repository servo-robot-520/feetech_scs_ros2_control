#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "feetech_servo_ros2_control/scs_register_map.hpp"

namespace feetech_servo_ros2_control {

    /// @brief 参数存储区/访问方式。
    enum class ParamAccess {
        /// 只读。
        kReadOnly,
        /// SRAM，实时可写。
        kRam,
        /// EEPROM，需解锁后写入，掉电保存。
        kEeprom,
    };

    /// @brief 参数字节宽度。
    enum class ParamWidth {
        kByte,
        kWord,
    };

    /// @brief 单个参数的描述：GUI 参数编辑器与配置校验的唯一元数据来源。
    ///
    /// 取值范围以《舵机协议内存表-电位器版本》为准；插件既有参数的范围与
    /// 历史行为保持一致。
    struct ScsParamDescriptor {
        /// 配置/接口使用的键名（snake_case）。
        const char* key;
        /// 英文显示名。
        const char* label;
        /// 寄存器起始地址。
        uint8_t address;
        /// 字节宽度。
        ParamWidth width;
        /// 存储区/访问方式。
        ParamAccess access;
        /// 单位；无单位为空字符串。
        const char* unit;
        /// 最小取值。
        int min;
        /// 最大取值。
        int max;
        /// 枚举取值表（值为索引时给出实际含义）；空为自由取值。
        const int* choices{};
        int choice_count{};
    };

    /// @brief SCS 电位器版全部可读写参数的描述表（按地址升序）。
    inline constexpr std::array<ScsParamDescriptor, 26> kParameterDescriptors{{
        {.key = "id",
         .label = "ID",
         .address = kRegisterId,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 253},
        {.key = "baud_rate",
         .label = "Baud rate (index)",
         .address = kRegisterBaudRate,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "index",
         .min = 0,
         .max = 7,
         .choices = kServoBaudRates.data(),
         .choice_count = static_cast<int>(kServoBaudRates.size())},
        {.key = "return_delay_time",
         .label = "Return delay",
         .address = kRegisterReturnDelay,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "2 us",
         .min = 0,
         .max = 254},
        {.key = "response_level",
         .label = "Response level",
         .address = kRegisterResponseLevel,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 1},
        {.key = "range_min",
         .label = "Min angle limit",
         .address = kRegisterMinAngleLimit,
         .width = ParamWidth::kWord,
         .access = ParamAccess::kEeprom,
         .unit = "tick",
         .min = 0,
         .max = 1022},
        {.key = "range_max",
         .label = "Max angle limit",
         .address = kRegisterMaxAngleLimit,
         .width = ParamWidth::kWord,
         .access = ParamAccess::kEeprom,
         .unit = "tick",
         .min = 0,
         .max = 1023},
        {.key = "max_temperature",
         .label = "Max temperature",
         .address = kRegisterMaxTemperature,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "deg C",
         .min = 0,
         .max = 100},
        {.key = "max_input_voltage",
         .label = "Max input voltage",
         .address = kRegisterMaxInputVoltage,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "0.1 V",
         .min = 0,
         .max = 254},
        {.key = "min_input_voltage",
         .label = "Min input voltage",
         .address = kRegisterMinInputVoltage,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "0.1 V",
         .min = 0,
         .max = 254},
        {.key = "max_torque",
         .label = "Max torque",
         .address = kRegisterMaxTorque,
         .width = ParamWidth::kWord,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 1000},
        {.key = "phase",
         .label = "Phase (special function)",
         .address = kRegisterPhase,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 254},
        {.key = "unloading_condition",
         .label = "Unloading condition",
         .address = kRegisterUnloadingCondition,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 254},
        {.key = "led_alarm_condition",
         .label = "LED alarm condition",
         .address = kRegisterLedAlarmCondition,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 254},
        {.key = "p_coefficient",
         .label = "P gain",
         .address = kRegisterPCoefficient,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 254},
        {.key = "d_coefficient",
         .label = "D gain",
         .address = kRegisterDCoefficient,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 254},
        {.key = "min_startup_torque",
         .label = "Min startup torque",
         .address = kRegisterMinStartupTorque,
         .width = ParamWidth::kWord,
         .access = ParamAccess::kEeprom,
         .unit = "",
         .min = 0,
         .max = 1000},
        {.key = "cw_dead",
         .label = "CW dead zone",
         .address = kRegisterCwDeadZone,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "tick",
         .min = 0,
         .max = 32},
        {.key = "ccw_dead",
         .label = "CCW dead zone",
         .address = kRegisterCcwDeadZone,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "tick",
         .min = 0,
         .max = 32},
        {.key = "protection_torque",
         .label = "Protection torque",
         .address = kRegisterProtectionTorque,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "%",
         .min = 0,
         .max = 100},
        {.key = "protection_time",
         .label = "Protection time",
         .address = kRegisterProtectionTime,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "40 ms",
         .min = 0,
         .max = 254},
        {.key = "overload_torque",
         .label = "Overload torque",
         .address = kRegisterOverloadTorque,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kEeprom,
         .unit = "%",
         .min = 0,
         .max = 100},
        {.key = "torque_enable",
         .label = "Torque switch",
         .address = kRegisterTorqueEnable,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kRam,
         .unit = "",
         .min = 0,
         .max = 3},
        {.key = "goal_position",
         .label = "Goal position",
         .address = kRegisterGoalPosition,
         .width = ParamWidth::kWord,
         .access = ParamAccess::kRam,
         .unit = "tick",
         .min = 0,
         .max = 1023},
        {.key = "goal_time",
         .label = "Goal time",
         .address = kRegisterGoalTime,
         .width = ParamWidth::kWord,
         .access = ParamAccess::kRam,
         .unit = "ms",
         .min = 0,
         .max = 9999},
        {.key = "goal_speed",
         .label = "Goal speed",
         .address = kRegisterGoalSpeed,
         .width = ParamWidth::kWord,
         .access = ParamAccess::kRam,
         .unit = "tick/s",
         .min = 0,
         .max = 1000},
        {.key = "eeprom_lock",
         .label = "EEPROM lock",
         .address = kRegisterEepromLock,
         .width = ParamWidth::kByte,
         .access = ParamAccess::kRam,
         .unit = "",
         .min = 0,
         .max = 1},
    }};

    /// @brief 按键名查找参数描述。
    /// @param key 参数键名。
    /// @return 描述指针；未找到返回 nullptr。
    inline const ScsParamDescriptor* find_parameter(const std::string_view key) {
        for (const auto& descriptor : kParameterDescriptors) {
            if (key == descriptor.key) {
                return &descriptor;
            }
        }
        return nullptr;
    }

} // namespace feetech_servo_ros2_control
