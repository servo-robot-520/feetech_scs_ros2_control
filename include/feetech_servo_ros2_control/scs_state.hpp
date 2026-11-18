#pragma once

#include <cstdint>
#include <span>
#include <string>

#include "feetech_servo_ros2_control/result.hpp"
#include "feetech_servo_ros2_control/scs_register_map.hpp"

namespace feetech_servo_ros2_control {

    /// @brief 舵机身份信息（寄存器 0x00..0x04）。
    struct ScsServoIdentity {
        /// 固件主版本号。
        uint8_t firmware_major{};
        /// 固件次版本号。
        uint8_t firmware_minor{};
        /// 型号标识：高字节舵机主版本号、低字节舵机次版本号（kRegisterModel）。
        uint16_t model{};
    };

    /// @brief 舵机反馈状态（寄存器 0x38..0x46 反馈块的解码结果）。
    struct ScsServoState {
        /// 当前位置，原始 tick，中位 kPositionMidpoint。
        uint16_t position_ticks{};
        /// 当前速度，带符号，单位 tick/s。
        int velocity_ticks_per_s{};
        /// 当前负载，-1..1，1000 = 100%。
        double load{};
        /// 当前电压，单位 V。
        double voltage{};
        /// 当前温度，单位 °C。
        double temperature{};
        /// 当前电流，单位 A。
        double current{};
        /// 移动标志：运动中为 true。
        bool moving{};
        /// 舵机状态位域（电压/温度/过载等错误位）。
        uint8_t status{};
    };

    /// @brief 解码 15 字节反馈块（kFeedbackLength，寄存器 0x38..0x46）。
    /// @param data 反馈块原始字节。
    /// @return 解码结果；长度不足 kFeedbackLength 时返回错误信息。
    inline Expected<ScsServoState> decode_feedback(const std::span<const uint8_t> data) {
        if (data.size() < kFeedbackLength) {
            return Expected<ScsServoState>::failure(
                "Feedback block requires " + std::to_string(kFeedbackLength) + " bytes, got " +
                std::to_string(data.size()));
        }
        // 块内偏移：位置 0、速度 2、负载 4、电压 6、温度 7、异步写标志 8、
        // 状态 9、移动标志 10、保留 11..12、电流 13。
        ScsServoState state;
        state.position_ticks = decode_word(data.data());
        state.velocity_ticks_per_s =
            decode_sign_magnitude(decode_word(data.data() + 2), kSpeedSignBit);
        state.load = static_cast<double>(
                         decode_sign_magnitude(decode_word(data.data() + 4), kLoadSignBit)) /
            1000.0;
        state.voltage = static_cast<double>(data[6]) * 0.1;
        state.temperature = static_cast<double>(data[7]);
        state.status = data[9];
        state.moving = data[10] != 0;
        state.current =
            static_cast<double>(
                decode_sign_magnitude(decode_word(data.data() + 13), kCurrentSignBit)) *
            kCurrentAmpsPerUnit;
        return Expected<ScsServoState>(state);
    }

} // namespace feetech_servo_ros2_control
