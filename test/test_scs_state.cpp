#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

#include "fake_scs_servo.hpp"
#include "feetech_scs_servo_ros2_control/scs_protocol.hpp"
#include "feetech_scs_servo_ros2_control/scs_register_map.hpp"
#include "feetech_scs_servo_ros2_control/scs_serial_port.hpp"
#include "feetech_scs_servo_ros2_control/scs_state.hpp"

namespace feetech_scs_servo_ros2_control {
    namespace {

        using test::FakeScsBus;

        TEST(ScsStateDecode, DecodesFeedbackBlock) {
            // 位置 600、速度 -50 tick/s、负载 250（0.25）、电压 74（7.4 V）、
            // 温度 30、状态 0x04、移动标志 1、电流 -20（-0.13 A）。
            std::array<uint8_t, kFeedbackLength> feedback{};
            encode_word(feedback.data(), 600);
            encode_word(feedback.data() + 2, static_cast<uint16_t>((1U << kSpeedSignBit) | 50U));
            encode_word(feedback.data() + 4, 250);
            feedback[6] = 74;
            feedback[7] = 30;
            feedback[9] = 0x04;
            feedback[10] = 1;
            encode_word(feedback.data() + 13, static_cast<uint16_t>((1U << kCurrentSignBit) | 20U));

            const auto state = decode_feedback(feedback);
            ASSERT_TRUE(state);
            EXPECT_EQ(state.value().position_ticks, 600);
            EXPECT_EQ(state.value().velocity_ticks_per_s, -50);
            EXPECT_DOUBLE_EQ(state.value().load, 0.25);
            EXPECT_DOUBLE_EQ(state.value().voltage, 7.4);
            EXPECT_DOUBLE_EQ(state.value().temperature, 30.0);
            EXPECT_TRUE(state.value().moving);
            EXPECT_EQ(state.value().status, 0x04);
            EXPECT_DOUBLE_EQ(state.value().current, -20 * kCurrentAmpsPerUnit);
        }

        TEST(ScsStateDecode, RejectsShortFeedbackBlock) {
            const std::array<uint8_t, 3> short_block{};
            EXPECT_FALSE(decode_feedback(short_block));
        }

        TEST(ScsStateBus, ScanFindsOnlineServos) {
            FakeScsBus bus({1, 2});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(20));

            const auto ids = protocol.scan(0, 5);
            ASSERT_TRUE(ids);
            EXPECT_EQ(ids.value(), std::vector<uint8_t>({1, 2}));

            const auto single = protocol.scan(2, 2);
            ASSERT_TRUE(single);
            EXPECT_EQ(single.value(), std::vector<uint8_t>({2}));
        }

        TEST(ScsStateBus, ReadIdentityReturnsFirmwareAndModel) {
            FakeScsBus bus({1}, kModelSc225);
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            const auto identity = protocol.read_identity(1);
            ASSERT_TRUE(identity);
            EXPECT_EQ(identity.value().firmware_major, 3);
            EXPECT_EQ(identity.value().firmware_minor, 24);
            EXPECT_EQ(identity.value().model, kModelSc225);
        }

        TEST(ScsStateBus, ReadStateDecodesFeedback) {
            FakeScsBus bus({1});
            bus.set_present_position(1, 600);
            bus.set_present_speed(1, 150);
            bus.set_present_load(1, static_cast<uint16_t>((1U << kLoadSignBit) | 300U));
            bus.set_present_current(1, 20);
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            const auto state = protocol.read_state(1);
            ASSERT_TRUE(state);
            EXPECT_EQ(state.value().position_ticks, 600);
            EXPECT_EQ(state.value().velocity_ticks_per_s, 150);
            EXPECT_DOUBLE_EQ(state.value().load, -0.3);
            EXPECT_DOUBLE_EQ(state.value().voltage, 7.4);
            EXPECT_DOUBLE_EQ(state.value().temperature, 30.0);
            EXPECT_DOUBLE_EQ(state.value().current, 20 * kCurrentAmpsPerUnit);
            EXPECT_FALSE(state.value().moving);
        }

        TEST(ScsStateBus, WriteWordWritesRegister) {
            FakeScsBus bus({1});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            ASSERT_TRUE(protocol.write_word(1, kRegisterMinStartupTorque, 30));
            // 高字节在前：30 = 0x001E，地址 24 存 0x00、地址 25 存 0x1E。
            EXPECT_EQ(bus.register_value(1, kRegisterMinStartupTorque), 0x00);
            EXPECT_EQ(bus.register_value(1, kRegisterMinStartupTorque + 1), 0x1E);
        }

        TEST(ScsStateBus, WriteEepromManagesLockAndTorque) {
            FakeScsBus bus({1});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            ASSERT_TRUE(protocol.set_torque_enabled(1, true));
            ASSERT_TRUE(bus.torque_enabled(1));

            const std::array<uint8_t, 1> value{75};
            ASSERT_TRUE(protocol.write_eeprom(1, kRegisterMaxTemperature, value));
            EXPECT_EQ(bus.register_value(1, kRegisterMaxTemperature), 75);
            EXPECT_FALSE(bus.torque_enabled(1));
            EXPECT_EQ(bus.register_value(1, kRegisterEepromLock), 1);
        }

        TEST(ScsStateBus, TraceHookCapturesFrames) {
            FakeScsBus bus({1});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            std::vector<std::pair<std::string, std::vector<uint8_t>>> frames;
            protocol.set_trace_hook(
                [&frames](const char* direction, const std::span<const uint8_t> frame) {
                    frames.emplace_back(direction, std::vector<uint8_t>(frame.begin(), frame.end()));
                });

            ASSERT_TRUE(protocol.ping(1));
            ASSERT_EQ(frames.size(), 2u);
            EXPECT_EQ(frames[0].first, "tx");
            // PING 帧：FF FF 01 02 01 FB
            EXPECT_EQ(frames[0].second, std::vector<uint8_t>({0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
            EXPECT_EQ(frames[1].first, "rx");
            // PING 应答帧：FF FF 01 02 <状态 0x00> <校验>，共 6 字节。
            ASSERT_EQ(frames[1].second.size(), 6u);
            EXPECT_EQ(frames[1].second[0], 0xFF);
            EXPECT_EQ(frames[1].second[1], 0xFF);
            EXPECT_EQ(frames[1].second[2], 0x01);
            EXPECT_EQ(frames[1].second[4], 0x00);

            // 清除钩子后不再记录。
            protocol.set_trace_hook(nullptr);
            ASSERT_TRUE(protocol.ping(1));
            EXPECT_EQ(frames.size(), 2u);
        }

    } // namespace
} // namespace feetech_scs_servo_ros2_control
