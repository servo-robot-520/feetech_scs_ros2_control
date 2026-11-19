#include "feetech_scs_ros2_control/scs_protocol.hpp"

#include <array>
#include <cstdio>
#include <string>

#include "feetech_scs_ros2_control/scs_register_map.hpp"

namespace feetech_scs_ros2_control {
    namespace {

        constexpr uint8_t kHeader = 0xFF;
        constexpr int kMaxHeaderBytes = 16;

        /// @brief 生成 “servo <id>” 形式的错误标签。
        std::string servo_label(const uint8_t id) {
            return "servo " + std::to_string(id);
        }

    } // namespace

    uint8_t packet_checksum(const uint8_t id, const uint8_t length, const uint8_t instruction,
                            const std::span<const uint8_t> params) {
        auto sum = static_cast<uint8_t>(id + length + instruction);
        for (const auto byte : params) {
            sum = static_cast<uint8_t>(sum + byte);
        }
        return static_cast<uint8_t>(~sum);
    }

    std::vector<uint8_t> make_packet(const uint8_t id, const uint8_t instruction,
                                     const std::span<const uint8_t> params) {
        const auto length = static_cast<uint8_t>(params.size() + 2);
        std::vector<uint8_t> packet;
        packet.reserve(static_cast<size_t>(length) + 4);
        packet.push_back(kHeader);
        packet.push_back(kHeader);
        packet.push_back(id);
        packet.push_back(length);
        packet.push_back(instruction);
        packet.insert(packet.end(), params.begin(), params.end());
        packet.push_back(packet_checksum(id, length, instruction, params));
        return packet;
    }

    ScsProtocol::ScsProtocol(ScsSerialPort& port, const std::chrono::milliseconds response_timeout) :
        port_(port), response_timeout_(response_timeout) {
    }

    void ScsProtocol::set_trace_hook(TraceHook hook) {
        trace_hook_ = std::move(hook);
    }

    Result ScsProtocol::transact(const std::vector<uint8_t>& packet, const uint8_t expected_id,
                                 const std::span<uint8_t> payload) const {
        port_.flush_input();
        if (trace_hook_) {
            trace_hook_("tx", packet);
        }
        if (const auto result = port_.write(packet); !result) {
            return Result::failure("Failed to send command to " + servo_label(expected_id) + ": " +
                result.error());
        }

        // 同步到 FF FF 帧头。上一次超时可能残留字节，因此在有限范围内逐字节
        // 扫描，而不是在第一次不匹配时直接失败。
        std::array<uint8_t, 2> header{};
        bool header_found = false;
        for (int attempt = 0; attempt < kMaxHeaderBytes; ++attempt) {
            if (attempt == 0) {
                if (const auto result = port_.read_exact(header, response_timeout_); !result) {
                    return Result::failure("No response from " + servo_label(expected_id) + ": " +
                        result.error());
                }
            }
            else {
                std::array<uint8_t, 1> next{};
                if (const auto result = port_.read_exact(next, response_timeout_); !result) {
                    return Result::failure("Incomplete response from " + servo_label(expected_id) + ": " +
                        result.error());
                }
                header[0] = header[1];
                header[1] = next[0];
            }
            if (header[0] == kHeader && header[1] == kHeader) {
                header_found = true;
                break;
            }
        }
        if (!header_found) {
            return Result::failure("Malformed response header from " + servo_label(expected_id));
        }

        std::array<uint8_t, 2> id_and_length{};
        if (const auto result = port_.read_exact(id_and_length, response_timeout_); !result) {
            return Result::failure("Incomplete response from " + servo_label(expected_id) + ": " +
                result.error());
        }

        const uint8_t id = id_and_length[0];
        const uint8_t length = id_and_length[1];
        if (length < 2) {
            return Result::failure("Invalid response length " + std::to_string(length) + " from " +
                servo_label(expected_id));
        }

        std::vector<uint8_t> body(length);
        if (const auto result = port_.read_exact(body, response_timeout_); !result) {
            return Result::failure("Incomplete response from " + servo_label(expected_id) + ": " +
                result.error());
        }
        if (trace_hook_) {
            std::vector<uint8_t> frame{kHeader, kHeader, id, length};
            frame.insert(frame.end(), body.begin(), body.end());
            trace_hook_("rx", frame);
        }

        auto sum = static_cast<uint8_t>(id + length);
        for (size_t i = 0; i + 1 < body.size(); ++i) {
            sum = static_cast<uint8_t>(sum + body[i]);
        }
        if (static_cast<uint8_t>(~sum) != body.back()) {
            return Result::failure("Checksum mismatch in response from " + servo_label(expected_id));
        }
        if (id != expected_id) {
            return Result::failure("Expected response from " + servo_label(expected_id) + ", got " +
                servo_label(id));
        }

        if (const auto response_payload_size = static_cast<size_t>(length) - 2; response_payload_size != payload.size()) {
            return Result::failure("Unexpected payload size " + std::to_string(response_payload_size) +
                " from " + servo_label(expected_id) + " (expected " +
                std::to_string(payload.size()) + ")");
        }
        for (size_t i = 0; i < payload.size(); ++i) {
            payload[i] = body[i + 1];
        }

        if (const uint8_t status = body[0]; status != 0) {
            char buffer[8]{};
            std::snprintf(buffer, sizeof(buffer), "0x%02X", static_cast<unsigned>(status));
            return Result::failure(servo_label(expected_id) + " reported status " + buffer);
        }
        return Result::success();
    }

    Result ScsProtocol::ping(const uint8_t id) const {
        return transact(make_packet(id, kInstructionPing, std::span<const uint8_t>{}), id,
                        std::span<uint8_t>{});
    }

    Result ScsProtocol::read_registers(const uint8_t id, const uint8_t address,
                                       const std::span<uint8_t> data) const {
        if (data.empty() || data.size() > 250) {
            return Result::failure("Invalid register read length " + std::to_string(data.size()));
        }
        const std::array<uint8_t, 2> params{address, static_cast<uint8_t>(data.size())};
        return transact(make_packet(id, kInstructionRead, params), id, data);
    }

    Result ScsProtocol::write_registers(const uint8_t id, const uint8_t address,
                                        const std::span<const uint8_t> data) const {
        if (data.empty() || data.size() > 250) {
            return Result::failure("Invalid register write length " + std::to_string(data.size()));
        }
        std::vector<uint8_t> params;
        params.reserve(data.size() + 1);
        params.push_back(address);
        params.insert(params.end(), data.begin(), data.end());
        return transact(make_packet(id, kInstructionWrite, params), id, std::span<uint8_t>{});
    }

    Result ScsProtocol::write_goal_position(const uint8_t id, const uint16_t position,
                                            const uint16_t time_ms, const uint16_t speed) const {
        std::array<uint8_t, 7> params{};
        params[0] = kRegisterGoalPosition;
        encode_word(params.data() + 1, position);
        encode_word(params.data() + 3, time_ms);
        encode_word(params.data() + 5, speed);
        return transact(make_packet(id, kInstructionWrite, params), id, std::span<uint8_t>{});
    }

    Result ScsProtocol::sync_write_goal_positions(const std::span<const uint8_t> ids,
                                                  const std::span<const uint16_t> positions,
                                                  const std::span<const uint16_t> times_ms,
                                                  const std::span<const uint16_t> speeds) const {
        if (ids.empty()) {
            return Result::success();
        }
        if (ids.size() != positions.size() || ids.size() != times_ms.size() ||
            ids.size() != speeds.size()) {
            return Result::failure("SYNC WRITE requires matching ids, positions, times and speeds");
        }

        constexpr size_t kDataLength = 6;
        if (const size_t length = 4 + (kDataLength + 1) * ids.size(); length > 255) {
            return Result::failure("Too many servos for one SYNC WRITE (" + std::to_string(ids.size()) + ")");
        }

        std::vector<uint8_t> params;
        params.reserve(2 + (kDataLength + 1) * ids.size());
        params.push_back(kRegisterGoalPosition);
        params.push_back(static_cast<uint8_t>(kDataLength));
        for (size_t i = 0; i < ids.size(); ++i) {
            params.push_back(ids[i]);
            const size_t offset = params.size();
            params.resize(offset + kDataLength);
            encode_word(params.data() + offset, positions[i]);
            encode_word(params.data() + offset + 2, times_ms[i]);
            encode_word(params.data() + offset + 4, speeds[i]);
        }

        port_.flush_input();
        const auto packet = make_packet(kBroadcastId, kInstructionSyncWrite, params);
        if (trace_hook_) {
            trace_hook_("tx", packet);
        }
        if (const auto result = port_.write(packet); !result) {
            return Result::failure("Failed to send SYNC WRITE: " + result.error());
        }
        return Result::success();
    }

    Result ScsProtocol::set_torque_enabled(const uint8_t id, const bool enabled) const {
        const std::array<uint8_t, 1> value{static_cast<uint8_t>(enabled ? 1 : 0)};
        return write_registers(id, kRegisterTorqueEnable, value);
    }

    Result ScsProtocol::set_eeprom_locked(const uint8_t id, const bool locked) const {
        const std::array<uint8_t, 1> value{static_cast<uint8_t>(locked ? 1 : 0)};
        return write_registers(id, kRegisterEepromLock, value);
    }

    Expected<uint16_t> ScsProtocol::read_word(const uint8_t id, const uint8_t address) const {
        std::array<uint8_t, 2> data{};
        if (const auto result = read_registers(id, address, data); !result) {
            return Expected<uint16_t>::failure(result.error());
        }
        return Expected<uint16_t>(decode_word(data.data()));
    }

    Expected<uint16_t> ScsProtocol::read_model(const uint8_t id) const {
        return read_word(id, kRegisterModel);
    }

    Expected<ScsServoIdentity> ScsProtocol::read_identity(const uint8_t id) const {
        // 身份块：固件主/次版本（0x00..0x01）、END（0x02）、舵机主/次版本（0x03..0x04）。
        std::array<uint8_t, 5> data{};
        if (const auto result = read_registers(id, kRegisterFirmwareMajor, data); !result) {
            return Expected<ScsServoIdentity>::failure(result.error());
        }
        ScsServoIdentity identity;
        identity.firmware_major = data[kRegisterFirmwareMajor];
        identity.firmware_minor = data[kRegisterFirmwareMinor];
        identity.model = static_cast<uint16_t>((data[kRegisterServoMajor] << 8) |
                                               data[kRegisterServoMinor]);
        return Expected<ScsServoIdentity>(identity);
    }

    Expected<ScsServoState> ScsProtocol::read_state(const uint8_t id) const {
        std::array<uint8_t, kFeedbackLength> feedback{};
        if (const auto result = read_registers(id, kRegisterPresentPosition, feedback); !result) {
            return Expected<ScsServoState>::failure(result.error());
        }
        return decode_feedback(feedback);
    }

    Result ScsProtocol::write_word(const uint8_t id, const uint8_t address,
                                   const uint16_t value) const {
        std::array<uint8_t, 2> data{};
        encode_word(data.data(), value);
        return write_registers(id, address, data);
    }

    Result ScsProtocol::reg_write(const uint8_t id, const uint8_t address,
                                  const std::span<const uint8_t> data) const {
        if (data.empty() || data.size() > 250) {
            return Result::failure("Invalid register write length " + std::to_string(data.size()));
        }
        std::vector<uint8_t> params;
        params.reserve(data.size() + 1);
        params.push_back(address);
        params.insert(params.end(), data.begin(), data.end());
        return transact(make_packet(id, kInstructionRegWrite, params), id, std::span<uint8_t>{});
    }

    Result ScsProtocol::reg_write_goal_position(const uint8_t id, const uint16_t position,
                                                const uint16_t time_ms,
                                                const uint16_t speed) const {
        std::array<uint8_t, 7> params{};
        params[0] = kRegisterGoalPosition;
        encode_word(params.data() + 1, position);
        encode_word(params.data() + 3, time_ms);
        encode_word(params.data() + 5, speed);
        return transact(make_packet(id, kInstructionRegWrite, params), id, std::span<uint8_t>{});
    }

    Result ScsProtocol::action(const uint8_t id) const {
        // ACTION 广播无应答：发送即完成。
        port_.flush_input();
        const auto packet = make_packet(id, kInstructionAction, std::span<const uint8_t>{});
        if (trace_hook_) {
            trace_hook_("tx", packet);
        }
        return port_.write(packet);
    }

    Result ScsProtocol::write_eeprom(const uint8_t id, const uint8_t address,
                                     const std::span<const uint8_t> data) const {
        if (const auto result = set_torque_enabled(id, false); !result) {
            return Result::failure("Disabling torque before EEPROM write failed: " +
                                   result.error());
        }
        if (const auto result = set_eeprom_locked(id, false); !result) {
            return Result::failure("Unlocking EEPROM failed: " + result.error());
        }
        const auto write_result = write_registers(id, address, data);
        // 写入失败也要回锁，避免舵机停留在解锁状态。
        const auto lock_result = set_eeprom_locked(id, true);
        if (!write_result) {
            return Result::failure("EEPROM write failed: " + write_result.error());
        }
        if (!lock_result) {
            return Result::failure("Locking EEPROM failed: " + lock_result.error());
        }
        return Result::success();
    }

    Expected<std::vector<uint8_t>> ScsProtocol::scan(const uint8_t first_id,
                                                     const uint8_t last_id) const {
        if (!port_.is_open()) {
            return Expected<std::vector<uint8_t>>::failure("Serial port is not open");
        }
        if (first_id > last_id) {
            return Expected<std::vector<uint8_t>>::failure(
                "Invalid scan range " + std::to_string(first_id) + ".." +
                std::to_string(last_id));
        }
        std::vector<uint8_t> ids;
        for (int id = first_id; id <= last_id; ++id) {
            if (ping(static_cast<uint8_t>(id))) {
                ids.push_back(static_cast<uint8_t>(id));
            }
        }
        return Expected<std::vector<uint8_t>>(ids);
    }

} // namespace feetech_scs_ros2_control
