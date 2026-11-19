#pragma once

#include <chrono>
#include <functional>
#include <span>
#include <vector>

#include "feetech_scs_ros2_control/result.hpp"
#include "feetech_scs_ros2_control/scs_serial_port.hpp"
#include "feetech_scs_ros2_control/scs_state.hpp"

namespace feetech_scs_ros2_control {

    /// @brief 构造飞特协议数据包：
    ///   FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
    /// 其中 LENGTH 为指令、参数和校验和的字节数之和。
    /// @param id 目标舵机 id。
    /// @param instruction 指令码。
    /// @param params 指令参数。
    /// @return 完整的数据包字节序列。
    std::vector<uint8_t> make_packet(uint8_t id, uint8_t instruction,
                                     std::span<const uint8_t> params);

    /// @brief 计算协议使用的低字节校验和。
    /// @param id 舵机 id。
    /// @param length 数据长度字段。
    /// @param instruction 指令码。
    /// @param params 指令参数。
    uint8_t packet_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                            std::span<const uint8_t> params);

    /// @brief SCS（SCSCL）协议客户端，一个实例对应一条串口总线。
    class ScsProtocol {
    public:
        /// @brief 帧级调试钩子：收发方向（"tx"/"rx"）+ 原始字节。
        using TraceHook = std::function<void(const char* direction, std::span<const uint8_t> frame)>;

        /// @brief 构造函数。
        /// @param port 已打开的串口。
        /// @param response_timeout 等待单个舵机应答的超时时间。
        ScsProtocol(ScsSerialPort& port, std::chrono::milliseconds response_timeout);

        /// @brief 设置/清除帧级调试钩子；回调在调用线程同步触发。
        void set_trace_hook(TraceHook hook);

        /// @brief 发送 PING 并等待应答，用于检测舵机是否在线。
        Result ping(uint8_t id) const;

        /// @brief 从指定地址连续读取寄存器。
        /// @param id 舵机 id。
        /// @param address 寄存器起始地址。
        /// @param data 接收缓冲区，长度即要读取的字节数。
        Result read_registers(uint8_t id, uint8_t address, std::span<uint8_t> data) const;

        /// @brief 向指定地址连续写入寄存器，并等待应答。
        /// @param id 舵机 id。
        /// @param address 寄存器起始地址。
        /// @param data 待写入的数据。
        Result write_registers(uint8_t id, uint8_t address, std::span<const uint8_t> data) const;

        /// @brief 向单个舵机写入目标位置、运行时间和速度。
        /// @param id 舵机 id。
        /// @param position 目标位置，单位 tick。
        /// @param time_ms 运行时间，单位 ms；为 0 时由速度决定。
        /// @param speed 运行速度，单位 tick/s。
        Result write_goal_position(uint8_t id, uint16_t position, uint16_t time_ms,
                                   uint16_t speed) const;

        /// @brief 广播一次 SYNC WRITE，同时下发所有舵机的目标位置（地址 0x2A），
        ///        包含位置、时间和速度。
        /// @param ids 舵机 id 列表。
        /// @param positions 目标位置列表，单位 tick。
        /// @param time_ms 运行时间，单位 ms。
        /// @param speeds 运行速度列表，单位 tick/s。
        Result sync_write_goal_positions(std::span<const uint8_t> ids,
                                         std::span<const uint16_t> positions, uint16_t time_ms,
                                         std::span<const uint16_t> speeds) const;

        /// @brief 使能或关闭扭矩输出。
        Result set_torque_enabled(uint8_t id, bool enabled) const;

        /// @brief 锁定或解锁 EEPROM；解锁后才能修改 EEPROM 参数。
        Result set_eeprom_locked(uint8_t id, bool locked) const;

        /// @brief 读取高字节在前的 16 位寄存器。
        Expected<uint16_t> read_word(uint8_t id, uint8_t address) const;

        /// @brief 读取 kRegisterModel 上的 16 位型号/版本标识。
        Expected<uint16_t> read_model(uint8_t id) const;

        /// @brief 读取身份信息（固件版本 + 型号，寄存器 0x00..0x04）。
        /// @param id 舵机 id。
        Expected<ScsServoIdentity> read_identity(uint8_t id) const;

        /// @brief 读取并解码反馈状态块（寄存器 0x38..0x46）。
        /// @param id 舵机 id。
        Expected<ScsServoState> read_state(uint8_t id) const;

        /// @brief 写入单个 16 位寄存器（高字节在前）。
        /// @param id 舵机 id。
        /// @param address 寄存器地址。
        /// @param value 待写入的值。
        Result write_word(uint8_t id, uint8_t address, uint16_t value) const;

        /// @brief EEPROM 安全写流程：关扭矩 → 解锁 → 写入 → 回锁。
        ///        写入失败时仍会尝试回锁。
        /// @param id 舵机 id。
        /// @param address EEPROM 寄存器地址。
        /// @param data 待写入的数据。
        Result write_eeprom(uint8_t id, uint8_t address, std::span<const uint8_t> data) const;

        /// @brief 扫描区间内在线的舵机，逐个 PING 并收集应答的 id。
        /// @param first_id 起始 id（含）。
        /// @param last_id 结束 id（含）。
        /// @return 在线 id 列表；串口未打开或区间非法时返回错误信息。
        Expected<std::vector<uint8_t>> scan(uint8_t first_id = 0,
                                            uint8_t last_id = kMaxServoId) const;

    private:
        /// @brief 发送指令并校验应答的帧头、长度、校验和与状态字节。
        /// @param packet 已组好的指令包。
        /// @param expected_id 期望应答的舵机 id。
        /// @param payload 应答数据缓冲区。
        Result transact(const std::vector<uint8_t>& packet, uint8_t expected_id,
                        std::span<uint8_t> payload) const;

        ScsSerialPort& port_;
        std::chrono::milliseconds response_timeout_;
        TraceHook trace_hook_;
    };

} // namespace feetech_scs_ros2_control
