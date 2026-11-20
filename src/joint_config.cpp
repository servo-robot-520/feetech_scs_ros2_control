#include "feetech_scs_ros2_control/joint_config.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

namespace feetech_scs_ros2_control {

    Result load_joint_config(const std::string& file_path, JointConfigFile* const config) {
        if (config == nullptr) {
            return Result::failure("load_joint_config: output is null");
        }
        *config = JointConfigFile{};

        if (std::error_code error; !std::filesystem::exists(file_path, error)) {
            std::string message = "joint_config_file '";
            message += file_path;
            message += "' does not exist";
            return Result::failure(std::move(message));
        }

        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::string message = "Failed to open joint_config_file '";
            message += file_path;
            message += '\'';
            return Result::failure(std::move(message));
        }

        try {
            const YAML::Node root = YAML::Load(file);

            // 顶层总线参数可覆盖 URDF 中的同名硬件参数。
            if (const YAML::Node usb_port = root["usb_port"]; usb_port) {
                if (!usb_port.IsScalar() || usb_port.Scalar().empty()) {
                    std::string message = "joint_config_file '";
                    message += file_path;
                    message += "': 'usb_port' must be a non-empty string";
                    return Result::failure(std::move(message));
                }
                config->usb_port = usb_port.Scalar();
            }

            if (const YAML::Node baud_rate = root["baud_rate"]; baud_rate) {
                if (!baud_rate.IsScalar()) {
                    std::string message = "joint_config_file '";
                    message += file_path;
                    message += "': 'baud_rate' must be a scalar";
                    return Result::failure(std::move(message));
                }
                config->baud_rate = baud_rate.Scalar();
            }

            // 关节名仅作标签，驱动按 id 将配置匹配到 URDF 关节。
            const YAML::Node joints = root["joints"];
            if (joints && !joints.IsMap()) {
                std::string message = "joint_config_file '";
                message += file_path;
                message += "': top-level 'joints' must be a map";
                return Result::failure(std::move(message));
            }

            for (auto it = joints.begin(); it != joints.end(); ++it) {
                const auto joint_name = it->first.as<std::string>();
                const YAML::Node joint_node = it->second;
                if (!joint_node.IsMap()) {
                    std::string message = "joint_config_file '";
                    message += file_path;
                    message += "': joint '";
                    message += joint_name;
                    message += "' is not a map";
                    return Result::failure(std::move(message));
                }

                JointParams params;
                for (auto param = joint_node.begin(); param != joint_node.end(); ++param) {
                    const auto key = param->first.as<std::string>();
                    if (!param->second.IsScalar()) {
                        std::string message = "joint_config_file '";
                        message += file_path;
                        message += "': parameter '";
                        message += key;
                        message += "' of joint '";
                        message += joint_name;
                        message += "' is not a scalar";
                        return Result::failure(std::move(message));
                    }
                    params[key] = param->second.Scalar();
                }
                config->joints.insert_or_assign(joint_name, std::move(params));
            }
        }
        catch (const YAML::Exception& error) {
            std::string message = "YAML parsing error in '";
            message += file_path;
            message += "': ";
            message += error.what();
            return Result::failure(std::move(message));
        }
        catch (const std::exception& error) {
            std::string message = "Failed to load joint_config_file '";
            message += file_path;
            message += "': ";
            message += error.what();
            return Result::failure(std::move(message));
        }

        return Result::success();
    }

    JointParams merge_joint_params(const JointParams& yaml_params, const JointParams& urdf_params) {
        JointParams merged = urdf_params;
        for (const auto& [key, value] : yaml_params) {
            merged[key] = value;
        }
        return merged;
    }

    Result load_calibration_file(const std::string& file_path, CalibrationFile* const calibration) {
        if (calibration == nullptr) {
            return Result::failure("load_calibration_file: output is null");
        }
        *calibration = CalibrationFile{};

        if (std::error_code error; !std::filesystem::exists(file_path, error)) {
            std::string message = "calibration file '";
            message += file_path;
            message += "' does not exist";
            return Result::failure(std::move(message));
        }

        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::string message = "Failed to open calibration file '";
            message += file_path;
            message += '\'';
            return Result::failure(std::move(message));
        }

        try {
            const YAML::Node root = YAML::Load(file);

            if (const YAML::Node version = root["version"]; version) {
                if (!version.IsScalar()) {
                    std::string message = "calibration file '";
                    message += file_path;
                    message += "': 'version' must be a scalar";
                    return Result::failure(std::move(message));
                }
                try {
                    calibration->version = version.as<int>();
                }
                catch (const YAML::Exception&) {
                    std::string message = "calibration file '";
                    message += file_path;
                    message += "': 'version' must be an integer";
                    return Result::failure(std::move(message));
                }
            }

            const YAML::Node joints = root["joints"];
            if (!joints || !joints.IsMap()) {
                std::string message = "calibration file '";
                message += file_path;
                message += "': top-level 'joints' must be a map";
                return Result::failure(std::move(message));
            }

            for (auto it = joints.begin(); it != joints.end(); ++it) {
                const auto joint_name = it->first.as<std::string>();
                const YAML::Node joint_node = it->second;
                if (!joint_node.IsMap()) {
                    std::string message = "calibration file '";
                    message += file_path;
                    message += "': joint '";
                    message += joint_name;
                    message += "' is not a map";
                    return Result::failure(std::move(message));
                }
                const YAML::Node tick = joint_node["calibration_tick"];
                if (!tick || !tick.IsScalar()) {
                    std::string message = "calibration file '";
                    message += file_path;
                    message += "': joint '";
                    message += joint_name;
                    message += "' has no scalar 'calibration_tick'";
                    return Result::failure(std::move(message));
                }
                int value = 0;
                try {
                    value = tick.as<int>();
                }
                catch (const YAML::Exception&) {
                    std::string message = "calibration file '";
                    message += file_path;
                    message += "': joint '";
                    message += joint_name;
                    message += "' 'calibration_tick' must be an integer";
                    return Result::failure(std::move(message));
                }
                if (value < 0 || value > 1023) {
                    std::string message = "calibration file '";
                    message += file_path;
                    message += "': joint '";
                    message += joint_name;
                    message += "' 'calibration_tick' out of range [0, 1023]: ";
                    message += std::to_string(value);
                    return Result::failure(std::move(message));
                }
                calibration->calibration_ticks.insert_or_assign(joint_name, static_cast<uint16_t>(value));
            }
        }
        catch (const YAML::Exception& error) {
            std::string message = "YAML parsing error in '";
            message += file_path;
            message += "': ";
            message += error.what();
            return Result::failure(std::move(message));
        }
        catch (const std::exception& error) {
            std::string message = "Failed to load calibration file '";
            message += file_path;
            message += "': ";
            message += error.what();
            return Result::failure(std::move(message));
        }

        return Result::success();
    }

} // namespace feetech_scs_ros2_control
