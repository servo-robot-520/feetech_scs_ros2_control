#include <gtest/gtest.h>

#include <hardware_interface/system_interface.hpp>

#include <exception>
#include <pluginlib/class_loader.hpp>


TEST(PluginLoading, LoadsFeetechScsHardwareInterface) {
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface", "hardware_interface::SystemInterface");

    pluginlib::UniquePtr<hardware_interface::SystemInterface> instance;
    try {
        instance = loader.createUniqueInstance(
            "feetech_scs_servo_ros2_control/FeetechScsHardwareInterface");
    }
    catch (const std::exception& error) {
        FAIL() << error.what();
    }
    EXPECT_TRUE(instance != nullptr);
}
