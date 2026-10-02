#ifndef HURC_ROVER_HARDWARE_HYBRID_ARM_SYSTEM_HPP_
#define HURC_ROVER_HARDWARE_HYBRID_ARM_SYSTEM_HPP_

#include <vector>

#include "hardware_interface/system_interface.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <std_msgs/msg/int32.hpp>

#include "EPOS.hpp"

namespace rover_hardware
{

class HybridArmSystemHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(HybridArmSystemHardware)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<rclcpp::Logger> logger_;
    // rclcpp::Node::SharedPtr node_;
    // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_rpm_publisher_;
    // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_rpm_publisher_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_rpm_

    // std::vector<uint> left_wheels_;
    // std::vector<uint> right_wheels_;

    void* epos_handle_;
    std::vector<EPOS::Motor*> epos_motors_;
    std::vector<double> motor_target_scales_;
    uint left_diff_idx_;
    uint right_diff_idx_;

    std::vector<double> joint_state_positions_;
    std::vector<double> joint_state_velocities_;
    std::vector<double> joint_command_positions_;
    std::vector<double> joint_command_velocities_;
};

}


#endif