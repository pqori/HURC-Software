#ifndef HURC_ROVER_HARDWARE_STEPPER_SYSTEM_HPP_
#define HURC_ROVER_HARDWARE_STEPPER_SYSTEM_HPP_

#include <vector>

#include "hardware_interface/system_interface.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rover_hardware
{
class StepperSystemHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(StepperSystemHardware)

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
    rclcpp::Node::SharedPtr node_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> target_publishers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> current_subscriptions_;

    std::vector<double> tick_resolutions_;

    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_commands_; // positions
};
}


#endif