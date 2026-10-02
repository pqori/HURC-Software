#include "rover_hardware/StepperSystem.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>

#define DICT_HAS(dict, key) (dict.find(key) != dict.end())

inline float rads_to_rpm(float ang_vel) {
    return ang_vel * 60.0 / (2.0 * M_PI) * 43.0;
}

inline float rpm_to_rads(float rpm) {
    return rpm * 2.0 * M_PI / 60.0 / 43.0;
}

namespace rover_hardware
{

hardware_interface::CallbackReturn StepperSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("StepperSystem"));
    RCLCPP_INFO(*logger_, "Starting custom HURC hardware plugin!");

    node_ = rclcpp::Node::make_shared("stepper_system");

    for (uint i = 0; i < info_.joints.size(); i++) {
        auto joint = info_.joints[i];
        RCLCPP_INFO(*logger_, "Got joint %s w/ type %s", joint.name.c_str(), joint.type.c_str());
        if (DICT_HAS(joint.parameters, "topic_name") && DICT_HAS(joint.parameters, "tick_resolution")) {
            tick_resolutions_.emplace_back(hardware_interface::stod(joint.parameters["tick_resolution"]));
            char pub_name[256];
            char sub_name[256];
            sprintf(pub_name, "/arm/set_%s", joint.parameters["topic_name"].c_str());
            sprintf(sub_name, "/arm/get_%s", joint.parameters["topic_name"].c_str());
            target_publishers_.emplace_back(node_->create_publisher<std_msgs::msg::Int32>(pub_name, rclcpp::QoS(5).reliable()));
            current_subscriptions_.emplace_back(node_->create_subscription<std_msgs::msg::Int32>(sub_name, rclcpp::QoS(5).reliable(), 
                [this, &joint, i](std_msgs::msg::Int32::SharedPtr msg) {
                    joint_positions_[i] = (msg->data / tick_resolutions_[i]) * (2.0 * M_PI);
                }));
        } else {
            RCLCPP_ERROR(*logger_, "Joint %s not configured properly", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        
    }

    joint_positions_.resize(info_.joints.size());
    joint_velocities_.resize(info_.joints.size());
    joint_commands_.resize(info_.joints.size());

    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn StepperSystemHardware::on_configure(const rclcpp_lifecycle::State&) {
    for (uint i = 0; i < joint_positions_.size(); i++) {
        joint_positions_[i] = 0;
        joint_velocities_[i] = 0;
        joint_commands_[i] = 0;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
};

std::vector<hardware_interface::StateInterface> StepperSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    }
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface> StepperSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]));
    }
    return command_interfaces;
};

hardware_interface::CallbackReturn StepperSystemHardware::on_activate(const rclcpp_lifecycle::State&) {
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn StepperSystemHardware::on_deactivate(const rclcpp_lifecycle::State&) {
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::return_type StepperSystemHardware::read(const rclcpp::Time&, const rclcpp::Duration&) {
    rclcpp::spin_some(node_);
    // for (uint i = 0; i < joint_positions_.size(); i++) {
        // joint_velocities_[i] = joint_commands_[i];
        // joint_positions_[i] += joint_velocities_[i] * dt.seconds();
    // }
    return hardware_interface::return_type::OK;
};

hardware_interface::return_type StepperSystemHardware::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (uint i = 0; i < joint_commands_.size(); i++) {
        std_msgs::msg::Int32 target_msg;
        target_msg.data = int(joint_commands_[i] * tick_resolutions_[i] / (2.0 * M_PI));
        target_publishers_[i]->publish(target_msg);
    }
    // std_msgs::msg::Int32 left_msg;
    // left_msg.data = int(rads_to_rpm(left_total / left_wheels_.size()));
    // std_msgs::msg::Int32 right_msg;
    // right_msg.data = int(rads_to_rpm(right_total / left_wheels_.size()));
    // left_rpm_publisher_->publish(left_msg);
    // right_rpm_publisher_->publish(right_msg);
    // for (uint i = 0; i < joint_positions_.size(); i++) {
    //     joint_velocities_[i] = joint_commands_[i];
    // }
    return hardware_interface::return_type::OK;
};

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rover_hardware::StepperSystemHardware, hardware_interface::SystemInterface)