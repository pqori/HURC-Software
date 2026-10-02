#include "rover_hardware/TopicDriveSystem.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>

inline float rads_to_rpm(float ang_vel) {
    return ang_vel * 60.0 / (2.0 * M_PI) * 43.0;
}

inline float rpm_to_rads(float rpm) {
    return rpm * 2.0 * M_PI / 60.0 / 43.0;
}

namespace rover_hardware
{

hardware_interface::CallbackReturn TopicDriveSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TopicDriveSystem"));
    RCLCPP_INFO(*logger_, "Starting custom HURC hardware plugin!");

    for (uint i = 0; i < info_.joints.size(); i++) {
        auto joint = info_.joints[i];
        RCLCPP_INFO(*logger_, "Got joint %s w/ type %s", joint.name.c_str(), joint.type.c_str());
        if (joint.parameters.find("is_left_wheel") != joint.parameters.end()) {
            if (hardware_interface::parse_bool(joint.parameters["is_left_wheel"])) {
                left_wheels_.emplace_back(i);
                RCLCPP_INFO(*logger_, "Added to left wheels");
            } else {
                right_wheels_.emplace_back(i);
                RCLCPP_INFO(*logger_, "Added to right wheels");
            }
        }
    }

    joint_positions_.resize(info_.joints.size());
    joint_velocities_.resize(info_.joints.size());
    joint_commands_.resize(info_.joints.size());

    node_ = rclcpp::Node::make_shared("topic_drive_system", "drive");
    left_rpm_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("left_set_rpm", rclcpp::QoS(1));
    right_rpm_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("right_set_rpm", rclcpp::QoS(1));
    left_rpm_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>("left_get_rpm", rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](std_msgs::msg::Float32::SharedPtr msg) {
            RCLCPP_INFO(*this->logger_, "Got left rpm of %f", msg->data);
            for (uint i : left_wheels_) {
                joint_velocities_[i] = rpm_to_rads(msg->data);
            }
        });
    right_rpm_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>("right_get_rpm", rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](std_msgs::msg::Float32::SharedPtr msg) {
            RCLCPP_INFO(*this->logger_, "Got right rpm of %f", msg->data);
            for (uint i : right_wheels_) {
                joint_velocities_[i] = rpm_to_rads(msg->data);
            }
        });

    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn TopicDriveSystemHardware::on_configure(const rclcpp_lifecycle::State&) {
    for (uint i = 0; i < joint_positions_.size(); i++) {
        joint_positions_[i] = 0;
        joint_velocities_[i] = 0;
        joint_commands_[i] = 0;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
};

std::vector<hardware_interface::StateInterface> TopicDriveSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    }
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface> TopicDriveSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_commands_[i]));
    }
    return command_interfaces;
};

hardware_interface::CallbackReturn TopicDriveSystemHardware::on_activate(const rclcpp_lifecycle::State&) {
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn TopicDriveSystemHardware::on_deactivate(const rclcpp_lifecycle::State&) {
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::return_type TopicDriveSystemHardware::read(const rclcpp::Time&, const rclcpp::Duration& dt) {
    rclcpp::spin_some(node_);
    for (uint i = 0; i < joint_positions_.size(); i++) {
        // joint_velocities_[i] = joint_commands_[i];
        joint_positions_[i] += joint_velocities_[i] * dt.seconds();
    }
    return hardware_interface::return_type::OK;
};

hardware_interface::return_type TopicDriveSystemHardware::write(const rclcpp::Time&, const rclcpp::Duration&) {
    float left_total = 0;
    float right_total = 0;
    for (uint i : left_wheels_) {
        left_total += joint_commands_[i];
        // RCLCPP_INFO(*logger_, "Got left vel %f", joint_commands_[i]);
    }
    for (uint i : right_wheels_) {
        right_total += joint_commands_[i];
        // RCLCPP_INFO(*logger_, "Got right vel %f", joint_commands_[i]);
    }
    std_msgs::msg::Int32 left_msg;
    left_msg.data = int(rads_to_rpm(left_total / left_wheels_.size()));
    std_msgs::msg::Int32 right_msg;
    right_msg.data = int(rads_to_rpm(right_total / left_wheels_.size()));
    left_rpm_publisher_->publish(left_msg);
    right_rpm_publisher_->publish(right_msg);
    // for (uint i = 0; i < joint_positions_.size(); i++) {
    //     joint_velocities_[i] = joint_commands_[i];
    // }
    return hardware_interface::return_type::OK;
};

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rover_hardware::TopicDriveSystemHardware, hardware_interface::SystemInterface)