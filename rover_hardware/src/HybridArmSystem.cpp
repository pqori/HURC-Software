#include "rover_hardware/HybridArmSystem.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>

// #include "rover_hardware/EPOSDefinitions.h"
#include "rover_hardware/EPOS.hpp"

#define DICT_HAS(dict, key) (dict.find(key) != dict.end())

inline float rads_to_rpm(float ang_vel) {
    return ang_vel * 60.0 / (2.0 * M_PI);
}

inline float rpm_to_rads(float rpm) {
    return rpm * 2.0 * M_PI / 60.0;
}

namespace rover_hardware
{

hardware_interface::CallbackReturn HybridArmSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("HybridArmSystem"));
    RCLCPP_INFO(*logger_, "Starting custom HURC hardware plugin!");

    // for (uint i = 0; i < info_.joints.size(); i++) {
    //     auto joint = info_.joints[i];
    //     RCLCPP_INFO(*logger_, "Got joint %s w/ type %s", joint.name.c_str(), joint.type.c_str());
    //     if (joint.parameters.find("is_left_wheel") != joint.parameters.end()) {
    //         if (hardware_interface::parse_bool(joint.parameters["is_left_wheel"])) {
    //             left_wheels_.emplace_back(i);
    //             RCLCPP_INFO(*logger_, "Added to left wheels");
    //         } else {
    //             right_wheels_.emplace_back(i);
    //             RCLCPP_INFO(*logger_, "Added to right wheels");
    //         }
    //     }
    // }

    epos_motors_.resize(info_.joints.size(), nullptr);
    motor_target_scales_.resize(info_.joints.size(), 0);
    for (uint i = 0; i < info_.joints.size(); i++) {
        if (!DICT_HAS(info_.joints[i].parameters, "position_target_scale")) {
            RCLCPP_WARN(*logger_, "Joint %s does not have position_target_scale, skipping...", info_.joints[i].name.c_str());
            continue;
        }
        motor_target_scales_[i] = hardware_interface::stod(info_.joints[i].parameters["position_target_scale"]);
        if (DICT_HAS(info_.joints[i].parameters, "differential_index")) {
            uint diff_idx = uint(hardware_interface::stod(info_.joints[i].parameters["differential_index"]));
            RCLCPP_INFO(*logger_, "Differential part %d set to joint %s", diff_idx, info_.joints[i].name.c_str());
            if (diff_idx == 0) {
                left_diff_idx_ = i;
            } else if (diff_idx == 1) {
                right_diff_idx_ = i;
            } else {
                RCLCPP_WARN(*logger_, "Differential index %d out of range", diff_idx);
            }
        }
        if (DICT_HAS(info_.joints[i].parameters, "epos_node_id")) {
            uint node = uint(hardware_interface::stod(info_.joints[i].parameters["epos_node_id"]));
            RCLCPP_INFO(*logger_, "Joint %s has epos node id %d", info_.joints[i].name.c_str(), node);
            epos_motors_[i] = new EPOS::Motor(&epos_handle_, node);
        }
    }

    for (uint i = 0; i < epos_motors_.size(); i++) {
        if (epos_motors_[i]) {
            RCLCPP_INFO(*logger_, "Joint index %d is EPOS", i);
        }
    }

    joint_state_positions_.resize(info_.joints.size());
    joint_state_velocities_.resize(info_.joints.size());
    joint_command_positions_.resize(info_.joints.size());
    joint_command_velocities_.resize(info_.joints.size());

    // node_ = rclcpp::Node::make_shared("topic_drive_system", "drive");
    // left_rpm_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("left_rpm", rclcpp::QoS(1));
    // right_rpm_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("right_rpm", rclcpp::QoS(1));

    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn HybridArmSystemHardware::on_configure(const rclcpp_lifecycle::State&) {
    for (uint i = 0; i < joint_state_positions_.size(); i++) {
        joint_state_positions_[i] = 0;
        joint_state_velocities_[i] = 0;
        joint_command_positions_[i] = 0;
        joint_command_velocities_[i] = 0;
    }

    unsigned int error = 0;
    epos_handle_ = EPOS::OpenDevice("EPOS4", "MAXON SERIAL V2", "USB", "USB0", &error);
    if (epos_handle_ == nullptr || error != 0) {
        RCLCPP_FATAL(*logger_, "Could not open EPOS device: error code %d", error);
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
};

std::vector<hardware_interface::StateInterface> HybridArmSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocities_[i]));
    }
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface> HybridArmSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_command_positions_[i]));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_velocities_[i]));
    }
    return command_interfaces;
};

hardware_interface::CallbackReturn HybridArmSystemHardware::on_activate(const rclcpp_lifecycle::State&) {
    bool all_success = true;
    for (EPOS::Motor* motor : epos_motors_) {
        if (motor) {
            if (!motor->enable()) {
                RCLCPP_WARN(*logger_, "Could not enable motor with node id %d due to error %x", motor->nodeId(), motor->lastError());
                all_success = false;
            }
        }
    }
    if (!all_success) {
        return hardware_interface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(*logger_, "Successfully enabled all EPOS motors");

    all_success = true;
    for (EPOS::Motor* motor : epos_motors_) {
        if (motor) {
            if (!motor->enableProfilePositionMode()) {
                RCLCPP_WARN(*logger_, "Could not enable position mode on motor with node id %d", motor->nodeId());
                all_success = false;
            }
        }
    }
    if (!all_success) {
        return hardware_interface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(*logger_, "Successfully set all EPOS motors to position mode");

    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn HybridArmSystemHardware::on_deactivate(const rclcpp_lifecycle::State&) {
    for (EPOS::Motor* motor : epos_motors_) {
        if (motor) {
            if (!motor->halt()) {
                RCLCPP_WARN(*logger_, "Failed halting motor with node id %d", motor->nodeId());
            }
        }
    }

    for (EPOS::Motor* motor : epos_motors_) {
        if (motor) {
            if (!motor->disable()) {
                RCLCPP_WARN(*logger_, "Failed disabling motor with node id %d", motor->nodeId());
            }
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::return_type HybridArmSystemHardware::read(const rclcpp::Time&, const rclcpp::Duration&) {
    {
        int current0;
        int current1;
        if (epos_motors_[left_diff_idx_]->getPosition(&current0) &&
            epos_motors_[right_diff_idx_]->getPosition(&current1)) {
            double m0 = double(current0) / motor_target_scales_[left_diff_idx_] * (2.0 * M_PI);
            double m1 = double(current1) / motor_target_scales_[right_diff_idx_] * (2.0 * M_PI);
            float j0 = (m0/2.0) - (m1/2.0);
            float j1 = (m0/2.0) + (m1/2.0);
            // RCLCPP_INFO(*logger_, "Converted motor angles %f, %f to joint angles %f, %f", m0, m1, j0, j1);
            joint_state_positions_[left_diff_idx_] = j0;
            joint_state_positions_[right_diff_idx_] = j1;
        } else {
            RCLCPP_WARN(*logger_, "Failed to get current position of at least one differential motor");
        }
    }
    for (uint i = 0; i < epos_motors_.size(); i++) {
        if (i == left_diff_idx_ || i == right_diff_idx_) continue;
        if (epos_motors_[i]) {
            int currentPos;
            if (epos_motors_[i]->getPosition(&currentPos)) {
                double newPos = double(currentPos) / motor_target_scales_[i] * (2.0 * M_PI);
                if (joint_state_positions_[i] != newPos) {
                    RCLCPP_INFO(*logger_, "Got new raw position of %d from EPOS motor %d", currentPos, epos_motors_[i]->nodeId());
                }
                joint_state_positions_[i] = newPos;
            } else {
                RCLCPP_WARN(*logger_, "Failed to get current position from EPOS motor %d", epos_motors_[i]->nodeId());
                return hardware_interface::return_type::ERROR;
            }
        }
    }
    return hardware_interface::return_type::OK;
};

hardware_interface::return_type HybridArmSystemHardware::write(const rclcpp::Time&, const rclcpp::Duration& dt) {
    RCLCPP_INFO(*logger_, "Write after %f ms", dt.seconds() * 1000);
    {
        float j0 = joint_command_positions_[left_diff_idx_];
        float j1 = joint_command_positions_[right_diff_idx_];
        float m0 = j0 + j1;
        float m1 = -j0 + j1;
        // RCLCPP_INFO(*logger_, "Converted joint angle %f, %f to motor angles %f, %f", j0, j1, m0, m1);
        long scaled_target0 = m0 * motor_target_scales_[left_diff_idx_] / (2.0 * M_PI);
        long scaled_target1 = m1 * motor_target_scales_[right_diff_idx_] / (2.0 * M_PI);
        if (!epos_motors_[left_diff_idx_]->setPosition(scaled_target0, true, true)) {
            RCLCPP_WARN(*logger_, "Failed sending target to motor with node id %d", epos_motors_[left_diff_idx_]->nodeId());
            return hardware_interface::return_type::ERROR;
        }
        if (!epos_motors_[right_diff_idx_]->setPosition(scaled_target1, true, true)) {
            RCLCPP_WARN(*logger_, "Failed sending target to motor with node id %d", epos_motors_[right_diff_idx_]->nodeId());
            return hardware_interface::return_type::ERROR;
        }
    }
    for (uint i = 0; i < joint_command_positions_.size(); i++) {
        if (joint_command_positions_[i] != joint_state_positions_[i]) {
            if (i == left_diff_idx_ || i == right_diff_idx_) continue;
            RCLCPP_INFO(*logger_, "Command on motor %d: %f", i, joint_command_positions_[i]);
            if (epos_motors_[i]) {
                long scaled_target = joint_command_positions_[i] * motor_target_scales_[i] / (2.0 * M_PI);
                RCLCPP_INFO(*logger_, "Sending scaled target %ld to EPOS node %d", scaled_target, epos_motors_[i]->nodeId());
                if (!epos_motors_[i]->setPosition(scaled_target, true, true)) {
                    RCLCPP_WARN(*logger_, "Failed sending target to motor with node id %d", epos_motors_[i]->nodeId());
                    return hardware_interface::return_type::ERROR;
                } else {
                    joint_state_positions_[i] = joint_command_positions_[i];
                }
            } else {
                joint_state_positions_[i] = joint_command_positions_[i];
            }
        }
    }
    // float left_total = 0;
    // float right_total = 0;
    // for (uint i : left_wheels_) {
    //     left_total += joint_commands_[i];
    //     RCLCPP_INFO(*logger_, "Got left vel %f", joint_commands_[i]);
    // }
    // for (uint i : right_wheels_) {
    //     right_total += joint_commands_[i];
    //     RCLCPP_INFO(*logger_, "Got right vel %f", joint_commands_[i]);
    // }
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

PLUGINLIB_EXPORT_CLASS(rover_hardware::HybridArmSystemHardware, hardware_interface::SystemInterface)