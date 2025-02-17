#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class MicroROSHardwareInterface : public hardware_interface::SystemInterface {
public:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double wheel_speeds_[4] = {0.0, 0.0, 0.0, 0.0};

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
        node_ = std::make_shared<rclcpp::Node>("micro_ros_interface");

        // Subscribe to the odometry topic published by the ESP32
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Update wheel speeds based on received odometry data
                wheel_speeds_[0] = msg->twist.twist.linear.x - msg->twist.twist.angular.z;
                wheel_speeds_[1] = msg->twist.twist.linear.x + msg->twist.twist.angular.z;
                wheel_speeds_[2] = msg->twist.twist.linear.x - msg->twist.twist.angular.z;
                wheel_speeds_[3] = msg->twist.twist.linear.x + msg->twist.twist.angular.z;
            });

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        return {
            {"front_left_wheel_joint", "velocity", &wheel_speeds_[0]},
            {"front_right_wheel_joint", "velocity", &wheel_speeds_[1]},
            {"rear_left_wheel_joint", "velocity", &wheel_speeds_[2]},
            {"rear_right_wheel_joint", "velocity", &wheel_speeds_[3]}
        };
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        return {}; // No command interfaces are needed since the ESP32 handles /cmd_vel
    }

    // Implement the read function (even if it's not doing anything specific)
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // This function could read data from the hardware if needed, but we can leave it empty
        return hardware_interface::return_type::OK;
    }

    // Implement the write function (even if it's not doing anything specific)
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // This function could write data to the hardware if needed, but we can leave it empty
        return hardware_interface::return_type::OK;
    }
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MicroROSHardwareInterface, hardware_interface::SystemInterface)
