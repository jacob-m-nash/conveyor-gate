#ifndef SWING_ARM_CONTROLLER_NODE_HPP
#define SWING_ARM_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono> 

class SwingArmControllerNode: public rclcpp::Node{
    public:
        SwingArmControllerNode();
    private:
        void commandCallback(const std_msgs::msg::String::ConstSharedPtr);
        void moveToPosition(double position);

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

        double swing_angle_;
        double debounce_duration_;
        double move_duration_;

        std::string last_command_;
        rclcpp::Time last_command_time_;

        
};

#endif