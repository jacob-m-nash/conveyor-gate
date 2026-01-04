#include "swing_arm_controller_node/swing_arm_controller_node.hpp"

SwingArmControllerNode::SwingArmControllerNode():Node("swing_arm_controller_node"){
    this->declare_parameter("swing_angle", 0.523599); // 30 degrees
    this->declare_parameter("debounce_duration", 3.0);
    this->declare_parameter("move_duration", 0.5);

    swing_angle_ = this->get_parameter("swing_angle").as_double();
    debounce_duration_ = this->get_parameter("debounce_duration").as_double();
    move_duration_ = this->get_parameter("move_duration").as_double();


    joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);
    

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/detected_box_colour", 10,
        std::bind(&SwingArmControllerNode::commandCallback, this, std::placeholders::_1));
    
    last_command_ ="";
    last_command_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Swing Arm Controller Started");
}

void SwingArmControllerNode::commandCallback(const std_msgs::msg::String::ConstSharedPtr msg)
    {

        auto current_time = this->now();

        if(msg->data == last_command_ && (current_time - last_command_time_).seconds() < debounce_duration_){
            return;
        }

        last_command_ = msg->data;
        last_command_time_ = current_time;
        double target_position;
        
        if (msg->data == "red") {
            target_position = -swing_angle_;
            RCLCPP_INFO(this->get_logger(), "Moving to Position 1: %.2f rad", target_position);
        }
        else if (msg->data == "blue") {
            target_position = swing_angle_;
            RCLCPP_INFO(this->get_logger(), "Moving to Position 2: %.2f rad", target_position);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Invalid command: %s. Use red or blue.", msg->data.c_str());
            return;
        }
        
        moveToPosition(target_position);
    }
    
    void SwingArmControllerNode::moveToPosition(double position)
    {
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        
        trajectory_msg.joint_names.push_back("boom_link_JOINT_0");
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.push_back(position);
        point.velocities.push_back(0.0);  
        point.time_from_start = rclcpp::Duration::from_seconds(move_duration_); 
        
        trajectory_msg.points.push_back(point);
        
        joint_pub_->publish(trajectory_msg);
    }
    

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwingArmControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}