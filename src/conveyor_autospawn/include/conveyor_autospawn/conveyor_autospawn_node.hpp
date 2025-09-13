#ifndef CONVEYOR_AUTOSPAWN_NODE_HPP
#define CONVEYOR_AUTOSPAWN_NODE_HPP

#include <chrono>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "conveyorbelt_msgs/srv/conveyor_belt_control.hpp"


class ConveyorAutospawnNode : public rclcpp::Node
{
private:
    void wait_for_services();

    void send_conveyor_speed(int power);

    std::string load_box_urdf();

    void schedule_next_spawn();

    void spawn_once();

    // Parameters
    int speed_;
    double y_,z_,x_min_,x_max_, min_interval_s_,max_interval_s_;
    rclcpp::Client<conveyorbelt_msgs::srv::ConveyorBeltControl>::SharedPtr conveyor_client_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uni_interval_;
    std::uniform_real_distribution<double> uni_x_;
    std::size_t count_;
public:
    ConveyorAutospawnNode();

};
#endif
