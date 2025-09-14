#include "conveyor_autospawn/conveyor_autospawn_node.hpp"
#include <tinyxml2.h>
#include <fstream>
#include <sstream>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <random>

using namespace std::chrono_literals;
using namespace tinyxml2;

ConveyorAutospawnNode::ConveyorAutospawnNode()
    :Node("conveyor_autospawn_node"),
    rng_(std::random_device{}()),
    count_(0)
    {
        speed_ = this->declare_parameter("speed",20);
        y_ = this->declare_parameter("y",-0.5);
        z_ = this->declare_parameter("z", 0.76);
        x_min_ = this->declare_parameter("x_min", -0.05);
        x_max_ = this->declare_parameter("x_max",  0.05);
        min_interval_s_ = this->declare_parameter("min_interval_s", 2.0);
        max_interval_s_ = this->declare_parameter("max_interval_s", 6.0);

        uni_interval_ = std::uniform_real_distribution<double>(min_interval_s_, max_interval_s_);
        uni_x_ = std::uniform_real_distribution<double>(x_min_,x_max_);
        conveyor_client_ = this->create_client<conveyorbelt_msgs::srv::ConveyorBeltControl>("/CONVEYORPOWER");
        spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        wait_for_services();
        send_conveyor_speed(speed_);
        RCLCPP_WARN(this->get_logger(),"conveyor speed: %i", speed_);
        schedule_next_spawn();
    }

    void ConveyorAutospawnNode::wait_for_services(){
        RCLCPP_INFO(get_logger(), "Waiting for services");
        while(!conveyor_client_-> wait_for_service(10s)){
            if(!rclcpp::ok()) throw std::runtime_error("Interrupted while waiting for conveyor service");
        }

        while(!spawn_client_-> wait_for_service(10s)){
            if(!rclcpp::ok()) throw std::runtime_error("Interrupted while waiting for spawn service");
        }
    }

    void ConveyorAutospawnNode::send_conveyor_speed (int power){
        auto req = std::make_shared<conveyorbelt_msgs::srv::ConveyorBeltControl::Request>();
        req->power = power;
        auto future = conveyor_client_->async_send_request(req);
        if(future.wait_for(10s) == std::future_status::ready){
            RCLCPP_INFO(get_logger(), "Conveyor set to %.2d", power);
        }else{
            RCLCPP_WARN(get_logger(), "Timed out setting conveyor");
        }
    }

    std::unique_ptr<XMLDocument> ConveyorAutospawnNode::load_box_urdf(){
        const std::string pkg_share = ament_index_cpp::get_package_share_directory("conveyorbelt_gazebo");
        const std::string urdf_path = pkg_share + "/urdf/box.urdf";
        auto doc = std::make_unique<XMLDocument>();
        if(doc->LoadFile(urdf_path.c_str()) != XML_SUCCESS){
            throw std::runtime_error("Cannot open URDF" + urdf_path);
        }
        return doc;
   }

   void ConveyorAutospawnNode::randomise_box_colour(XMLDocument& doc){
    std::uniform_int_distribution<int> pick(0,1);
    const char* name = pick(rng_) == 0 ? "Gazebo/Red" : "Gazebo/Blue";
    XMLElement* material = doc.FirstChildElement("robot")
                                 ->FirstChildElement("gazebo")
                                 ->FirstChildElement("material");
    if(material && material->GetText()){
        material->SetText(name);
    }
   }

    void ConveyorAutospawnNode::schedule_next_spawn(){
        const double dt = uni_interval_(rng_);
        auto ms = std::chrono::milliseconds(static_cast<int>(dt * 1000.0));
        timer_ = this->create_wall_timer(ms, [this](){
            this->spawn_once();
        });
    }

    void ConveyorAutospawnNode::spawn_once(){
        timer_ -> cancel();
        const double x = uni_x_(rng_);
        auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        auto doc = load_box_urdf();
        randomise_box_colour(*doc);
        XMLPrinter printer;
        doc->Print(&printer);
        req->name = "box_" + std::to_string(count_++);
        req -> xml = std::string(printer.CStr()); 
        req -> initial_pose.position.x = x;
        req -> initial_pose.position.y = y_;
        req -> initial_pose.position.z = z_;
        req -> initial_pose.orientation.w = 1.0;
        req-> reference_frame = "world";

        auto self = shared_from_this();
        spawn_client_ -> async_send_request(req,
        [this,self](rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture future){
            auto resp = future.get();
            if(!resp->success){
                RCLCPP_WARN(this -> get_logger(), "Spawn failed: %s", resp -> status_message.c_str());
            }
            this -> schedule_next_spawn();
        });
        }

    int main(int argc, char ** argv){
        rclcpp::init(argc, argv);
        auto node = std::make_shared<ConveyorAutospawnNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }