#ifndef COLOR_DETECTOR_NODE_HPP
#define COLOR_DETECTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
class BoxColorDetectorNode: public rclcpp::Node{
    public:
        BoxColorDetectorNode();
    private:
        void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        std::string image_topic_;
        double min_fraction_;

        image_transport::Subscriber img_sub_;
        image_transport::Publisher debug_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_pub_;
};
#endif