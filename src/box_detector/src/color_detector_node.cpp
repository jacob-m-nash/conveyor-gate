#include "box_detector/color_detector_node.hpp"
#include <opencv2/imgproc.hpp>

BoxColorDetectorNode::BoxColorDetectorNode():Node("box_color_detector_node"){
    image_topic_ = this->declare_parameter<std::string>("image_topic", "camera/image_raw");
    min_fraction_ = this->declare_parameter<double>("min_fraction", 0.02);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    img_sub_ = image_transport::create_subscription(this,image_topic_,std::bind(&BoxColorDetectorNode::imageCb, this,std::placeholders::_1),"raw", qos.get_rmw_qos_profile());

    color_pub_ = this->create_publisher<std_msgs::msg::String>("detected_box_colour",10);
    debug_pub_ = image_transport::create_publisher(this,"debug_img");
    debug_enabled_ = this->declare_parameter<bool>("show_debug", false);
    RCLCPP_INFO(this->get_logger(),"Subscribed to %s", image_topic_.c_str());
}

void BoxColorDetectorNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch(const cv_bridge::Exception & e)
    {
        RCLCPP_WARN(this->get_logger(), "CV bridge failed: %s", e.what());
        return;
    }

    const cv::Mat & bgr = cv_ptr->image;
    if(bgr.empty()) return;

    cv::Mat hsv;
    cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);

    cv::Mat blue_mask, red_mask, red_mask1, red_mask2;
    cv::inRange(hsv, cv::Scalar(100, 120, 60), cv::Scalar(140, 255, 255), blue_mask);
    cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), red_mask2);
    cv::bitwise_or(red_mask1, red_mask2, red_mask);

    double total = static_cast<double>(bgr.rows * bgr.cols);
    double blue_frac = cv::countNonZero(blue_mask) / total;
    double red_frac  = cv::countNonZero(red_mask)  / total;   

    std::string colour = "unknown";
    if (std::max(blue_frac, red_frac) >= min_fraction_) {
        colour = (blue_frac >= red_frac) ? "blue" : "red";
        std_msgs::msg::String out;
        out.data = colour;
        color_pub_->publish(out);
    }

    if (debug_enabled_) {
        cv::Mat debug_img = bgr.clone();
        
        if (colour != "unknown") {
            cv::Scalar text_color = (colour == "blue") ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
            int font = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 1.5;
            int thickness = 3;
            int baseline = 0;
            
            cv::Size text_size = cv::getTextSize(colour, font, font_scale, thickness, &baseline);
            
            int padding = 10;
            cv::Point text_org(
                debug_img.cols - text_size.width - padding,
                debug_img.rows - padding
            );
        
            cv::putText(debug_img, colour, text_org, font, font_scale, text_color, thickness);
        }
        
        sensor_msgs::msg::Image::SharedPtr debug_msg = 
            cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();
        debug_pub_.publish(debug_msg);
        }
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoxColorDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
