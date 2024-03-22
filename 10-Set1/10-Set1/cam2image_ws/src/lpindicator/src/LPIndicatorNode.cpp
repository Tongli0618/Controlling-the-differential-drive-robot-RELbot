#include "lpindicator/LPIndicatorNode.hpp"

LPIndicatorNode::LPIndicatorNode() : Node("lp_indicator_node") {
    // Declare and get brightness threshold parameters
    this->declare_parameter<double>("brightness_threshold", 130.0);
    this->get_parameter("brightness_threshold", brightness_threshold_);

    auto parameter_callback =
        [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &parameter : parameters) {
                if (parameter.get_name() == "brightness_threshold") {
                    brightness_threshold_ = parameter.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated brightness_threshold to: %f", brightness_threshold_);
                }
            }
            return result;
        };
    param_callback_handle_ = this->add_on_set_parameters_callback(parameter_callback);

    // Subscribe to image topic
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&LPIndicatorNode::image_callback, this, std::placeholders::_1));

    // Publish position message
    position_publisher_ = this->create_publisher<std_msgs::msg::String>("light_position_topic", 10);
    // NEW2
    positiondata_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("light_position_data_topic",1);
    // New try
    processed_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image_topic", 10);
}

void LPIndicatorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat gray_image = cv_image->image;
    
    // Apply threshold to find bright areas
    cv::Mat binary_image;
    cv::threshold(gray_image, binary_image, brightness_threshold_, 255, cv::THRESH_BINARY);

    // Compute the center of gravity of the white pixels

    cv::Point2d center = compute_center_of_gravity(binary_image);

    // New Draw a circle at the center point in the binary image
    cv::Mat colored_image;
    cv::cvtColor(binary_image, colored_image, cv::COLOR_GRAY2BGR);
    cv::circle(colored_image, center, 10, CV_RGB(0,0,255), 2);

    // New Convert the processed image back to a ROS message
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    cv_bridge::CvImage cv_image_processed(header, "bgr8", colored_image);
    sensor_msgs::msg::Image ros_image_processed;
    cv_image_processed.toImageMsg(ros_image_processed);


    // Publish the position
    std_msgs::msg::String position_message;
    position_message.data = "Light Position: (" + std::to_string(center.x) + ", " + std::to_string(center.y) + ")";
    position_publisher_->publish(position_message);
    //New2
    geometry_msgs::msg::PointStamped position_data_message;
    position_data_message.header.stamp = this->get_clock()->now();  
    position_data_message.header.frame_id = "";
    position_data_message.point.x = center.x;
    position_data_message.point.y = center.y;  
    position_data_message.point.z = 0.0; 
    positiondata_publisher_->publish(position_data_message);
    //New publish
    processed_image_publisher_->publish(ros_image_processed);
}

cv::Point2d LPIndicatorNode::compute_center_of_gravity(const cv::Mat& image) {
    cv::Moments m = cv::moments(image, true);
    return cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LPIndicatorNode>());
    rclcpp::shutdown();
    return 0;
}
