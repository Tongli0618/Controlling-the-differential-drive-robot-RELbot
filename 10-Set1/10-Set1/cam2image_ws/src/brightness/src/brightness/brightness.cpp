#include "brightness/brightness.hpp"


BrightnessNode::BrightnessNode() : Node("brightness_node") {
    // Declare and get brightness threshold parameters
    this->declare_parameter<float>("brightness_threshold", 130.0);
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
        "image", 10, std::bind(&BrightnessNode::image_callback, this, std::placeholders::_1));

    // Publish brightness status message
    brightness_publisher_ = this->create_publisher<std_msgs::msg::String>("brightness_topic", 10);
}

void BrightnessNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_image->image;
    cv::Mat grayScale;
    cv::cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);
    cv::Scalar meanValue = cv::mean(grayScale);
    double avg_brightness = meanValue[0];

    // Publish messages based on brightness threshold
    std_msgs::msg::String brightness_message;
    RCLCPP_INFO(this->get_logger(), "Brightness: %f, Threshold: %f", avg_brightness, brightness_threshold_);
    brightness_message.data = avg_brightness > brightness_threshold_ ? "Light is ON" : "Light is OFF";
    brightness_publisher_->publish(brightness_message);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightnessNode>());
    rclcpp::shutdown();
    return 0;
}
