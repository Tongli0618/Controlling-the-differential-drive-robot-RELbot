#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class LPIndicatorNode : public rclcpp::Node {
public:
    LPIndicatorNode();
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Point2d compute_center_of_gravity(const cv::Mat& image);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr position_publisher_;
    //New2
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr positiondata_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    double brightness_threshold_;
};
