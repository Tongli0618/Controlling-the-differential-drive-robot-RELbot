#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <chrono>
#include <cmath> 
class WheelControlNode : public rclcpp::Node {
public:
    WheelControlNode();

private:
    void light_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void camera_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void print_current_velocity();
    void generate_and_publish_setpoint();

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr light_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr camera_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    geometry_msgs::msg::Point light_pose_;
    
    bool light_pose_received_;
    int setpoint_index_ = 0;

    double current_left_motor_vel_ = 0.0;
    double current_right_motor_vel_ = 0.0;
    
    std::vector<double> speed_sequence = {0.0, 1.0, 2.0, 3.0};
    size_t current_index = 0;
};
