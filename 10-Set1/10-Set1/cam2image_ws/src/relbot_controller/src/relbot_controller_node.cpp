#include "relbot_controller/relbot_controller_node.hpp"

WheelControlNode::WheelControlNode() : Node("wheel_control_node"), light_pose_received_(false) {

    
    light_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "light_position_data_topic", 10, std::bind(&WheelControlNode::light_pose_callback, this, std::placeholders::_1));
    camera_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "output/camera_position", 10, std::bind(&WheelControlNode::camera_pose_callback, this, std::placeholders::_1));
    left_motor_vel_pub_  = this->create_publisher<std_msgs::msg::Float64>("/input/left_motor/setpoint_vel", 10);
    right_motor_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/input/right_motor/setpoint_vel", 10);

    // Set up a timer to call print_current_velocity at a regular interval
    timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&WheelControlNode::print_current_velocity, this));
}
//print_current_velocity
void WheelControlNode::print_current_velocity() {
    RCLCPP_INFO(this->get_logger(), "Current velocity - Left motor: %f, Right motor: %f", current_left_motor_vel_, current_right_motor_vel_);
    RCLCPP_INFO(this->get_logger(), "light_pose: %s", light_pose_received_ ? "true" : "false");
}
void WheelControlNode::light_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    light_pose_ = msg->point;
    light_pose_received_ = true;
}

void WheelControlNode::camera_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    //1.2.1  generates the sequence of setpoints  
    if (!light_pose_received_) {
            if (!setpoint_timer_) {
                setpoint_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(5000),
                    std::bind(&WheelControlNode::generate_and_publish_setpoint, this));
            }
        return;
    }
    if (setpoint_timer_) {
        setpoint_timer_->cancel();
        setpoint_timer_.reset();
    }
    //1.2.2  follow a bright light
    double x_difference = msg->point.x - light_pose_.x;
     if (std::abs(x_difference) > 200) {
        x_difference = 0;
    }
    const double k_p = 0.01;
    current_left_motor_vel_ = -k_p * x_difference;
    current_right_motor_vel_ = k_p * x_difference;
    
    std_msgs::msg::Float64 left_motor_vel, right_motor_vel;
    left_motor_vel.data = current_left_motor_vel_;
    right_motor_vel.data = current_right_motor_vel_;
    
    left_motor_vel_pub_->publish(left_motor_vel);
    right_motor_vel_pub_->publish(right_motor_vel);
}
//1.2.1  function for generating the sequence of setpoints 
void WheelControlNode::generate_and_publish_setpoint() {
    current_left_motor_vel_ = speed_sequence[current_index];
    current_right_motor_vel_= speed_sequence[current_index];
    //RCLCPP_INFO(this->get_logger(), "Left velocity: %f, Right velocity: %f", current_left_motor_vel_, current_right_motor_vel_);
    current_index = (current_index + 1) % speed_sequence.size();

    std_msgs::msg::Float64 left_motor_vel, right_motor_vel;
    left_motor_vel.data =  current_left_motor_vel_;
    right_motor_vel.data = current_right_motor_vel_;

    left_motor_vel_pub_->publish(left_motor_vel);
    right_motor_vel_pub_->publish(right_motor_vel);

}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
