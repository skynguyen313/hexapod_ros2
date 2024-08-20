#ifndef HEXAPOD_TELEOP_HPP_
#define HEXAPOD_TELEOP_HPP_

#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

class HexapodTeleopJoystick : public rclcpp::Node
{
public:
    HexapodTeleopJoystick();
    std_msgs::msg::Bool state_; // StandUp or SitDown
    std_msgs::msg::Bool imu_override_;
    geometry_msgs::msg::AccelStamped body_scalar_;
    geometry_msgs::msg::AccelStamped head_scalar_;
    geometry_msgs::msg::Twist cmd_vel_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_override_pub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr body_scalar_pub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr head_scalar_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    bool NON_TELEOP; // Shuts down cmd_vel broadcast

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void timerCallback();
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int STANDUP_BUTTON, SITDOWN_BUTTON, BODY_ROTATION_BUTTON, FORWARD_BACKWARD_AXES, LEFT_RIGHT_AXES, YAW_ROTATION_AXES, PITCH_ROTATION_AXES;
    double MAX_METERS_PER_SEC, MAX_RADIANS_PER_SEC;
};

#endif // HEXAPOD_TELEOP_HPP_