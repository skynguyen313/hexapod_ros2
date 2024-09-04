#ifndef HEXAPOD_CONTROLLER__GAIT_HPP_
#define HEXAPOD_CONTROLLER__GAIT_HPP_

#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "hexapod_msgs/msg/feet_positions.hpp"
#include "hexapod_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hexapod_controller/visibility.h"

//=============================================================================
// Define classes for gait system
//=============================================================================

static const double PI = atan(1.0)*4.0;

class Gait : public rclcpp::Node
{
public:
    HEXAPOD_CONTROLLER_PUBLIC Gait( const rclcpp::NodeOptions &options );
    void gaitCycle( const geometry_msgs::msg::Twist &cmd_vel, hexapod_msgs::msg::FeetPositions *feet, geometry_msgs::msg::Twist *gait_vel );
private:
    //void cyclePeriod( const geometry_msgs::msg::Pose2D &base, hexapod_msgs::msg::FeetPositions *feet, geometry_msgs::msg::Twist *gait_vel );
};


#endif // HEXAPOD_CONTROLLER__GAIT_HPP_