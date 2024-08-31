#ifndef HEXAPOD_CONTROLLER__IK_HPP_
#define HEXAPOD_CONTROLLER__IK_HPP_

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/msg/feet_positions.hpp"
#include "hexapod_msgs/msg/legs_joints.hpp"
#include "hexapod_controller/visibility.h"

//=============================================================================
// Define structs and classes for gait system
//=============================================================================

static const double PI = atan(1.0)*4.0;

struct Trig
{
    double sine;
    double cosine;
};

class Ik : public rclcpp::Node
{
public:
    HEXAPOD_CONTROLLER_PUBLIC Ik(const rclcpp::NodeOptions &options );
    void calculateIK(const hexapod_msgs::msg::FeetPositions::SharedPtr feet, const hexapod_msgs::msg::Pose::SharedPtr body,const hexapod_msgs::msg::LegsJoints::SharedPtr legs);
private:
    Trig getSinCos( double angle_rad );
    std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y; // Distance from coxa joint to the center pivot
    std::vector<double> INIT_COXA_ANGLE; // Initial coxa offsets in radians
    std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z; // Start position Of feet
    double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH; // Leg segment measurements
    int NUMBER_OF_LEGS; // Number of legs
};

#endif // HEXAPOD_CONTROLLER__IK_HPP_