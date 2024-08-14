#include "hexapod_teleop_joystick/hexapod_teleop_joystick.hpp"

//==============================================================================
// Constructor
//==============================================================================

HexapodTeleopJoystick::HexapodTeleopJoystick() : rclcpp::Node("hexapod_teleop_joystick")
{
    // Initialize variables
    state_.data=false;
    imu_override_.data=false;
    NON_TELEOP = false;

    this->get_parameter_or("STANDUP_BUTTON", STANDUP_BUTTON, 3);
    this->get_parameter_or("SITDOWN_BUTTON", SITDOWN_BUTTON, 0);
    this->get_parameter_or("BODY_ROTATION_BUTTON", BODY_ROTATION_BUTTON, 8);
    this->get_parameter_or("FORWARD_BACKWARD_AXES", FORWARD_BACKWARD_AXES, 1);
    this->get_parameter_or("LEFT_RIGHT_AXES", LEFT_RIGHT_AXES, 0);
    this->get_parameter_or("YAW_ROTATION_AXES", YAW_ROTATION_AXES, 2);
    this->get_parameter_or("PITCH_ROTATION_AXES", PITCH_ROTATION_AXES, 3);
    this->get_parameter_or("MAX_METERS_PER_SEC", MAX_METERS_PER_SEC, 0.0);
    this->get_parameter_or("MAX_RADIANS_PER_SEC", MAX_RADIANS_PER_SEC, 0.0);
    this->get_parameter_or("NON_TELEOP", NON_TELEOP, false);

    // Initialize publishers
    state_pub_=this->create_publisher<std_msgs::msg::Bool>("/state",100);
    imu_override_pub_=this->create_publisher<std_msgs::msg::Bool>("/imu/imu_override",100);
    body_scalar_pub_=this->create_publisher<geometry_msgs::msg::AccelStamped>("/body_scalar",100);
    head_scalar_pub_=this->create_publisher<geometry_msgs::msg::AccelStamped>("/head_scalar",100);
    cmd_vel_pub_=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",100);

    // Initialize subsribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 5, std::bind(&HexapodTeleopJoystick::joyCallback, this, std::placeholders::_1));

    // Timer Callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),std::bind(&HexapodTeleopJoystick::timerCallback,this));
}

//==============================================================================
// Joystick call reading joystick topics
//==============================================================================

void HexapodTeleopJoystick::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    auto current_time = this->now();
    
    if (joy->buttons[STANDUP_BUTTON] == 1)
    {
        if (state_.data == false)
        {
            state_.data= true;
        }
    }

    if (joy->buttons[SITDOWN_BUTTON] == 1)
    {
        if (state_.data == true)
        {
            state_.data = false;
        }
    }

    // Body rotation L1 Button for testing
    if (joy->buttons[BODY_ROTATION_BUTTON] == 1)
    {
        imu_override_.data= true;

        body_scalar_.header.stamp = current_time;
        body_scalar_.accel.angular.x = -joy->axes[LEFT_RIGHT_AXES];
        body_scalar_.accel.angular.y = -joy->axes[FORWARD_BACKWARD_AXES];

        head_scalar_.header.stamp = current_time;
        head_scalar_.accel.angular.z = joy->axes[YAW_ROTATION_AXES];
        head_scalar_.accel.angular.y = joy->axes[PITCH_ROTATION_AXES]; 
    }
    else
    {
        imu_override_.data= false;
    }

    // Travelling
    if (joy->buttons[BODY_ROTATION_BUTTON] != -1)
    {
        cmd_vel_.linear.x = joy->axes[FORWARD_BACKWARD_AXES] * MAX_METERS_PER_SEC;
        cmd_vel_.linear.y = joy->axes[LEFT_RIGHT_AXES] * MAX_METERS_PER_SEC;
        cmd_vel_.angular.z = joy->axes[YAW_ROTATION_AXES] * MAX_RADIANS_PER_SEC;
    }
}

void HexapodTeleopJoystick::timerCallback()
{
    if (NON_TELEOP == false)
    {
        cmd_vel_pub_->publish(cmd_vel_);
        body_scalar_pub_->publish(body_scalar_);
        head_scalar_pub_->publish(head_scalar_);
    }
    state_pub_->publish(state_);
    imu_override_pub_->publish(imu_override_); 
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodTeleopJoystick>());
    rclcpp::shutdown();
    return 0;
}