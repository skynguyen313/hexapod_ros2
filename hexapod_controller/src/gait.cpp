#include "hexapod_controller/gait.hpp"

Gait::Gait( const rclcpp::NodeOptions &options )
:rclcpp::Node("gait",options)
{

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Gait)
