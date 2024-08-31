#include <memory>
#include "hexapod_controller/ik.hpp"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  
  rclcpp::shutdown();
  return 0;
}
