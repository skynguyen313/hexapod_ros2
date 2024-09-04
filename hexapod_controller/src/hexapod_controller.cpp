#include <memory>
#include "hexapod_controller/ik.hpp"
#include "rclcpp/rclcpp.hpp"

class HexapodController : public rclcpp::Node
{
public:
  HexapodController()
  :rclcpp::Node("HexapodController")
  {
    // Init objects
    rclcpp::NodeOptions options;
    ik_ = std::make_unique<IK>(options);


  }

private:
  void initDefaultPose(){

  }

  void loop(){

  }

  


  std::unique_ptr<IK> ik_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
  bool prev_hex_active_state_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto hexapod_controller = std::make_shared<HexapodController>();
  rclcpp::spin(hexapod_controller);
  return 0;
}
