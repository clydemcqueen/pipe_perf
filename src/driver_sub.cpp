#include "orca_msgs/msg/driver.hpp"
#include "pipe_perf/stamped_sub.hpp"

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StampedNode<orca_msgs::msg::Driver>>("driver_sub", "/driver_status");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
