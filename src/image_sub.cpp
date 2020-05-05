#include "pipe_perf/stamped_sub.hpp"
#include "sensor_msgs/msg/image.hpp"

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StampedNode<sensor_msgs::msg::Image>>("image_sub", "/forward_camera/image_raw");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
