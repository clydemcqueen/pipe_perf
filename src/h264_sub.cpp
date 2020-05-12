#include "pipe_perf/stamped_sub.hpp"
#include "h264_msgs/msg/packet.hpp"

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StampedNode<h264_msgs::msg::Packet>>("h264_sub", "image_raw/h264");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
