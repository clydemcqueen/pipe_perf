#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "pipe_perf/stamped_sub.hpp"

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StampedNode<fiducial_vlam_msgs::msg::Observations>>("fiducial_sub", "/fiducial_observations");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
