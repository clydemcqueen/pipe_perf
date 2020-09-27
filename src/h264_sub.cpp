#include "h264_msgs/msg/packet.hpp"
#include "pipe_perf/stats.hpp"
#include "rclcpp/rclcpp.hpp"

// Sub to h264 packets directly
// Using StampedNode would give us time, but not size -- so sub directly
// TODO could use rmw layer to get size, and probably time, see:
//  https://discourse.ros.org/t/help-wanted-plotjuggler-for-ros2/11425/2
class H264SubNode : public rclcpp::Node
{
  typename rclcpp::Subscription<h264_msgs::msg::Packet>::SharedPtr sub_;
  bool receiving_;
  std::vector<double> lags_;
  std::vector<double> sizes_;

public:

  H264SubNode(const std::string &name, const std::string& topic) : Node{"image_transport_sub_node"}
  {
    (void) sub_;

    lags_.reserve(NUM_MEASUREMENTS);
    sizes_.reserve(NUM_MEASUREMENTS);

    sub_ = create_subscription<h264_msgs::msg::Packet>(
      topic, QUEUE_SIZE,
      [this](h264_msgs::msg::Packet::UniquePtr msg)
    {
      if (!receiving_) {
        receiving_ = true;
        intro("lag");
        intro("size");
      }

      lags_.push_back((now() - msg->header.stamp).seconds() * 1e6);
      sizes_.push_back(msg->data.size());

      if (lags_.size() >= NUM_MEASUREMENTS) {
        report(lags_);
        report(sizes_);
      }
    });
  }
};

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<H264SubNode>("h264_sub", "image_raw/h264");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
