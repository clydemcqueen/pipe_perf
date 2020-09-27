#include "image_transport/image_transport.h"
#include "pipe_perf/stats.hpp"
#include "rclcpp/rclcpp.hpp"

// Sub to h264 images using image transport -- this will call the h264 decoder behind the scenes
class ImageTransportSubNode : public rclcpp::Node
{
  image_transport::Subscriber sub_;
  bool receiving_;
  std::vector<double> values_;

public:

  ImageTransportSubNode() : Node{"image_transport_sub_node"}
  {
    (void) sub_;

    values_.reserve(NUM_MEASUREMENTS);

    // Subscribe to image_raw/h264, image_transport will call plugin to decode
    sub_ = image_transport::create_subscription(this, "image_raw", [this](const auto & msg)
    {
      if (!receiving_) {
        receiving_ = true;
        intro("lag");
      }

      values_.push_back((now() - msg->header.stamp).seconds() * 1e6);
      if (values_.size() >= NUM_MEASUREMENTS) {
        report(values_);
      }
    }, "h264"); // TODO param
  }
};

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageTransportSubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
