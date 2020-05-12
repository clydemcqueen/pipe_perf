#include <iomanip>
#include "image_transport/image_transport.h"
#include "pipe_perf/stats.hpp"
#include "rclcpp/rclcpp.hpp"

// Receive a simple std_msgs::msg::Header message and print some info about the time lag

constexpr int NUM_MEASUREMENTS = 300;

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

    sub_ = image_transport::create_subscription(this, "image_raw", [this](const auto & msg)
    {
      if (!receiving_) {
        receiving_ = true;
        std::cout << "receiving messages" << std::endl;
      }

      values_.push_back((now() - msg->header.stamp).seconds() * 1e6);
      if (values_.size() >= NUM_MEASUREMENTS) {
        auto u = mean(values_);
        auto s = stdev(values_, u);
        std::cout << std::fixed << std::setprecision(0)
                  << "average lag over " << NUM_MEASUREMENTS << " measurements: "
                  << u << " μs +/- " << s << std::endl;
        values_.clear();
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
