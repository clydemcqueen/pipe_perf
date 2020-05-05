#include <iomanip>
#include "pipe_perf/stats.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

// Receive a simple std_msgs::msg::Header message and print some info about the time lag

constexpr int QUEUE_SIZE = 10;
constexpr int NUM_MEASUREMENTS = 300;

class TimeReceiveNode : public rclcpp::Node
{
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_;
  bool receiving_;
  std::vector<double> values_;

public:

  TimeReceiveNode() : Node{"time_receive_node"}
  {
    (void) sub_;

    values_.reserve(NUM_MEASUREMENTS);

    sub_ = create_subscription<std_msgs::msg::Header>(
      "time_test", QUEUE_SIZE,
      [this](std_msgs::msg::Header::UniquePtr msg)
      {
        if (!receiving_) {
          receiving_ = true;
          std::cout << "receiving messages" << std::endl;
        }

        values_.push_back((now() - msg->stamp).seconds() * 1e6);
        if (values_.size() >= NUM_MEASUREMENTS) {
          auto u = mean(values_);
          auto s = stdev(values_, u);
          std::cout << std::fixed << std::setprecision(0)
                    << "average lag over " << NUM_MEASUREMENTS << " measurements: "
                    << u << " μs +/- " << s << std::endl;
          values_.clear();
        }
      });
  }
};

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeReceiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
