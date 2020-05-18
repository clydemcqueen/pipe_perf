#include "pipe_perf/stats.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

// Receive a simple std_msgs::msg::Header message and print some info about the time lag

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
          intro("lag");
        }

        values_.push_back((now() - msg->stamp).seconds() * 1e6);
        if (values_.size() >= NUM_MEASUREMENTS) {
          report(values_);
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
