#include "rclcpp/rclcpp.hpp"
#include "pipe_perf/stats.hpp"
#include "std_msgs/msg/header.hpp"

// Send a simple std_msgs::msg::Header message with the current time

class TimeSendNode : public rclcpp::Node
{
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:

  TimeSendNode() : Node{"time_send_node"}
  {
    (void) timer_;

    pub_ = create_publisher<std_msgs::msg::Header>("time_test", QUEUE_SIZE);

    timer_ = create_wall_timer(std::chrono::milliseconds{20}, [this]() -> void
    {
      std_msgs::msg::Header msg;
      msg.stamp = now();
      pub_->publish(msg);
    });
  }
};

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeSendNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
