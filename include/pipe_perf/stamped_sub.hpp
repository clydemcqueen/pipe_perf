#include "rclcpp/rclcpp.hpp"
#include "pipe_perf/stats.hpp"

// Receive a ROS2 stamped message and print some info about the time lag

template<typename M>
class StampedNode : public rclcpp::Node
{
  typename rclcpp::Subscription<M>::SharedPtr sub_;
  bool receiving_;
  std::vector<double> values_;

public:

  StampedNode(const std::string &name, std::string topic) :
    Node{name}
  {
    (void) sub_;

    values_.reserve(NUM_MEASUREMENTS);

    sub_ = create_subscription<M>(
      topic, QUEUE_SIZE,
      [this](typename M::UniquePtr msg)
      {
        if (!receiving_) {
          receiving_ = true;
          intro("lag");
        }

        values_.push_back((now() - msg->header.stamp).seconds() * 1e6);
        if (values_.size() >= NUM_MEASUREMENTS) {
          report(values_);
        }
      });
  }
};
