#include <iomanip>
#include "pipe_perf/stats.hpp"
#include "rclcpp/rclcpp.hpp"

// Receive a ROS2 stamped message and print some info about the time lag

constexpr int QUEUE_SIZE = 10;
constexpr int NUM_MEASUREMENTS = 300;

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
      });
  }
};
