#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SequenceController : public rclcpp::Node
{
  public:
    SequenceController()
    : Node("sequence_controller")
    {

      declare_parameter("left_waypoints", std::vector<double>{0.0, 5.0, 10.0, 5.0, 0.0});
      declare_parameter("right_waypoints", std::vector<double>{0.0, 5.0, 10.0, 5.0, 0.0});
      leftWaypoints_ = get_parameter("left_waypoints").as_double_array();
      rightWaypoints_ = get_parameter("right_waypoints").as_double_array();
      current_step_ = 0;

      leftPublisher_ = this->create_publisher<example_interfaces::msg::Float64>("output/left_motor/setpoint_vel", 10);
      rightPublisher_ = this->create_publisher<example_interfaces::msg::Float64>("output/right_motor/setpoint_vel", 10);

      timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() { sequence_step(); });
    }

  private:
    void sequence_step()
    {
      if (current_step_ >= leftWaypoints_.size()) {
      // RCLCPP_INFO(get_logger(), "Mission complete! Final position: %.2f", current_position_);
      timer_->cancel();  // Stop mission
      return;
    }
    
    // Get next waypoint
    double leftTarget = leftWaypoints_[current_step_];
    double rightTarget = rightWaypoints_[current_step_];
    
    // Publish target
    auto leftMsg = example_interfaces::msg::Float64();
    leftMsg.data = leftTarget;
    leftPublisher_->publish(leftMsg);

    auto rightMsg = example_interfaces::msg::Float64();
    rightMsg.data = rightTarget;
    rightPublisher_->publish(rightMsg);
    
    // RCLCPP_INFO(get_logger(), "Mission step %zu/%zu: Target = %.1f (current pos: %.2f)", 
    //             current_step_ + 1, waypoints_.size(), target, current_position_);
    
    current_step_++;
    }
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr leftPublisher_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr rightPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> leftWaypoints_;
    std::vector<double> rightWaypoints_;
    size_t current_step_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceController>());
  rclcpp::shutdown();
  return 0;
}