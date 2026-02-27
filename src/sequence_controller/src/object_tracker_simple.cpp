#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObjectTrackerSimple : public rclcpp::Node {
public:
    ObjectTrackerSimple() : Node("object_tracker_simple") {
        tau = this->declare_parameter<double>("tau", 1);
        objectPosition_sub = this->create_subscription<geometry_msgs::msg::Point>("input/object_position", 10,
            std::bind(&ObjectTrackerSimple::topic_callback, this, _1));
        left_pub = this->create_publisher<example_interfaces::msg::Float64>("output/left_motor/setpoint_vel", 10);
        right_pub = this->create_publisher<example_interfaces::msg::Float64>("output/right_motor/setpoint_vel", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr objectPosition_sub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub;
    double tau = 0;

    void topic_callback(const geometry_msgs::msg::Point::SharedPtr point) {
        // Get next setpoint
        // Clamp here because relbot_simulator clamping doesn't work well for extremely error values for some reason
        double setpoint = std::clamp(-point->x * tau, -2.0, 2.0);
        // Dead zone in the center so it doesn't twitch when viewing the object nearly directly
        if (std::abs(setpoint) < 0.5) {
            setpoint = 0;
        }

        // Publish setpoint
        auto leftMsg = example_interfaces::msg::Float64();
        leftMsg.data = setpoint;
        left_pub->publish(leftMsg);

        auto rightMsg = example_interfaces::msg::Float64();
        rightMsg.data = setpoint;
        right_pub->publish(rightMsg);

        RCLCPP_INFO(get_logger(), "setpoint=%.1f", setpoint);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackerSimple>());
    rclcpp::shutdown();
    return 0;
}