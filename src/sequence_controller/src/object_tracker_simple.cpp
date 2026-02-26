#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObjectTracker : public rclcpp::Node {
public:
    ObjectTracker() : Node("object_tracker") {
        width = this->declare_parameter("width", 1);
        height = this->declare_parameter("height", 1);
        objectPosition_sub = this->create_subscription<geometry_msgs::msg::Point>("input/object_position", 10,
            std::bind(&ObjectTracker::topic_callback, this, _1));
        left_pub = this->create_publisher<example_interfaces::msg::Float64>("output/left_motor/setpoint_vel", 10);
        right_pub = this->create_publisher<example_interfaces::msg::Float64>("output/right_motor/setpoint_vel", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr objectPosition_sub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub;
    int width;
    int height;

    void topic_callback(const geometry_msgs::msg::Point::SharedPtr point) {
        // Get next waypoint
        int offset = point->x - width/2;
        int target = 0;

        if (abs(offset) > 10) {
            target = offset > 0 ? -1 : 1;
        }

        // Publish target
        auto leftMsg = example_interfaces::msg::Float64();
        leftMsg.data = target;
        left_pub->publish(leftMsg);

        auto rightMsg = example_interfaces::msg::Float64();
        rightMsg.data = target;
        right_pub->publish(rightMsg);

        RCLCPP_INFO(get_logger(), "x diff: %d", target);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTracker>());
    rclcpp::shutdown();
    return 0;
}