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
        tau = this->declare_parameter("tau", 1.0);
        objectPosition_sub = this->create_subscription<geometry_msgs::msg::Point>("input/object_position", 10,
            std::bind(&ObjectTracker::topic_callback, this, _1));
        left_pub = this->create_publisher<example_interfaces::msg::Float64>("output/left_motor/setpoint_vel", 10);
        right_pub = this->create_publisher<example_interfaces::msg::Float64>("output/right_motor/setpoint_vel", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr objectPosition_sub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub;
    double tau;

    void topic_callback(const geometry_msgs::msg::Point::SharedPtr point) {
        // Get next waypoint
        double error_x = point->x != -1 ? point->x : 0;
        // double error_y = point->y != -1 ? point->y : 0;

        // double vel = 1 / tau * error_y;
        // double rot = 1 / tau * error_x;

        // double left = vel + rot;
        // double right = vel - rot;

        // The setpoint is equal to the error multiplied by a factor tau to make it slower.
        double setpoint = -error_x * tau;

        // Publish setpoint
        auto leftMsg = example_interfaces::msg::Float64();
        leftMsg.data = setpoint;
        left_pub->publish(leftMsg);

        auto rightMsg = example_interfaces::msg::Float64();
        rightMsg.data = setpoint;
        right_pub->publish(rightMsg);

        // RCLCPP_INFO(get_logger(), "vel: %.1f, rot: %.1f, left: %.1f, right: %.1f, error_x: %.1f, error_y: %.1f",
        //     vel, rot, left, right, error_x, error_y);
        RCLCPP_INFO(get_logger(), "setpoint: %.1f, error_x: %.1f",
            setpoint, error_x);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTracker>());
    rclcpp::shutdown();
    return 0;
}