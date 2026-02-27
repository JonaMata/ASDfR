#include <chrono>
#include <functional>
#include <memory>
#include <string>

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
        objectPosition_sub = this->create_subscription<geometry_msgs::msg::Point>("input/object_position", 10,
            std::bind(&ObjectTrackerSimple::objectPosition_callback, this, _1));
        cameraPosition_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("input/camera_position", 10,
            std::bind(&ObjectTrackerSimple::cameraPosition_callback, this, _1));
        left_pub = this->create_publisher<example_interfaces::msg::Float64>("output/left_motor/setpoint_vel", 10);
        right_pub = this->create_publisher<example_interfaces::msg::Float64>("output/right_motor/setpoint_vel", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr objectPosition_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cameraPosition_sub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub;
    double object_x = 0, object_y = 0, camera_x = 0, camera_y = 0;

    void objectPosition_callback(const geometry_msgs::msg::Point::SharedPtr point) {
        object_x = point->x;
        object_y = point->y;
        send_setpoint();
    }

    void cameraPosition_callback(const geometry_msgs::msg::PointStamped::SharedPtr point) {
        camera_x = point->point.x;
        camera_y = point->point.y;
        send_setpoint();
    }

    void send_setpoint() {
        // Get next waypoint
        double error = object_x - camera_x;
        double target = -error / 10.0;

        // Set velocity to 0 if no object is detected
        if (object_x == -1) {
            target = 0;
        }

        // Publish target
        auto leftMsg = example_interfaces::msg::Float64();
        leftMsg.data = target;
        left_pub->publish(leftMsg);

        auto rightMsg = example_interfaces::msg::Float64();
        rightMsg.data = target;
        right_pub->publish(rightMsg);

        RCLCPP_INFO(get_logger(), "x diff: %f", target);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackerSimple>());
    rclcpp::shutdown();
    return 0;
}