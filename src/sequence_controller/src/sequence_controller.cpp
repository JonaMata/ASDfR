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

class SequenceController : public rclcpp::Node {
public:
    SequenceController()
        : Node("sequence_controller") {

        bool track_object = this->declare_parameter("track_object", false);
        tau = this->declare_parameter("tau", 10.0);

        declare_parameter("left_waypoints", std::vector<double>{0.0, 1.0, -1.0, -2.0});
        declare_parameter("right_waypoints", std::vector<double>{0.0, -1.0, 1.0, -2.0});
        leftWaypoints_ = get_parameter("left_waypoints").as_double_array();
        rightWaypoints_ = get_parameter("right_waypoints").as_double_array();
        current_step_ = 0;

        if (track_object) {
            objectPosition_sub = this->create_subscription<geometry_msgs::msg::Point>("input/object_position", 10,
                std::bind(&SequenceController::objectPosition_callback, this, _1));
            cameraPosition_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("input/camera_position", 10,
                std::bind(&SequenceController::cameraPosition_callback, this, _1));
        } else {
            timer_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() { sequence_step(); });
        }
        
        left_pub = this->create_publisher<example_interfaces::msg::Float64>("output/left_motor/setpoint_vel", 10);
        right_pub = this->create_publisher<example_interfaces::msg::Float64>("output/right_motor/setpoint_vel", 10);
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr objectPosition_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cameraPosition_sub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> leftWaypoints_;
    std::vector<double> rightWaypoints_;
    size_t current_step_;
    double object_x = 0, object_y = 0, camera_x = 0, camera_y = 0;
    double tau;

    void sequence_step() {
        //   if (current_step_ >= leftWaypoints_.size()) {
        //   // RCLCPP_INFO(get_logger(), "Mission complete! Final position: %.2f", current_position_);
        //   timer_->cancel();  // Stop mission
        //   return;
        // }

        // Get next waypoint
        double leftTarget = leftWaypoints_[current_step_ % leftWaypoints_.size()];
        double rightTarget = rightWaypoints_[current_step_ % rightWaypoints_.size()];

        // Publish target
        auto leftMsg = example_interfaces::msg::Float64();
        leftMsg.data = leftTarget;
        left_pub->publish(leftMsg);

        auto rightMsg = example_interfaces::msg::Float64();
        rightMsg.data = rightTarget;
        right_pub->publish(rightMsg);

        // RCLCPP_INFO(get_logger(), "Mission step %zu/%zu: Target = %.1f (current pos: %.2f)", 
        //             current_step_ + 1, waypoints_.size(), target, current_position_);

        current_step_++;
    }

    void objectPosition_callback(const geometry_msgs::msg::Point::SharedPtr point) {
        object_x = point->x;
        object_y = point->y;
        calc_setpoint();
    }

    void cameraPosition_callback(const geometry_msgs::msg::PointStamped::SharedPtr point) {
        camera_x = point->point.x;
        camera_y = point->point.y;
        calc_setpoint();
    }

    void calc_setpoint() {
        // Get next waypoint
        double error = object_x - camera_x;
        double target = -error / tau;

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
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}