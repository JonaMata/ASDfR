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
        want_ball_size = declare_parameter("want_ball_size", 60.0);
        leftWaypoints_ = get_parameter("left_waypoints").as_double_array();
        rightWaypoints_ = get_parameter("right_waypoints").as_double_array();
        current_step_ = 0;

        if (track_object) {
            objectPosition_sub = this->create_subscription<geometry_msgs::msg::Point>("input/object_position", 10,
                std::bind(&SequenceController::topic_callback, this, _1));
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
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> leftWaypoints_;
    std::vector<double> rightWaypoints_;
    double want_ball_size;
    size_t current_step_;
    double tau;

    void sequence_step() {
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

        current_step_++;
    }

    void topic_callback(const geometry_msgs::msg::Point::SharedPtr point) {
        // Get next setpoint
        // Clamp here so it's not absurdly fast
        double setpointRotate = std::clamp(-point->x * tau, -2.0, 2.0);
        // Dead zone in the center so it doesn't twitch when viewing the object nearly directly
        if (abs(setpointRotate) < 1) {
            setpointRotate = 0;
        }

        // double setpointForward = std::clamp(-point->y * tau, -2.0, 2.0);
        double setpointForward = std::clamp(point->z != 0 ? want_ball_size - point->z : 0, -2.0, 2.0);
        if (abs(setpointForward) < 5) {
            setpointForward = 0;
        }

        double setpointLeft = (setpointForward + setpointRotate);
        double setpointRight = (setpointForward - setpointRotate);
        double scalingFactor = 2 / std::max(abs(setpointLeft), abs(setpointRight));

        // Publish setpoint
        auto leftMsg = example_interfaces::msg::Float64();
        leftMsg.data = setpointLeft * scalingFactor;
        left_pub->publish(leftMsg);

        auto rightMsg = example_interfaces::msg::Float64();
        rightMsg.data = setpointRight * scalingFactor;
        right_pub->publish(rightMsg);

        RCLCPP_INFO(get_logger(), "setpointLeft=%.1f, setpointRight=%.1f", setpointLeft, setpointRight);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}