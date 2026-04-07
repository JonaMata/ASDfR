#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SequenceController : public rclcpp::Node {
public:
    SequenceController()
        : Node("sequence_controller") {

        bool track_object = this->declare_parameter("track_object", false);
        tau = this->declare_parameter("tau", 10.0);

        want_ball_size = declare_parameter("want_ball_size", 60.0);
        current_step_ = 0;

        objectPosition_sub = this->create_subscription<geometry_msgs::msg::Point>("input/object_position", 10,
            std::bind(&SequenceController::topic_callback, this, _1));


        xenoStateSub = this->create_subscription<std_msgs::msg::Int32>("XenoState", 10, std::bind(&SequenceController::xenoState_callback, this, _1));
        xeno2RosSub = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>("Xeno2Ros", 10, std::bind(&SequenceController::xeno2Ros_callback, this, _1));
        xenoCmdPub = this->create_publisher<std_msgs::msg::Int32>("XenoCmd", 10);
        ros2XenoPub = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);

        auto msg = std_msgs::msg::Int32();
        msg.data = 1;
        xenoCmdPub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent initialisation msg");

    }

private:

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr objectPosition_sub;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr xenoStateSub;
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr xeno2RosSub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr xenoCmdPub;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr ros2XenoPub;

    double want_ball_size;
    size_t current_step_;
    double tau;
    bool initialised = false;

    void xenoState_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received XenoState from XRF2: %d", msg->data);
        if (msg->data == 4) {
            initialised = true;
        }
    }

    void xeno2Ros_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received position from XRF2: left=%f, right=%f", msg->pos_left, msg->pos_right);
    }

    void topic_callback(const geometry_msgs::msg::Point::SharedPtr point) {
        double setpointRotate = 0;
        double setpointForward = 0;
        double setpointLeft = 0;
        double setpointRight = 0;

        if (point->z != 0) {
            // Get next setpoint
            // Clamp here so it's not absurdly fast
            setpointRotate = std::clamp(-point->x * tau, -2.0, 2.0);
            // Dead zone in the center so it doesn't twitch when viewing the object nearly directly
            if (abs(setpointRotate) < 0.5) {
                setpointRotate = 0;
            }

            // double setpointForward = std::clamp(-point->y * tau, -2.0, 2.0);
            setpointForward = std::clamp(point->z != 0 ? want_ball_size - point->z : 0, -2.0, 2.0);
            if (abs(want_ball_size - point->z) < 5) {
                setpointForward = 0;
            }

            setpointLeft = (setpointForward + setpointRotate);
            setpointRight = (setpointForward - setpointRotate);

            double scalingFactor = 2 / std::max(abs(setpointLeft), abs(setpointRight));
            if(std::max(abs(setpointLeft), abs(setpointRight)) == 0) {
                scalingFactor = 1;
            }
            setpointLeft = setpointLeft * scalingFactor;
            setpointRight = setpointRight * scalingFactor;
        }

        // Publish setpoint
        auto msg = xrf2_msgs::msg::Ros2Xeno();
        msg.steer_left = setpointLeft;
        msg.steer_right = setpointRight;
        ros2XenoPub->publish(msg);

        RCLCPP_INFO(get_logger(), "setpointLeft=%.1f, setpointRight=%.1f", setpointLeft, setpointRight);
        RCLCPP_INFO(get_logger(), "forward=%.1f, rotate=%.1f", setpointForward, setpointRotate);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}