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
    SequenceController();
private:

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr objectPosition_sub;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr xenoStateSub;
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr xeno2RosSub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr xenoCmdPub;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr ros2XenoPub;

    double want_ball_size;
    size_t current_step_;
    double tau;
    double miauw;
    bool initialised = false;

    void xenoState_callback(const std_msgs::msg::Int32::SharedPtr msg);

    void xeno2Ros_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg);

    void topic_callback(const geometry_msgs::msg::Point::SharedPtr point);
};

int main(int argc, char* argv[]);