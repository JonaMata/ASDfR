#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Xrf2Test : public rclcpp::Node {
public:
    Xrf2Test();

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr xenoStateSub;
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr xeno2RosSub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr xenoCmdPub;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr ros2XenoPub;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<double>> setpoints;
    bool initialised = false;
    int stepIndex = 0;
    void xenoState_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void xeno2Ros_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg);

    void sequence_step();
};

int main(int argc, char* argv[]);