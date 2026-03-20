#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using std::placeholders::_1;

class Loop13 : public rclcpp::Node {
public:
    Loop13();

private:

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub;

    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg);
};

int main(int argc, char* argv[]);