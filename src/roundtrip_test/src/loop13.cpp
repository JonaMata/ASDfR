#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using std::placeholders::_1;

class Loop13 : public rclcpp::Node {
public:
    Loop13()
        : Node("loop13") {
        sub = this->create_subscription<std_msgs::msg::Int64>("roundtrip/start", 10, std::bind(&Loop13::topic_callback, this, _1));
        pub = this->create_publisher<std_msgs::msg::Int64>("roundtrip/end", 10);
    }

private:

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub;

    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        pub->publish(*msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Loop13>());
    rclcpp::shutdown();
    return 0;
}