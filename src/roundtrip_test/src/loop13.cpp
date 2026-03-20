#include "loop13.hpp"

Loop13::Loop13(): Node("loop13") {
    sub = this->create_subscription<std_msgs::msg::Int64>("roundtrip/start", 10, std::bind(&Loop13::topic_callback, this, _1));
    pub = this->create_publisher<std_msgs::msg::Int64>("roundtrip/end", 10);
}

void Loop13::topic_callback(const std_msgs::msg::Int64::SharedPtr msg) {
    pub->publish(*msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Loop13>());
    rclcpp::shutdown();
    return 0;
}