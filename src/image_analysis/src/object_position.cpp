#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObjectPosition : public rclcpp::Node {
public:
    ObjectPosition()
        : Node("object_position") {
        this->declare_parameter<int>("threshold", 128);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10, std::bind(&ObjectPosition::topic_callback, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("object_position", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    void topic_callback(const sensor_msgs::msg::Image& msg) const {
        RCLCPP_INFO(this->get_logger(), "Received Image");
        int total_pixels = msg.width * msg.height;
        // bool thresholded[total_pixels];
        int threshold = this->get_parameter("threshold").as_int();

        int total_above_threshold = 0;
        int sum_x = 0;
        int sum_y = 0;

        for (int i = 0; i < total_pixels; i++) {
            if ((msg.data[i * 3] + msg.data[i * 3 + 1] + msg.data[i * 3 + 2]) / 3 > threshold) {
                total_above_threshold++;
                sum_x += i % msg.width + 1;
                sum_y += i / msg.height + 1;
            }
        }

        int x = 0;
        int y = 0;
        if (total_above_threshold != 0) {
            x = sum_x / total_above_threshold - 1;
            y = sum_y / total_above_threshold - 1;
        }

        auto message = geometry_msgs::msg::Point();
        message.x = x;
        message.y = y;
        RCLCPP_INFO(this->get_logger(), "Publishing: x='%d', y='%d'", x, y);
        publisher_->publish(message);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectPosition>());
    rclcpp::shutdown();
    return 0;
}