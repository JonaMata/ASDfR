#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class BrightnessChecker : public rclcpp::Node
{
  public:
    BrightnessChecker()
    : Node("brightness_checker")
    {
      this->declare_parameter<int>("threshold", 128);
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&BrightnessChecker::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Bool>("light_on", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const
      {
        RCLCPP_INFO(this->get_logger(), "Received Image");
        int total_brightness = 0;
        int total_pixels = msg.width * msg.height;
        for (int i = 0; i < total_pixels*3; i++) {
          total_brightness += msg.data[i];
        }
        int avg_brightness = total_brightness / (total_pixels*3);
        RCLCPP_INFO(this->get_logger(), "Average brightness: '%d'", avg_brightness);
        int threshold = this->get_parameter("threshold").as_int();
        bool light_on = avg_brightness > threshold ? true : false;


        auto message = std_msgs::msg::Bool();
        message.data = light_on;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data ? "true" : "false");
        publisher_->publish(message);
      }
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  // rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::spin(std::make_shared<BrightnessChecker>());
  rclcpp::shutdown();
  return 0;
}