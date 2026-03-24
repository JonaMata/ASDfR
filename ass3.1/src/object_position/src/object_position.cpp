#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core.hpp"
#include <opencv2/highgui.hpp>

typedef cv::Point3_<uint8_t> Pixel;

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObjectPosition : public rclcpp::Node {
public:
    ObjectPosition() : Node("object_position") {
        threshold = this->declare_parameter<int>("threshold", 10);
        fromCenter = this->declare_parameter<bool>("from_center", false);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10, std::bind(&ObjectPosition::topic_callback, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("output/object_position", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    bool fromCenter;
    int threshold;

    void topic_callback(const sensor_msgs::msg::Image& msg) const {
        auto frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat frame_hsv;
        cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        cv::inRange(frame_hsv, cv::Scalar(35, 30, 30), cv::Scalar(70, 255, 255), mask);

        cv::erode(mask, mask, cv::Mat());
        cv::dilate(mask, mask, cv::Mat());

        cv::Mat result;
        cv::bitwise_and(frame, frame, result, mask);
        cv::imshow("green", result);
        cv::waitKey(1);

        // Total pixels in the msg, length of msg.data
        int total_pixels = msg.width * msg.height;

        // Sum pixel coordinates and count total amount of pixels to later get average position
        int total_above_threshold = 0;
        int sum_x = 0;
        int sum_y = 0;

        result.forEach<Pixel>([&](Pixel& p, const int* position) -> void {
            if ((p.x + p.y + p.z) / 3 > threshold) {
                total_above_threshold++;
                // x coordinate is the column
                sum_x += position[1] + 1;
                // y coordinate is the row
                sum_y += position[0] + 1;
            }
        });

        // If fromCenter is false, we don't calculate the distance from the center
        // and -1 is used to indicate no object. Otherwise 0 (the center) is used.
        int x = fromCenter ? 0 : -1;
        int y = fromCenter ? 0 : -1;
        if (total_above_threshold != 0) {
            // Get average pixel location
            x = sum_x / total_above_threshold - 1;
            y = sum_y / total_above_threshold - 1;
            // If fromCenter is true, we subtract half the image width to get
            // the distance from the center in range [-width/2, width/2]
            if (fromCenter) {
                x -= msg.width / 2;
                y -= msg.height / 2;
            }
        }

        // Publish location
        auto message = geometry_msgs::msg::Point();
        message.x = x;
        message.y = y;
        RCLCPP_INFO(this->get_logger(), "Publishing: x=%d, y=%d, width=%d, height=%d, fromCenter=%d", x, y, msg.width, msg.height, fromCenter ? 1 : 0);
        publisher_->publish(message);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectPosition>());
    rclcpp::shutdown();
    return 0;
}