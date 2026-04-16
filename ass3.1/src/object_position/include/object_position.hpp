#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "opencv2/core.hpp"
#include <opencv2/features2d.hpp>

typedef cv::Point3_<uint8_t> Pixel;

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObjectPosition : public rclcpp::Node {
    public:
    ObjectPosition();
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    bool fromCenter;
    bool showMask;
    mutable cv::Ptr<cv::SimpleBlobDetector> blobDetector;

    void topic_callback(const sensor_msgs::msg::Image& msg) const;
};