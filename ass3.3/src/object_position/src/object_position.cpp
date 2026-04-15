#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"

// Include check for cv_bridge. Ubuntu 22 is .h, Ubuntu 24 is .hpp
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "Required cv_bridge header file not found"
#endif

#include "opencv2/core.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <vector>

typedef cv::Point3_<uint8_t> Pixel;

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObjectPosition : public rclcpp::Node {
public:
    ObjectPosition() : Node("object_position") {
        lightness_lower = this->declare_parameter<int>("lightness_lower", 30);
        lightness_upper = this->declare_parameter<int>("lightness_upper", 200);
        mask_lightness_upper = this->declare_parameter<int>("mask_lightness_upper", 80);
        mask_lightness_lower = this->declare_parameter<int>("mask_lightness_lower", 30);
        threshold = this->declare_parameter<int>("threshold", 10);
        fromCenter = this->declare_parameter<bool>("from_center", false);
        showMask = declare_parameter<bool>("show_mask", false);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10, std::bind(&ObjectPosition::topic_callback, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("output/object_position", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    bool fromCenter;
    int threshold;
    int lightness_lower, lightness_upper, mask_lightness_lower, mask_lightness_upper;
    bool showMask;
    mutable cv::Ptr<cv::SimpleBlobDetector> blobDetector;
    
    void topic_callback(const sensor_msgs::msg::Image& msg) const {
        if (blobDetector == nullptr) {
            cv::SimpleBlobDetector::Params params;
            params.minDistBetweenBlobs = msg.height / 4;
            params.filterByArea = true;
            params.minArea = 20 * 20;
            params.maxArea = msg.height * msg.height;
            params.filterByColor = true;
            params.blobColor = 255;
            // params.filterByConvexity = true;
            params.filterByConvexity = false;
            params.minConvexity = 0.4;
            // params.filterByInertia = true;
            params.filterByInertia = false;
            params.minInertiaRatio = 0.5;
            blobDetector = cv::SimpleBlobDetector().create(params);
        }
        
        auto frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat frame_hsv;
        cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
        
        std::vector<cv::Mat> channels;
        cv::split(frame_hsv, channels);
        int lightness = (int)cv::mean(channels[2])[0];
        
        lightness = std::clamp(lightness, lightness_lower, lightness_upper);
        int mask_lightness = mask_lightness_upper - (lightness - lightness_lower) / (lightness_upper - lightness_lower) * (mask_lightness_upper - mask_lightness_lower);
        RCLCPP_INFO(this->get_logger(), "Lightness=%d, masklightness=%d", lightness, mask_lightness);
        
        cv::Mat mask;
        cv::inRange(frame_hsv, cv::Scalar(30, 85, mask_lightness), cv::Scalar(85, 255, 255), mask);
        
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::morphologyEx(mask, mask, cv::MORPH_DILATE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(25, 25)));
        cv::morphologyEx(mask, mask, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)));
        cv::morphologyEx(mask, mask, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        
        std::vector<cv::KeyPoint> blobs;
        blobDetector->detect(mask, blobs);

        float biggestBlob = 0;
        float biggestBlobCenterX = fromCenter ? 0 : -1;
        float biggestBlobCenterY = fromCenter ? 0 : -1;
        for (std::vector<cv::KeyPoint>::iterator blob = blobs.begin(); blob != blobs.end(); blob++) {
            if (blob->size > biggestBlob) {
                biggestBlob = blob->size;
                biggestBlobCenterX = blob->pt.x;
                biggestBlobCenterY = blob->pt.y;
            }
        }
        
        if (showMask) {
            cv::Mat result;
            cv::bitwise_and(frame, frame, result, mask);
            if (biggestBlob != 0) {
                cv::circle(result, cv::Point(biggestBlobCenterX, biggestBlobCenterY), biggestBlob / 2, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
            }

            cv::imshow("green", result);
            cv::waitKey(1);
        }

        // // Sum pixel coordinates and count total amount of pixels to later get average position
        // int total_above_threshold = 0;
        // int sum_x = 0;
        // int sum_y = 0;

        // result.forEach<Pixel>([&](Pixel& p, const int* position) -> void {
        //     if ((p.x + p.y + p.z) / 3 > threshold) {
        //         total_above_threshold++;
        //         // x coordinate is the column
        //         sum_x += position[1] + 1;
        //         // y coordinate is the row
        //         sum_y += position[0] + 1;
        //     }
        //     });

        // // If fromCenter is false, we don't calculate the distance from the center
        // // and -1 is used to indicate no object. Otherwise 0 (the center) is used.
        // int x = fromCenter ? 0 : -1;
        // int y = fromCenter ? 0 : -1;
        // if (total_above_threshold > msg.width * msg.height * 0.05) {
        //     // Get average pixel location
        //     x = sum_x / total_above_threshold - 1;
        //     y = sum_y / total_above_threshold - 1;
        //     // If fromCenter is true, we subtract half the image width to get
        //     // the distance from the center in range [-width/2, width/2]
        //     if (fromCenter) {
        //         x -= msg.width / 2;
        //         y -= msg.height / 2;
        //     }
        // }

        if (fromCenter) {
            if (biggestBlob == 0) {
                biggestBlobCenterX = 0;
                biggestBlobCenterY = 0;
            } else {
                biggestBlobCenterX -= msg.width / 2;
                biggestBlobCenterY -= msg.height / 2;
            }
        }

        // Publish location
        auto message = geometry_msgs::msg::Point();
        message.x = biggestBlobCenterX;
        message.y = biggestBlobCenterY;
        message.z = biggestBlob;
        RCLCPP_INFO(this->get_logger(), "Publishing: x=%.0f, y=%.0f, z=%.0f, fromCenter=%d", biggestBlobCenterX, biggestBlobCenterY, biggestBlob, fromCenter ? 1 : 0);
        publisher_->publish(message);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectPosition>());
    rclcpp::shutdown();
    return 0;
}
