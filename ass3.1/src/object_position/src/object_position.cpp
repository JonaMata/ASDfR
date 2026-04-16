#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Include check for cv_bridge. Ubuntu 22 is .h, Ubuntu 24 is .hpp
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "Required cv_bridge header file not found"
#endif

#include <opencv2/highgui.hpp>

#include <vector>

#include "object_position.hpp"

typedef cv::Point3_<uint8_t> Pixel;

using namespace std::chrono_literals;
using std::placeholders::_1;

ObjectPosition::ObjectPosition() : Node("object_position") {
    fromCenter = this->declare_parameter<bool>("from_center", false);
    showMask = declare_parameter<bool>("show_mask", false);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&ObjectPosition::topic_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("output/object_position", 10);
}

void ObjectPosition::topic_callback(const sensor_msgs::msg::Image& msg) const {
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

    cv::Mat mask;
    cv::inRange(frame_hsv, cv::Scalar(30, 85, 80), cv::Scalar(85, 255, 255), mask);

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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectPosition>());
    rclcpp::shutdown();
    return 0;
}
