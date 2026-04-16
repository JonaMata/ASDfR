#include "sequence_controller.hpp"


SequenceController::SequenceController() : Node("sequence_controller") {
    tau = this->declare_parameter("tau", 0.03);
    miauw = this->declare_parameter("miauw", 0.06);

    want_ball_size = declare_parameter("want_ball_size", 60.0);
    current_step_ = 0;

    objectPosition_sub = this->create_subscription<geometry_msgs::msg::Point>("input/object_position", 10,
        std::bind(&SequenceController::topic_callback, this, _1));


    xenoStateSub = this->create_subscription<std_msgs::msg::Int32>("XenoState", 10, std::bind(&SequenceController::xenoState_callback, this, _1));
    xeno2RosSub = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>("Xeno2Ros", 10, std::bind(&SequenceController::xeno2Ros_callback, this, _1));
    xenoCmdPub = this->create_publisher<std_msgs::msg::Int32>("XenoCmd", 10);
    ros2XenoPub = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);

    auto msg = std_msgs::msg::Int32();
    msg.data = 1;
    xenoCmdPub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent initialisation msg");
}


void SequenceController::xenoState_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received XenoState from XRF2: %d", msg->data);
    if (msg->data == 4) {
        initialised = true;
    }
}

void SequenceController::xeno2Ros_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received position from XRF2: left=%f, right=%f", msg->pos_left, msg->pos_right);
}

void SequenceController::topic_callback(const geometry_msgs::msg::Point::SharedPtr point) {
    double setpointRotate = 0;
    double setpointForward = 0;
    double setpointLeft = 0;
    double setpointRight = 0;

    if (point->z != 0) {
        int x = -point->x;
        if (abs(x) < 30) {
            x = 0;
        }
        setpointRotate = std::clamp(x * tau, -5.0, 5.0);
        
        int z = want_ball_size - point->z;
        if (abs(z) < 10) {
            z = 0;
        }
        setpointForward = std::clamp(z*miauw, -5.0, 5.0);

        setpointLeft = (setpointForward + setpointRotate);
        setpointRight = (setpointForward - setpointRotate);

        double scalingFactor = 5 / std::max(abs(setpointLeft), abs(setpointRight));
        if(std::max(abs(setpointLeft), abs(setpointRight)) == 0) {
            scalingFactor = 1;
        }
    scalingFactor = 1;
        setpointLeft = setpointLeft * scalingFactor;
        setpointRight = setpointRight * scalingFactor;
    }

    // Publish setpoint
    auto msg = xrf2_msgs::msg::Ros2Xeno();
    msg.steer_left = setpointLeft;
    msg.steer_right = setpointRight;
    ros2XenoPub->publish(msg);

    RCLCPP_INFO(get_logger(), "setpointLeft=%.1f, setpointRight=%.1f", setpointLeft, setpointRight);
    RCLCPP_INFO(get_logger(), "forward=%.1f, rotate=%.1f", setpointForward, setpointRotate);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}
