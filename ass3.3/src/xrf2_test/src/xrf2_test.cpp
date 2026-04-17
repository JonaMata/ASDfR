#include "xrf2_test.hpp"


Xrf2Test::Xrf2Test(): Node("xrf2_test") {
    xenoStateSub = this->create_subscription<std_msgs::msg::Int32>("XenoState", 10, std::bind(&Xrf2Test::xenoState_callback, this, _1));
    xeno2RosSub = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>("Xeno2Ros", 10, std::bind(&Xrf2Test::xeno2Ros_callback, this, _1));
    xenoCmdPub = this->create_publisher<std_msgs::msg::Int32>("XenoCmd", 10);
    ros2XenoPub = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);


    double wheel_r = 0.101;
    double wheel_circ = M_PI * wheel_r;
    double wheel_base = 0.2;
    double wheel_base_circ = M_PI * wheel_base;
    double wheel_speed = wheel_base_circ / 4.0 / 3.0;
    wheel_rads = wheel_speed/wheel_circ*2.0*M_PI;

    timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { sequence_step(); });
}

void Xrf2Test::xenoState_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received XenoState from XRF2: %d", msg->data);
    if (msg->data == 4) {
        initialised = true;
    }
}

void Xrf2Test::xeno2Ros_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received position from XRF2: left=%f, right=%f", msg->pos_left, msg->pos_right);
}

void Xrf2Test::sequence_step() {
    if (!initialised) {
        // Initialize XRF2 FSM

        auto msg = std_msgs::msg::Int32();
        msg.data = 1;
        xenoCmdPub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent initialisation msg");
        return;
    }

    xrf2_msgs::msg::Ros2Xeno msg;


    msg.steer_left = 0.0;
    msg.steer_right = 0.0;

    if (stepIndex > 0) {
        if (stepIndex <= 3) {
            msg.steer_left = 2.0;
            msg.steer_right = 2.0;
        } else if (stepIndex <= 6) {
            msg.steer_left = wheel_rads;
            msg.steer_right = -wheel_rads;
        }
    }
    ros2XenoPub->publish(msg);
    stepIndex++;
    RCLCPP_INFO(this->get_logger(), "Sent setpoint from ROS: left=%f, right=%f", msg.steer_left, msg.steer_right);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Xrf2Test>());
    rclcpp::shutdown();
    return 0;
}