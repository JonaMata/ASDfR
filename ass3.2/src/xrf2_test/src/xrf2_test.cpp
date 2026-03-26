#include "xrf2_test.hpp"

Xrf2Test::Xrf2Test(): Node("xrf2_test") {
    setpoints = this->declare_parameter("setpoints", std::vector<std::vector<double>>{{0.0, 0.0}, {1.0, 1.0}, {0.0, 0.0}, {-1.0, 1.0}, {0.0, 0.0}, {1.0, -1.0}, {0.0, 0.0}, {-1.0, -1.0}, {0.0, 0.0}});

    xenoStateSub = this->create_subscription<std_msgs::msg::Int32>("XenoState", 10, std::bind(&Xrf2Test::xenoState_callback, this, _1));
    xeno2RosSub = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>("Xeno2Ros", 10, std::bind(&Xrf2Test::xeno2Ros_callback, this, _1));
    xenoCmdPub = this->create_publisher<std_msgs::msg::Int32>("XenoCmd", 10);
    ros2XenoPub = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);


    // Initialize XRF2 FSM

    xenoCmdPub->publish(std_msgs::msg::Int32(1));

    while (!initialised) {
        sleep(1);
    }

    send_sequence();
}

void Xrf2Test::xenoState_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data == 3) {
        initialised = true;
    }
}

void Xrf2Test::xeno2Ros_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received position from XRF2: left=%f, right=%f", msg->pos_left, msg->pos_right);
}

void Xrf2Test::send_sequence() {
    xrf2_msgs::msg::Ros2Xeno msg;

    for (const auto& setpoint : setpoints) {
        msg.steer_left = setpoint[0];
        msg.steer_right = setpoint[1];
        ros2XenoPub->publish(msg);
        sleep(5);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Xrf2Test>());
    rclcpp::shutdown();
    return 0;
}