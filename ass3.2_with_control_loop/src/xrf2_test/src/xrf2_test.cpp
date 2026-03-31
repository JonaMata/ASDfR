#include "xrf2_test.hpp"


Xrf2Test::Xrf2Test(): Node("xrf2_test") {
    // setpoints = this->declare_parameter("setpoints", std::vector<std::vector<double>>{{0.0, 0.0}, {1.0, 1.0}, {0.0, 0.0}, {-1.0, 1.0}, {0.0, 0.0}, {1.0, -1.0}, {0.0, 0.0}, {-1.0, -1.0}, {0.0, 0.0}});
    setpoints = {{0.0, 0.0}, {1.0, 1.0}, {0.0, 0.0}, {-1.0, 1.0}, {0.0, 0.0}, {1.0, -1.0}, {0.0, 0.0}, {-1.0, -1.0}, {0.0, 0.0}};
    xenoStateSub = this->create_subscription<std_msgs::msg::Int32>("XenoState", 10, std::bind(&Xrf2Test::xenoState_callback, this, _1));
    xeno2RosSub = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>("Xeno2Ros", 10, std::bind(&Xrf2Test::xeno2Ros_callback, this, _1));
    xenoCmdPub = this->create_publisher<std_msgs::msg::Int32>("XenoCmd", 10);
    ros2XenoPub = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);


    // Initialize XRF2 FSM

    auto msg = std_msgs::msg::Int32();
    msg.data = 1;
    xenoCmdPub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent initialisation msg");

    // while (!initialised && !stop) {
    //     sleep(1);
    // }
    // RCLCPP_INFO(this->get_logger(), "Initialisation confirmed");

    timer_ = create_wall_timer(
            std::chrono::seconds(2),
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
    if (initialised) {
        xrf2_msgs::msg::Ros2Xeno msg;

        msg.steer_left = setpoints[stepIndex][0];
        msg.steer_right = setpoints[stepIndex][1];
        ros2XenoPub->publish(msg);
        stepIndex++;
        RCLCPP_INFO(this->get_logger(), "Sent setpoint from ROS: left=%f, right=%f", msg.steer_left, msg.steer_right);

    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Xrf2Test>());
    rclcpp::shutdown();
    return 0;
}