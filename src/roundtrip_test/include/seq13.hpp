#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

struct result {
    unsigned long start;
    unsigned long end;
};

class Seq13 : public rclcpp::Node {
public:
    Seq13();

private:
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub;
    timer_t timer;
    std::vector<result> results;

    static void timer_handler(union sigval sv);

    void on_timer();

    static void end_handler(union sigval sv);

    void on_end();

    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg);
};

int main(int argc, char* argv[]);