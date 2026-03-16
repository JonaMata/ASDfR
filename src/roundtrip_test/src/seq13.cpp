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
    Seq13()
        : Node("seq13") {
        sleep(1);
        sub = this->create_subscription<std_msgs::msg::Int64>("roundtrip/end", 10, std::bind(&Seq13::topic_callback, this, _1));
        pub = this->create_publisher<std_msgs::msg::Int64>("roundtrip/start", 10);

        sigevent sev{};
        sev.sigev_notify = SIGEV_THREAD;
        sev.sigev_notify_function = &Seq13::timer_handler;
        sev.sigev_value.sival_ptr = this;

        timer_create(CLOCK_MONOTONIC, &sev, &timer);

        itimerspec its{};
        its.it_value.tv_nsec = 1e6;
        its.it_interval.tv_nsec = 1e6;

        RCLCPP_INFO(this->get_logger(), "Starting timer");
        timer_settime(timer, 0, &its, nullptr);

        timer_t end_timer;

        sigevent end_sev{};
        end_sev.sigev_notify = SIGEV_THREAD;
        end_sev.sigev_notify_function = &Seq13::end_handler;
        end_sev.sigev_value.sival_ptr = this;

        timer_create(CLOCK_MONOTONIC, &end_sev, &end_timer);

        itimerspec end_its{};
        end_its.it_value.tv_sec = 1;

        RCLCPP_INFO(this->get_logger(), "Starting timer");
        timer_settime(end_timer, 0, &end_its, nullptr);
    }

private:

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub;
    timer_t timer;
    std::vector<result> results;

    static void timer_handler(union sigval sv) {
        Seq13* obj = static_cast<Seq13*>(sv.sival_ptr);
        obj->on_timer();
    }

    void on_timer() {
        timespec time;
        clock_gettime(CLOCK_MONOTONIC, &time);
        auto msg = std_msgs::msg::Int64();
        msg.data = time.tv_nsec;
        pub->publish(msg);
    }

    static void end_handler(union sigval sv) {
        Seq13* obj = static_cast<Seq13*>(sv.sival_ptr);
        obj->on_end();
    }

    void on_end() {

        RCLCPP_INFO(this->get_logger(), "Stopping timer");
        timer_delete(timer);
        sleep(1);

        RCLCPP_INFO(this->get_logger(), "Stopped timer");

        RCLCPP_INFO(this->get_logger(), "Vector size: %ld", results.size());

        // long global_start = results[0].start;

        unsigned long long total_roundtrip = 0;

        for(unsigned int i = 0; i < results.size(); i++) {
            total_roundtrip += results[i].end - results[i].start;
        }
        int average_roundtrip = total_roundtrip / results.size();

        std::ofstream file("seq13.csv");
        file << "start,end\n";

        // RCLCPP_INFO(this->get_logger(), "Jitter \t Roundtrip \t Roundtrip Jitter");

        for(unsigned int i = 0; i < results.size(); i++) {
            result res = results[i];
            file << res.start << "," << res.end << "\n";
        }
        file.close();

        RCLCPP_INFO(this->get_logger(), "Finished");

        rclcpp::shutdown();
    }

    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        unsigned long start_time = msg->data;
        timespec time;
        clock_gettime(CLOCK_MONOTONIC, &time);
        unsigned long end_time = time.tv_nsec;
        results.push_back({start_time, end_time});
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Seq13>());
    rclcpp::shutdown();
    return 0;
}