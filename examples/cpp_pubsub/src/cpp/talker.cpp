#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class talker : public rclcpp::Node {
public:
    talker() : Node("talker"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
        timer_ = this->create_wall_timer(1s, [this](){
            auto msg = std_msgs::msg::String();
            msg.data = "Cpp: " + std::to_string(count_++);
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Pub: '%s'", msg.data.c_str());
        });
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int a, char** b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<talker>());rclcpp::shutdown();return 0;}
