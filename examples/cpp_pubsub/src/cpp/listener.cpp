#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class listener : public rclcpp::Node {
public:
    listener() : Node("listener") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/chatter", 10, std::bind(&listener::topic_callback, this, _1));
    }
private:
    void topic_callback(const std_msgs::msg::String & msg) const {
        RCLCPP_INFO(this->get_logger(), "Heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int a, char** b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<listener>());rclcpp::shutdown();return 0;}
