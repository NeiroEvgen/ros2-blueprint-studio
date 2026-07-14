#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;

// Приём позы руки с хоста по UDP (порт 9877) -> /hand/pose
// Пакет: 4 x double little-endian: [x 0..1, y 0..1, tilt рад, grip 0..1]
class HandUdpReceiverNode : public rclcpp::Node {
public:
    HandUdpReceiverNode() : Node("hand_udp_receiver") {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_FATAL(get_logger(), "socket() failed");
            return;
        }
        int reuse = 1;
        setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(9877);
        if (bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(get_logger(), "bind(9877) failed — порт занят?");
            return;
        }
        fcntl(sock_, F_SETFL, O_NONBLOCK);   // не блокируем executor

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/hand/pose", 10);
        timer_ = create_wall_timer(10ms, std::bind(&HandUdpReceiverNode::poll, this));
        RCLCPP_INFO(get_logger(), " HandUdpReceiver: udp:9877 -> /hand/pose");
    }

    ~HandUdpReceiverNode() override {
        if (sock_ >= 0) close(sock_);
    }

private:
    void poll() {
        double buf[4];
        // выгребаем всё накопившееся, публикуем самый свежий пакет
        bool got = false;
        while (true) {
            ssize_t n = recv(sock_, buf, sizeof(buf), 0);
            if (n == (ssize_t)sizeof(buf)) { got = true; continue; }
            break;
        }
        if (!got) return;

        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(buf, buf + 4);
        pub_->publish(msg);
    }

    int sock_ = -1;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandUdpReceiverNode>());
    rclcpp::shutdown();
    return 0;
}