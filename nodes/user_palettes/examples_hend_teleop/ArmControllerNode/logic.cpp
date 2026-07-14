#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

// Маппинг позы руки (с камеры) в углы суставов манипулятора.
// Вход:  /hand/pose  Float64MultiArray data[4]:
//        [0] x запястья в кадре 0..1, [1] y в кадре 0..1 (0 = верх),
//        [2] tilt кисти (рад, 0 = вертикально вверх), [3] grip 0..1
// Выход: /arm/joint_cmd Float64MultiArray data[6] (контракт ArmRenderNode)
class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode() : Node("arm_controller_node") {
        // Целевые и текущие (сглаженные) углы; старт = поза ожидания
        target_ = idle_ = {0.0, 1.10, -0.60, 0.0, 0.0, 0.5};
        smooth_ = {0.0, 2.60, 0.20, 0.0, 0.0, 0.5};   // из сложенной
        last_hand_time_ = now();

        sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hand/pose", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 4) return;
                double x = std::clamp(msg->data[0], 0.0, 1.0);
                double y = std::clamp(msg->data[1], 0.0, 1.0);
                double tilt = std::clamp(msg->data[2], -1.2, 1.2);
                double grip = std::clamp(msg->data[3], 0.0, 1.0);

                // --- Маппинг кадр -> суставы ---
                target_[0] = (x - 0.5) * 1.6;            // yaw базы
                double lift = 1.0 - y;                    // выше рука = выше манипулятор
                target_[1] = 1.60 - 1.10 * lift;          // плечо
                target_[2] = -0.20 - 0.75 * lift;         // локоть
                target_[3] = tilt;                        // кисть повторяет наклон
                target_[5] = grip;

                last_hand_time_ = now();
            });

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/arm/joint_cmd", 10);
        timer_ = create_wall_timer(33ms, std::bind(&ArmControllerNode::update, this));
        RCLCPP_INFO(get_logger(),
            "🎮 ArmControllerNode: /hand/pose -> /arm/joint_cmd (EMA, failsafe)");
    }

private:
    void update() {
        // Failsafe: рука пропала из кадра -> плывём в позу ожидания
        bool hand_alive = (now() - last_hand_time_).seconds() < 0.5;
        const auto& goal = hand_alive ? target_ : idle_;

        // EMA-сглаживание: чем меньше alpha, тем плавнее (и ленивее)
        const double alpha = hand_alive ? 0.25 : 0.06;
        for (size_t i = 0; i < smooth_.size(); i++)
            smooth_[i] += alpha * (goal[i] - smooth_[i]);

        std_msgs::msg::Float64MultiArray out;
        out.data.assign(smooth_.begin(), smooth_.end());
        pub_->publish(out);
    }

    std::array<double, 6> target_, smooth_, idle_;
    rclcpp::Time last_hand_time_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControllerNode>());
    rclcpp::shutdown();
    return 0;
}