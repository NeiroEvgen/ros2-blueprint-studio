#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

// Maps the hand pose (from the camera) to manipulator joint angles.
// Input:  /hand/pose  Float64MultiArray data[4]:
//        [0] wrist x in the frame 0..1, [1] y in the frame 0..1 (0 = top),
//        [2] hand tilt (rad, 0 = straight up), [3] grip 0..1
// Output: /arm/joint_cmd Float64MultiArray data[6] (ArmRenderNode contract)
class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode() : Node("arm_controller_node") {
        // target and current (smoothed) angles; start = idle pose
        target_ = idle_ = {0.0, 1.10, -0.60, 0.0, 0.0, 0.5};
        smooth_ = {0.0, 2.60, 0.20, 0.0, 0.0, 0.5};   // from folded
        last_hand_time_ = now();

        sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hand/pose", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 4) return;
                double x = std::clamp(msg->data[0], 0.0, 1.0);
                double y = std::clamp(msg->data[1], 0.0, 1.0);
                double tilt = std::clamp(msg->data[2], -1.2, 1.2);
                double grip = std::clamp(msg->data[3], 0.0, 1.0);

                // --- frame -> joints mapping ---
                target_[0] = (x - 0.5) * 1.6;            // base yaw
                double lift = 1.0 - y;                    // hand higher = manipulator higher
                target_[1] = 1.60 - 1.10 * lift;          // shoulder
                target_[2] = -0.20 - 0.75 * lift;         // elbow
                target_[3] = tilt;                        // wrist follows the tilt
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
        // failsafe: hand left the frame -> drift to the idle pose
        bool hand_alive = (now() - last_hand_time_).seconds() < 0.5;
        const auto& goal = hand_alive ? target_ : idle_;

        // EMA smoothing: smaller alpha = smoother (and lazier)
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