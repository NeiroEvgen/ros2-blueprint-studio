#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

// Рендер 6-осевого манипулятора из углов суставов.
// Вход:  /arm/joint_cmd  (std_msgs/Float64MultiArray, data[6]:
//        [0] yaw базы, [1] плечо, [2] локоть, [3] кисть pitch,
//        [4] кисть roll (пока не рисуем), [5] раскрытие пальцев 0..1)
// Выход: /arm_viz_markers (MarkerArray, frame 'map')
class ArmRenderNode : public rclcpp::Node {
public:
    ArmRenderNode() : Node("arm_render_node") {
        // Стартовая поза: рука сложена вниз
        q_ = {0.0, 2.60, 0.20, 0.0, 0.0, 0.5};

        sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm/joint_cmd", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                for (size_t i = 0; i < q_.size() && i < msg->data.size(); i++)
                    q_[i] = msg->data[i];
            });

        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/arm_viz_markers", 10);
        timer_ = create_wall_timer(33ms, std::bind(&ArmRenderNode::render, this));
        RCLCPP_INFO(get_logger(),
            "🦾 ArmRenderNode: listening /arm/joint_cmd -> /arm_viz_markers");
    }

private:
    static constexpr double kPedestalH = 0.15;
    static constexpr double kL1 = 0.35, kL2 = 0.30, kL3 = 0.12;

    struct P { double x, y, z; };

    static geometry_msgs::msg::Quaternion quat_z_to(double dx, double dy, double dz) {
        geometry_msgs::msg::Quaternion q;
        double n = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (n < 1e-9) { q.w = 1.0; return q; }
        dx /= n; dy /= n; dz /= n;
        double ax = -dy, ay = dx;
        double an = std::sqrt(ax*ax + ay*ay);
        double cosang = std::clamp(dz, -1.0, 1.0);
        if (an < 1e-9) { if (cosang > 0) q.w = 1.0; else q.x = 1.0; return q; }
        ax /= an; ay /= an;
        double half = 0.5 * std::acos(cosang), s = std::sin(half);
        q.w = std::cos(half); q.x = ax*s; q.y = ay*s; q.z = 0.0;
        return q;
    }

    visualization_msgs::msg::Marker base_marker(int id, int type) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = get_clock()->now();
        m.ns = "arm"; m.id = id; m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.color.a = 1.0;
        return m;
    }

    visualization_msgs::msg::Marker sphere(int id, P p, double r,
                                           double cr, double cg, double cb) {
        auto m = base_marker(id, visualization_msgs::msg::Marker::SPHERE);
        m.pose.position.x = p.x; m.pose.position.y = p.y; m.pose.position.z = p.z;
        m.scale.x = m.scale.y = m.scale.z = r * 2.0;
        m.color.r = cr; m.color.g = cg; m.color.b = cb;
        return m;
    }

    visualization_msgs::msg::Marker link(int id, P a, P b, double radius,
                                         double cr, double cg, double cb) {
        auto m = base_marker(id, visualization_msgs::msg::Marker::CYLINDER);
        double dx = b.x-a.x, dy = b.y-a.y, dz = b.z-a.z;
        m.pose.position.x = (a.x+b.x)*0.5;
        m.pose.position.y = (a.y+b.y)*0.5;
        m.pose.position.z = (a.z+b.z)*0.5;
        m.pose.orientation = quat_z_to(dx, dy, dz);
        m.scale.x = m.scale.y = radius*2.0;
        m.scale.z = std::sqrt(dx*dx+dy*dy+dz*dz);
        m.color.r = cr; m.color.g = cg; m.color.b = cb;
        return m;
    }

    void render() {
        double yaw = q_[0], q1 = q_[1], q2 = q_[2], q4 = q_[3];
        double grip = std::clamp(q_[5], 0.0, 1.0);

        double a1 = q1, a2 = q1 + q2, a3 = q1 + q2 + q4;
        P shoulder{0, 0, kPedestalH + 0.05};
        P elbow_l{ shoulder.x + kL1*std::sin(a1), 0, shoulder.z + kL1*std::cos(a1) };
        P wrist_l{ elbow_l.x + kL2*std::sin(a2), 0, elbow_l.z + kL2*std::cos(a2) };
        P hand_l{  wrist_l.x + kL3*std::sin(a3), 0, wrist_l.z + kL3*std::cos(a3) };

        auto rot = [&](P p) -> P {
            return { p.x*std::cos(yaw) - p.y*std::sin(yaw),
                     p.x*std::sin(yaw) + p.y*std::cos(yaw), p.z };
        };
        P elbow = rot(elbow_l), wrist = rot(wrist_l), hand = rot(hand_l);

        visualization_msgs::msg::MarkerArray arr;
        {   // постамент
            auto m = base_marker(0, visualization_msgs::msg::Marker::CYLINDER);
            m.pose.position.z = kPedestalH*0.5;
            m.scale.x = m.scale.y = 0.20; m.scale.z = kPedestalH;
            m.color.r = m.color.g = m.color.b = 0.25;
            arr.markers.push_back(m);
        }
        arr.markers.push_back(sphere(1, shoulder, 0.045, 1.0, 0.45, 0.0));
        arr.markers.push_back(link(2, shoulder, elbow, 0.030, 0.55, 0.60, 0.65));
        arr.markers.push_back(sphere(3, elbow, 0.040, 1.0, 0.45, 0.0));
        arr.markers.push_back(link(4, elbow, wrist, 0.026, 0.55, 0.60, 0.65));
        arr.markers.push_back(sphere(5, wrist, 0.034, 1.0, 0.45, 0.0));
        arr.markers.push_back(link(6, wrist, hand, 0.030, 0.4, 1.0, 0.4));

        // Пальцы: раскрытие по grip (0 = сжаты вдоль ладони, 1 = веером)
        double dirx = std::sin(a3), dirz = std::cos(a3);
        for (int i = -1; i <= 1; i++) {
            double spread = 0.010 + 0.022 * grip;
            P f0_l{ hand_l.x, spread * i, hand_l.z };
            P f1_l{ hand_l.x + 0.08*dirx, spread * i * (1.0 + grip),
                    hand_l.z + 0.08*dirz };
            arr.markers.push_back(link(10+i, rot(f0_l), rot(f1_l), 0.011,
                                       0.4, 1.0, 0.4));
        }
        pub_->publish(arr);
    }

    std::array<double, 6> q_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmRenderNode>());
    rclcpp::shutdown();
    return 0;
}