
# === PYTHON TEMPLATES ===
PYTHON_TEMPLATES = {
    "Default": """import rclpy
from rclpy.node import Node

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        # [AUTO-GEN-PUBS] - маркер для добавления паблишеров
        {EXEC_OUT_INIT}
        
        # [AUTO-GEN-SUBS] - маркер для добавления подписчиков
        {EXEC_IN_INIT}
        
        self.get_logger().info('Node {node_name} started')

    # [AUTO-GEN-FIRE] - сюда можно добавлять колбэки
    {EXEC_FIRE}

def main():
    rclpy.init()
    node = {class_name}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
""",

    "Timer": """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        
        # [AUTO-GEN-PUBS]
        {EXEC_OUT_INIT}
        
        # Создаем встроенный триггер для topic_1
        self.trigger_pub = self.create_publisher(String, 'topic_1', 10)
        
        interval = float({interval})
        self.timer = self.create_timer(interval, self.timer_callback)
        self.get_logger().info(f'Timer Started: {interval}s')

    def timer_callback(self):
        # 1. Отправляем сигнал
        msg = String()
        msg.data = "tick"
        self.trigger_pub.publish(msg)
        
        # [AUTO-GEN-FIRE]
        {EXEC_FIRE}

def main():
    rclpy.init()
    node = {class_name}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
""",

    "Publisher": """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        
        # [AUTO-GEN-PUBS]
        {EXEC_OUT_INIT}

        # Основной паблишер ноды
        self.pub = self.create_publisher(String, '{topic_name}', 10)
        
        # [AUTO-GEN-SUBS]
        {EXEC_IN_INIT}
        
        # Слушаем триггер от таймера
        self.trigger_sub = self.create_subscription(String, 'topic_1', self.trigger_callback, 10)
        self.i = 0

    def trigger_callback(self, msg):
        out_msg = String()
        out_msg.data = f"Py: {self.i}"
        self.pub.publish(out_msg)
        self.i += 1
        
        # [AUTO-GEN-FIRE]
        {EXEC_FIRE}

def main():
    rclpy.init()
    node = {class_name}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
""",

    "Subscriber": """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        
        # [AUTO-GEN-PUBS]
        {EXEC_OUT_INIT}
        
        # [AUTO-GEN-SUBS]
        {EXEC_IN_INIT}

        self.sub = self.create_subscription(String, '{topic_name}', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')
        # [AUTO-GEN-FIRE]
        {EXEC_FIRE}

def main():
    rclpy.init()
    node = {class_name}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
}

# ==========================================
# C++ TEMPLATES
# ==========================================
CPP_TEMPLATES = {
    "Default": """#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class {class_name} : public rclcpp::Node {
public:
    {class_name}() : Node("{node_name}") {
        // [AUTO-GEN-EXEC-IN-INIT]
        {EXEC_IN_INIT}
        // [AUTO-GEN-EXEC-OUT-INIT]
        {EXEC_OUT_INIT}
        
        RCLCPP_INFO(this->get_logger(), "Node Started: {node_name}");
    }

private:
    // [AUTO-GEN-EXEC-VARS]
    {EXEC_VARS}
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<{class_name}>());
    rclcpp::shutdown();
    return 0;
}
""",

    "Timer": """#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class {class_name} : public rclcpp::Node {
public:
    {class_name}() : Node("{node_name}") {
        // [AUTO-GEN-EXEC-OUT]
        {EXEC_OUT_INIT}

        // Конвертация интервала
        long int ms = (long int)({interval} * 1000);
        if(ms <= 0) ms = 500; // защита от нуля
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(ms), [this](){
            // [AUTO-GEN-FIRE]
            {EXEC_FIRE}
            // RCLCPP_INFO(this->get_logger(), "Tick!");
        });
        
        RCLCPP_INFO(this->get_logger(), "Timer started: %ld ms", ms);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    // [AUTO-GEN-VARS]
    {EXEC_VARS}
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<{class_name}>());
    rclcpp::shutdown();
    return 0;
}
""",

    "Publisher": """#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class {class_name} : public rclcpp::Node {
public:
    {class_name}() : Node("{node_name}"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("{topic_name}", 10);
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

int main(int a, char** b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}
""",

    "Subscriber": """#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class {class_name} : public rclcpp::Node {
public:
    {class_name}() : Node("{node_name}") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "{topic_name}", 10, std::bind(&{class_name}::topic_callback, this, _1));
    }
private:
    void topic_callback(const std_msgs::msg::String & msg) const {
        RCLCPP_INFO(this->get_logger(), "Heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int a, char** b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}
""",

    "ActionClient": """#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp" 

class {class_name} : public rclcpp::Node {
public:
    using ActionType = example_interfaces::action::Fibonacci;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

    {class_name}() : Node("{node_name}") {
        this->client_ptr_ = rclcpp_action::create_client<ActionType>(this, "fibonacci");
        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&{class_name}::send_goal, this));
    }

    void send_goal() {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Action server not available...");
            return;
        }
        auto goal_msg = ActionType::Goal();
        goal_msg.order = 10;
        
        auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        this->timer_->cancel();
    }

private:
    rclcpp_action::Client<ActionType>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<{class_name}>());
    rclcpp::shutdown();
    return 0;
}
"""
}
TEMPLATES = {}
TEMPLATES.update(PYTHON_TEMPLATES)
TEMPLATES.update(CPP_TEMPLATES)