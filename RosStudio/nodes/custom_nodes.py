from NodeGraphQt import BaseNode

# ==============================================================================
#                                 COLORS
# ==============================================================================
MSG_COLORS = {
    'std_msgs/String':     (255, 235, 59),
    'std_msgs/Int32':      (76, 175, 80),
    'std_msgs/Bool':       (244, 67, 54),
    'geometry_msgs/Twist': (255, 152, 0),
    'sensor_msgs/Image':   (156, 39, 176),
    'nav_msgs/Odometry':   (233, 30, 99),
    'Any':                 (180, 180, 180),
    'Blue':                (80, 150, 255)
}

# ==============================================================================
#                                 TEMPLATES
# ==============================================================================
TEMPLATES = {
    # --- PYTHON CUSTOM (Обновлен) ---
    'py_custom': """import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        
        # [AUTO-GEN-PUBS]
        # <-- Сюда добавятся Publisher (например: self.pub_out = ...)

        # [AUTO-GEN-SUBS]
        # <-- Сюда добавятся Subscriber (например: self.sub_in = ...)

    # Сюда могут добавляться callback функции
    def listener_callback(self, msg):
        self.get_logger().info(f'Custom Got: {msg.data}')

def main():
    rclpy.init()
    node = {class_name}()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
""",

    # --- C++ CUSTOM (Полностью переписан под генератор) ---
    'cpp_custom': """#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
// [AUTO-GEN-INCLUDES] 
// <-- Сюда добавятся новые хидеры (например nav_msgs/...)

using std::placeholders::_1;

class {class_name} : public rclcpp::Node {
public:
    {class_name}() : Node("{node_name}") {
        // [AUTO-GEN-PUBS]
        // <-- Сюда встанет код: pub_ = create_publisher...
        
        // [AUTO-GEN-SUBS]
        // <-- Сюда встанет код: sub_ = create_subscription...
    }

private:
    // [AUTO-GEN-VARS]
    // <-- Сюда встанут объявления переменных: rclcpp::Publisher<...>::SharedPtr ...
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<{class_name}>());
    rclcpp::shutdown();
    return 0;
}
""",

    # --- PYTHON STANDARD (Без изменений, но приведены для целостности) ---
    'py_string_pub': """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.pub = self.create_publisher(String, '{topic_name}', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = "Py: " + str(self.i)
        self.pub.publish(msg)
        self.i += 1
def main(): rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()""",

    'py_string_sub': """import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.sub = self.create_subscription(String, '{topic_name}', self.listener_callback, 10)
    def listener_callback(self, msg): self.get_logger().info(f'Heard: {msg.data}')
def main(): rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()""",

    'py_twist_pub': """import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.pub = self.create_publisher(Twist, '{topic_name}', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    def timer_callback(self):
        msg = Twist(); msg.linear.x = 2.0; msg.angular.z = 1.0
        self.pub.publish(msg)
def main(): rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()""",

    'py_twist_sub': """import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.sub = self.create_subscription(Twist, '{topic_name}', self.listener_callback, 10)
    def listener_callback(self, msg): self.get_logger().info(f'Speed: {msg.linear.x}')
def main(): rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()""",

    # --- C++ STANDARD (Без изменений) ---
    'cpp_string_pub': """#include "rclcpp/rclcpp.hpp"
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
        });
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};
int main(int a, char** b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}""",

    'cpp_string_sub': """#include "rclcpp/rclcpp.hpp"
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
int main(int a, char** b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}""",

    'cpp_twist_pub': """#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std::chrono_literals;

class {class_name} : public rclcpp::Node {
public:
    {class_name}() : Node("{node_name}") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("{topic_name}", 10);
        timer_ = this->create_wall_timer(1s, [this](){
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = 2.0; msg.angular.z = 1.0;
            publisher_->publish(msg);
        });
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int a, char** b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}""",

    'cpp_twist_sub': """#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class {class_name} : public rclcpp::Node {
public:
    {class_name}() : Node("{node_name}") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "{topic_name}", 10, std::bind(&{class_name}::topic_callback, this, _1));
    }
private:
    void topic_callback(const geometry_msgs::msg::Twist & msg) const {
        RCLCPP_INFO(this->get_logger(), "Speed: '%.2f'", msg.linear.x);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};
int main(int a, char** b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}"""
}

# ==============================================================================
#                                 BASE CLASS
# ==============================================================================
class RosNodeBase(BaseNode):
    def __init__(self):
        super(RosNodeBase, self).__init__()
    
    def _init_template(self, template_key):
        self.create_property('template_key', template_key)
        raw = TEMPLATES.get(template_key, "")
        self.create_property('code_content', raw, widget_type=0)

    def get_template_data(self):
        c_name = self.get_property('class_name')
        n_name = self.get_property('node_name')
        
        final_class_name = c_name if c_name else n_name.replace(" ", "_")

        return {
            'language': 'cpp' if 'cpp' in self.type_ else 'python',
            'name': n_name,
            'topic': self.get_property('topic_name'),
            'class_name': final_class_name,
            'template_key': self.get_property('template_key'),
            'code': self.get_property('code_content')
        }

# ==============================================================================
#                                 STANDARD NODES
# ==============================================================================
class PyStringPubNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='StringPub'
    def __init__(self): super().__init__(); self.add_output('out',color=MSG_COLORS['std_msgs/String']); self.add_text_input('node_name','Name',text='py_str_pub'); self.add_text_input('topic_name','Topic',text='/chatter'); self._init_template('py_string_pub')

class PyStringSubNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='StringSub'
    def __init__(self): super().__init__(); self.add_input('in',color=MSG_COLORS['std_msgs/String']); self.add_text_input('node_name','Name',text='py_str_sub'); self.add_text_input('topic_name','Topic',text='/chatter'); self._init_template('py_string_sub')

class PyTwistPubNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='TwistPub'
    def __init__(self): super().__init__(); self.add_output('out',color=MSG_COLORS['geometry_msgs/Twist']); self.add_text_input('node_name','Name',text='py_cmd_vel_pub'); self.add_text_input('topic_name','Topic',text='/cmd_vel'); self._init_template('py_twist_pub')

class PyTwistSubNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='TwistSub'
    def __init__(self): super().__init__(); self.add_input('in',color=MSG_COLORS['geometry_msgs/Twist']); self.add_text_input('node_name','Name',text='py_cmd_vel_sub'); self.add_text_input('topic_name','Topic',text='/cmd_vel'); self._init_template('py_twist_sub')

class CppStringPubNode(RosNodeBase):
    __identifier__='ros.cpp'; NODE_NAME='StringPub'
    def __init__(self): super().__init__(); self.add_output('out',color=MSG_COLORS['std_msgs/String']); self.add_text_input('node_name','Name',text='cpp_str_pub'); self.add_text_input('topic_name','Topic',text='/chatter'); self._init_template('cpp_string_pub'); self.set_color(20,20,100)

class CppStringSubNode(RosNodeBase):
    __identifier__='ros.cpp'; NODE_NAME='StringSub'
    def __init__(self): super().__init__(); self.add_input('in',color=MSG_COLORS['std_msgs/String']); self.add_text_input('node_name','Name',text='cpp_str_sub'); self.add_text_input('topic_name','Topic',text='/chatter'); self._init_template('cpp_string_sub'); self.set_color(20,20,100)

class CppTwistPubNode(RosNodeBase):
    __identifier__='ros.cpp'; NODE_NAME='TwistPub'
    def __init__(self): super().__init__(); self.add_output('out',color=MSG_COLORS['geometry_msgs/Twist']); self.add_text_input('node_name','Name',text='cpp_cmd_vel_pub'); self.add_text_input('topic_name','Topic',text='/cmd_vel'); self._init_template('cpp_twist_pub'); self.set_color(100,50,0)

class CppTwistSubNode(RosNodeBase):
    __identifier__='ros.cpp'; NODE_NAME='TwistSub'
    def __init__(self): super().__init__(); self.add_input('in',color=MSG_COLORS['geometry_msgs/Twist']); self.add_text_input('node_name','Name',text='cpp_cmd_vel_sub'); self.add_text_input('topic_name','Topic',text='/cmd_vel'); self._init_template('cpp_twist_sub'); self.set_color(100,50,0)

# ==============================================================================
#                                 CUSTOM LOGIC NODES
# ==============================================================================

class CppCustomNode(RosNodeBase):
    __identifier__='ros.cpp'; NODE_NAME='CustomNode'
    def __init__(self): 
        super().__init__()
        self.add_input('in_data', color=(100,100,255), multi_input=True)
        self.add_text_input('node_name','Name',text='cpp_logic')
        self.add_text_input('topic_name','Base Topic',text='/data')
        self.add_text_input('class_name', 'Class Name', text='MyCppClass')
        self._init_template('cpp_custom')
        self.set_color(40,40,80)

class PyCustomNode(RosNodeBase):
    __identifier__ = 'ros.py'
    NODE_NAME = 'CustomNode'
    
    def __init__(self): 
        super().__init__()
        self.add_input('in_data', color=MSG_COLORS['Blue'], multi_input=True)
        self.add_text_input('node_name', 'Node Name', text='my_node')
        self.add_text_input('topic_name', 'Base Topic', text='/data')
        self.add_text_input('class_name', 'Class Name', text='MyClass')
        self._init_template('py_custom')

# --- COURSEWORK NODES ---

class LevelBuilderNode(PyCustomNode):
    __identifier__ = 'ros.coursework'
    NODE_NAME = 'LevelBuilderNode'
    def __init__(self):
        super().__init__()
        self.get_widget('node_name').set_value('level_builder')
        self.get_widget('class_name').set_value('LevelBuilder')

class HiveMindNode(PyCustomNode):
    __identifier__ = 'ros.coursework'
    NODE_NAME = 'HiveMindNode'
    def __init__(self):
        super().__init__()
        self.get_widget('node_name').set_value('hive_mind')
        self.get_widget('class_name').set_value('HiveMind')

class SmartAgentNode(PyCustomNode):
    __identifier__ = 'ros.coursework'
    NODE_NAME = 'SmartAgentNode'
    def __init__(self):
        super().__init__()
        self.get_widget('node_name').set_value('agent_1')
        self.get_widget('class_name').set_value('SmartAgent')