from NodeGraphQt import BaseNode
from functools import partial

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
    'Any':                 (180, 180, 180)
}

# ==============================================================================
#                                 TEMPLATES
# ==============================================================================
TEMPLATES = {
    # --- PYTHON CUSTOM ---
    'py_custom': """import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        
        # [AUTO-GEN-SUBS]
        # Uses {topic_name}, which the compiler will replace with the real topic
        self.sub = self.create_subscription(String, '{topic_name}', self.listener_callback, 10)
        
        # [AUTO-GEN-PUBS]
        # New publishers will be injected here automatically

    def listener_callback(self, msg):
        self.get_logger().info(f'Custom Got: {msg.data}')
        
        # Example logic (uncomment if you added port out_twist_1):
        # cmd = Twist()
        # cmd.linear.x = 2.0
        # if hasattr(self, 'pub_out_twist_1'):
        #     self.pub_out_twist_1.publish(cmd)

def main():
    rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
""",

    # --- PYTHON PUB/SUB ---
    'py_string_pub': """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.pub = self.create_publisher(String, '{topic_name}', 10)
        self.tmr = self.create_timer(1.0, self.cb)
        self.i=0
    def cb(self):
        msg = String(); msg.data = "Py: " + str(self.i)
        self.pub.publish(msg); self.i+=1

def main():
    rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
""",

    'py_string_sub': """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.sub = self.create_subscription(String, '{topic_name}', self.cb, 10)
    def cb(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')

def main():
    rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
""",

    'py_twist_pub': """import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.pub = self.create_publisher(Twist, '{topic_name}', 10)
        self.tmr = self.create_timer(1.0, self.cb)
    def cb(self):
        msg = Twist(); msg.linear.x = 2.0; msg.angular.z = 1.0
        self.pub.publish(msg)

def main():
    rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
""",

    'py_twist_sub': """import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.sub = self.create_subscription(Twist, '{topic_name}', self.cb, 10)
    def cb(self, msg):
        self.get_logger().info(f'Speed: {msg.linear.x}')

def main():
    rclpy.init(); n={class_name}(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
""",

    # --- C++ ---
    'cpp_custom': """#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class {class_name} : public rclcpp::Node {
public:
  {class_name}() : Node("{node_name}") {
    // [AUTO-GEN-SUBS]
    sub_ = this->create_subscription<std_msgs::msg::String>("{topic_name}", 10, std::bind(&{class_name}::cb, this, _1));
    // [AUTO-GEN-PUBS]
  }
private:
  void cb(const std_msgs::msg::String & msg) const {
    RCLCPP_INFO(this->get_logger(), "Custom: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  // [AUTO-GEN-VARS]
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<{class_name}>());
  rclcpp::shutdown(); return 0;
}
""",

    'cpp_string_pub': """#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class {class_name} : public rclcpp::Node {
public:
  {class_name}() : Node("{node_name}"), i(0) {
    pub_ = this->create_publisher<std_msgs::msg::String>("{topic_name}", 10);
    tmr_ = this->create_wall_timer(1s, [this](){
      auto m = std_msgs::msg::String(); m.data="Cpp: "+std::to_string(i++);
      pub_->publish(m);
    });
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr tmr_;
  size_t i;
};
int main(int argc, char** a){rclcpp::init(argc,a);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}
""",

    'cpp_string_sub': """#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class {class_name} : public rclcpp::Node {
public:
  {class_name}() : Node("{node_name}") {
    sub_ = this->create_subscription<std_msgs::msg::String>("{topic_name}", 10, std::bind(&{class_name}::cb, this, _1));
  }
private:
  void cb(const std_msgs::msg::String & m) const {RCLCPP_INFO(this->get_logger(), "Heard: %s", m.data.c_str());}
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
int main(int argc, char** a){rclcpp::init(argc,a);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}
""",

    'cpp_twist_pub': """#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std::chrono_literals;

class {class_name} : public rclcpp::Node {
public:
  {class_name}() : Node("{node_name}") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("{topic_name}", 10);
    tmr_ = this->create_wall_timer(1s, [this](){
      auto m = geometry_msgs::msg::Twist(); m.linear.x=2.0; m.angular.z=1.0;
      pub_->publish(m);
    });
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr tmr_;
};
int main(int argc, char** a){rclcpp::init(argc,a);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}
""",

    'cpp_twist_sub': """#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class {class_name} : public rclcpp::Node {
public:
  {class_name}() : Node("{node_name}") {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("{topic_name}", 10, std::bind(&{class_name}::cb, this, _1));
  }
private:
  void cb(const geometry_msgs::msg::Twist & m) const {RCLCPP_INFO(this->get_logger(), "Speed: %.2f", m.linear.x);}
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};
int main(int argc, char** a){rclcpp::init(argc,a);rclcpp::spin(std::make_shared<{class_name}>());rclcpp::shutdown();return 0;}
"""
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
        return {
            'language': 'cpp' if 'cpp' in self.type_ else 'python',
            'name': self.get_property('node_name'),
            'topic': self.get_property('topic_name'),
            'template_key': self.get_property('template_key'),
            'code': self.get_property('code_content')
        }

# ==============================================================================
#                                 NODES
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

class PyCustomNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='CustomNode'
    def __init__(self): super().__init__(); self.add_input('in_data',color=(200,200,200)); self.add_text_input('node_name','Name',text='py_logic'); self.add_text_input('topic_name','Base Topic',text='/data'); self._init_template('py_custom')

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

class CppCustomNode(RosNodeBase):
    __identifier__='ros.cpp'; NODE_NAME='CustomNode'
    def __init__(self): super().__init__(); self.add_input('in_data',color=(100,100,255)); self.add_text_input('node_name','Name',text='cpp_logic'); self.add_text_input('topic_name','Base Topic',text='/data'); self._init_template('cpp_custom'); self.set_color(40,40,80)