from NodeGraphQt import BaseNode, GroupNode

class RosNodeBase(BaseNode):
    def get_template_data(self): return {}

# === ГРУППОВАЯ НОДА ===
class RosGroupNode(GroupNode):
    __identifier__ = 'ros.nodes'
    NODE_NAME = 'RosGroup'
    
    def __init__(self):
        super(RosGroupNode, self).__init__()
        self.set_color(50, 50, 50)

# ================================
#       PYTHON NODES
# ================================

class RosPyPublisherNode(RosNodeBase):
    __identifier__ = 'ros.nodes.python'
    NODE_NAME = 'RosPyPublisher'
    
    def __init__(self):
        super().__init__()
        self.add_output('out', color=(100, 255, 100))
        self.add_text_input('node_name', 'Node Name', text='py_talker')
        self.add_text_input('msg_data', 'Message', text='Hello Python')

    def get_template_data(self):
        return {
            'language': 'python', 'type': 'publisher',
            'name': self.get_property('node_name'), 'msg': self.get_property('msg_data'),
            'template': """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        # {publish_code}
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = '{msg_data} ' + str(self.i)
        # {publish_call}
        self.get_logger().info(f'PyPub: {{msg.data}}')
        self.i += 1

def main():
    rclpy.init()
    node = {class_name}()
    try: rclpy.spin(node)
    except: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()"""
        }

class RosPySubscriberNode(RosNodeBase):
    __identifier__ = 'ros.nodes.python'
    NODE_NAME = 'RosPySubscriber'

    def __init__(self):
        super().__init__()
        self.add_input('in', color=(100, 255, 100))
        self.add_text_input('node_name', 'Node Name', text='py_listener')

    def get_template_data(self):
        return {
            'language': 'python', 'type': 'subscriber',
            'name': self.get_property('node_name'),
            'template': """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        # {subscribe_code}

    def listener_callback(self, msg):
        self.get_logger().info(f'PySub heard: {{msg.data}}')

def main():
    rclpy.init()
    node = {class_name}()
    try: rclpy.spin(node)
    except: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()"""
        }

class RosPyCustomNode(RosNodeBase):
    __identifier__ = 'ros.nodes.python'
    NODE_NAME = 'RosPyCustom'
    
    def __init__(self):
        super().__init__()
        self.add_input('in', color=(100, 255, 100))
        self.add_output('out', color=(100, 255, 100))
        self.add_text_input('node_name', 'Node Name', text='my_py_logic')
        
        default_code = """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        # {subscribe_code}
        # {publish_code}

    def listener_callback(self, msg):
        self.get_logger().info(f'Custom Logic: {{msg.data}}')
        out_msg = String()
        out_msg.data = msg.data + "_processed"
        # {publish_call}

def main():
    rclpy.init()
    node = {class_name}()
    try: rclpy.spin(node)
    except: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
"""
        self.create_property('code_content', default_code)

    def get_template_data(self):
        return {
            'language': 'python', 'type': 'custom',
            'name': self.get_property('node_name'),
            'template': self.get_property('code_content')
        }

# ================================
#       C++ NODES
# ================================

class RosCppPublisherNode(RosNodeBase):
    __identifier__ = 'ros.nodes.cpp'
    NODE_NAME = 'RosCppPublisher'
    
    def __init__(self):
        super().__init__()
        self.add_output('out', color=(100, 100, 255))
        self.add_text_input('node_name', 'Node Name', text='cpp_talker')
        self.add_text_input('msg_data', 'Message', text='Hello C++')

    def get_template_data(self):
        return {
            'language': 'cpp', 'type': 'publisher',
            'name': self.get_property('node_name'), 'msg': self.get_property('msg_data'),
            'template': """
#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class {class_name} : public rclcpp::Node {
public:
  {class_name}() : Node("{node_name}"), count_(0) {
    // {publish_code}
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&{class_name}::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "{msg_data} " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "CppPub: '%s'", message.data.c_str());
    // {publish_call}
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<{class_name}>());
  rclcpp::shutdown();
  return 0;
}
"""
        }

class RosCppSubscriberNode(RosNodeBase):
    __identifier__ = 'ros.nodes.cpp'
    NODE_NAME = 'RosCppSubscriber'
    
    def __init__(self):
        super().__init__()
        self.add_input('in', color=(100, 100, 255))
        self.add_text_input('node_name', 'Node Name', text='cpp_listener')

    def get_template_data(self):
        return {
            'language': 'cpp', 'type': 'subscriber',
            'name': self.get_property('node_name'),
            'template': """
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class {class_name} : public rclcpp::Node {
public:
  {class_name}() : Node("{node_name}") {
    // {subscribe_code}
  }

private:
  void listener_callback(const std_msgs::msg::String & msg) const {
    RCLCPP_INFO(this->get_logger(), "CppSub heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<{class_name}>());
  rclcpp::shutdown();
  return 0;
}
"""
        }

class RosCppCustomNode(RosNodeBase):
    __identifier__ = 'ros.nodes.cpp'
    NODE_NAME = 'RosCppCustom'
    
    def __init__(self):
        super().__init__()
        self.add_input('in', color=(100, 100, 255))
        self.add_output('out', color=(100, 100, 255))
        self.add_text_input('node_name', 'Node Name', text='my_cpp_logic')
        
        default_code = """#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class {class_name} : public rclcpp::Node {
public:
  {class_name}() : Node("{node_name}") {
    // {subscribe_code}
    // {publish_code}
  }

private:
  void listener_callback(const std_msgs::msg::String & msg) {
     std::string old_data = msg.data;
     std::string new_data = old_data + " -> [CPP PROCESSED]";
     RCLCPP_INFO(this->get_logger(), "Processing: '%s'", new_data.c_str());
     auto message = std_msgs::msg::String();
     message.data = new_data;
     // {publish_call}
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<{class_name}>());
  rclcpp::shutdown();
  return 0;
}
"""
        self.create_property('code_content', default_code)

    def get_template_data(self):
        return {
            'language': 'cpp',
            'type': 'custom',
            'name': self.get_property('node_name'),
            'template': self.get_property('code_content')
        }