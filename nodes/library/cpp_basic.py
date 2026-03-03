from ..base import RosNodeBase, MSG_COLORS

class CppStringPubNode(RosNodeBase):
    __identifier__='ros.cpp'; NODE_NAME='StringPub'
    def __init__(self): 
        super().__init__()
        self.add_output('out',color=MSG_COLORS['std_msgs/String'])
        self.add_text_input('node_name','Name',text='cpp_str_pub')
        self.add_text_input('topic_name','Topic',text='/chatter')
        self._init_template('cpp_string_pub')
        self.set_color(20,20,100)

class CppStringSubNode(RosNodeBase):
    __identifier__='ros.cpp'; NODE_NAME='StringSub'
    def __init__(self): 
        super().__init__()
        self.add_input('in',color=MSG_COLORS['std_msgs/String'])
        self.add_text_input('node_name','Name',text='cpp_str_sub')
        self.add_text_input('topic_name','Topic',text='/chatter')
        self._init_template('cpp_string_sub')
        self.set_color(20,20,100)