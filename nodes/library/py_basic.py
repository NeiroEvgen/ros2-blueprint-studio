from ..base import RosNodeBase, MSG_COLORS

class PyStringPubNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='StringPub'
    def __init__(self): 
        super().__init__()
        self.add_output('out',color=MSG_COLORS['std_msgs/String'])
        self.add_text_input('node_name','Name',text='py_str_pub')
        self.add_text_input('topic_name','Topic',text='/chatter')
        self._init_template('py_string_pub')

class PyStringSubNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='StringSub'
    def __init__(self): 
        super().__init__()
        self.add_input('in',color=MSG_COLORS['std_msgs/String'])
        self.add_text_input('node_name','Name',text='py_str_sub')
        self.add_text_input('topic_name','Topic',text='/chatter')
        self._init_template('py_string_sub')