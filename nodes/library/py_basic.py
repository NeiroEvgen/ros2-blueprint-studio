from ..base import RosNodeBase, MSG_COLORS, SRV_COLOR, SRV_SERVER_NODE_COLOR, SRV_CLIENT_NODE_COLOR

class PyStringPubNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='StringPub'
    def __init__(self): 
        super().__init__()
        self.add_output('out',color=MSG_COLORS['std_msgs/String'])
        self.add_text_input('node_name','Name',text='py_str_pub')
        self.add_text_input('topic_name','Topic',text='/chatter')
        self.add_text_input('interval','Interval (s)',text='0.5')
        self._init_template('py_string_pub')

class PyStringSubNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME='StringSub'
    def __init__(self): 
        super().__init__()
        self.add_input('in',color=MSG_COLORS['std_msgs/String'])
        self.add_text_input('node_name','Name',text='py_str_sub')
        self.add_text_input('topic_name','Topic',text='/chatter')
        self._init_template('py_string_sub')

class PyCustomNode(RosNodeBase):
    __identifier__='ros.py'; NODE_NAME= 'CustomNode'

    def __init__(self):
        super(PyCustomNode, self).__init__()
        self.add_text_input('node_name', 'Node Name', text='my_custom_node')
        self.set_color(40, 40, 80)
        self._init_template('Custom')


class PyServiceServerNode(RosNodeBase):
    """Answers example_interfaces/srv/AddTwoInts requests (official tutorial: service and client)."""
    __identifier__ = 'ros.py'
    NODE_NAME = 'ServiceServer'

    def __init__(self):
        super().__init__()
        self.add_input('srv', color=SRV_COLOR)
        self.add_text_input('node_name', 'Name', text='add_two_ints_server')
        self.add_text_input('service_name', 'Service', text='/add_two_ints')
        self._init_template('py_service_server')
        self.set_color(*SRV_SERVER_NODE_COLOR)


class PyServiceClientNode(RosNodeBase):
    """Calls an AddTwoInts service every Interval seconds and logs the answer."""
    __identifier__ = 'ros.py'
    NODE_NAME = 'ServiceClient'

    def __init__(self):
        super().__init__()
        self.add_output('call', color=SRV_COLOR)
        self.add_text_input('node_name', 'Name', text='add_two_ints_client')
        self.add_text_input('service_name', 'Service', text='/add_two_ints')
        self.add_text_input('a', 'a', text='2')
        self.add_text_input('b', 'b', text='3')
        self.add_text_input('interval', 'Interval (s)', text='1.0')
        self._init_template('py_service_client')
        self.set_color(*SRV_CLIENT_NODE_COLOR)
