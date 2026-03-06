from ..base import RosNodeBase

class PyTimerNode(RosNodeBase):
    __identifier__ = 'ros.py'
    NODE_NAME = 'Timer (Start)'
    
    def __init__(self):
        super().__init__()
        self.add_text_input('node_name', 'Name', text='timer_node')
        self.add_text_input('interval', 'Interval (s)', text='1.0')
        self.add_exec_output('ExecOut')
        self._init_template('Timer') 
        self.set_color(30, 150, 30)

class PyPrintNode(RosNodeBase):
    __identifier__ = 'ros.py'
    NODE_NAME = 'Print Action'
    
    def __init__(self):
        super().__init__()
        self.add_exec_input('ExecIn')
        self.add_exec_output('ExecOut')
        self.add_text_input('node_name', 'Name', text='printer')
        self.add_text_input('message', 'Message', text='Hello World')
        self._init_template('Default')
        self.set_color(50, 50, 50)