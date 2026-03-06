from ..base import RosNodeBase

class CppTimerNode(RosNodeBase):
    __identifier__ = 'ros.cpp' # ВАЖНО: Явно указываем C++
    NODE_NAME = 'Cpp Timer'
    
    def __init__(self):
        super().__init__()
        self.add_text_input('node_name', 'Name', text='cpp_timer')
        self.add_text_input('interval', 'Interval (s)', text='1.0')
        self.add_exec_output('ExecOut') 
        # Шаблон ищется в CPP_TEMPLATES
        self._init_template('Timer')
        self.set_color(20, 100, 20)

class CppPrintNode(RosNodeBase):
    __identifier__ = 'ros.cpp'
    NODE_NAME = 'Cpp Print Action'
    
    def __init__(self):
        super().__init__()
        self.add_exec_input('ExecIn')
        self.add_exec_output('ExecOut')
        self.add_text_input('node_name', 'Name', text='cpp_printer')
        self.add_text_input('message', 'Message', text='Hello Cpp')
        self._init_template('Default')
        self.set_color(40, 40, 60)