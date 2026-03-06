from NodeGraphQt import BaseNode
# Импортируем словари раздельно
from .templates import PYTHON_TEMPLATES, CPP_TEMPLATES

# Цвета
EXEC_COLOR = (255, 255, 255) # Белый для Логики
DATA_COLOR = (100, 100, 100)

MSG_COLORS = {
    "std_msgs/Bool":    (230, 50, 50),
    "std_msgs/Int32":   (0, 200, 200),
    "std_msgs/String":  (255, 0, 255),
    "geometry_msgs/Twist": (255, 140, 0),
    "sensor_msgs/Image":   (100, 100, 255),
    "DEFAULT": (100, 100, 100)
}

class RosNodeBase(BaseNode):
    def __init__(self):
        super(RosNodeBase, self).__init__()
        self.create_property('saved_ports_config', value=[], widget_type=0)
    
    def _init_template(self, template_key):
        """
        Умная загрузка: переводит старые ключи в новые универсальные
        и выбирает правильный язык.
        """
        self.create_property('template_key', template_key)

        is_cpp_id = 'cpp' in self.type_
        is_cpp_class = self.__class__.__name__.startswith('Cpp')
        is_cpp = is_cpp_id or is_cpp_class

        key_lower = template_key.lower()
        final_key = "Default" 
    
        if "custom" in key_lower:
            final_key = "Custom"
        elif "timer" in key_lower:
            final_key = "Timer"
        elif "pub" in key_lower:
            final_key = "Publisher"
        elif "sub" in key_lower:
            final_key = "Subscriber"
        elif "action" in key_lower:
            final_key = "ActionClient"
        elif "trigger" in key_lower:
            final_key = "Default"

        target_dict = CPP_TEMPLATES if is_cpp else PYTHON_TEMPLATES
        
        raw_code = target_dict.get(final_key, target_dict.get("Default", ""))
        
        if not raw_code:
            raw_code = f"// Error: Template '{final_key}' not found for lang {'Cpp' if is_cpp else 'Py'}"

        self.create_property('code_content', raw_code, widget_type=0)

    
    def add_exec_input(self, name='ExecIn'):
        return self.add_input(name, color=EXEC_COLOR, display_name=True)

    def add_exec_output(self, name='ExecOut'):
        return self.add_output(name, color=EXEC_COLOR, display_name=True)

    def get_template_data(self):
        c_name = self.get_property('class_name')
        n_name = self.get_property('node_name')
        final_class_name = c_name if c_name else n_name.replace(" ", "_")
        
        is_cpp_id = 'cpp' in self.type_
        is_cpp_class = self.__class__.__name__.startswith('Cpp')
        lang = 'cpp' if (is_cpp_id or is_cpp_class) else 'python'

        data = {
            'language': lang,
            'name': n_name,
            'topic': self.get_property('topic_name'),
            'class_name': final_class_name,
            'template_key': self.get_property('template_key'),
            'code': self.get_property('code_content')
        }
        
        if self.has_property('interval'):
            data['interval'] = self.get_property('interval')
        if self.has_property('message'):
            data['message'] = self.get_property('message')
            
        return data