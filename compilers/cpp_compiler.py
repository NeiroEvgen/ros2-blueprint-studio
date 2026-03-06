import re
from .base_compiler import BaseCompiler
from nodes.templates import CPP_TEMPLATES

class CppCompiler(BaseCompiler):
    def compile_node(self, node, data):
        """Генерирует код для ОДНОЙ C++ ноды"""
        raw_code = data.get('code')
        
        if not raw_code or "Error: Template" in raw_code:
            key = data.get('template_key')
            if key: 
                k_low = key.lower()
                if "custom" in k_low: key = "Custom"
                elif "pub" in k_low: key = "Publisher"
                elif "sub" in k_low: key = "Subscriber"
                elif "timer" in k_low: key = "Timer"
                raw_code = CPP_TEMPLATES.get(key, "")
                
        if not raw_code: return None

        exec_in_init = ""
        exec_out_init = ""
        exec_vars = ""      
        exec_fire_code = ""

        class_name = data.get('class_name', 'MyNode')

        # Входящие (Subscribers) - Обрабатываем ТОЛЬКО порты триггеров (Exec)
        for in_port in node.input_ports():
            if "Exec" in in_port.name() and in_port.connected_ports():
                src = in_port.connected_ports()[0]
                topic = self.connection_map.get((src.node().id, src.name()))
                if topic:
                    var_name = f"sub_{in_port.name()}_"
                    exec_in_init += f"        {var_name} = this->create_subscription<std_msgs::msg::Empty>(\"{topic}\", 10, std::bind(&{class_name}::on_trigger, this, std::placeholders::_1));\n"
                    exec_vars += f"    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr {var_name};\n"

        # Исходящие (Publishers) - Обрабатываем ТОЛЬКО порты триггеров (Exec)
        for out_port in node.output_ports():
            if "Exec" in out_port.name():
                topic = self.connection_map.get((node.id, out_port.name()))
                if topic:
                    var_name = f"pub_{out_port.name()}_"
                    exec_out_init += f"        {var_name} = this->create_publisher<std_msgs::msg::Empty>(\"{topic}\", 10);\n"
                    exec_fire_code += f"        {var_name}->publish(std_msgs::msg::Empty());\n"
                    exec_vars += f"    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr {var_name};\n"

        # Подстановка
        code = raw_code
        for k, v in data.items():
            if k not in ['code']: code = code.replace(f"{{{k}}}", str(v))

        code = code.replace("{EXEC_IN_INIT}", exec_in_init)
        code = code.replace("{EXEC_OUT_INIT}", exec_out_init)
        code = code.replace("{EXEC_FIRE}", exec_fire_code)
        code = code.replace("{EXEC_VARS}", exec_vars) 

        return code
    
    def generate_cmakelists(self, node_names):
        """Умный генератор CMakeLists.txt (STATELESS - сканирует граф сам)"""
        content = "cmake_minimum_required(VERSION 3.8)\nproject(cpp_blueprints_pkg)\n\n"
        content += "find_package(ament_cmake REQUIRED)\n"
        
        # 🌟 СКАНИРУЕМ ГРАФ ЗАНОВО ПРЯМО ЗДЕСЬ 🌟
        deps = set(['rclcpp', 'std_msgs'])
        pattern_msg = re.compile(r'#include\s+["<]([^/]+)/(msg|srv|action)/[^">]+[">]')
        pattern_libs = re.compile(r'#include\s+["<](tf2_ros|tf2|tf2_geometry_msgs|image_transport|cv_bridge)/[^">]+[">]')
        
        for node in self.graph.all_nodes():
            if hasattr(node, 'get_template_data'):
                data = node.get_template_data()
                code = data.get('code', '')
                
                # Если кода нет, берем из шаблона для проверки инклудов
                if not code or "Error: Template" in code:
                    key = data.get('template_key')
                    if key: 
                        k_low = key.lower()
                        if "custom" in k_low: key = "Custom"
                        elif "pub" in k_low: key = "Publisher"
                        elif "sub" in k_low: key = "Subscriber"
                        elif "timer" in k_low: key = "Timer"
                        code = CPP_TEMPLATES.get(key, "")
                        
                if code:
                    for match in pattern_msg.findall(code):
                        deps.add(match[0])
                    for match in pattern_libs.findall(code):
                        deps.add(match)
        
        # 1. Прописываем все найденные парсером библиотеки
        for dep in sorted(deps):
            content += f"find_package({dep} REQUIRED)\n"
        content += "\n"
        
        deps_str = " ".join(sorted(deps))
        
        # 2. Линкуем их ко всем экзешникам
        for name in node_names:
            content += f"add_executable({name} cpp/{name}.cpp)\n"
            content += f"ament_target_dependencies({name} {deps_str})\n"
            content += f"install(TARGETS {name} DESTINATION lib/${{PROJECT_NAME}})\n\n"
            
        content += "ament_package()\n"
        return content