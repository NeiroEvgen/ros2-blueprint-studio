from .base_compiler import BaseCompiler
from nodes.templates import TEMPLATES

class CppCompiler(BaseCompiler):
    def compile_node(self, node, data):
        """Генерирует код для ОДНОЙ C++ ноды"""
        
        raw_code = data.get('code')
        if not raw_code:
            key = data.get('template_key')
            if key: raw_code = TEMPLATES.get(key, "")
        if not raw_code: return None

        # Блоки кода для C++
        exec_in_init = ""
        exec_out_init = ""
        exec_vars = ""      # <--- ВАЖНО: Объявления переменных (.h style)
        exec_fire_code = ""

        class_name = data.get('class_name', 'MyNode')

        # Входящие (Subscribers)
        for in_port in node.input_ports():
            if "Exec" in in_port.name() and in_port.connected_ports():
                src = in_port.connected_ports()[0]
                topic = self.connection_map.get((src.node().id, src.name()))
                if topic:
                    var_name = f"sub_{in_port.name()}_"
                    # Инициализация (в конструкторе)
                    exec_in_init += f"        {var_name} = this->create_subscription<std_msgs::msg::Empty>(\"{topic}\", 10, std::bind(&{class_name}::on_trigger, this, _1));\n"
                    # Объявление (в private)
                    exec_vars += f"    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr {var_name};\n"

        # Исходящие (Publishers)
        for out_port in node.output_ports():
            if "Exec" in out_port.name():
                topic = self.connection_map.get((node.id, out_port.name()))
                if topic:
                    var_name = f"pub_{out_port.name()}_"
                    # Инициализация
                    exec_out_init += f"        {var_name} = this->create_publisher<std_msgs::msg::Empty>(\"{topic}\", 10);\n"
                    # Логика вызова
                    exec_fire_code += f"        {var_name}->publish(std_msgs::msg::Empty());\n"
                    # Объявление
                    exec_vars += f"    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr {var_name};\n"

        # Подстановка
        code = raw_code
        # ... (стандартные замены) ...
        for k, v in data.items():
            if k not in ['code']: code = code.replace(f"{{{k}}}", str(v))

        code = code.replace("{EXEC_IN_INIT}", exec_in_init)
        code = code.replace("{EXEC_OUT_INIT}", exec_out_init)
        code = code.replace("{EXEC_FIRE}", exec_fire_code)
        code = code.replace("{EXEC_VARS}", exec_vars) # <--- Только для C++

        return code
    
    def generate_cmakelists(self, node_names):
        """Генератор CMakeLists.txt теперь живет ЗДЕСЬ, а не в ProjectManager"""
        content = "cmake_minimum_required(VERSION 3.8)\nproject(cpp_blueprints_pkg)\n\n"
        content += "find_package(ament_cmake REQUIRED)\nfind_package(rclcpp REQUIRED)\n"
        content += "find_package(std_msgs REQUIRED)\n\n"
        
        for name in node_names:
            content += f"add_executable({name} src/{name}.cpp)\n"
            content += f"ament_target_dependencies({name} rclcpp std_msgs)\n"
            content += f"install(TARGETS {name} DESTINATION lib/${{PROJECT_NAME}})\n\n"
            
        content += "ament_package()\n"
        return content