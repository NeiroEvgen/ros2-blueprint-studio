from .base_compiler import BaseCompiler
from nodes.templates import TEMPLATES

class PythonCompiler(BaseCompiler):
    def compile_node(self, node, data):
        # 1. Получаем код шаблона
        raw_code = data.get('code')
        if not raw_code:
            key = data.get('template_key')
            if key: raw_code = TEMPLATES.get(key, "")
        if not raw_code: return None

        exec_in_code = ""
        exec_out_code = ""
        exec_fire_code = ""

        # 2. Генерируем код для ТРИГГЕРОВ
        # Входы (Subscribers)
        for in_port in node.input_ports():
            if "Exec" in in_port.name() and in_port.connected_ports():
                src = in_port.connected_ports()[0]
                topic = self.connection_map.get((src.node().id, src.name()))
                if topic:
                    exec_in_code += f"        self.sub_{in_port.name()} = self.create_subscription(Empty, '{topic}', self.on_trigger, 10)\n"

        # Выходы (Publishers)
        for out_port in node.output_ports():
            if "Exec" in out_port.name():
                topic = self.connection_map.get((node.id, out_port.name()))
                if topic:
                    exec_out_code += f"        self.pub_{out_port.name()} = self.create_publisher(Empty, '{topic}', 10)\n"
                    exec_fire_code += f"        self.pub_{out_port.name()}.publish(Empty())\n"

        # 3. ПОДСТАНОВКА (String Replace)
        code = raw_code
        
        # Сначала заменяем простые переменные ({interval}, {message}, {node_name})
        for k, v in data.items():
            if k != 'code': 
                code = code.replace(f"{{{k}}}", str(v))

        # Потом заменяем блоки кода триггеров
        code = code.replace("{EXEC_IN_INIT}", exec_in_code)
        code = code.replace("{EXEC_OUT_INIT}", exec_out_code)
        code = code.replace("{EXEC_FIRE}", exec_fire_code)
        
        # Чистим C++ мусор (на всякий случай)
        code = code.replace("{EXEC_VARS}", "")

        return code