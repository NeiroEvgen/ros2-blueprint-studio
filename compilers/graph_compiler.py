from compilers.base_compiler import BaseCompiler
from compilers.py_compiler import PythonCompiler
from compilers.cpp_compiler import CppCompiler

class GraphCompiler:
    def __init__(self, graph):
        self.graph = graph

    def compile(self):
        print("\n=== COMPILER START (SEGREGATED) ===")
        all_nodes = self.graph.all_nodes()
        
        # 1. Строим общую карту связей
        base = BaseCompiler(self.graph)
        connection_map = base.build_connection_map()
        
        # 2. Создаем специализированные компиляторы
        py_comp = PythonCompiler(self.graph)
        py_comp.connection_map = connection_map
        
        cpp_comp = CppCompiler(self.graph)
        cpp_comp.connection_map = connection_map
        
        compiled_scripts = []
        
        # 3. Распределяем работу
        for node in all_nodes:
            if not hasattr(node, 'get_template_data'): continue
            data = node.get_template_data()
            lang = data.get('language', 'python')
            name = data.get('name', 'node')

            try:
                if lang == 'python':
                    code = py_comp.compile_node(node, data)
                    filename = f"{name}.py"
                elif lang == 'cpp':
                    code = cpp_comp.compile_node(node, data)
                    filename = f"{name}.cpp"
                else:
                    continue
                
                if code:
                    compiled_scripts.append({
                        'language': lang, 
                        'filename': filename, 
                        'code': code
                    })
                    print(f"  -> Compiled {lang.upper()}: {name}")

            except Exception as e:
                print(f"  ❌ Error compiling {name}: {e}")

        return compiled_scripts