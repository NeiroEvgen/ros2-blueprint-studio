import json
import logging

class ProjectManager:
    def __init__(self, graph_py, graph_cpp):
        """
        Менеджер принимает ссылки на оба графа, чтобы управлять ими.
        """
        self.graph_py = graph_py
        self.graph_cpp = graph_cpp

    def save_project(self, file_path):
        """
        Сохраняет состояние обоих графов в один файл.
        """
        try:
            # 1. Сериализуем (превращаем в словарь) оба графа
            py_data = self.graph_py.serialize_session()
            cpp_data = self.graph_cpp.serialize_session()

            # 2. Формируем единую структуру проекта
            project_data = {
                "meta": {
                    "version": "2.0",
                    "description": "ROS2 Blueprint Project"
                },
                "graphs": {
                    "python": py_data,
                    "cpp": cpp_data
                }
            }

            # 3. Записываем в файл
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(project_data, f, indent=4, ensure_ascii=False)
            
            logging.info(f"Проект сохранен в: {file_path}")
            return True

        except Exception as e:
            logging.error(f"Ошибка сохранения проекта: {e}")
            return False

    def load_project(self, file_path):
        """
        Загружает проект из файла, очищая текущие графы.
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                project_data = json.load(f)

            # Проверка структуры файла
            if "graphs" not in project_data:
                raise ValueError("Некорректный файл проекта (нет ключа 'graphs')")

            # 1. Очищаем текущие сессии
            self.graph_py.clear_session()
            self.graph_cpp.clear_session()

            # 2. Загружаем Python граф
            py_data = project_data["graphs"].get("python")
            if py_data:
                self.graph_py.deserialize_session(py_data)

            # 3. Загружаем C++ граф
            cpp_data = project_data["graphs"].get("cpp")
            if cpp_data:
                self.graph_cpp.deserialize_session(cpp_data)

            logging.info(f"Проект загружен: {file_path}")
            return True

        except Exception as e:
            logging.error(f"Ошибка загрузки проекта: {e}")
            return False