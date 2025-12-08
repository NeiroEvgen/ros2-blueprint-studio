import json
import logging

class ProjectManager:
    def __init__(self, graph_py, graph_cpp):
        """
        Manages saving and loading of the dual-graph environment.
        """
        self.graph_py = graph_py
        self.graph_cpp = graph_cpp

    def save_project(self, file_path):
        """
        Saves the state of both graphs into a single JSON file.
        """
        try:
            # 1. Serialize both sessions
            py_data = self.graph_py.serialize_session()
            cpp_data = self.graph_cpp.serialize_session()

            # 2. Structure the project data
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

            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(project_data, f, indent=4, ensure_ascii=False)
            
            logging.info(f"Project saved to: {file_path}")
            return True

        except Exception as e:
            logging.error(f"Save error: {e}")
            return False

    def load_project(self, file_path):
        """
        Loads the project from a file, clearing current sessions first.
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                project_data = json.load(f)

            if "graphs" not in project_data:
                raise ValueError("Invalid project file (missing 'graphs' key)")

            # 1. Clear current sessions
            self.graph_py.clear_session()
            self.graph_cpp.clear_session()

            # 2. Load Python graph
            py_data = project_data["graphs"].get("python")
            if py_data:
                self.graph_py.deserialize_session(py_data)

            # 3. Load C++ graph
            cpp_data = project_data["graphs"].get("cpp")
            if cpp_data:
                self.graph_cpp.deserialize_session(cpp_data)

            logging.info(f"Project loaded: {file_path}")
            return True

        except Exception as e:
            logging.error(f"Load error: {e}")
            return False