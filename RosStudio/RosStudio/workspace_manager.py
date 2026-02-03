import os
import shutil
from PySide6 import QtCore

class WorkspaceManager:
    APP_DIR_NAME = "BlueprintStudioProjects"

    @staticmethod
    def get_workspace_root():
        """Возвращает путь к C:/Users/User/Documents/BlueprintStudioProjects"""
        docs_loc = QtCore.QStandardPaths.writableLocation(QtCore.QStandardPaths.DocumentsLocation)
        root = os.path.join(docs_loc, WorkspaceManager.APP_DIR_NAME)
        if not os.path.exists(root):
            os.makedirs(root)
        return root

    @staticmethod
    def list_projects():
        """Сканирует папки проектов"""
        root = WorkspaceManager.get_workspace_root()
        projects = []
        if os.path.exists(root):
            for name in os.listdir(root):
                full_path = os.path.join(root, name)
                if os.path.isdir(full_path):
                    projects.append(name)
        return sorted(projects)

    @staticmethod
    def create_project(name):
        """Создает новую папку проекта"""
        # Убираем пробелы и спецсимволы для безопасности
        safe_name = "".join([c for c in name if c.isalnum() or c in (' ', '_', '-')]).strip()
        if not safe_name:
            raise ValueError("Invalid project name")
            
        root = WorkspaceManager.get_workspace_root()
        project_path = os.path.join(root, safe_name)
        
        if os.path.exists(project_path):
            raise FileExistsError(f"Project '{safe_name}' already exists.")
            
        os.makedirs(project_path)
        return project_path