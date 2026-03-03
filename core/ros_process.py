import traceback
import re
from PySide6 import QtCore

class LogMonitorWorker(QtCore.QThread):
    new_log_line = QtCore.Signal(str)
    
    def __init__(self, manager):
        super().__init__()
        self.manager = manager
        self.is_running = True

    def run(self):
        if not self.manager or not self.manager.container: return
        try:
            # Читаем логи потоком
            for chunk in self.manager.container.logs(stream=True, follow=True):
                if not self.is_running: break
                text = chunk.decode('utf-8', errors='replace')
                self.new_log_line.emit(text)
        except Exception: 
            pass

    def stop(self): 
        self.is_running = False

class DeployWorker(QtCore.QThread):
    sys_signal = QtCore.Signal(str)    # Для System Log
    ros_signal = QtCore.Signal(str)    # Для Dashboard
    finished_signal = QtCore.Signal()
    
    def __init__(self, manager, project_path):
        super().__init__()
        self.manager = manager
        self.project_path = project_path
        self.is_running = True

    def run(self):
        self.sys_signal.emit("--- DEPLOY START ---")
        try:
            # 1. Проверка образа
            self.manager.ensure_image(lambda m: self.sys_signal.emit(m))
            
            # 2. Старт контейнера
            self.manager.start_session(self.project_path, lambda m: self.sys_signal.emit(m))
            
            # 3. Запуск ROS launch
            self.manager.run_project_launch(
                sys_callback=lambda m: self.sys_signal.emit(m),
                ros_callback=lambda m: self.ros_signal.emit(m)
            )
            
        except Exception as e:
            err_str = str(e)
            if "Container is not running" in err_str or "Read timed out" in err_str:
                self.sys_signal.emit(" ROS 2 Session Stopped.")
            else:
                self.sys_signal.emit(f" ERROR: {err_str}")
                traceback.print_exc()
        finally:
            self.finished_signal.emit()

    def stop(self):
        self.is_running = False
        try:
            self.manager.container.stop()
        except: pass