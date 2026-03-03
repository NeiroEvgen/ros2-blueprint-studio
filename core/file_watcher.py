import os
import time
import re
from PySide6 import QtCore

class FileWatcher(QtCore.QObject):
    # Сигнал: путь_к_файлу, содержимое, список_найденных_портов
    file_changed = QtCore.Signal(str, str, list)

    def __init__(self):
        super().__init__()
        self.watcher = QtCore.QFileSystemWatcher()
        self.watcher.fileChanged.connect(self.on_file_changed)
        # Храним список отслеживаемых файлов, чтобы не добавлять дважды
        self.watching_files = set()

    def start_watching(self, folder):
        """Рекурсивно подписываемся на все .py и .cpp файлы в папке src"""
        if not os.path.exists(folder): return
        
        # Сканируем папку и добавляем файлы в watcher
        for root, dirs, files in os.walk(folder):
            for file in files:
                if file.endswith('.py') or file.endswith('.cpp'):
                    path = os.path.join(root, file)
                    path = os.path.normpath(path) # Нормализация пути Windows
                    
                    if path not in self.watching_files:
                        self.watcher.addPath(path)
                        self.watching_files.add(path)
                        # print(f"DEBUG: Watching {file}")

    def on_file_changed(self, path):
        # Если файл удалили
        if not os.path.exists(path):
            if path in self.watching_files:
                self.watcher.removePath(path)
                self.watching_files.remove(path)
            return
        
        # Задержка, чтобы IDE успела дописать файл (убирает пустые чтения)
        time.sleep(0.1)
        
        content = ""
        # 3 попытки чтения (защита от блокировки файла системой)
        for _ in range(3):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    content = f.read()
                break
            except:
                time.sleep(0.1)
        
        if not content: return

        # Сканируем порты с учетом расширения файла
        ports = self._scan_ports(content, path)
        
        # Отправляем сигнал
        self.file_changed.emit(path, content, ports)

    def _scan_ports(self, content, filepath):
        found = []
        # Убираем переносы строк для многострочных команд
        clean_content = content.replace('\n', ' ').replace('\r', '')
        
        is_cpp = filepath.endswith('.cpp')

        if is_cpp:
            # === C++ ЛОГИКА ===
            # Ищем: create_publisher<Type>("topic", ...)
            # Паттерн ищет <...> затем ("...")
            pub_pattern = r'create_publisher\s*<\s*([\w:]+)\s*>\s*\(\s*["\']([^"\']+)["\']'
            for match in re.finditer(pub_pattern, clean_content):
                topic = match.group(2)
                name = topic.split('/')[-1]
                found.append({'mode': 'pub', 'name': name, 'topic': topic})

            sub_pattern = r'create_subscription\s*<\s*([\w:]+)\s*>\s*\(\s*["\']([^"\']+)["\']'
            for match in re.finditer(sub_pattern, clean_content):
                topic = match.group(2)
                name = topic.split('/')[-1]
                found.append({'mode': 'sub', 'name': name, 'topic': topic})
        
        else:
            # === PYTHON ЛОГИКА ===
            # Ищем: create_publisher(Type, 'topic', ...)
            # Паттерн: create_publisher ( Type , 'Topic'
            
            # 1. Паблишеры
            # \s*\(  -> открывающая скобка
            # \s*[^,]+, -> пропускаем Тип (String,)
            # \s*[\'"] -> открывающая кавычка
            # ([^\'"]+) -> ГРУППА 1: Имя топика
            py_pub = r'create_publisher\s*\(\s*[^,]+,\s*[\'"]([^\'"]+)[\'"]'
            
            for match in re.finditer(py_pub, clean_content):
                topic = match.group(1)
                # Игнорируем системный триггер topic_1, если хотим (или оставляем)
                # if topic == "topic_1": continue 
                
                name = topic.split('/')[-1]
                found.append({'mode': 'pub', 'name': name, 'topic': topic})

            # 2. Подписчики
            py_sub = r'create_subscription\s*\(\s*[^,]+,\s*[\'"]([^\'"]+)[\'"]'
            
            for match in re.finditer(py_sub, clean_content):
                topic = match.group(1)
                name = topic.split('/')[-1]
                found.append({'mode': 'sub', 'name': name, 'topic': topic})
                
        return found