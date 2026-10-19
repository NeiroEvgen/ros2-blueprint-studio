import os
import time
import re
from PySide6 import QtCore

class FileWatcher(QtCore.QObject):
    # Signal: file path, content, list of found ports
    file_changed = QtCore.Signal(str, str, list)

    def __init__(self):
        super().__init__()
        self.watcher = QtCore.QFileSystemWatcher()
        self.watcher.fileChanged.connect(self.on_file_changed)
        # watched files, so nothing is added twice
        self.watching_files = set()

    def start_watching(self, folder):
        """Watch every .py and .cpp file under src, recursively."""
        if not os.path.exists(folder): return
        
        # scan the folder and add the files to the watcher
        for root, dirs, files in os.walk(folder):
            for file in files:
                if file.endswith('.py') or file.endswith('.cpp'):
                    path = os.path.join(root, file)
                    path = os.path.normpath(path) # normalize Windows paths
                    
                    if path not in self.watching_files:
                        self.watcher.addPath(path)
                        self.watching_files.add(path)

    RESAVE_GRACE_MS = 300

    def on_file_changed(self, path):
        if not os.path.exists(path):
            # Atomic save (vim, some IDEs): the old file is deleted and the new one is about to
            # appear under the same name. Dropping the path here would stop syncing every later
            # edit from that editor into the node.
            QtCore.QTimer.singleShot(self.RESAVE_GRACE_MS, lambda: self._recheck(path))
            return
        self._process(path)

    def _recheck(self, path):
        if os.path.exists(path):
            self._process(path)
        elif path in self.watching_files:           # the file is really gone
            self.watcher.removePath(path)
            self.watching_files.discard(path)

    def _process(self, path):
        # the OS drops the watch when the file is replaced: re-arm it before any early return
        if path not in self.watcher.files():
            self.watcher.addPath(path)

        # give the IDE time to finish writing (avoids empty reads)
        time.sleep(0.1)
        
        content = ""
        # 3 read attempts (the file may be briefly locked by the OS)
        for _ in range(3):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    content = f.read()
                break
            except:
                time.sleep(0.1)
        
        if not content: return

        # scan ports according to the file extension
        ports = self._scan_ports(content, path)

        # emit the signal
        self.file_changed.emit(path, content, ports)

    def _scan_ports(self, content, filepath):
        found = []
        # flatten multi-line calls
        clean_content = content.replace('\n', ' ').replace('\r', '')
        
        is_cpp = filepath.endswith('.cpp')

        if is_cpp:
            # === C++ ===
            # looks for: create_publisher<Type>("topic", ...)
            # the pattern matches <...> then ("...")
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
            # === PYTHON ===
            # looks for: create_publisher(Type, 'topic', ...)
            # pattern: create_publisher ( Type , 'Topic'
            
            # 1. Publishers
            # \s*\(  -> opening parenthesis
            # \s*[^,]+, -> skip the type (String,)
            # \s*[\'"] -> opening quote
            # ([^\'"]+) -> GROUP 1: topic name
            py_pub = r'create_publisher\s*\(\s*[^,]+,\s*[\'"]([^\'"]+)[\'"]'
            
            for match in re.finditer(py_pub, clean_content):
                topic = match.group(1)
                
                name = topic.split('/')[-1]
                found.append({'mode': 'pub', 'name': name, 'topic': topic})

            # 2. Subscribers
            py_sub = r'create_subscription\s*\(\s*[^,]+,\s*[\'"]([^\'"]+)[\'"]'
            
            for match in re.finditer(py_sub, clean_content):
                topic = match.group(1)
                name = topic.split('/')[-1]
                found.append({'mode': 'sub', 'name': name, 'topic': topic})
                
        return found