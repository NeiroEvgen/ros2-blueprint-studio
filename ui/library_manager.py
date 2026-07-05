from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, 
                             QPushButton, QListWidget, QLabel, QComboBox, QTextEdit)



class _InstallWorker(QThread):
    log_line = Signal(str)
    done = Signal(bool)

    def __init__(self, docker_manager, pkg_name, pkg_type):
        super().__init__()
        self.docker_manager = docker_manager
        self.pkg_name = pkg_name
        self.pkg_type = pkg_type

    def run(self):
        try:
            ok = self.docker_manager.install_package(
                self.pkg_name, self.pkg_type,
                output_callback=lambda m: self.log_line.emit(m)
            )
            self.done.emit(bool(ok))
        except Exception as e:
            self.log_line.emit(f"Error: {e}")
            self.done.emit(False)

class LibraryManager(QWidget):
    def __init__(self, docker_manager):
        super().__init__()
        self.docker_manager = docker_manager
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Заголовок
        header = QLabel("Library & Dependency Manager")
        header.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(header)
        
        # Поле ввода и выбор типа
        input_layout = QHBoxLayout()
        
        self.pkg_input = QLineEdit()
        self.pkg_input.setPlaceholderText("Enter package name (e.g. opencv-python or libopencv-dev)")
        
        self.type_combo = QComboBox()
        self.type_combo.addItems(["apt (System/C++)", "pip (Python)"])
        
        self.install_btn = QPushButton("Install")
        self.install_btn.clicked.connect(self.start_install)
        
        input_layout.addWidget(self.pkg_input)
        input_layout.addWidget(self.type_combo)
        input_layout.addWidget(self.install_btn)
        layout.addLayout(input_layout)
        
        # Лог установки
        layout.addWidget(QLabel("Installation Output:"))
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: 'Consolas';")
        layout.addWidget(self.log_output)
        
        # Быстрые пресеты
        presets_layout = QHBoxLayout()
        presets_layout.addWidget(QLabel("Common Libraries:"))
        
        common_libs = [
            ("OpenCV (C++)", "libopencv-dev", "apt"),
            ("OpenCV (Py)", "opencv-python", "pip"),
            ("NumPy", "numpy", "pip"),
            ("Pillow", "Pillow", "pip")
        ]
        
        for label, name, p_type in common_libs:
            btn = QPushButton(label)
            btn.clicked.connect(lambda checked, n=name, t=p_type: self.quick_install(n, t))
            presets_layout.addWidget(btn)
        
        presets_layout.addStretch()
        layout.addLayout(presets_layout)

    def append_log(self, text):
        self.log_output.append(text)
        # Прокрутка вниз
        self.log_output.verticalScrollBar().setValue(self.log_output.verticalScrollBar().maximum())

    def quick_install(self, name, p_type):
        self.pkg_input.setText(name)
        idx = 0 if p_type == "apt" else 1
        self.type_combo.setCurrentIndex(idx)
        self.start_install()

    def start_install(self):
        pkg_name = self.pkg_input.text().strip()
        if not pkg_name:
            return

        pkg_type = "apt" if self.type_combo.currentIndex() == 0 else "pip"

        self.install_btn.setEnabled(False)
        self.log_output.clear()

        self._worker = _InstallWorker(self.docker_manager, pkg_name, pkg_type)
        self._worker.log_line.connect(self.append_log)
        self._worker.done.connect(self._on_install_done)
        self._worker.start()

    def _on_install_done(self, success):
        pkg_name = self.pkg_input.text().strip()
        if success:
            self.append_log(f"\n Successfully finished installation of {pkg_name}")
        else:
            self.append_log(f"\n Failed to install {pkg_name}")
        self.install_btn.setEnabled(True)
