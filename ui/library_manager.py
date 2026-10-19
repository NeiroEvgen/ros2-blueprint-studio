from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, 
                             QPushButton, QLabel, QComboBox, QTextEdit)
from core.i18n import t
from ui.fonts import MONO_CSS



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
            self.log_line.emit(t("lib.error", error=e))
            self.done.emit(False)

class _RebuildWorker(QThread):
    """Rebuilds the project image (after editing the Dockerfile) in the background."""
    log = Signal(str)
    done = Signal(bool)

    def __init__(self, docker_manager, project_path):
        super().__init__()
        self.docker_manager = docker_manager
        self.project_path = project_path

    def run(self):
        try:
            self.docker_manager.rebuild_project_image(
                self.project_path, status_callback=self.log.emit)
            self.done.emit(True)
        except Exception as e:
            self.log.emit(t("lib.error", error=e))
            self.done.emit(False)


class LibraryManager(QWidget):
    def __init__(self, docker_manager):
        super().__init__()
        self.docker_manager = docker_manager
        self.project_path = None      # set by the main window when a project opens
        self.remote_connected = False
        self._worker = None
        self.init_ui()

    def set_project(self, path):
        self.project_path = path

    def set_remote_connected(self, connected):
        """When connected to a remote target, installation still happens LOCALLY."""
        self.remote_connected = bool(connected)

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # title
        header = QLabel(t("lib.header"))
        header.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(header)
        
        # input field and type choice
        input_layout = QHBoxLayout()
        
        self.pkg_input = QLineEdit()
        self.pkg_input.setPlaceholderText(t("lib.pkg_hint"))
        
        self.type_combo = QComboBox()
        self.type_combo.addItems([t("lib.type_apt"), t("lib.type_pip")])
        
        self.install_btn = QPushButton(t("lib.install"))
        self.install_btn.clicked.connect(self.start_install)
        
        input_layout.addWidget(self.pkg_input)
        input_layout.addWidget(self.type_combo)
        input_layout.addWidget(self.install_btn)
        layout.addLayout(input_layout)
        
        # install log
        layout.addWidget(QLabel(t("lib.output")))
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: " + MONO_CSS + ";")
        layout.addWidget(self.log_output)
        
        # quick presets
        presets_layout = QHBoxLayout()
        presets_layout.addWidget(QLabel(t("lib.common")))
        
        common_libs = [
            ("OpenCV (C++)", "libopencv-dev", "apt"),
            ("OpenCV (Py)", "opencv-python", "pip"),
            ("NumPy", "numpy", "pip"),
            ("Pillow", "Pillow", "pip")
        ]
        
        for label, name, p_type in common_libs:
            btn = QPushButton(label)
            btn.clicked.connect(lambda checked, n=name, tp=p_type: self.quick_install(n, tp))
            presets_layout.addWidget(btn)
        
        presets_layout.addStretch()
        layout.addLayout(presets_layout)

    def append_log(self, text):
        self.log_output.append(text)
        # scroll down
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

        if not self.project_path:
            self.append_log(t("lib.open_project_first"))
            return
        if self._worker is not None and self._worker.isRunning():
            self.append_log(t("lib.build_running"))
            return
        if self.remote_connected:
            self.append_log(t("lib.remote_warning"))

        from core.dockerfile_manager import DockerfileManager
        try:
            dm = DockerfileManager(self.project_path)
            dm.add_manual_library(pkg_type, pkg_name)
        except Exception as e:
            self.append_log(t("lib.dockerfile_failed", error=e))
            return
        self.append_log(t("lib.added", name=pkg_name, type=pkg_type))

        self.install_btn.setEnabled(False)
        self._worker = _RebuildWorker(self.docker_manager, self.project_path)
        self._worker.log.connect(self.append_log)
        self._worker.done.connect(self._on_install_done)
        self._worker.start()

    def _on_install_done(self, success):
        pkg_name = self.pkg_input.text().strip()
        if success:
            self.append_log(t("lib.install_ok", name=pkg_name))
        else:
            self.append_log(t("lib.install_failed", name=pkg_name))
        self.install_btn.setEnabled(True)
