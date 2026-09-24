"""
Вкладка Remote: профиль удалённой цели, подключение, Sync, Build, Cancel.

Вся логика — в core/remote/*; здесь только форма и запуск операций в
фоновых потоках (SSH блокирует, UI не должен замирать).
"""
from PySide6 import QtWidgets, QtCore

from core.remote.profile import RemoteProfile, ProfileStore, HostKeyChanged
from core.remote.session import RemoteSession
from ui.qt_worker import TaskHost

_BTN = ("QPushButton { background-color: #555; color: white; border: none;"
        " padding: 5px 12px; border-radius: 3px; }"
        "QPushButton:hover { background-color: #666; }"
        "QPushButton:disabled { background-color: #2a2a2a; color: #666; }")


class RemotePanel(QtWidgets.QWidget):
    log = QtCore.Signal(str)
    # connected, "user@host", container
    connection_changed = QtCore.Signal(bool, str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.project_path = None
        self.session = None
        self._tasks = TaskHost()
        self._building = False
        self._build_ui()
        self._update_buttons()

    # ---------- форма ----------

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)

        form = QtWidgets.QFormLayout()
        self.ed_host = QtWidgets.QLineEdit(); self.ed_host.setPlaceholderText("192.168.123.164")
        self.sp_port = QtWidgets.QSpinBox(); self.sp_port.setRange(1, 65535); self.sp_port.setValue(22)
        self.ed_user = QtWidgets.QLineEdit(); self.ed_user.setPlaceholderText("unitree")
        self.ed_key = QtWidgets.QLineEdit(); self.ed_key.setPlaceholderText("пусто = ssh-agent / ~/.ssh")
        btn_key = QtWidgets.QPushButton("…"); btn_key.setFixedWidth(30)
        btn_key.clicked.connect(self._pick_key)
        key_row = QtWidgets.QHBoxLayout(); key_row.addWidget(self.ed_key); key_row.addWidget(btn_key)
        self.ed_pass = QtWidgets.QLineEdit(); self.ed_pass.setEchoMode(QtWidgets.QLineEdit.Password)
        self.ed_pass.setPlaceholderText("не сохраняется")
        self.ed_ws = QtWidgets.QLineEdit(); self.ed_ws.setPlaceholderText("/home/unitree/blueprint_ws")
        self.ed_container = QtWidgets.QLineEdit(); self.ed_container.setPlaceholderText("пусто = собирать на хосте")
        self.ed_cws = QtWidgets.QLineEdit(); self.ed_cws.setPlaceholderText("тот же каталог внутри контейнера")
        self.ed_ros = QtWidgets.QLineEdit("/opt/ros/humble/setup.bash")

        form.addRow("Хост", self.ed_host)
        form.addRow("Порт", self.sp_port)
        form.addRow("Пользователь", self.ed_user)
        form.addRow("Ключ", key_row)
        form.addRow("Пароль", self.ed_pass)
        form.addRow("Workspace на хосте", self.ed_ws)
        form.addRow("Контейнер", self.ed_container)
        form.addRow("Workspace в контейнере", self.ed_cws)
        form.addRow("ROS setup", self.ed_ros)
        outer.addLayout(form)

        row = QtWidgets.QHBoxLayout()
        self.btn_save = self._btn("💾 Сохранить профиль", row, self.on_save)
        self.btn_connect = self._btn("🔌 Подключить", row, self.on_connect_toggle)
        self.btn_sync = self._btn("⇅ Sync", row, self.on_sync)
        self.cb_dry = QtWidgets.QCheckBox("dry-run"); row.addWidget(self.cb_dry)
        self.btn_build = self._btn("🔨 Build", row, self.on_build)
        self.btn_cancel = self._btn("⏹ Cancel", row, self.on_cancel)
        row.addStretch()
        outer.addLayout(row)

        self.lbl_state = QtWidgets.QLabel("Не подключено")
        self.lbl_state.setStyleSheet("color: #aaa; padding: 4px;")
        outer.addWidget(self.lbl_state)
        outer.addStretch()

    def _btn(self, text, layout, slot):
        b = QtWidgets.QPushButton(text)
        b.setStyleSheet(_BTN)
        b.clicked.connect(slot)
        layout.addWidget(b)
        return b

    def _pick_key(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "SSH-ключ")
        if path:
            self.ed_key.setText(path)

    def profile_from_form(self):
        return RemoteProfile(
            host=self.ed_host.text().strip(), port=self.sp_port.value(),
            user=self.ed_user.text().strip(), key_path=self.ed_key.text().strip(),
            remote_workspace=self.ed_ws.text().strip(),
            container=self.ed_container.text().strip(),
            container_workspace=self.ed_cws.text().strip(),
            ros_setup=self.ed_ros.text().strip() or "/opt/ros/humble/setup.bash")

    def _fill_form(self, p):
        self.ed_host.setText(p.host); self.sp_port.setValue(int(p.port or 22))
        self.ed_user.setText(p.user); self.ed_key.setText(p.key_path)
        self.ed_ws.setText(p.remote_workspace); self.ed_container.setText(p.container)
        self.ed_cws.setText(p.container_workspace); self.ed_ros.setText(p.ros_setup)

    def set_project(self, path):
        if self.session:
            self._disconnect()
        self.project_path = path
        if path:
            self._fill_form(ProfileStore(path).load())
        self._update_buttons()

    # ---------- состояние ----------

    @property
    def connected(self):
        return bool(self.session and self.session.connected)

    def _update_buttons(self):
        busy = self._tasks.busy
        self.btn_save.setEnabled(bool(self.project_path) and not busy)
        self.btn_connect.setEnabled(bool(self.project_path) and not busy)
        self.btn_connect.setText("✂ Отключить" if self.connected else "🔌 Подключить")
        self.btn_sync.setEnabled(self.connected and not busy)
        self.btn_build.setEnabled(self.connected and not busy)
        self.btn_cancel.setEnabled(self._building)

    def _state(self, text):
        self.lbl_state.setText(text)

    def _fail(self, msg):
        self.log.emit(f"❌ {msg}")
        self._state(msg)
        self._building = False
        self._update_buttons()

    # ---------- действия ----------

    def on_save(self):
        try:
            ProfileStore(self.project_path).save(self.profile_from_form())
            self.log.emit("Профиль удалённой цели сохранён (.blueprint/remote.yaml, без пароля)")
        except ValueError as e:
            self._fail(f"Профиль не сохранён: {e}")

    def on_connect_toggle(self):
        if self.connected:
            self._disconnect()
            return
        prof = self.profile_from_form()
        errors = prof.validate()
        if errors:
            self._fail("; ".join(errors))
            return
        password = self.ed_pass.text() or None
        session = RemoteSession(self.project_path, prof)
        self._state(f"Подключение к {prof.display()}…")

        def work(emit):
            session.connect(password=password)
            return session

        def done(s):
            self.session = s
            self.ed_pass.clear()
            where = s.profile.display() + (f" / {s.profile.container}" if s.profile.container else "")
            self._state(f"Подключено: {where}")
            self.log.emit(f"🔌 Подключено к {where}")
            self.connection_changed.emit(True, s.profile.display(), s.profile.container)
            self._update_buttons()

        def fail(msg):
            if "HostKeyChanged" in msg:
                QtWidgets.QMessageBox.critical(self, "Ключ хоста изменился", msg)
            self._fail(f"Подключение не удалось: {msg}")

        self._tasks.start(work, done, fail, parent=self)
        self._update_buttons()

    def _disconnect(self):
        if self.session:
            self.session.disconnect()
        self.session = None
        self._state("Не подключено")
        self.log.emit("✂ Отключено от удалённой цели")
        self.connection_changed.emit(False, "", "")
        self._update_buttons()

    def on_sync(self):
        dry = self.cb_dry.isChecked()
        s = self.session
        self._state("Sync…")

        def done(report):
            self._state(report.summary())
            if report.conflicts:
                QtWidgets.QMessageBox.warning(
                    self, "Конфликты синхронизации",
                    "Эти файлы изменены и локально, и на удалённой стороне — "
                    "НЕ перезаписаны:\n\n" + "\n".join(f"{r}: {w}" for r, w in report.conflicts))
            self._update_buttons()

        self._tasks.start(lambda emit: s.sync(on_line=emit, dry_run=dry),
                          done, self._fail, on_line=self.log.emit, parent=self)
        self._update_buttons()

    def on_build(self):
        s = self.session
        self._building = True
        self._state("Сборка на удалённой цели…")

        def done(rc):
            self._building = False
            self._state("✅ Сборка прошла" if rc == 0 else f"❌ Сборка упала (rc={rc})")
            self.log.emit(f"--- REMOTE BUILD FINISHED rc={rc} ---")
            self._update_buttons()

        self._tasks.start(lambda emit: s.build(emit), done, self._fail,
                          on_line=self.log.emit, parent=self)
        self._update_buttons()

    def on_cancel(self):
        s = self.session
        self.log.emit("⏹ Отмена: останавливаю процесс на удалённой машине…")
        self._tasks.start(lambda emit: s.cancel(),
                          lambda ok: self.log.emit("⏹ Остановлено" if ok else "Нечего останавливать"),
                          self._fail, parent=self)
