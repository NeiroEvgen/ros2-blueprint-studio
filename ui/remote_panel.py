"""
Remote tab: remote target profile, connection, Sync, Build, Cancel.

All the logic is in core/remote/*; here only the form and running operations in
background threads (SSH blocks, the UI must not freeze).
"""
from PySide6 import QtWidgets, QtCore

from core.i18n import t
from ui import icons

from core.remote.profile import RemoteProfile, ProfileStore
from core.remote.session import RemoteSession
from core.remote.terminal import TerminalError, open_terminal, validate_target
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

    # ---------- form ----------

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)

        form = QtWidgets.QFormLayout()
        self.ed_host = QtWidgets.QLineEdit(); self.ed_host.setPlaceholderText("192.168.1.10")
        self.sp_port = QtWidgets.QSpinBox(); self.sp_port.setRange(1, 65535); self.sp_port.setValue(22)
        self.ed_user = QtWidgets.QLineEdit(); self.ed_user.setPlaceholderText("robot")
        self.ed_key = QtWidgets.QLineEdit(); self.ed_key.setPlaceholderText(t("rp.hint_key"))
        btn_key = QtWidgets.QPushButton("…"); btn_key.setFixedWidth(30)
        btn_key.clicked.connect(self._pick_key)
        key_row = QtWidgets.QHBoxLayout(); key_row.addWidget(self.ed_key); key_row.addWidget(btn_key)
        self.ed_pass = QtWidgets.QLineEdit(); self.ed_pass.setEchoMode(QtWidgets.QLineEdit.Password)
        self.ed_pass.setPlaceholderText(t("rp.hint_pass"))
        self.ed_ws = QtWidgets.QLineEdit(); self.ed_ws.setPlaceholderText("/home/robot/blueprint_ws")
        self.ed_container = QtWidgets.QLineEdit(); self.ed_container.setPlaceholderText(t("rp.hint_container"))
        self.ed_cws = QtWidgets.QLineEdit(); self.ed_cws.setPlaceholderText(t("rp.hint_cws"))
        self.ed_ros = QtWidgets.QLineEdit("/opt/ros/humble/setup.bash")

        form.addRow(t("rp.host"), self.ed_host)
        form.addRow(t("rp.port"), self.sp_port)
        form.addRow(t("rp.user"), self.ed_user)
        form.addRow(t("rp.key"), key_row)
        form.addRow(t("rp.password"), self.ed_pass)
        form.addRow(t("rp.ws_host"), self.ed_ws)
        form.addRow(t("rp.container"), self.ed_container)
        form.addRow(t("rp.ws_container"), self.ed_cws)
        form.addRow(t("rp.ros_setup"), self.ed_ros)
        outer.addLayout(form)
        self._form_fields = [self.ed_host, self.ed_user, self.ed_key, self.ed_container]

        row = QtWidgets.QHBoxLayout()
        self.btn_save = self._btn(t("rp.save_profile"), row, self.on_save, "save")
        self.btn_connect = self._btn(t("rp.connect"), row, self.on_connect_toggle, "plug")
        self.btn_sync = self._btn(t("rp.sync"), row, self.on_sync, "sync")
        self.cb_dry = QtWidgets.QCheckBox(t("rp.dry_run")); row.addWidget(self.cb_dry)
        self.btn_build = self._btn(t("rp.build"), row, self.on_build, "build")
        self.btn_cancel = self._btn(t("rp.cancel"), row, self.on_cancel, "stop")
        self.btn_terminal = self._btn(t("rp.terminal"), row, self.on_terminal, "terminal")
        self.btn_terminal.setToolTip(t("rp.terminal_tip"))
        row.addStretch()
        outer.addLayout(row)

        for w in self._form_fields:
            w.textChanged.connect(lambda *_: self._update_buttons())
        self.sp_port.valueChanged.connect(lambda *_: self._update_buttons())

        self.lbl_state = QtWidgets.QLabel(t("rp.not_connected"))
        self.lbl_state.setStyleSheet("color: #aaa; padding: 4px;")
        outer.addWidget(self.lbl_state)
        outer.addStretch()

    def _btn(self, text, layout, slot, icon_name=None):
        b = QtWidgets.QPushButton(text)
        if icon_name:
            b.setIcon(icons.icon(icon_name))
            b.setIconSize(QtCore.QSize(16, 16))
        b.setStyleSheet(_BTN)
        b.clicked.connect(slot)
        layout.addWidget(b)
        return b

    def _pick_key(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, t("rp.key_dialog"))
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

    # ---------- state ----------

    @property
    def connected(self):
        return bool(self.session and self.session.connected)

    def _update_buttons(self):
        busy = self._tasks.busy
        self.btn_save.setEnabled(bool(self.project_path) and not busy)
        self.btn_connect.setEnabled(bool(self.project_path) and not busy)
        self.btn_connect.setText(t("rp.disconnect") if self.connected else t("rp.connect"))
        self.btn_connect.setIcon(icons.icon("unplug" if self.connected else "plug"))
        self.btn_sync.setEnabled(self.connected and not busy)
        self.btn_build.setEnabled(self.connected and not busy)
        self.btn_cancel.setEnabled(self._building)
        # the console needs neither a Studio connection nor a project: host and user are enough
        self.btn_terminal.setEnabled(not validate_target(self.profile_from_form()))

    def _state(self, text):
        self.lbl_state.setText(text)

    def _fail(self, msg):
        self.log.emit(f"{msg}")
        self._state(msg)
        self._building = False
        if self.session and not self.session.connected:
            self._disconnect()           # the link is dead: Studio must not consider the target connected
            self._state(msg)
        self._update_buttons()

    # ---------- actions ----------

    def on_save(self):
        try:
            ProfileStore(self.project_path).save(self.profile_from_form())
            self.log.emit(t("rp.profile_saved"))
        except ValueError as e:
            self._fail(t("rp.profile_not_saved", error=e))

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
        self._state(t("rp.connecting", target=prof.display()))

        def work(emit):
            session.connect(password=password)
            return session

        def done(s):
            self.session = s
            self.ed_pass.clear()
            where = s.profile.display() + (f" / {s.profile.container}" if s.profile.container else "")
            self._state(t("rp.connected", where=where))
            self.log.emit(t("rp.connected_log", where=where))
            self.connection_changed.emit(True, s.profile.display(), s.profile.container)
            self._update_buttons()

        def fail(msg):
            if "paramiko" in msg and "No module named" in msg:
                msg = t("rp.no_paramiko")
            if "HostKeyChanged" in msg:
                QtWidgets.QMessageBox.critical(self, t("rp.hostkey_title"), msg)
            self._fail(t("rp.connect_failed", error=msg))

        self._tasks.start(work, done, fail, parent=self)
        self._update_buttons()

    def _disconnect(self):
        if self.session:
            self.session.disconnect()
        self.session = None
        self._state(t("rp.not_connected"))
        self.log.emit(t("rp.disconnected_log"))
        self.connection_changed.emit(False, "", "")
        self._update_buttons()

    def on_sync(self):
        dry = self.cb_dry.isChecked()
        s = self.session
        self._state(t("rp.sync_running"))

        def done(report):
            self._state(report.summary())
            if report.conflicts:
                QtWidgets.QMessageBox.warning(
                    self, t("rp.conflicts_title"),
                    t("rp.conflicts_text",
                      files="\n".join(f"{r}: {w}" for r, w in report.conflicts)))
            self._update_buttons()

        self._tasks.start(lambda emit: s.sync(on_line=emit, dry_run=dry),
                          done, self._fail, on_line=self.log.emit, parent=self)
        self._update_buttons()

    def on_build(self):
        s = self.session
        self._building = True
        self._state(t("rp.building"))

        def done(rc):
            self._building = False
            self._state(t("rp.build_ok") if rc == 0 else t("rp.build_failed", rc=rc))
            self.log.emit(t("rp.build_finished", rc=rc))
            self._update_buttons()

        self._tasks.start(lambda emit: s.build(emit), done, self._fail,
                          on_line=self.log.emit, parent=self)
        self._update_buttons()

    def on_terminal(self):
        prof = self.profile_from_form()
        try:
            shown = open_terminal(prof)
        except TerminalError as e:
            self._fail(t("rp.terminal_failed", error=e))
            return
        where = prof.display() + (f" / {prof.container}" if prof.container else "")
        self.log.emit(t("rp.terminal_opened", where=where, shown=shown))
        self.log.emit(t("rp.terminal_password"))

    def shutdown(self, timeout_ms=15000):
        """Window closing: cancel the build on the target, wait for the threads, disconnect. True if everything stopped."""
        if self._building and self.session:
            try:
                self.session.cancel()
            except Exception:
                pass
        self._tasks.wait_all(timeout_ms)
        stopped = not self._tasks.busy
        if stopped and self.session:
            try:
                self.session.disconnect()
            except Exception:
                pass
            self.session = None
        return stopped

    def on_cancel(self):
        s = self.session
        self.log.emit(t("rp.cancelling"))
        self._tasks.start(lambda emit: s.cancel(),
                          lambda ok: self.log.emit(t("rp.stopped") if ok else t("rp.nothing_to_stop")),
                          self._fail, parent=self)
