# ui/container_settings.py
# v0.6.0 — Вкладка настроек деплой-контейнеров.
# Слева список групп проекта, справа форма: образ, таргет, restart,
# ресурсы, env-таблица, DDS (вендор/QoS XML), netem-симуляция помех.
# Пресеты: встроенные + глобальные пользователя (Save as preset).

from PySide6 import QtWidgets, QtCore

from core.container_config import (ContainerConfigStore, GlobalPresetStore,
                                   make_config, safe_group_name, netem_command)


class ContainerSettingsPanel(QtWidgets.QWidget):
    config_saved = QtCore.Signal(str)   # имя сохранённой группы

    def __init__(self, get_docker_manager=None, parent=None):
        super().__init__(parent)
        self.store = None                    # ContainerConfigStore (после set_project)
        self.presets = GlobalPresetStore()
        self.get_docker_manager = get_docker_manager or (lambda: None)
        self._current_name = None
        self._build_ui()
        self._set_enabled(False)

    # ------------------------------------------------------------------ #
    def set_project(self, project_path):
        self.store = ContainerConfigStore(project_path)
        self._set_enabled(True)
        self.refresh_groups()

    def refresh_groups(self, node_counts=None):
        """node_counts: {group: int} — счётчик нод (раздел 2.4 roadmap)."""
        if not self.store:
            return
        self.group_list.blockSignals(True)
        self.group_list.clear()
        names = self.store.list_names()
        counts = node_counts or {}
        for n in names:
            label = f"{n}  ({counts[n]} nodes)" if n in counts else n
            item = QtWidgets.QListWidgetItem(label)
            item.setData(QtCore.Qt.UserRole, n)
            self.group_list.addItem(item)
        self.group_list.blockSignals(False)
        # мягкое предупреждение про оверхед
        self.overhead_hint.setVisible(len(names) > 5)
        if names:
            self.group_list.setCurrentRow(0)
            self._load_into_form(names[0])

    def group_names(self):
        return self.store.list_names() if self.store else ["main"]

    # ------------------------------------------------------------------ #
    def _build_ui(self):
        root = QtWidgets.QHBoxLayout(self)

        # === ЛЕВО: группы ===
        left = QtWidgets.QVBoxLayout()
        left.addWidget(QtWidgets.QLabel("Container groups"))
        self.group_list = QtWidgets.QListWidget()
        self.group_list.currentItemChanged.connect(self._on_group_selected)
        left.addWidget(self.group_list)

        btns = QtWidgets.QHBoxLayout()
        self.btn_add = QtWidgets.QPushButton("+ New")
        self.btn_del = QtWidgets.QPushButton("Delete")
        self.btn_add.clicked.connect(self._on_add_group)
        self.btn_del.clicked.connect(self._on_del_group)
        btns.addWidget(self.btn_add); btns.addWidget(self.btn_del)
        left.addLayout(btns)

        self.overhead_hint = QtWidgets.QLabel(
            "⚠ Много контейнеров = выше накладные расходы\n(каждый тащит свой ROS-рантайм и DDS discovery)")
        self.overhead_hint.setStyleSheet("color: #e0a030;")
        self.overhead_hint.setVisible(False)
        left.addWidget(self.overhead_hint)
        root.addLayout(left, 1)

        # === ПРАВО: форма ===
        right = QtWidgets.QVBoxLayout()

        preset_row = QtWidgets.QHBoxLayout()
        preset_row.addWidget(QtWidgets.QLabel("Preset:"))
        self.preset_combo = QtWidgets.QComboBox()
        self.btn_load_preset = QtWidgets.QPushButton("Load")
        self.btn_save_preset = QtWidgets.QPushButton("Save as preset…")
        self.btn_load_preset.clicked.connect(self._on_load_preset)
        self.btn_save_preset.clicked.connect(self._on_save_preset)
        preset_row.addWidget(self.preset_combo, 1)
        preset_row.addWidget(self.btn_load_preset)
        preset_row.addWidget(self.btn_save_preset)
        right.addLayout(preset_row)
        self._refresh_presets()

        form_scroll = QtWidgets.QScrollArea(); form_scroll.setWidgetResizable(True)
        form_host = QtWidgets.QWidget(); form = QtWidgets.QFormLayout(form_host)

        self.ed_image = QtWidgets.QLineEdit()
        form.addRow("Image", self.ed_image)

        # Таргет
        trow = QtWidgets.QHBoxLayout()
        self.cb_target = QtWidgets.QComboBox(); self.cb_target.addItems(["local", "ssh"])
        self.ed_host = QtWidgets.QLineEdit(); self.ed_host.setPlaceholderText("ssh://user@192.168.1.50")
        trow.addWidget(self.cb_target); trow.addWidget(self.ed_host, 1)
        w = QtWidgets.QWidget(); w.setLayout(trow)
        form.addRow("Target", w)

        self.cb_restart = QtWidgets.QComboBox()
        self.cb_restart.addItems(["no", "always", "on-failure", "unless-stopped"])
        form.addRow("Restart policy", self.cb_restart)

        self.cb_network = QtWidgets.QComboBox(); self.cb_network.addItems(["host", "bridge"])
        form.addRow("Network mode", self.cb_network)

        self.chk_priv = QtWidgets.QCheckBox("privileged")
        form.addRow("", self.chk_priv)

        rrow = QtWidgets.QHBoxLayout()
        self.ed_cpus = QtWidgets.QLineEdit(); self.ed_cpus.setPlaceholderText("напр. 2")
        self.ed_mem = QtWidgets.QLineEdit(); self.ed_mem.setPlaceholderText("напр. 1g")
        rrow.addWidget(QtWidgets.QLabel("CPUs")); rrow.addWidget(self.ed_cpus)
        rrow.addWidget(QtWidgets.QLabel("Memory")); rrow.addWidget(self.ed_mem)
        w = QtWidgets.QWidget(); w.setLayout(rrow)
        form.addRow("Resources", w)

        # ENV таблица
        self.env_table = QtWidgets.QTableWidget(0, 2)
        self.env_table.setHorizontalHeaderLabels(["KEY", "VALUE"])
        self.env_table.horizontalHeader().setStretchLastSection(True)
        self.env_table.setMinimumHeight(120)
        env_btns = QtWidgets.QHBoxLayout()
        b_add = QtWidgets.QPushButton("+ env"); b_del = QtWidgets.QPushButton("- env")
        b_add.clicked.connect(lambda: self.env_table.insertRow(self.env_table.rowCount()))
        b_del.clicked.connect(lambda: self.env_table.removeRow(self.env_table.currentRow()))
        env_btns.addWidget(b_add); env_btns.addWidget(b_del); env_btns.addStretch()
        env_host = QtWidgets.QVBoxLayout(); env_host.addWidget(self.env_table); env_host.addLayout(env_btns)
        w = QtWidgets.QWidget(); w.setLayout(env_host)
        form.addRow("ROS / env vars", w)

        # DDS
        dds_box = QtWidgets.QGroupBox("DDS")
        dds_l = QtWidgets.QFormLayout(dds_box)
        self.cb_dds = QtWidgets.QComboBox()
        self.cb_dds.addItems(["default", "fastrtps", "cyclonedds"])
        dds_l.addRow("Vendor (RMW)", self.cb_dds)
        qrow = QtWidgets.QHBoxLayout()
        self.ed_qos = QtWidgets.QLineEdit(); self.ed_qos.setPlaceholderText("путь к QoS XML профилю (опц.)")
        b_qos = QtWidgets.QPushButton("…")
        b_qos.clicked.connect(self._pick_qos)
        qrow.addWidget(self.ed_qos, 1); qrow.addWidget(b_qos)
        w = QtWidgets.QWidget(); w.setLayout(qrow)
        dds_l.addRow("QoS profile", w)
        form.addRow(dds_box)

        # NETEM
        net_box = QtWidgets.QGroupBox("Network fault simulation (tc netem)")
        net_l = QtWidgets.QFormLayout(net_box)
        self.chk_netem = QtWidgets.QCheckBox("enabled")
        net_l.addRow("", self.chk_netem)
        self.sp_loss = QtWidgets.QDoubleSpinBox(); self.sp_loss.setRange(0, 100); self.sp_loss.setSuffix(" %")
        self.sp_delay = QtWidgets.QSpinBox(); self.sp_delay.setRange(0, 10000); self.sp_delay.setSuffix(" ms")
        self.sp_jitter = QtWidgets.QSpinBox(); self.sp_jitter.setRange(0, 5000); self.sp_jitter.setSuffix(" ms")
        net_l.addRow("Packet loss", self.sp_loss)
        net_l.addRow("Delay", self.sp_delay)
        net_l.addRow("Jitter", self.sp_jitter)
        self.btn_apply_netem = QtWidgets.QPushButton("Apply to running session")
        self.btn_apply_netem.clicked.connect(self._on_apply_netem)
        net_l.addRow("", self.btn_apply_netem)
        form.addRow(net_box)

        form_scroll.setWidget(form_host)
        right.addWidget(form_scroll, 1)

        self.btn_save = QtWidgets.QPushButton("💾 Save group config")
        self.btn_save.clicked.connect(self._on_save)
        right.addWidget(self.btn_save)

        self.status = QtWidgets.QLabel("")
        self.status.setStyleSheet("color: #8c8;")
        right.addWidget(self.status)

        root.addLayout(right, 2)

    # ------------------------------------------------------------------ #
    def _set_enabled(self, on):
        for wdg in (self.group_list, self.btn_add, self.btn_del, self.btn_save,
                    self.btn_load_preset, self.btn_save_preset):
            wdg.setEnabled(on)
        if not on:
            self.status.setText("Open a project to edit container configs.")

    def _refresh_presets(self):
        self.preset_combo.clear()
        self.preset_combo.addItems(self.presets.list_names())

    def _pick_qos(self):
        p, _ = QtWidgets.QFileDialog.getOpenFileName(self, "QoS XML", "", "XML (*.xml)")
        if p:
            self.ed_qos.setText(p)

    # ------------------------------------------------------------------ #
    def _on_group_selected(self, cur, _prev):
        if cur:
            self._load_into_form(cur.data(QtCore.Qt.UserRole))

    def _on_add_group(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "New container group", "Group name:")
        if ok and name and self.store:
            cfg = make_config(name)
            self.store.save(cfg)
            self.refresh_groups()

    def _on_del_group(self):
        item = self.group_list.currentItem()
        if not item or not self.store:
            return
        name = item.data(QtCore.Qt.UserRole)
        if safe_group_name(name) == "main":
            self.status.setText("Группу 'main' удалить нельзя — это дефолт.")
            return
        self.store.delete(name)
        self.refresh_groups()

    def _on_load_preset(self):
        cfg = self.presets.get(self.preset_combo.currentText())
        cfg["name"] = self._current_name or "main"
        self._fill_form(cfg)
        self.status.setText(f"Preset '{self.preset_combo.currentText()}' загружен (не забудь Save).")

    def _on_save_preset(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "Save as preset", "Preset name:")
        if ok and name:
            if self.presets.save(name, self._form_to_cfg()):
                self._refresh_presets()
                self.status.setText(f"Preset '{name}' сохранён глобально (~/.ros_studio/presets).")

    def _on_save(self):
        if not self.store:
            return
        cfg = self._form_to_cfg()
        self.store.save(cfg)
        self.status.setText(f"Config '{cfg['name']}' saved.")
        self.config_saved.emit(cfg["name"])

    def _on_apply_netem(self):
        dm = self.get_docker_manager()
        cfg = self._form_to_cfg()
        cmd = netem_command(cfg)
        if not dm or not getattr(dm, "container", None):
            self.status.setText("Нет активной сессии контейнера.")
            return
        if not cmd:
            self.status.setText("netem выключен или нет параметров.")
            return
        try:
            dm.container.exec_run(f"bash -c '{cmd}'", privileged=True)
            self.status.setText(f"netem применён: {cmd}")
        except Exception as e:
            self.status.setText(f"netem error: {e}")

    # ------------------------------------------------------------------ #
    def _load_into_form(self, name):
        self._current_name = safe_group_name(name)
        self._fill_form(self.store.get(self._current_name))

    def _fill_form(self, cfg):
        self.ed_image.setText(cfg.get("image", ""))
        t = cfg.get("target") or {}
        self.cb_target.setCurrentText(t.get("type", "local"))
        self.ed_host.setText(t.get("host", ""))
        self.cb_restart.setCurrentText(cfg.get("restart", "no"))
        self.cb_network.setCurrentText(cfg.get("network_mode", "host"))
        self.chk_priv.setChecked(bool(cfg.get("privileged")))
        res = cfg.get("resources") or {}
        self.ed_cpus.setText(str(res.get("cpus", "")))
        self.ed_mem.setText(str(res.get("memory", "")))

        env = cfg.get("env") or {}
        self.env_table.setRowCount(0)
        for k, v in env.items():
            r = self.env_table.rowCount()
            self.env_table.insertRow(r)
            self.env_table.setItem(r, 0, QtWidgets.QTableWidgetItem(str(k)))
            self.env_table.setItem(r, 1, QtWidgets.QTableWidgetItem(str(v)))

        dds = cfg.get("dds") or {}
        self.cb_dds.setCurrentText(dds.get("vendor", "default"))
        self.ed_qos.setText(dds.get("qos_xml", ""))

        n = cfg.get("netem") or {}
        self.chk_netem.setChecked(bool(n.get("enabled")))
        self.sp_loss.setValue(float(n.get("loss_percent") or 0))
        self.sp_delay.setValue(int(n.get("delay_ms") or 0))
        self.sp_jitter.setValue(int(n.get("jitter_ms") or 0))

    def _form_to_cfg(self):
        env = {}
        for r in range(self.env_table.rowCount()):
            k = self.env_table.item(r, 0)
            v = self.env_table.item(r, 1)
            if k and k.text().strip():
                env[k.text().strip()] = v.text().strip() if v else ""
        return {
            "name": self._current_name or "main",
            "image": self.ed_image.text().strip() or "osrf/ros:humble-desktop",
            "env": env,
            "dds": {"vendor": self.cb_dds.currentText(), "qos_xml": self.ed_qos.text().strip()},
            "netem": {
                "enabled": self.chk_netem.isChecked(),
                "loss_percent": self.sp_loss.value(),
                "delay_ms": self.sp_delay.value(),
                "jitter_ms": self.sp_jitter.value(),
                "interface": "eth0",
            },
            "restart": self.cb_restart.currentText(),
            "privileged": self.chk_priv.isChecked(),
            "network_mode": self.cb_network.currentText(),
            "resources": {"cpus": self.ed_cpus.text().strip(), "memory": self.ed_mem.text().strip()},
            "target": {"type": self.cb_target.currentText(), "host": self.ed_host.text().strip()},
        }
