"""
Parameters window: live ROS 2 parameters of a running node, tuned with sliders.

Talks to the node through the parameter bridge (core/param_client.py). All bridge calls go
through one background thread in order, so the UI never waits for the robot and a later
value can never be overtaken by an earlier one.

Simulation: values are sent while the slider moves (only the latest one, ~10 per second).
Real robot: a value is sent only when the slider is released, and big or unbounded changes
ask for confirmation first (core.param_client.check_change).
"""
import json
import queue
import threading

from PySide6 import QtCore, QtWidgets

from core import param_store
from core.i18n import t
from core.param_client import Coalescer, ParamBridgeError, check_change, match_node, snap

SEND_INTERVAL = 0.1          # s: at most this often per parameter while a slider moves
SLIDER_STEPS = 1000          # resolution of a slider over a continuous range


class _Serial(QtCore.QObject):
    """One worker thread; results come back to the Qt thread as signals."""
    done = QtCore.Signal(object, object)       # callback, result
    failed = QtCore.Signal(object, object)     # callback, exception

    def __init__(self):
        super().__init__()
        self._q = queue.Queue()
        self.closed = False                     # window gone: answers still arrive, but nobody shows them
        self.done.connect(lambda cb, res: cb and not self.closed and cb(res))
        self.failed.connect(lambda cb, exc: cb and not self.closed and cb(exc))
        self._thread = threading.Thread(target=self._loop, name="params-window", daemon=True)
        self._thread.start()

    def submit(self, fn, on_done=None, on_fail=None):
        self._q.put((fn, on_done, on_fail))

    def _loop(self):
        while True:
            item = self._q.get()
            if item is None:
                return
            fn, on_done, on_fail = item
            try:
                res = fn()
            except Exception as e:
                self.failed.emit(on_fail, e)
            else:
                self.done.emit(on_done, res)

    @property
    def idle(self):
        return self._q.empty()

    def stop(self):
        """Queued requests still go out (the last slider value reaches the node); callbacks are dropped."""
        self.closed = True
        self._q.put(None)


def _decimals(info):
    step = float((info.range or {}).get("step") or 0)
    if step > 0:
        text = f"{step:.10f}".rstrip("0")
        return max(1, min(6, len(text.split(".")[1]) if "." in text else 0))
    return 4


class ParamRow(QtWidgets.QWidget):
    """One parameter: editor for its type + status mark. Emits edited(value, final)."""
    edited = QtCore.Signal(object, bool)

    def __init__(self, info, real_robot, parent=None):
        super().__init__(parent)
        self.info = info
        self.original = info.value          # value when the window loaded the node
        self.confirmed = info.value         # last value the node accepted
        self.seq = 0                        # latest request for this row: older answers are ignored
        self.slider = None
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.editor = self._make_editor(real_robot)
        lay.addWidget(self.editor, 1)
        self.status = QtWidgets.QLabel()
        self.status.setFixedWidth(18)
        lay.addWidget(self.status)
        self.setEnabled(not info.read_only)
        if info.read_only:
            self.setToolTip(t("params.read_only"))

    # ---- editors ----
    def _make_editor(self, real_robot):
        info = self.info
        if info.type == "bool":
            w = QtWidgets.QCheckBox()
            w.setChecked(bool(info.value))
            w.toggled.connect(lambda v: self.edited.emit(bool(v), True))
            self.value_widget = w
            return w
        if info.numeric:
            return self._numeric_editor(real_robot)
        w = QtWidgets.QLineEdit(self._text(info.value))
        w.editingFinished.connect(self._text_finished)
        self.value_widget = w
        return w

    @staticmethod
    def _text(v):
        if isinstance(v, list):
            return json.dumps(v)
        return "" if v is None else str(v)

    def _text_finished(self):
        text = self.value_widget.text()
        if self.info.type.endswith("_array"):
            try:
                value = json.loads(text)
            except ValueError:
                self.set_status(False, t("params.bad_list"))
                return
            if not isinstance(value, list):
                self.set_status(False, t("params.bad_list"))
                return
        else:
            value = text
        if value != self.confirmed:
            self.edited.emit(value, True)

    def _numeric_editor(self, real_robot):
        info = self.info
        box = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        r = info.range
        if info.type == "int":
            spin = QtWidgets.QSpinBox()
            spin.setRange(int(r["min"]) if r else -2**31, int(r["max"]) if r else 2**31 - 1)
            spin.setSingleStep(max(1, int((r or {}).get("step") or 1)))
        else:
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(_decimals(info))
            spin.setRange(float(r["min"]) if r else -1e12, float(r["max"]) if r else 1e12)
            step = float((r or {}).get("step") or 0)
            spin.setSingleStep(step if step > 0 else ((float(r["max"]) - float(r["min"])) / 100 if r else 0.1))
        spin.setKeyboardTracking(False)        # valueChanged only on Enter/arrows, not per keystroke
        spin.setValue(info.value if info.value is not None else 0)
        spin.valueChanged.connect(self._spin_changed)
        self.value_widget = spin
        if r and float(r["max"]) > float(r["min"]):
            self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.slider.setRange(0, self._slider_steps())
            self.slider.setValue(self._to_slider(info.value))
            self.slider.valueChanged.connect(self._slider_changed)
            self.slider.sliderReleased.connect(self._slider_released)
            lay.addWidget(self.slider, 3)
        lay.addWidget(spin, 1)
        return box

    def _slider_steps(self):
        r = self.info.range
        step = float(r.get("step") or 0)
        span = float(r["max"]) - float(r["min"])
        if step > 0 and span / step <= 10000:
            return max(1, int(round(span / step)))
        if self.info.type == "int" and span <= 10000:
            return max(1, int(span))
        return SLIDER_STEPS

    def _to_slider(self, v):
        r = self.info.range
        lo, hi = float(r["min"]), float(r["max"])
        frac = (float(v) - lo) / (hi - lo) if hi > lo else 0
        return int(round(min(max(frac, 0), 1) * self.slider.maximum()))

    def _from_slider(self, i):
        r = self.info.range
        lo, hi = float(r["min"]), float(r["max"])
        return snap(self.info, lo + (hi - lo) * i / self.slider.maximum())

    def _slider_changed(self, i):
        value = self._from_slider(i)
        self._show(value, slider=False)
        # dragging: not final; a click on the track or keyboard: final
        self.edited.emit(value, not self.slider.isSliderDown())

    def _slider_released(self):
        self.edited.emit(self._from_slider(self.slider.value()), True)

    def _spin_changed(self, v):
        value = snap(self.info, v)
        if self.slider:
            self.slider.blockSignals(True)
            self.slider.setValue(self._to_slider(value))
            self.slider.blockSignals(False)
        self.edited.emit(value, True)

    # ---- state ----
    def _show(self, value, slider=True):
        w = self.value_widget
        w.blockSignals(True)
        if isinstance(w, QtWidgets.QCheckBox):
            w.setChecked(bool(value))
        elif isinstance(w, QtWidgets.QLineEdit):
            w.setText(self._text(value))
        else:
            w.setValue(value)
        w.blockSignals(False)
        if slider and self.slider:
            self.slider.blockSignals(True)
            self.slider.setValue(self._to_slider(value))
            self.slider.blockSignals(False)

    def displayed(self):
        w = self.value_widget
        if isinstance(w, QtWidgets.QCheckBox):
            return w.isChecked()
        if isinstance(w, QtWidgets.QLineEdit):
            return w.text()
        return snap(self.info, w.value())

    def revert_display(self):
        self._show(self.confirmed)

    def set_status(self, ok, tip=""):
        self.status.setText("✓" if ok else "✗")
        self.status.setStyleSheet("color: #4caf50;" if ok else "color: #e57373;")
        self.status.setToolTip(tip)

    @property
    def changed(self):
        return self.confirmed != self.original


class ParamsWindow(QtWidgets.QWidget):
    """
    get_client(): returns a started ParamClient (may block: called in the worker thread).
    initial_node: ROS name to preselect (a guess from the graph node; the user can pick another).
    """
    log = QtCore.Signal(str)

    def __init__(self, get_client, project_path, real_robot, where, initial_node=None, parent=None):
        super().__init__(parent)
        self.setWindowFlags(QtCore.Qt.Window)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.get_client = get_client
        self.project_path = project_path
        self.real_robot = real_robot
        self.initial_node = initial_node
        self.client = None
        self.node = None
        self.rows = {}
        self._worker = _Serial()
        self._coalescer = Coalescer(SEND_INTERVAL)
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(int(SEND_INTERVAL * 500))
        self._timer.timeout.connect(self._send_due)
        self._timer.start()
        self.setWindowTitle(t("params.title", node=initial_node or "…"))
        self.resize(520, 480)
        self._build(where)
        self._set_state(t("params.connecting"))
        self._worker.submit(self._connect_and_list, self._on_nodes, self._on_error)

    # ---------- layout ----------
    def _build(self, where):
        lay = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel(t("params.node")))
        self.cb_node = QtWidgets.QComboBox()
        self.cb_node.currentTextChanged.connect(self._on_node_picked)
        top.addWidget(self.cb_node, 1)
        self.btn_refresh = QtWidgets.QPushButton(t("params.refresh"))
        self.btn_refresh.clicked.connect(self.refresh)
        top.addWidget(self.btn_refresh)
        lay.addLayout(top)

        info = QtWidgets.QLabel(where + (" · " + t("params.real_robot_note") if self.real_robot else ""))
        info.setWordWrap(True)
        info.setStyleSheet("color: #e0a040;" if self.real_robot else "color: #aaa;")
        lay.addWidget(info)

        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self._form_host = QtWidgets.QWidget()
        self.form = QtWidgets.QFormLayout(self._form_host)
        self.scroll.setWidget(self._form_host)
        lay.addWidget(self.scroll, 1)

        bottom = QtWidgets.QHBoxLayout()
        self.lbl_state = QtWidgets.QLabel()
        self.lbl_state.setWordWrap(True)
        bottom.addWidget(self.lbl_state, 1)
        self.btn_revert = QtWidgets.QPushButton(t("params.revert"))
        self.btn_revert.clicked.connect(self.revert)
        bottom.addWidget(self.btn_revert)
        self.btn_save = QtWidgets.QPushButton(t("params.save"))
        self.btn_save.clicked.connect(self.save)
        bottom.addWidget(self.btn_save)
        lay.addLayout(bottom)

    def _set_state(self, text):
        self.lbl_state.setText(text)

    # ---------- loading ----------
    def _connect_and_list(self):
        self.client = self.get_client()
        return self.client.list_nodes()

    def _on_error(self, exc):
        msg = str(exc) if isinstance(exc, ParamBridgeError) else t("params.err_internal", detail=exc)
        self._set_state(msg)
        self.log.emit(msg)

    def _on_nodes(self, nodes):
        pick = (match_node(nodes, self.node or self.initial_node) or nodes[0]) if nodes else ""
        self.cb_node.blockSignals(True)         # one load below, not a second one from the signal
        self.cb_node.clear()
        self.cb_node.addItems(nodes)
        self.cb_node.setCurrentText(pick)
        self.cb_node.blockSignals(False)
        if not nodes:
            self._clear_rows()
            self._set_state(t("params.no_nodes"))
            return
        self._on_node_picked(pick)

    def _on_node_picked(self, node):
        if not node or self.client is None:
            return
        self.node = node
        self.setWindowTitle(t("params.title", node=node))
        self._set_state(t("params.loading"))
        self._worker.submit(lambda: self.client.list_params(node),
                            lambda params, n=node: self._on_params(n, params), self._on_error)

    def refresh(self):
        if self.client is None or not self.client.alive:     # bridge gone: Studio starts a new one
            self._worker.submit(self._connect_and_list, self._on_nodes, self._on_error)
        else:
            self._worker.submit(self.client.list_nodes, self._on_nodes, self._on_error)

    def _clear_rows(self):
        while self.form.rowCount():
            self.form.removeRow(0)
        self.rows = {}

    def _on_params(self, node, params):
        if node != self.node:
            return                              # the user picked another node meanwhile
        self._clear_rows()
        for info in params:
            row = ParamRow(info, self.real_robot)
            row.edited.connect(lambda v, final, r=row: self._on_edit(r, v, final))
            label = QtWidgets.QLabel(info.name)
            tip = info.description or ""
            if info.range:
                tip = (tip + "\n" if tip else "") + f"[{info.range['min']} … {info.range['max']}]"
            label.setToolTip(tip)
            self.form.addRow(label, row)
            self.rows[info.name] = row
        self._set_state("" if params else t("params.no_params"))

    # ---------- editing ----------
    def _on_edit(self, row, value, final):
        if self.real_robot and not final:
            return                              # real robot: nothing is sent while dragging
        if final and value == row.confirmed and row.info.name not in self._coalescer.pending:
            return                              # the node already has it (e.g. released where it started)
        verdict, reason = check_change(row.info, row.confirmed, value, self.real_robot)
        if verdict == "reject":
            row.revert_display()
            row.set_status(False, reason)
            return
        if verdict == "confirm" and not self._confirm(reason):
            row.revert_display()
            return
        self._coalescer.submit(row.info.name, value)
        if final:
            self._send(self._coalescer.flush())

    def _confirm(self, reason):
        answer = QtWidgets.QMessageBox.question(self, t("params.confirm_title"), reason,
                                                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                                QtWidgets.QMessageBox.No)
        return answer == QtWidgets.QMessageBox.Yes

    def _send_due(self):
        due = self._coalescer.due()
        if due:
            self._send(due)

    def _send(self, items):
        node = self.node
        for name, value in items:
            row = self.rows.get(name)
            if row is None or self.client is None:
                continue
            row.seq += 1
            seq = row.seq
            self._worker.submit(
                lambda name=name, value=value: self.client.set_param(node, name, value),
                lambda applied, r=row, s=seq: self._on_applied(r, s, applied),
                lambda exc, r=row, s=seq: self._on_rejected(r, s, exc))

    def _on_applied(self, row, seq, applied):
        if self.rows.get(row.info.name) is not row:
            return                              # rows were rebuilt (another node, refresh)
        row.confirmed = applied
        if seq == row.seq:
            row.set_status(True, t("params.applied"))
            if not (row.slider and row.slider.isSliderDown()):
                row._show(applied)

    def _on_rejected(self, row, seq, exc):
        if self.rows.get(row.info.name) is not row or seq != row.seq:
            return                              # a newer value is already on its way
        row.revert_display()
        row.set_status(False, str(exc))
        self.log.emit(f"{self.node}/{row.info.name}: {exc}")

    # ---------- buttons ----------
    def revert(self):
        changed = [r for r in self.rows.values() if r.changed]
        if not changed:
            return
        if self.real_robot and not self._confirm(t("params.revert_confirm")):
            return
        for r in changed:
            r._show(r.original)
        self._send([(r.info.name, r.original) for r in changed])

    def save(self):
        values = {name: r.confirmed for name, r in self.rows.items() if r.changed}
        if not values or not self.node:
            self._set_state(t("params.nothing_to_save"))
            return
        if not self.project_path:
            self._set_state(t("log.no_project"))
            return
        try:
            path = param_store.save_values(self.project_path, self.node, values)
        except param_store.ParamFileError as e:
            self._set_state(str(e))
            self.log.emit(str(e))
            return
        msg = t("params.saved", path=path, names=", ".join(sorted(values)))
        self._set_state(msg)
        self.log.emit(msg)

    def closeEvent(self, event):
        self._timer.stop()
        pending = self._coalescer.flush()
        if pending:
            self._send(pending)                 # the last slider position is not lost
        self._worker.stop()
        super().closeEvent(event)
