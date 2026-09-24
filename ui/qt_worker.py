"""Фоновое выполнение блокирующих операций (SSH, sync, сборка) без фриза UI."""
from PySide6 import QtCore


class Task(QtCore.QThread):
    """fn(emit_line) -> result. Сигналы доставляются в главный поток."""
    line = QtCore.Signal(str)
    done = QtCore.Signal(object)
    failed = QtCore.Signal(str)

    def __init__(self, fn, parent=None):
        super().__init__(parent)
        self._fn = fn

    def run(self):
        try:
            self.done.emit(self._fn(self.line.emit))
        except Exception as e:
            self.failed.emit(f"{type(e).__name__}: {e}")


class TaskHost:
    """Держит ссылки на живые задачи, чтобы их не собрал GC посреди работы."""
    def __init__(self):
        self._tasks = set()

    def start(self, fn, on_done=None, on_fail=None, on_line=None, parent=None):
        t = Task(fn, parent)
        if on_line:
            t.line.connect(on_line)
        if on_done:
            t.done.connect(on_done)
        if on_fail:
            t.failed.connect(on_fail)
        self._tasks.add(t)
        t.finished.connect(lambda: self._tasks.discard(t))
        t.start()
        return t

    @property
    def busy(self):
        return any(t.isRunning() for t in self._tasks)

    def wait_all(self, ms=30000):
        for t in list(self._tasks):
            t.wait(ms)
