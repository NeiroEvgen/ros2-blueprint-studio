"""
Замер длительности этапов цикла «правка → работающий узел» (этап A для горячей замены).

PhaseTimer.phase("name") — контекст; итог: summary() для лога и record() — строка JSON
в <проект>/.blueprint/timings.jsonl (по ней видно, что реально съедает время: образ,
сессия, rosdep, colcon, запуск). Без Qt; ошибки записи не мешают работе.
"""
import json
import os
import time
from contextlib import contextmanager


class PhaseTimer:
    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._t0 = clock()
        self.phases = []                # [(name, seconds)]

    @contextmanager
    def phase(self, name):
        start = self._clock()
        try:
            yield
        finally:
            self.phases.append((name, self._clock() - start))

    def mark(self, name):
        """Время от создания таймера до события (например, первая строка вывода запуска)."""
        self.phases.append((name, self._clock() - self._t0))

    @property
    def total(self):
        return self._clock() - self._t0

    def summary(self):
        return ", ".join(f"{n} {s:.1f}s" for n, s in self.phases)

    def record(self, project_path, outcome="ok"):
        row = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "outcome": outcome,
               "total": round(self.total, 2),
               "phases": {n: round(s, 2) for n, s in self.phases}}
        try:
            d = os.path.join(project_path, ".blueprint")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, "timings.jsonl"), "a", encoding="utf-8") as f:
                f.write(json.dumps(row, ensure_ascii=False) + "\n")
            return True
        except OSError:
            return False
