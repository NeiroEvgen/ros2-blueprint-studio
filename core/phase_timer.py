"""
Timing of the "edit -> running node" cycle stages (groundwork for hot swap).

PhaseTimer.phase("name") is a context manager; results: summary() for the log and record(), a JSON line
in <project>/.blueprint/timings.jsonl (shows what really eats the time: image,
session, rosdep, colcon, launch). No Qt; write errors never get in the way.
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
        """Time from timer creation to an event (e.g. the first line of launch output)."""
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
