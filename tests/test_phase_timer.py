"""Замер этапов деплоя: таймер, запись в timings.jsonl, DeployWorker с поддельным менеджером."""
import json
import os

import pytest

from core.phase_timer import PhaseTimer


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_phases_and_total():
    c = Clock(); tm = PhaseTimer(c)
    with tm.phase("image"):
        c.now += 2
    with tm.phase("colcon"):
        c.now += 30.5
    tm.mark("first_output")
    assert [n for n, _ in tm.phases] == ["image", "colcon", "first_output"]
    assert tm.total == 32.5
    assert tm.summary().startswith("image 2.0s, colcon 30.5s")


def test_phase_recorded_even_on_exception():
    c = Clock(); tm = PhaseTimer(c)
    with pytest.raises(RuntimeError):
        with tm.phase("deps"):
            c.now += 1
            raise RuntimeError("x")
    assert tm.phases == [("deps", 1.0)]


def test_record_appends_json_lines(tmp_path):
    tm = PhaseTimer(Clock())
    assert tm.record(str(tmp_path), "ok") and tm.record(str(tmp_path), "build_failed")
    rows = [json.loads(l) for l in (tmp_path / ".blueprint/timings.jsonl").read_text().splitlines()]
    assert [r["outcome"] for r in rows] == ["ok", "build_failed"]


def test_record_failure_is_silent(tmp_path):
    f = tmp_path / "file"; f.write_text("x")
    assert PhaseTimer().record(str(f)) is False


class FakeManager:
    def __init__(self, ret=None, boom=None):
        self.ret, self.boom = ret, boom

    def ensure_image(self, cb): pass
    def start_session(self, path, cb): pass

    def run_project_launch(self, sys_callback, ros_callback, extra_apt_packages=None, timer=None):
        with timer.phase("colcon"):
            pass
        if self.boom:
            raise self.boom
        return self.ret


@pytest.mark.parametrize("ret,boom,outcome", [
    (None, None, "ok"), (False, None, "build_failed"),
    (None, RuntimeError("Container is not running"), "stopped"),
    (None, RuntimeError("boom"), "error")])
def test_deploy_worker_records_outcome(tmp_path, ret, boom, outcome):
    from core.ros_process import DeployWorker
    w = DeployWorker(FakeManager(ret, boom), str(tmp_path))
    msgs = []
    w.sys_signal.connect(msgs.append)
    w.run()                                       # синхронно, без потока
    row = json.loads((tmp_path / ".blueprint/timings.jsonl").read_text().splitlines()[-1])
    assert row["outcome"] == outcome and "colcon" in row["phases"] and "image" in row["phases"]
    assert any(m.startswith("Timings:") for m in msgs)
