"""
RemoteRunner: a named background process must not break Cancel of a running build.

Background runs (name=...) wrote their pgid into the foreground fields and cleared them on exit:
a node that finished during a build reset the build's pgid, cancel() returned False and colcon
kept running on the robot. Real sshd (fixture in conftest).
"""
import threading
import time

import pytest

from core.remote.profile import RemoteProfile
from core.remote.runner import RemoteRunner


@pytest.fixture
def client(sshd):
    import paramiko
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(sshd["host"], sshd["port"], sshd["user"], key_filename=sshd["key"],
              look_for_keys=False, allow_agent=False)
    yield c
    c.close()


def test_background_process_does_not_break_cancel(client, tmp_path):
    r = RemoteRunner(client, RemoteProfile(host="h", user="u", remote_workspace=str(tmp_path)), "pkg")
    bg = threading.Thread(target=lambda: r.run_streaming("sleep 0.5", lambda l: None, name="Node"))
    bg.start()
    fg = threading.Thread(target=lambda: r.run_streaming("sleep 30", lambda l: None))
    fg.start()
    bg.join(5)                       # the background process ended while the build is running
    time.sleep(0.3)
    assert r.cancel() is True        # used to be False: the background run had cleared the build's pgid
    fg.join(10)
    assert not fg.is_alive()


def test_named_run_is_stopped_by_name_not_by_cancel(client, tmp_path):
    r = RemoteRunner(client, RemoteProfile(host="h", user="u", remote_workspace=str(tmp_path)), "pkg")
    bg = threading.Thread(target=lambda: r.run_streaming("sleep 30", lambda l: None, name="Node"))
    bg.start()
    deadline = time.time() + 5
    while "Node" not in r.list_running() and time.time() < deadline:
        time.sleep(0.05)
    assert r.cancel() is False       # no build running: Cancel does not touch the node
    assert r.stop("Node") is True
    bg.join(10)
    assert not bg.is_alive()
