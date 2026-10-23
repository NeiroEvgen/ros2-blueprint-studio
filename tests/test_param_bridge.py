"""
Parameter bridge (core/param_bridge.py) and its Studio client (core/param_client.py).

Layers: protocol and type conversion without processes; client <-> real bridge loop over pipes
(fake backend); the same over real sshd; and the real rclpy backend against a ROS 2 Humble
container when Docker and the ros:humble-ros-core image are available.
"""
import io
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import types

import pytest

from core import param_bridge as pb
from core import param_client as pc
from core.param_client import ParamBridgeError, ParamClient, ParamInfo, PipeTransport

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FAKE = os.path.join(ROOT, "tests", "ros", "fake_bridge.py")
DEMO = os.path.join(ROOT, "tests", "ros", "demo_param_node.py")


# ---------- protocol without processes ----------

class TestCoerce:
    @pytest.mark.parametrize("type_, value, expected", [
        ("double", 2, 2.0), ("double", "1.5", 1.5), ("int", 3.0, 3), ("int", "7", 7),
        ("bool", "true", True), ("bool", False, False), ("string", "x", "x"),
        ("double_array", [1, 2], [1.0, 2.0]), ("byte_array", [0, 255], [0, 255]),
    ])
    def test_ok(self, type_, value, expected):
        out = pb.coerce(type_, value)
        assert out == expected and type(out) is type(expected)

    @pytest.mark.parametrize("type_, value", [
        ("int", 1.5), ("int", True), ("double", True), ("double", "abc"), ("bool", 1),
        ("string", 5), ("int_array", 5), ("byte_array", [256]), ("not_set", 1),
    ])
    def test_rejected(self, type_, value):
        with pytest.raises(pb.BridgeError) as e:
            pb.coerce(type_, value)
        assert e.value.error == "type"


class Backend:
    def list_nodes(self):
        return ["/b", "/a"]

    def list_params(self, node):
        return [{"name": "kp", "node": node}]

    def set_param(self, node, name, value):
        if name == "boom":
            raise ZeroDivisionError("x")
        return value


class TestHandle:
    bridge = pb.Bridge(Backend())

    def test_ops(self):
        assert self.bridge.handle({"id": 1, "op": "ping"}) == {"id": 1, "ok": True, "version": 1}
        assert self.bridge.handle({"id": 2, "op": "list_nodes"})["nodes"] == ["/a", "/b"]
        assert self.bridge.handle({"id": 3, "op": "list", "node": "pid"})["params"][0]["node"] == "/pid"
        assert self.bridge.handle({"id": 4, "op": "set", "node": "/p", "name": "kp", "value": 2})["value"] == 2

    @pytest.mark.parametrize("req, error", [
        ({"id": 1, "op": "nope"}, "unknown_op"),
        ({"id": 1, "op": "list"}, "bad_request"),
        ({"id": 1, "op": "set", "node": "/p", "value": 1}, "bad_request"),
        ({"id": 1, "op": "set", "node": "/p", "name": "kp"}, "bad_request"),
        ({"id": 1, "op": "set", "node": "/p", "name": "boom", "value": 1}, "internal"),
        ([1, 2], "bad_request"),
    ])
    def test_errors_are_answers_not_crashes(self, req, error):
        assert self.bridge.handle(req)["error"] == error

    def test_serve_loop(self):
        inp = io.StringIO('{"id": 1, "op": "ping"}\n\nnot json\n{"id": 2, "op": "list_nodes"}\n')
        out = io.StringIO()
        pb.serve(Backend(), inp, out)
        msgs = [json.loads(line) for line in out.getvalue().splitlines()]
        assert msgs[0] == {"event": "ready", "version": 1}
        assert msgs[1]["id"] == 1 and msgs[2]["error"] == "bad_json" and msgs[3]["nodes"] == ["/a", "/b"]

    def test_bridge_is_standalone(self):
        """Studio sends the source as `python3 -c`: no Studio imports allowed."""
        src = pc.bridge_source()
        assert "from core" not in src and "import core" not in src


# ---------- client <-> real bridge loop over pipes ----------

def fake_client(mode="normal", **kw):
    return ParamClient(PipeTransport([sys.executable, FAKE, mode]), **kw)


@pytest.fixture
def client():
    c = fake_client().start(timeout=10)
    yield c
    c.close()


class TestClientOverPipes:
    def test_list_and_set(self, client):
        assert client.list_nodes() == ["/pid"]
        params = {p.name: p for p in client.list_params("/pid")}
        assert params["kp"].range == {"min": 0.0, "max": 10.0, "step": 0.0} and params["frame"].read_only
        assert client.set_param("/pid", "kp", 2) == 2.0
        assert {p.name: p.value for p in client.list_params("/pid")}["kp"] == 2.0

    def test_refusals_are_translated_errors(self, client):
        cases = [(("/pid", "kp", 11), "rejected"), (("/pid", "rate_hz", "x"), "type"),
                 (("/nope", "kp", 1), "no_node"), (("/pid", "zzz", 1), "no_param")]
        for args, code in cases:
            with pytest.raises(ParamBridgeError) as e:
                client.set_param(*args)
            assert e.value.code == code
            assert not str(e.value).startswith("params.")      # a real text, not the key

    def test_concurrent_requests_get_their_own_answers(self, client):
        results, errors = {}, []

        def worker(i):
            try:
                results[i] = client.set_param("/pid", "rate_hz", i + 1)
            except Exception as e:                              # pragma: no cover
                errors.append(e)
        ts = [threading.Thread(target=worker, args=(i,)) for i in range(20)]
        [t_.start() for t_ in ts]
        [t_.join(10) for t_ in ts]
        assert not errors and results == {i: i + 1 for i in range(20)}

    def test_timeout_does_not_poison_next_request(self):
        c = fake_client(request_timeout=0.3).start(timeout=10)
        try:
            with pytest.raises(ParamBridgeError) as e:
                c.set_param("/pid", "slow", 1)
            assert e.value.code == "timeout"
            time.sleep(1)                                       # the late answer arrives and is dropped
            assert c.set_param("/pid", "kp", 3) == 3.0
        finally:
            c.close()

    def test_bridge_death_fails_pending_and_next_requests(self, client):
        with pytest.raises(ParamBridgeError) as e:
            client.set_param("/pid", "crash", 1)
        assert e.value.code == "closed"
        with pytest.raises(ParamBridgeError) as e:
            client.list_nodes()
        assert e.value.code == "closed" and not client.alive

    def test_fatal_at_start(self):
        with pytest.raises(ParamBridgeError) as e:
            fake_client("fatal").start(timeout=10)
        assert e.value.code == "no_rclpy" and "rclpy" in str(e.value)

    def test_setup_noise_before_ready_is_ignored(self):
        c = fake_client("noise").start(timeout=10)
        try:
            assert c.list_nodes() == ["/pid"]
        finally:
            c.close()

    def test_command_not_found(self):
        with pytest.raises(ParamBridgeError) as e:
            ParamClient(PipeTransport(["definitely-not-a-command-xyz"])).start(timeout=5)
        assert e.value.code == "start"

    def test_process_exits_without_ready(self):
        c = ParamClient(PipeTransport([sys.executable, "-c", "import sys; sys.stderr.write('boom\\n')"]))
        with pytest.raises(ParamBridgeError) as e:
            c.start(timeout=10)
        assert e.value.code == "start" and "boom" in str(e.value)

    def test_close_ends_the_bridge(self, client):
        proc = client.transport.proc
        client.close()
        assert proc.poll() is not None


# ---------- over real SSH ----------

@pytest.fixture
def ssh_client(sshd):
    import paramiko
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(sshd["host"], sshd["port"], sshd["user"], key_filename=sshd["key"],
              look_for_keys=False, allow_agent=False)
    yield c
    c.close()


class TestOverSSH:
    def test_fake_bridge_over_ssh(self, ssh_client):
        import shlex
        cmd = " ".join(shlex.quote(a) for a in (sys.executable, FAKE))
        c = ParamClient(pc.SSHTransport(ssh_client, cmd)).start(timeout=10)
        try:
            assert c.set_param("/pid", "kp", 4) == 4.0
            with pytest.raises(ParamBridgeError) as e:
                c.set_param("/pid", "kp", 99)
            assert e.value.code == "rejected"
        finally:
            c.close()

    def test_remote_command_quoting_reaches_python(self, ssh_client, tmp_path):
        """The whole bridge source survives ssh -> bash -c -> python3 -c; without ROS it says no_rclpy."""
        if shutil.which("python3") is None:
            pytest.skip("python3 is not on PATH")
        setup = tmp_path / "setup.bash"
        setup.write_text("echo sourced-ros-chatter\n", encoding="utf-8")
        profile = types.SimpleNamespace(container="", ros_setup=str(setup), build_dir=str(tmp_path))
        with pytest.raises(ParamBridgeError) as e:
            ParamClient(pc.SSHTransport(ssh_client, pc.remote_command(profile))).start(timeout=15)
        assert e.value.code == "no_rclpy"

    def test_missing_ros_setup_is_reported(self, ssh_client, tmp_path):
        profile = types.SimpleNamespace(container="", ros_setup=str(tmp_path / "nope.bash"),
                                        build_dir=str(tmp_path))
        with pytest.raises(ParamBridgeError) as e:
            ParamClient(pc.SSHTransport(ssh_client, pc.remote_command(profile))).start(timeout=15)
        assert e.value.code == "start" and "nope.bash" in str(e.value)


# ---------- UI helpers ----------

class TestCoalescer:
    def test_only_last_value_at_most_once_per_interval(self):
        now = [0.0]
        c = pc.Coalescer(interval=0.1, clock=lambda: now[0])
        c.submit("kp", 1); c.submit("kp", 2)
        assert c.due() == [("kp", 2)]
        c.submit("kp", 3); c.submit("kp", 4)
        assert c.due() == []                                    # interval not over yet
        now[0] = 0.1
        assert c.due() == [("kp", 4)]
        assert c.due() == []

    def test_flush_on_release(self):
        c = pc.Coalescer(interval=10, clock=lambda: 0.0)
        c.submit("kp", 1)
        c.due()
        c.submit("kp", 5); c.submit("ki", 1)
        assert sorted(c.flush()) == [("ki", 1), ("kp", 5)] and c.due() == []


def info(type_="double", rng=None, ro=False):
    return ParamInfo(name="p", type=type_, value=0, read_only=ro, range=rng)


class TestSnap:
    def test_clamp_and_step(self):
        assert pc.snap(info(rng={"min": 0.0, "max": 1.0, "step": 0.25}), 0.6) == 0.5
        assert pc.snap(info(rng={"min": 0.0, "max": 1.0, "step": 0.0}), 7) == 1.0
        assert pc.snap(info("int", {"min": 1, "max": 500, "step": 5}), 13) == 11
        assert pc.snap(info("int"), 2.6) == 3 and pc.snap(info("string"), "x") == "x"


class TestCheckChange:
    rng = {"min": 0.0, "max": 10.0, "step": 0.0}

    def test_sim_allows_any_change_inside_range(self):
        assert pc.check_change(info(rng=self.rng), 0, 9, real_robot=False) == ("ok", None)
        assert pc.check_change(info(), 0, 1e6, real_robot=False) == ("ok", None)

    def test_rejects_read_only_and_out_of_range(self):
        assert pc.check_change(info(ro=True), 0, 1, False)[0] == "reject"
        assert pc.check_change(info(rng=self.rng), 0, 11, False)[0] == "reject"

    def test_real_robot_confirms_big_jumps_and_unbounded(self):
        assert pc.check_change(info(rng=self.rng), 1.0, 1.5, True) == ("ok", None)
        verdict, reason = pc.check_change(info(rng=self.rng), 1.0, 5.0, True)
        assert verdict == "confirm" and "40" in reason
        assert pc.check_change(info(), 0, 1, True)[0] == "confirm"
        assert pc.check_change(info("bool"), False, True, True) == ("ok", None)


def test_commands_carry_the_bridge_source():
    argv = pc.local_argv("sess")
    assert argv[:4] == ["docker", "exec", "-i", "sess"] and argv[-1] == pc.bridge_source()
    prof = types.SimpleNamespace(container="g1pilot_run", ros_setup="/opt/ros/humble/setup.bash",
                                 build_dir="/ws/")
    cmd = pc.remote_command(prof)
    assert cmd.startswith("docker exec -i g1pilot_run bash -c ") and "/ws/install/setup.bash" in cmd


# ---------- real rclpy backend against ROS 2 Humble ----------

IMAGE = "ros:humble-ros-core"


def _docker_ready():
    if shutil.which("docker") is None:
        return False
    try:
        r = subprocess.run(["docker", "image", "inspect", IMAGE], capture_output=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return r.returncode == 0


@pytest.fixture(scope="module")
def ros_container():
    if not _docker_ready():
        pytest.skip(f"Docker with the {IMAGE} image is not available")
    name = f"studio_param_test_{os.getpid()}"
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    subprocess.run(["docker", "run", "-d", "--name", name, "-v", f"{os.path.dirname(DEMO)}:/demo:ro", IMAGE,
                    "bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 -u /demo/demo_param_node.py"],
                   check=True, capture_output=True)
    yield name
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)


class TestRealRos:
    def test_list_set_reject_and_latency(self, ros_container):
        c = ParamClient(PipeTransport(pc.local_argv(ros_container))).start()
        try:
            deadline = time.time() + 10
            while "/pid_demo" not in c.list_nodes() and time.time() < deadline:
                time.sleep(0.2)
            params = {p.name: p for p in c.list_params("/pid_demo")}
            assert params["kp"].range["max"] == 10.0 and params["rate_hz"].type == "int"
            assert params["frame"].read_only and params["weights"].value == [1.0, 2.0]

            t0 = time.time()
            for i in range(20):
                assert c.set_param("/pid_demo", "kp", i * 0.5) == i * 0.5
            per_set = (time.time() - t0) / 20
            assert per_set < 0.25, f"{per_set:.3f} s per set: too slow for a slider"

            for args, code in [(("kp", 11), "rejected"), (("kp", 6.66), "rejected"),
                               (("frame", "x"), "rejected"), (("rate_hz", "abc"), "type")]:
                with pytest.raises(ParamBridgeError) as e:
                    c.set_param("/pid_demo", *args)
                assert e.value.code == code
            assert c.set_param("/pid_demo", "mask", [4, 5]) == [4, 5]
            assert c.set_param("/pid_demo", "rate_hz", 100.0) == 100
        finally:
            c.close()
