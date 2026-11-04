"""
Studio side of the parameter bridge (core/param_bridge.py): read and set ROS 2 node parameters
of a running graph, locally (session container) or on the robot (over the Remote SSH connection).

One long-lived bridge process instead of `ros2 param set` per change: the CLI starts in 1-2 s,
the bridge answers in milliseconds, which is what a slider needs. No Qt here.

    client = ParamClient(PipeTransport(local_argv("ros2_orchestrator_session")))
    client.start()
    for p in client.list_params("/pid"):
        ...
    client.set_param("/pid", "kp", 1.5)
    client.close()
"""
import collections
import itertools
import json
import os
import queue
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass

from core.i18n import t

BRIDGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "param_bridge.py")
LOCAL_ROS_SETUP = "/opt/ros/humble/setup.bash"
LOCAL_WS_SETUP = "/root/ros2_ws/install/setup.bash"
START_TIMEOUT = 20.0            # rclpy init + DDS start on a slow robot computer
REQUEST_TIMEOUT = 8.0           # the bridge itself gives up on a service after 2 s per call


class ParamBridgeError(RuntimeError):
    """code: machine code from the bridge or the client; str(e) is the translated message."""
    def __init__(self, code, detail=""):
        self.code = code
        self.detail = detail
        key = f"params.err_{code}"
        msg = t(key, detail=detail)
        if msg == key:                         # code without its own text
            msg = t("params.err_internal", detail=f"{code}: {detail}")
        super().__init__(msg)


@dataclass
class ParamInfo:
    name: str
    type: str
    value: object
    read_only: bool = False
    description: str = ""
    range: dict = None          # {"min", "max", "step"} or None

    @classmethod
    def from_dict(cls, d):
        return cls(name=d["name"], type=d["type"], value=d.get("value"),
                   read_only=bool(d.get("read_only")), description=d.get("description") or "",
                   range=d.get("range"))

    @property
    def numeric(self):
        return self.type in ("int", "double")


# ---------- launching the bridge ----------

def bridge_source():
    with open(BRIDGE_PATH, encoding="utf-8") as f:
        return f.read()


def _bridge_script(ros_setup, ws_setup):
    """bash script that sources ROS (and the workspace, if built) and runs the source passed as $0."""
    return (f"source {shlex.quote(ros_setup)} || exit 3; "
            f"if [ -f {shlex.quote(ws_setup)} ]; then source {shlex.quote(ws_setup)}; fi; "
            'exec python3 -u -c "$0"')


def local_argv(container, ros_setup=LOCAL_ROS_SETUP, ws_setup=LOCAL_WS_SETUP):
    """argv for the bridge in the local session container (docker exec -i, stdin stays open)."""
    return ["docker", "exec", "-i", container, "bash", "-c",
            _bridge_script(ros_setup, ws_setup), bridge_source()]


def remote_command(profile):
    """exec_command string for the bridge on the robot: in the profile's container or on the host."""
    prefix = ["docker", "exec", "-i", profile.container] if profile.container else []
    ws_setup = f"{profile.build_dir.rstrip('/')}/install/setup.bash"
    argv = prefix + ["bash", "-c", _bridge_script(profile.ros_setup, ws_setup), bridge_source()]
    return " ".join(shlex.quote(a) for a in argv)


# ---------- transports: a bidirectional line stream ----------

class PipeTransport:
    """Local process (docker exec -i ...) with stdin/stdout pipes."""

    def __init__(self, argv):
        self.argv = argv
        self.proc = None
        self.stderr_tail = collections.deque(maxlen=40)

    def start(self):
        kw = {}
        if os.name == "nt":
            kw["creationflags"] = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.proc = subprocess.Popen(self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True, encoding="utf-8",
                                     errors="replace", bufsize=1, **kw)
        threading.Thread(target=self._drain_stderr, name="param-bridge-stderr", daemon=True).start()

    def _drain_stderr(self):
        for line in self.proc.stderr:
            self.stderr_tail.append(line.rstrip("\n"))

    def send(self, line):
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()

    def lines(self):
        for line in self.proc.stdout:
            yield line

    def close(self):
        if not self.proc:
            return
        try:
            self.proc.stdin.close()             # EOF: the bridge leaves its loop and shuts rclpy down
        except OSError:
            pass
        try:
            self.proc.wait(3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(3)


class SSHTransport:
    """The bridge on the robot through the Remote tab's paramiko connection."""

    def __init__(self, ssh, command):
        self.ssh = ssh
        self.command = command
        self.chan = None
        self.stderr_tail = collections.deque(maxlen=40)

    def start(self):
        self.chan = self.ssh.get_transport().open_session()
        self.chan.exec_command(self.command)
        self._out = self.chan.makefile("r")
        threading.Thread(target=self._drain_stderr, name="param-bridge-stderr", daemon=True).start()

    def _drain_stderr(self):
        try:
            for line in self.chan.makefile_stderr("r"):
                self.stderr_tail.append(line.decode("utf-8", "replace").rstrip("\n")
                                        if isinstance(line, bytes) else line.rstrip("\n"))
        except Exception:
            pass

    def send(self, line):
        self.chan.sendall((line + "\n").encode("utf-8"))

    def lines(self):
        for line in self._out:
            yield line.decode("utf-8", "replace") if isinstance(line, bytes) else line

    def close(self):
        if not self.chan:
            return
        try:
            self.chan.shutdown_write()          # EOF on the bridge's stdin
        except Exception:
            pass
        deadline = time.time() + 3
        while not self.chan.exit_status_ready() and time.time() < deadline:
            time.sleep(0.05)
        self.chan.close()


# ---------- client ----------

class ParamClient:
    def __init__(self, transport, request_timeout=REQUEST_TIMEOUT):
        self.transport = transport
        self.request_timeout = request_timeout
        self._ids = itertools.count(1)
        self._pending = {}                      # id -> Queue(1)
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._ready = threading.Event()
        self._fatal = None
        self._closed = False
        self._reader = None

    @property
    def alive(self):
        return self._ready.is_set() and not self._closed and bool(self._reader and self._reader.is_alive())

    def start(self, timeout=START_TIMEOUT):
        try:
            self.transport.start()
        except Exception as e:
            raise ParamBridgeError("start", str(e)) from e
        self._reader = threading.Thread(target=self._read_loop, name="param-bridge-reader", daemon=True)
        self._reader.start()
        # _ready is also set when the bridge dies before "ready" (to wake us up): _closed tells them apart
        if not self._ready.wait(timeout) or self._fatal or self._closed:
            fatal = self._fatal
            self.close()
            if fatal:
                raise ParamBridgeError(fatal.get("error", "start"), fatal.get("detail", ""))
            time.sleep(0.2)                     # let the stderr reader catch the last lines
            raise ParamBridgeError("start", self._stderr() or t("params.no_answer"))
        return self

    def _stderr(self):
        return "\n".join(list(getattr(self.transport, "stderr_tail", []))[-5:])

    def _read_loop(self):
        try:
            for line in self.transport.lines():
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue                    # not ours (a stray print from sourced setup files)
                if not isinstance(msg, dict):
                    continue
                if msg.get("event") == "ready":
                    self._ready.set()
                elif msg.get("event") == "fatal":
                    self._fatal = msg
                    self._ready.set()
                elif msg.get("id") is not None:
                    with self._lock:
                        q = self._pending.pop(msg["id"], None)
                    if q is not None:
                        q.put(msg)
        except Exception:
            pass
        finally:
            self._ready.set()                   # unblock start() if the bridge died before "ready"
            with self._lock:
                pending, self._pending = self._pending, {}
                self._closed = True
            for q in pending.values():
                q.put({"ok": False, "error": "closed", "detail": self._stderr()})

    def request(self, op, timeout=None, **fields):
        if not self.alive:
            raise ParamBridgeError("closed", self._stderr())
        rid = next(self._ids)
        q = queue.Queue(1)
        with self._lock:
            self._pending[rid] = q
        try:
            with self._send_lock:
                self.transport.send(json.dumps({"id": rid, "op": op, **fields}, ensure_ascii=False))
        except Exception as e:
            with self._lock:
                self._pending.pop(rid, None)
            raise ParamBridgeError("closed", str(e)) from e
        try:
            msg = q.get(timeout=timeout or self.request_timeout)
        except queue.Empty:
            with self._lock:
                self._pending.pop(rid, None)
            raise ParamBridgeError("timeout", op) from None
        if not msg.get("ok"):
            raise ParamBridgeError(msg.get("error", "internal"), msg.get("detail", ""))
        return msg

    def list_nodes(self):
        return self.request("list_nodes")["nodes"]

    def list_params(self, node):
        return [ParamInfo.from_dict(d) for d in self.request("list", node=node)["params"]]

    def set_param(self, node, name, value):
        """The value the node really has now (after type conversion). ParamBridgeError if refused."""
        return self.request("set", node=node, name=name, value=value)["value"]

    def lifecycle_get(self, node):
        """'unconfigured' | 'inactive' | 'active' | 'finalized' (state label of a managed node)."""
        return self.request("lifecycle_get", node=node)["state"]

    def lifecycle_set(self, node, transition):
        """One transition ('configure', 'activate', ...); returns the state after it."""
        return self.request("lifecycle_set", node=node, transition=transition)["state"]

    def close(self):
        self._closed = True
        try:
            self.transport.close()
        except Exception:
            pass


# ---------- helpers for the UI ----------

def match_node(nodes, guess):
    """
    The running node that a graph node most likely is: exact name, then the same last
    segment (a namespace added by launch), then case-insensitive. None if nothing fits.
    """
    if not guess:
        return None
    g = guess if guess.startswith("/") else "/" + guess
    for cond in (lambda n: n == g, lambda n: n.rsplit("/", 1)[-1] == g[1:],
                 lambda n: n.rsplit("/", 1)[-1].lower() == g[1:].lower()):
        for n in nodes:
            if cond(n):
                return n
    return None


# state -> transitions that bring a managed node to the goal ("frozen" = inactive)
_LIFECYCLE_PLAN = {
    ("unconfigured", "active"): ("configure", "activate"),
    ("inactive", "active"): ("activate",),
    ("active", "active"): (),
    ("active", "inactive"): ("deactivate",),
    ("inactive", "inactive"): (),
    ("unconfigured", "inactive"): (),       # not configured yet: nothing runs, nothing to freeze
}


def drive_lifecycle(client, node, goal):
    """
    Brings a managed node to goal ('active' | 'inactive'). Returns (state before, transitions done, state after).
    ParamBridgeError from the bridge (not_lifecycle, a transition the node refused, ...) goes to the caller.
    """
    before = client.lifecycle_get(node)
    plan = _LIFECYCLE_PLAN.get((before, goal))
    if plan is None:
        raise ParamBridgeError("bad_transition", f"{goal} from '{before}'")
    state = before
    for transition in plan:
        state = client.lifecycle_set(node, transition)
    return before, list(plan), state


class Coalescer:
    """
    Slider throttle: while the slider moves, only the LAST value per parameter is sent,
    at most once per `interval` seconds. The UI calls submit() on every move and due() from a timer.
    """

    def __init__(self, interval=0.1, clock=time.monotonic):
        self.interval = interval
        self.clock = clock
        self._pending = {}                      # key -> value
        self._last_sent = {}                    # key -> time

    def submit(self, key, value):
        self._pending[key] = value

    @property
    def pending(self):
        """Keys that still wait to be sent."""
        return set(self._pending)

    def due(self):
        """[(key, value)] to send now; the rest waits for its interval."""
        now = self.clock()
        out = []
        for key in list(self._pending):
            if now - self._last_sent.get(key, float("-inf")) >= self.interval:
                out.append((key, self._pending.pop(key)))
                self._last_sent[key] = now
        return out

    def flush(self):
        """Everything still pending (slider released)."""
        out = list(self._pending.items())
        now = self.clock()
        for key, _ in out:
            self._last_sent[key] = now
        self._pending.clear()
        return out


def snap(info, value):
    """Clamp a numeric value to the declared range and round it to the step (rclcpp rejects off-step values)."""
    if not info.numeric:
        return value
    v = float(value)
    r = info.range
    if r:
        lo, hi, step = float(r["min"]), float(r["max"]), float(r.get("step") or 0)
        v = min(max(v, lo), hi)
        if step > 0:
            v = lo + round((v - lo) / step) * step
            v = min(v, hi)
    return int(round(v)) if info.type == "int" else v


# share of the declared range above which a single change on the real robot asks for confirmation
REAL_ROBOT_JUMP = 0.2


def check_change(info, old, new, real_robot):
    """
    ("ok", None) | ("confirm", reason) | ("reject", reason): what the UI must do before sending.
    On the real robot a gain jump is a joint jerk, so large or unbounded numeric changes are confirmed.
    """
    if info.read_only:
        return "reject", t("params.read_only")
    if info.numeric and info.range:
        lo, hi = float(info.range["min"]), float(info.range["max"])
        if not lo <= float(new) <= hi:
            return "reject", t("params.out_of_range", min=lo, max=hi)
    if not real_robot or not info.numeric:
        return "ok", None
    if not info.range:
        return "confirm", t("params.confirm_no_range")
    span = float(info.range["max"]) - float(info.range["min"])
    try:
        jump = abs(float(new) - float(old))
    except (TypeError, ValueError):
        return "confirm", t("params.confirm_no_range")
    if span > 0 and jump > REAL_ROBOT_JUMP * span:
        return "confirm", t("params.confirm_jump", percent=round(100 * jump / span))
    return "ok", None


class BridgeHolder:
    """
    One bridge per target (local session container or the connected robot), shared by all
    Parameters windows. A new one is started when the target changes or the old one died.
    Thread-safe: get() is called from window worker threads, close() from the UI.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._key = None
        self._client = None

    def get(self, key, make_transport):
        with self._lock:
            if self._client is not None and self._client.alive and self._key == key:
                return self._client
            old, self._client, self._key = self._client, None, None
        if old is not None:
            old.close()
        client = ParamClient(make_transport()).start()
        with self._lock:
            if self._client is None:
                self._client, self._key = client, key
                return client
        client.close()                          # another window started one meanwhile
        return self.get(key, make_transport)

    def close(self):
        with self._lock:
            old, self._client, self._key = self._client, None, None
        if old is not None:
            old.close()
