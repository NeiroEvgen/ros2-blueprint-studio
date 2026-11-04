"""
The real bridge loop (core/param_bridge.serve) with an in-memory backend instead of rclpy.
Run as a process by the tests: python fake_bridge.py [normal|fatal|noise]
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from core import param_bridge as pb  # noqa: E402


class FakeBackend:
    def __init__(self):
        self.params = {"/pid": {
            "kp": {"type": "double", "value": 1.0, "range": {"min": 0.0, "max": 10.0, "step": 0.0}},
            "rate_hz": {"type": "int", "value": 50, "range": {"min": 1, "max": 500, "step": 1}},
            "frame": {"type": "string", "value": "base", "read_only": True},
            "slow": {"type": "int", "value": 0},
            "crash": {"type": "int", "value": 0},
        }}

    def list_nodes(self):
        return list(self.params) + list(self.lifecycle)

    def list_params(self, node):
        if node not in self.params:
            raise pb.BridgeError("no_node", node)
        return [{"name": n, "type": p["type"], "value": p["value"], "read_only": p.get("read_only", False),
                 "description": "", "range": p.get("range")} for n, p in sorted(self.params[node].items())]

    lifecycle = {"/cam": "unconfigured"}
    _moves = {("unconfigured", "configure"): "inactive", ("inactive", "activate"): "active",
              ("active", "deactivate"): "inactive", ("inactive", "cleanup"): "unconfigured"}

    def lifecycle_get(self, node):
        if node not in self.lifecycle:
            raise pb.BridgeError("not_lifecycle" if node in self.params else "no_node", node)
        return self.lifecycle[node]

    def lifecycle_set(self, node, transition):
        state = self.lifecycle_get(node)
        new = self._moves.get((state, transition))
        if new is None:
            raise pb.BridgeError("bad_transition", f"{transition} from '{state}'")
        self.lifecycle[node] = new
        return new

    def set_param(self, node, name, value):
        if node not in self.params:
            raise pb.BridgeError("no_node", node)
        p = self.params[node].get(name)
        if p is None:
            raise pb.BridgeError("no_param", name)
        if name == "slow":
            time.sleep(float(value))
        if name == "crash":
            os._exit(1)
        v = pb.coerce(p["type"], value)
        if p.get("read_only"):
            raise pb.BridgeError("rejected", "read-only")
        r = p.get("range")
        if r and not r["min"] <= v <= r["max"]:
            raise pb.BridgeError("rejected", "out of range")
        p["value"] = v
        return v


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "normal"
    if mode == "fatal":
        pb._write(sys.stdout, {"event": "fatal", "error": "no_rclpy", "detail": "No module named 'rclpy'"})
        sys.exit(2)
    if mode == "noise":
        print("sourcing workspace... some setup.bash chatter", flush=True)
    pb.serve(FakeBackend(), sys.stdin, sys.stdout)
