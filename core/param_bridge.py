"""
Parameter bridge: a long-lived process next to the ROS graph (in the session container or on the
robot) that reads and sets node parameters for Studio.

STANDALONE: Studio sends this file's source as `python3 -u -c <source>`, so it must not import
anything from Studio. Only the standard library at module level; rclpy is imported by RclpyBackend.

Protocol: one JSON object per line on stdin/stdout.
    -> {"id": 1, "op": "list_nodes"}
    <- {"id": 1, "ok": true, "nodes": ["/talker"]}
    -> {"id": 2, "op": "list", "node": "/talker"}
    <- {"id": 2, "ok": true, "params": [{"name": "kp", "type": "double", "value": 1.0,
                                         "read_only": false, "description": "",
                                         "range": {"min": 0.0, "max": 10.0, "step": 0.0}}]}
    -> {"id": 3, "op": "set", "node": "/talker", "name": "kp", "value": 1.5}
    <- {"id": 3, "ok": true, "value": 1.5}
    <- {"id": 3, "ok": false, "error": "rejected", "detail": "<reason from the node>"}
    -> {"id": 4, "op": "lifecycle_get", "node": "/camera"}
    <- {"id": 4, "ok": true, "state": "inactive"}
    -> {"id": 5, "op": "lifecycle_set", "node": "/camera", "transition": "activate"}
    <- {"id": 5, "ok": true, "state": "active"}
Events without an id: {"event": "ready", "version": 1} at start, {"event": "fatal", ...} before exit.
"""
import json
import os
import sys

PROTOCOL_VERSION = 1
LIFECYCLE_TRANSITIONS = ("configure", "cleanup", "activate", "deactivate", "shutdown")

# rcl_interfaces/msg/ParameterType
TYPE_NAMES = {0: "not_set", 1: "bool", 2: "int", 3: "double", 4: "string", 5: "byte_array",
              6: "bool_array", 7: "int_array", 8: "double_array", 9: "string_array"}
TYPE_IDS = {v: k for k, v in TYPE_NAMES.items()}
_VALUE_FIELDS = {1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value",
                 5: "byte_array_value", 6: "bool_array_value", 7: "integer_array_value",
                 8: "double_array_value", 9: "string_array_value"}


class BridgeError(Exception):
    """error: machine code for Studio (it translates it); detail: free text."""
    def __init__(self, error, detail=""):
        super().__init__(f"{error}: {detail}" if detail else error)
        self.error = error
        self.detail = detail


def _coerce_scalar(type_name, value):
    if type_name == "bool":
        if isinstance(value, bool):
            return value
        if isinstance(value, str) and value.strip().lower() in ("true", "false"):
            return value.strip().lower() == "true"
    elif type_name == "int":
        if isinstance(value, int) and not isinstance(value, bool):
            return value
        if isinstance(value, float) and value.is_integer():
            return int(value)
        if isinstance(value, str):
            try:
                return int(value.strip())
            except ValueError:
                pass
    elif type_name == "double":
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return float(value)
        if isinstance(value, str):
            try:
                return float(value.strip())
            except ValueError:
                pass
    elif type_name == "string":
        if isinstance(value, str):
            return value
    raise BridgeError("type", f"{value!r} is not {type_name}")


def coerce(type_name, value):
    """Value from the UI -> value of the parameter's declared type. BridgeError('type') if impossible."""
    if type_name.endswith("_array"):
        if not isinstance(value, list):
            raise BridgeError("type", f"{type_name} expects a list")
        if type_name == "byte_array":
            out = [_coerce_scalar("int", v) for v in value]
            if any(not 0 <= v <= 255 for v in out):
                raise BridgeError("type", "bytes are 0..255")
            return out
        return [_coerce_scalar(type_name[:-6], v) for v in value]
    if type_name == "not_set":
        raise BridgeError("type", "parameter is not set")
    return _coerce_scalar(type_name, value)


class Bridge:
    """Dispatches protocol requests to a backend (RclpyBackend or a fake in tests)."""

    def __init__(self, backend):
        self.backend = backend

    def handle(self, req):
        rid = req.get("id") if isinstance(req, dict) else None
        try:
            if not isinstance(req, dict):
                raise BridgeError("bad_request", "expected a JSON object")
            op = req.get("op")
            if op == "ping":
                body = {"version": PROTOCOL_VERSION}
            elif op == "list_nodes":
                body = {"nodes": sorted(self.backend.list_nodes())}
            elif op == "list":
                body = {"params": self.backend.list_params(self._node(req))}
            elif op == "set":
                name = req.get("name")
                if not isinstance(name, str) or not name:
                    raise BridgeError("bad_request", "name is required")
                if "value" not in req:
                    raise BridgeError("bad_request", "value is required")
                body = {"value": self.backend.set_param(self._node(req), name, req["value"])}
            elif op == "lifecycle_get":
                body = {"state": self.backend.lifecycle_get(self._node(req))}
            elif op == "lifecycle_set":
                transition = req.get("transition")
                if transition not in LIFECYCLE_TRANSITIONS:
                    raise BridgeError("bad_request", f"transition must be one of {', '.join(LIFECYCLE_TRANSITIONS)}")
                body = {"state": self.backend.lifecycle_set(self._node(req), transition)}
            else:
                raise BridgeError("unknown_op", str(op))
            return {"id": rid, "ok": True, **body}
        except BridgeError as e:
            return {"id": rid, "ok": False, "error": e.error, "detail": e.detail}
        except Exception as e:                      # the bridge must answer, not die
            return {"id": rid, "ok": False, "error": "internal", "detail": f"{type(e).__name__}: {e}"}

    @staticmethod
    def _node(req):
        node = req.get("node")
        if not isinstance(node, str) or not node:
            raise BridgeError("bad_request", "node is required")
        return node if node.startswith("/") else "/" + node


def _write(out, obj):
    out.write(json.dumps(obj, ensure_ascii=False) + "\n")
    out.flush()


def serve(backend, inp, out):
    """Request/response loop until stdin closes (Studio disconnected or closed the bridge)."""
    bridge = Bridge(backend)
    _write(out, {"event": "ready", "version": PROTOCOL_VERSION})
    for line in inp:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except ValueError as e:
            _write(out, {"id": None, "ok": False, "error": "bad_json", "detail": str(e)})
            continue
        _write(out, bridge.handle(req))


class RclpyBackend:
    """Parameters through the standard /<node>/{list,describe,get,set}_parameters services."""

    def __init__(self, timeout=2.0):
        import rclpy
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node(f"studio_param_bridge_{os.getpid()}")
        self.timeout = timeout
        self._clients = {}
        self._warmed_up = False

    def close(self):
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass

    def _call(self, srv_type, node, suffix, request):
        key = (node, suffix)
        cli = self._clients.get(key)
        if cli is None:
            cli = self._clients[key] = self.node.create_client(srv_type, f"{node}/{suffix}")
        if not cli.wait_for_service(timeout_sec=self.timeout):
            raise BridgeError("no_node", node)
        fut = cli.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.timeout)
        if not fut.done() or fut.result() is None:
            raise BridgeError("timeout", f"{node}/{suffix}")
        return fut.result()

    def _discover(self, settle=0.5):
        """
        Right after start DDS discovery delivers nodes one by one: wait until the node list
        has not changed for `settle` seconds (or the timeout), so the first answer is complete.
        """
        import time
        if self._warmed_up:
            return
        end = time.time() + self.timeout
        seen, stable_since = None, time.time()
        while time.time() < end:
            now = len(self.node.get_node_names_and_namespaces())
            if now != seen:
                seen, stable_since = now, time.time()
            elif now > 1 and time.time() - stable_since >= settle:
                break
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        self._warmed_up = True

    def list_nodes(self):
        self._discover()
        me = self.node.get_fully_qualified_name()
        out = []
        for name, ns in self.node.get_node_names_and_namespaces():
            full = (ns.rstrip("/") + "/" + name) if ns != "/" else "/" + name
            if full != me:
                out.append(full)
        return out

    @staticmethod
    def _value(pv):
        field = _VALUE_FIELDS.get(pv.type)
        if field is None:
            return None
        v = getattr(pv, field)
        if isinstance(v, (bool, int, float, str)):
            return v
        if pv.type == 5:                            # byte_array arrives as a list of bytes objects
            return [x if isinstance(x, int) else x[0] for x in v]
        return [x for x in v]

    @staticmethod
    def _range(d):
        r = list(d.floating_point_range) or list(d.integer_range)
        if not r:
            return None
        return {"min": r[0].from_value, "max": r[0].to_value, "step": r[0].step}

    def list_params(self, node):
        from rcl_interfaces.srv import ListParameters, DescribeParameters, GetParameters
        names = list(self._call(ListParameters, node, "list_parameters", ListParameters.Request()).result.names)
        if not names:
            return []
        descs = self._call(DescribeParameters, node, "describe_parameters",
                           DescribeParameters.Request(names=names)).descriptors
        values = self._call(GetParameters, node, "get_parameters", GetParameters.Request(names=names)).values
        out = []
        for name, d, v in zip(names, descs, values):
            out.append({"name": name, "type": TYPE_NAMES.get(v.type, "not_set"), "value": self._value(v),
                        "read_only": bool(d.read_only), "description": d.description,
                        "range": self._range(d)})
        return sorted(out, key=lambda p: p["name"])

    def set_param(self, node, name, value):
        from rcl_interfaces.msg import Parameter, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters
        current = self._call(GetParameters, node, "get_parameters", GetParameters.Request(names=[name])).values
        if not current or current[0].type == 0:
            raise BridgeError("no_param", name)
        type_id = current[0].type
        coerced = coerce(TYPE_NAMES[type_id], value)
        pv = ParameterValue(type=type_id)
        setattr(pv, _VALUE_FIELDS[type_id], [bytes([b]) for b in coerced] if type_id == 5 else coerced)
        res = self._call(SetParameters, node, "set_parameters",
                         SetParameters.Request(parameters=[Parameter(name=name, value=pv)])).results
        if not res or not res[0].successful:
            raise BridgeError("rejected", res[0].reason if res else "")
        return coerced


    # ---- lifecycle (managed nodes): /<node>/get_state, change_state, get_available_transitions ----

    def _require_lifecycle(self, node):
        """no_node if the node is not in the graph, not_lifecycle if it has no lifecycle services."""
        import time
        self._discover()
        ns, _, name = node.rpartition("/")
        end = time.time() + self.timeout
        while True:
            try:
                services = {s for s, _ in self.node.get_service_names_and_types_by_node(name, ns or "/")}
            except Exception:                       # the node is not (yet) known to the graph
                services = None
            if services is not None and f"{node}/get_state" in services:
                return
            if time.time() >= end:
                raise BridgeError("no_node" if services is None else "not_lifecycle", node)
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def lifecycle_get(self, node):
        from lifecycle_msgs.srv import GetState
        self._require_lifecycle(node)
        return self._call(GetState, node, "get_state", GetState.Request()).current_state.label

    def lifecycle_set(self, node, transition):
        from lifecycle_msgs.srv import ChangeState, GetAvailableTransitions
        state = self.lifecycle_get(node)
        available = self._call(GetAvailableTransitions, node, "get_available_transitions",
                               GetAvailableTransitions.Request()).available_transitions
        match = next((t.transition for t in available if t.transition.label == transition), None)
        if match is None:
            labels = ", ".join(sorted({t.transition.label for t in available})) or "-"
            raise BridgeError("bad_transition", f"{transition} from '{state}' (possible: {labels})")
        req = ChangeState.Request()
        req.transition.id = match.id
        req.transition.label = match.label
        if not self._call(ChangeState, node, "change_state", req).success:
            raise BridgeError("transition_failed", f"{transition} from '{state}'")
        return self.lifecycle_get(node)


def main():
    try:
        backend = RclpyBackend()
    except ImportError as e:
        _write(sys.stdout, {"event": "fatal", "error": "no_rclpy", "detail": str(e)})
        return 2
    except Exception as e:
        _write(sys.stdout, {"event": "fatal", "error": "start_failed", "detail": f"{type(e).__name__}: {e}"})
        return 2
    try:
        serve(backend, sys.stdin, sys.stdout)
    finally:
        backend.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
