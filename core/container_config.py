# core/container_config.py
# v0.6.0: deploy container configuration model + preset stores.
# Config = WHAT is inside the container. Target = WHERE it is deployed. One YAML format
# serves as the deploy config and as the base of the manifest system.

import os
import re
import yaml
import copy

DEFAULT_IMAGE = "osrf/ros:humble-desktop"

DEFAULT_CONFIG = {
    "name": "main",
    "image": DEFAULT_IMAGE,
    # Global ROS env
    "env": {
        "ROS_DOMAIN_ID": "0",
        "PYTHONUNBUFFERED": "1",
        "RCUTILS_LOGGING_BUFFERED_STREAM": "0",
    },
    # DDS: vendor -> RMW_IMPLEMENTATION, qos_xml -> mounted profile
    "dds": {
        "vendor": "default",        # default | fastrtps | cyclonedds
        "qos_xml": "",              # path to the vendor XML profile (optional)
    },
    # Network impairment simulation via tc netem. Needs NET_ADMIN.
    "netem": {
        "enabled": False,
        "loss_percent": 0.0,
        "delay_ms": 0,
        "jitter_ms": 0,
        "interface": "eth0",
    },
    "apt_packages": [],
    "pip_packages": [],
    "restart": "no",                # no | always | on-failure | unless-stopped
    "privileged": True,             # current default behaviour
    "network_mode": "host",         # host | bridge
    "resources": {"cpus": "", "memory": ""},   # empty = no limit
    # Deploy target: local or ssh://user@host
    "target": {"type": "local", "host": "", "real_robot": False},
}

RMW_BY_VENDOR = {
    "default": None,
    "fastrtps": "rmw_fastrtps_cpp",
    "cyclonedds": "rmw_cyclonedds_cpp",
}

# Built-in presets
BUILTIN_PRESETS = {
    "Local Default": {},
    "GPU Node": {
        "image": "osrf/ros:humble-desktop",  # replace with your CUDA image
        "resources": {"cpus": "", "memory": ""},
        "env": {"NVIDIA_VISIBLE_DEVICES": "all"},
    },
    "Critical Service": {
        "restart": "always",
        "network_mode": "host",
    },
    "Sandbox (draft)": {
        # NOTE: this is NOT a security sandbox.
        "privileged": False,
        "network_mode": "bridge",
        "resources": {"cpus": "1", "memory": "512m"},
    },
    "Remote Server": {
        "target": {"type": "ssh", "host": "ssh://user@192.168.1.50"},
    },
}


def _merge(base, override):
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def safe_group_name(name):
    s = re.sub(r"[^a-zA-Z0-9_]", "_", str(name or "main")).strip("_").lower()
    return s or "main"


def make_config(name, base_overrides=None):
    cfg = _merge(DEFAULT_CONFIG, base_overrides or {})
    cfg["name"] = safe_group_name(name)
    return cfg


def resolved_env(cfg):
    """Final env dict for the container, taking the DDS vendor into account."""
    env = dict(cfg.get("env") or {})
    rmw = RMW_BY_VENDOR.get((cfg.get("dds") or {}).get("vendor", "default"))
    if rmw:
        env["RMW_IMPLEMENTATION"] = rmw
    qos = (cfg.get("dds") or {}).get("qos_xml", "")
    if qos:
        vendor = (cfg.get("dds") or {}).get("vendor")
        if vendor == "fastrtps":
            env["FASTRTPS_DEFAULT_PROFILES_FILE"] = "/etc/dds_profile.xml"
        elif vendor == "cyclonedds":
            env["CYCLONEDDS_URI"] = "file:///etc/dds_profile.xml"
    return env


def netem_command(cfg):
    """tc netem command, or None if the simulation is off."""
    n = cfg.get("netem") or {}
    if not n.get("enabled"):
        return None
    parts = ["tc qdisc replace dev", n.get("interface", "eth0"), "root netem"]
    loss = float(n.get("loss_percent") or 0)
    delay = int(n.get("delay_ms") or 0)
    jitter = int(n.get("jitter_ms") or 0)
    if loss > 0:
        parts.append(f"loss {loss}%")
    if delay > 0:
        parts.append(f"delay {delay}ms" + (f" {jitter}ms" if jitter > 0 else ""))
    if len(parts) <= 3:
        return None
    return " ".join(parts)


class ContainerConfigStore:
    """Container group configs of the current project: <project>/.blueprint/containers/*.yaml"""

    def __init__(self, project_path):
        self.dir = os.path.join(project_path, ".blueprint", "containers")
        os.makedirs(self.dir, exist_ok=True)

    def _path(self, name):
        return os.path.join(self.dir, f"{safe_group_name(name)}.yaml")

    def list_names(self):
        names = []
        for f in sorted(os.listdir(self.dir)):
            if f.endswith(".yaml"):
                names.append(os.path.splitext(f)[0])
        if "main" not in names:
            names.insert(0, "main")
        return names

    def get(self, name):
        p = self._path(name)
        if os.path.exists(p):
            try:
                with open(p, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                return _merge(DEFAULT_CONFIG, data) | {"name": safe_group_name(name)}
            except Exception:
                pass
        return make_config(name)

    def save(self, cfg):
        cfg = _merge(DEFAULT_CONFIG, cfg)
        with open(self._path(cfg["name"]), "w", encoding="utf-8") as f:
            yaml.dump(cfg, f, sort_keys=False, allow_unicode=True)
        return cfg

    def delete(self, name):
        p = self._path(name)
        if os.path.exists(p) and safe_group_name(name) != "main":
            os.remove(p)


class GlobalPresetStore:
    """User's global presets (shared between projects):
    ~/.ros_studio/presets/*.yaml + the built-in BUILTIN_PRESETS."""

    def __init__(self):
        self.dir = os.path.join(os.path.expanduser("~"), ".ros_studio", "presets")
        os.makedirs(self.dir, exist_ok=True)

    def list_names(self):
        names = list(BUILTIN_PRESETS.keys())
        for f in sorted(os.listdir(self.dir)):
            if f.endswith(".yaml"):
                names.append(os.path.splitext(f)[0])
        return names

    def get(self, name):
        if name in BUILTIN_PRESETS:
            return _merge(DEFAULT_CONFIG, BUILTIN_PRESETS[name])
        p = os.path.join(self.dir, f"{name}.yaml")
        if os.path.exists(p):
            try:
                with open(p, "r", encoding="utf-8") as f:
                    return _merge(DEFAULT_CONFIG, yaml.safe_load(f) or {})
            except Exception:
                pass
        return copy.deepcopy(DEFAULT_CONFIG)

    def save(self, name, cfg):
        safe = re.sub(r'[\\/:*?"<>|]', "_", name).strip()
        if not safe:
            return False
        with open(os.path.join(self.dir, f"{safe}.yaml"), "w", encoding="utf-8") as f:
            yaml.dump(_merge(DEFAULT_CONFIG, cfg), f, sort_keys=False, allow_unicode=True)
        return True
