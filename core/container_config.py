# core/container_config.py
# v0.6.0 — Модель конфигурации деплой-контейнера + хранилища пресетов.
# Конфиг = ЧТО внутри контейнера. Таргет = КУДА деплоим. Один YAML-формат
# служит и деплой-конфигом, и основой системы манифестов (п.7 плана).

import os
import re
import yaml
import copy

DEFAULT_IMAGE = "osrf/ros:humble-desktop"

DEFAULT_CONFIG = {
    "name": "main",
    "image": DEFAULT_IMAGE,
    # Глобальные ROS env (раздел 1.1 roadmap)
    "env": {
        "ROS_DOMAIN_ID": "0",
        "PYTHONUNBUFFERED": "1",
        "RCUTILS_LOGGING_BUFFERED_STREAM": "0",
    },
    # DDS (раздел 1.2): vendor -> RMW_IMPLEMENTATION, qos_xml -> монтируемый профиль
    "dds": {
        "vendor": "default",        # default | fastrtps | cyclonedds
        "qos_xml": "",              # путь к XML-профилю вендора (опционально)
    },
    # Симуляция помех через tc netem (раздел 1.3). Требует NET_ADMIN.
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
    "privileged": True,             # текущее поведение по умолчанию
    "network_mode": "host",         # host | bridge
    "resources": {"cpus": "", "memory": ""},   # пусто = без лимита
    # Таргет деплоя (раздел 4): local или ssh://user@host
    "target": {"type": "local", "host": ""},
}

RMW_BY_VENDOR = {
    "default": None,
    "fastrtps": "rmw_fastrtps_cpp",
    "cyclonedds": "rmw_cyclonedds_cpp",
}

# Пресеты из коробки (раздел 3.2)
BUILTIN_PRESETS = {
    "Local Default": {},
    "GPU Node": {
        "image": "osrf/ros:humble-desktop",  # замени на свой CUDA-образ
        "resources": {"cpus": "", "memory": ""},
        "env": {"NVIDIA_VISIBLE_DEVICES": "all"},
    },
    "Critical Service": {
        "restart": "always",
        "network_mode": "host",
    },
    "Sandbox (заготовка)": {
        # ВНИМАНИЕ: это НЕ песочница безопасности, см. roadmap раздел 6.
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
    """Итоговый словарь env для контейнера с учётом DDS-вендора."""
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
    """Команда tc netem или None, если симуляция выключена."""
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
    """Конфиги контейнер-групп текущего проекта: <project>/.blueprint/containers/*.yaml"""

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
    """Глобальные пресеты пользователя (переиспользуются между проектами):
    ~/.ros_studio/presets/*.yaml + встроенные BUILTIN_PRESETS."""

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
