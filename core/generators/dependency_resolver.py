"""
Dependency resolver for ROS 2 Blueprint Studio.

Self-contained. Scans node source code for #include / import statements,
resolves them to ROS 2 package dependencies via a JSON registry, aggregates
the result across the whole graph.

Generators (Dockerfile / CMakeLists / package.xml) consume the aggregated
DepSet — they no longer need to know individual libraries by name.

"к какой либе обращаемся — то и тянем": deps come from what the code actually
uses. Unknown includes go to .unknown so the UI can surface them instead of
silently producing a broken build. The registry supports a 'local' flag,
reserved for future in-workspace custom .msg packages.
"""

import os
import re
import json
import logging

_REGISTRY_PATH = os.path.join(os.path.dirname(__file__), "dependency_registry.json")

_CPP_STDLIB = {
    "iostream", "vector", "string", "memory", "cmath", "math.h", "algorithm",
    "chrono", "functional", "map", "set", "array", "thread", "mutex", "cstdio",
    "cstdlib", "cstring", "sstream", "fstream", "utility", "tuple", "optional",
    "cstdint", "stdexcept", "type_traits", "limits", "numeric", "random",
}
_PY_STDLIB = {
    "os", "sys", "re", "json", "math", "time", "socket", "struct", "threading",
    "collections", "functools", "itertools", "typing", "datetime", "random",
    "subprocess", "pathlib", "logging", "enum", "abc", "copy",
}


class DepSet:
    def __init__(self):
        self.apt = set()
        self.pip = set()
        self.cmake_find = set()
        self.cmake_link = set()
        self.xml_depend = set()
        self.local = set()
        self.unknown = set()

    def merge_entry(self, entry):
        self.apt.update(entry.get("apt", []))
        self.pip.update(entry.get("pip", []))
        self.cmake_find.update(entry.get("cmake_find", []))
        self.cmake_link.update(entry.get("cmake_link", []))
        self.xml_depend.update(entry.get("xml_depend", []))
        if entry.get("local"):
            for name in entry.get("xml_depend", []):
                self.local.add(name)

    def as_dict(self):
        return {
            "apt": sorted(self.apt),
            "pip": sorted(self.pip),
            "cmake_find": sorted(self.cmake_find),
            "cmake_link": sorted(self.cmake_link),
            "xml_depend": sorted(self.xml_depend),
            "local": sorted(self.local),
            "unknown": sorted(self.unknown),
        }


class DependencyResolver:
    def __init__(self, registry_path=_REGISTRY_PATH):
        self.logger = logging.getLogger("[INFO] [DEPS]")
        self.registry = {}
        try:
            with open(registry_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.registry = {k: v for k, v in data.items() if not k.startswith("_")}
        except Exception as e:
            self.logger.warning(f"Could not load dependency registry: {e}")

    @staticmethod
    def scan_includes(code, language):
        if not code:
            return []
        if language == "cpp":
            raw = re.findall(r'#include\s*[<"]([^>"]+)[>"]', code)
            out = []
            for inc in raw:
                if inc in _CPP_STDLIB or os.path.basename(inc) in _CPP_STDLIB:
                    continue
                out.append(inc)
            return out
        else:
            raw = re.findall(r'^\s*(?:from|import)\s+([\w.]+)', code, re.M)
            return [r for r in raw if r.split('.')[0] not in _PY_STDLIB]

    def resolve_include(self, include):
        head = include.split('/')[0].split('.')[0]
        if head in self.registry:
            return self.registry[head]
        for key, entry in self.registry.items():
            if include.startswith(key):
                return entry
        return None

    def resolve_code(self, code, language, dep_set):
        for inc in self.scan_includes(code, language):
            entry = self.resolve_include(inc)
            if entry:
                dep_set.merge_entry(entry)
            else:
                dep_set.unknown.add(inc)

    def collect_from_nodes(self, flat_nodes, language):
        deps = DepSet()
        for entry in flat_nodes:
            node = entry.get('node', entry)
            code = node.get('custom', {}).get('code_content', '')
            self.resolve_code(code, language, deps)
        if deps.unknown:
            self.logger.info(f"Unresolved includes (add to registry?): {sorted(deps.unknown)}")
        return deps
