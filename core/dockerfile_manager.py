"""
Dockerfile manager for ROS 2 Blueprint Studio.

Self-contained. Owns the project's Dockerfile as the single source of truth
for its runtime environment — replacing the old behaviour of installing
packages into a live container that evaporates on every restart.

Structure of a managed Dockerfile:

    FROM <base_image>

    # === AUTO: dependencies from graph nodes (regenerated on save) ===
    RUN apt-get update && apt-get install -y \
        <pkg> <pkg> \
        && rm -rf /var/lib/apt/lists/*
    RUN pip install <pkg> <pkg>
    # === END AUTO ===

    # === MANUAL: user libraries (Library tab) ===
    RUN apt-get install -y <pkg>
    RUN pip install <pkg>
    # === END MANUAL ===

The AUTO block is fully owned by the dependency resolver (core/generators/
dependency_resolver.py) and is regenerated on every save_project — hand edits
inside it will be overwritten. The MANUAL block is fully owned by the Library
tab (via add_manual_library / remove_manual_library) and is never touched by
the auto-generation step. This split lets graph-driven deps and user-added
libraries coexist without clobbering each other.
"""

import os
import re

BASE_IMAGE_DEFAULT = "osrf/ros:humble-desktop"

_AUTO_START = "# === AUTO: dependencies from graph nodes (regenerated on save) ==="
_AUTO_END = "# === END AUTO ==="
_MANUAL_START = "# === MANUAL: user libraries (Library tab) ==="
_MANUAL_END = "# === END MANUAL ==="


def _apt_block(packages):
    if not packages:
        return ""
    pkgs = " \\\n        ".join(sorted(packages))
    return (f"RUN apt-get update && apt-get install -y \\\n        {pkgs} \\\n"
            f"    && rm -rf /var/lib/apt/lists/*\n")


def _pip_block(packages):
    if not packages:
        return ""
    return f"RUN pip install {' '.join(sorted(packages))}\n"


class DockerfileManager:
    def __init__(self, project_path: str, base_image: str = BASE_IMAGE_DEFAULT):
        self.project_path = project_path
        self.base_image = base_image
        self.path = os.path.join(project_path, "Dockerfile")

    # ---- reading ------------------------------------------------------

    def _read_raw(self):
        if not os.path.exists(self.path):
            return None
        with open(self.path, "r", encoding="utf-8") as f:
            return f.read()

    def _extract_section(self, raw, start_marker, end_marker):
        if raw is None:
            return ""
        m = re.search(re.escape(start_marker) + r"\n(.*?)\n" + re.escape(end_marker),
                      raw, re.S)
        return m.group(1) if m else ""

    def read_manual_libraries(self):
        """
        Returns list of (kind, package) tuples currently in the MANUAL block,
        e.g. [("apt", "libopencv-dev"), ("pip", "numpy")].
        Used by the Library tab to render its list.
        """
        raw = self._read_raw()
        section = self._extract_section(raw, _MANUAL_START, _MANUAL_END)
        libs = []
        for line in section.splitlines():
            line = line.strip()
            m_apt = re.match(r"RUN apt-get install -y (.+)", line)
            m_pip = re.match(r"RUN pip install (.+)", line)
            if m_apt:
                for pkg in m_apt.group(1).split():
                    libs.append(("apt", pkg))
            elif m_pip:
                for pkg in m_pip.group(1).split():
                    libs.append(("pip", pkg))
        return libs

    def read_auto_summary(self):
        """Returns (apt_packages, pip_packages) currently baked into the AUTO block."""
        raw = self._read_raw()
        section = self._extract_section(raw, _AUTO_START, _AUTO_END)
        apt, pip = [], []
        for line in section.splitlines():
            for pkg in re.findall(r"^\s*([a-zA-Z0-9][\w.+-]*)\s*\\?\s*$", line):
                if pkg not in ("apt-get", "install", "-y", "update", "&&", "rm", "-rf"):
                    apt.append(pkg)
            m_pip = re.match(r"\s*RUN pip install (.+)", line)
            if m_pip:
                pip.extend(m_pip.group(1).split())
        return apt, pip

    # ---- writing --------------------------------------------------------

    def _compose(self, auto_apt, auto_pip, manual_lines):
        parts = [f"FROM {self.base_image}", ""]
        parts.append(_AUTO_START)
        auto_body = (_apt_block(auto_apt) + _pip_block(auto_pip)).rstrip("\n")
        if auto_body:
            parts.append(auto_body)
        parts.append(_AUTO_END)
        parts.append("")
        parts.append(_MANUAL_START)
        if manual_lines:
            parts.append("\n".join(manual_lines))
        parts.append(_MANUAL_END)
        parts.append("")
        return "\n".join(parts)

    def _manual_lines_from_libs(self, libs):
        lines = []
        for kind, pkg in libs:
            if kind == "apt":
                lines.append(f"RUN apt-get install -y {pkg}")
            else:
                lines.append(f"RUN pip install {pkg}")
        return lines

    def write_auto_block(self, dep_set):
        """
        Regenerates the AUTO block from a resolved DepSet (see
        core/generators/dependency_resolver.py). Preserves the MANUAL block
        untouched. Called from save_project after dependency resolution.
        Returns True if the Dockerfile content actually changed (caller can
        use this to decide whether a rebuild is needed).
        """
        manual_libs = self.read_manual_libraries()
        manual_lines = self._manual_lines_from_libs(manual_libs)
        new_content = self._compose(dep_set.apt, dep_set.pip, manual_lines)

        old_content = self._read_raw()
        if old_content == new_content:
            return False

        with open(self.path, "w", encoding="utf-8") as f:
            f.write(new_content)
        return True

    def add_manual_library(self, kind: str, package: str):
        """kind: 'apt' or 'pip'. Called from the Library tab's Install button."""
        auto_apt, auto_pip = self.read_auto_summary()
        libs = self.read_manual_libraries()
        if (kind, package) not in libs:
            libs.append((kind, package))
        manual_lines = self._manual_lines_from_libs(libs)
        new_content = self._compose(auto_apt, auto_pip, manual_lines)
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(new_content)

    def remove_manual_library(self, kind: str, package: str):
        auto_apt, auto_pip = self.read_auto_summary()
        libs = [l for l in self.read_manual_libraries() if l != (kind, package)]
        manual_lines = self._manual_lines_from_libs(libs)
        new_content = self._compose(auto_apt, auto_pip, manual_lines)
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(new_content)

    def ensure_exists(self):
        """Creates a minimal Dockerfile if the project doesn't have one yet."""
        if not os.path.exists(self.path):
            with open(self.path, "w", encoding="utf-8") as f:
                f.write(self._compose(set(), set(), []))
