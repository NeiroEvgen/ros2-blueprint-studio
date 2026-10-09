"""Сборка colcon: команда и разбор вывода. Чистая логика, без docker/Qt."""
import re
from dataclasses import dataclass, field
from typing import List, Optional

WS = "/root/ros2_ws"


def build_command(ws: str = WS) -> str:
    """Команда сборки для `bash -c`. Завершается кодом colcon (не маскируется)."""
    return (f"source /opt/ros/humble/setup.bash && cd {ws} && "
            "colcon build --symlink-install --event-handlers console_direct+")


@dataclass
class BuildError:
    message: str
    file: Optional[str] = None
    line: Optional[int] = None
    kind: str = "compile"   # compile | cmake | linker


@dataclass
class BuildResult:
    ok: bool = True
    built: List[str] = field(default_factory=list)
    failed: List[str] = field(default_factory=list)
    errors: List[BuildError] = field(default_factory=list)

    def summary(self, limit: int = 5) -> str:
        if self.ok:
            return "Build OK: " + (", ".join(self.built) or "nothing to build")
        out = ["Build FAILED: " + (", ".join(self.failed) or "unknown package")]
        for e in self.errors[:limit]:
            where = f"{e.file}:{e.line}: " if e.file and e.line else (f"{e.file}: " if e.file else "")
            out.append(f"  [{e.kind}] {where}{e.message}")
        if len(self.errors) > limit:
            out.append(f"  ... and {len(self.errors) - limit} more")
        return "\n".join(out)


_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
_FINISHED = re.compile(r"^Finished\s+<<<\s+(\S+)")
_FAILED = re.compile(r"^Failed\s+<<<\s+(\S+)")
_GCC = re.compile(r"^(?P<file>[^\s:][^:]*?):(?P<line>\d+)(?::\d+)?:\s+(?:fatal )?error:\s+(?P<msg>.*)$")
_CMAKE_AT = re.compile(r"^CMake Error at (?P<file>\S+?):(?P<line>\d+)\s+\((?P<cmd>[^)]*)\):?\s*$")
_CMAKE_PLAIN = re.compile(r"^CMake Error:\s+(?P<msg>.+)$")
_LD = re.compile(r"^(?:\S*ld\S*:|/usr/bin/ld:)\s*(?P<msg>.*(?:undefined reference|cannot find).*)$")


def parse_output(lines) -> BuildResult:
    res = BuildResult()
    pending_cmake: Optional[BuildError] = None
    for raw in lines:
        s = _ANSI.sub("", raw).rstrip()
        t = s.strip()
        if not t:
            continue
        if pending_cmake is not None:
            pending_cmake.message = (pending_cmake.message + " " + t).strip()
            res.errors.append(pending_cmake)
            pending_cmake = None
            continue
        m = _FINISHED.match(t)
        if m:
            res.built.append(m.group(1))
            continue
        m = _FAILED.match(t)
        if m:
            res.failed.append(m.group(1))
            continue
        m = _CMAKE_AT.match(t)
        if m:
            pending_cmake = BuildError(message=m.group("cmd") + ":", file=m.group("file"),
                                       line=int(m.group("line")), kind="cmake")
            continue
        m = _CMAKE_PLAIN.match(t)
        if m:
            res.errors.append(BuildError(message=m.group("msg"), kind="cmake"))
            continue
        m = _GCC.match(t)
        if m:
            res.errors.append(BuildError(message=m.group("msg"), file=m.group("file"),
                                         line=int(m.group("line"))))
            continue
        m = _LD.match(t)
        if m:
            res.errors.append(BuildError(message=m.group("msg"), kind="linker"))
    if pending_cmake is not None:
        res.errors.append(pending_cmake)
    res.ok = not res.failed and not res.errors
    return res
