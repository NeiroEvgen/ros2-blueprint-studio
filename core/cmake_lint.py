"""Lightweight CMakeLists linter: catches typical ament project breakages before colcon."""
import re
from dataclasses import dataclass
from typing import List

_KW = {"PUBLIC", "PRIVATE", "INTERFACE"}


@dataclass
class LintIssue:
    line: int
    target: str
    code: str            # localization key: lint.<code>

    @property
    def message(self) -> str:
        from core.i18n import t
        return t(f"lint.{self.code}", target=self.target, line=self.line)


def _calls(text: str, name: str):
    """Yield (line_no, [args]) for calls name(...), comments stripped."""
    clean = re.sub(r"#[^\n]*", lambda m: " " * len(m.group(0)), text)
    for m in re.finditer(rf"(?<![\w]){name}\s*\(([^()]*)\)", clean, re.I | re.S):
        yield clean.count("\n", 0, m.start()) + 1, m.group(1).split()


def lint(cmake_text: str) -> List[LintIssue]:
    issues: List[LintIssue] = []
    ament = {a[0] for _, a in _calls(cmake_text, "ament_target_dependencies") if a}
    for ln, args in _calls(cmake_text, "target_link_libraries"):
        if args and args[0] in ament and any(a in _KW for a in args[1:]):
            issues.append(LintIssue(ln, args[0], "mixed_link_signature"))
    return issues
