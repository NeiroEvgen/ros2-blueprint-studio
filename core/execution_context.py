"""
Where Studio commands are executed right now.

The most common mistake when working with a robot is losing track of where you are:
on your own machine, on a remote host or inside a container. Commands go to
the wrong place: `docker` inside a container, a Windows path in scp from an SSH session,
`ros2 launch` in the wrong environment. The only hint used to be the shell
prompt, and it is easy to miss.

The model is pure (no Qt) so it can be tested and reused
by the remote workspace.
"""
from dataclasses import dataclass
from enum import Enum

from core.i18n import t


class ContextKind(Enum):
    LOCAL = "local"          # no session, everything on the Studio machine
    SSH = "ssh"              # connected to a remote Docker host, no session
    CONTAINER = "container"  # a live session in a container


@dataclass(frozen=True)
class ExecutionContext:
    kind: ContextKind = ContextKind.LOCAL
    host: str = ""            # "" = this machine; otherwise user@host
    container: str = ""       # session container name
    real_robot: bool = False  # the target is real hardware, not a simulation

    # ---- presentation ------------------------------------------------

    @property
    def where(self):
        return self.host or "local"

    def label(self):
        """Full label for the badge and the window title."""
        if self.kind is ContextKind.CONTAINER:
            base = f"CONTAINER: {self.container} @ {self.where}"
        elif self.kind is ContextKind.SSH:
            base = f"SSH: {self.host}"
        else:
            base = "LOCAL"
        return f"⚠ REAL ROBOT · {base}" if self.real_robot else base

    def short(self):
        """Short label for every System Log line."""
        if self.kind is ContextKind.CONTAINER:
            base = f"{self.container}@{_short_host(self.where)}"
        elif self.kind is ContextKind.SSH:
            base = f"ssh:{_short_host(self.host)}"
        else:
            base = "local"
        return f"[⚠ROBOT {base}]" if self.real_robot else f"[{base}]"

    def describe(self):
        """Detailed explanation for the tooltip."""
        if self.kind is ContextKind.CONTAINER:
            where = t("ctx.where_here") if not self.host else t("ctx.where_remote", host=self.host)
            text = t("ctx.in_container", container=self.container, where=where)
        elif self.kind is ContextKind.SSH:
            text = t("ctx.ssh_no_session", host=self.host)
        else:
            text = t("ctx.local")
        if self.real_robot:
            text = t("ctx.real_robot") + "\n\n" + text
        return text

    def palette(self):
        """(background, text) of the badge. The colour duplicates the text, it does not replace it."""
        if self.real_robot:
            return "#b71c1c", "#ffffff"
        return {
            ContextKind.LOCAL: ("#424242", "#e0e0e0"),
            ContextKind.SSH: ("#1565c0", "#ffffff"),
            ContextKind.CONTAINER: ("#2e7d32", "#ffffff"),
        }[self.kind]


def _short_host(host):
    """user@192.168.1.10 -> .10 ; local -> local ; a host name as is."""
    if not host or host == "local":
        return "local"
    h = host.split("@", 1)[-1]
    parts = h.split(".")
    if len(parts) == 4 and all(p.isdigit() for p in parts):
        return "." + parts[-1]
    return h


def host_from_target(target):
    """
    target from ContainerConfigStore: {"type": "local"|"ssh", "host": "ssh://user@h"}.
    Returns "user@h" for ssh and "" for local.
    """
    if not target or target.get("type") != "ssh":
        return ""
    h = (target.get("host") or "").strip()
    return h[len("ssh://"):] if h.startswith("ssh://") else h


def resolve_context(target=None, session_active=False, container_name="",
                    real_robot=False):
    """The single place where the context is computed from Studio state."""
    host = host_from_target(target)
    if session_active:
        return ExecutionContext(ContextKind.CONTAINER, host, container_name, real_robot)
    if host:
        return ExecutionContext(ContextKind.SSH, host, "", real_robot)
    return ExecutionContext(ContextKind.LOCAL, "", "", real_robot)
