"""
Interactive console on the remote target: a real `ssh -t` in a separate OS
terminal window. Studio has no built-in terminal emulator and does not need one:
the system ssh asks for the password itself, knows ssh-agent and behaves like
a normal console (Ctrl+C, vim, tab completion).

Studio does NOT pass the password or key passphrase here: they are typed in the ssh window.
The system ssh has its own known_hosts (not ~/.blueprint/known_hosts).

host/user/container values go into the command line, so they are
strictly validated: on Windows the line goes to cmd.exe, and characters
like & | > ^ would turn into running someone else's commands.
"""
import os
import re
import shutil
import subprocess

from core.i18n import t

_HOST_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]*$")      # name, IPv4, simple IPv6
_USER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9._-]*$")
_CONTAINER_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")   # docker naming rule
_BAD_PATH_CHARS = set('&|<>^"%\r\n\x00')

# terminal lookup order on Linux
_LINUX_TERMINALS = [
    ("x-terminal-emulator", ["-e"]),
    ("gnome-terminal", ["--"]),
    ("konsole", ["-e"]),
    ("xfce4-terminal", ["-x"]),
    ("xterm", ["-e"]),
]


class TerminalError(RuntimeError):
    pass


def validate_target(profile):
    """Returns a list of errors; empty means it can be opened."""
    errors = []
    if not profile.host or not _HOST_RE.match(profile.host):
        errors.append(t("rp.err_host"))
    if not profile.user or not _USER_RE.match(profile.user):
        errors.append(t("rp.err_user"))
    try:
        if not 0 < int(profile.port) < 65536:
            errors.append(t("rp.err_port"))
    except (TypeError, ValueError):
        errors.append(t("rp.err_port"))
    if profile.container and not _CONTAINER_RE.match(profile.container):
        errors.append(t("rp.err_container"))
    key = (profile.key_path or "").strip()
    if key and any(c in _BAD_PATH_CHARS for c in key):
        errors.append(t("rp.err_key_chars"))
    return errors


def ssh_argv(profile):
    """argv for the system ssh. With a container, go straight into it."""
    errors = validate_target(profile)
    if errors:
        raise TerminalError("; ".join(errors))
    argv = ["ssh", "-t", "-p", str(int(profile.port)),
            "-o", "ServerAliveInterval=30"]
    key = (profile.key_path or "").strip()
    if key:
        argv += ["-i", key]
    argv.append(f"{profile.user}@{profile.host}")
    if profile.container:
        # the name was validated by the regex above, no quoting needed
        argv.append(f"docker exec -it {profile.container} bash -l")
    return argv


def _windows_command(argv):
    """cmd /k keeps the window open after ssh exits, so the error text stays visible."""
    return "cmd.exe /k " + subprocess.list2cmdline(argv)


def open_terminal(profile, popen=subprocess.Popen, which=shutil.which, os_name=None):
    """
    Opens a new terminal window with ssh to the target from the profile.
    Returns the command string to show in the log. popen/which/os_name
    are replaced in tests.
    """
    argv = ssh_argv(profile)
    os_name = os_name or os.name
    shown = subprocess.list2cmdline(argv)

    if os_name == "nt":
        if not which("ssh"):
            raise TerminalError(t("term.no_ssh_windows"))
        flags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0x00000010)
        popen(_windows_command(argv), creationflags=flags)
        return shown

    if not which("ssh"):
        raise TerminalError(t("term.no_ssh"))
    for name, opt in _LINUX_TERMINALS:
        exe = which(name)
        if exe:
            popen([exe] + opt + argv)
            return shown
    raise TerminalError(t("term.no_emulator", command=shown))
