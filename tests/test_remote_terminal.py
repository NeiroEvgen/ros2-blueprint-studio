"""
Console on the remote target: building the ssh command, checking dangerous values,
choosing the terminal by OS. Nothing is really started (Popen is faked).
"""
import os
import subprocess

import pytest

from core.remote.profile import RemoteProfile
from core.remote.terminal import (
    TerminalError, ssh_argv, open_terminal, validate_target, _windows_command)


def prof(**kw):
    base = dict(host="192.168.1.10", user="robot", port=22)
    base.update(kw)
    return RemoteProfile(**base)


class TestSshArgv:
    def test_plain_host(self):
        a = ssh_argv(prof())
        assert a[0] == "ssh" and "-t" in a
        assert a[-1] == "robot@192.168.1.10"
        assert "-i" not in a

    def test_port_and_key(self):
        a = ssh_argv(prof(port=2222, key_path="/home/me/.ssh/id_ed25519"))
        assert a[a.index("-p") + 1] == "2222"
        assert a[a.index("-i") + 1] == "/home/me/.ssh/id_ed25519"

    def test_container_enters_docker(self):
        a = ssh_argv(prof(container="demo_run"))
        assert a[-2] == "robot@192.168.1.10"
        assert a[-1] == "docker exec -it demo_run bash -l"

    def test_no_password_anywhere(self):
        # the profile has no password field; it cannot be in argv
        assert not any("pass" in x.lower() for x in ssh_argv(prof()))


class TestValidation:
    @pytest.mark.parametrize("host", [
        "", "robot; rm -rf /", "a&calc", "x|y", "h>o", "h o", "$(id)", "-oProxyCommand=x"])
    def test_bad_host(self, host):
        assert validate_target(prof(host=host))
        with pytest.raises(TerminalError):
            ssh_argv(prof(host=host))

    @pytest.mark.parametrize("user", ["", "u;id", "u&v", "u v", "-x", "a|b"])
    def test_bad_user(self, user):
        assert validate_target(prof(user=user))

    @pytest.mark.parametrize("c", ["a;b", "a b", "a&b", "$(x)", "-rm", "a|b"])
    def test_bad_container(self, c):
        assert validate_target(prof(container=c))

    @pytest.mark.parametrize("key", ['C:\\k&calc', 'a"b', "x|y", "k%PATH%", "a>b", "a^b"])
    def test_bad_key_chars(self, key):
        assert validate_target(prof(key_path=key))

    def test_bad_port(self):
        assert validate_target(prof(port=0))
        assert validate_target(prof(port=70000))

    def test_normal_values_ok(self):
        assert validate_target(prof(container="demo_run", key_path="C:\\Users\\me\\id")) == []
        assert validate_target(prof(host="robot.local")) == []


class TestOpenTerminal:
    def test_windows_uses_new_console_and_cmd_k(self):
        calls = []
        shown = open_terminal(
            prof(container="demo_run"),
            popen=lambda cmd, **kw: calls.append((cmd, kw)),
            which=lambda n: "C:\\Windows\\System32\\OpenSSH\\ssh.exe" if n == "ssh" else None,
            os_name="nt")
        cmd, kw = calls[0]
        assert cmd.startswith("cmd.exe /k ssh ")
        assert "docker exec -it demo_run bash -l" in cmd
        assert kw["creationflags"] == getattr(subprocess, "CREATE_NEW_CONSOLE", 0x10)
        assert "robot@192.168.1.10" in shown

    def test_windows_without_ssh_says_how_to_enable(self):
        with pytest.raises(TerminalError, match="OpenSSH"):
            open_terminal(prof(), popen=lambda *a, **k: None,
                          which=lambda n: None, os_name="nt")

    def test_windows_key_path_with_spaces_is_quoted(self):
        cmd = _windows_command(["ssh", "-i", "C:\\my keys\\id", "u@h"])
        assert '"C:\\my keys\\id"' in cmd

    def test_linux_picks_first_available_terminal(self):
        calls = []
        have = {"ssh": "/usr/bin/ssh", "konsole": "/usr/bin/konsole"}
        open_terminal(prof(), popen=lambda cmd, **kw: calls.append(cmd),
                      which=lambda n: have.get(n), os_name="posix")
        assert calls[0][0] == "/usr/bin/konsole"
        assert calls[0][1] == "-e" and calls[0][2] == "ssh"

    def test_linux_no_terminal_gives_manual_command(self):
        with pytest.raises(TerminalError, match="ssh -t"):
            open_terminal(prof(), popen=lambda *a, **k: None,
                          which=lambda n: "/usr/bin/ssh" if n == "ssh" else None,
                          os_name="posix")

    def test_invalid_profile_never_reaches_popen(self):
        called = []
        with pytest.raises(TerminalError):
            open_terminal(prof(host="a&calc"), popen=lambda *a, **k: called.append(1),
                          which=lambda n: "/x", os_name="nt")
        assert not called
