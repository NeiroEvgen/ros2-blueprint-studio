"""
2.1 — индикация контекста.

Критерии приёмки:
- смена профиля/состояния меняет метку
- в логе видно, где выполнена команда (короткая метка на каждой строке)
- режим «реальный робот» отличается однозначно: цвет + ТЕКСТ, не только цвет
"""
import pytest

from core.execution_context import (
    ContextKind, ExecutionContext, resolve_context, host_from_target,
)

SSH_TARGET = {"type": "ssh", "host": "ssh://unitree@192.168.123.164"}
LOCAL_TARGET = {"type": "local", "host": ""}


class TestResolve:

    def test_no_session_local(self):
        ctx = resolve_context(LOCAL_TARGET, session_active=False)
        assert ctx.kind is ContextKind.LOCAL
        assert ctx.label() == "LOCAL"

    def test_no_session_remote_target_is_ssh(self):
        ctx = resolve_context(SSH_TARGET, session_active=False)
        assert ctx.kind is ContextKind.SSH
        assert ctx.label() == "SSH: unitree@192.168.123.164"

    def test_local_session_is_container_at_local(self):
        ctx = resolve_context(LOCAL_TARGET, True, "ros2_orchestrator_session")
        assert ctx.label() == "CONTAINER: ros2_orchestrator_session @ local"

    def test_remote_session(self):
        ctx = resolve_context(SSH_TARGET, True, "g1pilot_run")
        assert ctx.label() == "CONTAINER: g1pilot_run @ unitree@192.168.123.164"

    def test_missing_target_defaults_to_local(self):
        assert resolve_context(None).kind is ContextKind.LOCAL

    def test_state_change_changes_label(self):
        """Критерий: переключение меняет метку."""
        labels = {
            resolve_context(LOCAL_TARGET, False).label(),
            resolve_context(SSH_TARGET, False).label(),
            resolve_context(LOCAL_TARGET, True, "c").label(),
            resolve_context(SSH_TARGET, True, "c").label(),
        }
        assert len(labels) == 4


class TestShortLogPrefix:

    @pytest.mark.parametrize("ctx,expected", [
        (resolve_context(LOCAL_TARGET), "[local]"),
        (resolve_context(SSH_TARGET), "[ssh:.164]"),
        (resolve_context(LOCAL_TARGET, True, "ros2_orchestrator_session"),
         "[ros2_orchestrator_session@local]"),
        (resolve_context(SSH_TARGET, True, "g1pilot_run"), "[g1pilot_run@.164]"),
    ])
    def test_short(self, ctx, expected):
        assert ctx.short() == expected

    def test_hostname_kept_when_not_ip(self):
        ctx = resolve_context({"type": "ssh", "host": "ssh://me@jetson.lab"}, True, "c")
        assert ctx.short() == "[c@jetson.lab]"


class TestRealRobot:

    @pytest.mark.parametrize("kind_args", [
        (LOCAL_TARGET, False, ""),
        (SSH_TARGET, False, ""),
        (SSH_TARGET, True, "g1pilot_run"),
    ])
    def test_real_robot_marked_in_text_not_only_color(self, kind_args):
        """Критерий: не только цвет — дальтоник и скриншот в ч/б тоже поймут."""
        target, active, name = kind_args
        ctx = resolve_context(target, active, name, real_robot=True)
        assert "REAL ROBOT" in ctx.label()
        assert "ROBOT" in ctx.short()
        assert "РЕАЛЬНЫЙ РОБОТ" in ctx.describe()

    def test_real_robot_color_differs_from_every_normal_state(self):
        robot_bg = resolve_context(SSH_TARGET, True, "c", real_robot=True).palette()[0]
        for ctx in (resolve_context(LOCAL_TARGET), resolve_context(SSH_TARGET),
                    resolve_context(LOCAL_TARGET, True, "c")):
            assert ctx.palette()[0] != robot_bg

    def test_normal_states_not_marked_as_robot(self):
        for ctx in (resolve_context(LOCAL_TARGET), resolve_context(SSH_TARGET, True, "c")):
            assert "ROBOT" not in ctx.label()
            assert "ROBOT" not in ctx.short()


class TestDescribe:

    def test_container_warns_about_host_tools(self):
        d = resolve_context(SSH_TARGET, True, "g1pilot_run").describe()
        assert "g1pilot_run" in d and "docker" in d

    def test_frozen(self):
        ctx = ExecutionContext()
        with pytest.raises(Exception):
            ctx.real_robot = True


class TestHostFromTarget:

    @pytest.mark.parametrize("target,expected", [
        ({"type": "ssh", "host": "ssh://a@1.2.3.4"}, "a@1.2.3.4"),
        ({"type": "ssh", "host": "a@1.2.3.4"}, "a@1.2.3.4"),
        ({"type": "ssh", "host": "  ssh://a@h  "}, "a@h"),
        ({"type": "local", "host": "ssh://ignored@h"}, ""),
        ({}, ""),
        (None, ""),
    ])
    def test_parse(self, target, expected):
        assert host_from_target(target) == expected
