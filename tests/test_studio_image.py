"""
1.2 — кастомный образ blueprint-studio:humble.

Критерий приёмки: после docker rm -f и нового Run пакеты на месте без
единого apt install. Реальную сборку образа здесь не проверить (нет
Docker-демона) — проверяем всё, что вокруг неё:
- студия выбирает студийный образ, если он собран, и откатывается иначе
- на студийном образе пакеты сессии не доустанавливаются
- всё, что студия ждёт в сессии, реально запечено в Dockerfile.studio
- проектные образы наследуются от студийного через ARG BASE_IMAGE
"""
import os
import re
import types
import pytest

from core.docker_manager import (
    RosContainerManager, pick_base_image, STUDIO_IMAGE, FALLBACK_IMAGE,
)
from core.dockerfile_manager import DockerfileManager
from core.generators.dependency_resolver import DepSet
from tests.conftest import requires_posix_shell

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STUDIO_DOCKERFILE = os.path.join(ROOT, "docker_image", "Dockerfile.studio")


class TestPickBaseImage:

    def test_studio_image_preferred(self):
        assert pick_base_image(lambda t: True) == STUDIO_IMAGE

    def test_fallback_when_not_built(self):
        assert pick_base_image(lambda t: False) == FALLBACK_IMAGE

    def test_fallback_when_check_explodes(self):
        def boom(_):
            raise RuntimeError("docker hiccup")
        assert pick_base_image(boom) == FALLBACK_IMAGE


class TestBakedPackages:

    @pytest.fixture
    def baked(self):
        text = open(STUDIO_DOCKERFILE, encoding="utf-8").read()
        return set(re.findall(r"\b(ros-humble-[a-z0-9-]+|lib[a-z0-9+.-]+-dev|python3-pip)\b",
                              text))

    def test_session_packages_are_baked(self, baked):
        """Иначе на студийном образе apt всё равно полезет при каждой сессии."""
        missing = set(RosContainerManager.SESSION_PACKAGES) - baked
        assert not missing, f"не запечены: {missing}"

    @pytest.mark.parametrize("pkg", [
        "ros-humble-moveit",
        "ros-humble-ros2-control",
        "ros-humble-ros2-controllers",
        "ros-humble-joint-trajectory-controller",
        "ros-humble-joint-state-broadcaster",
        "ros-humble-foxglove-bridge",
        "ros-humble-turtlesim",
    ])
    def test_plan_packages_are_baked(self, baked, pkg):
        assert pkg in baked

    def test_image_has_marker_label(self):
        text = open(STUDIO_DOCKERFILE, encoding="utf-8").read()
        assert 'org.blueprint.studio-image' in text

    def test_bat_is_ascii(self):
        raw = open(os.path.join(ROOT, "docker_image", "build_studio_image.bat"), "rb").read()
        raw.decode("ascii")   # кириллицу cmd.exe превращает в мусор

    @requires_posix_shell
    def test_sh_is_executable(self):
        # На Windows у .sh нет бита исполнения — проверять нечего
        assert os.access(os.path.join(ROOT, "docker_image", "build_studio_image.sh"), os.X_OK)


class FakeImages:
    def __init__(self, present):
        self.present = set(present)
        self.pulled, self.built = [], []

    def get(self, tag):
        import docker
        if tag not in self.present:
            raise docker.errors.ImageNotFound(tag)
        return object()

    def pull(self, tag):
        self.pulled.append(tag)
        self.present.add(tag)

    def build(self, **kw):
        self.built.append(kw)


def _manager(present):
    m = RosContainerManager.__new__(RosContainerManager)
    m.client = types.SimpleNamespace(images=FakeImages(present))
    m.container = None
    m.image_name = pick_base_image(m._image_exists)
    return m


class TestManagerImageFlow:

    def test_manager_picks_studio_when_present(self):
        assert _manager({STUDIO_IMAGE}).image_name == STUDIO_IMAGE

    def test_manager_falls_back(self):
        assert _manager(set()).image_name == FALLBACK_IMAGE

    def test_ensure_image_never_pulls_studio(self):
        m = _manager(set())
        m.image_name = STUDIO_IMAGE          # как будто образ удалили после старта
        m.ensure_image(lambda msg: None)
        assert STUDIO_IMAGE not in m.client.images.pulled
        assert m.image_name == FALLBACK_IMAGE

    def test_ensure_image_hints_when_on_fallback(self):
        m = _manager({FALLBACK_IMAGE})
        msgs = []
        m.ensure_image(msgs.append)
        assert any("build_studio_image" in x for x in msgs)

    def test_project_image_built_from_studio_base(self, tmp_path):
        (tmp_path / "Dockerfile").write_text("FROM x\n")
        m = _manager({STUDIO_IMAGE})
        m.rebuild_project_image(str(tmp_path), lambda msg: None)
        assert m.client.images.built[0]["buildargs"] == {"BASE_IMAGE": STUDIO_IMAGE}

    def test_project_image_without_studio_uses_fallback(self, tmp_path):
        (tmp_path / "Dockerfile").write_text("FROM x\n")
        m = _manager(set())
        m.rebuild_project_image(str(tmp_path), lambda msg: None)
        assert m.client.images.built[0]["buildargs"] == {"BASE_IMAGE": FALLBACK_IMAGE}


class TestProjectDockerfile:

    def _deps(self, apt=()):
        d = DepSet()
        d.apt.update(apt)
        return d

    def test_from_uses_build_arg(self, tmp_path):
        DockerfileManager(str(tmp_path)).ensure_exists()
        text = (tmp_path / "Dockerfile").read_text()
        lines = text.splitlines()
        assert lines[0] == f"ARG BASE_IMAGE={FALLBACK_IMAGE}"
        assert lines[1] == "FROM ${BASE_IMAGE}"

    def test_manual_block_survives_auto_regeneration(self, tmp_path):
        """Существующее поведение: смена графа не трогает библиотеки из Library."""
        dm = DockerfileManager(str(tmp_path))
        dm.ensure_exists()
        dm.add_manual_library("apt", "libyaml-cpp-dev")
        dm.add_manual_library("pip", "numpy")

        dm.write_auto_block(self._deps(apt={"ros-humble-moveit"}))
        dm.write_auto_block(self._deps(apt={"ros-humble-tf2-ros"}))

        assert dm.read_manual_libraries() == [("apt", "libyaml-cpp-dev"), ("pip", "numpy")]

    def test_arg_line_not_parsed_as_package(self, tmp_path):
        dm = DockerfileManager(str(tmp_path))
        dm.write_auto_block(self._deps(apt={"ros-humble-moveit"}))
        apt, _ = dm.read_auto_summary()
        assert apt == ["ros-humble-moveit"]
