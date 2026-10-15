"""Повторный Run переиспользует контейнер того же проекта (в том числе остановленный) — зависимости не теряются."""
import os
import types

import docker
import pytest

from core.docker_manager import RosContainerManager, PROJECT_LABEL


class Box:
    def __init__(self, status, image, label):
        self.status = status
        self.attrs = {"Config": {"Image": image}}
        self.labels = {PROJECT_LABEL: label} if label else {}
        self.started = self.removed = False
        self.created_kwargs = None

    def reload(self): pass
    def start(self): self.started = True; self.status = "running"
    def remove(self, force=False): self.removed = True

    def exec_run(self, cmd, **kw):
        return types.SimpleNamespace(output=b"", exit_code=0)


class Client:
    def __init__(self, existing=None):
        self.existing, self.created = existing, None
        self.images = types.SimpleNamespace(get=lambda tag: (_ for _ in ()).throw(docker.errors.ImageNotFound("x")))
        self.containers = types.SimpleNamespace(get=self._get, run=self._run)

    def _get(self, name):
        if self.existing is None:
            raise docker.errors.NotFound("none")
        return self.existing

    def _run(self, image, **kw):
        self.created = (image, kw)
        return Box("running", image, kw["labels"][PROJECT_LABEL])


def start(tmp_path, existing, **kw):
    m = RosContainerManager.__new__(RosContainerManager)
    m.client, m.image_name, m.container_name = Client(existing), "img:1", "sess"
    m.extra_env = {}
    proj = tmp_path / "p"; (proj / "src").mkdir(parents=True, exist_ok=True)
    src = os.path.abspath(proj / "src")
    return m, str(proj), src


def test_stopped_container_of_same_project_is_restarted(tmp_path):
    m, proj, src = start(tmp_path, None)
    box = Box("exited", "img:1", os.path.abspath(os.path.join(proj, "src")))
    m.client.existing = box
    m.start_session(proj)
    assert box.started and not box.removed and m.container is box and m.client.created is None


def test_running_container_of_same_project_is_reused(tmp_path):
    m, proj, src = start(tmp_path, None)
    box = Box("running", "img:1", src); m.client.existing = box
    m.start_session(proj)
    assert m.container is box and not box.removed and not box.started


def test_other_project_container_is_replaced(tmp_path):
    m, proj, src = start(tmp_path, None)
    box = Box("running", "img:1", "/somewhere/else/src"); m.client.existing = box
    m.start_session(proj)
    assert box.removed and m.client.created is not None


def test_legacy_unlabeled_running_reused_but_stopped_replaced(tmp_path):
    m, proj, _ = start(tmp_path, None)
    live = Box("running", "img:1", None); m.client.existing = live
    m.start_session(proj)
    assert m.container is live and not live.removed
    m2, proj2, _ = start(tmp_path, None)
    dead = Box("exited", "img:1", None); m2.client.existing = dead
    m2.start_session(proj2)
    assert dead.removed and m2.client.created is not None


def test_different_image_or_force_recreate_replaces(tmp_path):
    m, proj, src = start(tmp_path, None)
    box = Box("exited", "other:2", src); m.client.existing = box
    m.start_session(proj)
    assert box.removed
    m2, proj2, src2 = start(tmp_path, None)
    box2 = Box("exited", "img:1", src2); m2.client.existing = box2
    m2.start_session(proj2, force_recreate=True)
    assert box2.removed and not box2.started


def test_new_container_is_labeled_with_project(tmp_path):
    m, proj, src = start(tmp_path, None)
    m.start_session(proj)
    assert m.client.created[1]["labels"] == {PROJECT_LABEL: src}
