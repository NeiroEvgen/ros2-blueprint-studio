import os
import pytest
from core.workspace_manager import WorkspaceManager
from core.layout import ProjectLayout


@pytest.fixture
def ws(tmp_path, monkeypatch):
    monkeypatch.setattr(WorkspaceManager, "get_workspace_root", staticmethod(lambda: str(tmp_path)))
    return tmp_path


@pytest.mark.parametrize("ptype", ["cpp", "python"])
def test_new_project_has_clean_layout(ws, ptype):
    path = WorkspaceManager.create_project("demo", ptype)
    top = set(os.listdir(path))
    assert {"src", "docker", ".blueprint", "project_config.json"} <= top
    # никакого ROS-мусора в корне
    assert not top & {"launch", "CMakeLists.txt", "package.xml", "cpp", "include"}
    assert os.path.isdir(os.path.join(path, "src", "launch"))
    assert not ProjectLayout(path).is_legacy()


def test_duplicate_project_refused(ws):
    WorkspaceManager.create_project("demo", "cpp")
    with pytest.raises(FileExistsError):
        WorkspaceManager.create_project("demo", "cpp")


def test_docker_template_still_created(ws):
    path = WorkspaceManager.create_project("demo", "cpp")
    assert os.path.isfile(os.path.join(path, "docker", "docker-compose.yaml"))
