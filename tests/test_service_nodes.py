"""
ServiceServer / ServiceClient palette nodes (AddTwoInts, as in the official service tutorial).
Templates, dependencies, palette, and the wire keeping the service name in step. Qt offscreen.
The generated projects were also built and run against ROS 2 Humble for docs/tutorials chapter 4
(Python and C++); the e2e test below repeats that when the image with example_interfaces exists.
"""
import ast
import os
import shutil
import subprocess
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")

from core.template_render import render_pristine_template  # noqa: E402
from nodes.templates import CPP_TEMPLATES, PYTHON_TEMPLATES, template_name  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIELDS = {"node_name": "srv", "service_name": "/add_two_ints", "a": "2", "b": "3", "interval": "1.0"}


@pytest.mark.parametrize("key, name", [("py_service_server", "ServiceServer"), ("cpp_service_client", "ServiceClient")])
def test_template_names(key, name):
    assert template_name(key) == name


@pytest.mark.parametrize("which", ["ServiceServer", "ServiceClient"])
def test_python_templates_render_to_valid_code(which):
    key = "py_service_server" if which == "ServiceServer" else "py_service_client"
    code = render_pristine_template(PYTHON_TEMPLATES[which], {"template_key": key, **FIELDS}, "srv", False)
    ast.parse(code)
    assert "{" + "service_name}" not in code and "'/add_two_ints'" in code
    assert "from example_interfaces.srv import AddTwoInts" in code


def test_python_client_never_blocks_its_executor():
    code = PYTHON_TEMPLATES["ServiceClient"]
    assert "call_async" in code and "add_done_callback" in code
    assert "spin_until_future_complete" not in code and ".result()" not in code.split("add_done_callback")[0]


def test_cpp_client_answer_in_callback():
    code = render_pristine_template(CPP_TEMPLATES["ServiceClient"], {"template_key": "cpp_service_client", **FIELDS},
                                    "client", True)
    assert "async_send_request(request," in code and "request->a = 2;" in code
    assert "std::chrono::duration<double>(1.0)" in code and "{a}" not in code


def test_dependency_comes_from_the_include():
    from core.generators.dependency_resolver import DependencyResolver, DepSet
    deps = DepSet()
    DependencyResolver().resolve_code(CPP_TEMPLATES["ServiceServer"], "cpp", deps)
    assert "ros-humble-example-interfaces" in deps.apt and "example_interfaces" not in deps.unknown


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def win(app, tmp_path):
    from ui import main_window as mw
    p = tmp_path / "proj"
    shutil.copytree(os.path.join(ROOT, "examples", "py_pubsub"), p)
    w = mw.RosVisualRunner()
    w.on_load_project_init(str(p))
    logs = []
    w.system_log = logs.append
    w._logs = logs
    yield w
    w._mark_clean()
    w.close()


def test_palette_has_a_services_group(win):
    items = [win.ui.node_palette.item(i).text() for i in range(win.ui.node_palette.count())]
    i = items.index("--- Services ---")
    assert items[i + 1:i + 3] == ["ServiceServer", "ServiceClient"]


def test_wire_gives_client_the_server_name(win):
    g = win.graphs.all()[0]
    srv = g.create_node("ros.py.PyServiceServerNode", name="server")
    cli = g.create_node("ros.py.PyServiceClientNode", name="client")
    srv.set_property("service_name", "/sum")
    cli.outputs()["call"].connect_to(srv.inputs()["srv"])
    assert cli.get_property("service_name") == "/sum"
    srv.set_property("service_name", "/sum2")                 # the server owns the name
    assert cli.get_property("service_name") == "/sum2"
    cli.set_property("service_name", "/typo")                # a client edit does not push back
    assert srv.get_property("service_name") == "/sum2"
    assert ("service_name", "server", "client", "/sum2", "/typo") in win._topic_mismatches()


def test_server_and_client_look_different(win):
    g = win.graphs.all()[0]
    srv = g.create_node("ros.cpp.CppServiceServerNode", name="s")
    cli = g.create_node("ros.cpp.CppServiceClientNode", name="c")
    assert srv.color() != cli.color()


# ---------- generated project against ROS 2 Humble ----------

SRV_IMAGE = "studio-test:humble-srv"     # ros:humble-ros-base + example_interfaces (not in the base image)


def _image_ready():
    if shutil.which("docker") is None:
        return False
    try:
        return subprocess.run(["docker", "image", "inspect", SRV_IMAGE], capture_output=True,
                              timeout=15).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def test_generated_python_service_pair_talks(win, tmp_path):
    if not _image_ready():
        pytest.skip(f"Docker image {SRV_IMAGE} (ROS 2 Humble with example_interfaces) is not available")
    g = win.graphs.all()[0]
    for n in list(g.all_nodes()):
        g.delete_node(n)
    srv = g.create_node("ros.py.PyServiceServerNode", name="server")
    cli = g.create_node("ros.py.PyServiceClientNode", name="client")
    cli.outputs()["call"].connect_to(srv.inputs()["srv"])
    win._save_now()
    src = os.path.join(win.current_project_path, "src")
    name = f"studio_srv_test_{os.getpid()}"
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    subprocess.run(["docker", "run", "-d", "--name", name, "-e", "ROS_DOMAIN_ID=49",
                    "-v", f"{src}:/root/ros2_ws/src/user_project", SRV_IMAGE, "bash", "-c",
                    "source /opt/ros/humble/setup.bash && exec ros2 launch "
                    "/root/ros2_ws/src/user_project/launch/project_launch.py"], check=True, capture_output=True)
    try:
        deadline, out = time.time() + 20, ""
        while "for 2 + 3 = 5" not in out and time.time() < deadline:
            time.sleep(1)
            out = subprocess.run(["docker", "logs", name], capture_output=True, text=True).stdout
        assert "Incoming request: a: 2 b: 3" in out and "Result of add_two_ints: for 2 + 3 = 5" in out
    finally:
        subprocess.run(["docker", "rm", "-f", name], capture_output=True)
