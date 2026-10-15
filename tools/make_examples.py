"""
Пересобирает учебные примеры examples/{py,cpp,mixed}_pubsub средствами самой Studio
(offscreen Qt), чтобы формат проекта всегда совпадал с текущей версией.

  python -m tools.make_examples [--out examples]

Каждый пример: издатель std_msgs/String на /chatter и подписчик, соединённые проводом.
mixed_pubsub — Python-издатель и C++-подписчик на одном холсте.
"""
import argparse
import os
import shutil
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Шаблон Python-издателя из палитры ждёт триггер от ноды-таймера; для самодостаточного
# примера издатель получает собственный код с таймером.
PY_TALKER = """import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, '/chatter', 10)
        self.i = 0
        self.create_timer(1.0, self.tick)

    def tick(self):
        msg = String()
        msg.data = f'Py: {self.i}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.i += 1


def main():
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""

EXAMPLES = {
    # имя: (тип проекта, (тип издателя, тип подписчика))
    "py_pubsub": ("python", ("ros.py.PyStringPubNode", "ros.py.PyStringSubNode")),
    "cpp_pubsub": ("cpp", ("ros.cpp.CppStringPubNode", "ros.cpp.CppStringSubNode")),
    "mixed_pubsub": ("mixed", ("ros.py.PyStringPubNode", "ros.cpp.CppStringSubNode")),
}


def build(out_dir: str) -> list:
    from PySide6 import QtWidgets
    from core import settings
    from core.workspace_manager import WorkspaceManager

    os.environ["STUDIO_SETTINGS_DIR"] = tempfile.mkdtemp(prefix="studio_examples_")
    settings.set("single_canvas", True)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    from ui import main_window as mw

    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    WorkspaceManager.get_workspace_root = staticmethod(lambda: out_dir)

    made = []
    for name, (ptype, (pub_t, sub_t)) in EXAMPLES.items():
        target = os.path.join(out_dir, name)
        if os.path.exists(target):
            shutil.rmtree(target)
        path = WorkspaceManager.create_project(name, ptype)
        win = mw.RosVisualRunner()
        g = win.graph_py
        pub = g.create_node(pub_t, name="talker", pos=[0, 0])
        sub = g.create_node(sub_t, name="listener", pos=[420, 0])
        for node, node_name in ((pub, "talker"), (sub, "listener")):
            node.set_property("node_name", node_name)
            node.set_property("topic_name", "/chatter")
        if pub_t.startswith("ros.py."):
            pub.set_property("code_content", PY_TALKER)
        pub.outputs()["out"].connect_to(sub.inputs()["in"])
        res = win.project_manager.save_project(path)
        if not (res and res[0]):
            raise RuntimeError(f"не удалось сохранить пример {name}")
        win.close()
        win.deleteLater()
        made.append(path)
    app.processEvents()
    return made


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="examples")
    for p in build(ap.parse_args(argv).out):
        print("OK", p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
