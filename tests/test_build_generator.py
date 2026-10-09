"""Golden + юнит-тесты build_generator. Golden снят с кода ДО выноса из ProjectManager."""
import ast
import os
import xml.dom.minidom
import pytest
from core.generators import build_generator as bg
from core.project_manager import ProjectManager

GOLD = os.path.join(os.path.dirname(__file__), "golden", "build")


def _read(p):
    with open(p, encoding="utf-8", newline="") as f:
        return f.read().replace("\r\n", "\n")


def make_project(tmp_path):
    root = tmp_path / "My Proj-1"
    cpp = root / "src" / "cpp"
    cpp.mkdir(parents=True)
    (cpp / "talker.cpp").write_text('#include <rclcpp/rclcpp.hpp>\n#include <std_msgs/msg/string.hpp>\n#include <trajectory_msgs/msg/joint_trajectory.hpp>\n#include <tf2_ros/transform_listener.h>\n#include <opencv2/opencv.hpp>\nint main(int a,char**b){return 0;}\n')
    (cpp / "helper.cpp").write_text('#include <my_pkg/msg/thing.hpp>\nvoid f(){}\n')
    (cpp / "helper.hpp").write_text('#include <cv_bridge/cv_bridge.h>\n')
    (cpp / "listener.cpp").write_text('#include <rclcpp/rclcpp.hpp>\n  int  main ( ) { }\n')
    return str(root / "src")


class TestGolden:
    def test_cmake_and_package_xml_unchanged_by_refactor(self, tmp_path):
        src = make_project(tmp_path)
        apt, ex = bg.generate_cpp_build_files(src)
        assert _read(os.path.join(src, "CMakeLists.txt")) == _read(os.path.join(GOLD, "CMakeLists.txt"))
        assert _read(os.path.join(src, "package.xml")) == _read(os.path.join(GOLD, "package.xml"))
        assert (sorted(apt), sorted(ex)) == eval(_read(os.path.join(GOLD, "meta.txt")))

    def test_project_manager_delegates_identically(self, tmp_path):
        src = make_project(tmp_path)
        ProjectManager(None, None)._generate_cpp_build_files(src, [])
        assert _read(os.path.join(src, "CMakeLists.txt")) == _read(os.path.join(GOLD, "CMakeLists.txt"))


class TestPure:
    def test_package_name(self):
        assert bg.package_name_from_dir("/x/My Proj-1") == "my_proj_1"
        import re
        assert re.fullmatch(r"[a-z0-9_]+", bg.package_name_from_dir("/x/Проект"))

    def test_package_xml_wellformed_and_sorted(self):
        x = bg.render_package_xml("p", {"b", "a"})
        xml.dom.minidom.parseString(x)
        assert x.index("<depend>a") < x.index("<depend>b")

    def test_cmake_auto_targets(self):
        a = bg.render_cmake_auto("p", {"rclcpp", "opencv"}, ["n.cpp"])
        assert "add_executable(n cpp/n.cpp)" in a
        assert "find_package(OpenCV REQUIRED)" in a and "opencv REQUIRED" not in a
        assert "install(TARGETS n DESTINATION lib/${PROJECT_NAME})" in a

    def test_cmake_zones_order(self):
        c = bg.render_cmake("AUTO\n", "USER\n")
        assert c.index(bg.CMAKE_AUTO_BEGIN) < c.index(bg.CMAKE_AUTO_END) < c.index(bg.CMAKE_USER_BEGIN) \
            < c.index(bg.CMAKE_USER_END) < c.index("ament_package()")

    def test_write_if_changed(self, tmp_path):
        p = str(tmp_path / "f.txt")
        assert bg.write_if_changed(p, "a") is True
        assert bg.write_if_changed(p, "a") is False
        assert bg.write_if_changed(p, "b") is True


class TestScan:
    def test_only_main_files_are_executables(self, tmp_path):
        src = make_project(tmp_path)
        sc = bg.scan_sources(os.path.join(src, "cpp"))
        assert sc.executables == ["listener.cpp", "talker.cpp"]

    def test_msg_include_adds_dep(self, tmp_path):
        src = make_project(tmp_path)
        assert "trajectory_msgs" in bg.scan_sources(os.path.join(src, "cpp")).deps

    def test_missing_dir_gives_base_deps(self, tmp_path):
        sc = bg.scan_sources(str(tmp_path / "nope"))
        assert sc.deps == set(bg.BASE_DEPS) and sc.executables == []

    def test_no_executables_still_writes_files(self, tmp_path):
        root = tmp_path / "p"; (root / "src" / "cpp").mkdir(parents=True)
        (root / "src" / "cpp" / "lib.cpp").write_text("void f(){}")
        apt, ex = bg.generate_cpp_build_files(str(root / "src"))
        assert ex == set() and (root / "src" / "CMakeLists.txt").is_file()


class TestNoDuplicateDeps:
    def test_opencv_once_everywhere(self, tmp_path):
        src = make_project(tmp_path)
        bg.generate_cpp_build_files(src)
        cm = _read(os.path.join(src, "CMakeLists.txt"))
        px = _read(os.path.join(src, "package.xml"))
        assert cm.count("find_package(OpenCV REQUIRED)") == 1
        for line in cm.splitlines():
            if line.startswith("ament_target_dependencies(talker"):
                args = line[line.index(" ") + 1:-1].split()[1:]
                assert len(args) == len(set(args))
        assert px.count("<depend>opencv</depend>") == 1 and "<depend>OpenCV</depend>" not in px

    def test_find_package_lines_unique(self, tmp_path):
        src = make_project(tmp_path)
        bg.generate_cpp_build_files(src)
        lines = [l for l in _read(os.path.join(src, "CMakeLists.txt")).splitlines()
                 if l.startswith("find_package(")]
        assert len(lines) == len(set(lines))
