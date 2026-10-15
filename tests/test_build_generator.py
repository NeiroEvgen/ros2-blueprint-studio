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


class TestLinkOnlyLibs:
    def _proj(self, tmp_path, code):
        root = tmp_path / "p"; (root / "src" / "cpp").mkdir(parents=True)
        (root / "src" / "cpp" / "n.cpp").write_text(code, encoding="utf-8")
        return str(root / "src")

    def test_yaml_cpp_found_and_linked_plain(self, tmp_path):
        src = self._proj(tmp_path, '#include <yaml-cpp/yaml.h>\nint main(){}\n')
        apt, _ = bg.generate_cpp_build_files(src)
        cm = _read(os.path.join(src, "CMakeLists.txt"))
        assert "find_package(yaml-cpp REQUIRED)" in cm
        assert "target_link_libraries(n yaml-cpp)" in cm
        ament = [l for l in cm.splitlines() if l.startswith("ament_target_dependencies(n")][0]
        assert "yaml-cpp" not in ament            # yaml-cpp не ament-пакет
        assert "libyaml-cpp-dev" in apt
        assert "<depend>libyaml-cpp-dev</depend>" in _read(os.path.join(src, "package.xml"))

    def test_generated_cmake_passes_lint(self, tmp_path):
        from core.cmake_lint import lint
        src = self._proj(tmp_path, '#include <yaml-cpp/yaml.h>\nint main(){}\n')
        bg.generate_cpp_build_files(src)
        assert lint(_read(os.path.join(src, "CMakeLists.txt"))) == []

    def test_no_link_line_without_link_only_libs(self, tmp_path):
        src = self._proj(tmp_path, '#include <rclcpp/rclcpp.hpp>\nint main(){}\n')
        bg.generate_cpp_build_files(src)
        auto = _read(os.path.join(src, "CMakeLists.txt")).split(bg.CMAKE_AUTO_END)[0]  # в USER-шаблоне есть пример
        assert "target_link_libraries" not in auto


class TestCompanionSources:
    def _w(self, cpp, name, text):
        p = cpp / name; p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text, encoding="utf-8")

    def _setup(self, tmp_path):
        cpp = tmp_path / "p" / "src" / "cpp"; cpp.mkdir(parents=True)
        return cpp, str(tmp_path / "p" / "src")

    def _exec_line(self, src, name):
        import re
        return re.search(rf"add_executable\({name} ([^)]*)\)", _read(os.path.join(src, "CMakeLists.txt"))).group(1)

    def test_companion_added(self, tmp_path):
        cpp, src = self._setup(tmp_path)
        self._w(cpp, "n.cpp", '#include "solver.hpp"\nint main(){}\n')
        self._w(cpp, "solver.hpp", "double s();\n"); self._w(cpp, "solver.cpp", "double s(){return 1;}\n")
        bg.generate_cpp_build_files(src)
        assert self._exec_line(src, "n") == "cpp/n.cpp cpp/solver.cpp"

    def test_transitive_through_header_and_companion(self, tmp_path):
        cpp, src = self._setup(tmp_path)
        self._w(cpp, "n.cpp", '#include "a.hpp"\nint main(){}\n')
        self._w(cpp, "a.hpp", '#include "b.hpp"\n'); self._w(cpp, "a.cpp", "void a(){}\n")
        self._w(cpp, "b.hpp", "void b();\n"); self._w(cpp, "b.cpp", "void b(){}\n")
        bg.generate_cpp_build_files(src)
        assert self._exec_line(src, "n") == "cpp/n.cpp cpp/a.cpp cpp/b.cpp"

    def test_companion_with_main_is_not_linked(self, tmp_path):
        cpp, src = self._setup(tmp_path)
        self._w(cpp, "n.cpp", '#include "other.hpp"\nint main(){}\n')
        self._w(cpp, "other.hpp", ""); self._w(cpp, "other.cpp", "int main(){}\n")
        bg.generate_cpp_build_files(src)
        assert self._exec_line(src, "n") == "cpp/n.cpp"

    def test_subdirectory_header(self, tmp_path):
        cpp, src = self._setup(tmp_path)
        self._w(cpp, "n.cpp", '#include "lib/k.hpp"\nint main(){}\n')
        self._w(cpp, "lib/k.hpp", ""); self._w(cpp, "lib/k.cpp", "void k(){}\n")
        bg.generate_cpp_build_files(src)
        assert self._exec_line(src, "n") == "cpp/n.cpp cpp/lib/k.cpp"

    def test_include_escaping_dir_is_ignored(self, tmp_path):
        cpp, src = self._setup(tmp_path)
        self._w(cpp, "n.cpp", '#include "../secret.hpp"\nint main(){}\n')
        (cpp.parent / "secret.hpp").write_text(""); (cpp.parent / "secret.cpp").write_text("")
        bg.generate_cpp_build_files(src)
        assert self._exec_line(src, "n") == "cpp/n.cpp"

    def test_no_local_includes_unchanged(self, tmp_path):
        cpp, src = self._setup(tmp_path)
        self._w(cpp, "n.cpp", '#include <rclcpp/rclcpp.hpp>\nint main(){}\n')
        bg.generate_cpp_build_files(src)
        assert self._exec_line(src, "n") == "cpp/n.cpp"

    def test_include_cycle_terminates(self, tmp_path):
        cpp, src = self._setup(tmp_path)
        self._w(cpp, "n.cpp", '#include "a.hpp"\nint main(){}\n')
        self._w(cpp, "a.hpp", '#include "b.hpp"\n'); self._w(cpp, "b.hpp", '#include "a.hpp"\n')
        self._w(cpp, "a.cpp", ""); self._w(cpp, "b.cpp", "")
        bg.generate_cpp_build_files(src)
        assert self._exec_line(src, "n") == "cpp/n.cpp cpp/a.cpp cpp/b.cpp"


class TestExampleInterfacesOptional:
    """example_interfaces есть не в каждом образе робота: требуем его, только если код его include'ит."""

    def _scan(self, tmp_path, body):
        cpp = tmp_path / "cpp"
        cpp.mkdir()
        (cpp / "n.cpp").write_text(body, encoding="utf-8")
        return bg.scan_sources(str(cpp))

    def test_not_required_by_default(self, tmp_path):
        scan = self._scan(tmp_path, '#include <rclcpp/rclcpp.hpp>\nint main(){return 0;}\n')
        assert "example_interfaces" not in scan.deps
        text = bg.render_cmake_auto("p", scan.deps, scan.executables)
        assert "example_interfaces" not in text

    def test_added_when_the_code_uses_it(self, tmp_path):
        scan = self._scan(tmp_path, '#include "example_interfaces/action/fibonacci.hpp"\nint main(){return 0;}\n')
        assert "example_interfaces" in scan.deps
