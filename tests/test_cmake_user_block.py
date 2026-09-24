"""
1.1 — CMake protected block.

Критерии приёмки:
- вписанное в USER-блок переживает Save/регенерацию
- старый проект без USER-блока: блок создаётся, ручные правки не теряются
- чисто сгенерированный старый файл мигрирует без шума и без .bak
- ament_package() остаётся последним
"""
import re

from core.project_manager import (
    CMAKE_AUTO_BEGIN, CMAKE_AUTO_END, CMAKE_USER_BEGIN, CMAKE_USER_END,
)
from tests.conftest import MAIN_CPP


def _cmake(src):
    return (src / "CMakeLists.txt").read_text(encoding="utf-8")


def _user_block(text):
    m = re.search(re.escape(CMAKE_USER_BEGIN) + r"\n(.*?)" + re.escape(CMAKE_USER_END),
                  text, re.S)
    assert m, "USER-блок не найден"
    return m.group(1)


def _put_in_user_block(src, lines):
    path = src / "CMakeLists.txt"
    text = path.read_text(encoding="utf-8")
    body = _user_block(text)
    path.write_text(text.replace(body, body + "".join(l + "\n" for l in lines)),
                    encoding="utf-8")


def _as_old_format(new_text):
    """Во что превращался файл старым генератором: AUTO без маркеров + ament_package()."""
    auto = new_text.split(CMAKE_AUTO_BEGIN + "\n", 1)[1].split(CMAKE_AUTO_END, 1)[0]
    return auto + "ament_package()\n"


class TestUserBlock:

    def test_fresh_project_has_both_blocks(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        text = _cmake(src)
        for marker in (CMAKE_AUTO_BEGIN, CMAKE_AUTO_END, CMAKE_USER_BEGIN, CMAKE_USER_END):
            assert marker in text

    def test_ament_package_stays_last(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        _put_in_user_block(src, ["target_link_libraries(Node yaml-cpp)"])
        pm._generate_cpp_build_files(str(src), [])
        text = _cmake(src)
        assert text.rstrip().endswith("ament_package()")
        assert text.index(CMAKE_USER_END) < text.index("ament_package()")

    def test_user_lines_survive_regeneration(self, pm, cpp_project):
        """Главный критерий: ручная правка переживает Save."""
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        _put_in_user_block(src, ["target_link_libraries(Node yaml-cpp)"])

        pm._generate_cpp_build_files(str(src), [])

        assert "target_link_libraries(Node yaml-cpp)" in _user_block(_cmake(src))

    def test_user_lines_survive_when_auto_part_changes(self, pm, cpp_project):
        """Изменились зависимости → AUTO переписан, USER цел."""
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        _put_in_user_block(src, ["target_link_libraries(Node yaml-cpp)"])

        add("Plan.cpp", '#include "moveit/move_group_interface/move_group_interface.h"\n'
                        + MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])

        text = _cmake(src)
        assert "add_executable(Plan cpp/Plan.cpp)" in text
        assert "moveit_ros_planning_interface" in text
        assert "target_link_libraries(Node yaml-cpp)" in _user_block(text)

    def test_real_case_trajectory_player_with_kinlib(self, pm, cpp_project):
        """Ровно то, что стиралось дважды на G1-проекте."""
        src, add = cpp_project
        add("TrajectoryPlayerNode.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        manual = [
            "find_package(yaml-cpp REQUIRED)",
            "target_sources(TrajectoryPlayerNode PRIVATE "
            "cpp/kinematics_lib/trajectory_solver.cpp)",
            "target_link_libraries(TrajectoryPlayerNode yaml-cpp)",
        ]
        _put_in_user_block(src, manual)

        for _ in range(3):
            pm._generate_cpp_build_files(str(src), [])

        block = _user_block(_cmake(src))
        for line in manual:
            assert line in block

    def test_user_block_preserved_verbatim(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        _put_in_user_block(src, ["  # мой комментарий с отступом", "", "set(FOO bar)"])
        before = _user_block(_cmake(src))

        pm._generate_cpp_build_files(str(src), [])

        assert _user_block(_cmake(src)) == before

    def test_idempotent_no_rewrite(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        _put_in_user_block(src, ["set(FOO bar)"])
        pm._generate_cpp_build_files(str(src), [])
        mtime = (src / "CMakeLists.txt").stat().st_mtime_ns

        pm._generate_cpp_build_files(str(src), [])

        assert (src / "CMakeLists.txt").stat().st_mtime_ns == mtime


class TestMigrationFromOldFormat:

    def test_pure_generated_old_file_migrates_silently(self, pm, cpp_project):
        """Старый файл без правок → никаких .bak и перенесённых строк."""
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        (src / "CMakeLists.txt").write_text(_as_old_format(_cmake(src)), encoding="utf-8")

        pm._generate_cpp_build_files(str(src), [])

        assert not list(src.glob("CMakeLists.txt.bak*"))
        assert "перенесено из старого" not in _cmake(src)
        assert CMAKE_USER_BEGIN in _cmake(src)

    def test_hand_edited_old_file_is_backed_up(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        old = _as_old_format(_cmake(src)).replace(
            "ament_package()", "target_link_libraries(Node yaml-cpp)\nament_package()")
        (src / "CMakeLists.txt").write_text(old, encoding="utf-8")

        pm._generate_cpp_build_files(str(src), [])

        baks = list(src.glob("CMakeLists.txt.bak*"))
        assert len(baks) == 1
        assert baks[0].read_text(encoding="utf-8") == old

    def test_hand_edits_migrated_commented_out(self, pm, cpp_project):
        """Ручная строка не теряется, но и не активируется вслепую."""
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        old = _as_old_format(_cmake(src)).replace(
            "ament_package()", "target_link_libraries(Node yaml-cpp)\nament_package()")
        (src / "CMakeLists.txt").write_text(old, encoding="utf-8")

        pm._generate_cpp_build_files(str(src), [])

        block = _user_block(_cmake(src))
        assert "# target_link_libraries(Node yaml-cpp)" in block
        active = [l for l in block.splitlines() if l.strip() and not l.strip().startswith("#")]
        assert active == []

    def test_stale_executable_not_reactivated(self, pm, cpp_project):
        """add_executable на удалённый файл не должен молча вернуться в сборку."""
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        old = _as_old_format(_cmake(src)).replace(
            "ament_package()", "add_executable(Deleted cpp/Deleted.cpp)\nament_package()")
        (src / "CMakeLists.txt").write_text(old, encoding="utf-8")

        pm._generate_cpp_build_files(str(src), [])

        text = _cmake(src)
        active = [l for l in text.splitlines() if not l.strip().startswith("#")]
        assert not any("Deleted" in l for l in active)

    def test_migration_happens_once(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        old = _as_old_format(_cmake(src)).replace(
            "ament_package()", "set(FOO bar)\nament_package()")
        (src / "CMakeLists.txt").write_text(old, encoding="utf-8")

        pm._generate_cpp_build_files(str(src), [])
        pm._generate_cpp_build_files(str(src), [])
        pm._generate_cpp_build_files(str(src), [])

        assert len(list(src.glob("CMakeLists.txt.bak*"))) == 1
        assert _cmake(src).count("перенесено из старого") == 1

    def test_existing_bak_is_not_overwritten(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        (src / "CMakeLists.txt.bak").write_text("precious", encoding="utf-8")
        old = _as_old_format(_cmake(src)).replace(
            "ament_package()", "set(FOO bar)\nament_package()")
        (src / "CMakeLists.txt").write_text(old, encoding="utf-8")

        pm._generate_cpp_build_files(str(src), [])

        assert (src / "CMakeLists.txt.bak").read_text(encoding="utf-8") == "precious"
        assert (src / "CMakeLists.txt.bak1").read_text(encoding="utf-8") == old
