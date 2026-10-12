"""
Генерация CMakeLists.txt и package.xml для C++-проекта.

Разделение:
  scan_sources()          — читает исходники, ищет зависимости и executable (единственное чтение ФС кода)
  render_cmake_auto()     — чистая: AUTO-зона CMake
  render_cmake()          — чистая: AUTO + USER + ament_package()
  render_package_xml()    — чистая
  resolve_user_block()    — читает старый CMakeLists, сохраняет USER-зону / мигрирует ручные правки
  write_if_changed()      — запись только при изменении
  generate_cpp_build_files() — оркестратор (то, что раньше было ProjectManager._generate_cpp_build_files)

Без Qt, без docker.
"""
import logging
import os
import re
import shutil
from dataclasses import dataclass, field
from typing import Set, List

# ==========================================
#   CMakeLists.txt: зоны владения
# ==========================================
CMAKE_AUTO_BEGIN = "# === BLUEPRINT AUTO BEGIN — regenerated on every Save, do not edit ==="
CMAKE_AUTO_END   = "# === BLUEPRINT AUTO END ==="
CMAKE_USER_BEGIN = "# === BLUEPRINT USER BEGIN — never touched by the generator ==="
CMAKE_USER_END   = "# === BLUEPRINT USER END ==="

CMAKE_USER_TEMPLATE = (
    "# Сюда пиши всё, что генератор не знает сам. Примеры:\n"
    "#   target_sources(MyNode PRIVATE cpp/kinematics_lib/trajectory_solver.cpp)\n"
    "#   find_package(yaml-cpp REQUIRED)\n"
    "#   target_link_libraries(MyNode yaml-cpp)\n"
)

BASE_DEPS = ['rclcpp', 'rclcpp_action', 'rclcpp_lifecycle', 'std_msgs',
             'geometry_msgs', 'sensor_msgs', 'example_interfaces']

_PATTERN_MSG = re.compile(r'#include\s+["<]([^/]+)/(msg|srv|action)/[^">]+[">]')
_PATTERN_LIBS = re.compile(r'#include\s+["<](tf2_ros|tf2|tf2_geometry_msgs|image_transport|cv_bridge|opencv2|rclcpp_lifecycle|rclcpp_components|lifecycle_msgs)/[^">]+[">]')
_MAIN_RE = re.compile(r'\bint\s+main\s*\(')


def package_name_from_dir(project_dir: str) -> str:
    """Имя ROS-пакета из имени папки проекта: a-z, 0-9, _."""
    return re.sub(r'[^a-z0-9_]', '_', os.path.basename(project_dir).lower())


@dataclass
class SourceScan:
    deps: Set[str] = field(default_factory=lambda: set(BASE_DEPS))
    apt: Set[str] = field(default_factory=set)
    unresolved: Set[str] = field(default_factory=set)
    executables: List[str] = field(default_factory=list)   # имена файлов .cpp с main(), отсортированы
    # библиотеки, которые НЕ ament-пакеты (yaml-cpp): find_package + plain target_link_libraries
    find_only: Set[str] = field(default_factory=set)
    link_libs: Set[str] = field(default_factory=set)
    xml_extra: Set[str] = field(default_factory=set)
    # exec-файл → доп. исходники (.cpp соседей локальных заголовков), пути от cpp_dir
    companions: dict = field(default_factory=dict)


def _canon(dep: str) -> str:
    """Каноническое имя зависимости: реестр отдаёт 'OpenCV' (cmake), regex — 'opencv'.
    Храним одно ('opencv'), а в CMake рендерим как OpenCV — иначе дубли."""
    return "opencv" if dep == "OpenCV" else dep


def scan_sources(cpp_dir: str) -> SourceScan:
    from core.generators.dependency_resolver import DependencyResolver
    resolver = DependencyResolver()
    scan = SourceScan()
    if not os.path.exists(cpp_dir):
        return scan

    for file in os.listdir(cpp_dir):
        if file.endswith(('.cpp', '.hpp', '.h')):
            with open(os.path.join(cpp_dir, file), 'r', encoding='utf-8') as f:
                content = f.read()
            for match in _PATTERN_MSG.findall(content):
                scan.deps.add(match[0])
            for match in _PATTERN_LIBS.findall(content):
                scan.deps.add("opencv" if match == "opencv2" else match)
            # apt-имя пакета часто НЕ совпадает с именем для find_package —
            # реестр знает оба.
            for inc in resolver.scan_includes(content, "cpp"):
                entry = resolver.resolve_include(inc)
                if entry:
                    scan.apt.update(entry.get("apt", []))
                    if entry.get("link_only"):
                        scan.find_only.update(entry.get("cmake_find", []))
                        scan.link_libs.update(entry.get("cmake_link", []))
                        scan.xml_extra.update(entry.get("xml_depend", []))
                    else:
                        scan.deps.update(_canon(d) for d in entry.get("cmake_find", []))
                else:
                    scan.unresolved.add(inc)

    # Executable только из файлов с int main(): остальные .cpp — библиотечные хелперы.
    for file in sorted(os.listdir(cpp_dir)):
        if file.endswith('.cpp'):
            with open(os.path.join(cpp_dir, file), 'r', encoding='utf-8') as f:
                if _MAIN_RE.search(f.read()):
                    scan.executables.append(file)

    for file in scan.executables:
        extra = _companions(cpp_dir, file, set(scan.executables))
        if extra:
            scan.companions[file] = extra
    return scan


_LOCAL_INC_RE = re.compile(r'#include\s*"([^"]+)"')


def _companions(cpp_dir: str, exec_file: str, exec_set) -> List[str]:
    """
    Соседние .cpp локальных заголовков (solver.hpp → solver.cpp), транзитивно.
    Файлы с main() в companions не попадают — иначе два main() в одном бинарнике.
    """
    found, seen, work = [], set(), [exec_file]
    while work:
        cur = work.pop()
        if cur in seen:
            continue
        seen.add(cur)
        try:
            with open(os.path.join(cpp_dir, cur), 'r', encoding='utf-8') as f:
                text = f.read()
        except OSError:
            continue
        base = os.path.dirname(cur)
        for inc in _LOCAL_INC_RE.findall(text):
            hdr = os.path.normpath(os.path.join(base, inc)).replace(os.sep, "/")
            if hdr.startswith("..") or not os.path.isfile(os.path.join(cpp_dir, hdr)):
                continue
            work.append(hdr)
            comp = os.path.splitext(hdr)[0] + ".cpp"
            if (comp != exec_file and comp not in exec_set and comp not in found
                    and os.path.isfile(os.path.join(cpp_dir, comp))):
                found.append(comp)
                work.append(comp)
    return sorted(found)


def _cmake_name(dep: str) -> str:
    return "OpenCV" if dep == "opencv" else dep


def render_cmake_auto(pkg_name: str, deps, executables, find_only=(), link_libs=(), companions=None) -> str:
    companions = companions or {}
    auto = "cmake_minimum_required(VERSION 3.8)\n"
    auto += f"project({pkg_name})\n\n"
    auto += "if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")\n"
    auto += "  add_compile_options(-Wall -Wextra -Wpedantic)\nendif()\n\n"
    auto += "find_package(ament_cmake REQUIRED)\n"
    for dep in sorted(deps):
        auto += f"find_package({_cmake_name(dep)} REQUIRED)\n"
    for lib in sorted(set(find_only) - {_cmake_name(d) for d in deps}):
        auto += f"find_package({lib} REQUIRED)\n"
    auto += "\ninclude_directories(include)\n\n"

    deps_str = " ".join(_cmake_name(d) for d in sorted(deps))
    for file in executables:
        exec_name = os.path.splitext(file)[0]
        sources = " ".join(f"cpp/{x}" for x in [file] + list(companions.get(file, [])))
        auto += f"add_executable({exec_name} {sources})\n"
        auto += f"ament_target_dependencies({exec_name} {deps_str})\n"
        if link_libs:
            # plain-сигнатура, как и у ament_target_dependencies (смешивать с PUBLIC нельзя)
            auto += f"target_link_libraries({exec_name} {' '.join(sorted(link_libs))})\n"
        auto += f"install(TARGETS {exec_name} DESTINATION lib/${{PROJECT_NAME}})\n\n"

    auto += "install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})\n"
    return auto


def render_cmake(auto: str, user_body: str) -> str:
    # ament_package() стоит после USER-блока: он обязан быть последним.
    return (
        f"{CMAKE_AUTO_BEGIN}\n{auto}{CMAKE_AUTO_END}\n\n"
        f"{CMAKE_USER_BEGIN}\n{user_body}{CMAKE_USER_END}\n\n"
        "ament_package()\n"
    )


def render_package_xml(pkg_name: str, deps) -> str:
    x = '<?xml version="1.0"?>\n'
    x += '<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>\n'
    x += f'<package format="3">\n  <name>{pkg_name}</name>\n  <version>0.0.0</version>\n'
    x += '  <description>Auto-generated by ROS2 Blueprint Studio</description>\n'
    x += '  <maintainer email="user@email.com">User</maintainer>\n'
    x += '  <license>TODO</license>\n  <buildtool_depend>ament_cmake</buildtool_depend>\n\n'
    for dep in sorted(deps):
        x += f"  <depend>{dep}</depend>\n"
    x += '\n  <export><build_type>ament_cmake</build_type></export>\n</package>'
    return x


def resolve_user_block(cmake_path: str, new_auto: str) -> str:
    """
    Возвращает тело USER-блока для записи.

    - Файла нет                → шаблон с примерами.
    - Есть USER-блок           → его содержимое как есть, дословно.
    - Старый файл без маркеров → одноразовая миграция: строки, которых
      нет в новом AUTO (т.е. дописанные человеком), переносятся в
      USER-блок ЗАКОММЕНТИРОВАННЫМИ, а оригинал сохраняется в .bak.
      Закомментированными — потому что среди них могут оказаться
      устаревшие add_executable на удалённые файлы, которые молча
      сломали бы сборку. Человек раскомментирует нужное сам.
    """
    if not os.path.exists(cmake_path):
        return CMAKE_USER_TEMPLATE
    try:
        with open(cmake_path, 'r', encoding='utf-8') as f:
            old = f.read()
    except Exception:
        return CMAKE_USER_TEMPLATE

    m = re.search(re.escape(CMAKE_USER_BEGIN) + r"\n(.*?)" + re.escape(CMAKE_USER_END),
                  old, re.S)
    if m:
        return m.group(1)

    auto_lines = {l.strip() for l in new_auto.splitlines() if l.strip()}
    extra = []
    for line in old.splitlines():
        st = line.strip()
        if not st or st.startswith("#") or st == "ament_package()":
            continue
        if st not in auto_lines:
            extra.append(line)

    if not extra:
        return CMAKE_USER_TEMPLATE

    bak = cmake_path + ".bak"
    n = 1
    while os.path.exists(bak):
        bak = f"{cmake_path}.bak{n}"
        n += 1
    try:
        shutil.copy2(cmake_path, bak)
    except Exception as e:
        logging.warning(f"CMake migration: backup failed: {e}")

    logging.warning(
        f"CMakeLists.txt: найдены ручные правки без USER-блока "
        f"({len(extra)} строк). Перенесены в USER-блок закомментированными, "
        f"оригинал: {os.path.basename(bak)}. Раскомментируй нужное.")

    body = CMAKE_USER_TEMPLATE
    body += "#\n# --- перенесено из старого CMakeLists.txt, проверь и раскомментируй ---\n"
    body += "".join(f"# {l}\n" for l in extra)
    return body


def write_if_changed(path: str, content: str) -> bool:
    """Пишет файл только если содержимое отличается. True — если записали."""
    if os.path.exists(path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                if f.read() == content:
                    return False
        except Exception:
            pass
    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)
    return True


def generate_cpp_build_files(src_dir: str, created_files=None):
    """
    Пишет src/CMakeLists.txt и src/package.xml.
    Возвращает (apt_packages: set, executable_names: set).
    """
    pkg_name = package_name_from_dir(os.path.dirname(src_dir))
    scan = scan_sources(os.path.join(src_dir, "cpp"))

    if scan.unresolved:
        logging.warning(f"CMake deps found but not in apt registry (add to "
                        f"core/generators/dependency_registry.json): "
                        f"{sorted(scan.unresolved)}")
    if not scan.executables:
        logging.warning("No .cpp file with int main() found — nothing to build as executable.")

    cmake_path = os.path.join(src_dir, "CMakeLists.txt")
    auto = render_cmake_auto(pkg_name, scan.deps, scan.executables,
                             scan.find_only, scan.link_libs, scan.companions)
    cmake = render_cmake(auto, resolve_user_block(cmake_path, auto))
    logging.info("CMakeLists.txt сгенерирован/обновлен" if write_if_changed(cmake_path, cmake)
                 else "⏭ CMakeLists.txt пропущен (нет изменений)")

    pkg_path = os.path.join(src_dir, "package.xml")
    logging.info("package.xml сгенерирован/обновлен"
                 if write_if_changed(pkg_path, render_package_xml(pkg_name, scan.deps | scan.xml_extra))
                 else "⏭ package.xml пропущен (нет изменений)")

    return scan.apt, {os.path.splitext(f)[0] for f in scan.executables}
