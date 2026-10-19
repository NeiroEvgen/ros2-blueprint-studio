"""
CMakeLists.txt and package.xml generation for a C++ project.

Split:
  scan_sources()          — reads the sources, finds dependencies and executables (the only filesystem read of the code)
  render_cmake_auto()     — pure: CMake AUTO zone
  render_cmake()          — pure: AUTO + USER + ament_package()
  render_package_xml()    — pure
  resolve_user_block()    — reads the old CMakeLists, keeps the USER zone / migrates hand edits
  write_if_changed()      — writes only when the content changed
  generate_cpp_build_files() — orchestrator (ProjectManager delegates here)

No Qt, no docker.
"""
import logging
import os
import re
import shutil
from dataclasses import dataclass, field
from typing import Set, List

# ==========================================
#   CMakeLists.txt: ownership zones
# ==========================================
CMAKE_AUTO_BEGIN = "# === BLUEPRINT AUTO BEGIN — regenerated on every Save, do not edit ==="
CMAKE_AUTO_END   = "# === BLUEPRINT AUTO END ==="
CMAKE_USER_BEGIN = "# === BLUEPRINT USER BEGIN — never touched by the generator ==="
CMAKE_USER_END   = "# === BLUEPRINT USER END ==="

CMAKE_USER_TEMPLATE = (
    "# Put here everything the generator does not know about. Examples:\n"
    "#   target_sources(MyNode PRIVATE cpp/kinematics_lib/trajectory_solver.cpp)\n"
    "#   find_package(yaml-cpp REQUIRED)\n"
    "#   target_link_libraries(MyNode yaml-cpp)\n"
)

BASE_DEPS = ['rclcpp', 'rclcpp_action', 'rclcpp_lifecycle', 'std_msgs',
             'geometry_msgs', 'sensor_msgs']

_PATTERN_MSG = re.compile(r'#include[ \t]+["<]([A-Za-z_][A-Za-z0-9_]*)/(msg|srv|action)/[^">\n]+[">]')
_PATTERN_LIBS = re.compile(r'#include\s+["<](tf2_ros|tf2|tf2_geometry_msgs|image_transport|cv_bridge|opencv2|rclcpp_lifecycle|rclcpp_components|lifecycle_msgs)/[^">]+[">]')
_MAIN_RE = re.compile(r'\bint\s+main\s*\(')


def package_name_from_dir(project_dir: str) -> str:
    """
    ROS package name from the project folder name: a-z, 0-9, _, starts with a letter (REP 144).
    The only place of this rule: the generator, Save and Remote all use it.
    Valid names are unchanged; "2robot" -> "pkg_2robot", a non-Latin name -> "pkg_<hash>".
    """
    base = os.path.basename(os.path.normpath(project_dir))
    name = re.sub(r'[^a-z0-9_]', '_', base.lower())
    if not name.strip('_'):
        import hashlib
        return "pkg_" + hashlib.sha1(base.encode("utf-8")).hexdigest()[:8]
    return name if name[0].isalpha() else "pkg_" + name.lstrip('_')


@dataclass
class SourceScan:
    deps: Set[str] = field(default_factory=lambda: set(BASE_DEPS))
    apt: Set[str] = field(default_factory=set)
    unresolved: Set[str] = field(default_factory=set)
    executables: List[str] = field(default_factory=list)   # .cpp file names with main(), sorted
    # libraries that are NOT ament packages (yaml-cpp): find_package + plain target_link_libraries
    find_only: Set[str] = field(default_factory=set)
    link_libs: Set[str] = field(default_factory=set)
    xml_extra: Set[str] = field(default_factory=set)
    # executable file -> extra sources (.cpp next to local headers), paths relative to cpp_dir
    companions: dict = field(default_factory=dict)
    # files marked by the user (paths relative to src/)
    extra_sources: List[str] = field(default_factory=list)   # added to every executable
    marked_execs: List[str] = field(default_factory=list)    # separate executables (have main)
    include_dirs: List[str] = field(default_factory=list)    # → include_directories


def _canon(dep: str) -> str:
    """Canonical dependency name: the registry returns 'OpenCV' (cmake), the regex 'opencv'.
    We keep one ('opencv') and render it as OpenCV in CMake, otherwise there are duplicates."""
    return "opencv" if dep == "OpenCV" else dep


def _scan_text(content: str, scan: "SourceScan", resolver) -> None:
    for match in _PATTERN_MSG.findall(content):
        scan.deps.add(match[0])
    for match in _PATTERN_LIBS.findall(content):
        scan.deps.add("opencv" if match == "opencv2" else match)
    # the apt package name often differs from the find_package name;
    # the registry knows both.
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


def scan_sources(cpp_dir: str, marked=(), explicit=None) -> SourceScan:
    """
    marked: files marked by the user in the Files tab (paths relative to src/).
    .cpp without main() is added to every executable, .cpp with main() becomes
    a separate executable, header folders go to include_directories.
    explicit: paths marked as files (not via a folder mark); None = all of them.
    A file covered by a folder mark does NOT become an executable (so test/ or scripts/
    do not spawn binaries) and, if it lies directly in cpp/, is not duplicated: the scanner already handles it.
    """
    from core.generators.dependency_resolver import DependencyResolver
    resolver = DependencyResolver()
    scan = SourceScan()
    src_dir = os.path.dirname(os.path.normpath(cpp_dir))
    if not os.path.exists(cpp_dir) and not marked:
        return scan

    listing = os.listdir(cpp_dir) if os.path.isdir(cpp_dir) else []
    for file in listing:
        if file.endswith(('.cpp', '.hpp', '.h')):
            with open(os.path.join(cpp_dir, file), 'r', encoding='utf-8') as f:
                _scan_text(f.read(), scan, resolver)

    # Executables only from files with int main(): other .cpp files are library helpers.
    for file in sorted(listing):
        if file.endswith('.cpp'):
            with open(os.path.join(cpp_dir, file), 'r', encoding='utf-8') as f:
                if _MAIN_RE.search(f.read()):
                    scan.executables.append(file)

    for file in scan.executables:
        extra = _companions(cpp_dir, file, set(scan.executables))
        if extra:
            scan.companions[file] = extra

    _apply_marked(scan, src_dir, marked, resolver, explicit)
    return scan


def _apply_marked(scan, src_dir, marked, resolver, explicit=None) -> None:
    top = {f"cpp/{f}" for f in scan.executables}
    for rel in marked:
        ext = os.path.splitext(rel)[1].lower()
        if ext not in (".cpp", ".cc", ".cxx", ".hpp", ".h", ".hh"):
            continue
        try:
            with open(os.path.join(src_dir, rel), 'r', encoding='utf-8') as f:
                text = f.read()
        except (OSError, UnicodeDecodeError):
            logging.warning(f"Marked file unreadable, skipped: {rel}")
            continue
        _scan_text(text, scan, resolver)
        d = os.path.dirname(rel)
        if d and d not in scan.include_dirs and d != "include":      # include/ is already listed
            scan.include_dirs.append(d)
        if ext in (".hpp", ".h", ".hh") or rel in top:
            continue
        by_folder = explicit is not None and rel not in explicit
        if _MAIN_RE.search(text):
            if not by_folder:
                scan.marked_execs.append(rel)
        elif by_folder and os.path.dirname(rel) == "cpp":
            continue                     # helper directly in cpp/: the scanner already adds it via #include
        else:
            scan.extra_sources.append(rel)
    scan.include_dirs.sort()
    scan.extra_sources = sorted(set(scan.extra_sources))


_LOCAL_INC_RE = re.compile(r'#include\s*"([^"]+)"')


def _companions(cpp_dir: str, exec_file: str, exec_set) -> List[str]:
    """
    .cpp files next to local headers (solver.hpp -> solver.cpp), transitively.
    Files with main() never become companions, otherwise one binary gets two main().
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


def render_cmake_auto(pkg_name: str, deps, executables, find_only=(), link_libs=(), companions=None,
                      extra_sources=(), marked_execs=(), include_dirs=()) -> str:
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
    auto += "\ninclude_directories(" + " ".join(["include"] + list(include_dirs)) + ")\n\n"

    deps_str = " ".join(_cmake_name(d) for d in sorted(deps))
    jobs = [(os.path.splitext(f)[0], [f"cpp/{x}" for x in [f] + list(companions.get(f, []))])
            for f in executables]
    taken = {n for n, _ in jobs}
    for rel in marked_execs:
        name = os.path.splitext(os.path.basename(rel))[0]
        if name in taken:
            logging.warning(f"Marked file {rel}: executable '{name}' already exists, skipped")
            continue
        taken.add(name)
        jobs.append((name, [rel]))
    for exec_name, srcs in jobs:
        sources = " ".join(srcs + [x for x in extra_sources if x not in srcs])
        auto += f"add_executable({exec_name} {sources})\n"
        auto += f"ament_target_dependencies({exec_name} {deps_str})\n"
        if link_libs:
            # plain signature, like ament_target_dependencies (cannot be mixed with PUBLIC)
            auto += f"target_link_libraries({exec_name} {' '.join(sorted(link_libs))})\n"
        auto += f"install(TARGETS {exec_name} DESTINATION lib/${{PROJECT_NAME}})\n\n"

    auto += "install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})\n"
    return auto


def render_cmake(auto: str, user_body: str) -> str:
    # ament_package() goes after the USER block: it must be last.
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


def _read_text_any(path: str):
    """
    (text, encoding). A CMakeLists edited in Windows Notepad may be cp1251:
    reading it only as UTF-8 failed, and the USER block was silently replaced by the template.
    """
    with open(path, 'rb') as f:
        raw = f.read()
    for enc in ("utf-8", "cp1251"):
        try:
            return raw.decode(enc), enc
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1"), "latin-1"      # single-byte: decodes anything


def _backup(path: str):
    """Copy path to the first free .bak / .bakN. Returns the copy's path or None."""
    bak = path + ".bak"
    n = 1
    while os.path.exists(bak):
        bak = f"{path}.bak{n}"
        n += 1
    try:
        shutil.copy2(path, bak)
        return bak
    except OSError as e:
        logging.warning(f"CMake: backup failed: {e}")
        return None


def resolve_user_block(cmake_path: str, new_auto: str) -> str:
    """
    Returns the USER block body to write.

    - No file                     -> template with examples.
    - Has a USER block            -> its content verbatim.
    - Old file without markers    -> one-time migration: lines that are not
      in the new AUTO zone (i.e. added by a person) are moved into the
      USER block COMMENTED OUT, and the original is kept as .bak.
      Commented out because they may include stale add_executable lines
      for deleted files, which would silently break the build.
      The person uncomments what is needed.
    """
    if not os.path.exists(cmake_path):
        return CMAKE_USER_TEMPLATE
    try:
        old, encoding = _read_text_any(cmake_path)
    except OSError:
        return CMAKE_USER_TEMPLATE
    if encoding != "utf-8":
        # the file is rewritten as UTF-8: original bytes go to .bak, hand edits are kept
        bak = _backup(cmake_path)
        logging.warning(f"CMakeLists.txt was {encoding}, not UTF-8: "
                        f"re-encoded, original: {os.path.basename(bak or '')}")

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

    bak = _backup(cmake_path) or cmake_path + ".bak"

    logging.warning(
        f"CMakeLists.txt: hand edits found without a USER block "
        f"({len(extra)} lines). Moved into the USER block commented out, "
        f"original: {os.path.basename(bak)}. Uncomment what you need.")

    body = CMAKE_USER_TEMPLATE
    body += "#\n# --- migrated from the old CMakeLists.txt: review and uncomment ---\n"
    body += "".join(f"# {l}\n" for l in extra)
    return body


def write_if_changed(path: str, content: str) -> bool:
    """Writes the file only if the content differs. True if it was written."""
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
    Writes src/CMakeLists.txt and src/package.xml.
    Returns (apt_packages: set, executable_names: set).
    """
    pkg_name = package_name_from_dir(os.path.dirname(src_dir))
    from core import build_marks
    marks = build_marks.load(os.path.dirname(src_dir))
    marked = build_marks.marked_files(src_dir, marks)
    scan = scan_sources(os.path.join(src_dir, "cpp"), marked, set(marks))

    if scan.unresolved:
        logging.warning(f"CMake deps found but not in apt registry (add to "
                        f"core/generators/dependency_registry.json): "
                        f"{sorted(scan.unresolved)}")
    if not scan.executables and not scan.marked_execs:
        logging.warning("No .cpp file with int main() found — nothing to build as executable.")

    cmake_path = os.path.join(src_dir, "CMakeLists.txt")
    auto = render_cmake_auto(pkg_name, scan.deps, scan.executables,
                             scan.find_only, scan.link_libs, scan.companions,
                             scan.extra_sources, scan.marked_execs, scan.include_dirs)
    cmake = render_cmake(auto, resolve_user_block(cmake_path, auto))
    logging.info("CMakeLists.txt generated/updated" if write_if_changed(cmake_path, cmake)
                 else "⏭ CMakeLists.txt skipped (no changes)")

    pkg_path = os.path.join(src_dir, "package.xml")
    logging.info("package.xml generated/updated"
                 if write_if_changed(pkg_path, render_package_xml(pkg_name, scan.deps | scan.xml_extra))
                 else "⏭ package.xml skipped (no changes)")

    names = {os.path.splitext(f)[0] for f in scan.executables}
    names |= {os.path.splitext(os.path.basename(f))[0] for f in scan.marked_execs}
    return scan.apt, names
