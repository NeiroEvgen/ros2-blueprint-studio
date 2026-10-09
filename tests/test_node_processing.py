"""Характеризация обработки нод: имена файлов, группы, порты → топики, коллизии."""
import pytest

NODE_T = "ros.cpp.CppCustomNode"


def node(nid, name=None, source="", code="", ports=(), group=None, pgid=None, t=NODE_T):
    custom = {"node_name": name or nid, "code_content": code, "source_file": source}
    if group:
        custom["container_group"] = group
    if pgid:
        custom["parent_group_id"] = pgid
    return {"id": nid, "type_": t, "name": name or nid, "custom": custom, "ports": list(ports)}


def group(gid, name, internal, container=None):
    c = {"node_name": name, "internal_nodes": list(internal)}
    if container:
        c["container_group"] = container
    return {"id": gid, "type_": "ros.nodes.RosGroup", "name": name, "custom": c, "ports": []}


@pytest.fixture
def d(tmp_path):
    p = tmp_path / "proj" / "src" / "cpp"
    p.mkdir(parents=True)
    return p


def run(pm, nodes, d, t="cpp"):
    return pm._process_and_save_files({"nodes": nodes}, str(d), t)


class TestFileNames:
    def test_name_from_visual_name_sanitized(self, pm, d):
        created = run(pm, [node("n1", name="my node-1")], d)
        assert created[0]["filename"] == "my_node_1.cpp"

    def test_source_file_wins_and_extension_forced(self, pm, d):
        created = run(pm, [node("n1", name="x", source="whatever.py")], d)
        assert created[0]["filename"] == "whatever.cpp"

    def test_dotfile_source_falls_back_to_name(self, pm, d):
        created = run(pm, [node("n1", name="abc", source=".hidden")], d)
        assert created[0]["filename"] == "abc.cpp"

    def test_python_mode_extension(self, pm, tmp_path):
        p = tmp_path / "proj" / "src" / "python"; p.mkdir(parents=True)
        created = pm._process_and_save_files({"nodes": [node("n", name="py_n")]}, str(p), "python")
        assert created[0]["filename"] == "py_n.py" and created[0]["language"] == "python"
        assert (p / "py_n.py").is_file()

    def test_path_in_source_file_cannot_escape_dir(self, pm, d):
        created = run(pm, [node("n", name="x", source="../../evil.cpp")], d)
        assert created[0]["filename"] == "evil.cpp"
        assert not (d.parent.parent / "evil.cpp").exists()
        assert (d / "evil.cpp").exists()


class TestGroups:
    def test_top_level_node_in_main(self, pm, d):
        assert run(pm, [node("a")], d)[0]["group"] == "main"

    def test_group_namespace_becomes_deploy_group_lowercase(self, pm, d):
        g = group("g", "Robot", ["a"])
        a = node("a", pgid="g")
        created = run(pm, [g, a], d)
        assert [(c["filename"], c["group"]) for c in created] == [("a.cpp", "robot")]

    @pytest.mark.xfail(strict=True, reason="БАГ: вложенная группа теряет детей — рекурсия получает только прямых "
                                           "потомков внешней группы, а nodes_dict строится из них же")
    def test_nested_group_children_are_generated(self, pm, d):
        outer = group("o", "Outer", ["i"])
        inner = group("i", "Inner", ["a"])
        inner["custom"]["parent_group_id"] = "o"
        a = node("a", pgid="i")
        created = run(pm, [outer, inner, a], d)
        assert [(c["filename"], c["group"]) for c in created] == [("a.cpp", "outer")]

    @pytest.mark.xfail(strict=True, reason="БАГ: если внешняя группа перечисляет всех потомков, нода обходится дважды "
                                           "(/Outer/Inner и /Outer) → два одинаковых файла в created → дубль в launch")
    def test_nested_group_no_duplicate_when_outer_lists_all_descendants(self, pm, d):
        outer = group("o", "Outer", ["i", "a"])
        inner = group("i", "Inner", ["a"])
        inner["custom"]["parent_group_id"] = "o"
        a = node("a", pgid="i")
        created = run(pm, [outer, inner, a], d)
        assert [c["filename"] for c in created] == ["a.cpp"] and created[0]["group"] == "outer"

    def test_children_of_group_not_duplicated_at_top_level(self, pm, d):
        g = group("g", "G", ["a"])
        created = run(pm, [g, node("a", pgid="g"), node("b")], d)
        assert sorted(c["filename"] for c in created) == ["a.cpp", "b.cpp"]


class TestPortsToTopics:
    def test_output_port_becomes_publisher_with_namespaced_topic(self, pm, d):
        g = group("g", "Arm", ["a"])
        a = node("a", name="a", pgid="g", ports=[{"name": "state", "type": "out"}])
        run(pm, [g, a], d)
        text = (d / "a.cpp").read_text(encoding="utf-8")
        assert 'create_publisher<std_msgs::msg::String>("/Arm/state"' in text

    def test_global_topic_not_prefixed(self, pm, d):
        g = group("g", "Arm", ["a"])
        a = node("a", name="a", pgid="g", ports=[{"name": "/global", "type": "out"}])
        run(pm, [g, a], d)
        text = (d / "a.cpp").read_text(encoding="utf-8")
        assert '"/global"' in text and "/Arm//global" not in text

    def test_top_level_topic_unprefixed(self, pm, d):
        run(pm, [node("n", name="n", ports=[{"name": "state", "type": "out"}])], d)
        assert '"state"' in (d / "n.cpp").read_text(encoding="utf-8")


class TestEdgeCases:
    def test_empty_session(self, pm, d):
        assert run(pm, [], d) == []

    def test_internal_subgraph_nodes_skipped(self, pm, d):
        created = run(pm, [node("i", t="ros.nodes.internal.SubGraphInputNode"), node("a")], d)
        assert [c["filename"] for c in created] == ["a.cpp"]

    def test_second_run_does_not_rewrite_unchanged(self, pm, d):
        run(pm, [node("a")], d)
        m1 = (d / "a.cpp").stat().st_mtime_ns
        run(pm, [node("a")], d)
        assert (d / "a.cpp").stat().st_mtime_ns == m1

    @pytest.mark.xfail(reason="два узла с именами 'a b' и 'a_b' дают один файл: второй молча затирает первого; "
                              "в created_files два одинаковых filename → дубль в launch", strict=True)
    def test_filename_collision_is_detected(self, pm, d):
        created = run(pm, [node("1", name="a b"), node("2", name="a_b")], d)
        names = [c["filename"] for c in created]
        assert len(names) == len(set(names))


class TestModuleLayer:
    def test_core_module_has_no_ui_or_qt(self):
        import core.node_sources as m
        src = open(m.__file__, encoding="utf-8").read()
        assert "ui." not in src and "PyQt" not in src and "PySide" not in src

    def test_manager_delegates_to_module(self, pm, d):
        from core import node_sources
        data = {"nodes": [node("a")]}
        assert pm._flatten_nodes(data) == node_sources.flatten_nodes(data)
