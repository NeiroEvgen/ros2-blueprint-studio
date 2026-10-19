"""Characterization of node processing: file names, groups, ports -> topics, collisions."""
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

    def test_nested_group_children_are_generated(self, pm, d):
        outer = group("o", "Outer", ["i"])
        inner = group("i", "Inner", ["a"])
        inner["custom"]["parent_group_id"] = "o"
        a = node("a", pgid="i")
        created = run(pm, [outer, inner, a], d)
        assert [(c["filename"], c["group"]) for c in created] == [("a.cpp", "outer")]

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


class TestFixedBugs:
    def test_collision_gets_suffix_and_is_stable(self, pm, d):
        nodes = [node("1", name="a b"), node("2", name="a_b"), node("3", name="a-b")]
        created = run(pm, nodes, d)
        names = [c["filename"] for c in created]
        assert names == ["a_b.cpp", "a_b_2.cpp", "a_b_3.cpp"]
        assert sorted(p.name for p in d.glob("*.cpp")) == sorted(names)
        # node.custom.source_file is updated -> the second Save gives the same names
        assert [c["filename"] for c in run(pm, nodes, d)] == names

    def test_explicit_distinct_source_files_untouched(self, pm, d):
        created = run(pm, [node("1", source="x.cpp"), node("2", source="y.cpp")], d)
        assert [c["filename"] for c in created] == ["x.cpp", "y.cpp"]

    def test_three_level_nesting(self, pm, d):
        g1 = group("g1", "L1", ["g2", "g3", "a"])
        g2 = group("g2", "L2", ["g3", "b"]); g2["custom"]["parent_group_id"] = "g1"
        g3 = group("g3", "L3", ["c"]); g3["custom"]["parent_group_id"] = "g2"
        a = node("a", pgid="g1"); b = node("b", pgid="g2"); c = node("c", pgid="g3")
        flat = pm._flatten_nodes({"nodes": [g1, g2, g3, a, b, c]})
        got = {e["node"]["id"]: e["namespace"] for e in flat}
        assert got == {"a": "/L1", "b": "/L1/L2", "c": "/L1/L2/L3"} and len(flat) == 3

    def test_sibling_nested_groups(self, pm, d):
        o = group("o", "O", ["i1", "i2"])
        i1 = group("i1", "I1", ["a"]); i1["custom"]["parent_group_id"] = "o"
        i2 = group("i2", "I2", ["b"]); i2["custom"]["parent_group_id"] = "o"
        flat = pm._flatten_nodes({"nodes": [o, i1, i2, node("a", pgid="i1"), node("b", pgid="i2")]})
        assert {e["node"]["id"]: e["namespace"] for e in flat} == {"a": "/O/I1", "b": "/O/I2"}

    def test_nested_group_deploy_group_is_outermost_in_flatten_and_files(self, pm, d):
        o = group("o", "Out", ["i"]); i = group("i", "In", ["a"]); i["custom"]["parent_group_id"] = "o"
        flat = pm._flatten_nodes({"nodes": [o, i, node("a", pgid="i")]})
        assert flat[0]["container_group"] == "out"
        created = run(pm, [o, i, node("a", pgid="i")], d)
        assert created[0]["group"] == "out"

    def test_explicit_node_group_is_used_for_files(self, pm, d):
        created = run(pm, [node("a", group="robot")], d)
        assert created[0]["group"] == "robot"

    def test_explicit_node_group_beats_subgraph(self, pm, d):
        g = group("g", "Robot", ["a"])
        created = run(pm, [g, node("a", pgid="g", group="pc")], d)
        assert created[0]["group"] == "pc"

    def test_explicit_group_on_subgraph_is_inherited_by_nested_children(self, pm, d):
        o = group("o", "Out", ["i"], container="rig")
        i = group("i", "In", ["a"]); i["custom"]["parent_group_id"] = "o"
        created = run(pm, [o, i, node("a", pgid="i")], d)
        assert created[0]["group"] == "rig"


def test_flatten_failure_is_logged_not_raised(tmp_path, monkeypatch, caplog):
    """process_and_save_files called traceback.print_exc() without importing traceback:
    a broken session crashed Save with NameError instead of logging the error."""
    from core import node_sources

    def broken(*a, **k):
        raise KeyError("nodes")
    monkeypatch.setattr(node_sources, "flatten_nodes", broken)
    assert node_sources.process_and_save_files({}, str(tmp_path), "python") == []
    assert "flatten_nodes failed" in caplog.text
