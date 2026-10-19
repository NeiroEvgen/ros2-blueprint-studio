import copy
import pytest
import yaml
from core import graph_ir as ir
from core import state_store as ss


def n(nid, t, pos=(0, 0), **custom):
    return {"type_": t, "name": nid, "pos": list(pos), "custom": dict(custom), "id": nid}


def conn(a, b, pa="out", pb="in"):
    return {"out": [a, pa], "in": [b, pb]}


class TestLanguage:
    @pytest.mark.parametrize("t,exp", [
        ("ros.cpp.CppCustomNode", "cpp"), ("ros.py.PyCustomNode", "python"),
        ("ros.cpp.CppTimerNode", "cpp"), ("ros.nodes.RosGroup", None),
        ("ros.nodes.meta.NoteNode", None), ("nodes.utility.MonitorNode", None),
        ("ros.nodes.internal.SubGraphInputNode", None), ("", None),
    ])
    def test_by_type(self, t, exp):
        assert ir.language_of({"type_": t}) == exp

    def test_explicit_language_wins(self):
        assert ir.language_of({"type_": "ros.cpp.X", "custom": {"language": "python"}}) == "python"

    def test_legacy_type_key(self):
        assert ir.language_of({"type": "ros.py.A"}) == "python"

    def test_project_languages(self):
        assert ir.project_languages("cpp") == ("cpp",)
        assert ir.project_languages("python") == ("python",)
        assert ir.project_languages("mixed") == ("cpp", "python")
        assert ir.default_language("mixed") == "cpp" and ir.default_language("python") == "python"

    def test_partition_neutral_goes_to_default(self):
        flat = [{"node": n("a", "ros.cpp.X")}, {"node": n("b", "ros.py.Y")}, {"node": n("c", "user.Custom")}]
        out = ir.partition_by_language(flat, "python")
        assert [e["node"]["id"] for e in out["cpp"]] == ["a"]
        assert [e["node"]["id"] for e in out["python"]] == ["b", "c"]


PY = {"nodes": {"p1": n("p1", "ros.py.PyCustomNode", (0, 0)), "p2": n("p2", "ros.py.PyCustomNode", (100, 200))},
      "connections": [conn("p1", "p2")], "graph": {"acyclic": True}}
CPP = {"nodes": {"c1": n("c1", "ros.cpp.CppCustomNode", (10, 20)), "c2": n("c2", "ros.cpp.CppCustomNode", (50, 90))},
       "connections": [conn("c1", "c2")], "graph": {"acyclic": False}}


class TestMergeSplit:
    def test_merge_has_all_nodes_and_connections(self):
        m, meta = ir.merge_sessions(PY, CPP)
        assert set(m["nodes"]) == {"p1", "p2", "c1", "c2"} and len(m["connections"]) == 2

    def test_cpp_shifted_below_python(self):
        m, meta = ir.merge_sessions(PY, CPP)
        py_bottom = max(m["nodes"][i]["pos"][1] for i in ("p1", "p2"))
        cpp_top = min(m["nodes"][i]["pos"][1] for i in ("c1", "c2"))
        assert cpp_top > py_bottom

    def test_roundtrip_exact(self):
        m, meta = ir.merge_sessions(PY, CPP)
        py, cpp = ir.split_session(m, meta)
        assert py == PY and cpp == CPP

    def test_inputs_not_mutated(self):
        a, b = copy.deepcopy(PY), copy.deepcopy(CPP)
        ir.merge_sessions(a, b)
        assert a == PY and b == CPP

    def test_id_collision_is_remapped_and_restored(self):
        cpp = {"nodes": {"p1": n("p1", "ros.cpp.CppCustomNode", (0, 0)), "c2": n("c2", "ros.cpp.CppCustomNode")},
               "connections": [conn("p1", "c2")]}
        m, meta = ir.merge_sessions(PY, cpp)
        assert len(m["nodes"]) == 4 and meta["remap"] == {"p1": "p1_cpp2"}
        assert m["connections"][1]["out"][0] == "p1_cpp2"
        py, c = ir.split_session(m, meta)
        assert py == PY and c == cpp

    def test_group_references_remapped(self):
        g = n("p1", "ros.nodes.RosGroup", internal_nodes=["c2"])
        child = n("c2", "ros.cpp.CppCustomNode", parent_group_id="p1")
        cpp = {"nodes": {"p1": g, "c2": child}, "connections": []}
        m, meta = ir.merge_sessions(PY, cpp)
        new = meta["remap"]["p1"]
        assert m["nodes"][new]["custom"]["internal_nodes"] == ["c2"]
        assert m["nodes"]["c2"]["custom"]["parent_group_id"] == new
        py, c = ir.split_session(m, meta)
        assert c == cpp

    def test_empty_sides(self):
        for a, b in ((PY, {}), ({}, CPP), ({}, {})):
            m, meta = ir.merge_sessions(a, b)
            py, cpp = ir.split_session(m, meta)
            assert len(py.get("nodes", {})) == len((a or {}).get("nodes", {}))
            assert len(cpp.get("nodes", {})) == len((b or {}).get("nodes", {}))

    def test_list_format_nodes(self):
        a = {"nodes": [n("a", "ros.py.A")], "connections": []}
        b = {"nodes": [n("b", "ros.cpp.B")], "connections": []}
        m, meta = ir.merge_sessions(a, b)
        assert isinstance(m["nodes"], list) and len(m["nodes"]) == 2
        py, cpp = ir.split_session(m, meta)
        assert py == a and cpp == b

    def test_cross_links_dropped_on_split_and_reported(self):
        m, meta = ir.merge_sessions(PY, CPP)
        m["connections"].append(conn("p1", "c1"))
        assert ir.cross_links(m, meta) == [conn("p1", "c1")]
        py, cpp = ir.split_session(m, meta)
        assert py == PY and cpp == CPP     # a cross-language link leaked in neither direction

    def test_cyrillic_and_tuples_survive(self):
        py = {"nodes": {"a": n("a", "ros.py.A", (0, 0), color=(1, 2, 3), title="Привет")}, "connections": []}
        m, meta = ir.merge_sessions(py, {})
        assert m["nodes"]["a"]["custom"]["title"] == "Привет"


V36DOC = {"meta": {"version": "3.6"}, "graphs": {"python": PY, "cpp": CPP}}


class TestStateStore:
    def test_parse_v36(self):
        st = ss.parse_state(copy.deepcopy(V36DOC))
        assert st.version == "3.6" and st.py == PY and st.cpp == CPP and len(st.merged["nodes"]) == 4

    def test_v36_to_v4_and_back(self, tmp_path):
        d = tmp_path / "proj"; (d / ".blueprint").mkdir(parents=True)
        (d / ".blueprint" / "state.yaml").write_text(yaml.dump(V36DOC, allow_unicode=True), encoding="utf-8")
        st = ss.read_state(str(d))
        ss.write_state_v4(str(d), st.merged, st.merge_meta)
        assert (d / ".blueprint" / "state.v3.6.yaml.bak").is_file()
        st2 = ss.read_state(str(d))
        assert st2.version == "4.0" and st2.py == PY and st2.cpp == CPP

    def test_backup_made_once_and_never_overwritten(self, tmp_path):
        d = tmp_path / "proj"; (d / ".blueprint").mkdir(parents=True)
        (d / ".blueprint" / "state.yaml").write_text(yaml.dump(V36DOC), encoding="utf-8")
        st = ss.read_state(str(d))
        ss.write_state_v4(str(d), st.merged, st.merge_meta)
        bak = (d / ".blueprint" / "state.v3.6.yaml.bak").read_text(encoding="utf-8")
        ss.write_state_v4(str(d), {"nodes": {}, "connections": []}, {"cpp_ids": [], "cpp_dy": 0, "remap": {}})
        assert (d / ".blueprint" / "state.v3.6.yaml.bak").read_text(encoding="utf-8") == bak

    def test_read_does_not_modify_v36_file(self, tmp_path):
        d = tmp_path / "proj"; (d / ".blueprint").mkdir(parents=True)
        f = d / ".blueprint" / "state.yaml"
        f.write_text(yaml.dump(V36DOC), encoding="utf-8")
        before = f.read_bytes()
        ss.read_state(str(d))
        assert f.read_bytes() == before and not list((d / ".blueprint").glob("*.bak"))

    def test_missing_state_returns_none(self, tmp_path):
        assert ss.read_state(str(tmp_path)) is None

    def test_tuples_roundtrip_through_file(self, tmp_path):
        py = {"nodes": {"a": n("a", "ros.py.A", color=(10, 20, 30))}, "connections": []}
        m, meta = ir.merge_sessions(py, {})
        ss.write_state_v4(str(tmp_path), m, meta)
        st = ss.read_state(str(tmp_path))
        assert list(st.py["nodes"]["a"]["custom"]["color"]) == [10, 20, 30]

    def test_garbage_state_rejected(self):
        with pytest.raises(ValueError):
            ss.parse_state(["not", "a", "dict"])

    def test_v36_without_meta_treated_as_v36(self):
        st = ss.parse_state({"graphs": {"python": PY, "cpp": {}}})
        assert st.version == "3.6" and st.py == PY
