"""
Regression test for the first-save race.

The bug: a node's code was not saved the first time / was overwritten
by the graph version if the file had already been edited outside (in an editor).
Fix: in _process_and_save_files the disk is the source of truth. If the file
exists on disk and is not empty, it wins over code_content from the graph.

Regression check: comment out the "disk re-read" block in
_process_and_save_files and test_disk_wins_over_graph_code turns red.
"""
import os
import pytest


def _session(*nodes):
    return {"nodes": list(nodes)}


def _cpp_node(nid, code="", source_file=""):
    return {
        "id": nid,
        "type_": "ros.cpp.CppCustomNode",
        "name": nid,
        "custom": {"node_name": nid, "code_content": code, "source_file": source_file},
        "ports": [],
    }


@pytest.fixture
def code_dir(tmp_path):
    d = tmp_path / "proj" / "src" / "cpp"
    d.mkdir(parents=True)
    return d


class TestFirstSave:

    def test_new_node_gets_generated_file(self, pm, code_dir):
        node = _cpp_node("Fresh")

        created = pm._process_and_save_files(_session(node), str(code_dir), "cpp")

        assert [c["filename"] for c in created] == ["Fresh.cpp"]
        path = code_dir / "Fresh.cpp"
        assert path.exists() and path.read_text(encoding="utf-8").strip()
        assert node["custom"]["source_file"] == "Fresh.cpp"

    def test_graph_code_written_when_no_file_on_disk(self, pm, code_dir):
        node = _cpp_node("Mine", code="// from graph\nint main(){return 0;}\n")

        pm._process_and_save_files(_session(node), str(code_dir), "cpp")

        assert (code_dir / "Mine.cpp").read_text(encoding="utf-8") == \
            "// from graph\nint main(){return 0;}\n"

    def test_disk_wins_over_graph_code(self, pm, code_dir):
        """The main regression scenario."""
        (code_dir / "Edited.cpp").write_text("// edited in external editor\n",
                                             encoding="utf-8")
        node = _cpp_node("Edited", code="// stale graph version\n",
                         source_file="Edited.cpp")

        pm._process_and_save_files(_session(node), str(code_dir), "cpp")

        on_disk = (code_dir / "Edited.cpp").read_text(encoding="utf-8")
        assert on_disk == "// edited in external editor\n"
        assert node["custom"]["code_content"] == "// edited in external editor\n"

    def test_empty_file_on_disk_does_not_win(self, pm, code_dir):
        (code_dir / "Blank.cpp").write_text("   \n", encoding="utf-8")
        node = _cpp_node("Blank", code="// real code\n", source_file="Blank.cpp")

        pm._process_and_save_files(_session(node), str(code_dir), "cpp")

        assert (code_dir / "Blank.cpp").read_text(encoding="utf-8") == "// real code\n"

    def test_save_then_reload_roundtrip(self, pm, code_dir):
        """create -> save -> reload: the code matches what was in the graph."""
        code = "// roundtrip\nint main(){return 0;}\n"
        pm._process_and_save_files(_session(_cpp_node("Rt", code=code)),
                                   str(code_dir), "cpp")

        reloaded = _cpp_node("Rt", code="", source_file="Rt.cpp")
        pm._process_and_save_files(_session(reloaded), str(code_dir), "cpp")

        assert reloaded["custom"]["code_content"] == code
