"""
0.2 — Регрессионный тест на first-save race.

Баг был: код ноды не сохранялся с первого раза / перезаписывался
версией из графа, если файл уже правили снаружи (в редакторе).
Фикс: в _process_and_save_files диск — источник истины. Если файл на
диске существует и не пуст, он побеждает code_content из графа.

Проверка регрессии: закомментируй блок «disk re-read» в
_process_and_save_files — тест test_disk_wins_over_graph_code покраснеет.
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
        """Главный регрессионный сценарий."""
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
        """create → save → reload: код совпадает с тем, что было в графе."""
        code = "// roundtrip\nint main(){return 0;}\n"
        pm._process_and_save_files(_session(_cpp_node("Rt", code=code)),
                                   str(code_dir), "cpp")

        reloaded = _cpp_node("Rt", code="", source_file="Rt.cpp")
        pm._process_and_save_files(_session(reloaded), str(code_dir), "cpp")

        assert reloaded["custom"]["code_content"] == code
