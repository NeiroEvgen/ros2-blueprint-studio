"""Import closure: local files, packages, relative imports, stdlib/third-party, optional, dynamic."""
import os

from core.import_closure import compute


def w(root, rel, text=""):
    p = root / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding="utf-8")
    return str(p)


def test_follows_local_modules_and_classifies_rest(tmp_path):
    entry = w(tmp_path, "node.py", "import os, numpy\nimport helpers\nfrom lib import util\n")
    w(tmp_path, "helpers.py", "import json\n")
    w(tmp_path, "lib/__init__.py")
    w(tmp_path, "lib/util.py", "import yaml\n")
    w(tmp_path, "unused.py")
    c = compute(entry)
    assert set(c.local) == {"node.py", "helpers.py", os.path.join("lib", "__init__.py"),
                            os.path.join("lib", "util.py")}
    assert c.third_party == {"numpy", "yaml"}
    assert {"os", "json"} <= c.stdlib


def test_relative_imports(tmp_path):
    entry = w(tmp_path, "pkg/main.py", "from . import a\nfrom .b import f\nfrom ..outside import g\n")
    w(tmp_path, "pkg/__init__.py")
    w(tmp_path, "pkg/a.py")
    w(tmp_path, "pkg/b.py")
    c = compute(entry, [str(tmp_path)])
    assert os.path.join("pkg", "a.py") in c.local and os.path.join("pkg", "b.py") in c.local
    assert any("..outside" in u for u in c.unresolved)


def test_optional_and_dynamic(tmp_path):
    entry = w(tmp_path, "n.py",
              "try:\n    import cv2\nexcept ImportError:\n    cv2 = None\n"
              "import importlib\nname = 'x'\nimportlib.import_module(name)\n"
              "importlib.import_module('literal_mod')\n")
    c = compute(entry)
    assert c.optional == {"cv2"}
    assert "cv2" not in c.third_party
    assert len(c.dynamic) == 1 and c.dynamic[0].startswith("n.py:")
    assert "literal_mod" in c.third_party


def test_cycles_terminate_and_syntax_errors_reported(tmp_path):
    entry = w(tmp_path, "a.py", "import b\n")
    w(tmp_path, "b.py", "import a\nimport broken\n")
    w(tmp_path, "broken.py", "def (:\n")
    c = compute(entry)
    assert set(c.local) == {"a.py", "b.py", "broken.py"}
    assert c.errors and "broken.py" in c.errors[0]


def test_cli_exit_code_and_json(tmp_path, capsys):
    from tools import import_closure as cli
    entry = w(tmp_path, "n.py", "import os\n")
    assert cli.main([entry, "--json"]) == 0
    assert '"stdlib"' in capsys.readouterr().out
