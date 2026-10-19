import os
import pytest
from core import settings, i18n


def test_defaults_when_no_file():
    assert settings.get("language") is None


def test_roundtrip(tmp_path):
    assert settings.set("language", "ru") is True
    assert settings.get("language") == "ru"
    assert os.path.isfile(settings.settings_path())


def test_corrupt_file_gives_defaults():
    os.makedirs(settings.settings_dir(), exist_ok=True)
    open(settings.settings_path(), "w").write("{broken")
    assert settings.load() == settings.DEFAULTS


def test_unknown_keys_in_file_ignored():
    os.makedirs(settings.settings_dir(), exist_ok=True)
    open(settings.settings_path(), "w", encoding="utf-8").write('{"language":"ru","evil":1}')
    assert settings.load() == {**settings.DEFAULTS, "language": "ru"}


def test_set_unknown_key_rejected():
    with pytest.raises(KeyError):
        settings.set("nope", 1)


def test_unwritable_dir_returns_false(monkeypatch, tmp_path):
    f = tmp_path / "file"; f.write_text("x")
    monkeypatch.setenv("STUDIO_SETTINGS_DIR", str(f / "sub"))     # the parent is a file
    assert settings.set("language", "ru") is False


def test_apply_language():
    settings.set("language", "ru")
    assert settings.apply_language() == "ru" and i18n.get_language() == "ru"


def test_apply_language_unknown_keeps_current():
    settings.set("language", "xx")
    i18n.set_language("en")
    assert settings.apply_language() == "en"


def test_cyrillic_home_dir_ok(monkeypatch, tmp_path):
    monkeypatch.setenv("STUDIO_SETTINGS_DIR", str(tmp_path / "Евгений" / ".studio"))
    assert settings.set("language", "ru") and settings.get("language") == "ru"
