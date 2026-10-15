import json
import os
import pytest
from core import i18n
from tools import i18n_check


class TestTranslate:
    def test_basic_and_params(self):
        assert i18n.t("colcon.build_failed", packages="x") == "Build FAILED: x"

    def test_switch_language(self):
        assert i18n.set_language("ru")
        assert i18n.t("build.aborted").startswith("Сборка")

    def test_unknown_language_rejected_and_unchanged(self):
        i18n.set_language("ru")
        assert i18n.set_language("xx") is False and i18n.get_language() == "ru"

    def test_locale_codes_normalized(self):
        assert i18n.set_language("ru_RU") and i18n.get_language() == "ru"
        assert i18n.set_language("en-US") and i18n.get_language() == "en"

    def test_missing_key_returns_key(self):
        assert i18n.t("no.such.key") == "no.such.key"

    def test_missing_param_left_in_place_no_crash(self):
        assert i18n.t("colcon.build_failed") == "Build FAILED: {packages}"

    def test_extra_params_ignored(self):
        assert i18n.t("build.finished", unused=1) == "Build phase finished."

    def test_fallback_to_english_for_missing_key(self, tmp_path, monkeypatch):
        (tmp_path / "en.json").write_text(json.dumps({"a": "english", "b": "only en"}), encoding="utf-8")
        (tmp_path / "de.json").write_text(json.dumps({"_meta": {"name": "Deutsch"}, "a": "deutsch"}), encoding="utf-8")
        monkeypatch.setattr(i18n, "LOCALES_DIR", str(tmp_path))
        i18n.reload()
        try:
            assert i18n.set_language("de")
            assert i18n.t("a") == "deutsch" and i18n.t("b") == "only en"
            assert {"code": "de", "name": "Deutsch"} in i18n.available_languages()
        finally:
            monkeypatch.undo(); i18n.reload()

    def test_new_language_by_dropping_a_file(self, tmp_path, monkeypatch):
        (tmp_path / "en.json").write_text(json.dumps({"k": "hello"}), encoding="utf-8")
        monkeypatch.setattr(i18n, "LOCALES_DIR", str(tmp_path)); i18n.reload()
        assert i18n.set_language("fr") is False
        (tmp_path / "fr.json").write_text(json.dumps({"k": "bonjour"}), encoding="utf-8"); i18n.reload()
        assert i18n.set_language("fr") and i18n.t("k") == "bonjour"
        monkeypatch.undo(); i18n.reload()

    def test_broken_json_does_not_crash(self, tmp_path, monkeypatch):
        (tmp_path / "en.json").write_text(json.dumps({"k": "ok"}), encoding="utf-8")
        (tmp_path / "bad.json").write_text("{not json", encoding="utf-8")
        monkeypatch.setattr(i18n, "LOCALES_DIR", str(tmp_path)); i18n.reload()
        assert i18n.set_language("bad") is False and i18n.t("k") == "ok"
        monkeypatch.undo(); i18n.reload()

    def test_braces_in_translation_do_not_crash(self, tmp_path, monkeypatch):
        (tmp_path / "en.json").write_text(json.dumps({"k": "a { b"}), encoding="utf-8")
        monkeypatch.setattr(i18n, "LOCALES_DIR", str(tmp_path)); i18n.reload()
        assert i18n.t("k", x=1) == "a { b"
        monkeypatch.undo(); i18n.reload()

    def test_env_language_detection(self, monkeypatch):
        monkeypatch.setenv("STUDIO_LANG", "ru_RU.UTF-8")
        assert i18n.detect_system_language() == "ru"


class TestPlural:
    @pytest.mark.parametrize("n,exp", [(1, "1 пакет"), (2, "2 пакета"), (5, "5 пакетов"), (11, "11 пакетов"),
                                       (21, "21 пакет"), (22, "22 пакета"), (111, "111 пакетов"), (0, "0 пакетов")])
    def test_russian(self, n, exp):
        i18n.set_language("ru")
        assert i18n.t_plural("plural.packages", n) == exp

    @pytest.mark.parametrize("n,exp", [(1, "1 package"), (2, "2 packages"), (0, "0 packages")])
    def test_english(self, n, exp):
        assert i18n.t_plural("plural.packages", n) == exp

    def test_missing_plural_key_returns_key(self):
        assert i18n.t_plural("nope", 3) == "nope"


class TestDictionaries:
    def test_dictionaries_consistent(self):
        assert i18n_check.check() == []

    def test_checker_detects_missing_and_placeholder_mismatch(self, tmp_path, monkeypatch):
        d = tmp_path / "loc"; d.mkdir()
        (d / "en.json").write_text(json.dumps({"a": "x {n}", "b": "y"}), encoding="utf-8")
        (d / "ru.json").write_text(json.dumps({"a": "икс {m}", "c": "лишний"}), encoding="utf-8")
        monkeypatch.setattr(i18n_check, "LOCALES", str(d))
        monkeypatch.setattr(i18n_check, "used_keys", lambda *a, **k: ({}, {}))
        probs = "\n".join(i18n_check.check())
        assert "нет перевода: b" in probs and "лишний ключ" in probs and "плейсхолдеры a" in probs

    def test_checker_detects_undefined_key_in_code(self, tmp_path, monkeypatch):
        d = tmp_path / "loc"; d.mkdir()
        (d / "en.json").write_text(json.dumps({"a": "x"}), encoding="utf-8")
        monkeypatch.setattr(i18n_check, "LOCALES", str(d))
        monkeypatch.setattr(i18n_check, "used_keys", lambda *a, **k: ({"zzz": "f.py:1"}, {"pl": "f.py:2"}))
        probs = "\n".join(i18n_check.check())
        assert "zzz" in probs and "pl.other" in probs

    def test_used_keys_scanner_finds_calls(self, tmp_path):
        (tmp_path / "m.py").write_text("from core.i18n import t\nx = t('a.b', n=1)\ny = i18n.t_plural('p', 2)\n", encoding="utf-8")
        plain, plural = i18n_check.used_keys(str(tmp_path))
        assert "a.b" in plain and "p" in plural

    def test_all_core_messages_translated_in_every_language(self):
        loc = i18n_check.load_locales()
        for lang, d in loc.items():
            for k in loc["en"]:
                if not k.endswith((".one", ".other", ".few", ".many")):
                    assert k in d, (lang, k)
