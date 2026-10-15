"""Шаблоны нод палитры заполняются перед записью файла; поток colcon делится на строки."""
import pytest

from core import colcon
from core.template_render import render_pristine_template
from nodes.templates import CPP_TEMPLATES, PYTHON_TEMPLATES, template_name


def test_template_name_mapping():
    assert template_name("cpp_string_pub") == "Publisher"
    assert template_name("py_string_sub") == "Subscriber"
    assert template_name("cpp_custom") == "Custom"
    assert template_name("whatever") == "Default"


@pytest.mark.parametrize("key,is_cpp", [
    ("cpp_string_pub", True), ("cpp_string_sub", True), ("py_string_pub", False),
    ("py_string_sub", False), ("cpp_timer", True), ("py_timer", False), ("cpp_lifecycle", True)])
def test_pristine_templates_have_no_placeholders_left(key, is_cpp):
    table = CPP_TEMPLATES if is_cpp else PYTHON_TEMPLATES
    code = table[template_name(key)]
    out = render_pristine_template(
        code, {"template_key": key, "node_name": "my node", "topic_name": "/t"}, "my node", is_cpp)
    for ph in ("{class_name}", "{node_name}", "{topic_name}", "{EXEC_", "{interval}"):
        assert ph not in out, ph
    assert "my node" in out or "my_node" in out


def test_cpp_publisher_content():
    code = CPP_TEMPLATES["Publisher"]
    out = render_pristine_template(code, {"template_key": "cpp_string_pub",
                                          "node_name": "talker", "topic_name": "/chat"}, "talker", True)
    assert "class talker : public rclcpp::Node" in out
    assert 'create_publisher<std_msgs::msg::String>("/chat"' in out
    assert "std::make_shared<talker>()" in out


def test_user_edited_code_is_left_alone():
    code = "print(f'{node_name}')  # my code {class_name}"
    assert render_pristine_template(code, {"template_key": "py_string_pub"}, "n", False) == code


def test_no_template_key_is_left_alone():
    assert render_pristine_template("x {class_name}", {}, "n", False) == "x {class_name}"


def test_numeric_class_name_gets_prefix():
    out = render_pristine_template(CPP_TEMPLATES["Publisher"],
                                   {"template_key": "cpp_string_pub", "node_name": "1node"}, "1node", True)
    assert "class N1node" in out


def test_process_flat_nodes_writes_rendered_file(tmp_path):
    from core.node_sources import process_flat_nodes
    node = {"id": "1", "name": "StringPub", "type_": "ros.cpp.CppStringPubNode", "ports": [],
            "custom": {"node_name": "talker", "topic_name": "/chat", "template_key": "cpp_string_pub",
                       "code_content": CPP_TEMPLATES["Publisher"], "source_file": ""}}
    process_flat_nodes([{"node": node, "namespace": ""}], str(tmp_path), "cpp")
    text = (tmp_path / "StringPub.cpp").read_text(encoding="utf-8")
    assert "{class_name}" not in text and "class talker" in text


def test_broken_file_from_previous_build_is_repaired(tmp_path):
    from core.node_sources import process_flat_nodes
    (tmp_path / "StringPub.cpp").write_text(CPP_TEMPLATES["Publisher"], encoding="utf-8")   # старый сломанный
    node = {"id": "1", "name": "StringPub", "type_": "ros.cpp.CppStringPubNode", "ports": [],
            "custom": {"node_name": "talker", "template_key": "cpp_string_pub",
                       "code_content": CPP_TEMPLATES["Publisher"], "source_file": "StringPub.cpp"}}
    process_flat_nodes([{"node": node, "namespace": ""}], str(tmp_path), "cpp")
    assert "{class_name}" not in (tmp_path / "StringPub.cpp").read_text(encoding="utf-8")


def test_iter_lines_splits_multiline_and_partial_chunks():
    chunks = [b"abc\nFailed   <<< gfds [2s]\nde", b"f\r\n", b"tail"]
    assert list(colcon.iter_lines(chunks)) == ["abc", "Failed   <<< gfds [2s]", "def", "tail"]


def test_failed_package_found_in_multiline_chunk():
    lines = list(colcon.iter_lines([b"--- stderr: gfds\nerr\n---\nFailed   <<< gfds [2.65s, exited with code 2]\n"]))
    assert colcon.parse_output(lines).failed == ["gfds"]
