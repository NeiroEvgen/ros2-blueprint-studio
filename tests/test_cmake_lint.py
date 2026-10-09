from core.cmake_lint import lint

BAD = """add_library(mission_core src/mission_core.cpp)
ament_target_dependencies(mission_core rclcpp)
target_link_libraries(mission_core PUBLIC kinematics_lib)
"""
GOOD = """add_library(mission_core src/mission_core.cpp)
ament_target_dependencies(mission_core rclcpp)
target_link_libraries(mission_core kinematics_lib)
"""


def test_mixed_flagged():
    i = lint(BAD)
    assert len(i) == 1 and i[0].target == "mission_core" and i[0].line == 3


def test_plain_ok():
    assert lint(GOOD) == []


def test_keyword_without_ament_ok():
    assert lint("target_link_libraries(t PUBLIC a)\n") == []


def test_commented_ignored():
    assert lint("ament_target_dependencies(t rclcpp)\n# target_link_libraries(t PUBLIC a)\n") == []


def test_other_target_not_flagged():
    assert lint("ament_target_dependencies(a rclcpp)\ntarget_link_libraries(b PUBLIC x)\n") == []
