"""Static safety checks for the ROS1 RGB-D package; no ROS install required."""
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros1" / "x3_target_navigation"


def test_catkin_package_has_required_runtime_files():
    required = [
        "package.xml",
        "CMakeLists.txt",
        "msg/TargetEstimate.msg",
        "launch/target_navigation.launch",
        "config/astra_pro.yaml",
        "src/rgbd_target_node.cpp",
        "src/target_fusion_node.cpp",
        "src/target_task_manager.cpp",
    ]
    assert all((PACKAGE / path).is_file() for path in required)
    assert ET.parse(PACKAGE / "package.xml").getroot().tag == "package"
    assert ET.parse(PACKAGE / "launch/target_navigation.launch").getroot().tag == "launch"


def test_only_confirmed_task_manager_can_publish_approach_velocity():
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (PACKAGE / "src").glob("*.cpp")
        if path.name != "target_task_manager.cpp"
    )
    assert "cmd_vel" not in sources
    assert "geometry_msgs/Twist" not in sources
    manager = (PACKAGE / 'src/target_task_manager.cpp').read_text(encoding='utf-8')
    assert "MoveBaseAction" in manager
    assert '"/move_base/make_plan"' in manager
    assert 'advertise<geometry_msgs::Twist>("approach_velocity", 1)' in manager
    assert 'obstacle_stop_approach_ && state_ == "NAVIGATING"' in manager


def test_execution_is_disabled_by_default_and_requires_service():
    launch = (PACKAGE / "launch" / "target_navigation.launch").read_text(
        encoding="utf-8"
    )
    manager = (PACKAGE / "src" / "target_task_manager.cpp").read_text(
        encoding="utf-8"
    )
    assert '<arg name="execute" default="false"/>' in launch
    assert '"confirm"' in manager
    assert "one-shot confirmed goal sent" in manager
    assert "execute_enabled is false; preview only" in manager


def test_vendor_stacks_are_opt_in():
    launch = (PACKAGE / "launch" / "target_navigation.launch").read_text(
        encoding="utf-8"
    )
    assert '<arg name="start_astra" default="false"/>' in launch
    assert '<arg name="start_navigation" default="false"/>' in launch
