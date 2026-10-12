"""Create a task-local copy of vendor bringup without joystick control."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path.home() / "rosmaster_x3_deploy" / "live_test"
root.mkdir(parents=True, exist_ok=True)
source = Path.home() / "yahboomcar_ws/src/yahboomcar_bringup/launch/bringup.launch"
tree = ET.parse(str(source))
for node in list(tree.getroot()):
    if node.tag == "include" and "yahboomcar_ctrl" in node.get("file", ""):
        tree.getroot().remove(node)
tree.write(str(root / "bringup_no_joy.launch"), encoding="utf-8", xml_declaration=True)
launch = ET.Element("launch")
ET.SubElement(launch, "include", file=str(root / "bringup_no_joy.launch"))
ET.SubElement(launch, "include", file="$(find rplidar_ros)/launch/rplidar.launch")
ET.SubElement(launch, "node", pkg="tf", type="static_transform_publisher",
              name="base_link_to_laser", args="0.0435 0.00005258 0.11 3.1416 0 0 base_link laser 30")
ET.ElementTree(launch).write(str(root / "sensors.launch"), encoding="utf-8", xml_declaration=True)
print(str(root / "sensors.launch"))
