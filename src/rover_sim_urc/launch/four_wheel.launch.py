"""Suspend an isolated ROS-control derivative of the existing four-wheel SDF.

The original SDF remains untouched. Copy its link geometry/inertia and joint
frames/axes into URDF for gazebo_ros2_control; add only a world fixture,
ROS interfaces, and simulation effort/velocity caps. No steering or kinematics.
"""
import copy
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

NAMESPACE = "assignment1_four_wheel"
JOINTS = [side + "_wheel_joint" for side in ["front_left", "rear_left", "front_right", "rear_right"]]

def description_from_sdf(share):
    """Convert only the explicitly supported source layout, refusing structural changes."""
    model = ET.parse(os.path.join(share, "models/lusi_rover/model.sdf")).getroot().find("model")
    joints = {j.get("name"): j for j in model.findall("joint")}
    assert set(joints) == set(JOINTS), "Unexpected source joints"
    assert {l.get("name") for l in model.findall("link")} == {"chassis", *[j.findtext("child") for j in joints.values()]}
    robot = ET.Element("robot", name="assignment1_four_wheel")
    ET.SubElement(robot, "link", name="world")
    mount = ET.SubElement(robot, "joint", name="four_wheel_stand_mount", type="fixed")
    ET.SubElement(mount, "parent", link="world"); ET.SubElement(mount, "child", link="chassis")
    ET.SubElement(mount, "origin", xyz="0 0 0.65")
    for source in model.findall("link"):
        link = ET.SubElement(robot, "link", name=source.get("name"))
        inertia = source.find("inertial")
        dest = ET.SubElement(link, "inertial")
        ET.SubElement(dest, "mass", value=inertia.findtext("mass"))
        ET.SubElement(dest, "inertia", **{n: inertia.findtext("inertia/" + n) for n in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]})
        for kind in ["visual", "collision"]:
            item = source.find(kind)
            if item is None: continue
            dest = ET.SubElement(link, kind)
            pose = item.findtext("pose", "0 0 0 0 0 0").split()
            ET.SubElement(dest, "origin", xyz=" ".join(pose[:3]), rpy=" ".join(pose[3:]))
            geom = ET.SubElement(dest, "geometry")
            box = item.find("geometry/box")
            cylinder = item.find("geometry/cylinder")
            if box is not None: ET.SubElement(geom, "box", size=box.findtext("size"))
            elif cylinder is not None: ET.SubElement(geom, "cylinder", radius=cylinder.findtext("radius"), length=cylinder.findtext("length"))
            else: raise ValueError("Unsupported source geometry")
    for name in JOINTS:
        j = joints[name]
        assert j.get("type") == "revolute" and j.findtext("parent") == "chassis"
        assert j.find("pose") is None, "Joint frame conversion needs review"
        child = j.findtext("child")
        pose = model.find("link[@name='" + child + "']/pose").text.split()
        assert pose[3:] == ["0", "0", "0"], "Rotated source wheel frame needs review"
        dest = ET.SubElement(robot, "joint", name=name, type="revolute")
        ET.SubElement(dest, "parent", link="chassis"); ET.SubElement(dest, "child", link=child)
        ET.SubElement(dest, "origin", xyz=" ".join(pose[:3]))
        ET.SubElement(dest, "axis", xyz=j.findtext("axis/xyz"))
        ET.SubElement(dest, "limit", lower=j.findtext("axis/limit/lower"), upper=j.findtext("axis/limit/upper"), effort="20", velocity="6")
    control = ET.SubElement(robot, "ros2_control", name="FourWheelGazeboSystem", type="system")
    ET.SubElement(ET.SubElement(control, "hardware"), "plugin").text = "gazebo_ros2_control/GazeboSystem"
    for name in JOINTS:
        j = ET.SubElement(control, "joint", name=name)
        interface = ET.SubElement(j, "command_interface", name="velocity")
        ET.SubElement(interface, "param", name="min").text = "-6"
        ET.SubElement(interface, "param", name="max").text = "6"
        ET.SubElement(j, "state_interface", name="position"); ET.SubElement(j, "state_interface", name="velocity")
    plugin = ET.SubElement(ET.SubElement(robot, "gazebo"), "plugin", name="gazebo_ros2_control", filename="libgazebo_ros2_control.so")
    ET.SubElement(ET.SubElement(plugin, "ros"), "namespace").text = "/" + NAMESPACE
    ET.SubElement(plugin, "robot_param").text = "robot_description"
    ET.SubElement(plugin, "robot_param_node").text = "robot_state_publisher"
    ET.SubElement(plugin, "parameters").text = os.path.join(share, "config/four_wheel_controllers.yaml")
    return ET.tostring(robot, encoding="unicode")

def generate_launch_description():
    share = get_package_share_directory("rover_sim_urc")
    description = description_from_sdf(share)
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch/gazebo.launch.py")), launch_arguments={"world": os.path.join(share, "worlds/assignment1.world"), "gui": "false", "verbose": "true"}.items())
    state = Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=NAMESPACE, name="robot_state_publisher", parameters=[{"robot_description": description, "use_sim_time": True}], remappings=[("joint_states", "joint_state_broadcaster/joint_states")])
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py", output="screen", arguments=["-entity", NAMESPACE, "-topic", "/" + NAMESPACE + "/robot_description", "-timeout", "60"])
    controllers = [Node(package="controller_manager", executable="spawner", output="screen", arguments=[name, "--controller-manager", "/" + NAMESPACE + "/controller_manager", "--controller-manager-timeout", "60", "--switch-timeout", "30"]) for name in ["joint_state_broadcaster", "wheel_velocity_controller"]]
    bridge = Node(package="rover_sim_urc", executable="four_wheel_bridge.py", output="screen")
    gate = Node(package="rover_sim_urc", executable="four_wheel_gate.py", output="screen",
                sigterm_timeout="0.5", sigkill_timeout="0.5")
    monitor = Node(package="rover_sim_urc", executable="gate_health_monitor.py", output="screen")
    def after_spawn(event, context):
        return controllers if event.returncode == 0 else [EmitEvent(event=Shutdown(reason="Four-wheel spawn failed"))]
    actions = [RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn))]
    for process in [bridge, gate, monitor]:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process, on_exit=[EmitEvent(event=Shutdown(reason="Four-wheel command process exited"))])))
    return LaunchDescription(actions + [gazebo, state, spawn, monitor, gate, bridge])
