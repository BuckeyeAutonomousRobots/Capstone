import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def load_yaml(package_name: str, relative_path: str):
    pkg_share = get_package_share_directory(package_name)
    path = os.path.join(pkg_share, relative_path)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _launch_setup(context, *args, **kwargs):
    ur_type_value = LaunchConfiguration("ur_type").perform(context)
    simulation_controllers_value = LaunchConfiguration("simulation_controllers").perform(context)
    initial_positions_file_value = LaunchConfiguration("initial_positions_file").perform(context)

    urdf_xacro = os.path.join(
        get_package_share_directory("ur_hande_description"), "urdf", "ur_hande.urdf.xacro"
    )
    robot_description_config = xacro.process_file(
        urdf_xacro,
        mappings={
            "name": "ur",
            "ur_type": ur_type_value,
            "use_fake_hardware": "false",
            "sim_gazebo": "false",
            "sim_ignition": "true",
            "simulation_controllers": simulation_controllers_value,
            "initial_positions_file": initial_positions_file_value,
        },
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    srdf_xacro = os.path.join(get_package_share_directory("ur_moveit_config"), "srdf", "ur.srdf.xacro")
    semantic_config = xacro.process_file(srdf_xacro, mappings={"name": "ur", "prefix": ""})
    robot_description_semantic = {"robot_description_semantic": semantic_config.toxml()}

    kin = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    robot_kin = kin.get("/**", {}).get("ros__parameters", {}).get("robot_description_kinematics", {})
    robot_description_kinematics = {"robot_description_kinematics": robot_kin}

    servo_yaml = load_yaml("servo_test_config", "config/servo_gz.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    joint_states_filter_node = Node(
        package="servo_test_config",
        executable="joint_states_filter",
        name="joint_states_filter",
        output="screen",
        parameters=[
            {
                "source_topic": "/joint_states",
                "output_topic": "/joint_states_servo",
                "drop_suffixes": ["_mimic"],
            }
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return [joint_states_filter_node, servo_node]


def generate_launch_description():
    default_initial_positions = os.path.join(
        get_package_share_directory("servo_test_config"), "config", "initial_positions.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument(
                "simulation_controllers",
                default_value="/home/noah/ws_moveit/simulation/config/ur5e_gz_controllers.yaml",
            ),
            DeclareLaunchArgument("initial_positions_file", default_value=default_initial_positions),
            OpaqueFunction(function=_launch_setup),
        ]
    )
