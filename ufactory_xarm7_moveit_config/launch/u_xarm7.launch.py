from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

# from launch.actions import DeclareLaunchArgument, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoi
# from launch_ros.substitutions import FindPackageShare
# import xacro
# import yaml


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ufactory-xarm7-robot", package_name="ufactory_xarm7_moveit_config")
        .robot_description(file_path="config/ufactory-xarm7-robot.urdf.xacro")
        .robot_description_semantic(file_path="config/ufactory-xarm7-robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines = ["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ufactory_xarm7_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # Spawner for Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner for Arm Trajectory Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm7_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"ros_args": ["--log-level", "info"]},
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("ufactory_xarm7_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    return LaunchDescription(
        [
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
        ]
    )