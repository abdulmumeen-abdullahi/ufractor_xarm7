import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Use tutorial RViz config"
    )

    # Load MoveIt configuration for xArm7
    moveit_config = MoveItConfigsBuilder(
        "ufactory-xarm7-robot", package_name="ufactory_xarm7_moveit_config"
    ).to_moveit_configs()

    # ROS2 Control node for simulation
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(
            get_package_share_directory("ufactory_xarm7_moveit_config"),
            "config", "ros2_controllers.yaml"
        )],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen"
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    # Spawn xArm7 arm controller
    xarm7_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm7_controller", "-c", "/controller_manager"],
        output="screen"
    )

    # Static TF publisher (world -> base link of xArm7)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description]
    )

    # Move group node (planning + execution)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # robot description + semantic + pipelines
            os.path.join(
                get_package_share_directory("ufactory_xarm7_moveit_config"),
                "config",
                "moveit_controllers.yaml"
            )
        ]
    )


    # RViz node
    rviz_full_config = os.path.join(
        get_package_share_directory("ufactory_xarm7_moveit_config"),
        "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics
        ]
    )

    return LaunchDescription([
        rviz_arg,
        static_tf_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        xarm7_controller_spawner,
        move_group_node,
        rviz_node
    ])
