from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("layered_hardware_ign"),
                    "examples/single_joint/description",
                    "robot_description.urdf.xacro",
                ]
            ),
        ]
    )

    # Get display configs
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("layered_hardware"),
            "examples/single_actuator/config",
            "display_config.rviz"
        ]
    )

    # Declare simulation nodes
    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": "-r -s -v 4 empty.sdf",
        }.items()
    )
    ign_robot_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        output="both",
        arguments=["-string", robot_description_content,
                   "-name", "single_joint_example",
                   "-allow_renaming", "true"],
    )
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="both"
    )

    # Declare control nodes
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": ParameterValue(robot_description_content, value_type=str),
            "use_sim_time": use_sim_time}],
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "effort_controller",
                   "--controller-manager", "/controller_manager"],
    )
    sub_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "position_controller",
                   "--inactive", "--controller-manager", "/controller_manager"],
    )

    # Declare visualization nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Build all nodes
    nodes = [
        ign_gazebo_launch,
        ign_robot_spawner,
        ign_bridge,
        robot_state_pub_node,
        controller_spawner,
        sub_controller_spawner,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)
