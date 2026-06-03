import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file = LaunchConfiguration("map_file")
    rviz_config_file = os.path.join(get_package_share_directory('social_cost_map'), 'rviz_conf', 'navigation_view.rviz')
    social_params_file = LaunchConfiguration("social_params_file")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("social_cost_map"), "maps", "my_map.yaml"]),
        description="Full path to the yaml map file",
    )

    amcl_params_file_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("social_cost_map"), "config", "amcl.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for the amcl node",
    )

    social_params_arg = DeclareLaunchArgument(
    "social_params_file",
    default_value=PathJoinSubstitution(
        [FindPackageShare("social_cost_map"), "config", "social_params.yaml"]
    ),
    description="Full path to the nav2 parameters file with social layers"
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
    )

    planner_server = Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    parameters=[social_params_file, {'use_sim_time': use_sim_time}]
    )

    controller_server = Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    parameters=[social_params_file, {'use_sim_time': use_sim_time}]
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[amcl_params_file, {"use_sim_time": use_sim_time}],
    )

    nav_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav_manager",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "amcl", "planner_server", "controller_server"]},
        ],
    )
    
    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            amcl_params_file_arg,
            map_file_arg,
            social_params_arg,
            map_server_node,
            planner_server,
            controller_server,
            amcl_node,
            nav_manager,
            rviz_node,
        ]
    )
