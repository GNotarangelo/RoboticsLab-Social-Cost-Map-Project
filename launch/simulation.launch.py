import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project = get_package_share_directory('social_cost_map')

    models_path = os.path.join(pkg_project, 'models')
    worlds_path = os.path.join(pkg_project, 'worlds')
    urdf_path = os.path.join(pkg_project, 'urdf')

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [models_path, ':', worlds_path]
    )

    world_file = os.path.join(worlds_path, 'Mondo.sdf')
    xacro_fra2mo = os.path.join(urdf_path, 'fra2mo', 'fra2mo.urdf.xacro')
    xacro_iiwa = os.path.join(urdf_path, 'iiwa', 'iiwa_spawn.urdf.xacro')

    fra2mo_description_xacro = {"robot_description": 
                                ParameterValue(Command(['xacro ', xacro_fra2mo]),
                                    value_type=str)}

    iiwa_description_xacro = {"robot_description":
                               ParameterValue(
                                Command(['xacro ', xacro_iiwa]),
                                value_type=str)}

    fra2mo_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='fra2mo',
        output='both',
        parameters=[fra2mo_description_xacro,
                    {"use_sim_time": True}
            ]
    )

    fra2mo_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}]
    )

    iiwa_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='iiwa',
        output='both',
        parameters=[iiwa_description_xacro,
                    {"use_sim_time": True}
            ],
    )

    """iiwa_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='iiwa',
        parameters=[{"use_sim_time": True}]
    )"""


    load_joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/iiwa/controller_manager"],
    )

    # Nodo per attivare il controllo di posizione del braccio
    load_iiwa_arm_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["iiwa_arm_trajectory_controller", "-c", "/iiwa/controller_manager"],
    )

    world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    position_fra2mo = [-8.0, -6.0, 0.100]

    gz_spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/fra2mo/robot_description',
                   '-name', 'fra2mo',
                   '-allow_renaming', 'true',
                    "-x", str(position_fra2mo[0]),
                    "-y", str(position_fra2mo[1]),
                    "-z", str(position_fra2mo[2]),]
    )

    fra2mo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fra2mo_bridge',
        arguments=['/fra2mo/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/fra2mo/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/world/social_cost_map_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output='screen'
    )

    odom_tf = Node(
        package='social_cost_map',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )

    position_iiwa = [-2.75, -8.75, 0.100]

    gz_spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        name='iiwa_bridge',
        output='screen',
        arguments=['-topic', '/iiwa/robot_description',
                   '-name', 'iiwa_robot',
                   '-allow_renaming', 'true',  
                    "-x", str(position_iiwa[0]),
                    "-y", str(position_iiwa[1]),
                    "-z", str(position_iiwa[2]),]
    )

    iiwa_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        fra2mo_state_publisher_node,
        fra2mo_joint_state_publisher_node,
        iiwa_state_publisher_node,
        load_joint_state_broadcaster,
        load_iiwa_arm_controller,
        world_launch,
        gz_spawn_fra2mo,
        fra2mo_bridge,
        odom_tf,
        gz_spawn_iiwa,
        iiwa_bridge
    ])