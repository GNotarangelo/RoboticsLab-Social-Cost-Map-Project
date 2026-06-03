import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('social_cost_map')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'iiwa', 'iiwa_spawn.urdf.xacro'])
    robot_description = {'robot_description': Command([FindExecutable(name='xacro'), ' ', xacro_file])}

    srdf_xacro_file = PathJoinSubstitution([pkg_share, 'srdf', 'iiwa.srdf.xacro'])
    robot_description_semantic = {
        'robot_description_semantic': Command([
            FindExecutable(name='xacro'), ' ', srdf_xacro_file,
            ' prefix:=iiwa_ name:=iiwa_robot'
        ])
    }

    kinematics_yaml_path = PathJoinSubstitution([pkg_share, 'moveit2', 'kinematics.yaml'])
    limits_yaml_path = PathJoinSubstitution([pkg_share, 'moveit2', 'iiwa_joint_limits.yaml'])
    ompl_yaml_path = PathJoinSubstitution([pkg_share, 'moveit2', 'ompl_planning.yaml'])

    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': ['iiwa_arm_trajectory_controller'],
            'iiwa_arm_trajectory_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': [
                    'iiwa_joint_a1', 'iiwa_joint_a2', 'iiwa_joint_a3',
                    'iiwa_joint_a4', 'iiwa_joint_a5', 'iiwa_joint_a6', 'iiwa_joint_a7'
                ]
            }
        },
    }

    moveit_params = {
        'use_sim_time': True,
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False,
    }

    common_remappings = [
        ('/iiwa/clock', '/clock'),
        ('/iiwa/tf', '/tf'),
        ('/iiwa/tf_static', '/tf_static'),
        ('joint_states', '/iiwa/joint_states'),
    ]

    aruco_tracker_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'marker_dict': '4X4_50',
            'marker_size': 0.15,
            'cam_base_topic': '/camera',
            'image_is_rectified': True,
        }]
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        namespace='iiwa',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml_path,
            limits_yaml_path,
            ompl_yaml_path,
            moveit_controllers,
            moveit_params,
        ],
        remappings=common_remappings,
    )

    # Do not set name= here: the executable creates its own node and a launch
    # name would collide with MoveGroupInterface internals.
    iiwa_routine_node = Node(
        package='social_cost_map',
        executable='iiwa_cube_routine',
        output='screen',
        namespace='iiwa',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml_path,
            limits_yaml_path,
            ompl_yaml_path,
            moveit_params,
            {'use_sim_time': True},
        ],
        remappings=common_remappings + [
            ('aruco_detections', '/aruco_detections'),
        ],
    )

    delayed_routine = TimerAction(
        period=8.0,
        actions=[iiwa_routine_node],
    )

    return LaunchDescription([
        aruco_tracker_node,
        move_group_node,
        delayed_routine,
    ])
