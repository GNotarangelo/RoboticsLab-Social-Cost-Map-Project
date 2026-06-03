import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Nodo di conversione e calcolo delle pose sociali
    social_bridge_node = Node(
        package='social_cost_map',
        executable='social_costmap_bridge',
        name='social_costmap_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}] # Fondamentale per il timer sincronizzato!
    )

    return LaunchDescription([
        social_bridge_node
    ])