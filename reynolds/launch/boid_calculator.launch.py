
import os
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # package_share = FindPackageShare('reynolds').perform(context)
    # config_path = os.path.join(package_share, 'launch', 'params.yaml')
    # with open(config_path, 'r') as stream:
    #     config = yaml.safe_load(stream)
    
    robots_launch = os.path.join(
        get_package_share_directory('sphero_stage'),
        'launch',
        'start.launch.py'
    )
    
    # robots_launch = os.path.join(
    #         get_package_share_directory('stage_ros2'),
    #         'launch',
    #         'stage.launch.py'
    # )
    
    perspective = os.path.join(get_package_share_directory('reynolds'), 'launch', 'boid_speed.perspective')
    
    
    
    return LaunchDescription([
        
        
        
        
        Node(
            package='reynolds',
            executable='boid_calculator',
            name='boid_calculator',
            output='screen',
            parameters=[{
                'coefs': [5.0, 1.0, 2.0, 0.0],
                'fov': 330.0,
                'max_distance': 10.0,
                'min_spaceing': 6.0, # wall
                'min_distance':4.5, # boids
                'spheroID': list(range(10)),
                'max_linear': 0.3,
                'max_angular': 3.0,
            }]
        ),
        
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(robots_launch)
                ),
        
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt',
            arguments=['--perspective-file', perspective],
        )
    ])