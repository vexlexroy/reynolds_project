
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
    
    perspective = os.path.join(get_package_share_directory('reynolds'), 'launch', 'boid_speed_lab2.perspective')
    
    ids = range(0,10) # num of robots
    turning = False # if it uses turning logic
    return LaunchDescription([
        *[Node(
            package='reynolds',
            executable='boid_calculator_decent',
            name=f'boid_calculator_decent_{i}',
            output='screen',
            parameters=[{
                'id' : i,
                'coefs': [5.4, 0.3, 5.4, 0.5, 1.5], # sep, al, coh, obst, goal
                'mass': 7.5,
                'is_turning': turning,
                'max_linear': 0.3,
                'min_linear': 0.0,
                'max_acceleration' : 0.6, 
                'max_angular': 1.5*3.14,
                'turn_rate' : 3.5,
            }]
        ) for i in ids],
        
        Node(
            package='reynolds',
            executable='neighbor_alocator',
            name='neighbor_alocator',
            output='screen',
            parameters=[{
                'spheroID' : ids,
                'fov' : 250.0,
                'max_distance' : 3.0,
                'is_turning': turning,
            }]
        ),
        
        Node(
            package='reynolds',
            executable='goal_point_selector',
            name='goal_point_selector',
            output='screen',
            parameters=[{
                'path_l': 60,
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