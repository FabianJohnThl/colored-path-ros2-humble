from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='col_pth', executable='col_pth',
            remappings=[('path_in', '/lio_livox/odometry_path_mapped'),                         # Map your input path to your 
                        ('path_colored', '/colored_path')],           # Map the merged and filtered cloud output to the desired output topic
            parameters=[{'colorizer': 'RM520N',                       # string to select a source for colorizing: {'RM520N', 'DUMMY'}
                         'movement_min_distance': 1.0                 # Minimum distance to be moved on path before drawing/querying new actual value for colorizing
                        }],
            name='col_pth'
        )
    ])
