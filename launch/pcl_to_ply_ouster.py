from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_to_ply',
            executable='pcl_to_ply',
            name='pcl_to_ply',
            # remappings=[
            #     ('/input/pose', '/turtlesim1/turtle1/pose'),
            # ]
        )
    ])