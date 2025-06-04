from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    topic = LaunchConfiguration('topic', default='/dlio/odom_node/pointcloud/deskewed')

    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'topic',
        default_value=topic,
        description='Pointcloud topic name'
    )
    node = Node(
        package='pcl_to_ply',
        executable='pcl_to_ply',
        name='pcl_to_ply',
        output='screen',
        remappings=[
            ('pointcloud', topic),
        ]
    )
    return LaunchDescription([
        declare_pointcloud_topic_arg,
        node
    ])