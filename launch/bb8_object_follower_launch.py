from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bb8_object_follower',
            namespace='image_subscriber',
            executable='bb8_object_follower_node',
            name=''
        ),
        Node(
            package='bb8_object_follower',
            namespace='debug_subscriber',
            executable='bb8_object_follower_node',
            name=''
        ),
        Node(
            package='',
            executable='',
            name='',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
