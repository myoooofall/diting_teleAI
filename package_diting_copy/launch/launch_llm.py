import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动recognize节点
        Node(
            package='package_diting',
            executable='recognize',
            name='recognize_node',
            output='screen'
        ),

        # 启动detect节点
        Node(
            package='package_diting',
            executable='detect',
            name='detect_node',
            output='screen'
        ),

        # 启动gen_voice节点
        Node(
            package='package_diting',
            executable='voice',
            name='gen_voice_node',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
