from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('mocap_optitrack_driver'),
                                            'config', 'rigid_body_transform_view.rviz')]
        )
    ])
