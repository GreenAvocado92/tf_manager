from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


PACKAGE_NAME = 'tf_manager'
TF_TOPIC = '/tf_manager/tf'
TF_STATIC_TOPIC = '/tf_manager/tf_static'


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    default_yaml = os.path.join(package_share, 'config', 'calibration.yaml')
    default_pointcloud_yaml = os.path.join(
        package_share, 'config', 'realsense_pointcloud_transform.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('calibration_yaml', default_value=default_yaml),
        DeclareLaunchArgument(
            'pointcloud_config', default_value=default_pointcloud_yaml),
        Node(
            package=PACKAGE_NAME,
            executable='calibration_tf_broadcaster',
            name='calibration_tf_broadcaster',
            output='screen',
            remappings=[
                ('/tf', TF_TOPIC),
                ('/tf_static', TF_STATIC_TOPIC),
            ],
            parameters=[{
                'calibration_yaml': LaunchConfiguration('calibration_yaml'),
            }],
        ),
        Node(
            package=PACKAGE_NAME,
            executable='pointcloud_tf_transformer',
            name='pointcloud_tf_transformer',
            output='screen',
            remappings=[
                ('/tf', TF_TOPIC),
                ('/tf_static', TF_STATIC_TOPIC),
            ],
            parameters=[{
                'pointcloud_config': LaunchConfiguration('pointcloud_config'),
            }],
        ),
    ])
