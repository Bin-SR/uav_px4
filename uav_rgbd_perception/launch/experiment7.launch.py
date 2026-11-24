from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('uav_rgbd_perception'), 'config', 'experiment7.yaml'
    ])
    return LaunchDescription([
        Node(
            package='uav_rgbd_perception',
            executable='obj_detector',
            name='obj_detector_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='uav_rgbd_perception',
            executable='depth_processor',
            name='depth_processor_node',
            output='screen',
            parameters=[{
                'show_depth': False,
                'depth_scale_16uc1': 0.001,
            }],
        ),
        Node(
            package='uav_rgbd_perception',
            executable='pointcloud_processor',
            name='pointcloud_processor_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='uav_rgbd_perception',
            executable='world_state',
            name='world_state_node',
            output='screen',
            parameters=[params],
        ),
    ])
