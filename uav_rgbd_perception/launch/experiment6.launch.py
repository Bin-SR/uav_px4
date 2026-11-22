from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('uav_rgbd_perception'), 'config', 'experiment6.yaml'
    ])
    return LaunchDescription([
        Node(package='uav_rgbd_perception', executable='rgb_detector',
             name='rgb_detector_node', output='screen',
             parameters=[{'show_windows': True, 'min_contour_area': 150.0}]),
        Node(package='uav_rgbd_perception', executable='depth_processor',
             name='depth_processor_node', output='screen',
             parameters=[{'show_depth': True, 'depth_scale_16uc1': 0.001}]),
        Node(package='uav_rgbd_perception', executable='pointcloud_processor',
             name='pointcloud_processor_node', output='screen',
             parameters=[params]),
    ])
