from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/ov2slam/accurate/kitti/kitti_00-02.yaml'
    ])

    # Define the node
    ov2slam = Node(
        package='ov2slam',
        executable='ov2slam_node',
        name='ov2slam_node',
        output='screen',
        parameters=[{
            'config_file': config_path,
        }]
    )

    # rviz_config_path = PathJoinSubstitution([
    #     config_pkg_path,
    #     'config/vins_euroc_rviz.rviz'
    # ])

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     output='screen'
    # )

    return LaunchDescription([
        ov2slam,
    ])