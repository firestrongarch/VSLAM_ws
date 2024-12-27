from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/vins/uma_vi.yaml'
    ])

    support_path = PathJoinSubstitution([
        config_pkg_path,
        'voc'
    ])
    # Define the node
    feature_tracker_node = Node(
        package='vins_mono',
        executable='feature_tracker_illustration',
        name='feature_tracker',
        namespace='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': config_path,
        }]
    )
    
    # Define the vins_estimator node
    vins_estimator_node = Node(
        package='vins_mono',
        executable='vins_estimator',
        name='vins_estimator',
        namespace='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': config_path,
        }]
    )

    # Define the pose_graph node
    pose_graph_node = Node(
        package='vins_mono',
        executable='pose_graph',
        name='pose_graph',
        namespace='pose_graph',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'support_file': support_path,
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0
        }]
    )

    rviz_config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/vins/rviz.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        vins_estimator_node,
        pose_graph_node,
        TimerAction(
            period=2.0,  # 这里设置延迟时间为2秒
            actions=[feature_tracker_node]
        ),
    ])