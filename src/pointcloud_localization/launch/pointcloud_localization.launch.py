from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动您的重定位节点
        Node(
            package='pointcloud_localization',
            executable='initial_localization',  # 对应CMake中的可执行文件名
            name='pointcloud_localization',
            output='screen',
            parameters=[{
                # 可在此添加参数配置
                'pcd_path': "/home/ubt2204/CODE/Datasets/save/0918/GlobalMap.pcd", 
                'input_pointcloud_topic': "lio_sam/feature/cloud_merged"
            }]
        ),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/ubt2204/CODE/Datasets/save/0918.yaml'  # 替换为实际路径
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
