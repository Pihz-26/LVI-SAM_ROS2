import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ####################################################### tf static #############################################################
    # 发布静态关节数据  
    # base_link 和 livox_frame 和 lidar_frame 关节静态位置信息

    tf_group = GroupAction(
        actions = [
            Node(
                package = 'tf2_ros',
                executable = 'static_transform_publisher',
                arguments = ['0','0','0','0','0','0','map','odom']
            ),
            Node(
                package = 'tf2_ros',
                executable = 'static_transform_publisher',
                arguments = ['0','0','0','0','0','0','base_link','livox_frame']
            ),
        ]
    )

    # ###############################################   start mid360    ##############################################################
    xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    cur_config_path = cur_path + '../config'
    user_config_path = os.path.join(cur_config_path, 'MID360_config.json')

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    ########################################################### lio sam ##################################################################
    lio_sam_launch = os.path.join(get_package_share_directory('lio_sam'), 'launch','run.launch.py')
    lio_sam=TimerAction(period=0.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(lio_sam_launch))])

    ################################################### 取特定z范围压缩 #########################################################
    sector_filter_launch = os.path.join(get_package_share_directory('imu_transformer'), 'launch','sector_filter.launch.py')
    sector_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sector_filter_launch),
        launch_arguments={
        'input_topic': '/lio_sam/mapping/cloud_registered_raw',
        # 'input_topic': '/lio_sam/mapping/map_local',
        'output_topic': '/lidar/to_map',
        'z_min': '0.4',
        'z_max': '0.6',
        }.items()
    )

    ################################################### octomap_server2 生成二维地图 #########################################################
    octomap_server2_launch = os.path.join(get_package_share_directory('octomap_server2'), 'launch','octomap_server_launch.py')
    octomap_server2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(octomap_server2_launch),
        launch_arguments={
        'input_cloud_topic': '/lidar/to_map',
        'resolution': '0.05',
        }.items()
    )

    ################################################### launch description #########################################################
    ld = LaunchDescription()
    ld.add_action(tf_group)
    ld.add_action(livox_driver)
    ld.add_action(lio_sam)
    ld.add_action(sector_filter)
    ld.add_action(octomap_server2)
    return ld
