import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    ###############################################     global    ##############################################################
    livox_dir = get_package_share_directory('livox_pro')
    livox_launch_dir = os.path.join(get_package_share_directory('livox_pro'),'launch')
    livox_config_dir = os.path.join(get_package_share_directory('livox_pro'),'config')

    # 声明 map_yaml_file 参数
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    map_yaml_file_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value='/home/booster/Workspace/save/0918.yaml',  # 可根据需要设置默认地图名称
        description='Name of the map file '
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ####################################################### tf static #############################################################
    tf_group = GroupAction(
        actions = [
            Node(
                package = 'tf2_ros',
                executable = 'static_transform_publisher',
                arguments = ['0','0','0','0','0','0','base_link','livox_frame']
            ),
            Node(
                package = 'tf2_ros',
                executable = 'static_transform_publisher',
                arguments = ['0','0','0','0','0','0','base_link','lidar_frame']
            ),
        ]
    )

    ###############################################   start mid360    ##############################################################
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

    #####################################################  imu #############################################################
    # imu_transformer_cmd = Node(
    #     package='imu_transformer',
    #     executable='imu_transformer_node',
    #     name='imu_transformer',
    #     output='screen',
    #     parameters=[
    #         {
    #             'roll': 0.0,       # 绕X轴旋转角度（弧度），此处为180度
    #             'pitch': 0.5585,   # 绕Y轴旋转角度（弧度）
    #             'yaw': 0.0         # 绕Z轴旋转角度（弧度）
    #         }
    #     ]
    # )

    ################################################### ground segmentation #########################################################
    livox_dir = get_package_share_directory('livox_pro')
    segmentation_params = os.path.join(livox_dir , 'config', 'segmentation_real.yaml')

    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params]
    )

    ################################################### pointcloud to laserscan #########################################################
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        # remappings=[('cloud_in', '/segmentation/obstacle/to_map'),  # 修正：移除列表包装
        remappings=[('cloud_in', '/segmentation/obstacle'),  # 修正：移除列表包装
                    ('scan', '/scan')],                     # 修正：移除列表包装
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.6,
            'max_height': -0.1,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.1,
            'range_max': 30.0,
            'use_inf': False,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    ################################################### nav2 #########################################################
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    params_file = os.path.join(livox_dir, 'config', 'nav2.yaml')
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'slam': 'False',
                        'map': map_yaml_file,
                        'use_sim_time': 'False',
                        'params_file': params_file,
                        'use_rviz':'False'}.items()
                        )

    ################################################### lio_sam odom #########################################################
    lio_sam_launch = os.path.join(get_package_share_directory('lio_sam'), 'launch','run.launch.py')
    lio_sam_params_file=os.path.join(livox_config_dir,'lio_sam_odom_params.yaml')
    start_lio_sam=TimerAction(period=0.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lio_sam_launch),
            launch_arguments={'params_file': lio_sam_params_file}.items()
        )
    ])

    odom_relay_node = Node(
        package='imu_transformer', executable='odom_relay',
        parameters=[{
            'in_topic': '/lio_sam/mapping/odometry',
            'out_topic': '/odom',
            'frame_id_override': 'odom',
            'child_frame_id_override': 'base_link',
        }],
        name='odom_relay_node'
    )

    livox_to_point2 =  Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        output='screen',
    )
    ################################################# localizer ###################################################################
    icp_dir=get_package_share_directory('icp_registration')
    icp_pcd_dir = PythonExpression([
        "('/'.join('", LaunchConfiguration('map_yaml_file'),
        "'.split('/')[:-1]) + '/' + '",
        LaunchConfiguration('map_yaml_file'),
        "'.split('/')[-1].replace('.yaml','') + '/GlobalMap.pcd')"
    ])
    icp_registration_params_dir = os.path.join(icp_dir, 'config', 'icp.yaml')
    start_icp_registration_launch=TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package='icp_registration',
                        executable='icp_registration_node',
                        output='screen',
                        parameters=[
                            icp_registration_params_dir,
                            {'use_sim_time': use_sim_time,
                            # 'pointcloud_topic': "/lio_sam/mapping/cloud_registered_raw",
                            'pointcloud_topic': "/lio_sam/mapping/map_local",
                            'pcd_path': icp_pcd_dir,
                            'initial_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # x,y,z,roll,pitch,yaw
                            }
                        ],
                    ),
                ]
            )
    ################################################# 代价地图融合 ###################################################################
    map_fusion = Node(
        package='map_fusion',
        executable='map_fusion_node',
        name='map_fusion',
        output='screen'
    )

    ############################################ 机器人控制器 ################################################
    command_controller = Node(
        package='controller',
        executable='command_controller_node',
        name='command_controller',
        output='screen'
    )

    ################################################### launch description #########################################################
    ld = LaunchDescription()
    ld.add_action(tf_group)
    ld.add_action(map_yaml_file_arg)
    ld.add_action(livox_driver)
    ld.add_action(livox_to_point2)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(start_lio_sam)
    ld.add_action(odom_relay_node)
    ld.add_action(bringup_cmd)
    ld.add_action(map_fusion)
    # ld.add_action(start_icp_registration_launch)
    ld.add_action(command_controller)

    return ld
