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



    # ################################################### nav2 #########################################################
    # bringup_dir = get_package_share_directory('nav2_bringup')
    # launch_dir = os.path.join(bringup_dir, 'launch')
    # params_file = os.path.join(livox_dir, 'config', 'nav2.yaml')
    
    # bringup_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
    #     launch_arguments={'slam': 'False',
    #                     'map': map_yaml_file,
    #                     'use_sim_time': 'False',
    #                     'params_file': params_file,
    #                     'use_rviz':'False'}.items()
    #                     )

    ################################################### lio_sam odom #########################################################
    lio_sam_launch = os.path.join(get_package_share_directory('lio_sam'), 'launch','run.launch.py')
    lio_sam_params_file=os.path.join(livox_config_dir,'lio_sam_odom_params.yaml')
    start_lio_sam=TimerAction(period=0.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lio_sam_launch),
            launch_arguments={'params_file': lio_sam_params_file}.items()
        )
    ])

    ################################################# localizer ###################################################################
    # relocation_launch = os.path.join(get_package_share_directory('pointcloud_localization'), 'launch','pointcloud_localization.launch.py')
    # start_relocation=TimerAction(period=0.0, actions=[
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(relocation_launch)
    #     )
    # ])
    
    

    ################################################### launch description #########################################################
    ld = LaunchDescription()
    ld.add_action(tf_group)
    ld.add_action(map_yaml_file_arg)

    ld.add_action(start_lio_sam)
    # ld.add_action(start_relocation)

    return ld
