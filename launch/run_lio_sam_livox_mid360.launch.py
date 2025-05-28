import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('liorf')
    parameter_file = LaunchConfiguration('params_file')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'mapping.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'lio_sam_livox_mid360.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    return LaunchDescription([
        params_declare,
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform',
        #     output='screen',
        #     parameters=[
        #         {
        #             'use_odometry': True,
        #             'use_manual_datum': False,
        #             'datum_lat': 0.0,
        #             'datum_lon': 0.0,
        #             'datum_height': 0.0,
        #             'yaw_offset': 0.0,
        #             'zero_altitude': False,
        #             'broadcast_utm_transform': True,
        #             'broadcast_map_transform': True,
        #         }
        #     ],
        #     remappings=[
        #         ('/gps', '/gps/raw_fix'),         # 센서에서 오는 원본 GPS 데이터를 '/gps/raw_fix'로 처리
        #         ('/odom', '/robot_odometry'),    # 로봇 odometry 데이터를 '/robot_odometry'로 처리
        #         ('/odometry/gpsz', '/gps/odometry')  # 변환된 GPS odometry 데이터
        #     ]
        # ),       
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_livox',
            arguments=['--x', '0.23', '--y', '0', '--z', '0.805',
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                       '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
            output='screen'
        ),        
        Node(
            package='liorf',
            executable='liorf_imuPreintegration',
            name='liorf_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='liorf',
            executable='liorf_imageProjection',
            name='liorf_imageProjection',
            parameters=[parameter_file,
                    {'use_livox_data': True,
                    'livox_time_field': 'offset_time'}],
            output='screen'
        ),
        # Node(
        #     package='liorf',
        #     executable='liorf_featureExtraction',
        #     name='liorf_featureExtraction',
        #     parameters=[parameter_file],
        #     output='screen'
        # ),
        Node(
            package='liorf',
            executable='liorf_mapOptmization',
            name='liorf_mapOptmization',
            parameters=[parameter_file, {
                'costmap_resolution': 0.1,        # 격자 해상도 (m)
                'costmap_width': 1.0, # 수정금지 1.0 고정           # 초기 맵 폭 (m)
                'costmap_height': 1.0, # 수정금지 1.0 고정           # 초기 맵 높이 (m)
                'min_height_threshold': 0.15,      # 포인트 클라우드 최소 높이 임계값 (m)
                'max_height_threshold': 1.1,      # 포인트 클라우드 최대 높이 임계값 (m) - 2.0보다 더 높게 설정
                'obstacle_threshold': 2,          # 장애물로 판단할 최소 포인트 수 - 민감도 높임
                'point_threshold': 1,             # 그리드 셀 처리를 위한 최소 포인트 수
                'height_diff_threshold': 0.01,     # 장애물 판단 높이 차이 임계값 (m)
                'base_frame_id': 'base_link',     # 베이스 프레임 ID
                'auto_resize_map': True,          # 맵 자동 크기 조정 활성화
                'use_database_mode': True,
                'database_path': '/root/liorf_ws2/src/liorf/maps/slam_map.db', # 경로 수정금지 /root/liorf_ws2/src/liorf/maps/slam_map.db 고정
                'clouds_directory': '/root/liorf_ws2/src/liorf/maps/clouds', # 경로 수정금지 /root/liorf_ws2/src/liorf/maps/clouds 고정
                'active_keyframes_window_size': 9999999999,
                'spatial_query_radius': 10.0
            }],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
