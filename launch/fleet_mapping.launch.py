from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    data_root_arg = DeclareLaunchArgument(
        'data_root',
        default_value='/ros2_ws/data_pipeline/nuscenes2mcap/data',
        description='Absolute path to the nuScenes v1.0-mini dataset'
    )

    ld = LaunchDescription([
        data_root_arg,
        # The Foxglove Bridge (Cloud Uplink)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'asset_uri_allowlist': ['^package://(.*)$', '^https://(.*)$']
            }]
        )
    ])
    
    # We will simulate 4 discrete autonomous edge-vehicles concurrently
    fleet = [
        ('scene-0103', 'car_0103'),
        ('scene-0553', 'car_0553'),
        ('scene-0655', 'car_0655'),
        ('scene-0061', 'car_0061')
    ]
    
    for scene, car in fleet:
        # 1. Edge Telemetry Ingestion (Raw Data)
        ld.add_action(Node(
            package='fleet_mapper',
            executable='nuscenes_ingestor',
            name=f'{car}_ingestor',
            output='screen',
            parameters=[{
                'scene_name': scene,
                'vehicle_name': car,
                'data_root': LaunchConfiguration('data_root')
            }]
        ))
        
        # 2. Edge Processing (LiDAR -> HD Voxel Map)
        ld.add_action(Node(
            package='octomap_server',
            executable='octomap_server_node',
            name=f'{car}_octomap',
            output='screen',
            parameters=[{
                'resolution': 0.1,                 # 10cm voxel resolution
                'frame_id': 'map',                 # Global map frame
                'base_frame_id': f'{car}_base_link', # Dynamic origin
                'latch': True,                     # Latch the map for late Foxglove joiners
                'height_map': True,                # Rainbow color gradient based on altitude
                'sensor_model/max_range': 40.0,    # Clip noise beyond 40m
                'occupancy_min_z': 0.3,            # Delete the floor (anything below 30cm)
                'occupancy_max_z': 20.0,           # Delete floating sky reflections (above 20m)
            }],
            remappings=[
                # Wire the agent's LiDAR directly into its edge processor
                ('cloud_in', f'/{car}/lidar_top'),
                
                # Output the finished voxel map into a dedicated cloud namespace
                ('occupied_cells_vis_array', f'/{car}/occupied_cells_vis_array')
            ]
        ))
        
    return ld
