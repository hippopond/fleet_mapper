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

    return LaunchDescription([
        data_root_arg,
        # 1. The Foxglove Bridge (Cloud Uplink)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'asset_uri_allowlist': ['^package://(.*)$', '^https://(.*)$']
            }]
        ),
        
        # 2. Our Custom Edge Ingestion Node
        Node(
            package='fleet_mapper',
            executable='nuscenes_ingestor',
            name='nuscenes_ingestor',
            output='screen',
            parameters=[{
                'data_root': LaunchConfiguration('data_root')
            }]
        ),
        
        # 3. The OctoMap Server (Cloud Mapping Orchestrator)
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'resolution': 0.1,         # 10cm voxel resolution (High Detail)
                'frame_id': 'map',         # The global map frame
                'base_frame_id': 'car_0103_base_link',
                'latch': True,
                'height_map': True,             # Explicitly enable height-based color scaling
                'sensor_model/max_range': 40.0, # Truncate long-range noisy points
                'occupancy_min_z': 0.3,         # Visually clip out the road voxels (anything below 30cm)
                'occupancy_max_z': 20.0,        # Clip out noisy reflections in the sky
            }],
            remappings=[
                # Wire the LiDAR straight into the server!
                ('cloud_in', '/car_0103/lidar_top'),
                # Remap the output to match the multi-scene namespace!
                ('occupied_cells_vis_array', '/car_0103/occupied_cells_vis_array')
            ]
        )
    ])
