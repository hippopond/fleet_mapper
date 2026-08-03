from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # The Foxglove Bridge (Cloud Uplink)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'asset_uri_allowlist': ['^package://(.*)$', '^https://(.*)$']
            }]
        ),
        
        # ==========================================
        # CAR A (Scene 0553)
        # ==========================================
        Node(
            package='fleet_mapper',
            executable='nuscenes_ingestor',
            name='car_a_ingestor',
            output='screen',
            parameters=[{
                'scene_name': 'scene-0553',
                'vehicle_name': 'car_0553'
            }]
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='car_a_octomap',
            output='screen',
            parameters=[{
                'resolution': 0.1,
                'frame_id': 'map',
                'base_frame_id': 'car_0553_base_link',
                'latch': False,
                'sensor_model/max_range': 40.0,
                'occupancy_min_z': 0.3,
                'occupancy_max_z': 20.0,
            }],
            remappings=[
                ('cloud_in', '/car_0553/lidar_top'),
                ('occupied_cells_vis_array', '/car_0553/occupied_cells_vis_array')
            ]
        ),

        # ==========================================
        # CAR B (Scene 0655)
        # ==========================================
        Node(
            package='fleet_mapper',
            executable='nuscenes_ingestor',
            name='car_b_ingestor',
            output='screen',
            parameters=[{
                'scene_name': 'scene-0655',
                'vehicle_name': 'car_0655'
            }]
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='car_b_octomap',
            output='screen',
            parameters=[{
                'resolution': 0.1,
                'frame_id': 'map',
                'base_frame_id': 'car_0655_base_link',
                'latch': False,
                'sensor_model/max_range': 40.0,
                'occupancy_min_z': 0.3,
                'occupancy_max_z': 20.0,
            }],
            remappings=[
                ('cloud_in', '/car_0655/lidar_top'),
                ('occupied_cells_vis_array', '/car_0655/occupied_cells_vis_array')
            ]
        )
    ])
