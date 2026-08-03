#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os.path as osp
import numpy as np
import math

from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud

from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster

class NuscenesIngestor(Node):
    def __init__(self):
        super().__init__('nuscenes_ingestor')
        self.get_logger().info("Initializing NuScenes SDK...")
        
        self.declare_parameter('data_root', '/ros2_ws/data_pipeline/nuscenes2mcap/data')
        data_root = self.get_parameter('data_root').get_parameter_value().string_value
        
        self.nusc = NuScenes(version='v1.0-mini', dataroot=data_root, verbose=True)
        
        # 0. Declare Parameters for Multi-Agent Scaling
        self.declare_parameter('scene_name', 'scene-0103')
        self.declare_parameter('vehicle_name', 'car_0103')
        
        self.current_scene_name = self.get_parameter('scene_name').get_parameter_value().string_value
        self.vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value
        
        # 1. Setup the ROS 2 Publishers & TF Broadcasters
        self.pub_lidar = self.create_publisher(PointCloud2, f'/{self.vehicle_name}/lidar_top', 10)
        self.pub_marker = self.create_publisher(Marker, f'/{self.vehicle_name}/ego_vehicle', 1)
        self.pub_cam = self.create_publisher(CompressedImage, f'/{self.vehicle_name}/cam_front', 5)
        
        self.last_cam_token = ""
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # 2. Find the requested Scene
        self.target_scene = None
        for scene in self.nusc.scene:
            if scene['name'] == self.current_scene_name:
                self.target_scene = scene
                break
                
        if not self.target_scene:
            self.get_logger().error(f"Could not find {self.current_scene_name}!")
            return
            
        # 3. Traverse to the very first LiDAR sweep of the scene
        first_sample_token = self.target_scene['first_sample_token']
        first_sample = self.nusc.get('sample', first_sample_token)
        self.current_sd_token = first_sample['data']['LIDAR_TOP']
        
        self.get_logger().info(f"Ready to broadcast {self.current_scene_name}!")
        
        # 4. Create a timer to publish the data.
        # We previously tried 20Hz (0.05s) for maximum map density, but Foxglove WebGL 
        # silently drops the massive MarkerArray CUBE_LIST due to WebSocket limits!
        # Throttling back to 5Hz (0.2s) keeps the map lightweight enough for Foxglove to render the solid cubes.
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        # Stop if we reached the end of the scene
        if self.current_sd_token == "":
            self.get_logger().info(f"Finished broadcasting {self.current_scene_name}!")
            self.timer.cancel()
            return
            
        # Get the current sensor data record from the database
        current_sd = self.nusc.get('sample_data', self.current_sd_token)
        
        # --- SYNCHRONIZE TIMESTAMP ---
        # With multiple cars running, the CPU is choked. We MUST use the exact same timestamp 
        # for TF and PointCloud, otherwise octomap_server's MessageFilter queue will fill up and drop messages!
        now = self.get_clock().now().to_msg()
        
        # --- DYNAMIC TF (ODOMETRY) ---
        # Fetch the exact GPS/Odometry pose of the car for this specific LiDAR sweep
        ego_pose = self.nusc.get('ego_pose', current_sd['ego_pose_token'])
        
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.vehicle_name}_base_link'
        t.transform.translation.x = ego_pose['translation'][0]
        t.transform.translation.y = ego_pose['translation'][1]
        t.transform.translation.z = ego_pose['translation'][2]
        t.transform.rotation.x = ego_pose['rotation'][1]
        t.transform.rotation.y = ego_pose['rotation'][2]
        t.transform.rotation.z = ego_pose['rotation'][3]
        t.transform.rotation.w = ego_pose['rotation'][0]
        
        # --- LIDAR TF ---
        # We must publish the mathematical relationship between the Car and the LiDAR!
        t_lidar = TransformStamped()
        t_lidar.header.stamp = now
        t_lidar.header.frame_id = f'{self.vehicle_name}_base_link'
        t_lidar.child_frame_id = f'{self.vehicle_name}_lidar'
        t_lidar.transform.translation.x = 0.0
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 1.84
        yaw = -math.pi / 2.0
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = math.sin(yaw / 2.0)
        t_lidar.transform.rotation.w = math.cos(yaw / 2.0)
        
        self.tf_broadcaster.sendTransform([t, t_lidar])
        
        # --- LIDAR PUBLISHER ---
        # Load the binary .pcd.bin file into a Numpy array
        pc_path = osp.join(self.nusc.dataroot, current_sd['filename'])
        pc = LidarPointCloud.from_file(pc_path)
        
        # pc.points is a 4xN array (X, Y, Z, Intensity). Transpose it to Nx4 for ROS 2.
        points = pc.points.T
        
        # Define the PointCloud2 memory layout
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        header = Header()
        header.stamp = now  # Use exact same synced clock for Foxglove/Octomap playback
        header.frame_id = f"{self.vehicle_name}_lidar"
        
        msg = point_cloud2.create_cloud(header, fields, points)
        self.pub_lidar.publish(msg)
        
        # Render the Renault Zoe 3D Mesh natively in base_link!
        marker = Marker()
        marker.header.stamp = header.stamp
        marker.header.frame_id = f"{self.vehicle_name}_base_link"  # Place it natively on the car chassis!
        marker.ns = "ego_car"
        marker.id = 2  # Bumping ID again just in case
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "https://assets.foxglove.dev/NuScenes_car_uncompressed.glb"
        
        # 1. Fix the "Sketch" look by forcing a solid metallic color
        marker.mesh_use_embedded_materials = False
        marker.color.a = 1.0  # Solid, not transparent
        marker.color.r = 0.6
        marker.color.g = 0.6
        marker.color.b = 0.6
        
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # NuScenes defines base_link at the REAR AXLE. 
        # The GLB mesh has its origin at the physical center of the car.
        # We must push the mesh forward 1.0 meters to perfectly align the roof with the LiDAR!
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0 
        marker.pose.position.z = 0.0 
        marker.pose.orientation.w = 1.0
        
        self.pub_marker.publish(marker)
        
        # --- CAM FRONT PUBLISHER ---
        # NuScenes only takes pictures at 2Hz (keyframes), but LiDAR spins at 20Hz.
        # We look up the parent "sample" (keyframe) for this exact LiDAR sweep to find the closest photo.
        sample_token = current_sd['sample_token']
        sample = self.nusc.get('sample', sample_token)
        cam_token = sample['data']['CAM_FRONT']
        
        # Only read from disk and publish if this is a NEW photo we haven't seen yet
        if cam_token != self.last_cam_token:
            self.last_cam_token = cam_token
            cam_path = self.nusc.get_sample_data_path(cam_token)
            
            with open(cam_path, 'rb') as f:
                img_data = list(f.read())  # ROS 2 expects a flat array of bytes
                
            img_msg = CompressedImage()
            img_msg.header.stamp = now
            img_msg.header.frame_id = f"{self.vehicle_name}_cam_front"
            img_msg.format = "jpeg"
            img_msg.data = img_data
            self.pub_cam.publish(img_msg)
            
        # Move to the exact next 20Hz sweep using the Linked List!
        self.current_sd_token = current_sd['next']

def main(args=None):
    rclpy.init(args=args)
    node = NuscenesIngestor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
