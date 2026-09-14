#!/usr/bin/env python3
"""
Depth Image to PointCloud2 Converter using Projective Geometry (Pinhole Model)
Designed for ROS 2 Humble.

Projective Geometry:
-------------------
Given camera intrinsic matrix K:
    [ fx   0  cx ]
K = [  0  fy  cy ]
    [  0   0   1 ]

For every pixel (u, v) with depth Z = depth_map[v, u]:
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    Z = Z
in camera optical frame (X: right, Y: down, Z: forward).
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError


# Default max_depth set to 3.0m to discard all points beyond 3 meters
# Commented out min_depth=0.1:
# def depth_to_points_numpy(depth_map, fx, fy, cx, cy, min_depth=0.1, max_depth=3.0, decimation=1):
# Added: Default min_depth set to 0.4m (40cm blind zone) to ignore close-range artifacts / robot body
# Commented out previous signature:
# def depth_to_points_numpy(depth_map, fx, fy, cx, cy, min_depth=0.4, max_depth=3.0, decimation=1):
# Added: Signature with 3m fake background canvas parameters
# Commented out previous signature:
# def depth_to_points_numpy(
#     depth_map,
#     fx,
#     fy,
#     cx,
#     cy,
#     min_depth=0.4,
#     max_depth=2.5,#3.0
#     decimation=1,
#     fill_background=True,
#     background_distance=2.5, #3.0
#     background_mode='plane',
#     ignore_blind_zone=True
# ):
# Added: Signature with canvas_padding_ratio to extend fake canvas beyond camera FOV
def depth_to_points_numpy(
    depth_map,
    fx,
    fy,
    cx,
    cy,
    min_depth=0.4,
    max_depth=2.5,
    decimation=1,
    fill_background=True,
    background_distance=2.5,
    background_mode='plane',
    ignore_blind_zone=True,
    canvas_padding_ratio=0.15
):
    """
    Pure NumPy vectorized pinhole back-projection with optional extended fake background canvas.
    
    Parameters:
        depth_map: 2D numpy array (float32 in meters)
        fx, fy: Focal lengths in pixels
        cx, cy: Principal point offsets in pixels
        min_depth: Minimum valid depth in meters (40cm blind zone)
        max_depth: Maximum valid depth in meters (3.0m limit)
        decimation: Downsampling step (1 = full resolution, 2 = half, etc.)
        fill_background: When True, paints a background canvas for empty spots / points >= max_depth
        background_distance: Distance of the fake canvas in meters (default: 2.5m/3.0m)
        background_mode: 'plane' for flat backdrop canvas, or 'radial' for spherical dome
        ignore_blind_zone: If True, pixels inside the <40cm blind zone are omitted rather than painted
        canvas_padding_ratio: Ratio to expand canvas beyond actual FOV (default 0.15 = 15% extra border)
        
    Returns:
        points: (N, 3) float32 numpy array of [X, Y, Z] coordinates
    """
    height, width = depth_map.shape

    # Apply decimation if requested (great for saving CPU/bandwidth)
    if decimation > 1:
        depth_map = depth_map[::decimation, ::decimation]

    # Added: Check if fake background canvas is requested
    if fill_background:
        # Added: When canvas_padding_ratio > 0, expand pixel grid beyond camera FOV
        if canvas_padding_ratio > 0.0:
            pad_u = (int(round(width * canvas_padding_ratio)) // decimation) * decimation
            pad_v = (int(round(height * canvas_padding_ratio)) // decimation) * decimation
            u_ext = np.arange(-pad_u, width + pad_u, decimation)
            v_ext = np.arange(-pad_v, height + pad_v, decimation)
            u_grid, v_grid = np.meshgrid(u_ext, v_ext)

            bg_dist = float(background_distance if background_distance > 0.0 else max_depth)
            effective_depth = np.full(u_grid.shape, bg_dist, dtype=np.float32)

            v_start = pad_v // decimation
            u_start = pad_u // decimation
            v_end = v_start + (height // decimation)
            u_end = u_start + (width // decimation)

            # Step 1: Detect valid obstacles inside sensor FOV: [min_depth, max_depth)
            is_obstacle = np.isfinite(depth_map) & (depth_map >= min_depth) & (depth_map < max_depth)

            # Step 2: Insert sensor depth inside the FOV region
            fov_effective = np.where(is_obstacle, depth_map, bg_dist).astype(np.float32)
            effective_depth[v_start:v_end, u_start:u_end] = fov_effective

            # Step 3: Handle near-field blind zone inside sensor FOV
            valid_mask = np.ones(u_grid.shape, dtype=bool)
            if ignore_blind_zone:
                is_blind = np.isfinite(depth_map) & (depth_map > 0.0) & (depth_map < min_depth)
                valid_mask[v_start:v_end, u_start:u_end] = ~is_blind

            z = effective_depth[valid_mask]
            u = u_grid[valid_mask]
            v = v_grid[valid_mask]

            if background_mode.lower() == 'radial':
                rx = (u - cx) / fx
                ry = (v - cy) / fy
                x = rx * z
                y = ry * z
                is_bg = np.ones(u_grid.shape, dtype=bool)
                is_bg[v_start:v_end, u_start:u_end] = ~is_obstacle
                is_bg_valid = is_bg[valid_mask]
                ray_len = np.sqrt(rx**2 + ry**2 + 1.0)
                scale = np.where(is_bg_valid, bg_dist / (ray_len * z), 1.0).astype(np.float32)
                x *= scale
                y *= scale
                z *= scale
            else:
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
        else:
            # Standard in-FOV background without padding
            u_coords = np.arange(0, width, decimation)
            v_coords = np.arange(0, height, decimation)
            u_grid, v_grid = np.meshgrid(u_coords, v_coords)

            is_obstacle = np.isfinite(depth_map) & (depth_map >= min_depth) & (depth_map < max_depth)
            if ignore_blind_zone:
                is_blind = np.isfinite(depth_map) & (depth_map > 0.0) & (depth_map < min_depth)
                valid_mask = ~is_blind
            else:
                valid_mask = np.ones_like(depth_map, dtype=bool)

            bg_dist = float(background_distance if background_distance > 0.0 else max_depth)
            effective_depth = np.where(is_obstacle, depth_map, bg_dist).astype(np.float32)

            z = effective_depth[valid_mask]
            u = u_grid[valid_mask]
            v = v_grid[valid_mask]

            if background_mode.lower() == 'radial':
                rx = (u - cx) / fx
                ry = (v - cy) / fy
                x = rx * z
                y = ry * z
                is_bg = ~is_obstacle[valid_mask]
                ray_len = np.sqrt(rx**2 + ry**2 + 1.0)
                scale = np.where(is_bg, bg_dist / (ray_len * z), 1.0).astype(np.float32)
                x *= scale
                y *= scale
                z *= scale
            else:
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
    else:
        # Fallback when fill_background is False
        u_coords = np.arange(0, width, decimation)
        v_coords = np.arange(0, height, decimation)
        u_grid, v_grid = np.meshgrid(u_coords, v_coords)
        valid_mask = np.isfinite(depth_map) & (depth_map > min_depth) & (depth_map < max_depth)
        z = depth_map[valid_mask]
        u = u_grid[valid_mask]
        v = v_grid[valid_mask]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

    # Stack into Nx3 points array
    points = np.column_stack((x, y, z)).astype(np.float32)
    return points


# Added: 3D Voxel Grid Filter with Centroid Averaging to simultaneously kill noise and downsample points
def voxel_grid_filter(points, voxel_size=0.05, min_points_per_voxel=2):
    """
    3D Voxel Grid Filter with Centroid Averaging and Outlier Rejection.
    
    1. Quantizes 3D space into uniform cubic voxels of size `voxel_size`.
    2. Computes the centroid (mean) of all points within each voxel, which averages out Gaussian depth sensor noise.
    3. Drops voxels having fewer than `min_points_per_voxel` points to eliminate floating speckles/dust artifacts.
    """
    if len(points) == 0:
        return points

    # Quantize into 3D voxel integer coordinates
    voxel_coords = np.floor(points / voxel_size).astype(np.int64)
    min_coords = voxel_coords.min(axis=0)
    voxel_coords -= min_coords
    dims = voxel_coords.max(axis=0) + 1

    # Map 3D coordinates to unique 1D hash indices
    h = voxel_coords[:, 0] + dims[0] * (voxel_coords[:, 1] + dims[1] * voxel_coords[:, 2])

    # Find unique voxels and inverse index mapping
    _, inv, counts = np.unique(h, return_inverse=True, return_counts=True)

    # Filter out voxels with too few points (flying speckle noise)
    valid_voxels = counts >= min_points_per_voxel
    if not np.any(valid_voxels):
        return np.empty((0, 3), dtype=np.float32)

    valid_counts = counts[valid_voxels]

    # Compute centroids by summing point coordinates in each voxel and dividing by count
    sum_x = np.bincount(inv, weights=points[:, 0])[valid_voxels]
    sum_y = np.bincount(inv, weights=points[:, 1])[valid_voxels]
    sum_z = np.bincount(inv, weights=points[:, 2])[valid_voxels]

    centroids = np.column_stack((sum_x / valid_counts, sum_y / valid_counts, sum_z / valid_counts)).astype(np.float32)
    return centroids


class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_node')

        # Declare parameters
        # Commented out generic default depth topic:
        # self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        # Added: Default to active RealSense D415 rectified depth topic
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')

        # Commented out generic default camera_info topic:
        # self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        # Added: Default to active RealSense D415 depth camera_info topic
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')

        self.declare_parameter('pointcloud_topic', '/camera/depth/points')
        # Commented out empty default frame_id:
        # self.declare_parameter('pointcloud_frame_id', '')
        # Added: Default frame name set to 'camera'
        self.declare_parameter('pointcloud_frame_id', 'camera')
        # Added: Frame convention parameter ('flu' = Forward/Left/Up, 'optical' = Right/Down/Forward)
        self.declare_parameter('frame_convention', 'flu')
        # Commented out 0.1m min depth:
        # self.declare_parameter('min_depth', 0.1)      # meters
        # Added: Blind zone / dead zone of 40cm (0.40m) to reject points closer than 40cm
        self.declare_parameter('min_depth', 0.4)      # meters (40cm blind zone)
        # self.declare_parameter('max_depth', 10.0)     # meters
        # Added: Hard 3.0m range limit to cut off points beyond 3 meters
        self.declare_parameter('max_depth', 2.5)      # meters
        # Commented out raw full resolution:
        # self.declare_parameter('decimation', 1)       # 1 = full, 2 = 2x downsampled
        # Added: Default 2x decimation cuts 75% redundant pixels before 3D voxelization
        self.declare_parameter('decimation', 2)
        # Added: Parameters for 3D Voxel Grid filtering (noise removal + spatial downsampling)
        self.declare_parameter('enable_voxel_filter', True)
        self.declare_parameter('voxel_size', 0.05)            # 5cm voxel cube matching Nav2 costmap resolution
        self.declare_parameter('min_points_per_voxel', 2)     # Discard voxels with < 2 points to reject floating noise/dust
        # Added: Parameters for 3m fake background canvas population
        self.declare_parameter('enable_fake_background', True)
        self.declare_parameter('background_distance', 2.5)        # Distance of canvas in meters
        self.declare_parameter('background_mode', 'plane')         # 'plane' (flat backdrop at 3m) or 'radial' (3m dome)
        self.declare_parameter('ignore_blind_zone_in_background', True) # Omit <40cm blind zone rather than painting at 3m
        # Added: Padding ratio to expand canvas beyond actual camera FOV (ensures edges always have points)
        self.declare_parameter('canvas_padding_ratio', 0.15)      # 15% bigger than camera FOV

        # Fallback intrinsics if no CameraInfo topic is available
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        # Retrieve parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.custom_frame_id = self.get_parameter('pointcloud_frame_id').value
        # Added: Retrieve frame convention ('flu' or 'optical')
        self.frame_convention = str(self.get_parameter('frame_convention').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.decimation = max(1, int(self.get_parameter('decimation').value))
        # Added: Retrieve voxel filter parameters
        self.enable_voxel_filter = bool(self.get_parameter('enable_voxel_filter').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.min_points_per_voxel = max(1, int(self.get_parameter('min_points_per_voxel').value))
        # Added: Retrieve 3m fake background canvas parameters
        self.enable_fake_background = bool(self.get_parameter('enable_fake_background').value)
        self.background_distance = float(self.get_parameter('background_distance').value)
        self.background_mode = str(self.get_parameter('background_mode').value)
        self.ignore_blind_zone_in_bg = bool(self.get_parameter('ignore_blind_zone_in_background').value)
        # Added: Retrieve canvas padding ratio
        self.canvas_padding_ratio = float(self.get_parameter('canvas_padding_ratio').value)

        # Intrinsics state
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.has_intrinsics = (self.fx > 0.0 and self.fy > 0.0)

        self.bridge = CvBridge()

        # Best effort sensor QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)

        # Subscriptions
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, sensor_qos)

        # Commented out original logger call:
        # self.get_logger().info(
        #     f"Depth-to-PointCloud node started.\n"
        #     f"  Subscribing to: {self.depth_topic} and {self.camera_info_topic}\n"
        #     f"  Publishing to:  {self.pointcloud_topic}\n"
        #     f"  Decimation:     {self.decimation}x | Depth range: [{self.min_depth:.2f}m, {self.max_depth:.2f}m]"
        # )
        # Commented out previous logger statement:
        # self.get_logger().info(
        #     f"Depth-to-PointCloud node started.\n"
        #     f"  Subscribing to: {self.depth_topic} and {self.camera_info_topic}\n"
        #     f"  Publishing to:  {self.pointcloud_topic} (frame_id: '{self.custom_frame_id or 'camera'}')\n"
        #     f"  Convention:     {self.frame_convention.upper()} (X:forward, Y:left, Z:up)\n"
        #     f"  Decimation:     {self.decimation}x | Depth range: [{self.min_depth:.2f}m, {self.max_depth:.2f}m] | Voxel: {self.voxel_size}m"
        # )
        # Commented out previous canvas_str:
        # canvas_str = f"ON ({self.background_distance:.1f}m {self.background_mode.upper()})" if self.enable_fake_background else "OFF"
        # Added: Enhanced logger statement including fake canvas details and FOV padding
        pad_str = f", +{int(self.canvas_padding_ratio*100)}% FOV border" if self.canvas_padding_ratio > 0.0 else ""
        canvas_str = f"ON ({self.background_distance:.1f}m {self.background_mode.upper()}{pad_str})" if self.enable_fake_background else "OFF"
        self.get_logger().info(
            f"Depth-to-PointCloud node started.\n"
            f"  Subscribing to: {self.depth_topic} and {self.camera_info_topic}\n"
            f"  Publishing to:  {self.pointcloud_topic} (frame_id: '{self.custom_frame_id or 'camera'}')\n"
            f"  Convention:     {self.frame_convention.upper()} (X:forward, Y:left, Z:up)\n"
            f"  Depth range:    [{self.min_depth:.2f}m, {self.max_depth:.2f}m] | Canvas: {canvas_str}\n"
            f"  Decimation:     {self.decimation}x | Voxel: {self.voxel_size}m"
        )

    def info_callback(self, msg: CameraInfo):
        """Extract intrinsic parameters from CameraInfo K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]"""
        if not self.has_intrinsics:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            if self.fx > 0.0 and self.fy > 0.0:
                self.has_intrinsics = True
                self.get_logger().info(
                    f"Camera intrinsics received: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}"
                )

    def depth_callback(self, msg: Image):
        """Process incoming depth image and publish 3D PointCloud2."""
        if not self.has_intrinsics:
            # Commented out ROS 1 style warn_throttle:
            # self.get_logger().warn_throttle(
            #     2.0, "Waiting for camera intrinsics on /camera_info topic (or set fx/fy/cx/cy parameters)..."
            # )
            # Added: ROS 2 Humble rclpy throttle logging using throttle_duration_sec
            self.get_logger().warn(
                "Waiting for camera intrinsics on /camera_info topic (or set fx/fy/cx/cy parameters)...",
                throttle_duration_sec=2.0
            )
            return

        # Convert ROS Image to NumPy 2D array
        try:
            if msg.encoding == '16UC1':
                # 16-bit depth in millimeters (standard RealSense/OpenNI) -> convert to meters
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_m = depth_raw.astype(np.float32) / 1000.0
            elif msg.encoding in ('32FC1', 'TYPE_32FC1'):
                # 32-bit floating point depth directly in meters
                depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                # Fallback passthrough
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if depth_raw.dtype == np.uint16:
                    depth_m = depth_raw.astype(np.float32) / 1000.0
                else:
                    depth_m = depth_raw.astype(np.float32)
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")
            return

        # Commented out previous depth_to_points_numpy call:
        # points = depth_to_points_numpy(
        #     depth_map=depth_m,
        #     fx=self.fx,
        #     fy=self.fy,
        #     cx=self.cx,
        #     cy=self.cy,
        #     min_depth=self.min_depth,
        #     max_depth=self.max_depth,
        #     decimation=self.decimation
        # )
        # Commented out previous depth_to_points_numpy call:
        # points = depth_to_points_numpy(
        #     depth_map=depth_m,
        #     fx=self.fx,
        #     fy=self.fy,
        #     cx=self.cx,
        #     cy=self.cy,
        #     min_depth=self.min_depth,
        #     max_depth=self.max_depth,
        #     decimation=self.decimation,
        #     fill_background=self.enable_fake_background,
        #     background_distance=self.background_distance,
        #     background_mode=self.background_mode,
        #     ignore_blind_zone=self.ignore_blind_zone_in_bg
        # )
        # Added: Vectorized inverse projection with expanded fake background canvas
        points = depth_to_points_numpy(
            depth_map=depth_m,
            fx=self.fx,
            fy=self.fy,
            cx=self.cx,
            cy=self.cy,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
            decimation=self.decimation,
            fill_background=self.enable_fake_background,
            background_distance=self.background_distance,
            background_mode=self.background_mode,
            ignore_blind_zone=self.ignore_blind_zone_in_bg,
            canvas_padding_ratio=self.canvas_padding_ratio
        )

        if len(points) == 0:
            return

        # Added: Apply 3D Voxel Grid filtering with centroid averaging to smooth sensor noise & reduce point count
        if self.enable_voxel_filter:
            points = voxel_grid_filter(
                points,
                voxel_size=self.voxel_size,
                min_points_per_voxel=self.min_points_per_voxel
            )

            if len(points) == 0:
                return

        # Added: Transform point coordinates to FLU body frame (X: forward, Y: left, Z: up) if requested
        if self.frame_convention.lower() == 'flu':
            # In optical frame: X is Right, Y is Down, Z is Forward
            # In FLU frame: X_flu = Z_opt (Forward), Y_flu = -X_opt (Left), Z_flu = -Y_opt (Up)
            points = np.column_stack((points[:, 2], -points[:, 0], -points[:, 1])).astype(np.float32)

        # Create PointCloud2 message
        out_msg = PointCloud2()
        out_msg.header = msg.header
        # Commented out previous frame_id assignment:
        # if self.custom_frame_id:
        #     out_msg.header.frame_id = self.custom_frame_id
        # Added: Set pointcloud frame_id explicitly to custom_frame_id (defaults to 'camera')
        if self.custom_frame_id:
            out_msg.header.frame_id = self.custom_frame_id
        else:
            out_msg.header.frame_id = 'camera'

        cloud_msg = pc2.create_cloud_xyz32(out_msg.header, points)
        self.pc_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
