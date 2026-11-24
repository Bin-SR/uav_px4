#!/usr/bin/env python3
import time
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge


class PointCloudProcessorNode(Node):
    def __init__(self):
        super().__init__('pointcloud_processor_node')
        self.bridge = CvBridge()
        self.mask = None
        self.mask_header = None
        self.rgb_intrinsics = None
        self.depth_intrinsics = None
        self.last_process_time = 0.0
        self.last_log_time = 0.0

        # Tunable parameters: values are in metres unless noted.
        defaults = {
            'voxel_size': 0.04,
            'statistical_nb_neighbors': 20,
            'statistical_std_ratio': 2.0,
            'remove_ground_plane': True,
            'plane_distance_threshold': 0.06,
            'plane_ransac_n': 3,
            'plane_num_iterations': 100,
            'target_dbscan_eps': 0.10,
            'target_dbscan_min_points': 8,
            'scene_dbscan_eps': 0.18,
            'scene_dbscan_min_points': 10,
            'min_target_points': 30,
            'min_cluster_points': 20,
            'max_depth': 12.0,
            'process_hz': 2.0,
            'max_mask_cloud_dt': 0.25,
            'visualize_open3d': True,
            'publish_scene_clusters': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda n: self.get_parameter(n).value
        self.voxel_size = float(p('voxel_size'))
        self.nb_neighbors = int(p('statistical_nb_neighbors'))
        self.std_ratio = float(p('statistical_std_ratio'))
        self.remove_ground_plane = bool(p('remove_ground_plane'))
        self.plane_distance_threshold = float(p('plane_distance_threshold'))
        self.plane_ransac_n = int(p('plane_ransac_n'))
        self.plane_num_iterations = int(p('plane_num_iterations'))
        self.target_eps = float(p('target_dbscan_eps'))
        self.target_min_points = int(p('target_dbscan_min_points'))
        self.scene_eps = float(p('scene_dbscan_eps'))
        self.scene_min_points = int(p('scene_dbscan_min_points'))
        self.min_target_points = int(p('min_target_points'))
        self.min_cluster_points = int(p('min_cluster_points'))
        self.max_depth = float(p('max_depth'))
        self.process_period = 1.0 / max(0.1, float(p('process_hz')))
        self.max_mask_cloud_dt = float(p('max_mask_cloud_dt'))
        self.publish_scene_clusters = bool(p('publish_scene_clusters'))
        self.visualize = bool(p('visualize_open3d'))

        self.create_subscription(Image, '/red_box/mask', self.mask_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.rgb_info_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.depth_info_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/camera/depth/points', self.cloud_cb, qos_profile_sensor_data)

        self.position_pub = self.create_publisher(PointStamped, '/red_box/position', 10)
        self.size_pub = self.create_publisher(Vector3Stamped, '/red_box/size', 10)
        self.marker_pub = self.create_publisher(Marker, '/red_box/marker', 10)
        self.target_cloud_pub = self.create_publisher(PointCloud2, '/red_box/points', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, '/pointcloud/filtered', 10)
        self.cluster_marker_pub = self.create_publisher(MarkerArray, '/pointcloud/clusters', 10)

        self.vis = None
        self.vis_cloud = None
        self.vis_geometries = []
        if self.visualize:
            try:
                self.vis = o3d.visualization.Visualizer()
                self.vis.create_window('UAV PointCloud Filtering and Clustering', 1100, 800)
                self.vis_cloud = o3d.geometry.PointCloud()
            except Exception as exc:
                self.get_logger().warning(f'Open3D window unavailable: {exc}')
                self.vis = None

        self.timer = self.create_timer(0.03, self.visualization_spin)
        self.pending_scene = None
        self.get_logger().info('PointCloud Processor Node started')

    def mask_cb(self, msg):
        try:
            self.mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self.mask_header = msg.header
        except Exception as exc:
            self.get_logger().error(f'Mask conversion failed: {exc}')

    def rgb_info_cb(self, msg):
        self.rgb_intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def depth_info_cb(self, msg):
        self.depth_intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def mask_registered_to_cloud(self, cloud_h, cloud_w):
        """Map each depth/cloud pixel to an RGB mask pixel.

        This pinhole-only mapping assumes RGB and depth optical frames are co-located
        and parallel. A non-zero RGB-depth extrinsic requires a true 3-D reprojection.
        """
        if self.mask is None:
            return None
        mh, mw = self.mask.shape[:2]
        if self.rgb_intrinsics is None or self.depth_intrinsics is None:
            if (mh, mw) == (cloud_h, cloud_w):
                return self.mask > 0
            import cv2
            self.get_logger().warning(
                'CameraInfo not ready: resizing RGB mask to cloud dimensions as a temporary fallback')
            return cv2.resize(self.mask, (cloud_w, cloud_h), interpolation=cv2.INTER_NEAREST) > 0

        rfx, rfy, rcx, rcy = self.rgb_intrinsics
        dfx, dfy, dcx, dcy = self.depth_intrinsics
        if min(rfx, rfy, dfx, dfy) <= 0:
            return None

        uu, vv = np.meshgrid(np.arange(cloud_w, dtype=np.float32),
                             np.arange(cloud_h, dtype=np.float32))
        x_norm = (uu - dcx) / dfx
        y_norm = (vv - dcy) / dfy
        u_rgb = np.rint(rfx * x_norm + rcx).astype(np.int32)
        v_rgb = np.rint(rfy * y_norm + rcy).astype(np.int32)
        valid = (u_rgb >= 0) & (u_rgb < mw) & (v_rgb >= 0) & (v_rgb < mh)
        registered = np.zeros((cloud_h, cloud_w), dtype=bool)
        registered[valid] = self.mask[v_rgb[valid], u_rgb[valid]] > 0
        return registered

    @staticmethod
    def xyz_from_msg(msg):
        # ROS 2 Humble-compatible conversion. Keep the organized HxW pixel layout.
        points = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        points = np.asarray(points)
        if points.dtype.names is not None:
            xyz = np.column_stack((points['x'], points['y'], points['z']))
        else:
            xyz = np.asarray(list(points), dtype=np.float32)
        xyz = np.asarray(xyz, dtype=np.float32).reshape(msg.height, msg.width, 3)
        return xyz

    def cloud_cb(self, msg):
        now = time.monotonic()
        if now - self.last_process_time < self.process_period:
            return
        self.last_process_time = now

        if self.mask is None or self.mask_header is None:
            return

        # Reject clearly stale mask/cloud pairs. The two sensors are not guaranteed
        # to have identical stamps, so allow a configurable tolerance.
        mask_t = (self.mask_header.stamp.sec +
                  self.mask_header.stamp.nanosec * 1e-9)
        cloud_t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if mask_t > 0.0 and cloud_t > 0.0:
            if abs(mask_t - cloud_t) > self.max_mask_cloud_dt:
                return

        try:
            organized = self.xyz_from_msg(msg)
        except Exception as exc:
            self.get_logger().error(f'PointCloud2 conversion failed: {exc}')
            return

        h, w, _ = organized.shape
        target_mask = self.mask_registered_to_cloud(h, w)
        if target_mask is None:
            self.get_logger().warning('Waiting for valid RGB/Depth CameraInfo')
            return

        flat = organized.reshape(-1, 3)
        ranges = np.linalg.norm(flat, axis=1)
        valid_scene = (
            np.isfinite(flat).all(axis=1)
            & (ranges > 0.05)
            & (ranges < self.max_depth)
        )
        scene_points = flat[valid_scene]
        if scene_points.shape[0] < 20:
            return

        # 1) Full-scene filtering: voxel downsample + statistical outlier removal.
        scene_pcd = o3d.geometry.PointCloud()
        scene_pcd.points = o3d.utility.Vector3dVector(scene_points.astype(np.float64))
        if self.voxel_size > 0:
            scene_pcd = scene_pcd.voxel_down_sample(self.voxel_size)
        if len(scene_pcd.points) > max(self.nb_neighbors, 30):
            scene_pcd, _ = scene_pcd.remove_statistical_outlier(
                nb_neighbors=self.nb_neighbors, std_ratio=self.std_ratio)

        # 1b) Remove the dominant plane (normally the ground) before obstacle clustering.
        # The red target extraction below still uses the original organized cloud + MASK.
        if self.remove_ground_plane and len(scene_pcd.points) >= 100:
            try:
                _, inliers = scene_pcd.segment_plane(
                    distance_threshold=self.plane_distance_threshold,
                    ransac_n=self.plane_ransac_n,
                    num_iterations=self.plane_num_iterations)
                if len(inliers) > 0:
                    scene_pcd = scene_pcd.select_by_index(inliers, invert=True)
            except Exception as exc:
                self.get_logger().warning(f'Ground plane removal skipped: {exc}')

        filtered = np.asarray(scene_pcd.points, dtype=np.float32)
        if filtered.size:
            filtered_msg = point_cloud2.create_cloud_xyz32(msg.header, filtered.tolist())
            self.filtered_cloud_pub.publish(filtered_msg)

        # 2) Target extraction from the full 2-D mask, preserving organized-cloud pixel correspondence.
        target_points = organized[target_mask]
        target_ranges = np.linalg.norm(target_points, axis=1)
        target_valid = (
            np.isfinite(target_points).all(axis=1)
            & (target_ranges > 0.05)
            & (target_ranges < self.max_depth)
        )
        target_points = target_points[target_valid]
        if target_points.shape[0] < self.min_target_points:
            self.publish_delete_target_marker(msg.header)
            return



        # 点云数量
        # if len(object_points) < 200:
        #     self.publish_delete_target_marker(msg.header)
        #     return
        # 尺寸异常检查
        # 还有待商榷，因为比如朝向正面，那就会丢失长度x的信息，变成0.01
        # 怎么加入一个目标有效状态话题，eg. /reb_box/valid, 消息类型可以是std_msgs/Bool
        # if np.any(size < 0.05):
        #     self.publish_delete_target_marker(msg.header)
        #     return



        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(target_points.astype(np.float64))

        # YOLO provides a rectangular 2-D box, so its mask can include ground
        # pixels. For the sphere experiment, remove a dominant plane from this
        # cropped target cloud before clustering. This is configurable because
        # planar targets may need this disabled in later experiments.
        try:
            if len(target_pcd.points) >= 80:
                _, target_plane_inliers = target_pcd.segment_plane(
                    distance_threshold=0.035,
                    ransac_n=3,
                    num_iterations=120)
                plane_ratio = len(target_plane_inliers) / float(len(target_pcd.points))
                if len(target_plane_inliers) >= 30 and plane_ratio < 0.85:
                    target_pcd = target_pcd.select_by_index(
                        target_plane_inliers, invert=True)
        except Exception as exc:
            self.get_logger().warning(f'Target plane removal skipped: {exc}')

        if self.voxel_size > 0:
            target_pcd = target_pcd.voxel_down_sample(max(self.voxel_size * 0.5, 0.01))
        if len(target_pcd.points) > max(self.nb_neighbors, 30):
            target_pcd, _ = target_pcd.remove_statistical_outlier(
                nb_neighbors=min(self.nb_neighbors, max(5, len(target_pcd.points)-1)),
                std_ratio=self.std_ratio)
        target_filtered = np.asarray(target_pcd.points, dtype=np.float32)
        if target_filtered.shape[0] < self.min_target_points:
            self.publish_delete_target_marker(msg.header)
            return

        # 3) DBSCAN on target points. Select the largest non-noise cluster.
        labels = np.asarray(target_pcd.cluster_dbscan(
            eps=self.target_eps, min_points=self.target_min_points, print_progress=False))
        valid_labels = labels[labels >= 0]
        if valid_labels.size:
            ids, counts = np.unique(valid_labels, return_counts=True)
            chosen = int(ids[np.argmax(counts)])
            object_points = target_filtered[labels == chosen]
        else:
            # If DBSCAN finds no cluster, retain the filtered target points as a
            # conservative fallback. This is preferable to leaving object_points
            # undefined; the downstream world-state node still applies freshness
            # and finite/positive-size checks.
            object_points = target_filtered



        
        if object_points.shape[0] < self.min_target_points:
            self.publish_delete_target_marker(msg.header)
            return

        # Percentile bounds reduce the influence of a few residual edge outliers.
        lower = np.percentile(object_points, 2.0, axis=0)
        upper = np.percentile(object_points, 98.0, axis=0)
        center = (lower + upper) * 0.5
        size = np.maximum(upper - lower, 0.01)

        pos_msg = PointStamped()
        pos_msg.header = msg.header
        pos_msg.point.x, pos_msg.point.y, pos_msg.point.z = map(float, center)
        self.position_pub.publish(pos_msg)

        size_msg = Vector3Stamped()
        size_msg.header = msg.header
        size_msg.vector.x, size_msg.vector.y, size_msg.vector.z = map(float, size)
        self.size_pub.publish(size_msg)

        target_msg = point_cloud2.create_cloud_xyz32(msg.header, object_points.tolist())
        self.target_cloud_pub.publish(target_msg)
        self.publish_target_marker(msg.header, center, size)

        # 4) Whole-scene DBSCAN for other visible clusters / obstacles.
        if self.publish_scene_clusters and filtered.shape[0] >= self.min_cluster_points:
            self.publish_scene_dbscan(msg.header, filtered)

        self.pending_scene = filtered
        if now - self.last_log_time > 1.0:
            self.get_logger().info(
                f'Target points={len(object_points)}, center='
                f'({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}), '
                f'size=({size[0]:.2f}, {size[1]:.2f}, {size[2]:.2f}), '
                f'filtered scene points={len(filtered)}')
            self.last_log_time = now

    def publish_target_marker(self, header, center, size):
        marker = Marker()
        marker.header = header
        marker.ns = 'detected_target'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, center)
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.05, 0.05, 0.45
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)

    def publish_delete_target_marker(self, header):
        marker = Marker()
        marker.header = header
        marker.ns = 'detected_target'
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def publish_scene_dbscan(self, header, points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        labels = np.asarray(pcd.cluster_dbscan(
            eps=self.scene_eps, min_points=self.scene_min_points, print_progress=False))
        marker_array = MarkerArray()
        cluster_ids = [int(x) for x in np.unique(labels) if x >= 0]
        palette = [
            (0.1, 0.8, 0.2), (0.1, 0.4, 1.0), (1.0, 0.6, 0.0),
            (0.8, 0.2, 0.8), (0.0, 0.8, 0.8), (1.0, 1.0, 0.1)
        ]
        for idx, cid in enumerate(cluster_ids[:30]):
            cluster = points[labels == cid]
            if len(cluster) < self.min_cluster_points:
                continue
            lo, hi = np.percentile(cluster, [2, 98], axis=0)
            c = (lo + hi) * 0.5
            s = np.maximum(hi - lo, 0.03)
            marker = Marker()
            marker.header = header
            marker.ns = 'scene_clusters'
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, c)
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, s)
            color = palette[idx % len(palette)]
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 0.35
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)

        # Explicitly delete old IDs when fewer clusters are present this frame.
        for idx in range(len(cluster_ids[:30]), 30):
            marker = Marker()
            marker.header = header
            marker.ns = 'scene_clusters'
            marker.id = idx
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        self.cluster_marker_pub.publish(marker_array)

    def visualization_spin(self):
        if self.vis is None:
            return
        if self.pending_scene is not None and len(self.pending_scene):
            self.vis_cloud.points = o3d.utility.Vector3dVector(self.pending_scene.astype(np.float64))
            if not self.vis_geometries:
                self.vis.add_geometry(self.vis_cloud)
                self.vis_geometries.append(True)
            else:
                self.vis.update_geometry(self.vis_cloud)
            self.pending_scene = None
        try:
            self.vis.poll_events()
            self.vis.update_renderer()
        except Exception as exc:
            self.get_logger().warning(f'Open3D visualizer stopped: {exc}')
            self.vis = None

    def destroy_node(self):
        if self.vis is not None:
            self.vis.destroy_window()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
