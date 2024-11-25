import open3d as o3d
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel


class GetPoint:
    def __init__(self):
        self.pcd = None
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
    def set_camera_info(self, fx: float, fy: float, cx: float, cy: float):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
    def set_camera_info_from_msg(self, camera_info_msg: CameraInfo):
        self.fx = camera_info_msg.K[0]
        self.fy = camera_info_msg.K[4]
        self.cx = camera_info_msg.K[2]
        self.cy = camera_info_msg.K[5]
        
    def get_point_from_depth(
        self,
        depth: np.ndarray,
        need_cluster: bool = False
    ):
        """
        Get the center point of the object from the depth image.

        Args:
            depth (np.ndarray): The depth image.
            need_cluster (bool): Whether to cluster the object.

        Returns:
            np.ndarray: The center point of the object.
        """
        points_array = []
        points = np.zeros((0.0, 0.0, 0.0))  # 创建一个二维空数组
        for uy in depth.shape[0]:
            for ux in depth.shape[1]:
                if np.isnan(depth[uy, ux]) and depth[uy, ux]==0:
                    continue
                z = depth[uy, ux]*0.001 # 将深度值转换为米
                if z == 0:
                    continue
                x = (ux - self.cx) * z / self.fx
                y = (uy - self.cy) * z / self.fy
                points = np.vstack((points, np.array([x, y, z])))
        if need_cluster:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
            max_label = labels.max()
            for i in range(max_label):
                if i == -1:
                    continue
                cluster_indices = np.where(labels == i)[0]
                cluster_points = np.asarray(pcd.points)[cluster_indices]
                points_array.append(cluster_points)
        else:
            points_array.append(points)
        return points_array
            


                
            

        return points
