import open3d as o3d
import numpy as np
from sensor_msgs.msg import CameraInfo

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

    def get_one_point_from_depth(
        self, depth: np.ndarray, ux: int, uy: int, auto_fix: bool = False, fix_ux: int = 5, fix_uy: int = 5
    ):
        """
        Get the center point of the object from the depth image.

        Args:
            depth (np.ndarray): The depth image.
            ux (int): The x coordinate of the object.
            uy (int): The y coordinate of the object.

        Returns:
            np.ndarray: The center point of the object.
        """
        if np.isnan(depth[uy, ux]) or depth[uy, ux] == 0:
            print("The target depth value is NaN or 0.")
            find_near = False
            if not auto_fix:
                return None
            else:
                print("Try to find the nearest point.")
                for i in range(1, 5):
                    if find_near:
                        break
                    for j in range(1, 5):
                        if (
                            ux + i < depth.shape[1]
                            and uy + j < depth.shape[0]
                            and depth[uy + j, ux + i] != 0
                            and np.isnan(depth[uy + j, ux + i])
                        ):
                            ux = ux + i
                            uy = uy + j
                            find_near = True
                            break
                        if (
                            ux - i >= 0
                            and uy - j >= 0
                            and depth[uy - j, ux - i] != 0
                            and np.isnan(depth[uy - j, ux - i])
                        ):
                            ux = ux - i
                            uy = uy - j
                            find_near = True
                            break
                        if (
                            ux + i < depth.shape[1]
                            and uy - j >= 0
                            and depth[uy - j, ux + i] != 0
                            and np.isnan(depth[uy - j, ux + i])
                        ):
                            ux = ux + i
                            uy = uy - j
                            find_near = True
                            break
                        if (
                            ux - i >= 0
                            and uy + j < depth.shape[0]
                            and depth[uy + j, ux - i] != 0
                            and np.isnan(depth[uy + j, ux - i])
                        ):
                            ux = ux - i
                            uy = uy + j
                            find_near = True
                            break
                if not find_near:
                    print("No valid point found.")
                    return None
        z = depth[uy, ux] * 0.001
        x = (ux - self.cx) * z / self.fx
        y = (uy - self.cy) * z / self.fy
        return np.array([x, y, z])

    def get_points_from_depth(self, depth: np.ndarray, need_cluster: bool = False):
        """
        Get the center point of the object from the depth image.

        Args:
            depth (np.ndarray): The depth image.
            need_cluster (bool): Whether to cluster the object.

        Returns:
            np.ndarray: The center point of the object.
        """
        points_array = []
        points = []  # 创建一个二维空数组
        for uy in range(depth.shape[0]):
            for ux in range(depth.shape[1]):
                if np.isnan(depth[uy, ux]) or depth[uy, ux] == 0:
                    continue
                z = depth[uy, ux] * 0.001  # 将深度值转换为米
                if z == 0:
                    continue
                x = (ux - self.cx) * z / self.fx
                y = (uy - self.cy) * z / self.fy
                points.append([x, y, z])
        if points == []:
            return None
        if need_cluster:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            labels = np.array(
                pcd.cluster_dbscan(eps=0.05, min_points=100)
            )
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

