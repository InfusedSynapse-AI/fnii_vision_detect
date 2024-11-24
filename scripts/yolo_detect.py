# YOLO 11
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import numpy as np
from cv_bridge import CvBridge
from fnii_vision_detect.srv import Detect, DetectResponse
from fnii_vision_detect.msg import Object, Objects, MultiStreamObjects
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
class YoloDetect:
    def __init__(
        self,
        ros_nh,
        model: str = "",
        imgsiz: int = 640,
        conf: int = 0.5,
    ):
        self.nh_ = ros_nh
        if model == "":
            rospy.logerr("please set model")
            raise ValueError("please set model")
        else:
            self.model_ = YOLO(model)
            self.model_.MODE(imgsize=imgsiz, conf=conf)
            rospy.ServiceProxy("detect", Detect, self.detect_callback)
            self.result_pub = rospy.Publisher("/detect_results", MultiStreamObjects, queue_size=10)
            self.result_img = list[np.ndarray]

    def ros_img_to_cv(self, img):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
        return cv_image

    def cv_to_ros_img(self, cv_image):
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        return ros_image

    def detect_callback(self, req):
        res = DetectResponse()
        try:
            ros_rgb = rospy.wait_for_message(req.rgb_topic, Image, timeout=2)
            res.results = self.process_yolo_result2fnii_ros_objects(self.detect_from_ros_img(ros_rgb))
            if len(res.results.results)==0:
                res.err_msg = "no object find"
            else:
                res.success = True
            
        except rospy.ROSException as e:
            rospy.logerr(e)
            res.err_msg = f"rgb topic: {req.rgb_topic} not pub"
        return res


    def detect_from_ros_img(self, img: Image):
        cv_image = self.ros_img_to_cv(img)
        results = self.model_(cv_image)
        return results

    def detect_fron_ndarray(self, img: np.ndarray):
        results = self.model_(img)
        return results

    def process_yolo_result2fnii_ros_objects(self, yolo_results, target: str = ""):
        my_results = MultiStreamObjects()
        self.result_img.clear()
        for result in yolo_results:
            self.result_img.append(result.plot())
            objects = Objects()
            class_names = result.boxes.cls.cpu().numpy()
            for idx, res in enumerate(result):
                tmp_object = Object()
                tmp_array = []
                tmp_object.object_name = result.names[int(class_names[idx])]
                if target!="" and tmp_object.object_name!=target:
                    continue
                tmp_object.bboxs = res.boxes.xyxy.cpu().numpy()[0].astype(np.uint32)
                tmp_object.confidence = res.boxes.conf.cpu().numpy()[0].astype(float)
                if res.masks is not None:
                    tmp_array = res.masks.data.cpu().numpy()[0, :, :].astype(int)
                    tmp_object.mask_height = len(tmp_array)
                    tmp_object.mask_width = len(tmp_array[0])
                    for a in tmp_array:
                        tmp_object.mask.extend(a)
                if res.keypoints is not None:
                    tmp_array = res.keypoints.data.cpu().numpy().astype(int)
                    for keypoint in tmp_array[0]:
                        tmp_object.keypoints.extend(keypoint)
                objects.objects.append(tmp_object)
            if len(objects.objects)>0:
                my_results.results.append(objects)
        return my_results
    def pointcloud2_to_array(self, cloud_msg:PointCloud2):
        """
        Convert a ROS PointCloud2 message to a numpy array without using ros_numpy.

        Args:
            pointcloud2 (PointCloud2): The PointCloud2 message.

        Returns:
            tuple: Tuple containing (xyz, rgb) arrays.
        """
        # 使用 point_cloud2.read_points() 来读取点云数据
        # 读取 XYZ 坐标和 RGB 数据
        points_list = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=False)

        # 将生成器转换为一个 NumPy 数组
        points_array = np.array(list(points_list))
        
        xyz = points_array[:, :3]  # 前三列是 (x, y, z) 坐标
        
        # 将 xyz 重塑为高度和宽度的形状
        xyz = xyz.reshape((cloud_msg.height, cloud_msg.width, 3))
        
        # 将包含 NaN 的行处理为 [0, 0, 0]，以防止损坏的数据
        nan_rows = np.isnan(xyz).all(axis=2)
        xyz[nan_rows] = [0, 0, 0]
        return xyz
    def get_mask_o3d_point(self, mask, cloud_msgs:PointCloud2):
        xyz = self.pointcloud2_to_array(cloud_msgs)
        mask_expanded = np.stack([mask, mask, mask], axis=2)
        obj_xyz = xyz * mask_expanded
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(obj_xyz.reshape((cloud_msgs.height * cloud_msgs.width, 3)))
        labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
        max_label = labels.max()
        # 统计每个聚类的点数
        unique_labels, counts = np.unique(labels, return_counts=True)
        # 找到点数最多的聚类（排除噪声点 -1）
        largest_cluster_label = unique_labels[np.argmax(counts[unique_labels >= 0])]
        largest_cluster_indices = np.where(labels == largest_cluster_label)[0]
        largest_cluster_points = np.asarray(pcd.points)[largest_cluster_indices]
        # 创建一个新的点云，仅包含最大的聚类
        # largest_cluster_pcd = o3d.geometry.PointCloud()
        # largest_cluster_pcd.points = o3d.utility.Vector3dVector(largest_cluster_points)
        # return largest_cluster_pcd

        return largest_cluster_points

    def get_centor_point(self, points:np.array):
        return points.mean(axis=0)

    def get_mask_depth(self, mask, depth:np.ndarray):
        obj = depth[mask == 1]
        obj = obj[~np.isnan(obj)]




                

            
