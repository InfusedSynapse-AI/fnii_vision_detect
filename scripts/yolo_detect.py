# YOLO 11
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Quaternion
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
import numpy as np
from cv_bridge import CvBridge
from fnii_vision_detect.srv import Detect, DetectResponse, DetectRequest
from fnii_vision_detect.msg import Object, Objects, MultiStreamObjects
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from get_point import GetPoint
import math
import cv2


class YoloDetect:
    def __init__(
        self,
        model: str = "",
        imgsiz: int = 640,
        conf: int = 0.5,
    ):
        if model == "":
            rospy.logerr("please set model")
            raise ValueError("please set model")
        else:
            self.model_ = YOLO(model)
            # self.model_.MODE(imgsize=imgsiz, conf=conf)
            self.server = rospy.Service("detect", Detect, self.detect_callback)
            self.result_pub = rospy.Publisher(
                "detect_results", MultiStreamObjects, queue_size=10, latch=True
            )
            self.result_img_pub = rospy.Publisher(
                "detect_results_img", Image, queue_size=10, latch=True
            )
            self.result_img = []
            self.get_point_tool = GetPoint()
            self.tf_buffer = Buffer()
            self.listener = TransformListener(self.tf_buffer)

    def ros_img_to_cv(self, img, encoding="bgr8"):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img, encoding)
        return cv_image

    def cv_to_ros_img(self, cv_image, encoding="bgr8"):
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding)
        return ros_image
    def get_transform(self, ref_frame, frame_id, time=1.0):
        # self.listener.waitForTransform(ref_frame, frame_id, rospy.Time(0), rospy.Duration(time))
        try_time = 0
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(
                    ref_frame, frame_id, rospy.Time(0.5), rospy.Duration(0.1)
                )
                return transform
            except Exception as e:
                try_time += 1
                if try_time > 20:
                    rospy.logerr(e)
                    return None
    def detect_callback(self, req: DetectRequest):
        res = DetectResponse()
        res.success = True
        try:
            ros_rgb = rospy.wait_for_message(req.rgb_topic, Image)
            ros_depth = None
            if req.depth_topic != "" and req.depth_cam_info != "":
                ros_depth = rospy.wait_for_message(req.depth_topic, Image)
                depth_cam_info = rospy.wait_for_message(req.depth_cam_info, CameraInfo)
                self.get_point_tool.set_camera_info_from_msg(depth_cam_info)
                print("will cauculate point after detect")

            detect_results = self.process_yolo_result2fnii_ros_objects(
                self.detect_from_ros_img(ros_rgb), req.target_class
            )
            if len(detect_results.results) == 0:
                res.success = False
                res.err_msg = "no object find"
            else:
                if ros_depth is not None:
                    transform = None
                    if req.frame_id != "":
                        transform = self.get_transform(req.frame_id,
                            ros_depth.header.frame_id, 0.5)

                    depth_image = self.ros_img_to_cv(ros_depth, encoding="passthrough")
                    # detect_results 考虑到多个输入源的情况，但目前只考虑一个输入源
                    for objs in detect_results.results:
                        combined_img = None
                        for obj in objs.objects:
                            if req.cal_centor_method == 1:
                                if len(obj.mask) == 0:
                                    bbox = np.array(obj.bboxs).reshape((2, 2))
                                    point_pixel = bbox.mean(axis=0)
                                    point_pixel = point_pixel.astype(int)
                                    point = (
                                        self.get_point_tool.get_one_point_from_depth(
                                            depth_image,
                                            point_pixel[0],
                                            point_pixel[1],
                                            auto_fix=True,
                                        )
                                    )
                                    if point is not None:
                                        tmp_ros_point = Point()
                                        tmp_ros_point.x = point[0]
                                        tmp_ros_point.y = point[1]
                                        tmp_ros_point.z = point[2]
                                        obj.centor_point.append(tmp_ros_point)
                                else:
                                    height = obj.mask_height
                                    width = obj.mask_width
                                    mymask = np.array(obj.mask)
                                    mask_tmp = mymask.reshape((height, width))
                                    masked_depth = depth_image.copy()
                                    masked_depth[mask_tmp == 0] = 0
                                    if combined_img is None:
                                        combined_img = masked_depth
                                    else:
                                        combined_img = np.maximum(
                                            combined_img, masked_depth
                                        )
                                    points = self.get_point_tool.get_points_from_depth(
                                        masked_depth, need_cluster=True
                                    )
                                    if len(points) > 0:
                                        mean_point = np.mean(points[0], axis=0)
                                        tmp_ros_point = Point()
                                        tmp_ros_point.x = mean_point[0]
                                        tmp_ros_point.y = mean_point[1]
                                        tmp_ros_point.z = mean_point[2]
                                        obj.centor_point.append(tmp_ros_point)
                            elif req.cal_centor_method == 2:
                                if len(obj.keypoints) == 0:
                                    print(
                                        "no keypoints, can't get geometry_center, and will caluculate centor by bbox"
                                    )
                                    bbox = np.array(obj.bboxs).reshape((2, 2))
                                    point_pixel = bbox.mean(axis=0)
                                    point_pixel = point_pixel.astype(int)
                                    point = (
                                        self.get_point_tool.get_one_point_from_depth(
                                            depth_image,
                                            point_pixel[0],
                                            point_pixel[1],
                                            auto_fix=True,
                                        )
                                    )
                                    if point is not None:
                                        tmp_ros_point = Point()
                                        tmp_ros_point.x = point[0]
                                        tmp_ros_point.y = point[1]
                                        tmp_ros_point.z = point[2]
                                        obj.centor_point.append(tmp_ros_point)
                                else:
                                    keypoints = np.array(obj.keypoints).reshape((2, 2))
                                    point_pixel = keypoints.mean(axis=0)
                                    point = (
                                        self.get_point_tool.get_one_point_from_depth(
                                            depth_image,
                                            point_pixel[0],
                                            point_pixel[1],
                                            auto_fix=True,
                                            fix_ux=keypoints
                                        )
                                    )
                                    if point is not None:
                                        tmp_ros_point = Point()
                                        tmp_ros_point.x = point[0]
                                        tmp_ros_point.y = point[1]
                                        tmp_ros_point.z = point[2]
                                        obj.centor_point.append(tmp_ros_point)
                            else:
                                bbox = np.array(obj.bboxs).reshape((2, 2))
                                point_pixel = bbox.mean(axis=0)
                                point_pixel = point_pixel.astype(int)
                                point = self.get_point_tool.get_one_point_from_depth(
                                    depth_image,
                                    point_pixel[0],
                                    point_pixel[1],
                                    auto_fix=True,
                                )
                                if point is not None:
                                    tmp_ros_point = Point()
                                    tmp_ros_point.x = point[0]
                                    tmp_ros_point.y = point[1]
                                    tmp_ros_point.z = point[2]
                                    obj.centor_point.append(tmp_ros_point)
                            if transform is not None and len(obj.centor_point) > 0:
                                obj.centor_point = tf2_geometry_msgs.do_transform_point(obj.centor_point, transform).point
                        if combined_img is not None:
                            combined_img = self.depth_to_gray_color(combined_img)
                            self.result_img.append(combined_img)
                self.result_pub.publish(detect_results)
                img = self.stitch_n_images_cv2(self.result_img, (1280, 960))
                self.result_img_pub.publish(self.cv_to_ros_img(img))
                res.results = detect_results
        except rospy.ROSException as e:
            rospy.logerr(e)
            res.err_msg = f"topic: {req.rgb_topic} not pub"
        return res

    def detect_from_ros_img(self, img: Image):
        cv_image = self.ros_img_to_cv(img)
        results = self.model_(cv_image)
        return results

    def detect_from_ndarray(self, img: np.ndarray):
        results = self.model_(img)
        return results

    def process_yolo_result2fnii_ros_objects(
        self, yolo_results, target: str = ""
    ) -> MultiStreamObjects:
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
                if target != "" and tmp_object.object_name != target:
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
            if len(objects.objects) > 0:
                my_results.results.append(objects)
        return my_results

    def depth_to_gray_color(self, depth_image):
        """
        将深度图转换为灰度图，方便显示。
        """
        depth_image = np.clip(depth_image, 0, 3)
        grayscale_image = (depth_image * 50).astype(np.uint8)

        # grayscale_image = cv2.applyColorMap(grayscale_image, cv2.COLORMAP_JET)
        # cv2.imwrite("depth.jpg", grayscale_image)
        return cv2.cvtColor(grayscale_image, cv2.COLOR_GRAY2BGR)

    def resize_and_pad_cv2(self, image, target_size):
        """
        使用 OpenCV 调整图片大小以适配目标尺寸，同时保持宽高比。
        用黑色填充剩余部分，使图片尺寸与目标一致。
        """
        h, w = image.shape[:2]
        target_w, target_h = target_size
        scale = min(target_w / w, target_h / h)  # 缩放比例，保证不变形
        new_w, new_h = int(w * scale), int(h * scale)  # 计算缩放后的尺寸
        resized = cv2.resize(
            image, (new_w, new_h), interpolation=cv2.INTER_AREA
        )  # 缩放图片

        # 创建黑色背景并将图片粘贴到中心
        padded_image = np.ones((target_h, target_w, 3), dtype=np.uint8)*255
        offset_x = (target_w - new_w) // 2
        offset_y = (target_h - new_h) // 2
        padded_image[offset_y : offset_y + new_h, offset_x : offset_x + new_w] = resized
        return padded_image

    def stitch_n_images_cv2(self, images, output_size=(640, 480)):
        """
        使用 OpenCV 将 n 张图片按行列动态排列，生成指定大小的图片（默认 640, 480）。
        如果图片不足，空白区域用黑色填充。
        """
        n = len(images)
        if n == 0:
            return np.zeros(
                (output_size[1], output_size[0], 3), dtype=np.uint8
            )  # 返回全黑图片

        # 动态计算行数和列数
        rows = math.ceil(math.sqrt(n))  # 行数
        cols = math.ceil(n / rows)  # 列数

        # 每个子图的目标大小
        sub_width = output_size[0] // cols
        sub_height = output_size[1] // rows

        # 创建最终拼接的黑色背景图片
        stitched_image = np.ones((output_size[1], output_size[0], 3), dtype=np.uint8)*255

        for idx, img in enumerate(images):
            resized_img = self.resize_and_pad_cv2(
                img, (sub_width, sub_height)
            )  # 调整大小并填充
            # 计算子图位置
            row, col = divmod(idx, cols)
            x_start, y_start = col * sub_width, row * sub_height
            stitched_image[
                y_start : y_start + sub_height, x_start : x_start + sub_width
            ] = resized_img  # 放置图片

        return stitched_image
