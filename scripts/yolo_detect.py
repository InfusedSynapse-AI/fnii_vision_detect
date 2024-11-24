# YOLO 11
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from cv_bridge import CvBridge
from fnii_vision_detect.srv import Detect, DetectResponse
from fnii_vision_detect.msg import Object, Objects


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
            self.model_.predict(imgsize=imgsiz, conf=conf)
            rospy.ServiceProxy("detect", Detect, self.detect_callback)
            self.self.result = rospy.Publisher("/detect_image", Image, queue_size=10)
            self.masks = list[np.ndarray]
            self.bbox = list[np.ndarray]

    def ros_img_to_cv(self, img):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
        return cv_image

    def cv_to_ros_img(self, cv_image):
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        return ros_image

    def detect_callback(self, req):
        cv_image = self.ros_img_to_cv(req.image)
        result = self.model_.predict(cv_image)
        result_image = self.cv_to_ros_img(result)
        self.result.publish(result_image)
        return DetectResponse(result_image)

    def detect(self, img: Image):
        cv_image = self.ros_img_to_cv(img)
        results = self.model_.predict(cv_image)
        return results

    def detect(self, img: np.ndarray):
        results = self.model_.predict(img)
        return results

    def ProcesYoloResultToFniiRosObjects(self, results, target: str = ""):
        objs = Objects()
        for result in results:
            im = result.plot()
            if len(result.names)==0:
                continue    

            for ci, c in enumerate(result):
                my_obj = Object()
                my_obj.class_ = c.names
                if len(labels) > 0:
                
                if len(c.masks.xy) > 0:
                    for mask in enumerate(c.masks.xy):
                        

    # def rgb_callback(self, msg):
