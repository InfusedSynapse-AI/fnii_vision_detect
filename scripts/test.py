from get_point import GetPoint
from yolo_detect import YoloDetect
import numpy as np
import cv2

# rospy.init_node("test")
yolo_detector = YoloDetect(model="yolo11m-seg")
results = yolo_detector.detect_from_ndarray(cv2.imread("/home/zhaoqing/wulong_color.png"))
depth = cv2.imread("/home/zhaoqing/wulong_depth.png")
img = results[0].plot()
class_names = results[0].boxes.cls.cpu().numpy()
object_name = results[0].names[int(class_names[1])]
print(object_name)
masks = results[0][1].masks.data.cpu().numpy()[0, :, :].astype(int)
print(masks)
masked_depth = depth.copy()
masked_depth[masks == 0] = 0
print(depth[320, 240])

cv2.imwrite("/home/zhaoqing/detected_image.png", img)
cv2.imwrite("/home/zhaoqing/detected_depth_image.png", masked_depth)
