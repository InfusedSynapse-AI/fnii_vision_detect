
# YOLO 11
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image, CameraInfo
class YoloDetect:
    def __init__(self, ros_nh, rgb_topic: str="", real_time:bool=False, model:str="", imgsiz:int=640, conf:int=0.5):
        self.nh_ = ros_nh
        if model=="":
            rospy.logerr("please set model")
            raise ValueError("please set model")
        else:
            self.model_ = YOLO(model)
        if real_time:
            if rgb_topic=="":
                rospy.logerr("please set rgb topic")
                raise ValueError("please set rgb topic")
            else:
                rospy.Subscriber(rgb_topic, Image, self.rgb_callback)
        

    # def rgb_callback(self, msg):
        


# Load a pretrained YOLO11n model
model = YOLO("yolo11n.pt")

# Run inference on the source
results = model(source=0, stream=True) 
for r in results:
    r.masks 
            
            

        