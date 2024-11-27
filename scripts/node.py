#!/usr/bin/env python3
from yolo_detect import YoloDetect
import rospy
def main():
    rospy.init_node("yolo_detect")
    weight = rospy.get_param("~weight", "yolo11m-seg.pt")
    print(weight)
    my_yolo = YoloDetect(model=weight)
    rospy.loginfo("detet service ready")
    rospy.spin()
if __name__ == "__main__":
    main()


