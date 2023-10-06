#!/usr/bin/env python3
import rospkg
import rospy
import cv2
import os.path
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

PACK_DIR = rospkg.RosPack().get_path("theta_object_detect")
IMAGE_DIR = os.path.join(PACK_DIR, "resource/object.png")

model = YOLO('yolov8n.pt') 

bridge = CvBridge()

def image_detect(data):
    rospy.Subscriber("bridge/original_image", Image, image_detect, queue_size=1)
    bridge_op = CvBridge()
    image = bridge_op.imgmsg_to_cv2(data, "bgr8")

    results = model.predict(source = image, show =True)
    cv2.imwrite(IMAGE_DIR, results)

if __name__ == '__main__':
    try:
        rospy.init_node('object_detect_node', anonymous=True)
        object_detect = rospy.Subscriber('/objetc_detect', Empty, image_detect)
        
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass