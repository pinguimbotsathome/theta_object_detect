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

model = YOLO('yolov8s.pt') 

bridge = CvBridge()

def image_detect(req):

    #Foto com o kinect
    #rospy.Subscriber("bridge/original_image", Image, image_detect, queue_size=1)
    #bridge_op = CvBridge()
    #image = bridge_op.imgmsg_to_cv2(data, "bgr8")
     #if os.path.exists(IMAGE_DIR):
    #            breakpoint
    #        else:
    #            cv2.imwrite(IMAGE_DIR, image)

    #foto com a webcam
    webcam = cv2.VideoCapture(0)
    if webcam.isOpened():
        validacao, frame = webcam.read()
        while validacao:
            validacao, frame = webcam.read()
            if os.path.exists(IMAGE_DIR):
                breakpoint
            else:
                cv2.imwrite(IMAGE_DIR, frame)

    #Identificação 
    rospy.loginfo("locating objects")
    results = model.predict(IMAGE_DIR)
    rospy.loginfo("Object located")
    result = results[0]
    num_boxes = len(result.boxes)
    box = result.boxes[0]

    rospy.loginfo("Number of objects: %d", num_boxes)
    for box in result.boxes:
        class_id = result.names[box.cls[0].item()]
        cords = box.xyxy[0].tolist()
        cords = [round(x) for x in cords]
        conf = round(box.conf[0].item(), 2)
        rospy.loginfo("Object type: %s", class_id)
        rospy.loginfo("Probability: %.2f", conf)
        rospy.loginfo("---")



if __name__ == '__main__':
    try:
        rospy.init_node('object_detect_node', anonymous=True)
        object_detect = rospy.Subscriber('/object_detect', Empty, image_detect)
        
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass