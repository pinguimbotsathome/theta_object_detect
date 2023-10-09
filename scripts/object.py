#!/usr/bin/env python3
import rospkg
import rospy
import cv2
import os.path
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from theta_object_detect.srv import ObjectDetect, ObjectDetectResponse
import time

PACK_DIR = rospkg.RosPack().get_path("theta_object_detect")
IMAGE_DIR = os.path.join(PACK_DIR, "resource/object.png")
MODEL_DIR = os.path.join(PACK_DIR, "last.pt")
model = YOLO(MODEL_DIR, task='detect') 

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

    # #foto com a webcam
    # webcam = cv2.VideoCapture(0) #theta usar VideoCapture(1)
    # if webcam.isOpened():
    #     validacao, frame = webcam.read()
    #     # while validacao:
    #     #     validacao, frame = webcam.read()
    #     cv2.imwrite(IMAGE_DIR, frame)
    #         # validacao = 0


    # time.sleep(3)
    #Identificação 
    rospy.loginfo("locating objects")
    results = model.predict(source=IMAGE_DIR, save = True)
    rospy.loginfo("Object located")
    result = results[0]
    num_boxes = len(result.boxes)
    box = result.boxes[0]

    list_objects=None

    rospy.loginfo("Number of objects: %d", num_boxes)
    for box in result.boxes:
        class_id = result.names[box.cls[0].item()]
        cords = box.xyxy[0].tolist()
        cords = [round(x) for x in cords]
        rospy.loginfo("Object type: %s", class_id)

        if(class_id=='Nescau' or class_id=='Kuat' or class_id=='Coconut Water' or class_id=='Fanta'):
            list_objects = "Drinks"
            rospy.loginfo("Class: Drinks")
        if(class_id=='Detergent' or class_id=='Sponge' or class_id=='Cloth'):
            list_objects = "Cleaning supplies"
            rospy.loginfo("Class: Cleaning supplies")
        if(class_id=='Gelatin' or class_id=='Mustard' or class_id=='Shoyo' or class_id=='Sauce' or class_id=='Tea'):
            list_objects = "Pantry items"
            rospy.loginfo("Class: Pantry items")
        if(class_id=='Apple' or class_id=='Pear' or class_id=='Tangerine'):
            list_objects = "Fruits"
            rospy.loginfo("Class: Fruits")
        if(class_id=='Treloso' or class_id=='Chocolate' or class_id=='Peanut'):
            list_objects = "Snacks"
            rospy.loginfo("Class: Snacks")
        rospy.loginfo("---")
 
        list_objects = f"{class_id}" # : {list_objects}"

    return ObjectDetectResponse(n_objects=num_boxes, object_list=list_objects)

if __name__ == '__main__':
    try:
        rospy.init_node('object_detect_node', anonymous=True)
        object_recognition_node = rospy.Subscriber('/object_detect', Empty, image_detect)
        rospy.Service("services/objectDetection", ObjectDetect , image_detect)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
