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

PACK_DIR = rospkg.RosPack().get_path("theta_object_detect")
IMAGE_DIR = os.path.join(PACK_DIR, "resource/object.png")
MODEL_DIR = os.path.join(PACK_DIR, "last.pt")

model = YOLO(MODEL_DIR, task='detect') 

bridge = CvBridge()

def image_detect(req):

    webcam = cv2.VideoCapture(0) #theta usar VideoCapture(2)
    if webcam.isOpened():
        validacao, frame = webcam.read()
        # while validacao:
        #     validacao, frame = webcam.read()
        cv2.imwrite(IMAGE_DIR, frame)
        webcam.release()

    # time.sleep(3)
    #Identificação 
    rospy.loginfo("locating objects")
    results = model.predict(source= IMAGE_DIR, save= True, save_dir= IMAGE_DIR)
    rospy.loginfo("Object located")
    result = results[0]
    num_boxes = len(result.boxes)
    box = result.boxes[0]
    list_object = ''

    rospy.loginfo("Number of objects: %d", num_boxes)
    for box in result.boxes:
        name_id = result.names[box.cls[0].item()]
        cords = box.xyxy[0].tolist()
        cords = [round(x) for x in cords]
        rospy.loginfo("Object type: %s", name_id)

        if(name_id =='Nescau' or name_id =='Kuat' or name_id =='Coconut Water' or name_id =='Fanta'):
            class_id = "Drinks"
            rospy.loginfo("Class: Drinks")
        if(name_id =='Detergent' or name_id =='Sponge' or name_id =='Cloth'):
            class_id = "Cleaning supplies"
            rospy.loginfo("Class: Cleaning supplies")
        if(name_id =='Gelatin' or name_id =='Mustard' or name_id =='Shoyo' or name_id =='Sauce' or name_id =='Tea'):
            class_id = "Pantry items"
            rospy.loginfo("Class: Pantry items")
        if(name_id =='Apple' or name_id =='Pear' or name_id =='Tangerine'):
            class_id = "Fruits"
            rospy.loginfo("Class: Fruits")
        if(name_id =='Treloso' or name_id =='Chocolate' or name_id =='Peanut'):
            class_id = "Snacks"
            rospy.loginfo("Class: Snacks")
        rospy.loginfo("---")
 
        list_object = str(list_object) + f" The object is {name_id} for class {class_id};"
        
    return ObjectDetectResponse(n_objects=num_boxes, object_list=list_object)

if __name__ == '__main__':
    try:
        rospy.init_node('object_detect_node', anonymous=True)
        object_recognition_node = rospy.Subscriber('/object_detect', Empty, image_detect)
        rospy.Service("services/objectDetection", ObjectDetect , image_detect)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
