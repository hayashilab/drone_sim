from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import rospy

rospy.init_node('opencv', anonymous=True)
bridge = CvBridge()
pub = rospy.Publisher("webcam",Image,queue_size=10)
vid = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    ret, frame = vid.read()
    cv2.imshow('frame', frame)
    ros_image = bridge.cv2_to_imgmsg(frame,"bgr8")
    #print(ros_image)
    pub.publish(ros_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()