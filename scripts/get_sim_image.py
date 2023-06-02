#!/usr/bin/python3
import time
from timeit import default_timer as timer
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
rospy.init_node('opencv', anonymous=True)

bridge = CvBridge()
start = timer()
time_interval = 1.00
image_iteration = 0
while not rospy.is_shutdown():
    #print("start")
    
    data = rospy.wait_for_message("/webcam/image_raw", Image)
    #print(data)
    try:
        img = bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError:
        rospy.logerr("CvBawridge Error: {0}".format(e))
    frame = np.array(img)

    cv2.imshow("",frame)
    end = timer()
    time_total = np.round(end-start,2)
    
    if time_total >= time_interval:
        print(str(time_interval)+" seconds passed, iteration: "+str(image_iteration))
        cv2.imwrite("images/images"+str(image_iteration)+".jpg", frame)
        start = timer()
        image_iteration+=1
    if cv2.waitKey(2) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()

# from timeit import default_timer as timer
# from datetime import timedelta
# import time
# import numpy as np
# start = timer()
# while True:
#     end = timer()
#     time_total = np.round(end-start,2)
#     print(time_total)
#     if time_total >= 5.00:break

# print("finished")