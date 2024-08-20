from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, Range
import numpy as np
import cv2
import rospy
from datetime import datetime


filename = datetime.now().strftime("output_%Y%m%d_%H%M%S.avi")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(filename,fourcc, 20.0, (1280, 720))

def camcallback(color_img_ros):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(color_img_ros,"bgr8")
    except CvBridgeError:
        rospy.logerr("CvBawridge Error: {0}".format(e))                                
    frame = np.array(img)
    cv2.imshow("Camera Image", frame)
    cv2.waitKey(1)
    out.write(frame)
    return frame




def main():
    rospy.init_node('video_saver', anonymous=True)
    img_sub = rospy.Subscriber("/camera/image_raw", Image, camcallback) 
    try: rospy.spin()
    except KeyboardInterrupt:
        out.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


